function _update_parameter_values!(
    ::AbstractArray{T},
    pt::ParameterType,
    ::NoAttributes,
    args...,
) where {T <: Union{Float64, JuMP.VariableRef}}
    # NoAttributes signals a parameter whose values are set at build and never refreshed.
    # If you reach this method with a parameter type that *should* be updated, dispatch
    # is wrong: add a specialized method for the parameter's concrete attribute type.
    @debug "No-op parameter update for $(typeof(pt)) with NoAttributes"
    return
end

######################## Methods to update Parameters from Time Series #####################
function _get_time_series_window(
    ::DenseAxisArray{T, 2},
    ::Type{U},
    model::DecisionModel,
    component,
    ts_name,
    initial_time,
) where {T <: Union{JuMP.VariableRef, Float64}, U <: PSY.AbstractDeterministic}
    return get_time_series_values!(
        U,
        model,
        component,
        ts_name,
        initial_time,
        get_time_steps(get_optimization_container(model))[end];
        interval = get_interval(get_settings(model)),
    )
end

# A container with an extra axis holds a vector per time step, as a series with value axes
# gives. A TimeArray holds two dims at most, so read the window the way the build reads it.
function _get_time_series_window(
    ::DenseAxisArray{T, 3},
    ::Type{U},
    model::DecisionModel,
    component,
    ts_name,
    initial_time,
) where {T <: Union{JuMP.VariableRef, Float64}, U <: PSY.AbstractDeterministic}
    return get_time_series_window(
        U,
        component,
        ts_name,
        initial_time,
        get_time_steps(get_optimization_container(model))[end];
        interval = get_interval(get_settings(model)),
        resolution = get_resolution(model),
    )
end

# An emulator step reads one value per component.
function _get_time_series_window(
    ::DenseAxisArray{T, 2},
    ::Type{U},
    model::EmulationModel,
    component,
    ts_name,
    initial_time,
) where {T <: Union{JuMP.VariableRef, Float64}, U <: PSY.SingleTimeSeries}
    return get_time_series_values!(U, model, component, ts_name, initial_time)
end

function _get_time_series_window(
    ::DenseAxisArray{T, 3},
    ::Type{U},
    model::EmulationModel,
    component,
    ts_name,
    initial_time,
) where {T <: Union{JuMP.VariableRef, Float64}, U <: PSY.SingleTimeSeries}
    return get_time_series_window(
        U,
        component,
        ts_name,
        initial_time,
        1;
        resolution = get_resolution(model),
    )
end

function _set_time_series_row!(
    parent_param,
    ::W,
    ts_vector,
    additional_axes,
    i_param::Int,
    ts_name,
) where {W <: ParameterType}
    for (t, value) in enumerate(ts_vector)
        # first two axes of parameter_array are component, time; we care about any additional ones
        unwrapped_value = unwrap_for_param(W(), value, additional_axes)
        if !all(isfinite, unwrapped_value)
            error("The value for the time series $(ts_name) is not finite. \
                  Check that the data in the time series is valid.")
        end
        _set_param_value_at!(parent_param, unwrapped_value, i_param, t)
    end
    return
end

function _set_param_value!(
    param::JuMPVariableTensor,
    value::Union{T, AbstractVector{T}},
    name::String,
    t::Int,
) where {T <: ValidDataParamEltypes}
    fix_maybe_broadcast!(param, value, (name, t))
    return
end

function _set_param_value!(
    param::DenseAxisArray{T, 2},
    value::T,
    name::String,
    t::Int,
) where {T <: ValidDataParamEltypes}
    param[name, t] = value
    return
end

function _set_param_value!(
    param::DenseAxisArray{T},
    value::Union{T, AbstractVector{T}},
    name::String,
    t::Int,
) where {T <: ValidDataParamEltypes}
    assign_maybe_broadcast!(param, value, (name, t))
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    param::W,
    attributes::TimeSeriesAttributes{U},
    ::Type{V},
    model::Union{DecisionModel, EmulationModel},
    ::DatasetContainer{InMemoryDataset},
) where {
    T <: Union{JuMP.VariableRef, Float64},
    U <: Union{PSY.AbstractDeterministic, PSY.SingleTimeSeries},
    V <: PSY.Component,
    W <: ParameterType,
}
    source = _reader_source(W, V, parameter_array, attributes, model)
    if source === nothing
        _refresh_time_series_rows!(parameter_array, param, attributes, V, model)
    else
        # Updated on its own, a parameter reads the reader for itself. Between solves,
        # `_refresh_reader_parameters!` refreshes it with the others that share the read.
        reader = get_time_series_reader!(model, source...)
        _read_reader!(reader, get_current_time(model))
        horizon = length(get_time_steps(get_optimization_container(model)))
        _write_reader_rows!(reader, parameter_array, param, attributes, V, horizon)
    end
    return
end

const _ReaderSeries = Union{PSY.AbstractDeterministic, PSY.SingleTimeSeries}

# Which reader refreshes a time series parameter between solves, as (series type, name),
# or `nothing` for one refreshed a series at a time. The per-key update and the batch in
# `update_parameters!` both ask this, so they cannot disagree about a parameter.
# A decision model's parameters hold forecasts and an emulator's hold static series, so the
# series type alone picks the reader.
_reader_source(
    ::Type{<:ParameterType},
    ::Type{<:PSY.Component},
    ::Any,
    ::Any,
    ::IOM.AbstractOptimizationModel,
) = nothing

_reader_source(
    ::Type{<:ParameterType},
    ::Type{<:PSY.Component},
    ::DenseAxisArray{<:Union{JuMP.VariableRef, Float64}},
    attributes::TimeSeriesAttributes{U},
    ::IOM.AbstractOptimizationModel,
) where {U <: _ReaderSeries} = (U, get_time_series_name(attributes))

# Reduced branches refresh through the network reduction catalog instead.
_reader_source(
    ::Type{<:TimeSeriesParameter},
    ::Type{<:PSY.ACTransmission},
    ::DenseAxisArray{<:Union{JuMP.VariableRef, Float64}},
    ::TimeSeriesAttributes{<:PSY.AbstractDeterministic},
    ::DecisionModel,
) = nothing

# A service parameter keyed to one service (`key.meta`) refreshes through its own method.
# Time series service parameters are keyed per service type and refresh like devices.
_reader_source(
    ::Type{<:ParameterType},
    ::Type{<:PSY.Service},
    ::DenseAxisArray{<:Union{JuMP.VariableRef, Float64}},
    ::TimeSeriesAttributes{<:_ReaderSeries},
    ::IOM.AbstractOptimizationModel,
) = nothing

_reader_source(
    ::Type{<:TimeSeriesParameter},
    ::Type{<:PSY.Service},
    ::DenseAxisArray{<:Union{JuMP.VariableRef, Float64}},
    attributes::TimeSeriesAttributes{U},
    ::IOM.AbstractOptimizationModel,
) where {U <: _ReaderSeries} = (U, get_time_series_name(attributes))

"""
Refresh every parameter a time series reader serves, reading each reader once for all the
parameters that share its series name. Returns the keys it refreshed.
"""
function _refresh_reader_parameters!(model::IOM.AbstractOptimizationModel)
    container = get_optimization_container(model)
    by_source = Dict{Tuple{Type, String}, Vector{ParameterKey}}()
    refreshed = Set{ParameterKey}()
    for (key, parameter) in get_parameters(container)
        source = _reader_source(
            get_entry_type(key),
            get_component_type(key),
            get_parameter_array(parameter),
            get_attributes(parameter),
            model,
        )
        # Parameters that refresh some other way.
        source === nothing && continue
        push!(get!(() -> ParameterKey[], by_source, source), key)
        push!(refreshed, key)
    end
    horizon = length(get_time_steps(container))
    for ((U, ts_name), source_keys) in by_source
        reader = get_time_series_reader!(model, U, ts_name)
        _read_reader!(reader, get_current_time(model))
        for key in source_keys
            parameter = get_parameter(container, key)
            _write_reader_rows!(
                reader,
                get_parameter_array(parameter),
                get_entry_type(key)(),
                get_attributes(parameter),
                get_component_type(key),
                horizon,
            )
        end
    end
    return refreshed
end

_read_reader!(reader::IS.ForecastReader, time) = IS.read_forecast_window!(reader, time)
_read_reader!(reader::IS.StaticTimeSeriesReader, time) =
    IS.read_static_time_series_values!(reader, time)

_num_reader_groups(reader::IS.ForecastReader) = IS.get_num_forecast_groups(reader)
_num_reader_groups(reader::IS.StaticTimeSeriesReader) =
    IS.get_num_static_time_series_groups(reader)

# A group's values, one per entry, and its entries.
_reader_group(reader::IS.ForecastReader, group) =
    (
        IS.get_forecast_group_windows(reader, group),
        IS.get_forecast_group_entries(reader, group),
    )
_reader_group(reader::IS.StaticTimeSeriesReader, group) = (
    IS.get_static_time_series_group_values(reader, group),
    IS.get_static_time_series_group_entries(reader, group),
)

# Writes the reader's current values into one parameter's rows. Components that share a
# stored array share a row, so a row is written once.
function _write_reader_rows!(
    reader::Union{IS.ForecastReader, IS.StaticTimeSeriesReader},
    parameter_array::DenseAxisArray{<:Union{JuMP.VariableRef, Float64}},
    param::ParameterType,
    attributes::TimeSeriesAttributes,
    ::Type{V},
    horizon::Int,
) where {V <: PSY.Component}
    written = falses(length(axes(parameter_array, 1)))
    for group in 1:_num_reader_groups(reader)
        _write_reader_group!(
            parameter_array,
            param,
            attributes,
            V,
            written,
            _reader_group(reader, group)...,
            horizon,
        )
    end
    if !all(written)
        error(
            "The time series reader for $(get_time_series_name(attributes)) has no " *
            "entry for $(count(!, written)) of the $(length(written)) rows of " *
            "$(typeof(param)) for $V.",
        )
    end
    return
end

# Function barrier: `values` is concrete per group, so each entry reads without dispatch.
function _write_reader_group!(
    parameter_array::DenseAxisArray{<:Union{JuMP.VariableRef, Float64}},
    param::ParameterType,
    attributes::TimeSeriesAttributes,
    ::Type{V},
    written::BitVector,
    values::AbstractArray,
    entries::AbstractVector{
        <:Union{IS.ForecastReaderEntry, IS.StaticTimeSeriesReaderEntry},
    },
    horizon::Int,
) where {V <: PSY.Component}
    ts_name = get_time_series_name(attributes)
    parent_param = parameter_array.data
    name_lookup = parameter_array.lookup[1]
    component_names = get_component_names(attributes)
    additional_axes = lookup_additional_axes(parameter_array)
    for (i_entry, entry) in enumerate(entries)
        # The reader holds every series of this name; keep the ones this parameter models.
        owner = entry.owner
        owner isa V || continue
        name = PSY.get_name(owner)
        name in component_names || continue
        i_param = name_lookup[_get_ts_uuid(attributes, name)]
        written[i_param] && continue
        _set_time_series_row!(
            parent_param,
            param,
            _entry_row(values, i_entry, horizon),
            additional_axes,
            i_param,
            ts_name,
        )
        written[i_param] = true
    end
    return
end

# A series with value axes holds a small array at each time step (say, one value per block
# and product), and the parameter has an extra dimension for those values. Each step's array
# goes into the row as one vector, flattened the same way the build flattens it (IOM
# `get_time_series_window`).

# Entry `i_entry`'s row, one value per step of the model's horizon, out of its group's
# values. A forecast group is a vector of windows, one per entry. A static group's first
# axis is the entry; the emulator has a single step, so the row holds one value.
_entry_row(windows::AbstractVector{<:AbstractArray}, i_entry, horizon) =
    _window_steps(windows[i_entry], horizon)
_entry_row(values::AbstractVector{<:Real}, i_entry, _) = view(values, i_entry:i_entry)
_entry_row(values::AbstractArray{<:Real}, i_entry, _) =
    (vec(selectdim(values, 1, i_entry)),)

# One entry's forecast window as a row, one value per step of the horizon. The window's
# first axis is time; any further axes are the series' value axes. Steps are views, so a
# step is strided across the window (time-first) but nothing is copied.
_window_steps(window::AbstractVector, horizon) = view(window, 1:horizon)
_window_steps(window::AbstractArray, horizon) =
    (vec(selectdim(window, 1, t)) for t in 1:horizon)

function _refresh_time_series_rows!(
    parameter_array::DenseAxisArray,
    param::ParameterType,
    attributes::TimeSeriesAttributes{U},
    ::Type{V},
    model::Union{DecisionModel, EmulationModel},
) where {U, V <: PSY.Component}
    initial_forecast_time = get_current_time(model)
    ts_name = get_time_series_name(attributes)
    sys = get_system(model)
    # Hoist the underlying dense storage and per-component name lookup once so each
    # write skips DenseAxisArray's String-keyed axis lookup. `additional_axes` is
    # invariant for the lifetime of this update call.
    parent_param = parameter_array.data
    name_lookup = parameter_array.lookup[1]
    additional_axes = lookup_additional_axes(parameter_array)
    ts_uuids = Set{String}()
    # Refresh the rows the build made. Availability can change after the build, and a
    # component type without an availability flag cannot list its components by it.
    for name in get_component_names(attributes)
        ts_uuid = _get_ts_uuid(attributes, name)
        if !(ts_uuid in ts_uuids)
            ts_vector = _get_time_series_window(
                parameter_array,
                U,
                model,
                PSY.get_component(V, sys, name),
                ts_name,
                initial_forecast_time,
            )
            _set_time_series_row!(
                parent_param,
                param,
                ts_vector,
                additional_axes,
                name_lookup[ts_uuid],
                ts_name,
            )
            push!(ts_uuids, ts_uuid)
        end
    end
    return
end

# Time-series parameter update for reduced ACTransmission branches
function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    ::W,
    attributes::TimeSeriesAttributes{U},
    ::Type{V},
    model::DecisionModel,
    ::DatasetContainer{InMemoryDataset},
) where {
    T <: Union{JuMP.VariableRef, Float64},
    U <: PSY.AbstractDeterministic,
    V <: PSY.ACTransmission,
    W <: TimeSeriesParameter,
}
    initial_forecast_time = get_current_time(model)
    horizon = get_time_steps(get_optimization_container(model))[end]
    ts_name = get_time_series_name(attributes)
    model_interval = get_interval(get_settings(model))
    ts_interval = model_interval

    catalog = POM.get_branch_catalog(get_network_model(get_template(model)))

    # Hoist the underlying dense storage and per-component name lookup once so each
    # write skips DenseAxisArray's String-keyed axis lookup.
    parent_param = parameter_array.data
    name_lookup = parameter_array.lookup[1]

    ts_uuids_updated = Set{String}()
    for (name, arc) in PNM.get_name_to_arc_map(catalog, V)
        reduction_entry = PNM.get_reduction_entry(catalog, arc)
        PNM.has_time_series(reduction_entry, U, ts_name) || continue
        device_with_time_series =
            PNM.get_device_with_time_series(reduction_entry, U, ts_name)
        ts_uuid = _get_ts_uuid(attributes, name)
        if ts_uuid in ts_uuids_updated
            continue
        end
        ts_vector = get_time_series_values!(
            U,
            model,
            device_with_time_series,
            ts_name,
            initial_forecast_time,
            horizon;
            interval = ts_interval,
        )
        i_param = name_lookup[ts_uuid]
        for (t, value) in enumerate(ts_vector)
            if !isfinite(value)
                error(
                    "The value for the time series $(ts_name) is not finite. \
                    Check that the data in the time series is valid.",
                )
            end
            _set_param_value_at!(parent_param, value, i_param, t)
        end
        push!(ts_uuids_updated, ts_uuid)
    end
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    param::W,
    attributes::TimeSeriesAttributes{U},
    service::V,
    model::Union{DecisionModel, EmulationModel},
    ::DatasetContainer{InMemoryDataset},
) where {
    T <: Union{JuMP.VariableRef, Float64},
    U <: Union{PSY.AbstractDeterministic, PSY.SingleTimeSeries},
    V <: PSY.Service,
    W <: ParameterType,
}
    ts_name = get_time_series_name(attributes)
    ts_vector = _get_time_series_window(
        parameter_array,
        U,
        model,
        service,
        ts_name,
        get_current_time(model),
    )
    _set_time_series_row!(
        parameter_array.data,
        param,
        ts_vector,
        lookup_additional_axes(parameter_array),
        parameter_array.lookup[1][_get_ts_uuid(attributes, PSY.get_name(service))],
        ts_name,
    )
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    ::ParameterType,
    attributes::VariableValueAttributes,
    ::Type{<:PSY.Device},
    model::DecisionModel,
    state::DatasetContainer{InMemoryDataset},
) where {T <: Union{JuMP.VariableRef, Float64}}
    current_time = get_current_time(model)
    state_values = get_dataset_values(state, get_attribute_key(attributes))
    component_names, time = axes(parameter_array)
    model_resolution = get_resolution(model)
    state_data = get_dataset(state, get_attribute_key(attributes))
    state_timestamps = state_data.timestamps
    max_state_index = get_num_rows(state_data)
    if model_resolution < state_data.resolution
        t_step = 1
    else
        t_step = model_resolution ÷ state_data.resolution
    end
    state_data_index = find_aligned_timestamp_index(state_data, current_time)
    sim_timestamps = range(current_time; step = model_resolution, length = time[end])
    # Hoist underlying dense storage and per-axis name lookups so the inner loop
    # can index by integer pair, skipping DenseAxisArray's String-keyed lookup.
    parent_param = parameter_array.data
    parent_state = state_values.data
    param_lookup = parameter_array.lookup[1]
    state_lookup = state_values.lookup[1]
    for t in time
        timestamp_ix = min(max_state_index, state_data_index + t_step)
        @debug "parameter horizon is over the step" max_state_index > state_data_index + 1
        if state_timestamps[timestamp_ix] <= sim_timestamps[t]
            state_data_index = timestamp_ix
        end
        for name in component_names
            i_state = state_lookup[name]
            i_param = param_lookup[name]
            state_value = parent_state[i_state, state_data_index]
            _check_state_value(state_value, get_attribute_key(attributes), name)
            _set_param_value_at!(parent_param, state_value, i_param, t)
        end
    end
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    ::ParameterType,
    attributes::VariableValueAttributes,
    ::PSY.Reserve,
    model::DecisionModel,
    state::DatasetContainer{InMemoryDataset},
) where {T <: Union{JuMP.VariableRef, Float64}}
    current_time = get_current_time(model)
    state_values = get_dataset_values(state, get_attribute_key(attributes))
    component_names, time = axes(parameter_array)
    model_resolution = get_resolution(model)
    state_data = get_dataset(state, get_attribute_key(attributes))
    state_timestamps = state_data.timestamps
    max_state_index = get_num_rows(state_data)
    if model_resolution < state_data.resolution
        t_step = 1
    else
        t_step = model_resolution ÷ state_data.resolution
    end
    state_data_index = find_aligned_timestamp_index(state_data, current_time)
    sim_timestamps = range(current_time; step = model_resolution, length = time[end])
    # Hoist underlying dense storage and per-axis name lookups so the inner loop
    # can index by integer pair, skipping DenseAxisArray's String-keyed lookup.
    parent_param = parameter_array.data
    parent_state = state_values.data
    param_lookup = parameter_array.lookup[1]
    state_lookup = state_values.lookup[1]
    for t in time
        timestamp_ix = min(max_state_index, state_data_index + t_step)
        @debug "parameter horizon is over the step" max_state_index > state_data_index + 1
        if state_timestamps[timestamp_ix] <= sim_timestamps[t]
            state_data_index = timestamp_ix
        end
        for name in component_names
            i_state = state_lookup[name]
            i_param = param_lookup[name]
            state_value = parent_state[i_state, state_data_index]
            _check_state_value(state_value, get_attribute_key(attributes), name)
            _set_param_value_at!(parent_param, state_value, i_param, t)
        end
    end
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    ::ParameterType,
    attributes::VariableValueAttributes{VariableKey{OnVariable, U}},
    ::Type{U},
    model::DecisionModel,
    state::DatasetContainer{InMemoryDataset},
) where {T <: Union{JuMP.VariableRef, Float64}, U <: PSY.Device}
    current_time = get_current_time(model)
    state_values = get_dataset_values(state, get_attribute_key(attributes))
    component_names, time = axes(parameter_array)
    model_resolution = get_resolution(model)
    state_data = get_dataset(state, get_attribute_key(attributes))
    state_timestamps = state_data.timestamps
    max_state_index = get_num_rows(state_data)
    if model_resolution < state_data.resolution
        t_step = 1
    else
        t_step = model_resolution ÷ state_data.resolution
    end
    state_data_index = find_aligned_timestamp_index(state_data, current_time)

    sim_timestamps = range(current_time; step = model_resolution, length = time[end])
    # Hoist underlying dense storage and per-axis name lookups so the inner loop
    # can index by integer pair, skipping DenseAxisArray's String-keyed lookup.
    parent_param = parameter_array.data
    parent_state = state_values.data
    param_lookup = parameter_array.lookup[1]
    state_lookup = state_values.lookup[1]
    for t in time
        timestamp_ix = min(max_state_index, state_data_index + t_step)
        @debug "parameter horizon is over the step" max_state_index > state_data_index + 1
        if state_timestamps[timestamp_ix] <= sim_timestamps[t]
            state_data_index = timestamp_ix
        end
        for name in component_names
            i_state = state_lookup[name]
            i_param = param_lookup[name]
            value = round(parent_state[i_state, state_data_index])
            _check_state_value(value, get_attribute_key(attributes), name)
            if 0.0 > value || value > 1.0
                error(
                    "The value for the system state used in $(encode_key_as_string(get_attribute_key(attributes))): $(value) is out of the [0, 1] range",
                )
            end
            _set_param_value_at!(parent_param, value, i_param, t)
        end
    end
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    ::ParameterType,
    attributes::VariableValueAttributes,
    ::Type{<:PSY.Component},
    model::EmulationModel,
    state::DatasetContainer{InMemoryDataset},
) where {T <: Union{JuMP.VariableRef, Float64}}
    current_time = get_current_time(model)
    state_values = get_dataset_values(state, get_attribute_key(attributes))
    component_names, _ = axes(parameter_array)
    state_data = get_dataset(state, get_attribute_key(attributes))
    state_data_index = find_aligned_timestamp_index(state_data, current_time)
    # Hoist underlying dense storage and per-axis name lookups.
    parent_param = parameter_array.data
    parent_state = state_values.data
    param_lookup = parameter_array.lookup[1]
    state_lookup = state_values.lookup[1]
    for name in component_names
        i_state = state_lookup[name]
        i_param = param_lookup[name]
        state_value = parent_state[i_state, state_data_index]
        _check_state_value(state_value, get_attribute_key(attributes), name)
        _set_param_value_at!(parent_param, state_value, i_param, 1)
    end
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    ::ParameterType,
    attributes::VariableValueAttributes{VariableKey{OnVariable, U}},
    ::Type{<:PSY.Component},
    model::EmulationModel,
    state::DatasetContainer{InMemoryDataset},
) where {T <: Union{JuMP.VariableRef, Float64}, U <: PSY.Component}
    current_time = get_current_time(model)
    state_values = get_dataset_values(state, get_attribute_key(attributes))
    component_names, _ = axes(parameter_array)
    state_data = get_dataset(state, get_attribute_key(attributes))
    state_data_index = find_aligned_timestamp_index(state_data, current_time)
    has_outage = haskey(
        get_parameters_values(state),
        ParameterKey{
            AvailableStatusParameter,
            U,
        }(
            "",
        ),
    )
    if has_outage
        status_values = get_dataset_values(
            state,
            ParameterKey{
                AvailableStatusParameter,
                U,
            }(
                "",
            ),
        )
        status_data = get_dataset(
            state,
            ParameterKey{
                AvailableStatusParameter,
                U,
            }(
                "",
            ),
        )
        status_data_index = find_aligned_timestamp_index(status_data, current_time)
        parent_status = status_values.data
        # `_AxisLookup{Dict{String,Int64}}` wraps a `Dict`; reach for `.data`
        # so we can `haskey` and integer-index without a String-keyed scan.
        status_lookup_dict = status_values.lookup[1].data
    end
    # Hoist underlying dense storage and per-axis name lookups for the inner loop.
    parent_param = parameter_array.data
    parent_state = state_values.data
    param_lookup = parameter_array.lookup[1]
    state_lookup = state_values.lookup[1]
    for name in component_names
        i_state = state_lookup[name]
        i_param = param_lookup[name]
        if has_outage && haskey(status_lookup_dict, name) &&
           parent_status[status_lookup_dict[name], status_data_index] == 0.0 &&
           round(parent_state[i_state, state_data_index]) == 1.0
            # Override feed forward based on status parameter
            value = 0.0
        else
            value = round(parent_state[i_state, state_data_index])
        end
        _check_state_value(value, get_attribute_key(attributes), name)
        if 0.0 > value || value > 1.0
            error(
                "The value for the system state used in $(encode_key_as_string(get_attribute_key(attributes))): $(value) is out of the [0, 1] range",
            )
        end
        _set_param_value_at!(parent_param, value, i_param, 1)
    end
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    ::EventParametersAttributes{W, U},
    ::Type{V},
    model::DecisionModel,
    state::DatasetContainer{InMemoryDataset},
) where {
    T <: Union{JuMP.VariableRef, Float64},
    W <: PSY.Contingency,
    U <: EventParameter,
    V <: PSY.Component,
}
    current_time = get_current_time(model)
    key = ParameterKey(U, V)
    state_values = get_dataset_values(state, key)
    component_names, time = axes(parameter_array)
    model_resolution = get_resolution(model)
    state_data = get_dataset(state, key)
    state_timestamps = state_data.timestamps
    max_state_index = get_num_rows(state_data)
    if model_resolution < state_data.resolution
        t_step = 1
    else
        t_step = model_resolution ÷ state_data.resolution
    end
    state_data_index = find_aligned_timestamp_index(state_data, current_time)

    sim_timestamps = range(current_time; step = model_resolution, length = time[end])
    # Hoist underlying dense storage and per-axis name lookups for the inner loop.
    parent_param = parameter_array.data
    parent_state = state_values.data
    param_lookup = parameter_array.lookup[1]
    state_lookup = state_values.lookup[1]
    for t in time
        timestamp_ix = min(max_state_index, state_data_index + t_step)
        @debug "parameter horizon is over the step" max_state_index > state_data_index + 1
        if state_timestamps[timestamp_ix] <= sim_timestamps[t]
            state_data_index = timestamp_ix
        end
        for name in component_names
            i_state = state_lookup[name]
            i_param = param_lookup[name]
            value = parent_state[i_state, state_data_index]
            _check_state_value(value, key, name)
            _set_param_value_at!(parent_param, value, i_param, t)
        end
    end
    return
end

function _update_parameter_values!(
    parameter_array::DenseAxisArray{T},
    ::EventParametersAttributes{W, U},
    ::Type{V},
    model::EmulationModel,
    state::DatasetContainer{InMemoryDataset},
) where {
    T <: Union{JuMP.VariableRef, Float64},
    W <: PSY.Contingency,
    U <: EventParameter,
    V <: PSY.Component,
}
    current_time = get_current_time(model)
    state_values = get_dataset_values(state, U(), V)
    component_names, _ = axes(parameter_array)
    state_data = get_dataset(state, U(), V)
    state_data_index = find_aligned_timestamp_index(state_data, current_time)

    # Hoist underlying dense storage and per-axis name lookups.
    parent_param = parameter_array.data
    parent_state = state_values.data
    param_lookup = parameter_array.lookup[1]
    state_lookup = state_values.lookup[1]
    for name in component_names
        i_state = state_lookup[name]
        i_param = param_lookup[name]
        _set_param_value_at!(
            parent_param,
            parent_state[i_state, state_data_index],
            i_param,
            1,
        )
    end
    return
end

"""
Update parameter function an IOM.AbstractOptimizationModel
"""
function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ParameterType, U <: PSY.Component}
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = get_parameter_array(optimization_container, key)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    _update_parameter_values!(parameter_array, T(), parameter_attributes, U, model, input)
    return
end

function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: EventParameter, U <: PSY.Component}
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = get_parameter_array(optimization_container, key)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    _update_parameter_values!(parameter_array, parameter_attributes, U, model, input)
    return
end

function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ObjectiveFunctionParameter, U <: PSY.Component}
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = get_parameter_array(optimization_container, key)
    # Multiplier is only needed for the objective function since `_update_parameter_values!` also updates the objective function
    parameter_multiplier = get_parameter_multiplier_array(optimization_container, key)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    _update_parameter_values!(
        parameter_array,
        T(),
        parameter_multiplier,
        parameter_attributes,
        U,
        model,
        input,
    )
    return
end

function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ObjectiveFunctionParameter, U <: PSY.Service}
    # Note: Do not instantiate a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = get_parameter_array(optimization_container, key)
    # Multiplier is only needed for the objective function since `_update_parameter_values!` also updates the objective function
    parameter_multiplier = get_parameter_multiplier_array(optimization_container, key)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    _update_service_cost_parameter_values!(
        parameter_array,
        T(),
        parameter_multiplier,
        parameter_attributes,
        U,
        model,
        input,
    )
    return
end

function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{FixValueParameter, U},
    input::DatasetContainer{InMemoryDataset},
) where {U <: PSY.Component}
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = get_parameter_array(optimization_container, key)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    _update_parameter_values!(
        parameter_array,
        FixValueParameter(),
        parameter_attributes,
        U,
        model,
        input,
    )
    _fix_parameter_value!(optimization_container, parameter_array, parameter_attributes)
    return
end

function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{FixValueParameter, U},
    input::DatasetContainer{InMemoryDataset},
) where {U <: PSY.Service}
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = get_parameter_array(optimization_container, key)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    service = PSY.get_component(U, get_system(model), key.meta)
    isnothing(service) && error(
        "Service $(U) named '$(key.meta)' is no longer in the system; " *
        "cannot update parameter $(key)",
    )
    _update_parameter_values!(
        parameter_array,
        FixValueParameter(),
        parameter_attributes,
        U,
        model,
        input,
    )
    _fix_parameter_value!(optimization_container, parameter_array, parameter_attributes)
    return
end

function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: TimeSeriesParameter, U <: PSY.Service}
    # Time-series service parameters (e.g. `RequirementTimeSeriesParameter`) are per-type
    # containers keyed `(T, ServiceType)` with an empty `key.meta` -- the name axis inside
    # `parameter_array` holds every service of type `U` (POM
    # `common_models/add_parameters.jl`). There is no single named component to look up by
    # `key.meta`, so this routes through the generic per-type `_update_parameter_values!`
    # used for device time series instead of the per-instance lookup the method below
    # performs for feedforward-sourced (`VariableValueAttributes`) service parameters.
    parameter_array = get_parameter_array(optimization_container, key)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    _update_parameter_values!(parameter_array, T(), parameter_attributes, U, model, input)
    return
end

function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ParameterType, U <: PSY.Service}
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = get_parameter_array(optimization_container, key)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    service = PSY.get_component(U, get_system(model), key.meta)
    isnothing(service) && error(
        "Service $(U) named '$(key.meta)' is no longer in the system; " *
        "cannot update parameter $(key)",
    )
    _update_parameter_values!(
        parameter_array,
        T(),
        parameter_attributes,
        service,
        model,
        input,
    )
    return
end

# This method is included to avoid ambiguities
function IOM.update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: EventParameter, U <: PSY.Service}
    return
end
