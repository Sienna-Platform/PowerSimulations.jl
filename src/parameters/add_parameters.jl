function add_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    devices::U,
    model::DeviceModel{D, W},
) where {
    T <: ParameterType,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    if get_rebuild_model(get_settings(container)) && has_container_key(container, T, D)
        return
    end
    _add_parameters!(container, T(), devices, model)
    return
end

function add_branch_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    devices::U,
    model::DeviceModel{D, W},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {
    T <: ParameterType,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.ACTransmission}
    if get_rebuild_model(get_settings(container)) && has_container_key(container, T, D)
        return
    end
    _add_time_series_parameters!(container, T(), network_model, devices, model)
    return
end

function add_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    devices::U,
    device_model::DeviceModel{D, W},
    event_model::EventModel{V, X},
) where {
    T <: ParameterType,
    U <: Vector{D},
    V <: PSY.Contingency,
    W <: AbstractDeviceFormulation,
    X <: AbstractEventCondition,
} where {D <: PSY.Component}
    if get_rebuild_model(get_settings(container)) && has_container_key(container, T, D)
        return
    end
    _add_parameters!(container, T(), devices, device_model, event_model)
    return
end

function add_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    ff::LowerBoundFeedforward,
    model::ServiceModel{S, W},
    devices::V,
) where {
    S <: PSY.AbstractReserve,
    T <: VariableValueParameter,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractReservesFormulation,
} where {D <: PSY.Component}
    if get_rebuild_model(get_settings(container)) && has_container_key(container, T, S)
        return
    end
    source_key = get_optimization_container_key(ff)
    _add_parameters!(container, T(), source_key, model, devices)
    return
end

function add_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    service::U,
    model::ServiceModel{U, V},
) where {T <: TimeSeriesParameter, U <: PSY.Service, V <: AbstractServiceFormulation}
    if get_rebuild_model(get_settings(container)) &&
       has_container_key(container, T, U, PSY.get_name(service))
        return
    end
    _add_parameters!(container, T(), service, model)
    return
end

function add_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    ff::AbstractAffectFeedforward,
    model::DeviceModel{D, W},
    devices::V,
) where {
    T <: VariableValueParameter,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    if get_rebuild_model(get_settings(container)) && has_container_key(container, T, D)
        return
    end
    source_key = get_optimization_container_key(ff)
    _add_parameters!(container, T(), source_key, model, devices)
    return
end

function add_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    ff::FixValueFeedforward,
    model::DeviceModel{D, W},
    devices::V,
) where {
    T <: VariableValueParameter,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    if get_rebuild_model(get_settings(container)) && has_container_key(container, T, D)
        return
    end
    source_key = get_optimization_container_key(ff)
    _add_parameters!(container, T(), source_key, model, devices)
    _set_affected_variables!(container, T(), D, ff)
    return
end

function add_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    ff::FixValueFeedforward,
    model::ServiceModel{K, W},
    devices::V,
) where {
    T <: VariableValueParameter,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractServiceFormulation,
    K <: PSY.Reserve,
} where {D <: PSY.Component}
    if get_rebuild_model(get_settings(container)) && has_container_key(container, T, D)
        return
    end
    source_key = get_optimization_container_key(ff)
    _add_parameters!(container, T(), source_key, model, devices)
    _set_affected_variables!(container, T(), K, ff)
    return
end

function _set_affected_variables!(
    container::OptimizationContainer,
    ::T,
    device_type::Type{U},
    ff::FixValueFeedforward,
) where {
    T <: VariableValueParameter,
    U <: PSY.Component,
}
    source_key = get_optimization_container_key(ff)
    var_type = get_entry_type(source_key)
    parameter_container = get_parameter(container, T(), U, "$var_type")
    param_attributes = get_attributes(parameter_container)
    affected_variables = get_affected_values(ff)
    push!(param_attributes.affected_keys, affected_variables...)
    return
end

function _set_affected_variables!(
    container::OptimizationContainer,
    ::T,
    device_type::Type{U},
    ff::FixValueFeedforward,
) where {
    T <: VariableValueParameter,
    U <: PSY.Service,
}
    meta = ff.optimization_container_key.meta
    parameter_container = get_parameter(container, T(), U, meta)
    param_attributes = get_attributes(parameter_container)
    affected_variables = get_affected_values(ff)
    push!(param_attributes.affected_keys, affected_variables...)
    return
end

function _add_parameters!(
    container::OptimizationContainer,
    param::T,
    devices::U,
    model::DeviceModel{D, W},
) where {
    T <: TimeSeriesParameter,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    _add_time_series_parameters!(container, param, devices, model)
    return
end

function _check_dynamic_branch_rating_ts(
    ts::AbstractArray,
    ::T,
    device::PSY.Device,
    model::DeviceModel{D, W},
) where {D <: PSY.Component, T <: TimeSeriesParameter, W <: AbstractDeviceFormulation}
    if !(T <: AbstractDynamicBranchRatingTimeSeriesParameter)
        return
    end

    rating = PSY.get_rating(device)
    if (T <: PostContingencyDynamicBranchRatingTimeSeriesParameter)
        if !(PSY.get_rating_b(device) === nothing)
            rating = PSY.get_rating_b(device)
        else
            @warn "Device $(typeof(device)) '$(PSY.get_name(device))' has Parameter $T but it has no static 'rating_b' defined."
        end
    end

    multiplier = get_multiplier_value(T(), device, W())
    if !all(x -> x >= rating, multiplier * ts)
        @warn "There are values of Parameter $T associated with $(typeof(device)) '$(PSY.get_name(device))' lower than the device static rating $(rating)."
    end
    return
end

# Extends `size` to tuples, treating them like scalars
_size_wrapper(elem) = size(elem)
_size_wrapper(::Tuple) = ()

function _add_time_series_parameters!(
    container::OptimizationContainer,
    param::T,
    network_model::NetworkModel{<:AbstractPTDFModel},
    devices,
    model::DeviceModel{D, W},
) where {D <: PSY.ACTransmission, T <: TimeSeriesParameter, W <: AbstractDeviceFormulation}
    ts_type = get_default_time_series_type(container)
    if !(ts_type <: Union{PSY.AbstractDeterministic, PSY.StaticTimeSeries})
        error("add_parameters! for TimeSeriesParameter is not compatible with $ts_type")
    end
    time_steps = get_time_steps(container)

    net_reduction_data = network_model.network_reduction
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    # TODO: Temporary workaround to get the name where we assume all the names are the same accross devices.
    ts_name = _get_time_series_name(T(), first(devices), model)
    device_name_axis, ts_uuid_axis =
        get_branch_argument_parameter_axes(net_reduction_data, devices, ts_type, ts_name)
    if isempty(device_name_axis)
        @info "No devices with time series $ts_name found for $D devices. Skipping parameter addition."
        return
    end
    additional_axes = ()
    param_container = add_param_container!(
        container,
        param,
        D,
        ts_type,
        ts_name,
        ts_uuid_axis,
        device_name_axis,
        additional_axes,
        time_steps,
    )
    set_subsystem!(get_attributes(param_container), get_subsystem(model))
    ts_uuid_axis
    param_instance = T()
    for (name, (arc, reduction)) in PNM.get_name_to_arc_map(net_reduction_data, D)
        reduction_entry = all_branch_maps_by_type[reduction][D][arc]
        if !PNM.has_time_series(reduction_entry, ts_type, ts_name)
            continue
        end
        device_with_time_series =
            PNM.get_device_with_time_series(reduction_entry, ts_type, ts_name)
        ts_uuid =
            string(IS.get_time_series_uuid(ts_type, device_with_time_series, ts_name))

        has_entry, tracker_container = search_for_reduced_branch_parameter!(
            reduced_branch_tracker,
            arc,
            T,
        )

        if has_entry
            @assert !isempty(tracker_container) name arc reduction
        else
            raw_ts_vals = get_time_series_initial_values!(
                container,
                ts_type,
                device_with_time_series,
                ts_name,
            )
            ts_vals =
                _unwrap_for_param.(Ref(param_instance), raw_ts_vals, Ref(additional_axes))
            @assert all(_size_wrapper.(ts_vals) .== Ref(length.(additional_axes)))
        end
        multiplier = get_multiplier_value(T(), reduction_entry, W())
        jump_model = get_jump_model(container)
        param_array = get_parameter_array(param_container)
        for t in time_steps
            if !has_entry
                # Store raw float in tracker for non-recurrent builds. For recurrent
                # builds (JuMP parameters), read back the VariableRef that set_parameter!
                # creates so that parallel branch types share the same JuMP parameter.
                set_parameter!(param_container, jump_model, ts_vals[t], ts_uuid, t)
                if built_for_recurrent_solves(container)
                    tracker_container[t] = param_array[ts_uuid, t]
                else
                    tracker_container[t] = ts_vals[t]
                end
            else
                # Reuse the value (Float64) or VariableRef already stored by the first
                # branch type that processed this arc.
                set_parameter!(
                    param_container,
                    jump_model,
                    tracker_container[t],
                    ts_uuid,
                    t,
                )
            end
            set_multiplier!(param_container, multiplier, name, t)
        end
        add_component_name!(
            get_attributes(param_container),
            name,
            string(IS.get_time_series_uuid(ts_type, device_with_time_series, ts_name)),
        )
    end
    return
end

# NOTE direct equivalent of _add_parameters! on ObjectiveFunctionParameter
# PERF: compilation hotspot. Switch to TSC.
function _add_time_series_parameters!(
    container::OptimizationContainer,
    param::T,
    devices,
    model::DeviceModel{D, W},
) where {D <: PSY.Component, T <: TimeSeriesParameter, W <: AbstractDeviceFormulation}
    ts_type = get_default_time_series_type(container)
    if !(ts_type <: Union{PSY.AbstractDeterministic, PSY.StaticTimeSeries})
        error("add_parameters! for TimeSeriesParameter is not compatible with $ts_type")
    end

    time_steps = get_time_steps(container)
    # TODO: Temporary workaround to get the name where we assume all the names are the same accross devices.
    ts_name = _get_time_series_name(T(), first(devices), model)

    device_names = String[]
    devices_with_time_series = D[]
    initial_values = Dict{String, AbstractArray}()

    @debug "adding" T D ts_name ts_type _group = LOG_GROUP_OPTIMIZATION_CONTAINER

    for device::D in devices
        if !PSY.has_time_series(device, ts_type, ts_name)
            @info "Time series $(ts_type):$(ts_name) for $D, $(PSY.get_name(device)) not found skipping parameter addition."
            continue
        end
        push!(device_names, PSY.get_name(device))
        push!(devices_with_time_series, device)
        ts_uuid = string(IS.get_time_series_uuid(ts_type, device, ts_name))
        if !(ts_uuid in keys(initial_values))
            initial_values[ts_uuid] =
                get_time_series_initial_values!(container, ts_type, device, ts_name)
            _check_dynamic_branch_rating_ts(initial_values[ts_uuid], param, device, model)
        end
    end

    #=
    # NOTE this is always the case for "normal" time series, but it is currently not enforced in PSY for MBC time series.
    # TODO decide whether this is an acceptable restriction or whether we need to support multiple time series names
    # JD: Yes, the restriction are that the names for this has to be unique as they are specified from the model attributes
    if isempty(active_devices)
        return
    end
    unique_ts_names = unique(ts_names)
    if length(unique_ts_names) > 1
        throw(
            ArgumentError(
                "All time series names must be equal for parameter $T within a given device type. Got $unique_ts_names for device type $D",
            ),
        )
    end
    ts_name = only(unique_ts_names)
    =#

    if isempty(device_names)
        @info "No devices with time series $ts_name found for $D devices. Skipping parameter addition."
        return
    end

    additional_axes =
        calc_additional_axes(container, param, devices_with_time_series, model)
    param_container = add_param_container!(
        container,
        param,
        D,
        ts_type,
        ts_name,
        collect(keys(initial_values)),
        device_names,
        additional_axes,
        time_steps,
    )
    set_subsystem!(get_attributes(param_container), get_subsystem(model))

    jump_model = get_jump_model(container)
    param_instance = T()
    for (ts_uuid, raw_ts_vals) in initial_values
        ts_vals = _unwrap_for_param.(Ref(param_instance), raw_ts_vals, Ref(additional_axes))
        @assert all(_size_wrapper.(ts_vals) .== Ref(length.(additional_axes)))

        for step in time_steps
            set_parameter!(param_container, jump_model, ts_vals[step], ts_uuid, step)
        end
    end

    for device in devices_with_time_series
        multiplier = get_multiplier_value(T(), device, W())
        device_name = PSY.get_name(device)
        for step in time_steps
            set_multiplier!(param_container, multiplier, device_name, step)
        end
        add_component_name!(
            get_attributes(param_container),
            device_name,
            string(IS.get_time_series_uuid(ts_type, device, ts_name)),
        )
    end
    return
end

# Layer of indirection to deal with the fact that some time series names are stored in the component
_get_time_series_name(::T, ::PSY.Component, model::DeviceModel) where {T <: ParameterType} =
    get_time_series_names(model)[T]

_get_time_series_name(::StartupCostParameter, device::PSY.Component, ::DeviceModel) =
    get_name(PSY.get_start_up(PSY.get_operation_cost(device)))

_get_time_series_name(::ShutdownCostParameter, device::PSY.Component, ::DeviceModel) =
    get_name(PSY.get_shut_down(PSY.get_operation_cost(device)))

_get_time_series_name(
    ::IncrementalCostAtMinParameter,
    device::PSY.Device,
    ::DeviceModel,
) =
    get_name(PSY.get_incremental_initial_input(PSY.get_operation_cost(device)))

_get_time_series_name(
    ::DecrementalCostAtMinParameter,
    device::PSY.Device,
    ::DeviceModel,
) =
    get_name(PSY.get_decremental_initial_input(PSY.get_operation_cost(device)))

_get_time_series_name(
    ::Union{
        IncrementalPiecewiseLinearSlopeParameter,
        IncrementalPiecewiseLinearBreakpointParameter,
    },
    device::PSY.Device,
    ::DeviceModel,
) =
    get_name(get_output_offer_curves(PSY.get_operation_cost(device)))

_get_time_series_name(
    ::Union{
        DecrementalPiecewiseLinearSlopeParameter,
        DecrementalPiecewiseLinearBreakpointParameter,
    },
    device::PSY.Device,
    ::DeviceModel,
) =
    get_name(get_input_offer_curves(PSY.get_operation_cost(device)))

# Layer of indirection to figure out what eltype we expect to find in various time series
# (we could just read the time series and figure it out dynamically if this becomes too brittle)
_get_expected_time_series_eltype(::T) where {T <: ParameterType} = Float64
_get_expected_time_series_eltype(::StartupCostParameter) = NTuple{3, Float64}

# Lookup that defines which variables the ObjectiveFunctionParameter corresponds to
_param_to_vars(::FuelCostParameter, ::AbstractDeviceFormulation) = (ActivePowerVariable,)
_param_to_vars(::StartupCostParameter, ::AbstractThermalFormulation) = (StartVariable,)
_param_to_vars(::StartupCostParameter, ::ThermalMultiStartUnitCommitment) =
    MULTI_START_VARIABLES
_param_to_vars(::ShutdownCostParameter, ::AbstractThermalFormulation) = (StopVariable,)
_param_to_vars(::AbstractCostAtMinParameter, ::AbstractDeviceFormulation) = (OnVariable,)
_param_to_vars(
    ::Union{
        IncrementalPiecewiseLinearSlopeParameter,
        IncrementalPiecewiseLinearBreakpointParameter,
    },
    ::AbstractDeviceFormulation,
) =
    (PiecewiseLinearBlockIncrementalOffer,)
_param_to_vars(
    ::Union{
        DecrementalPiecewiseLinearSlopeParameter,
        DecrementalPiecewiseLinearBreakpointParameter,
    },
    ::AbstractDeviceFormulation,
) =
    (PiecewiseLinearBlockDecrementalOffer,)

# Layer of indirection to handle possible additional axes. Most parameters have just the two
# usual axes (device, timestamp), but some have a third (e.g., piecewise tranche)
calc_additional_axes(
    ::OptimizationContainer,
    ::T,
    ::U,
    ::DeviceModel{D, W},
) where {
    T <: ParameterType,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component} = ()

calc_additional_axes(
    ::OptimizationContainer,
    ::T,
    ::U,
    ::ServiceModel{D, W},
) where {
    T <: ParameterType,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractServiceFormulation,
} where {D <: PSY.Service} = ()

_get_max_tranches(data::Vector{IS.PiecewiseStepData}) = maximum(length.(data))
_get_max_tranches(data::TimeSeries.TimeArray) = _get_max_tranches(values(data))
_get_max_tranches(data::AbstractDict) = maximum(_get_max_tranches.(values(data)))

# Iterate through all periods of a piecewise time series and return the maximum number of tranches
function get_max_tranches(device::PSY.Device, piecewise_ts::IS.TimeSeriesKey)
    data = PSY.get_data(PSY.get_time_series(device, piecewise_ts))
    max_tranches = _get_max_tranches(data)
    return max_tranches
end

# It's nice for debugging purposes to have meaningful labels on the tranche axis. These
# labels are never relied upon in the current implementation
make_tranche_axis(n_tranches) = "tranche_" .* string.(1:n_tranches)

# Find the global maximum number of tranches we'll have to handle and create the parameter with an axis of that length
function calc_additional_axes(
    ::OptimizationContainer,
    ::P,
    devices::U,
    ::DeviceModel{D, W},
) where {
    P <: AbstractPiecewiseLinearSlopeParameter,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    curves = _get_parameter_field.((P(),), PSY.get_operation_cost.(devices))
    max_tranches = maximum(get_max_tranches.(devices, curves))
    return (make_tranche_axis(max_tranches),)
end

function calc_additional_axes(
    ::OptimizationContainer,
    ::P,
    devices::U,
    ::DeviceModel{D, W},
) where {
    P <: AbstractPiecewiseLinearBreakpointParameter,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    curves = _get_parameter_field.((P(),), PSY.get_operation_cost.(devices))
    max_tranches = maximum(get_max_tranches.(devices, curves))
    return (make_tranche_axis(max_tranches + 1),)  # one more breakpoint than tranches
end

"""
Given a parameter array, get any additional axes, i.e., those that aren't the first
(component) or the last (time)
"""
lookup_additional_axes(parameter_array) = axes(parameter_array)[2:(end - 1)]

# Layer of indirection to handle the fact that some parameters come from time series that
# represent multiple things (e.g., both slopes and breakpoints come from the same time
# series of `FunctionData`). This function is called on every element of the time series
# with an expected output axes tuple.
_unwrap_for_param(::ParameterType, ts_elem, expected_axs) = ts_elem

# For piecewise MarketBidCost-like data, the number of tranches can vary over time, so the
# parameter container is sized for the maximum number of tranches and in smaller cases we
# have to pad. We do this by creating additional "degenerate" tranches at the top end of the
# curve with dx = 0 such that their dispatch variables are constrained to 0. In theory, the
# slope shouldn't matter for these degenerate segments. In practice, we'll use slope = 0 so
# the term can be more trivially dropped from the objective function.
function _unwrap_for_param(
    ::AbstractPiecewiseLinearSlopeParameter,
    ts_elem::IS.PiecewiseStepData,
    expected_axs,
)
    max_len = length(only(expected_axs))
    y_coords = IS.get_y_coords(ts_elem)
    @assert length(y_coords) <= max_len
    fill_value = 0.0  # pad with slope = 0 if necessary (see above)
    padded_y_coords = vcat(y_coords, fill(fill_value, max_len - length(y_coords)))
    return padded_y_coords
end

function _unwrap_for_param(
    ::AbstractPiecewiseLinearBreakpointParameter,
    ts_elem::IS.PiecewiseStepData,
    expected_axs,
)
    max_len = length(only(expected_axs))
    x_coords = IS.get_x_coords(ts_elem)
    @assert length(x_coords) <= max_len
    fill_value = x_coords[end]  # if padding is necessary, repeat the last breakpoint so dx = 0 (see above)
    padded_x_coords = vcat(x_coords, fill(fill_value, max_len - length(x_coords)))
    return padded_x_coords
end

# PERF: compilation hotspot. Switch to TSC.
# NOTE direct equivalent of _add_time_series_parameters! for TimeSeriesParameter
function _add_parameters!(
    container::OptimizationContainer,
    param::T,
    devices::U,
    model::DeviceModel{D, W},
) where {
    T <: ObjectiveFunctionParameter,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    ts_type = get_default_time_series_type(container)
    if !(ts_type <: Union{PSY.AbstractDeterministic, PSY.StaticTimeSeries})
        error(
            "add_parameters! for ObjectiveFunctionParameter is not compatible with $ts_type",
        )
    end
    time_steps = get_time_steps(container)

    ts_names = String[]
    device_names = String[]
    active_devices = D[]
    for device in devices
        ts_name = _get_time_series_name(T(), device, model)
        if PSY.has_time_series(device, ts_type, ts_name)
            push!(ts_names, ts_name)
            push!(device_names, PSY.get_name(device))
            push!(active_devices, device)
        else
            @debug "Skipped time series for $D, $(PSY.get_name(device))"
        end
    end
    if isempty(active_devices)
        return
    end
    jump_model = get_jump_model(container)

    additional_axes = calc_additional_axes(container, param, active_devices, model)
    param_container = add_param_container!(
        container,
        param,
        D,
        _param_to_vars(T(), W()),
        SOSStatusVariable.NO_VARIABLE,
        false,
        _get_expected_time_series_eltype(T()),
        device_names,
        additional_axes...,
        time_steps,
    )

    param_instance = T()
    for (ts_name, device_name, device) in zip(ts_names, device_names, active_devices)
        raw_ts_vals = get_time_series_initial_values!(container, ts_type, device, ts_name)
        ts_vals = _unwrap_for_param.(Ref(param_instance), raw_ts_vals, Ref(additional_axes))
        @assert all(_size_wrapper.(ts_vals) .== Ref(length.(additional_axes)))
        for step in time_steps
            set_parameter!(param_container, jump_model, ts_vals[step], device_name, step)
            set_multiplier!(
                param_container,
                get_multiplier_value(T(), device, W()),
                device_name,
                step,
            )
        end
    end
    return
end

function _add_parameters!(
    container::OptimizationContainer,
    ::T,
    service::U,
    model::ServiceModel{U, V},
) where {T <: TimeSeriesParameter, U <: PSY.Service, V <: AbstractServiceFormulation}
    ts_type = get_default_time_series_type(container)
    if !(ts_type <: Union{PSY.AbstractDeterministic, PSY.StaticTimeSeries})
        error("add_parameters! for TimeSeriesParameter is not compatible with $ts_type")
    end
    ts_name = get_time_series_names(model)[T]
    time_steps = get_time_steps(container)
    name = PSY.get_name(service)
    ts_uuid = string(IS.get_time_series_uuid(ts_type, service, ts_name))
    @debug "adding" T U _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    additional_axes = calc_additional_axes(container, T(), [service], model)
    parameter_container = add_param_container!(
        container,
        T(),
        U,
        ts_type,
        ts_name,
        [ts_uuid],
        [name],
        additional_axes,
        time_steps;
        meta = name,
    )

    set_subsystem!(get_attributes(parameter_container), get_subsystem(model))
    jump_model = get_jump_model(container)
    ts_vector = get_time_series(container, service, T(), name)
    multiplier = get_multiplier_value(T(), service, V())
    for t in time_steps
        set_multiplier!(parameter_container, multiplier, name, t)
        set_parameter!(parameter_container, jump_model, ts_vector[t], ts_uuid, t)
    end
    add_component_name!(get_attributes(parameter_container), name, ts_uuid)
    return
end

function _add_parameters!(
    container::OptimizationContainer,
    ::T,
    key::VariableKey{U, D},
    model::DeviceModel{D, W},
    devices::V,
) where {
    T <: VariableValueParameter,
    U <: VariableType,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    @debug "adding" T D U _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    names = [PSY.get_name(device) for device in devices]
    time_steps = get_time_steps(container)
    parameter_container = add_param_container!(container, T(), D, key, names, time_steps)
    jump_model = get_jump_model(container)
    for d in devices
        name = PSY.get_name(d)
        if get_variable_warm_start_value(U(), d, W()) === nothing
            inital_parameter_value = 0.0
        else
            inital_parameter_value = get_variable_warm_start_value(U(), d, W())
        end
        for t in time_steps
            set_multiplier!(
                parameter_container,
                get_parameter_multiplier(T(), d, W()),
                name,
                t,
            )
            set_parameter!(
                parameter_container,
                jump_model,
                inital_parameter_value,
                name,
                t,
            )
        end
    end
    return
end

function _add_parameters!(
    container::OptimizationContainer,
    ::T,
    key::VariableKey{U, D},
    model::DeviceModel{D, W},
    devices::V,
) where {
    T <: OnStatusParameter,
    U <: OnVariable,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractThermalFormulation,
} where {D <: PSY.ThermalGen}
    @debug "adding" T D U _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    names = [PSY.get_name(device) for device in devices if !PSY.get_must_run(device)]
    time_steps = get_time_steps(container)
    parameter_container = add_param_container!(container, T(), D, key, names, time_steps)
    jump_model = get_jump_model(container)
    for d in devices
        if PSY.get_must_run(d)
            continue
        end
        name = PSY.get_name(d)
        if get_variable_warm_start_value(U(), d, W()) === nothing
            inital_parameter_value = 0.0
        else
            inital_parameter_value = get_variable_warm_start_value(U(), d, W())
        end
        for t in time_steps
            set_multiplier!(
                parameter_container,
                get_parameter_multiplier(T(), d, W()),
                name,
                t,
            )
            set_parameter!(
                parameter_container,
                jump_model,
                inital_parameter_value,
                name,
                t,
            )
        end
    end
    return
end

function _add_parameters!(
    container::OptimizationContainer,
    ::T,
    key::VariableKey{U, D},
    model::DeviceModel{D, W},
    devices::V,
) where {
    T <: FixValueParameter,
    U <: VariableType,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    @debug "adding" T D U _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    names = [PSY.get_name(device) for device in devices]
    time_steps = get_time_steps(container)
    parameter_container =
        add_param_container!(container, T(), D, key, names, time_steps; meta = "$U")
    jump_model = get_jump_model(container)
    for d in devices
        name = PSY.get_name(d)
        if get_variable_warm_start_value(U(), d, W()) === nothing
            inital_parameter_value = 0.0
        else
            inital_parameter_value = get_variable_warm_start_value(U(), d, W())
        end
        for t in time_steps
            set_multiplier!(
                parameter_container,
                get_parameter_multiplier(T(), d, W()),
                name,
                t,
            )
            set_parameter!(
                parameter_container,
                jump_model,
                inital_parameter_value,
                name,
                t,
            )
        end
    end
    return
end

function _add_parameters!(
    container::OptimizationContainer,
    ::T,
    key::AuxVarKey{U, D},
    model::DeviceModel{D, W},
    devices::V,
) where {
    T <: VariableValueParameter,
    U <: AuxVariableType,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    @debug "adding" T D U _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    names = [PSY.get_name(device) for device in devices]
    time_steps = get_time_steps(container)
    parameter_container = add_param_container!(
        container,
        T(),
        D,
        key,
        names,
        time_steps,
    )
    jump_model = get_jump_model(container)

    for d in devices
        name = PSY.get_name(d)
        for t in time_steps
            set_multiplier!(
                parameter_container,
                get_parameter_multiplier(T(), d, W()),
                name,
                t,
            )
            set_parameter!(
                parameter_container,
                jump_model,
                get_initial_parameter_value(T(), d, W()),
                name,
                t,
            )
        end
    end
    return
end

function _add_parameters!(
    container::OptimizationContainer,
    ::T,
    devices::V,
    model::DeviceModel{D, W},
) where {
    T <: OnStatusParameter,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    @debug "adding" T D V _group = LOG_GROUP_OPTIMIZATION_CONTAINER

    # We do this to handle cases where the same parameter is also added as a Feedforward.
    # When the OnStatusParameter is added without a feedforward it takes a Float value.
    # This is used to handle the special case of compact formulations.
    !isempty(get_feedforwards(model)) && return
    names = [PSY.get_name(device) for device in devices]
    time_steps = get_time_steps(container)
    parameter_container = add_param_container!(
        container,
        T(),
        D,
        VariableKey(OnVariable, D),
        names,
        time_steps,
    )
    jump_model = get_jump_model(container)

    for d in devices
        name = PSY.get_name(d)
        for t in time_steps
            set_multiplier!(
                parameter_container,
                get_parameter_multiplier(T(), d, W()),
                name,
                t,
            )
            set_parameter!(
                parameter_container,
                jump_model,
                get_initial_parameter_value(T(), d, W()),
                name,
                t,
            )
        end
    end
    return
end

function _add_parameters!(
    container::OptimizationContainer,
    ::T,
    key::VariableKey{U, S},
    model::ServiceModel{S, W},
    devices::V,
) where {
    S <: PSY.AbstractReserve,
    T <: VariableValueParameter,
    U <: VariableType,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractReservesFormulation,
} where {D <: PSY.Component}
    @debug "adding" T D U _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    contributing_devices = get_contributing_devices(model)
    names = [PSY.get_name(device) for device in contributing_devices]
    time_steps = get_time_steps(container)
    parameter_container = add_param_container!(
        container,
        T(),
        S,
        key,
        names,
        time_steps;
        meta = get_service_name(model),
    )
    jump_model = get_jump_model(container)
    for d in contributing_devices
        name = PSY.get_name(d)
        for t in time_steps
            set_multiplier!(
                parameter_container,
                get_parameter_multiplier(T(), S, W()),
                name,
                t,
            )
            set_parameter!(
                parameter_container,
                jump_model,
                get_initial_parameter_value(T(), S, W()),
                name,
                t,
            )
        end
    end
    return
end
