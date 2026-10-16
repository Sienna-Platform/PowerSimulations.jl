struct SimulationState
    current_time::Base.RefValue{Dates.DateTime}
    last_decision_model::Base.RefValue{Symbol}
    decision_states::DatasetContainer{InMemoryDataset}
    system_states::DatasetContainer{InMemoryDataset}
end

function SimulationState()
    return SimulationState(
        Ref(UNSET_INI_TIME),
        Ref(:None),
        DatasetContainer{InMemoryDataset}(),
        DatasetContainer{InMemoryDataset}(),
    )
end

IOM.get_current_time(s::SimulationState) = s.current_time[]
get_last_decision_model(s::SimulationState) = s.last_decision_model[]
get_decision_states(s::SimulationState) = s.decision_states
get_system_states(s::SimulationState) = s.system_states

# Not to be used in hot loops
function get_system_states_resolution(s::SimulationState)
    system_state = get_system_states(s)
    # All the system states have the same resolution
    return get_data_resolution(
        get_dataset(system_state, first(get_dataset_keys(system_state))),
    )
end

function set_current_time!(s::SimulationState, val::Dates.DateTime)
    s.current_time[] = val
    return
end

function set_last_decision_model!(s::SimulationState, val::Symbol)
    s.last_decision_model[] = val
    return
end

const STATE_TIME_PARAMS = NamedTuple{(:horizon, :resolution), NTuple{2, Dates.Millisecond}}

function _get_state_params(models::SimulationModels, simulation_step::Dates.Millisecond)
    params = OrderedDict{OptimizationContainerKey, STATE_TIME_PARAMS}()
    for model in get_decision_models(models)
        container = get_optimization_container(model)
        model_resolution = get_resolution(model)
        model_interval = get_interval(model)
        horizon_length = get_horizon(model)
        # This is the portion of the Horizon that "overflows" into the next step
        time_residual = horizon_length - model_interval
        @assert_op time_residual >= zero(Dates.Millisecond)
        num_runs = simulation_step / model_interval
        total_time = (num_runs - 1) * model_interval + horizon_length
        for type in fieldnames(DatasetContainer)
            field_containers = getfield(container, type)
            for key in keys(field_containers)
                !should_write_resulting_value(key) && continue
                if !haskey(params, key)
                    params[key] = (
                        horizon = max(simulation_step + time_residual, total_time),
                        resolution = model_resolution,
                    )
                else
                    params[key] = (
                        horizon = max(params[key].horizon, total_time),
                        resolution = min(params[key].resolution, model_resolution),
                    )
                end
                @debug get_name(model) key params[key]
            end
        end
    end
    model = get_emulation_model(models)
    if !isnothing(model)
        container = get_optimization_container(model)
        model_resolution = get_resolution(model)
        for type in fieldnames(DatasetContainer)
            field_containers = getfield(container, type)
            for key in keys(field_containers)
                !should_write_resulting_value(key) && continue
                if !haskey(params, key)
                    @warn "New parameter $key found in emulator only; \
                        not registered by any decision model"
                else
                    params[key] = (
                        horizon = params[key].horizon,
                        resolution = min(params[key].resolution, model_resolution),
                    )
                end
                @debug get_name(model) key params[key]
            end
        end
    end
    return params
end

_state_models(models::SimulationModels) = _state_models(models, get_emulation_model(models))
_state_models(models::SimulationModels, ::Nothing) = get_decision_models(models)
_state_models(models::SimulationModels, em::EmulationModel) =
    vcat(get_decision_models(models), em)

_axis_labels(axis) = [string(label) for label in axis]

"""
Merge the columns that `model_name` has for `key` into the state columns collected from
`owners` so far. Simulation state keeps one dataset per key for all models, so the component
axis (the first) is the union across models, in the order the models list them. Every other
axis must be the same in every model. Returns `current` itself when `new` adds no component.
"""
function _union_state_columns(
    key::OptimizationContainerKey,
    current::Tuple,
    owners,
    new::Tuple,
    model_name::Symbol,
)
    if length(current) != length(new) ||
       any(_axis_labels(current[i]) != _axis_labels(new[i]) for i in 2:length(current))
        throw(
            IS.InvalidValue(
                "Simulation state for $(encode_key_as_string(key)) needs the same axes \
                besides the component axis in every model. Model $model_name has \
                $(_axis_labels.(new[2:end])) and model(s) $(join(owners, ", ")) have \
                $(_axis_labels.(current[2:end])).",
            ),
        )
    end
    known = Set(current[1])
    added = [name for name in new[1] if !(name in known)]
    isempty(added) && return current
    return (vcat(current[1], added), current[2:end]...)
end

function _get_state_columns(models::SimulationModels)
    columns = Dict{OptimizationContainerKey, Tuple}()
    owners = Dict{OptimizationContainerKey, Vector{Symbol}}()
    for model in _state_models(models)
        container = get_optimization_container(model)
        model_name = get_name(model)
        for field in fieldnames(DatasetContainer)
            for (key, value) in getfield(container, field)
                !should_write_resulting_value(key) && continue
                model_columns = get_column_names(container, field, value, key)
                if haskey(columns, key)
                    columns[key] = _union_state_columns(
                        key,
                        columns[key],
                        owners[key],
                        model_columns,
                        model_name,
                    )
                    push!(owners[key], model_name)
                else
                    columns[key] = model_columns
                    owners[key] = [model_name]
                end
            end
        end
    end
    return columns
end

function _initialize_decision_states!(
    sim_state::SimulationState,
    simulation_initial_time::Dates.DateTime,
    simulation_step::Dates.Millisecond,
    params::OrderedDict{OptimizationContainerKey, STATE_TIME_PARAMS},
    columns::Dict{OptimizationContainerKey, Tuple},
)
    states = get_decision_states(sim_state)
    for (key, key_params) in params
        set_dataset!(
            states,
            key,
            InMemoryDataset(
                NaN,
                simulation_initial_time,
                key_params.resolution,
                Int(simulation_step / key_params.resolution),
                key_params.horizon ÷ key_params.resolution,
                columns[key],
            ),
        )
    end
    return
end

function _initialize_system_states!(
    sim_state::SimulationState,
    ::Nothing,
    simulation_initial_time::Dates.DateTime,
    params::OrderedDict{OptimizationContainerKey, STATE_TIME_PARAMS},
    columns::Dict{OptimizationContainerKey, Tuple},
)
    _set_system_states!(
        sim_state,
        get_dataset_keys(get_decision_states(sim_state)),
        simulation_initial_time,
        params,
        columns,
    )
    return
end

function _initialize_system_states!(
    sim_state::SimulationState,
    emulation_model::EmulationModel,
    simulation_initial_time::Dates.DateTime,
    params::OrderedDict{OptimizationContainerKey, STATE_TIME_PARAMS},
    columns::Dict{OptimizationContainerKey, Tuple},
)
    state_keys = Set{OptimizationContainerKey}(
        get_dataset_keys(get_decision_states(sim_state)),
    )
    for key in get_container_keys(get_optimization_container(emulation_model))
        should_write_resulting_value(key) && push!(state_keys, key)
    end
    _set_system_states!(sim_state, state_keys, simulation_initial_time, params, columns)
    return
end

function _set_system_states!(
    sim_state::SimulationState,
    state_keys,
    simulation_initial_time::Dates.DateTime,
    params::OrderedDict{OptimizationContainerKey, STATE_TIME_PARAMS},
    columns::Dict{OptimizationContainerKey, Tuple},
)
    system_states = get_system_states(sim_state)
    min_res = minimum([v.resolution for v in values(params)])
    for key in state_keys
        set_dataset!(
            system_states,
            key,
            make_system_state(simulation_initial_time, min_res, columns[key]),
        )
    end
    return
end

"""
Create the decision and system states of a simulation. There is one dataset per key for all
models; its component axis is the union of the components the models have for the key (see
`update_missing_state_columns!`).
"""
function initialize_simulation_state!(
    sim_state::SimulationState,
    models::SimulationModels,
    simulation_step::Dates.Millisecond,
    simulation_initial_time::Dates.DateTime,
)
    params = _get_state_params(models, simulation_step)
    columns = _get_state_columns(models)
    _initialize_decision_states!(
        sim_state,
        simulation_initial_time,
        simulation_step,
        params,
        columns,
    )
    set_last_decision_model!(sim_state, get_name(last(get_decision_models(models))))
    em = get_emulation_model(models)
    _initialize_system_states!(sim_state, em, simulation_initial_time, params, columns)
    return
end

"""
Floor `timestamp` onto `dataset`'s own time grid: the row containing it. On-grid timestamps
are returned unchanged. IOM's `find_timestamp_index` rejects off-grid timestamps, so a finer
simulation clock must be aligned before reading a coarser dataset.
"""
function align_to_dataset_grid(dataset::InMemoryDataset, timestamp::Dates.DateTime)
    grid_start = first(dataset.timestamps)
    if timestamp < grid_start
        error(
            "Timestamp $timestamp precedes the dataset grid starting at $grid_start; " *
            "a read this early has no containing row",
        )
    end
    resolution = get_data_resolution(dataset)
    return grid_start + ((timestamp - grid_start) ÷ resolution) * resolution
end

function find_aligned_timestamp_index(dataset::InMemoryDataset, timestamp::Dates.DateTime)
    return find_timestamp_index(
        dataset.timestamps,
        align_to_dataset_grid(dataset, timestamp),
    )
end

function update_decision_state!(
    state::SimulationState,
    key::ParameterKey{AvailableStatusChangeCountdownParameter, T},
    store_data::DenseAxisArray{Float64, 2},
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: PSY.Component}
    state_data = get_decision_state_data(state, key)
    column_names = axes(store_data)[1]
    model_resolution = get_resolution(model_params)
    state_resolution = get_data_resolution(state_data)
    resolution_ratio = model_resolution ÷ state_resolution
    state_timestamps = state_data.timestamps
    @assert_op resolution_ratio >= 1
    if simulation_time > get_end_of_step_timestamp(state_data)
        state_data_index = 1
        state_data.timestamps[:] .=
            range(
                simulation_time;
                step = state_resolution,
                length = get_num_rows(state_data),
            )
    else
        state_data_index = find_timestamp_index(state_timestamps, simulation_time)
    end
    offset = resolution_ratio - 1
    result_time_index = axes(store_data)[2]
    set_update_timestamp!(state_data, simulation_time)
    for t in result_time_index
        state_range = state_data_index:(state_data_index + offset)
        for name in column_names
            # One countdown step per state step inside the model's interval.
            trajectory =
                POM.countdown_trajectory(store_data[name, t], length(state_range))
            for (ix, i) in enumerate(state_range)
                state_data.values[name, i] = trajectory[ix]
            end
        end
        set_last_recorded_row!(state_data, state_range[end])
        state_data_index += resolution_ratio
    end
    return
end

function update_decision_state!(
    state::SimulationState,
    key::ParameterKey{AvailableStatusParameter, T},
    store_data::DenseAxisArray{Float64, 2},
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: PSY.Component}
    state_data = get_decision_state_data(state, key)
    countdown_data =
        get_decision_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    column_names = axes(store_data)[1]
    model_resolution = get_resolution(model_params)
    state_resolution = get_data_resolution(state_data)
    resolution_ratio = model_resolution ÷ state_resolution
    state_timestamps = state_data.timestamps
    @assert_op resolution_ratio >= 1

    if simulation_time > get_end_of_step_timestamp(state_data)
        state_data_index = 1
        state_data.timestamps[:] .=
            range(
                simulation_time;
                step = state_resolution,
                length = get_num_rows(state_data),
            )
    else
        state_data_index = find_timestamp_index(state_timestamps, simulation_time)
    end

    offset = resolution_ratio - 1
    result_time_index = axes(store_data)[2]
    set_update_timestamp!(state_data, simulation_time)
    for t in result_time_index
        state_range = state_data_index:(state_data_index + offset)
        for name in column_names, i in state_range
            state_data.values[name, i] =
                POM.availability_from_countdown(countdown_data.values[name, i])
        end
        set_last_recorded_row!(state_data, state_range[end])
        state_data_index += resolution_ratio
    end
    return
end

function update_decision_state!(
    state::SimulationState,
    key::ParameterKey{ActivePowerOffsetParameter, T},
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: PSY.Component}
    event_occurrence_data =
        get_decision_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    activepower_data =
        get_decision_state_data(state, ActivePowerTimeSeriesParameter(), T)
    state_data = get_decision_state_data(state, key)
    model_resolution = get_resolution(model_params)
    state_resolution = get_data_resolution(state_data)
    resolution_ratio = model_resolution ÷ state_resolution
    @assert_op resolution_ratio >= 1
    state_data_index = find_aligned_timestamp_index(state_data, simulation_time)
    # The offset follows the countdown: it cancels the device's own time series while
    # the outage runs and is cleared everywhere else, so a stale offset cannot survive.
    for name in column_names, ix in axes(state_data.values)[2]
        state_data.values[name, ix] = POM.outage_power_offset(
            event_occurrence_data.values[name, ix],
            activepower_data.values[name, ix],
        )
    end
    return
end

"""
Write a model's results for `key` into the decision state. The state holds the union of the
key's components across models, and each model writes only the components it has, so the
last model to write a component wins. The components a model lacks are handled by
`update_missing_state_columns!`.
"""
function update_decision_state!(
    state::SimulationState,
    key::OptimizationContainerKey,
    store_data::DenseAxisArray{Float64, 2},
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
)
    state_data = get_decision_state_data(state, key)
    column_names = axes(store_data)[1]
    model_resolution = get_resolution(model_params)
    state_resolution = get_data_resolution(state_data)
    resolution_ratio = model_resolution ÷ state_resolution
    state_timestamps = state_data.timestamps
    @assert_op resolution_ratio >= 1

    if simulation_time > get_end_of_step_timestamp(state_data)
        state_data_index = 1
        state_data.timestamps[:] .=
            range(
                simulation_time;
                step = state_resolution,
                length = get_num_rows(state_data),
            )
    else
        state_data_index = find_timestamp_index(state_timestamps, simulation_time)
    end

    offset = resolution_ratio - 1
    result_time_index = axes(store_data)[2]
    set_update_timestamp!(state_data, simulation_time)
    for t in result_time_index
        state_range = state_data_index:(state_data_index + offset)
        for name in column_names, i in state_range
            # TODO: We could also interpolate here
            state_data.values[name, i] = store_data[name, t]
        end
        set_last_recorded_row!(state_data, state_range[end])
        state_data_index += resolution_ratio
    end
    return
end

function update_decision_state!(
    state::SimulationState,
    key::OptimizationContainerKey,
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
)
    return
end

function update_decision_state!(
    state::SimulationState,
    key::ParameterKey{AvailableStatusChangeCountdownParameter, T},
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    ::ModelStoreParams,
) where {T <: PSY.Component}
    # The system state already holds the countdown POM computed for this step; the
    # decision state gets that countdown projected across the rest of the horizon.
    event_occurrence_data =
        get_system_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    event_occurrence_values = get_last_recorded_value(event_occurrence_data)
    state_data = get_decision_state_data(state, key)
    state_timestamps = state_data.timestamps
    state_data_index = find_aligned_timestamp_index(state_data, simulation_time)
    set_update_timestamp!(state_data, simulation_time)
    for name in column_names
        countdown = event_occurrence_values[name, 1]
        # The outage takes effect at the step *after* the one that detected it, which is
        # also the step the occurrence was read from, so the projection starts one index
        # ahead. An outage running past the end of the state window is truncated here;
        # the countdown survives in the system state, so the next projection continues it.
        remaining = length(state_timestamps) - state_data_index
        trajectory = POM.countdown_trajectory(countdown, remaining)
        for (ix, value) in enumerate(trajectory)
            state_data.values[name, state_data_index + ix] = value
        end
    end
    return
end

function update_decision_state!(
    state::SimulationState,
    key::ParameterKey{AvailableStatusParameter, T},
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: PSY.Component}
    event_occurrence_data =
        get_decision_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    state_data = get_decision_state_data(state, key)
    model_resolution = get_resolution(model_params)
    state_resolution = get_data_resolution(state_data)
    resolution_ratio = model_resolution ÷ state_resolution
    @assert_op resolution_ratio >= 1
    state_data_index = find_aligned_timestamp_index(state_data, simulation_time)
    for name in column_names, ix in axes(state_data.values)[2]
        state_data.values[name, ix] =
            POM.availability_from_countdown(event_occurrence_data.values[name, ix])
    end
    return
end

function update_decision_state!(
    state::SimulationState,
    key::AuxVarKey{TimeDurationOn, T},
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: PSY.Component}
    event_occurrence_data =
        get_decision_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    state_data = get_decision_state_data(state, key)

    state_data_index = find_aligned_timestamp_index(state_data, simulation_time)
    event_occurence_index =
        find_aligned_timestamp_index(event_occurrence_data, simulation_time)
    for name in column_names
        if event_occurrence_data.values[name, event_occurence_index] > 0.0
            state_data.values[name, (state_data_index + 1):end] .=
                MISSING_INITIAL_CONDITIONS_TIME_COUNT
        end
    end
    return
end

function update_decision_state!(
    state::SimulationState,
    key::AuxVarKey{TimeDurationOff, T},
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: PSY.Component}
    event_occurrence_data =
        get_decision_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    state_data = get_decision_state_data(state, key)

    state_data_index = find_aligned_timestamp_index(state_data, simulation_time)
    event_occurence_index =
        find_aligned_timestamp_index(event_occurrence_data, simulation_time)
    for name in column_names
        if event_occurrence_data.values[name, event_occurence_index] > 0.0
            for (time_off, ix) in
                enumerate((state_data_index + 1):length(state_data.values[name, :]))
                state_data.values[name, ix] = time_off
            end
        end
    end
    return
end

function update_decision_state!(
    state::SimulationState,
    key::VariableKey{T, U},
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: Union{ActivePowerVariable, OnVariable}, U <: PSY.Component}
    event_occurrence_data =
        get_decision_state_data(state, AvailableStatusChangeCountdownParameter(), U)
    state_data = get_decision_state_data(state, key)

    state_data_index = find_aligned_timestamp_index(state_data, simulation_time)
    event_occurence_index =
        find_aligned_timestamp_index(event_occurrence_data, simulation_time)
    for name in column_names
        if event_occurrence_data.values[name, event_occurence_index] > 0.0
            state_data.values[name, (state_data_index + 1):end] .= 0.0
        end
    end
    return
end
function update_decision_state!(
    state::SimulationState,
    key::OptimizationContainerKey,
    store_data::DenseAxisArray{Float64, 3},
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
)
    state_data = get_decision_state_data(state, key)
    outages = axes(store_data)[1]

    model_resolution = get_resolution(model_params)
    state_resolution = get_data_resolution(state_data)
    resolution_ratio = model_resolution ÷ state_resolution
    state_timestamps = state_data.timestamps
    @assert_op resolution_ratio >= 1

    if simulation_time > get_end_of_step_timestamp(state_data)
        state_data_index = 1
        state_data.timestamps[:] .=
            range(
                simulation_time;
                step = state_resolution,
                length = get_num_rows(state_data),
            )
    else
        state_data_index = find_timestamp_index(state_timestamps, simulation_time)
    end

    offset = resolution_ratio - 1
    result_time_index = axes(store_data)[3]
    # The state names its second axis as the output's column names do: always strings.
    labels = axes(store_data)[2]
    columns = get_column_names_from_axis_array(store_data)[2]
    set_update_timestamp!(state_data, simulation_time)
    for t in result_time_index
        state_range = state_data_index:(state_data_index + offset)
        for (name, column) in zip(labels, columns), i in state_range
            for outage in outages
                # TODO: We could also interpolate here
                state_data.values[outage, column, i] = store_data[outage, name, t]
            end
        end
        set_last_recorded_row!(state_data, state_range[end])
        state_data_index += resolution_ratio
    end
    return
end

function update_decision_state!(
    state::SimulationState,
    key::AuxVarKey{S, T},
    store_data::DenseAxisArray{Float64, 2},
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: PSY.Component, S <: Union{TimeDurationOff, TimeDurationOn}}
    state_data = get_decision_state_data(state, key)
    model_resolution = get_resolution(model_params)
    state_resolution = get_data_resolution(state_data)
    resolution_ratio = model_resolution ÷ state_resolution
    @assert_op resolution_ratio >= 1

    if simulation_time > get_end_of_step_timestamp(state_data)
        state_data_index = 1
        state_data.timestamps[:] .=
            range(
                simulation_time;
                step = state_resolution,
                length = get_num_rows(state_data),
            )
    else
        state_data_index = find_timestamp_index(state_data.timestamps, simulation_time)
    end

    offset = resolution_ratio - 1
    result_time_index = axes(store_data)[2]
    set_update_timestamp!(state_data, simulation_time)

    if isone(resolution_ratio)
        increment_per_period = 1.0
    elseif state_resolution < Dates.Day(365) && state_resolution > Dates.Minute(1)
        increment_per_period = Dates.value(Dates.Minute(state_resolution))
    else
        error("Incorrect Problem Resolution specification: $(state_resolution)")
    end

    column_names = axes(store_data)[1]
    for t in result_time_index
        state_range = state_data_index:(state_data_index + offset)
        @assert_op state_range[end] <= get_num_rows(state_data)
        for name in column_names, i in state_range
            if t == 1 && i == 1
                state_data.values[name, i] = store_data[name, t] * resolution_ratio
            else
                state_data.values[name, i] =
                    if store_data[name, t] > 0
                        state_data.values[name, i - 1] + increment_per_period
                    else
                        0
                    end
            end
        end
        set_last_recorded_row!(state_data, state_range[end])
        state_data_index += resolution_ratio
    end

    return
end

"""
    update_missing_state_columns!(
        state_data::InMemoryDataset,
        key::OptimizationContainerKey,
        model::IOM.AbstractOptimizationModel,
        columns::AbstractVector{String},
        state_rows::UnitRange{Int},
        row_timestamps::AbstractVector{Dates.DateTime},
    )

Extension point for the state components a model does not produce.

A simulation keeps one state dataset per key for all models, over the union of the
components the models have for that key. When `model` updates the state it writes the
components it has, and the last model to write a component wins. This function is then
called with the components of `key` that `model` lacks (`columns`, on the dataset's first
axis), and also with every component of each state key that `model` does not have at all.

`state_rows` are the rows of `state_data` that cover `model`'s window at the dataset's
resolution, and `row_timestamps` their times. For a key the model does not write, the dataset
is first advanced to the model's window, as it is for a model that writes the key, so the
rows always start at the model's current time; they stop at the end of the dataset. An implementation may write `state_data.values[column, ..., row]` for those
columns and rows, computing the values from `model`'s own data (for example its system). It
must not change other rows, the dataset's timestamps or its update bookkeeping.

The default writes nothing: a component keeps the value its last writer left, and a component
no model has written keeps the initial `NaN`.

It is called for the decision state after a decision model updates it, with the rows of the
model's horizon, and for the system state after the emulator updates it, with `state_rows`
`1:1`. In the system state, the decision-state values copied after the emulator update
overwrite the components the emulator does not have.
"""
function update_missing_state_columns!(
    ::InMemoryDataset,
    ::OptimizationContainerKey,
    ::IOM.AbstractOptimizationModel,
    ::AbstractVector{String},
    ::UnitRange{Int},
    ::AbstractVector{Dates.DateTime},
)
    return
end

# Advance a dataset to the window starting at `simulation_time`, as a model writing the key
# does when the window passes the end of the dataset's step. A model writing the key then
# overwrites the rows it covers; here no row is written, so the rows keep their timestamps:
# they move up by the number of rows the window advanced, and the rows that enter the window
# hold the values of the last row.
function _advance_state_window!(
    state_data::InMemoryDataset,
    simulation_time::Dates.DateTime,
)
    simulation_time > get_end_of_step_timestamp(state_data) || return
    timestamps = state_data.timestamps
    resolution = get_data_resolution(state_data)
    window_start = align_to_dataset_grid(state_data, simulation_time)
    shift = (window_start - first(timestamps)) ÷ resolution
    num_rows = length(timestamps)
    values = state_data.values.data
    time_axis = ndims(values)
    for row in 1:num_rows
        source = min(row + shift, num_rows)
        selectdim(values, time_axis, row) .= selectdim(values, time_axis, source)
    end
    timestamps .= range(window_start; step = resolution, length = num_rows)
    set_last_recorded_row!(state_data, max(get_last_recorded_row(state_data) - shift, 0))
    return
end

# The rows of a dataset that cover `[simulation_time, simulation_time + horizon)`, starting at
# the row that contains `simulation_time`.
function _window_rows(
    state_data::InMemoryDataset,
    simulation_time::Dates.DateTime,
    horizon::Dates.Millisecond,
)
    timestamps = state_data.timestamps
    first_row = max(searchsortedlast(timestamps, simulation_time), 1)
    last_row = searchsortedfirst(timestamps, simulation_time + horizon) - 1
    return first_row:last_row
end

# The state components missing from `model_columns`, or `nothing` when there are none. The
# model's components are a subset of the state's, so equal counts mean none are missing.
function _missing_state_columns(state_data::InMemoryDataset, model_columns)
    state_columns = axes(state_data.values)[1]
    length(model_columns) == length(state_columns) && return nothing
    present = Set(model_columns)
    return String[name for name in state_columns if !(name in present)]
end

function get_decision_state_data(state::SimulationState, key::OptimizationContainerKey)
    return get_dataset(get_decision_states(state), key)
end

function get_decision_state_value(state::SimulationState, key::OptimizationContainerKey)
    return get_dataset_values(get_decision_states(state), key)
end

function get_decision_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {
    T <: Union{VariableType, AuxVariableType, ConstraintType, ParameterType},
    U <: Union{PSY.Component, PSY.System},
}
    return get_dataset(get_decision_states(state), T, U)
end

function get_decision_state_value(
    state::SimulationState,
    key::OptimizationContainerKey,
    date::Dates.DateTime,
)
    return get_dataset_values(get_decision_states(state), key, date)
end

function get_system_state_data(state::SimulationState, key::OptimizationContainerKey)
    return get_dataset(get_system_states(state), key)
end

"""
Error when a state value read for a component is not finite. Every state value starts as
`NaN`, so a non-finite value means no model has written it.
"""
function _check_state_value(
    value::Float64,
    key::OptimizationContainerKey,
    component_name::AbstractString,
)
    isfinite(value) || _throw_state_value_error(value, key, component_name)
    return
end

@noinline function _throw_state_value_error(
    value::Float64,
    key::OptimizationContainerKey,
    component_name::AbstractString,
)
    throw(
        IS.InvalidValue(
            "The simulation state value of $(component_name) in \
            $(encode_key_as_string(key)) is $(value). Either no model has made this decision \
            for the time being read, or no model writes this component for this key, which \
            happens when device filters or formulations differ across models. Review the \
            models' horizons, intervals and device filters.",
        ),
    )
end

function _get_state_value(
    values::DenseAxisArray{Float64, 1},
    key::OptimizationContainerKey,
    component_name::AbstractString,
)
    value = values[component_name]
    _check_state_value(value, key, component_name)
    return value
end

function get_system_state_value(state::SimulationState, key::OptimizationContainerKey)
    return get_dataset_values(get_system_states(state), key)[:, 1]
end

function get_system_state_value(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {
    T <: Union{VariableType, AuxVariableType, ConstraintType, ParameterType},
    U <: Union{PSY.Component, PSY.System},
}
    return get_dataset_values(get_system_states(state), T, U)[:, 1]
end

function get_system_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {
    T <: Union{VariableType, AuxVariableType, ConstraintType, ParameterType},
    U <: Union{PSY.Component, PSY.System},
}
    return get_dataset(get_system_states(state), T, U)
end

function update_system_state!(
    state::DatasetContainer{InMemoryDataset},
    key::OptimizationContainerKey,
    store::SimulationStore,
    model_name::Symbol,
    simulation_time::Dates.DateTime,
)
    em_data = get_em_data(store)
    ix = get_last_recorded_row(em_data, key)
    res = read_result(DenseAxisArray, store, model_name, key, ix)
    dataset = get_dataset(state, key)
    set_update_timestamp!(dataset, simulation_time)
    _set_state_row!(dataset, _last_recorded_state_value(store, res, ix), 1)
    set_last_recorded_row!(dataset, 1)
    return
end

# Write one row of a model's output into a state dataset that may hold more components than
# the model has, or the same components in another order. The output has the components on
# its first axis, optionally followed by a trailing time axis of length one.
function _set_state_row!(dataset::InMemoryDataset, value::DenseAxisArray, row::Int)
    if axes(value)[1] == axes(dataset.values)[1]
        set_value!(dataset, value, row)
    else
        for name in axes(value)[1]
            _set_component_row!(dataset, value, name, row)
        end
    end
    return
end

_set_component_row!(
    dataset::InMemoryDataset{2},
    value::DenseAxisArray{Float64, 1},
    name,
    row,
) =
    dataset.values[name, row] = value[name]
_set_component_row!(
    dataset::InMemoryDataset{2},
    value::DenseAxisArray{Float64, 2},
    name,
    row,
) =
    dataset.values[name, row] = value[name, first(axes(value)[2])]

function _set_component_row!(
    dataset::InMemoryDataset{3},
    value::DenseAxisArray{Float64, 2},
    name,
    row,
)
    component = value[name, :].data
    for (ix, column) in enumerate(axes(dataset.values)[2])
        dataset.values[name, column, row] = component[ix]
    end
    return
end

function _set_component_row!(
    dataset::InMemoryDataset{3},
    value::DenseAxisArray{Float64, 3},
    name,
    row,
)
    component = value[name, :, first(axes(value)[3])].data
    for (ix, column) in enumerate(axes(dataset.values)[2])
        dataset.values[name, column, row] = component[ix]
    end
    return
end

# Event parameters are exogenous state the models read, not results they produce. Copying
# the emulator's own values back over them resets the countdown every step, which is the
# outage's memory: the device would be re-outaged forever and never recover.
function update_system_state!(
    ::DatasetContainer{InMemoryDataset},
    ::ParameterKey{T, U},
    ::SimulationStore,
    ::Symbol,
    ::Dates.DateTime,
) where {T <: EventParameter, U <: PSY.Component}
    return
end

function update_system_state!(
    state::SimulationState,
    key::OptimizationContainerKey,
    column_names_::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    rng::AbstractRNG,
)
    return
end

function update_system_state!(
    state::SimulationState,
    key::ParameterKey{AvailableStatusParameter, T},
    column_names_::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    rng::AbstractRNG,
) where {T <: PSY.Device}
    available_status_parameter = get_system_state_data(state, key)
    available_status_parameter_values = get_last_recorded_value(available_status_parameter)

    available_status_change_parameter =
        get_system_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    available_status_change_parameter_values =
        get_last_recorded_value(available_status_change_parameter)

    for name in column_names_
        available_status_parameter.values[name, 1] =
            POM.availability_from_countdown(available_status_change_parameter_values[name])
    end
    return
end

function update_system_state!(
    state::SimulationState,
    key::ParameterKey{AvailableStatusChangeCountdownParameter, T},
    column_names_::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    rng::AbstractRNG,
    may_start::Bool = true,
) where {T <: PSY.Component}
    sym_state = get_system_states(state)
    system_dataset = get_dataset(sym_state, key)
    available_status_change_parameter = get_system_state_data(state, key)
    resolution = get_data_resolution(available_status_change_parameter)
    previous = get_last_recorded_value(available_status_change_parameter)
    set_update_timestamp!(system_dataset, simulation_time)

    for name in column_names_
        # POM decides whether an outage begins and how long it runs; the countdown it
        # returns is the number of state steps the device stays out.
        values = POM.event_step_values(
            event,
            event_model,
            simulation_time,
            previous[name];
            resolution = resolution,
            rng = rng,
            # The condition gates only whether a new outage may begin; the countdown
            # decays either way, which is why this runs every step.
            may_start = may_start,
            # TODO(events): PowerSystems documents `mean_time_to_recovery` in minutes
            # while this runtime has always read it as hours. Passing the unit
            # explicitly preserves existing behavior; the contradiction is upstream.
            mttr_units = Dates.Hour,
        )
        values.occurred &&
            @warn "Outage occurred at time $simulation_time for device $name"
        available_status_change_parameter.values[name, 1] = values.countdown
    end
    return
end

function update_system_state!(
    state::SimulationState,
    key::ParameterKey{ActivePowerOffsetParameter, T},
    column_names_::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    rng::AbstractRNG,
) where {T <: PSY.Component}
    available_status_parameter = get_system_state_data(state, AvailableStatusParameter(), T)
    available_status_parameter_values = get_last_recorded_value(available_status_parameter)

    available_status_change_parameter =
        get_system_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    available_status_change_parameter_values =
        get_last_recorded_value(available_status_change_parameter)

    active_power_offset_parameter = get_system_state_data(state, key)
    active_power_offset_parameter_values =
        get_last_recorded_value(active_power_offset_parameter)

    active_power_timeseries_parameter =
        get_system_state_data(state, ActivePowerTimeSeriesParameter(), T)
    active_power_timeseries_parameter_values =
        get_last_recorded_value(active_power_timeseries_parameter)

    for name in column_names_
        active_power_offset_parameter.values[name, 1] = POM.outage_power_offset(
            available_status_change_parameter_values[name],
            active_power_timeseries_parameter_values[name],
        )
    end
    return
end

function update_system_state!(
    state::SimulationState,
    key::AuxVarKey{TimeDurationOff, T},
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    rng::AbstractRNG,
) where {T <: PSY.Component}
    sym_state = get_system_states(state)
    event_occurrence_data =
        get_system_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    event_occurrence_values = get_last_recorded_value(event_occurrence_data)

    system_dataset = get_dataset(sym_state, key)
    current_status_data = get_system_state_data(state, key)
    current_status_values = get_last_recorded_value(current_status_data)
    set_update_timestamp!(system_dataset, simulation_time)
    for name in column_names
        if event_occurrence_values[name] > 0.0
            current_status_data.values[name, 1] = 0.0
        end
    end
    return
end

function update_system_state!(
    state::SimulationState,
    key::AuxVarKey{TimeDurationOn, T},
    column_names::Set{String},
    event::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    rng::AbstractRNG,
) where {T <: PSY.Component}
    sym_state = get_system_states(state)
    event_occurrence_data =
        get_system_state_data(state, AvailableStatusChangeCountdownParameter(), T)
    event_occurrence_values = get_last_recorded_value(event_occurrence_data)

    system_dataset = get_dataset(sym_state, key)
    current_status_data = get_system_state_data(state, key)
    current_status_values = get_last_recorded_value(current_status_data)
    set_update_timestamp!(system_dataset, simulation_time)
    for name in column_names
        if event_occurrence_values[name] > 0.0
            current_status_data.values[name, 1] = MISSING_INITIAL_CONDITIONS_TIME_COUNT
        end
    end
    return
end

function update_system_state!(
    state::SimulationState,
    key::VariableKey{T, U},
    column_names::Set{String},
    ::PSY.Outage,
    event_model::EventModel,
    simulation_time::Dates.DateTime,
    rng::AbstractRNG,
) where {T <: Union{ActivePowerVariable, OnVariable}, U <: PSY.Component}
    sym_state = get_system_states(state)
    event_occurrence_data =
        get_system_state_data(state, AvailableStatusChangeCountdownParameter(), U)
    event_occurrence_values = get_last_recorded_value(event_occurrence_data)

    system_dataset = get_dataset(sym_state, key)
    current_status_data = get_system_state_data(state, key)
    current_status_values = get_last_recorded_value(current_status_data)
    set_update_timestamp!(system_dataset, simulation_time)
    for name in column_names
        if event_occurrence_values[name] > 0.0
            current_status_data.values[name, 1] = 0.0
        end
    end
    return
end
function update_system_state!(
    state::DatasetContainer{InMemoryDataset},
    key::OptimizationContainerKey,
    decision_state::DatasetContainer{InMemoryDataset},
    simulation_time::Dates.DateTime,
)
    decision_dataset = get_dataset(decision_state, key)
    aligned_time = align_to_dataset_grid(decision_dataset, simulation_time)
    # Gets the timestamp of the value used for the update, which might not match exactly the
    # simulation time since the value might have not been updated yet
    ts = get_value_timestamp(decision_dataset, aligned_time)
    system_dataset = get_dataset(state, key)
    if ts == get_update_timestamp(system_dataset)
        return
    end

    # TODO: past-timestamp protection (`get_update_timestamp(system_dataset) > ts`)
    # was removed because the emulator now updates at a higher cadence than the decision
    # rate, so an event firing inside an hourly decision variable would update the whole
    # hour and trip the guard. New ordering logic is required to restore the safety net.

    # Writes the timestamp of the value used for the update
    set_update_timestamp!(system_dataset, ts)
    # Keep coordination between fields. System state is an array of size 1
    system_dataset.timestamps[1] = ts
    data_set_value = get_dataset_value(decision_dataset, aligned_time)
    set_dataset_values!(state, key, 1, data_set_value)
    # This value shouldn't be other than one and after one execution is no-op.
    set_last_recorded_row!(system_dataset, 1)
    return
end

function update_system_state!(
    state::DatasetContainer{InMemoryDataset},
    key::AuxVarKey{T, U},
    decision_state::DatasetContainer{InMemoryDataset},
    simulation_time::Dates.DateTime,
) where {T <: Union{TimeDurationOn, TimeDurationOff}, U <: PSY.Component}
    decision_dataset = get_dataset(decision_state, key)
    aligned_time = align_to_dataset_grid(decision_dataset, simulation_time)
    # Gets the timestamp of the value used for the update, which might not match exactly the
    # simulation time since the value might have not been updated yet

    ts = get_value_timestamp(decision_dataset, aligned_time)
    system_dataset = get_dataset(state, key)
    if ts == get_update_timestamp(system_dataset)
        return
    end
    # TODO: past-timestamp protection removed; see note in the dataset variant above.
    system_state_resolution = get_data_resolution(system_dataset)
    decision_state_resolution = get_data_resolution(decision_dataset)
    decision_state_value = get_dataset_value(decision_dataset, aligned_time)

    # Writes the timestamp of the value used for the update
    set_update_timestamp!(system_dataset, ts)
    # Keep coordination between fields. System state is an array of size 1
    system_dataset.timestamps[1] = ts
    time_ratio = (decision_state_resolution / system_state_resolution)
    # Don't use set_dataset_values!(state, key, 1, decision_state_value).
    # For the time variables we need to grab the values to avoid mutation of the
    # dataframe row
    set_value!(system_dataset, values(decision_state_value) .* time_ratio, 1)
    # This value shouldn't be other than one and after one execution is no-op.
    set_last_recorded_row!(system_dataset, 1)
    return
end
