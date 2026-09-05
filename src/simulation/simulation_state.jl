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
    return get_data_resolution(first(values(system_state.variables)))
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

function _initialize_model_states!(
    sim_state::SimulationState,
    model::IOM.AbstractOptimizationModel,
    simulation_initial_time::Dates.DateTime,
    simulation_step::Dates.Millisecond,
    params::OrderedDict{OptimizationContainerKey, STATE_TIME_PARAMS},
)
    states = get_decision_states(sim_state)
    container = get_optimization_container(model)
    for field in fieldnames(DatasetContainer)
        field_containers = getfield(container, field)
        field_states = getfield(states, field)
        for (key, field_container) in field_containers
            !should_write_resulting_value(key) && continue
            value_counts = params[key].horizon ÷ params[key].resolution
            column_names = get_column_names(container, field, field_container, key)
            # TODO DT: why would we overwrite a key? Is this a bug?
            if !haskey(field_states, key) || get_num_rows(field_states[key]) < value_counts
                field_states[key] = InMemoryDataset(
                    NaN,
                    simulation_initial_time,
                    params[key].resolution,
                    Int(simulation_step / params[key].resolution),
                    value_counts,
                    column_names)
            end
        end
    end
    return
end

function _initialize_system_states!(
    sim_state::SimulationState,
    ::Nothing,
    simulation_initial_time::Dates.DateTime,
    params::OrderedDict{OptimizationContainerKey, STATE_TIME_PARAMS},
)
    decision_states = get_decision_states(sim_state)
    emulator_states = get_system_states(sim_state)
    min_res = minimum([v.resolution for v in values(params)])
    for key in get_dataset_keys(decision_states)
        cols = get_column_names(key, get_dataset(decision_states, key))
        set_dataset!(
            emulator_states,
            key,
            make_system_state(
                simulation_initial_time,
                min_res,
                cols,
            ),
        )
    end
    return
end

function _initialize_system_states!(
    sim_state::SimulationState,
    emulation_model::EmulationModel,
    simulation_initial_time::Dates.DateTime,
    params::OrderedDict{OptimizationContainerKey, STATE_TIME_PARAMS},
)
    decision_states = get_decision_states(sim_state)
    emulator_states = get_system_states(sim_state)
    emulation_container = get_optimization_container(emulation_model)
    min_res = minimum([v.resolution for v in values(params)])

    for field in fieldnames(DatasetContainer)
        field_containers = getfield(emulation_container, field)
        for (key, value) in field_containers
            !should_write_resulting_value(key) && continue
            if field == :parameters
                column_names = get_column_names(key, value)
            else
                column_names = get_column_names_from_axis_array(key, value)
            end
            set_dataset!(
                emulator_states,
                key,
                make_system_state(
                    simulation_initial_time,
                    min_res,
                    column_names,
                ),
            )
        end
    end

    for key in get_dataset_keys(decision_states)
        dm_cols = get_column_names(key, get_dataset(decision_states, key))
        if has_dataset(emulator_states, key)
            em_cols = get_column_names(key, get_dataset(emulator_states, key))
            if length(dm_cols) != length(em_cols)
                error(
                    "The number of dimensions between the decision states and emulator states don't match",
                )
            end
            if !isempty(symdiff(first(dm_cols), first(em_cols)))
                error(
                    "Mismatch in column names for dataset $key: $(symdiff(dm_cols, em_cols)) \
                    This issue is common when filters are applied to the decision model but not to the emulator model.",
                )
            end
            continue
        end

        set_dataset!(
            emulator_states,
            key,
            make_system_state(
                simulation_initial_time,
                min_res,
                dm_cols,
            ),
        )
    end
    return
end

function initialize_simulation_state!(
    sim_state::SimulationState,
    models::SimulationModels,
    simulation_step::Dates.Millisecond,
    simulation_initial_time::Dates.DateTime,
)
    params = _get_state_params(models, simulation_step)
    for model in get_decision_models(models)
        _initialize_model_states!(
            sim_state,
            model,
            simulation_initial_time,
            simulation_step,
            params,
        )
    end
    set_last_decision_model!(sim_state, get_name(last(get_decision_models(models))))
    em = get_emulation_model(models)
    _initialize_system_states!(sim_state, em, simulation_initial_time, params)
    return
end

function update_decision_state!(
    state::SimulationState,
    key::ParameterKey{AvailableStatusChangeCountdownParameter, T},
    store_data::DenseAxisArray{Float64, 2},
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
) where {T <: PSY.Component}
    state_data = get_decision_state_data(state, key)
    column_names = get_column_names(key, state_data)[1]
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
    column_names = get_column_names(key, state_data)[1]
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
    state_timestamps = state_data.timestamps
    state_data_index = find_timestamp_index(state_timestamps, simulation_time)
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
function update_decision_state!(
    state::SimulationState,
    key::OptimizationContainerKey,
    store_data::DenseAxisArray{Float64, 2},
    simulation_time::Dates.DateTime,
    model_params::ModelStoreParams,
)
    state_data = get_decision_state_data(state, key)
    column_names = get_column_names(key, state_data)[1]
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
    state_data_index = find_timestamp_index(state_timestamps, simulation_time)
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
    state_timestamps = state_data.timestamps
    state_data_index = find_timestamp_index(state_timestamps, simulation_time)
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

    state_timestamps = state_data.timestamps
    state_data_index = find_timestamp_index(state_timestamps, simulation_time)
    event_occurence_index =
        find_timestamp_index(event_occurrence_data.timestamps, simulation_time)
    for name in column_names
        if event_occurrence_data.values[name, event_occurence_index] > 0.0
            state_data.values[name, (state_data_index + 1):end] .=
                IOM.MISSING_INITIAL_CONDITIONS_TIME_COUNT
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

    state_timestamps = state_data.timestamps
    state_data_index = find_timestamp_index(state_timestamps, simulation_time)
    event_occurence_index =
        find_timestamp_index(event_occurrence_data.timestamps, simulation_time)
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

    state_timestamps = state_data.timestamps
    state_data_index = find_timestamp_index(state_timestamps, simulation_time)
    event_occurence_index =
        find_timestamp_index(event_occurrence_data.timestamps, simulation_time)
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
    outages = get_column_names(key, state_data)[1]

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
    set_update_timestamp!(state_data, simulation_time)
    for t in result_time_index
        state_range = state_data_index:(state_data_index + offset)
        for name in axes(store_data)[2], i in state_range
            #loop pelo -outages, names t
            for outage in outages
                # TODO: We could also interpolate here
                state_data.values[outage, name, i] = store_data[outage, name, t]
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

    if resolution_ratio == 1.0
        increment_per_period = 1.0
    elseif state_resolution < Dates.Day(365) && state_resolution > Dates.Minute(1)
        increment_per_period = Dates.value(Dates.Minute(state_resolution))
    else
        error("Incorrect Problem Resolution specification: $(state_resolution)")
    end

    column_names = axes(state_data.values)[1]
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
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    return get_decision_state_data(state, VariableKey(T, U))
end

function get_decision_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: AuxVariableType, U <: Union{PSY.Component, PSY.System}}
    return get_decision_state_data(state, AuxVarKey(T, U))
end

function get_decision_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    return get_decision_state_data(state, ConstraintKey(T, U))
end

function get_decision_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_decision_state_data(state, ParameterKey(T, U))
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

function get_system_state_value(state::SimulationState, key::OptimizationContainerKey)
    return get_dataset_values(get_system_states(state), key)[:, 1]
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
    if typeof(store) == HdfSimulationStore
        set_dataset_values!(state, key, 1, res)
    else
        # Handle different dimensionality of results
        num_dims = ndims(res)
        if num_dims == 2
            set_dataset_values!(state, key, 1, res[:, ix])
        elseif num_dims == 3
            set_dataset_values!(state, key, 1, res[:, :, ix])
        else
            error("Unsupported number of dimensions for emulation result: $num_dims")
        end
    end
    set_last_recorded_row!(dataset, 1)
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
            current_status_data.values[name, 1] = IOM.MISSING_INITIAL_CONDITIONS_TIME_COUNT
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
    # Gets the timestamp of the value used for the update, which might not match exactly the
    # simulation time since the value might have not been updated yet
    ts = get_value_timestamp(decision_dataset, simulation_time)
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
    data_set_value = get_dataset_value(decision_dataset, simulation_time)
    set_dataset_values!(state, key, 1, data_set_value)
    # This value shouldn't be other than one and after one execution is no-op.
    set_last_recorded_row!(system_dataset, 1)
    return
end

function update_system_state!(
    state::DatasetContainer{InMemoryDataset},
    key::AuxVarKey{T, PSY.ThermalStandard},
    decision_state::DatasetContainer{InMemoryDataset},
    simulation_time::Dates.DateTime,
) where {T <: Union{TimeDurationOn, TimeDurationOff}}
    decision_dataset = get_dataset(decision_state, key)
    # Gets the timestamp of the value used for the update, which might not match exactly the
    # simulation time since the value might have not been updated yet

    ts = get_value_timestamp(decision_dataset, simulation_time)
    system_dataset = get_dataset(state, key)
    system_state_resolution = get_data_resolution(system_dataset)
    decision_state_resolution = get_data_resolution(decision_dataset)

    decision_state_value = get_dataset_value(decision_dataset, simulation_time)

    if ts == get_update_timestamp(system_dataset)
        return
    end
    # TODO: past-timestamp protection removed; see note in the dataset variant above.

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

function get_system_state_value(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    return get_system_state_value(state, VariableKey(T, U))
end

function get_system_state_value(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: AuxVariableType, U <: Union{PSY.Component, PSY.System}}
    return get_system_state_value(state, AuxVarKey(T, U))
end

function get_system_state_value(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    return get_system_state_value(state, ConstraintKey(T, U))
end

function get_system_state_value(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_system_state_value(state, ParameterKey(T, U))
end

function get_system_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    return get_system_state_data(state, VariableKey(T, U))
end

function get_system_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: AuxVariableType, U <: Union{PSY.Component, PSY.System}}
    return get_system_state_data(state, AuxVarKey(T, U))
end

function get_system_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    return get_system_state_data(state, ConstraintKey(T, U))
end

function get_system_state_data(
    state::SimulationState,
    ::T,
    ::Type{U},
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_system_state_data(state, ParameterKey(T, U))
end
