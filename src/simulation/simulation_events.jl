function apply_simulation_events!(simulation::Simulation)
    sequence = get_sequence(simulation)
    events = get_events(sequence)
    isempty(events) && return
    em_model = get_emulation_model(get_models(simulation))
    isnothing(em_model) && error(
        "Simulation has events configured but no emulation model; events require an emulator",
    )
    simulation_state = get_simulation_state(simulation)
    for event_model in events
        extend_event_parameters!(simulation, event_model)
        if check_condition(simulation_state, event_model)
            # TODO: Events for other event categories we need to do something else
            sys = get_system(em_model)
            model_name = get_name(em_model)
            for (event_uuid, device_type_maps) in
                event_model.attribute_device_map[model_name]
                event = PSY.get_supplemental_attribute(sys, event_uuid)
                apply_affect!(simulation, event_model, event, device_type_maps)
            end
        end
    end
end

# Device types that the event-extension state-projection logic supports today.
# RenewableDispatch is intentionally excluded: its outage affect is applied via
# `apply_affect!` but the per-period countdown projection here is not yet implemented.
_supports_event_extension(::Type{<:PSY.Component}) = true
_supports_event_extension(::Type{PSY.RenewableDispatch}) = false

function extend_event_parameters!(simulation::Simulation, event_model)
    sequence = get_sequence(simulation)
    sim_state = get_simulation_state(simulation)
    em_model = get_emulation_model(get_models(simulation))
    isnothing(em_model) && return
    model_name = get_name(em_model)
    for (event_uuid, device_type_maps) in
        event_model.attribute_device_map[model_name]
        sim_time = get_current_time(simulation)
        for (dtype, device_names) in device_type_maps
            _supports_event_extension(dtype) || continue
            em_model = get_emulation_model(get_models(simulation))
            status_change_countdown_data = get_decision_state_data(
                sim_state,
                ParameterKey(AvailableStatusChangeCountdownParameter, dtype),
            )
            status_data = get_decision_state_data(
                sim_state,
                ParameterKey(AvailableStatusParameter, dtype),
            )
            state_timestamps = status_data.timestamps
            state_data_index = find_timestamp_index(state_timestamps, sim_time)
            if state_data_index == 1
                for name in device_names
                    if status_change_countdown_data.values[name, 1] > 1.0
                        starting_count = status_change_countdown_data.values[name, 1]
                        for i in 1:length(status_change_countdown_data.values[name, :])
                            countdown_val = max(starting_count + 1 - i, 0.0)
                            if countdown_val == 0.0
                                status_val = 1.0
                            else
                                status_val = 0.0
                            end
                            status_change_countdown_data.values[name, i] = countdown_val
                            status_data.values[name, i] = status_val
                        end
                    end
                end
            end
        end
    end
end

function check_condition(
    ::SimulationState,
    ::EventModel{<:PSY.Contingency, ContinuousCondition},
)
    return true
end

function check_condition(
    simulation_state::SimulationState,
    event_model::EventModel{<:PSY.Contingency, PresetTimeCondition},
)
    condition = get_event_condition(event_model)
    event_times = get_time_stamps(condition)
    current_time = get_current_time(simulation_state)
    if current_time in event_times
        return true
    else
        return false
    end
end

function check_condition(
    simulation_state::SimulationState,
    event_model::EventModel{<:PSY.Contingency, StateVariableValueCondition},
)
    condition = get_event_condition(event_model)
    variable_type = get_variable_type(condition)
    device_type = get_device_type(condition)
    device_name = get_device_name(condition)
    event_value = get_value(condition)

    system_value =
        get_system_state_data(simulation_state, variable_type, device_type).values[
            device_name,
            1,
        ]
    if isapprox(system_value, event_value; atol = ABSOLUTE_TOLERANCE)
        return true
    else
        return false
    end
end

function check_condition(
    simulation_state::SimulationState,
    event_model::EventModel{<:PSY.Contingency, DiscreteEventCondition},
)
    condition = get_event_condition(event_model)
    f = condition.condition_function
    if f(simulation_state)
        return true
    else
        return false
    end
end

function apply_affect!(
    simulation::Simulation,
    event_model::EventModel{
        T,
        <:AbstractEventCondition,
    },
    event::T,
    device_type_maps::Dict{DataType, Set{String}},
) where {T <: PSY.Contingency}
    sim_state = get_simulation_state(simulation)
    sim_time = get_current_time(simulation)
    rng = get_rng(simulation)
    for (dtype, device_names) in device_type_maps
        if !supports_outages(dtype)
            continue
        end
        em_model = get_emulation_model(get_models(simulation))
        em_model_store = get_store_params(em_model)
        # Order required: event parameters must be updated first to indicate a change in other parameters/variables.
        update_system_state!(
            sim_state,
            ParameterKey(AvailableStatusChangeCountdownParameter, dtype),
            device_names,
            event,
            event_model,
            sim_time,
            rng,
        )
        if haskey(
            sim_state.system_states.parameters,
            ParameterKey(ActivePowerOffsetParameter, dtype),
        )
            update_system_state!(
                sim_state,
                ParameterKey(ActivePowerOffsetParameter, dtype),
                device_names,
                event,
                event_model,
                sim_time,
                rng,
            )
        end
        if haskey(
            sim_state.system_states.parameters,
            ParameterKey(ReactivePowerOffsetParameter, dtype),
        )
            update_system_state!(
                sim_state,
                ParameterKey(ReactivePowerOffsetParameter, dtype),
                device_names,
                event,
                event_model,
                sim_time,
                rng,
            )
        end
        update_system_state!(
            sim_state,
            ParameterKey(AvailableStatusParameter, dtype),
            device_names,
            event,
            event_model,
            sim_time,
            rng,
        )
        for k in keys(sim_state.system_states.variables)  #Not an OrderedDict
            if typeof(k).parameters[2] != dtype
                continue
            end
            update_system_state!(
                sim_state,
                k,
                device_names,
                event,
                event_model,
                sim_time,
                rng,
            )
        end
        for k in keys(sim_state.system_states.aux_variables)  #Not an OrderedDict
            if typeof(k).parameters[2] != dtype
                continue
            end
            update_system_state!(
                sim_state,
                k,
                device_names,
                event,
                event_model,
                sim_time,
                rng,
            )
        end

        # Order is required here too AvailableStatusChangeCountdownParameter needs to
        # go first to indicate that there is a change in the other values
        update_decision_state!(
            sim_state,
            ParameterKey(AvailableStatusChangeCountdownParameter, dtype),
            device_names,
            event,
            event_model,
            sim_time,
            em_model_store,
        )
        if haskey(
            sim_state.decision_states.parameters,
            ParameterKey(ActivePowerOffsetParameter, dtype),
        )
            update_decision_state!(
                sim_state,
                ParameterKey(ActivePowerOffsetParameter, dtype),
                device_names,
                event,
                event_model,
                sim_time,
                em_model_store,
            )
        end
        if haskey(
            sim_state.decision_states.parameters,
            ParameterKey(ReactivePowerOffsetParameter, dtype),
        )
            update_decision_state!(
                sim_state,
                ParameterKey(ReactivePowerOffsetParameter, dtype),
                device_names,
                event,
                event_model,
                sim_time,
                em_model_store,
            )
        end
        update_decision_state!(
            sim_state,
            ParameterKey(AvailableStatusParameter, dtype),
            device_names,
            event,
            event_model,
            sim_time,
            em_model_store,
        )

        for k in keys(sim_state.decision_states.variables)  #Not an OrderedDict
            if typeof(k).parameters[2] != dtype
                continue
            end
            update_decision_state!(
                sim_state,
                k,
                device_names,
                event,
                event_model,
                sim_time,
                em_model_store,
            )
        end
        for k in keys(sim_state.decision_states.aux_variables)  #Not an OrderedDict
            if typeof(k).parameters[2] != dtype
                continue
            end
            update_decision_state!(
                sim_state,
                k,
                device_names,
                event,
                event_model,
                sim_time,
                em_model_store,
            )
        end
    end
    IS.@record :execution StateUpdateEvent(sim_time, "Emulator", "SystemState")
end
