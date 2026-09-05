"""
Apply the simulation's outage events for the current step.

The semantics live in PowerOperationsModels: whether an outage begins, how long it runs,
how availability and balance offsets follow from the countdown, and which parameters have
to be written in which order. This file is the runtime half — reading the previous
countdown out of `SimulationState`, resolving whatever a condition needs, and writing
values back at the state's own resolution.
"""
function apply_simulation_events!(simulation::Simulation)
    sequence = get_sequence(simulation)
    events = get_events(sequence)
    isempty(events) && return
    em_model = get_emulation_model(get_models(simulation))
    isnothing(em_model) && error(
        "Simulation has events configured but no emulation model; events require an emulator",
    )
    sys = get_system(em_model)
    for event_model in events
        # The condition gates only whether a *new* outage may begin. Every device is
        # updated every step regardless, or a running outage would freeze the moment the
        # condition stopped holding and never recover.
        may_start = _condition_holds(simulation, event_model)
        for (attribute_id, device_type_maps) in POM.get_attribute_device_map(event_model)
            apply_event_step!(
                simulation,
                event_model,
                PSY.get_supplemental_attribute(sys, attribute_id),
                device_type_maps;
                may_start = may_start,
            )
        end
    end
    return
end

"""
Resolve one input an event condition declared. POM says what it needs; this says where it
comes from.
"""
function resolve_condition_input(input::POM.StateValueInput, state::SimulationState)
    data = get_system_state_data(
        state,
        POM.get_variable_type(input),
        POM.get_device_type(input),
    )
    return data.values[POM.get_device_name(input), 1]
end

# The escape hatch: `DiscreteEventCondition`'s function is user code POM does not
# interpret, so it gets the simulation state itself.
resolve_condition_input(::POM.RuntimeStateInput, state::SimulationState) = state

function _condition_holds(simulation::Simulation, event_model::POM.EventModel)
    state = get_simulation_state(simulation)
    condition = POM.get_event_condition(event_model)
    inputs = map(
        input -> resolve_condition_input(input, state),
        POM.required_inputs(condition),
    )
    return POM.is_triggered(condition, get_current_time(simulation), inputs)
end

"""
Advance one device type's outage state by one step: write the availability this step
inherits, let POM decay the countdown and decide whether a new outage begins, then write
the offsets and the projection the next decision model reads.

`may_start` is the event condition's verdict. It gates only the *start* of an outage;
everything else here runs every step so a running outage decays and recovers.
"""
function apply_event_step!(
    simulation::Simulation,
    event_model::POM.EventModel{T, <:POM.AbstractEventCondition},
    event::T,
    device_type_maps::Dict{DataType, Set{String}};
    may_start::Bool = true,
) where {T <: PSY.Contingency}
    sim_state = get_simulation_state(simulation)
    sim_time = get_current_time(simulation)
    rng = get_rng(simulation)
    em_model = get_emulation_model(get_models(simulation))
    em_model_store = get_store_params(em_model)
    for (dtype, device_names) in device_type_maps
        POM.supports_events(dtype) || continue
        countdown_key = ParameterKey(AvailableStatusChangeCountdownParameter, dtype)
        haskey(sim_state.system_states.parameters, countdown_key) || continue

        # An outage detected at one step takes effect at the next, so availability now
        # follows the countdown as it stood at the end of the previous step -- before
        # this step decays it or samples a new outage into it.
        status_key = ParameterKey(AvailableStatusParameter, dtype)
        if haskey(sim_state.system_states.parameters, status_key)
            countdown_values =
                get_last_recorded_value(get_system_state_data(sim_state, countdown_key))
            status_data = get_system_state_data(sim_state, status_key)
            for name in device_names
                status_data.values[name, 1] =
                    POM.availability_from_countdown(countdown_values[name])
            end
        end

        # POM decays the countdown and, if `may_start`, samples a new outage into it.
        update_system_state!(
            sim_state,
            countdown_key,
            device_names,
            event,
            event_model,
            sim_time,
            rng,
            may_start,
        )
        # The rest of the event parameters follow the countdown, in POM's declared order.
        for P in POM.EVENT_PARAMETER_UPDATE_ORDER
            P in (AvailableStatusChangeCountdownParameter, AvailableStatusParameter) &&
                continue
            key = ParameterKey(P, dtype)
            haskey(sim_state.system_states.parameters, key) || continue
            update_system_state!(
                sim_state,
                key,
                device_names,
                event,
                event_model,
                sim_time,
                rng,
            )
        end
        _update_states_for_device_type!(
            update_system_state!,
            sim_state.system_states,
            sim_state,
            dtype,
            device_names,
            event,
            event_model,
            sim_time,
            rng,
        )

        # The emulator can step faster than the decision state's resolution (a 5-minute
        # emulator against an hourly unit commitment). Projecting on a step the decision
        # state has no row for would be an alignment error; the system state keeps the
        # countdown, so nothing is lost by waiting for the next aligned step.
        haskey(sim_state.decision_states.parameters, countdown_key) || continue
        sim_time in get_decision_state_data(sim_state, countdown_key).timestamps ||
            continue
        for P in POM.EVENT_PARAMETER_UPDATE_ORDER
            key = ParameterKey(P, dtype)
            haskey(sim_state.decision_states.parameters, key) || continue
            update_decision_state!(
                sim_state,
                key,
                device_names,
                event,
                event_model,
                sim_time,
                em_model_store,
            )
        end
        _update_states_for_device_type!(
            update_decision_state!,
            sim_state.decision_states,
            sim_state,
            dtype,
            device_names,
            event,
            event_model,
            sim_time,
            em_model_store,
        )
    end
    IS.@record :execution StateUpdateEvent(sim_time, "Emulator", "SystemState")
    return
end

# Variables and aux variables of the outaged device type also have to be reset; which
# ones exist is a property of the built models, so they are discovered rather than listed.
function _update_states_for_device_type!(
    update!,
    datasets,
    sim_state::SimulationState,
    dtype::DataType,
    device_names::Set{String},
    event::PSY.Outage,
    event_model::POM.EventModel,
    sim_time::Dates.DateTime,
    last_arg,
)
    for container in (datasets.variables, datasets.aux_variables)
        for key in keys(container)  # not an OrderedDict
            get_component_type(key) == dtype || continue
            update!(sim_state, key, device_names, event, event_model, sim_time, last_arg)
        end
    end
    return
end
