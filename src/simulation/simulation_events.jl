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
        event_type = POM.get_event_type(event_model)
        for (attribute_id, device_type_maps) in POM.get_attribute_device_map(event_model)
            attribute = PSY.get_supplemental_attribute(sys, attribute_id)
            attribute isa event_type || error(
                "Supplemental attribute $attribute_id of the emulator's system is a \
                 $(typeof(attribute)), but the $event_type event model was filled with \
                 it. Event models must be filled from the emulator's system.",
            )
            apply_event_step!(
                simulation,
                event_model,
                attribute,
                device_type_maps;
                may_start = may_start,
            )
        end
    end
    return
end

"""
Fill every event model of the simulation from the emulator's system, before any model is
built. Events take their outages from the emulator's system only; each model then applies
the event to the outaged devices it has. Outage attributes in a decision model's own system
do not affect events; they may serve other formulations, such as security-constrained ones.

Throws `IS.ConflictingInputsError` when the simulation has event models but no emulation
model.
"""
function _fill_event_models!(simulation::Simulation)
    events = get_events(get_sequence(simulation))
    isempty(events) && return
    models = get_models(simulation)
    em_model = get_emulation_model(models)
    if isnothing(em_model)
        throw(
            IS.ConflictingInputsError(
                "The simulation sequence has event models but the simulation has no \
                 emulation model. Outage events are drawn and applied by the emulation \
                 model from its system: add an emulation model or remove the events from \
                 the SimulationSequence.",
            ),
        )
    end
    em_sys = get_system(em_model)
    for event_model in events
        outages = POM.discover_event_attribute_device_map(event_model, em_sys)
        copy!(POM.get_attribute_device_map(event_model), outages)
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
        has_dataset(get_system_states(sim_state), countdown_key) || continue

        # An outage detected at one step takes effect at the next, so availability now
        # follows the countdown as it stood at the end of the previous step -- before
        # this step decays it or samples a new outage into it.
        status_key = ParameterKey(AvailableStatusParameter, dtype)
        if has_dataset(get_system_states(sim_state), status_key)
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
            has_dataset(get_system_states(sim_state), key) || continue
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
        has_dataset(get_decision_states(sim_state), countdown_key) || continue
        sim_time in get_decision_state_data(sim_state, countdown_key).timestamps ||
            continue
        for P in POM.EVENT_PARAMETER_UPDATE_ORDER
            key = ParameterKey(P, dtype)
            has_dataset(get_decision_states(sim_state), key) || continue
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
    event::PSY.UnplannedOutage,
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
