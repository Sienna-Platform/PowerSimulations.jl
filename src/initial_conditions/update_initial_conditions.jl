function _update_initial_conditions!(
    model::IOM.AbstractOptimizationModel,
    key::InitialConditionKey{T, U},
    source, # Store or State are used in simulations by default
) where {T <: InitialConditionType, U <: PSY.Component}
    if get_execution_count(model) < 1
        return
    end
    container = get_optimization_container(model)
    model_resolution = get_resolution(get_store_params(model))
    ini_conditions_vector = get_initial_condition(container, key)
    timestamp = get_current_timestamp(model)
    previous_values = get_condition.(ini_conditions_vector)
    # The implementation of specific update_initial_conditions! is located in the files
    # update_initial_conditions_in_memory_store.jl and update_initial_conditions_simulation.jl
    IOM.update_initial_conditions!(ini_conditions_vector, source, model_resolution)
    for (i, initial_condition) in enumerate(ini_conditions_vector)
        IS.@record :execution InitialConditionUpdateEvent(
            timestamp,
            initial_condition,
            previous_values[i],
            get_name(model),
        )
    end
    return
end

# Note to devs: Implemented this way to avoid ambiguities and future proof custom ic updating
function IOM.update_initial_conditions!(
    model::DecisionModel,
    key::InitialConditionKey{T, U},
    source, # Store or State are used in simulations by default
) where {T <: InitialConditionType, U <: PSY.Component}
    _update_initial_conditions!(model, key, source)
    return
end

function IOM.update_initial_conditions!(
    model::EmulationModel,
    key::InitialConditionKey{T, U},
    source, # Store or State are used in simulations by default
) where {T <: InitialConditionType, U <: PSY.Component}
    _update_initial_conditions!(model, key, source)
    return
end
