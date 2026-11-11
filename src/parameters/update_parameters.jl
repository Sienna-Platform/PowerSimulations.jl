"""
Update parameter function an IOM.AbstractOptimizationModel
"""
function update_parameter_values!(
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
    simulation_state::SimulationState,
) where {T <: ParameterType, U <: PSY.Component}
    optimization_container = get_optimization_container(model)
    input = get_decision_states(simulation_state)
    update_container_parameter_values!(optimization_container, model, key, input)
    _record_parameter_update(model, key)
    return
end

function _record_parameter_update(
    model::IOM.AbstractOptimizationModel,
    key::ParameterKey{T, U},
) where {T <: ParameterType, U <: PSY.Component}
    parameter_attributes = get_parameter_attributes(get_optimization_container(model), key)
    IS.@record :execution ParameterUpdateEvent(
        T,
        U,
        parameter_attributes,
        get_current_timestamp(model),
        get_name(model),
    )
    return
end
