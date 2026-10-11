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
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    IS.@record :execution ParameterUpdateEvent(
        T,
        U,
        parameter_attributes,
        get_current_timestamp(model),
        get_name(model),
    )
    return
end

function _fix_parameter_value!(
    container::OptimizationContainer,
    parameter_array::DenseAxisArray{Float64, 2},
    parameter_attributes::VariableValueAttributes,
)
    affected_variable_keys = parameter_attributes.affected_keys
    @assert !isempty(affected_variable_keys)
    # Hoist underlying dense storage for the parameter array once. The variable
    # array's storage is hoisted per affected key (different arrays per key).
    parent_param = parameter_array.data
    component_names, time = axes(parameter_array)
    param_lookup = parameter_array.lookup[1]
    for var_key in affected_variable_keys
        variable = get_variable(container, var_key)
        parent_var = variable.data
        var_lookup = variable.lookup[1]
        for name in component_names
            i_param = param_lookup[name]
            i_var = var_lookup[name]
            for t in time
                JuMP.fix(
                    parent_var[i_var, t],
                    parent_param[i_param, t];
                    force = true,
                )
            end
        end
    end
    return
end

# Under recurrent solves without a rebuild the parameter array holds JuMP variables, and
# `_set_param_value_at!` has already fixed each one to its state value. The affected
# variables are tied to those parameters by the `FeedforwardFixValueConstraint` equality
# rows built with the model, so the fix is complete once the parameters are set.
function _fix_parameter_value!(
    ::OptimizationContainer,
    ::DenseAxisArray{JuMP.VariableRef, 2},
    ::VariableValueAttributes,
)
    return
end
