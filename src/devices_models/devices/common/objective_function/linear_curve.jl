# Add proportional terms to objective function and expression
function _add_linearcurve_variable_term_to_model!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    proportional_term_per_unit::Float64,
    time_period::Int,
) where {T <: VariableType}
    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR
    linear_cost = _add_proportional_term!(
        container,
        T(),
        component,
        proportional_term_per_unit * dt,
        time_period,
    )
    add_to_expression!(
        container,
        FuelCostExpression,
        linear_cost,
        component,
        time_period,
    )
    return
end

# Dispatch for vector of proportional terms
function _add_linearcurve_variable_cost!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    proportional_terms_per_unit::Vector{Float64},
) where {T <: VariableType}
    for t in get_time_steps(container)
        _add_linearcurve_variable_term_to_model!(
            container,
            T(),
            component,
            proportional_terms_per_unit[t],
            t,
        )
    end
    return
end

# Dispatch for scalar proportional terms
function _add_linearcurve_variable_cost!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    proportional_term_per_unit::Float64,
) where {T <: VariableType}
    for t in get_time_steps(container)
        _add_linearcurve_variable_term_to_model!(
            container,
            T(),
            component,
            proportional_term_per_unit,
            t,
        )
    end
    return
end

"""
Adds to the cost function cost terms for sum of variables with common factor to be used for cost expression for optimization_container model.

# Arguments

  - container::OptimizationContainer : the optimization_container model built in PowerSimulations
  - var_key::VariableKey: The variable name
  - component_name::String: The component_name of the variable container
  - cost_component::PSY.CostCurve{PSY.LinearCurve} : container for cost to be associated with variable
"""
function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    cost_function::PSY.CostCurve{PSY.LinearCurve},
    ::U,
) where {T <: VariableType, U <: AbstractDeviceFormulation}
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)
    value_curve = PSY.get_value_curve(cost_function)
    power_units = PSY.get_power_units(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    proportional_term = PSY.get_proportional_term(cost_component)
    proportional_term_per_unit = get_proportional_cost_per_system_unit(
        proportional_term,
        power_units,
        base_power,
        device_base_power,
    )
    multiplier = objective_function_multiplier(T(), U())
    _add_linearcurve_variable_cost!(
        container,
        T(),
        component,
        multiplier * proportional_term_per_unit,
    )
    return
end

function _add_fuel_linear_variable_cost!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    fuel_curve::Float64,
    fuel_cost::Float64,
) where {T <: VariableType}
    _add_linearcurve_variable_cost!(container, T(), component, fuel_curve * fuel_cost)
end

function _add_fuel_linear_variable_cost!(
    container::OptimizationContainer,
    ::T,
    component::V,
    ::Float64, # already normalized in MMBTU/p.u.
    fuel_cost::IS.TimeSeriesKey,
) where {T <: VariableType, V <: PSY.Component}
    _add_time_varying_fuel_variable_cost!(container, T(), component, fuel_cost)
    return
end

"""
Adds to the cost function cost terms for sum of variables with common factor to be used for cost expression for optimization_container model.

# Arguments

  - container::OptimizationContainer : the optimization_container model built in PowerSimulations
  - var_key::VariableKey: The variable name
  - component_name::String: The component_name of the variable container
  - cost_component::PSY.FuelCurve{PSY.LinearCurve} : container for cost to be associated with variable
"""
function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    cost_function::PSY.FuelCurve{PSY.LinearCurve},
    ::U,
) where {T <: VariableType, U <: AbstractDeviceFormulation}
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)
    value_curve = PSY.get_value_curve(cost_function)
    power_units = PSY.get_power_units(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    proportional_term = PSY.get_proportional_term(cost_component)
    fuel_curve_per_unit = get_proportional_cost_per_system_unit(
        proportional_term,
        power_units,
        base_power,
        device_base_power,
    )
    fuel_cost = PSY.get_fuel_cost(cost_function)
    # Multiplier is not necessary here. There is no negative cost for fuel curves.
    _add_fuel_linear_variable_cost!(
        container,
        T(),
        component,
        fuel_curve_per_unit,
        fuel_cost,
    )
    return
end

function _add_curtailment_cost!(
    container::OptimizationContainer,
    ::T,
    component::PSY.RenewableDispatch,
    cost_function::PSY.CostCurve{PSY.LinearCurve},
    ::U,
) where {T <: VariableType, U <: AbstractDeviceFormulation}
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)

    value_curve = PSY.get_value_curve(cost_function)
    power_units = PSY.get_power_units(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    proportional_term = PSY.get_proportional_term(cost_component)
    iszero(proportional_term) && return

    proportional_term_per_unit = get_proportional_cost_per_system_unit(
        proportional_term,
        power_units,
        base_power,
        device_base_power,
    )

    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR

    name = PSY.get_name(component)
    dispatch_vars = get_variable(container, T(), PSY.RenewableDispatch)

    for t in get_time_steps(container)
        breakpoints, _ = _get_pwl_data(false, container, component, t)
        offer_max = breakpoints[end]

        dispatch = dispatch_vars[name, t]
        curtailment_cost = proportional_term_per_unit * dt * (offer_max - dispatch)

        add_to_expression!(
            container,
            CurtailmentCostExpression,
            curtailment_cost,
            component,
            t,
        )
    end

    return
end

function _add_curtailment_cost!(
    container::OptimizationContainer,
    ::T,
    component::PSY.RenewableGen,
    cost_function::PSY.CostCurve{PSY.LinearCurve},
    ::U,
) where {T <: VariableType, U <: AbstractDeviceFormulation}
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)

    value_curve = PSY.get_value_curve(cost_function)
    power_units = PSY.get_power_units(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    proportional_term = PSY.get_proportional_term(cost_component)
    iszero(proportional_term) && return

    proportional_term_per_unit = get_proportional_cost_per_system_unit(
        proportional_term,
        power_units,
        base_power,
        device_base_power,
    )

    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR

    name = PSY.get_name(component)
    dispatch_vars = get_variable(container, T(), PSY.RenewableGen)

    for t in get_time_steps(container)
        breakpoints, _ = _get_pwl_data(false, container, component, t)
        offer_max = breakpoints[end]

        dispatch = dispatch_vars[name, t]
        curtailment_cost = proportional_term_per_unit * dt * (offer_max - dispatch)

        add_to_expression!(
            container,
            CurtailmentCostExpression,
            curtailment_cost,
            component,
            t,
        )
    end

    return
end
