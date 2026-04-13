##################################################
################# SOS Methods ####################
##################################################

function _get_sos_value(
    container::OptimizationContainer,
    ::Type{V},
    component::T,
) where {T <: PSY.Component, V <: AbstractDeviceFormulation}
    if has_container_key(container, OnStatusParameter, T)
        sos_val = SOSStatusVariable.PARAMETER
    else
        sos_val = sos_status(component, V())
    end
    return sos_val
end

function _get_sos_value(
    container::OptimizationContainer,
    ::Type{V},
    component::T,
) where {T <: PSY.Component, V <: AbstractServiceFormulation}
    return SOSStatusVariable.NO_VARIABLE
end

##################################################
################# PWL Variables ##################
##################################################

# This cases bounds the data by 1 - 0
function _add_pwl_variables!(
    container::OptimizationContainer,
    ::Type{T},
    component_name::String,
    time_period::Int,
    cost_data::PSY.PiecewiseLinearData,
) where {T <: PSY.Component}
    var_container = lazy_container_addition!(container, PiecewiseLinearCostVariable(), T)
    # length(PiecewiseStepData) gets number of segments, here we want number of points
    pwlvars = Array{JuMP.VariableRef}(undef, length(cost_data) + 1)
    for i in 1:(length(cost_data) + 1)
        pwlvars[i] =
            var_container[(component_name, i, time_period)] = JuMP.@variable(
                get_jump_model(container),
                base_name = "PiecewiseLinearCostVariable_$(component_name)_{pwl_$(i), $time_period}",
                lower_bound = 0.0,
                upper_bound = 1.0
            )
    end
    return pwlvars
end

##################################################
################# PWL Constraints ################
##################################################

function _determine_bin_lhs(
    container::OptimizationContainer,
    sos_status::SOSStatusVariable,
    component::T,
    period::Int) where {T <: PSY.Component}
    name = PSY.get_name(component)
    if sos_status == SOSStatusVariable.NO_VARIABLE
        return 1.0
        @debug "Using Piecewise Linear cost function but no variable/parameter ref for ON status is passed. Default status will be set to online (1.0)" _group =
            LOG_GROUP_COST_FUNCTIONS

    elseif sos_status == SOSStatusVariable.PARAMETER
        param = get_default_on_parameter(component)
        return get_parameter(container, param, T).parameter_array[name, period]
        @debug "Using Piecewise Linear cost function with parameter OnStatusParameter, $T" _group =
            LOG_GROUP_COST_FUNCTIONS
    elseif sos_status == SOSStatusVariable.VARIABLE
        var = get_default_on_variable(component)
        return get_variable(container, var, T)[name, period]
        @debug "Using Piecewise Linear cost function with variable OnVariable $T" _group =
            LOG_GROUP_COST_FUNCTIONS
    else
        @assert false
    end
end

function _get_bin_lhs(
    container::OptimizationContainer,
    sos_status::SOSStatusVariable,
    component::T,
    period::Int) where {T <: PSY.Component}
    return _determine_bin_lhs(container, sos_status, component, period)
end

function _get_bin_lhs(
    container::OptimizationContainer,
    sos_status::SOSStatusVariable,
    component::PSY.ThermalGen,
    period::Int)
    if PSY.get_must_run(component)
        return 1.0
    else
        return _determine_bin_lhs(container, sos_status, component, period)
    end
end

"""
Implement the constraints for PWL variables. That is:

```math
\\sum_{k\\in\\mathcal{K}} P_k^{max} \\delta_{k,t} = p_t \\\\
\\sum_{k\\in\\mathcal{K}} \\delta_{k,t} = on_t
```
"""
function _add_pwl_constraint!(
    container::OptimizationContainer,
    component::T,
    ::U,
    break_points::Vector{Float64},
    sos_status::SOSStatusVariable,
    period::Int,
) where {T <: PSY.Component, U <: VariableType}
    variables = get_variable(container, U(), T)
    const_container = lazy_container_addition!(
        container,
        PiecewiseLinearCostConstraint(),
        T,
        axes(variables)...,
    )
    len_cost_data = length(break_points)
    jump_model = get_jump_model(container)
    pwl_vars = get_variable(container, PiecewiseLinearCostVariable(), T)
    name = PSY.get_name(component)
    const_container[name, period] = JuMP.@constraint(
        jump_model,
        variables[name, period] ==
        sum(pwl_vars[name, ix, period] * break_points[ix] for ix in 1:len_cost_data)
    )
    bin = _get_bin_lhs(container, sos_status, component, period)
    const_normalization_container = lazy_container_addition!(
        container,
        PiecewiseLinearCostConstraint(),
        T,
        axes(variables)...;
        meta = "normalization",
    )

    const_normalization_container[name, period] = JuMP.@constraint(
        jump_model,
        sum(pwl_vars[name, i, period] for i in 1:len_cost_data) == bin
    )
    return
end

"""
Implement the constraints for PWL variables for Compact form. That is:

```math
\\sum_{k\\in\\mathcal{K}} P_k^{max} \\delta_{k,t} = p_t + P_min * u_t \\\\
\\sum_{k\\in\\mathcal{K}} \\delta_{k,t} = on_t
```
"""
function _add_pwl_constraint!(
    container::OptimizationContainer,
    component::T,
    ::U,
    break_points::Vector{Float64},
    sos_status::SOSStatusVariable,
    period::Int,
) where {T <: PSY.Component, U <: PowerAboveMinimumVariable}
    variables = get_variable(container, U(), T)
    const_container = lazy_container_addition!(
        container,
        PiecewiseLinearCostConstraint(),
        T,
        axes(variables)...,
    )
    len_cost_data = length(break_points)
    jump_model = get_jump_model(container)
    pwl_vars = get_variable(container, PiecewiseLinearCostVariable(), T)
    name = PSY.get_name(component)

    if sos_status == SOSStatusVariable.NO_VARIABLE
        bin = 1.0
        @debug "Using Piecewise Linear cost function but no variable/parameter ref for ON status is passed. Default status will be set to online (1.0)" _group =
            LOG_GROUP_COST_FUNCTIONS

    elseif sos_status == SOSStatusVariable.PARAMETER
        param = get_default_on_parameter(component)
        bin = get_parameter(container, param, T).parameter_array[name, period]
        @debug "Using Piecewise Linear cost function with parameter OnStatusParameter, $T" _group =
            LOG_GROUP_COST_FUNCTIONS
    elseif sos_status == SOSStatusVariable.VARIABLE
        var = get_default_on_variable(component)
        bin = get_variable(container, var, T)[name, period]
        @debug "Using Piecewise Linear cost function with variable OnVariable $T" _group =
            LOG_GROUP_COST_FUNCTIONS
    else
        @assert false
    end
    P_min = PSY.get_active_power_limits(component).min

    const_container[name, period] = JuMP.@constraint(
        jump_model,
        bin * P_min + variables[name, period] ==
        sum(pwl_vars[name, ix, period] * break_points[ix] for ix in 1:len_cost_data)
    )

    const_normalization_container = lazy_container_addition!(
        container,
        PiecewiseLinearCostConstraint(),
        T,
        axes(variables)...;
        meta = "normalization",
    )

    const_normalization_container[name, period] = JuMP.@constraint(
        jump_model,
        sum(pwl_vars[name, i, period] for i in 1:len_cost_data) == bin
    )
    return
end

"""
Implement the SOS for PWL variables. That is:

```math
\\{\\delta_{i,t}, ..., \\delta_{k,t}\\} \\in \\text{SOS}_2
```
"""
function _add_pwl_sos_constraint!(
    container::OptimizationContainer,
    component::T,
    ::U,
    break_points::Vector{Float64},
    sos_status::SOSStatusVariable,
    period::Int,
) where {T <: PSY.Component, U <: VariableType}
    name = PSY.get_name(component)
    @warn(
        "The cost function provided for $(name) is not compatible with a linear PWL cost function.
  An SOS-2 formulation will be added to the model. This will result in additional binary variables."
    )

    jump_model = get_jump_model(container)
    pwl_vars = get_variable(container, PiecewiseLinearCostVariable(), T)
    bp_count = length(break_points)
    pwl_vars_subset = [pwl_vars[name, i, period] for i in 1:bp_count]
    JuMP.@constraint(jump_model, pwl_vars_subset in MOI.SOS2(collect(1:bp_count)))
    return
end

##################################################
################ PWL Expressions #################
##################################################

function _get_pwl_cost_expression(
    container::OptimizationContainer,
    component::T,
    time_period::Int,
    cost_data::PSY.PiecewiseLinearData,
    multiplier::Float64,
) where {T <: PSY.Component}
    name = PSY.get_name(component)
    pwl_var_container = get_variable(container, PiecewiseLinearCostVariable(), T)
    gen_cost = JuMP.AffExpr(0.0)
    y_coords_cost_data = PSY.get_y_coords(cost_data)
    for (i, cost) in enumerate(y_coords_cost_data)
        JuMP.add_to_expression!(
            gen_cost,
            (cost * multiplier),
            pwl_var_container[(name, i, time_period)],
        )
    end
    return gen_cost
end

function _get_pwl_cost_expression(
    container::OptimizationContainer,
    component::T,
    time_period::Int,
    cost_function::PSY.CostCurve{PSY.PiecewisePointCurve},
    ::U,
    ::V,
) where {T <: PSY.Component, U <: VariableType, V <: AbstractDeviceFormulation}
    value_curve = PSY.get_value_curve(cost_function)
    power_units = PSY.get_power_units(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)
    cost_data_normalized = get_piecewise_pointcurve_per_system_unit(
        cost_component,
        power_units,
        base_power,
        device_base_power,
    )
    multiplier = objective_function_multiplier(U(), V())
    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR
    return _get_pwl_cost_expression(
        container,
        component,
        time_period,
        cost_data_normalized,
        multiplier * dt,
    )
end

function _get_pwl_cost_expression(
    container::OptimizationContainer,
    component::T,
    time_period::Int,
    cost_function::PSY.FuelCurve{PSY.PiecewisePointCurve},
    ::U,
    ::V,
) where {T <: PSY.Component, U <: VariableType, V <: AbstractDeviceFormulation}
    value_curve = PSY.get_value_curve(cost_function)
    power_units = PSY.get_power_units(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)
    cost_data_normalized = get_piecewise_pointcurve_per_system_unit(
        cost_component,
        power_units,
        base_power,
        device_base_power,
    )
    # Multiplier is not necessary here. There is no negative cost for fuel curves.
    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR
    fuel_consumption_expression = _get_pwl_cost_expression(
        container,
        component,
        time_period,
        cost_data_normalized,
        dt,
    )
    return fuel_consumption_expression
end

##################################################
######## CostCurve: PiecewisePointCurve ##########
##################################################

"""
Add PWL cost terms for data coming from a PiecewisePointCurve
"""
function _add_pwl_term!(
    container::OptimizationContainer,
    component::T,
    cost_function::Union{
        PSY.CostCurve{PSY.PiecewisePointCurve},
        PSY.FuelCurve{PSY.PiecewisePointCurve},
    },
    ::U,
    ::V,
) where {T <: PSY.Component, U <: VariableType, V <: AbstractDeviceFormulation}
    # multiplier = objective_function_multiplier(U(), V())
    name = PSY.get_name(component)
    value_curve = PSY.get_value_curve(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)
    power_units = PSY.get_power_units(cost_function)

    # Normalize data
    data = get_piecewise_pointcurve_per_system_unit(
        cost_component,
        power_units,
        base_power,
        device_base_power,
    )

    if all(iszero.((point -> point.y).(PSY.get_points(data))))  # TODO I think this should have been first. before?
        @debug "All cost terms for component $(name) are 0.0" _group =
            LOG_GROUP_COST_FUNCTIONS
        return
    end

    # Compact PWL data does not exists anymore

    cost_is_convex = PSY.is_convex(data)
    break_points = PSY.get_x_coords(data)
    time_steps = get_time_steps(container)
    pwl_cost_expressions = Vector{JuMP.AffExpr}(undef, time_steps[end])
    sos_val = _get_sos_value(container, V, component)
    for t in time_steps
        _add_pwl_variables!(container, T, name, t, data)
        _add_pwl_constraint!(container, component, U(), break_points, sos_val, t)
        if !cost_is_convex
            _add_pwl_sos_constraint!(container, component, U(), break_points, sos_val, t)
        end
        pwl_cost =
            _get_pwl_cost_expression(container, component, t, cost_function, U(), V())
        pwl_cost_expressions[t] = pwl_cost
    end
    return pwl_cost_expressions
end

"""
Add PWL cost terms for data coming from a PiecewisePointCurve for ThermalDispatchNoMin formulation
"""
function _add_pwl_term!(
    container::OptimizationContainer,
    component::T,
    cost_function::Union{
        PSY.CostCurve{PSY.PiecewisePointCurve},
        PSY.FuelCurve{PSY.PiecewisePointCurve},
    },
    ::U,
    ::V,
) where {T <: PSY.ThermalGen, U <: VariableType, V <: ThermalDispatchNoMin}
    name = PSY.get_name(component)
    value_curve = PSY.get_value_curve(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)
    power_units = PSY.get_power_units(cost_function)

    # Normalize data
    data = get_piecewise_pointcurve_per_system_unit(
        cost_component,
        power_units,
        base_power,
        device_base_power,
    )
    @debug "PWL cost function detected for device $(name) using $V"
    slopes = PSY.get_slopes(data)
    if any(slopes .< 0) || !PSY.is_convex(data)
        throw(
            IS.InvalidValue(
                "The PWL cost data provided for generator $(name) is not compatible with $U.",
            ),
        )
    end

    # Compact PWL data does not exists anymore
    x_coords = PSY.get_x_coords(data)
    if x_coords[1] != 0.0
        y_coords = PSY.get_y_coords(data)
        x_first = round(x_coords[1]; digits = 3)
        y_first = round(y_coords[1]; digits = 3)
        slope_first = round(slopes[1]; digits = 3)
        guess_y_zero = y_coords[1] - slopes[1] * x_coords[1]
        @warn(
            "PWL has no 0.0 intercept for generator $(name). First point is given at (x = $(x_first), y = $(y_first)). Adding a first intercept at (x = 0.0, y = $(round(guess_y_zero, digits = 3)) to have equal initial slope $(slope_first)"
        )
        if guess_y_zero < 0.0
            error(
                "Added zero intercept has negative cost for generator $(name). Consider using other formulation or improve data.",
            )
        end
        # adds a first intercept a x = 0.0 and y above the intercept of the first tuple to make convex equivalent (avoid floating point issues of almost equal slopes)
        intercept_point = (x = 0.0, y = guess_y_zero + COST_EPSILON)
        data = PSY.PiecewiseLinearData(vcat(intercept_point, PSY.get_points(data)))
        @assert PSY.is_convex(data)
    end

    time_steps = get_time_steps(container)
    pwl_cost_expressions = Vector{JuMP.AffExpr}(undef, time_steps[end])
    break_points = PSY.get_x_coords(data)
    sos_val = _get_sos_value(container, V, component)
    temp_cost_function =
        create_temporary_cost_function_in_system_per_unit(cost_function, data)
    for t in time_steps
        _add_pwl_variables!(container, T, name, t, data)
        _add_pwl_constraint!(container, component, U(), break_points, sos_val, t)
        pwl_cost =
            _get_pwl_cost_expression(container, component, t, temp_cost_function, U(), V())
        pwl_cost_expressions[t] = pwl_cost
    end
    return pwl_cost_expressions
end

"""
Creates piecewise linear cost function using a sum of variables and expression with sign and time step included.

# Arguments

  - container::OptimizationContainer : the optimization_container model built in PowerSimulations
  - var_key::VariableKey: The variable name
  - component_name::String: The component_name of the variable container
  - cost_function::PSY.CostCurve{PSY.PiecewisePointCurve}: container for piecewise linear cost
"""
function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    cost_function::PSY.CostCurve{PSY.PiecewisePointCurve},
    ::U,
) where {T <: VariableType, U <: AbstractDeviceFormulation}
    component_name = PSY.get_name(component)
    @debug "PWL Variable Cost" _group = LOG_GROUP_COST_FUNCTIONS component_name
    # If array is full of tuples with zeros return 0.0
    value_curve = PSY.get_value_curve(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    if all(iszero.((point -> point.y).(PSY.get_points(cost_component))))  # TODO I think this should have been first. before?
        @debug "All cost terms for component $(component_name) are 0.0" _group =
            LOG_GROUP_COST_FUNCTIONS
        return
    end
    pwl_cost_expressions =
        _add_pwl_term!(container, component, cost_function, T(), U())
    for t in get_time_steps(container)
        add_to_expression!(
            container,
            FuelCostExpression,
            pwl_cost_expressions[t],
            component,
            t,
        )
        add_to_objective_invariant_expression!(container, pwl_cost_expressions[t])
    end
    return
end

"""
Creates piecewise linear cost function using a sum of variables and expression with sign and time step included.
# Arguments
  - container::OptimizationContainer : the optimization_container model built in PowerSimulations
  - var_key::VariableKey: The variable name
  - component_name::String: The component_name of the variable container
  - cost_function::PSY.CostCurve{PSY.PiecewisePointCurve}: container for piecewise linear cost
"""
function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    cost_function::PSY.FuelCurve{PSY.PiecewisePointCurve},
    ::U,
) where {T <: VariableType, U <: AbstractDeviceFormulation}
    component_name = PSY.get_name(component)
    @debug "PWL Variable Cost" _group = LOG_GROUP_COST_FUNCTIONS component_name
    # If array is full of tuples with zeros return 0.0
    value_curve = PSY.get_value_curve(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    if all(iszero.((point -> point.y).(PSY.get_points(cost_component))))  # TODO I think this should have been first. before?
        @debug "All cost terms for component $(component_name) are 0.0" _group =
            LOG_GROUP_COST_FUNCTIONS
        return
    end
    pwl_fuel_consumption_expressions =
        _add_pwl_term!(container, component, cost_function, T(), U())

    is_time_variant_ = is_time_variant(PSY.get_fuel_cost(cost_function))
    for t in get_time_steps(container)
        fuel_cost_value = get_fuel_cost_value(
            container,
            component,
            t,
            is_time_variant_,
        )
        pwl_cost_expression = pwl_fuel_consumption_expressions[t] * fuel_cost_value
        add_to_expression!(
            container,
            FuelCostExpression,
            pwl_cost_expression,
            component,
            t,
        )
        add_to_expression!(
            container,
            FuelConsumptionExpression,
            pwl_fuel_consumption_expressions[t],
            component,
            t,
        )
        if is_time_variant_
            add_to_objective_variant_expression!(container, pwl_cost_expression)
        else
            add_to_objective_invariant_expression!(container, pwl_cost_expression)
        end
    end
    return
end

##################################################
###### CostCurve: PiecewiseIncrementalCurve ######
######### and PiecewiseAverageCurve ##############
##################################################

"""
Creates piecewise linear cost function using a sum of variables and expression with sign and time step included.

# Arguments

  - container::OptimizationContainer : the optimization_container model built in PowerSimulations
  - var_key::VariableKey: The variable name
  - component_name::String: The component_name of the variable container
  - cost_function::PSY.Union{PSY.CostCurve{PSY.PiecewiseIncrementalCurve}, PSY.CostCurve{PSY.PiecewiseAverageCurve}}: container for piecewise linear cost
"""
function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    cost_function::V,
    ::U,
) where {
    T <: VariableType,
    V <: Union{
        PSY.CostCurve{PSY.PiecewiseIncrementalCurve},
        PSY.CostCurve{PSY.PiecewiseAverageCurve},
    },
    U <: AbstractDeviceFormulation,
}
    # Create new PiecewisePointCurve
    value_curve = PSY.get_value_curve(cost_function)
    power_units = PSY.get_power_units(cost_function)
    pointbased_value_curve = PSY.InputOutputCurve(value_curve)
    pointbased_cost_function =
        PSY.CostCurve(; value_curve = pointbased_value_curve, power_units = power_units)
    # Call method for PiecewisePointCurve
    _add_variable_cost_to_objective!(
        container,
        T(),
        component,
        pointbased_cost_function,
        U(),
    )
    return
end

##################################################
###### FuelCurve: PiecewiseIncrementalCurve ######
######### and PiecewiseAverageCurve ##############
##################################################

"""
Creates piecewise linear fuel cost function using a sum of variables and expression with sign and time step included.

# Arguments

  - container::OptimizationContainer : the optimization_container model built in PowerSimulations
  - var_key::VariableKey: The variable name
  - component_name::String: The component_name of the variable container
  - cost_function::PSY.Union{PSY.FuelCurve{PSY.PiecewiseIncrementalCurve}, PSY.FuelCurve{PSY.PiecewiseAverageCurve}}: container for piecewise linear cost
"""
function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    cost_function::V,
    ::U,
) where {
    T <: VariableType,
    V <: Union{
        PSY.FuelCurve{PSY.PiecewiseIncrementalCurve},
        PSY.FuelCurve{PSY.PiecewiseAverageCurve},
    },
    U <: AbstractDeviceFormulation,
}
    # Create new PiecewisePointCurve
    value_curve = PSY.get_value_curve(cost_function)
    power_units = PSY.get_power_units(cost_function)
    fuel_cost = PSY.get_fuel_cost(cost_function)
    pointbased_value_curve = PSY.InputOutputCurve(value_curve)
    pointbased_cost_function =
        PSY.FuelCurve(;
            value_curve = pointbased_value_curve,
            power_units = power_units,
            fuel_cost = fuel_cost,
        )
    # Call method for PiecewisePointCurve
    _add_variable_cost_to_objective!(
        container,
        T(),
        component,
        pointbased_cost_function,
        U(),
    )
    return
end
