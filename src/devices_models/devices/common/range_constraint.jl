######## CONSTRAINTS ############

_add_lb(::RangeConstraintLBExpressions) = true
_add_ub(::RangeConstraintLBExpressions) = false

_add_lb(::RangeConstraintUBExpressions) = false
_add_ub(::RangeConstraintUBExpressions) = true

_add_lb(::ExpressionType) = true
_add_ub(::ExpressionType) = false

# Generic fallback functions
function get_startup_shutdown(
    device,
    ::Type{<:VariableType},
    ::Type{<:AbstractDeviceFormulation},
) #  -> Union{Nothing, NamedTuple{(:startup, :shutdown), Tuple{Float64, Float64}}}
    nothing
end

@doc raw"""
Constructs min/max range constraint from device variable.


If min and max within an epsilon width:

``` variable[name, t] == limits.max ```

Otherwise:

``` limits.min <= variable[name, t] <= limits.max ```

where limits in constraint_infos.

# LaTeX

`` x = limits^{max}, \text{ for } |limits^{max} - limits^{min}| < \varepsilon ``

`` limits^{min} \leq x \leq limits^{max}, \text{ otherwise } ``
"""
function add_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: VariableType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_variable(container, U(), V)
    _add_lower_bound_range_constraints_impl!(container, T, array, devices, model)
    _add_upper_bound_range_constraints_impl!(container, T, array, devices, model)
    return
end

function add_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: RangeConstraintLBExpressions,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_expression(container, U(), V)
    _add_lower_bound_range_constraints_impl!(container, T, array, devices, model)
    return
end

function add_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: RangeConstraintUBExpressions,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_expression(container, U(), V)
    _add_upper_bound_range_constraints_impl!(container, T, array, devices, model)
    return
end

function _add_lower_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {T <: ConstraintType, V <: PSY.Component, W <: AbstractDeviceFormulation}
    time_steps = get_time_steps(container)
    device_names = PSY.get_name.(devices)

    con_lb =
        add_constraints_container!(container, T(), V, device_names, time_steps; meta = "lb")

    for device in devices, t in time_steps
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W) # depends on constraint type and formulation type
        con_lb[ci_name, t] =
            JuMP.@constraint(get_jump_model(container), array[ci_name, t] >= limits.min)
    end
    return
end

function _add_upper_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {T <: ConstraintType, V <: PSY.Component, W <: AbstractDeviceFormulation}
    time_steps = get_time_steps(container)
    device_names = PSY.get_name.(devices)

    con_ub =
        add_constraints_container!(container, T(), V, device_names, time_steps; meta = "ub")

    for device in devices, t in time_steps
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W) # depends on constraint type and formulation type
        con_ub[ci_name, t] =
            JuMP.@constraint(get_jump_model(container), array[ci_name, t] <= limits.max)
    end
    return
end

@doc raw"""
Constructs min/max range constraint from device variable and on/off decision variable.


If device min = 0:

``` varcts[name, t] <= limits.max*varbin[name, t]) ```

``` varcts[name, t] >= 0.0 ```

Otherwise:

``` varcts[name, t] <= limits.max*varbin[name, t] ```

``` varcts[name, t] >= limits.min*varbin[name, t] ```

where limits in constraint_infos.

# LaTeX

`` 0 \leq x^{cts} \leq limits^{max} x^{bin}, \text{ for } limits^{min} = 0 ``

`` limits^{min} x^{bin} \leq x^{cts} \leq limits^{max} x^{bin}, \text{ otherwise } ``
"""
function add_semicontinuous_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: VariableType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_variable(container, U(), V)
    _add_semicontinuous_lower_bound_range_constraints_impl!(
        container,
        T,
        array,
        devices,
        model,
    )
    _add_semicontinuous_upper_bound_range_constraints_impl!(
        container,
        T,
        array,
        devices,
        model,
    )
    return
end

function add_semicontinuous_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: RangeConstraintLBExpressions,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_expression(container, U(), V)
    _add_semicontinuous_lower_bound_range_constraints_impl!(
        container,
        T,
        array,
        devices,
        model,
    )
    return
end

function add_semicontinuous_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: RangeConstraintUBExpressions,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_expression(container, U(), V)
    _add_semicontinuous_upper_bound_range_constraints_impl!(
        container,
        T,
        array,
        devices,
        model,
    )
    return
end

function _add_semicontinuous_lower_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
) where {T <: ConstraintType, V <: PSY.Component, W <: AbstractDeviceFormulation}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    con_lb = add_constraints_container!(container, T(), V, names, time_steps; meta = "lb")
    varbin = get_variable(container, OnVariable(), V)

    for device in devices
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W) # depends on constraint type and formulation type
        for t in time_steps
            con_lb[ci_name, t] = JuMP.@constraint(
                get_jump_model(container),
                array[ci_name, t] >= limits.min * varbin[ci_name, t]
            )
        end
    end
    return
end

function _add_semicontinuous_lower_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
) where {T <: ConstraintType, V <: PSY.ThermalGen, W <: AbstractDeviceFormulation}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    con_lb = add_constraints_container!(container, T(), V, names, time_steps; meta = "lb")
    varbin = get_variable(container, OnVariable(), V)

    for device in devices
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W) # depends on constraint type and formulation type
        if PSY.get_must_run(device)
            for t in time_steps
                con_lb[ci_name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    array[ci_name, t] >= limits.min
                )
            end
        else
            for t in time_steps
                con_lb[ci_name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    array[ci_name, t] >= limits.min * varbin[ci_name, t]
                )
            end
        end
    end
    return
end

function _add_semicontinuous_upper_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {T <: ConstraintType, V <: PSY.ThermalGen, W <: AbstractDeviceFormulation}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    con_ub = add_constraints_container!(container, T(), V, names, time_steps; meta = "ub")
    varbin = get_variable(container, OnVariable(), V)

    for device in devices
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W) # depends on constraint type and formulation type
        if PSY.get_must_run(device)
            for t in time_steps
                con_ub[ci_name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    array[ci_name, t] <= limits.max
                )
            end
        else
            for t in time_steps
                con_ub[ci_name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    array[ci_name, t] <= limits.max * varbin[ci_name, t]
                )
            end
        end
    end
    return
end

function _add_semicontinuous_upper_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {T <: ConstraintType, V <: PSY.Component, W <: AbstractDeviceFormulation}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    con_ub = add_constraints_container!(container, T(), V, names, time_steps; meta = "ub")
    varbin = get_variable(container, OnVariable(), V)

    for device in devices, t in time_steps
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W) # depends on constraint type and formulation type
        con_ub[ci_name, t] = JuMP.@constraint(
            get_jump_model(container),
            array[ci_name, t] <= limits.max * varbin[ci_name, t]
        )
    end
    return
end

@doc raw"""
Constructs min/max range constraint from device variable and reservation decision variable.



``` varcts[name, t] <= limits.max * (1 - varbin[name, t]) ```

``` varcts[name, t] >= limits.min * (1 - varbin[name, t]) ```

where limits in constraint_infos.

# LaTeX

`` 0 \leq x^{cts} \leq limits^{max} (1 - x^{bin}), \text{ for } limits^{min} = 0 ``

`` limits^{min} (1 - x^{bin}) \leq x^{cts} \leq limits^{max} (1 - x^{bin}), \text{ otherwise } ``
"""
function add_reserve_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: InputActivePowerVariableLimitsConstraint,
    U <: VariableType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_variable(container, U(), V)
    _add_reserve_upper_bound_range_constraints!(container, T, array, devices, model)
    _add_reserve_lower_bound_range_constraints!(container, T, array, devices, model)
    return
end

function add_reserve_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: InputActivePowerVariableLimitsConstraint,
    U <: ExpressionType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_expression(container, U(), W)
    _add_ub(U()) &&
        _add_reserve_upper_bound_range_constraints!(container, T, array, devices, model)
    _add_lb(U()) &&
        _add_reserve_lower_bound_range_constraints!(container, T, array, devices, model)
    return
end

function _add_reserve_lower_bound_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
) where {
    T <: InputActivePowerVariableLimitsConstraint,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    binary_variables = [ReservationVariable()]

    IS.@assert_op length(binary_variables) == 1
    varbin = get_variable(container, only(binary_variables), V)

    names = [PSY.get_name(x) for x in devices]
    # MOI has a semicontinous set, but after some tests is not clear most MILP solvers support it.
    # In the future this can be updated
    con_lb = add_constraints_container!(container, T(), V, names, time_steps; meta = "lb")

    for device in devices, t in time_steps
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W)
        con_lb[ci_name, t] = JuMP.@constraint(
            get_jump_model(container),
            array[ci_name, t] >= limits.min * (1 - varbin[ci_name, t])
        )
    end
    return
end

function _add_reserve_upper_bound_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {
    T <: InputActivePowerVariableLimitsConstraint,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    binary_variables = [ReservationVariable()]

    IS.@assert_op length(binary_variables) == 1
    varbin = get_variable(container, only(binary_variables), V)

    names = [PSY.get_name(x) for x in devices]
    # MOI has a semicontinous set, but after some tests is not clear most MILP solvers support it.
    # In the future this can be updated
    con_ub = add_constraints_container!(container, T(), V, names, time_steps; meta = "ub")

    for device in devices, t in time_steps
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W)
        con_ub[ci_name, t] = JuMP.@constraint(
            get_jump_model(container),
            array[ci_name, t] <= limits.max * (1 - varbin[ci_name, t])
        )
    end
    return
end

@doc raw"""
Constructs min/max range constraint from device variable and reservation decision variable.



``` varcts[name, t] <= limits.max * varbin[name, t] ```

``` varcts[name, t] >= limits.min * varbin[name, t] ```

where limits in constraint_infos.

# LaTeX

`` limits^{min} x^{bin} \leq x^{cts} \leq limits^{max} x^{bin},``
"""
function add_reserve_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{W},
    model::DeviceModel{W, X},
    ::Type{Y},
) where {
    T <:
    Union{
        ReactivePowerVariableLimitsConstraint,
        ActivePowerVariableLimitsConstraint,
        OutputActivePowerVariableLimitsConstraint,
    },
    U <: VariableType,
    W <: PSY.Component,
    X <: AbstractDeviceFormulation,
    Y <: PM.AbstractPowerModel,
}
    array = get_variable(container, U(), W)
    _add_reserve_upper_bound_range_constraints!(container, T, array, devices, model)
    _add_reserve_lower_bound_range_constraints!(container, T, array, devices, model)
    return
end

@doc raw"""
Constructs min/max range constraint from device variable and reservation decision variable.



``` varcts[name, t] <= limits.max * varbin[name, t] ```

``` varcts[name, t] >= limits.min * varbin[name, t] ```

where limits in constraint_infos.

# LaTeX

`` limits^{min} x^{bin} \leq x^{cts} \leq limits^{max} x^{bin},``
"""
function add_reserve_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{W},
    model::DeviceModel{W, X},
    ::Type{Y},
) where {
    T <:
    Union{
        ReactivePowerVariableLimitsConstraint,
        ActivePowerVariableLimitsConstraint,
        OutputActivePowerVariableLimitsConstraint,
    },
    U <: ExpressionType,
    W <: PSY.Component,
    X <: AbstractDeviceFormulation,
    Y <: PM.AbstractPowerModel,
}
    array = get_expression(container, U(), W)
    _add_ub(U()) &&
        _add_reserve_upper_bound_range_constraints!(container, T, array, devices, model)
    _add_lb(U()) &&
        _add_reserve_lower_bound_range_constraints!(container, T, array, devices, model)
    return
end

function _add_reserve_lower_bound_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{W},
    ::DeviceModel{W, X},
) where {
    T <:
    Union{
        ReactivePowerVariableLimitsConstraint,
        ActivePowerVariableLimitsConstraint,
        OutputActivePowerVariableLimitsConstraint,
    },
    W <: PSY.Component,
    X <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    binary_variables = [ReservationVariable()]

    con_lb = add_constraints_container!(container, T(), W, names, time_steps; meta = "lb")

    @assert length(binary_variables) == 1 "Expected $(binary_variables) for $U $V $T $W to be length 1"
    varbin = get_variable(container, only(binary_variables), W)

    for device in devices, t in time_steps
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, X) # depends on constraint type and formulation type
        con_lb[ci_name, t] = JuMP.@constraint(
            get_jump_model(container),
            array[ci_name, t] >= limits.min * varbin[ci_name, t]
        )
    end
    return
end

function _add_reserve_upper_bound_range_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    devices::IS.FlattenIteratorWrapper{W},
    ::DeviceModel{W, X},
) where {
    T <:
    Union{
        ReactivePowerVariableLimitsConstraint,
        ActivePowerVariableLimitsConstraint,
        OutputActivePowerVariableLimitsConstraint,
    },
    W <: PSY.Component,
    X <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    binary_variables = [ReservationVariable()]

    con_ub = add_constraints_container!(container, T(), W, names, time_steps; meta = "ub")

    @assert length(binary_variables) == 1 "Expected $(binary_variables) for $U $V $T $W to be length 1"
    varbin = get_variable(container, only(binary_variables), W)

    for device in devices, t in time_steps
        ci_name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, X) # depends on constraint type and formulation type
        con_ub[ci_name, t] = JuMP.@constraint(
            get_jump_model(container),
            array[ci_name, t] <= limits.max * varbin[ci_name, t]
        )
    end
    return
end

function add_parameterized_lower_bound_range_constraints(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    ::Type{P},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: ExpressionType,
    P <: ParameterType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_expression(container, U(), V)
    _add_parameterized_lower_bound_range_constraints_impl!(
        container,
        T,
        array,
        P(),
        devices,
        model,
    )
    return
end

function add_parameterized_lower_bound_range_constraints(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    ::Type{P},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: VariableType,
    P <: ParameterType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_variable(container, U(), V)
    _add_parameterized_lower_bound_range_constraints_impl!(
        container,
        T,
        array,
        P(),
        devices,
        model,
    )
    return
end

# This function is re-used in SemiContinuousFeedforward
function lower_bound_range_with_parameter!(
    container::OptimizationContainer,
    constraint_container::JuMPConstraintArray,
    lhs_array,
    param::P,
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    ::DeviceModel{V, W},
) where {P <: ParameterType, V <: PSY.Component, W <: AbstractDeviceFormulation}
    param_array = get_parameter_array(container, param, V)
    param_multiplier = get_parameter_multiplier_array(container, P(), V)
    jump_model = get_jump_model(container)
    time_steps = axes(constraint_container)[2]
    for device in devices, t in time_steps
        name = PSY.get_name(device)
        constraint_container[name, t] = JuMP.@constraint(
            jump_model,
            lhs_array[name, t] >= param_multiplier[name, t] * param_array[name, t]
        )
    end
    return
end

function lower_bound_range_with_parameter!(
    container::OptimizationContainer,
    constraint_container::JuMPConstraintArray,
    lhs_array,
    param::P,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {P <: TimeSeriesParameter, V <: PSY.Component, W <: AbstractDeviceFormulation}
    param_container = get_parameter(container, param, V)
    mult = get_multiplier_array(param_container)
    jump_model = get_jump_model(container)
    time_steps = axes(constraint_container)[2]
    ts_name = get_time_series_names(model)[P]
    ts_type = get_default_time_series_type(container)
    for device in devices
        name = PSY.get_name(device)
        if !(PSY.has_time_series(device, ts_type, ts_name))
            continue
        end
        param = get_parameter_column_refs(param_container, name)
        for t in time_steps
            constraint_container[name, t] =
                JuMP.@constraint(jump_model, lhs_array[name, t] >= mult[name, t] * param[t])
        end
    end
    return
end

function _add_parameterized_lower_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    param::P,
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
) where {
    T <: ConstraintType,
    P <: TimeSeriesParameter,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    ts_name = get_time_series_names(model)[P]
    ts_type = get_default_time_series_type(container)
    # PERF: compilation hotspot. Switch to TSC.
    names = [PSY.get_name(d) for d in devices if PSY.has_time_series(d, ts_type, ts_name)]
    if isempty(names)
        @debug "There are no $V devices with time series data $ts_type, $ts_name"
        return
    end

    constraint =
        add_constraints_container!(container, T(), V, names, time_steps; meta = "lb")

    lower_bound_range_with_parameter!(container, constraint, array, param, devices, model)
    return
end

function _add_parameterized_lower_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    param::P,
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
) where {
    T <: ConstraintType,
    P <: ParameterType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    constraint =
        add_constraints_container!(container, T(), V, names, time_steps; meta = "lb")

    lower_bound_range_with_parameter!(container, constraint, array, param, devices, model)
    return
end

function add_parameterized_upper_bound_range_constraints(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    ::Type{P},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: ExpressionType,
    P <: ParameterType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_expression(container, U(), V)
    _add_parameterized_upper_bound_range_constraints_impl!(
        container,
        T,
        array,
        P(),
        devices,
        model,
    )
    return
end

function add_parameterized_upper_bound_range_constraints(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    ::Type{P},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
    ::Type{X},
) where {
    T <: ConstraintType,
    U <: VariableType,
    P <: ParameterType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    array = get_variable(container, U(), V)
    _add_parameterized_upper_bound_range_constraints_impl!(
        container,
        T,
        array,
        P(),
        devices,
        model,
    )
    return
end

# This function is re-used in SemiContinuousFeedforward
function upper_bound_range_with_parameter!(
    container::OptimizationContainer,
    constraint_container::JuMPConstraintArray,
    lhs_array,
    param::P,
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    ::DeviceModel{V, W},
) where {P <: AvailableStatusParameter, V <: PSY.Component, W <: AbstractDeviceFormulation}
    param_array = get_parameter_array(container, param, V)
    param_multiplier = get_parameter_multiplier_array(container, P(), V)
    jump_model = get_jump_model(container)
    time_steps = axes(constraint_container)[2]
    for device in devices, t in time_steps
        ub = PSY.get_max_active_power(device)
        name = PSY.get_name(device)
        constraint_container[name, t] = JuMP.@constraint(
            jump_model,
            lhs_array[name, t] <= ub * param_array[name, t]
        )
    end
    return
end

# This function is re-used in SemiContinuousFeedforward
function upper_bound_range_with_parameter!(
    container::OptimizationContainer,
    constraint_container::JuMPConstraintArray,
    lhs_array,
    param::P,
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    ::DeviceModel{V, W},
) where {P <: ParameterType, V <: PSY.Component, W <: AbstractDeviceFormulation}
    param_array = get_parameter_array(container, param, V)
    param_multiplier = get_parameter_multiplier_array(container, P(), V)
    jump_model = get_jump_model(container)
    time_steps = axes(constraint_container)[2]
    for device in devices, t in time_steps
        name = PSY.get_name(device)
        constraint_container[name, t] = JuMP.@constraint(
            jump_model,
            lhs_array[name, t] <= param_multiplier[name, t] * param_array[name, t]
        )
    end
    return
end

function upper_bound_range_with_parameter!(
    container::OptimizationContainer,
    constraint_container::JuMPConstraintArray,
    lhs_array,
    param::P,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {P <: TimeSeriesParameter, V <: PSY.Component, W <: AbstractDeviceFormulation}
    param_container = get_parameter(container, param, V)
    mult = get_multiplier_array(param_container)
    jump_model = get_jump_model(container)
    time_steps = axes(constraint_container)[2]
    ts_name = get_time_series_names(model)[P]
    ts_type = get_default_time_series_type(container)
    for device in devices
        name = PSY.get_name(device)
        if !(PSY.has_time_series(device, ts_type, ts_name))
            continue
        end
        param = get_parameter_column_refs(param_container, name)
        for t in time_steps
            constraint_container[name, t] =
                JuMP.@constraint(jump_model, lhs_array[name, t] <= mult[name, t] * param[t])
        end
    end
    return
end

function _add_parameterized_upper_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    param::P,
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
) where {
    T <: ConstraintType,
    P <: TimeSeriesParameter,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    ts_name = get_time_series_names(model)[P]
    ts_type = get_default_time_series_type(container)
    # PERF: compilation hotspot. Switch to TSC.
    names = [PSY.get_name(d) for d in devices if PSY.has_time_series(d, ts_type, ts_name)]
    if isempty(names)
        @debug "There are no $V devices with time series data $ts_type, $ts_name"
        return
    end

    constraint =
        add_constraints_container!(container, T(), V, names, time_steps; meta = "ub")

    upper_bound_range_with_parameter!(container, constraint, array, param, devices, model)
    return
end

function _add_parameterized_upper_bound_range_constraints_impl!(
    container::OptimizationContainer,
    ::Type{T},
    array,
    param::P,
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
) where {
    T <: ConstraintType,
    P <: ParameterType,
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    constraint =
        add_constraints_container!(container, T(), V, names, time_steps; meta = "ub")

    upper_bound_range_with_parameter!(container, constraint, array, param, devices, model)
    return
end
