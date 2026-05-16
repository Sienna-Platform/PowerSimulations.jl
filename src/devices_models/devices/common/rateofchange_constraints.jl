function _get_minutes_per_period(container::OptimizationContainer)
    resolution = get_resolution(container)
    if resolution > Dates.Minute(1)
        minutes_per_period = Dates.value(Dates.Minute(resolution))
    else
        @warn("Not all formulations support under 1-minute resolutions. Exercise caution.")
        minutes_per_period = Dates.value(Dates.Second(resolution)) / 60
    end
    return minutes_per_period
end

function _get_ramp_constraint_devices(
    container::OptimizationContainer,
    devices::Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
) where {U <: PSY.Component}
    minutes_per_period = _get_minutes_per_period(container)
    filtered_device = Vector{U}()
    for d in devices
        ramp_limits = PSY.get_ramp_limits(d)
        if !isnothing(ramp_limits)
            p_lims = PSY.get_active_power_limits(d)
            max_rate = abs(p_lims.min - p_lims.max) / minutes_per_period
            if (ramp_limits.up >= max_rate) & (ramp_limits.down >= max_rate)
                @debug "Generator has a nonbinding ramp limits. Constraints Skipped" PSY.get_name(
                    d,
                )
                continue
            else
                push!(filtered_device, d)
            end
        end
    end
    return filtered_device
end

function _get_ramp_slack_vars(
    container::OptimizationContainer,
    model::DeviceModel{V, W},
    name::String,
    t::Int,
) where {V <: PSY.Component, W <: AbstractDeviceFormulation}
    if get_use_slacks(model)
        slack_ub = get_variable(container, RateofChangeConstraintSlackUp(), V)
        slack_lb = get_variable(container, RateofChangeConstraintSlackDown(), V)
        sl_ub = slack_ub[name, t]
        sl_lb = slack_lb[name, t]
    else
        sl_ub = 0.0
        sl_lb = 0.0
    end
    return sl_ub, sl_lb
end

@doc raw"""
Constructs allowed rate-of-change constraints from variables, initial condtions, and rate data.


If t = 1:

``` variable[name, 1] - initial_conditions[ix].value <= rate_data[1][ix].up ```

``` initial_conditions[ix].value - variable[name, 1] <= rate_data[1][ix].down ```

If t > 1:

``` variable[name, t] - variable[name, t-1] <= rate_data[1][ix].up ```

``` variable[name, t-1] - variable[name, t] <= rate_data[1][ix].down ```

# LaTeX

`` r^{down} \leq x_1 - x_{init} \leq r^{up}, \text{ for } t = 1 ``

`` r^{down} \leq x_t - x_{t-1} \leq r^{up}, \forall t \geq 2 ``

"""
function add_linear_ramp_constraints!(
    container::OptimizationContainer,
    T::Type{<:ConstraintType},
    U::Type{S},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{<:PM.AbstractPowerModel},
) where {
    S <: Union{PowerAboveMinimumVariable, ActivePowerVariable},
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    variable = get_variable(container, U(), V)
    ramp_devices = _get_ramp_constraint_devices(container, devices)
    minutes_per_period = _get_minutes_per_period(container)
    IC = _get_initial_condition_type(T, V, W)
    initial_conditions_power = get_initial_condition(container, IC(), V)
    expr_dn = get_expression(container, ActivePowerRangeExpressionLB(), V)
    expr_up = get_expression(container, ActivePowerRangeExpressionUB(), V)

    device_name_set = PSY.get_name.(ramp_devices)
    con_up =
        add_constraints_container!(
            container,
            T(),
            V,
            device_name_set,
            time_steps;
            meta = "up",
        )
    con_down =
        add_constraints_container!(
            container,
            T(),
            V,
            device_name_set,
            time_steps;
            meta = "dn",
        )

    for ic in initial_conditions_power
        name = get_component_name(ic)
        # This is to filter out devices that dont need a ramping constraint
        name ∉ device_name_set && continue
        ramp_limits = PSY.get_ramp_limits(get_component(ic))
        ic_power = get_value(ic)
        @debug "add rate_of_change_constraint" name ic_power
        sl_ub, sl_lb = _get_ramp_slack_vars(container, model, name, 1)
        con_up[name, 1] = JuMP.@constraint(
            get_jump_model(container),
            expr_up[name, 1] - ic_power - sl_ub <= ramp_limits.up * minutes_per_period
        )
        con_down[name, 1] = JuMP.@constraint(
            get_jump_model(container),
            ic_power - expr_dn[name, 1] + sl_lb >=
            -1 * ramp_limits.down * minutes_per_period
        )
        for t in time_steps[2:end]
            sl_ub, sl_lb = _get_ramp_slack_vars(container, model, name, t)
            con_up[name, t] = JuMP.@constraint(
                get_jump_model(container),
                expr_up[name, t] - variable[name, t - 1] - sl_ub <=
                ramp_limits.up * minutes_per_period
            )
            con_down[name, t] = JuMP.@constraint(
                get_jump_model(container),
                variable[name, t - 1] - expr_dn[name, t] + sl_lb >=
                -1 * ramp_limits.down * minutes_per_period
            )
        end
    end
    return
end

# Helper function containing the shared ramp constraint logic
function _add_linear_ramp_constraints_impl!(
    container::OptimizationContainer,
    T::Type{<:ConstraintType},
    U::Type{<:VariableType},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {V <: PSY.Component, W <: AbstractDeviceFormulation}
    parameters = built_for_recurrent_solves(container)
    time_steps = get_time_steps(container)
    variable = get_variable(container, U(), V)
    ramp_devices = _get_ramp_constraint_devices(container, devices)
    minutes_per_period = _get_minutes_per_period(container)
    IC = _get_initial_condition_type(T, V, W)
    initial_conditions_power = get_initial_condition(container, IC(), V)

    device_name_set = PSY.get_name.(ramp_devices)
    con_up =
        add_constraints_container!(
            container,
            T(),
            V,
            device_name_set,
            time_steps;
            meta = "up",
        )
    con_down =
        add_constraints_container!(
            container,
            T(),
            V,
            device_name_set,
            time_steps;
            meta = "dn",
        )

    for ic in initial_conditions_power
        name = get_component_name(ic)
        # This is to filter out devices that dont need a ramping constraint
        name ∉ device_name_set && continue
        ramp_limits = PSY.get_ramp_limits(get_component(ic))
        ic_power = get_value(ic)
        @debug "add rate_of_change_constraint" name ic_power
        @assert (parameters && isa(ic_power, JuMP.VariableRef)) || !parameters
        sl_ub, sl_lb = _get_ramp_slack_vars(container, model, name, 1)
        con_up[name, 1] = JuMP.@constraint(
            get_jump_model(container),
            variable[name, 1] - ic_power - sl_ub <= ramp_limits.up * minutes_per_period
        )
        con_down[name, 1] = JuMP.@constraint(
            get_jump_model(container),
            ic_power - variable[name, 1] - sl_lb <= ramp_limits.down * minutes_per_period
        )
        for t in time_steps[2:end]
            sl_ub, sl_lb = _get_ramp_slack_vars(container, model, name, t)
            con_up[name, t] = JuMP.@constraint(
                get_jump_model(container),
                variable[name, t] - variable[name, t - 1] - sl_ub <=
                ramp_limits.up * minutes_per_period
            )
            con_down[name, t] = JuMP.@constraint(
                get_jump_model(container),
                variable[name, t - 1] - variable[name, t] - sl_lb <=
                ramp_limits.down * minutes_per_period
            )
        end
    end
    return
end

function add_linear_ramp_constraints!(
    container::OptimizationContainer,
    T::Type{<:ConstraintType},
    U::Type{<:VariableType},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.Component, W <: AbstractDeviceFormulation}
    return _add_linear_ramp_constraints_impl!(container, T, U, devices, model)
end

function add_linear_ramp_constraints!(
    container::OptimizationContainer,
    T::Type{<:ConstraintType},
    U::Type{ActivePowerVariable},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: AbstractThermalDispatchFormulation}

    # Fallback to generic implementation if OnStatusParameter is not present
    if !has_container_key(container, OnStatusParameter, V)
        return _add_linear_ramp_constraints_impl!(container, T, U, devices, model)
    end

    time_steps = get_time_steps(container)
    variable = get_variable(container, U(), V)
    ramp_devices = _get_ramp_constraint_devices(container, devices)
    minutes_per_period = _get_minutes_per_period(container)
    IC = _get_initial_condition_type(T, V, W)
    initial_conditions_power = get_initial_condition(container, IC(), V)

    # Commitment path from UC as a PARAMETER (fixed 0/1)
    on_param = get_parameter(container, OnStatusParameter(), V)
    on_status = on_param.parameter_array  # on_status[name, t] ∈ {0,1} (fixed)

    set_name = [PSY.get_name(r) for r in ramp_devices]
    con_up =
        add_constraints_container!(container, T(), V, set_name, time_steps; meta = "up")
    con_down =
        add_constraints_container!(container, T(), V, set_name, time_steps; meta = "dn")

    jump_model = get_jump_model(container)

    for dev in ramp_devices
        name = PSY.get_name(dev)
        ramp_limits = PSY.get_ramp_limits(dev)
        power_limits = PSY.get_active_power_limits(dev)

        # --- t = 1: Use ic_power to determine starting ramp condition
        ic_idx = findfirst(ic -> get_component_name(ic) == name, initial_conditions_power)
        ic_power = get_value(initial_conditions_power[ic_idx])
        ycur = on_status[name, 1]
        sl_ub, sl_lb = _get_ramp_slack_vars(container, model, name, 1)

        # Ramp UP from IC
        con_up[name, 1] = JuMP.@constraint(jump_model,
            variable[name, 1] - ic_power - sl_ub <=
            ramp_limits.up * minutes_per_period + power_limits.max * (1 - ycur)
        )

        # Ramp DOWN from IC  
        con_down[name, 1] = JuMP.@constraint(jump_model,
            ic_power - variable[name, 1] - sl_lb <=
            ramp_limits.down * minutes_per_period + power_limits.max * (1 - ycur)
        )

        # --- t ≥ 2: gate by previous status y_{t-1}
        for t in time_steps[2:end]
            yprev = on_status[name, t - 1]   # 0/1 fixed from UC
            ycur = on_status[name, t]       # 0/1 fixed from UC
            sl_ub, sl_lb = _get_ramp_slack_vars(container, model, name, t)

            # Ramp UP when already ON previously
            con_up[name, t] = JuMP.@constraint(jump_model,
                variable[name, t] - variable[name, t - 1] - sl_ub <=
                ramp_limits.up * minutes_per_period + power_limits.max * (2 - yprev - ycur)
            )

            # Ramp DOWN when already ON previously
            con_down[name, t] = JuMP.@constraint(jump_model,
                variable[name, t - 1] - variable[name, t] - sl_lb <=
                ramp_limits.down * minutes_per_period +
                power_limits.max * (2 - yprev - ycur)
            )
        end
    end

    return
end

@doc raw"""
Constructs allowed rate-of-change constraints from variables, initial condtions, start/stop status, and rate data

# Equations
If t = 1:

``` variable[name, 1] - initial_conditions[ix].value <= rate_data[1][ix].up + rate_data[2][ix].max*varstart[name, 1] ```

``` initial_conditions[ix].value - variable[name, 1] <= rate_data[1][ix].down + rate_data[2][ix].min*varstop[name, 1] ```

If t > 1:

``` variable[name, t] - variable[name, t-1] <= rate_data[1][ix].up + rate_data[2][ix].max*varstart[name, t] ```

``` variable[name, t-1] - variable[name, t] <= rate_data[1][ix].down + rate_data[2][ix].min*varstop[name, t] ```

# LaTeX

`` r^{down} + r^{min} x^{stop}_1 \leq x_1 - x_{init} \leq r^{up} + r^{max} x^{start}_1, \text{ for } t = 1 ``

`` r^{down} + r^{min} x^{stop}_t \leq x_t - x_{t-1} \leq r^{up} + r^{max} x^{start}_t, \forall t \geq 2 ``
"""
function add_semicontinuous_ramp_constraints!(
    container::OptimizationContainer,
    T::Type{<:ConstraintType},
    U::Type{S},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::Type{<:PM.AbstractPowerModel},
) where {
    S <: Union{PowerAboveMinimumVariable, ActivePowerVariable},
    V <: PSY.Component,
    W <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    variable = get_variable(container, U(), V)
    varstart = get_variable(container, StartVariable(), V)
    varstop = get_variable(container, StopVariable(), V)

    ramp_devices = _get_ramp_constraint_devices(container, devices)
    minutes_per_period = _get_minutes_per_period(container)
    IC = _get_initial_condition_type(T, V, W)
    initial_conditions_power = get_initial_condition(container, IC(), V)
    expr_dn = get_expression(container, ActivePowerRangeExpressionLB(), V)
    expr_up = get_expression(container, ActivePowerRangeExpressionUB(), V)

    device_name_set = PSY.get_name.(ramp_devices)
    con_up =
        add_constraints_container!(
            container,
            T(),
            V,
            device_name_set,
            time_steps;
            meta = "up",
        )
    con_down =
        add_constraints_container!(
            container,
            T(),
            V,
            device_name_set,
            time_steps;
            meta = "dn",
        )

    for ic in initial_conditions_power
        component = get_component(ic)
        name = get_component_name(ic)
        # This is to filter out devices that dont need a ramping constraint
        name ∉ device_name_set && continue
        device = get_component(ic)
        ramp_limits = PSY.get_ramp_limits(device)
        power_limits = PSY.get_active_power_limits(device)
        ic_power = get_value(ic)
        @debug "add rate_of_change_constraint" name ic_power

        if hasmethod(PSY.get_must_run, Tuple{V})
            must_run = PSY.get_must_run(component)
        else
            must_run = false
        end

        if must_run
            rhs_up = ramp_limits.up * minutes_per_period
            rhd_dn = ramp_limits.down * minutes_per_period
        else
            rhs_up =
                ramp_limits.up * minutes_per_period + power_limits.min * varstart[name, 1]
            rhd_dn =
                ramp_limits.down * minutes_per_period + power_limits.min * varstop[name, 1]
        end
        sl_ub, sl_lb = _get_ramp_slack_vars(container, model, name, 1)
        con_up[name, 1] = JuMP.@constraint(
            get_jump_model(container),
            expr_up[name, 1] - ic_power - sl_ub <=
            if must_run
                ramp_limits.up * minutes_per_period
            else
                ramp_limits.up * minutes_per_period + power_limits.min * varstart[name, 1]
            end
        )
        con_down[name, 1] = JuMP.@constraint(
            get_jump_model(container),
            ic_power - expr_dn[name, 1] - sl_lb <=
            if must_run
                ramp_limits.down * minutes_per_period
            else
                ramp_limits.down * minutes_per_period + power_limits.min * varstop[name, 1]
            end
        )
        for t in time_steps[2:end]
            sl_ub, sl_lb = _get_ramp_slack_vars(container, model, name, t)
            con_up[name, t] = JuMP.@constraint(
                get_jump_model(container),
                expr_up[name, t] - variable[name, t - 1] - sl_ub <=
                if must_run
                    ramp_limits.up * minutes_per_period
                else
                    ramp_limits.up * minutes_per_period + power_limits.min * varstart[name, t]
                end
            )
            con_down[name, t] = JuMP.@constraint(
                get_jump_model(container),
                variable[name, t - 1] - expr_dn[name, t] - sl_lb <=
                if must_run
                    ramp_limits.down * minutes_per_period
                else
                    ramp_limits.down * minutes_per_period + power_limits.min * varstop[name, t]
                end
            )
        end
    end
    return
end
