@doc raw"""
This formulation of the duration constraints adds over the start times looking backwards.

# LaTeX

* Minimum up-time constraint:

If ``t \leq d_{min}^{up} - d_{init}^{up}`` and ``d_{init}^{up} > 0``

`` 1 + \sum_{i=t-d_{min}^{up} + 1}^t x_i^{start} - x_t^{on} \leq 0 ``

for i in the set of time steps. Otherwise:

`` \sum_{i=t-d_{min}^{up} + 1}^t x_i^{start} - x_t^{on} \leq 0 ``

for i in the set of time steps.

* Minimum down-time constraint:

If ``t \leq d_{min}^{down} - d_{init}^{down}`` and ``d_{init}^{down} > 0``

`` 1 + \sum_{i=t-d_{min}^{down} + 1}^t x_i^{stop} + x_t^{on} \leq 1 ``

for i in the set of time steps. Otherwise:

`` \sum_{i=t-d_{min}^{down} + 1}^t x_i^{stop} + x_t^{on} \leq 1 ``

for i in the set of time steps.


# Arguments
* container::OptimizationContainer : the optimization_container model built in PowerSimulations
* duration_data::Vector{UpDown} : gives how many time steps variable needs to be up or down
* initial_duration::Matrix{InitialCondition} : gives initial conditions for up (column 1) and down (column 2)
* cons_name::Symbol : name of the constraint
* var_keys::Tuple{VariableKey, VariableKey, VariableKey}) : names of the variables
- : var_keys[1] : varon
- : var_keys[2] : varstart
- : var_keys[3] : varstop
"""
function device_duration_retrospective!(
    container::OptimizationContainer,
    duration_data::Vector{UpDown},
    initial_duration::Matrix{InitialCondition},
    cons_type::ConstraintType,
    var_types::Tuple{VariableType, VariableType, VariableType},
    ::Type{T},
) where {T <: PSY.Component}
    time_steps = get_time_steps(container)

    varon = get_variable(container, var_types[1], T)
    varstart = get_variable(container, var_types[2], T)
    varstop = get_variable(container, var_types[3], T)

    device_name_sets = [
        get_component_name(ic) for
        ic in initial_duration[:, 1] if !isnothing(get_value(ic))
    ]
    con_up = add_constraints_container!(
        container,
        cons_type,
        T,
        device_name_sets,
        time_steps;
        meta = "up",
    )
    con_down = add_constraints_container!(
        container,
        cons_type,
        T,
        device_name_sets,
        time_steps;
        meta = "dn",
    )

    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            isnothing(get_value(ic)) && continue
            name = get_component_name(ic)
            # Minimum Up-time Constraint
            lhs_on = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in UnitRange{Int}(Int(t - duration_data[ix].up + 1), t)
                if i in time_steps
                    JuMP.add_to_expression!(lhs_on, varstart[name, i])
                end
            end
            if t <= max(0, duration_data[ix].up - get_value(ic)) && get_value(ic) > 0
                JuMP.add_to_expression!(lhs_on, 1)
            end
            con_up[name, t] =
                JuMP.@constraint(get_jump_model(container), lhs_on - varon[name, t] <= 0.0)
        end

        for (ix, ic) in enumerate(initial_duration[:, 2])
            isnothing(get_value(ic)) && continue
            name = get_component_name(ic)
            # Minimum Down-time Constraint
            lhs_off = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in UnitRange{Int}(Int(t - duration_data[ix].down + 1), t)
                if i in time_steps
                    JuMP.add_to_expression!(lhs_off, varstop[name, i])
                end
            end
            if t <= max(0, duration_data[ix].down - get_value(ic)) && get_value(ic) > 0
                JuMP.add_to_expression!(lhs_off, 1)
            end
            con_down[name, t] =
                JuMP.@constraint(get_jump_model(container), lhs_off + varon[name, t] <= 1.0)
        end
    end
    return
end

@doc raw"""
This formulation of the duration constraints looks ahead in the time frame of the model.

# LaTeX

* Minimum up-time constraint:

If ``t \leq d_{min}^{up}``

`` d_{min}^{down}x_t^{stop} - \sum_{i=t-d_{min}^{up} + 1}^t x_i^{on} - x_{init}^{up} \leq 0 ``

for i in the set of time steps. Otherwise:

`` d_{min}^{down}x_t^{stop} - \sum_{i=t-d_{min}^{up} + 1}^t x_i^{on} \leq 0 ``

for i in the set of time steps.

* Minimum down-time constraint:

If ``t \leq d_{min}^{down}``

`` d_{min}^{up}x_t^{start} - \sum_{i=t-d_{min^{down} + 1}^t (1 - x_i^{on}) - x_{init}^{down} \leq 0 ``

for i in the set of time steps. Otherwise:

`` d_{min}^{up}x_t^{start} - \sum_{i=t-d_{min^{down} + 1}^t (1 - x_i^{on}) \leq 0 ``

for i in the set of time steps.


# Arguments
* container::OptimizationContainer : the optimization_container model built in PowerSimulations
* duration_data::Vector{UpDown} : gives how many time steps variable needs to be up or down
* initial_duration::Matrix{InitialCondition} : gives initial conditions for up (column 1) and down (column 2)
* cons_name::Symbol : name of the constraint
* var_keys::Tuple{VariableKey, VariableKey, VariableKey}) : names of the variables
- : var_keys[1] : varon
- : var_keys[2] : varstart
- : var_keys[3] : varstop
"""
function device_duration_look_ahead!(
    container::OptimizationContainer,
    duration_data::Vector{UpDown},
    initial_duration::Matrix{InitialCondition},
    cons_type_up::ConstraintType,
    cons_type_down::ConstraintType,
    var_types::Tuple{VariableType, VariableType, VariableType},
    ::Type{T},
) where {T <: PSY.Component}
    time_steps = get_time_steps(container)
    varon = get_variable(container, var_types[1], T)
    varstart = get_variable(container, var_types[2], T)
    varstop = get_variable(container, var_types[3], T)

    device_name_sets = [get_component_name(ic) for ic in initial_duration[:, 1]]
    con_up =
        add_constraints_container!(container, cons_type_up, device_name_sets, time_steps)
    con_down =
        add_constraints_container!(container, cons_type_down, device_name_sets, time_steps)

    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            isnothing(get_value(ic)) && continue
            name = get_component_name(ic)
            # Minimum Up-time Constraint
            lhs_on = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in UnitRange{Int}(Int(t - duration_data[ix].up + 1), t)
                if i in time_steps
                    JuMP.add_to_expression!(lhs_on, varon[name, i])
                end
            end
            if t <= duration_data[ix].up
                JuMP.add_to_expression!(lhs_on, get_value(ic))
            end
            con_up[name, t] = JuMP.@constraint(
                get_jump_model(container),
                varstop[name, t] * duration_data[ix].up - lhs_on <= 0.0
            )
        end

        for (ix, ic) in enumerate(initial_duration[:, 2])
            isnothing(get_value(ic)) && continue
            name = get_component_name(ic)
            # Minimum Down-time Constraint
            lhs_off = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in UnitRange{Int}(Int(t - duration_data[ix].down + 1), t)
                if i in time_steps
                    JuMP.add_to_expression!(lhs_off, 1)
                    JuMP.add_to_expression!(lhs_off, -1, varon[name, i])
                end
            end
            if t <= duration_data[ix].down
                JuMP.add_to_expression!(lhs_off, get_value(ic))
            end
            con_down[name, t] = JuMP.@constraint(
                get_jump_model(container),
                varstart[name, t] * duration_data[ix].down - lhs_off <= 0.0
            )
        end
    end

    return
end

@doc raw"""
This formulation of the duration constraints considers parameters.

# LaTeX

* Minimum up-time constraint:

If ``t \leq d_{min}^{up}``

`` d_{min}^{down}x_t^{stop} - \sum_{i=t-d_{min}^{up} + 1}^t x_i^{on} - x_{init}^{up} \leq 0 ``

for i in the set of time steps. Otherwise:

`` \sum_{i=t-d_{min}^{up} + 1}^t x_i^{start} - x_t^{on} \leq 0 ``

for i in the set of time steps.

* Minimum down-time constraint:

If ``t \leq d_{min}^{down}``

`` d_{min}^{up}x_t^{start} - \sum_{i=t-d_{min^{down} + 1}^t (1 - x_i^{on}) - x_{init}^{down} \leq 0 ``

for i in the set of time steps. Otherwise:

`` \sum_{i=t-d_{min}^{down} + 1}^t x_i^{stop} + x_t^{on} \leq 1 ``

for i in the set of time steps.

# Arguments
* container::OptimizationContainer : the optimization_container model built in PowerSimulations
* duration_data::Vector{UpDown} : gives how many time steps variable needs to be up or down
* initial_duration_on::Vector{InitialCondition} : gives initial number of time steps variable is up
* initial_duration_off::Vector{InitialCondition} : gives initial number of time steps variable is down
* cons_name::Symbol : name of the constraint
* var_keys::Tuple{VariableKey, VariableKey, VariableKey}) : names of the variables
- : var_keys[1] : varon
- : var_keys[2] : varstart
- : var_keys[3] : varstop
"""
function device_duration_parameters!(
    container::OptimizationContainer,
    duration_data::Vector{UpDown},
    initial_duration::Matrix{InitialCondition},
    cons_type::ConstraintType,
    var_types::Tuple{VariableType, VariableType, VariableType},
    ::Type{T},
) where {T <: PSY.Component}
    time_steps = get_time_steps(container)

    varon = get_variable(container, var_types[1], T)
    varstart = get_variable(container, var_types[2], T)
    varstop = get_variable(container, var_types[3], T)

    device_name_sets = [get_component_name(ic) for ic in initial_duration[:, 1]]
    con_up = add_constraints_container!(
        container,
        cons_type,
        T,
        device_name_sets,
        time_steps;
        meta = "up",
    )
    con_down = add_constraints_container!(
        container,
        cons_type,
        T,
        device_name_sets,
        time_steps;
        meta = "dn",
    )

    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            isnothing(get_value(ic)) && continue
            name = get_component_name(ic)
            # Minimum Up-time Constraint
            lhs_on = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in UnitRange{Int}(Int(t - duration_data[ix].up + 1), t)
                if t <= duration_data[ix].up
                    if in(i, time_steps)
                        JuMP.add_to_expression!(lhs_on, varon[name, i])
                    end
                else
                    JuMP.add_to_expression!(lhs_on, varstart[name, i])
                end
            end
            if t <= duration_data[ix].up
                JuMP.add_to_expression!(lhs_on, get_value(ic))
                con_up[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    varstop[name, t] * duration_data[ix].up - lhs_on <= 0.0
                )
            else
                con_up[name, t] =
                    JuMP.@constraint(
                        get_jump_model(container),
                        lhs_on - varon[name, t] <= 0.0
                    )
            end
        end

        for (ix, ic) in enumerate(initial_duration[:, 2])
            isnothing(get_value(ic)) && continue
            name = get_component_name(ic)
            # Minimum Down-time Constraint
            lhs_off = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in UnitRange{Int}(Int(t - duration_data[ix].down + 1), t)
                if t <= duration_data[ix].down
                    if in(i, time_steps)
                        JuMP.add_to_expression!(lhs_off, 1)
                        JuMP.add_to_expression!(lhs_off, -1, varon[name, i])
                    end
                else
                    JuMP.add_to_expression!(lhs_off, varstop[name, i])
                end
            end
            if t <= duration_data[ix].down
                JuMP.add_to_expression!(lhs_off, get_value(ic))
                con_down[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    varstart[name, t] * duration_data[ix].down - lhs_off <= 0.0
                )
            else
                con_down[name, t] =
                    JuMP.@constraint(
                        get_jump_model(container),
                        lhs_off + varon[name, t] <= 1.0
                    )
            end
        end
    end
    return
end

@doc raw"""
This formulation of the duration constraints adds over the start times looking backwards.

# LaTeX

* Minimum up-time constraint:

`` \sum_{i=t-min(d_{min}^{up}, T)+ 1}^t x_i^{start} - x_t^{on} \leq 0 ``

for i in the set of time steps.

* Minimum down-time constraint:

`` \sum_{i=t-min(d_{min}^{down}, T) + 1}^t x_i^{stop} + x_t^{on} \leq 1 ``

for i in the set of time steps.


# Arguments
* container::OptimizationContainer : the optimization_container model built in PowerSimulations
* duration_data::Vector{UpDown} : gives how many time steps variable needs to be up or down
* initial_duration::Matrix{InitialCondition} : gives initial conditions for up (column 1) and down (column 2)
* cons_name::Symbol : name of the constraint
* var_keys::Tuple{VariableKey, VariableKey, VariableKey}) : names of the variables
- : var_keys[1] : varon
- : var_keys[2] : varstart
- : var_keys[3] : varstop
"""
function device_duration_compact_retrospective!(
    container::OptimizationContainer,
    duration_data::Vector{UpDown},
    initial_duration::Matrix{InitialCondition},
    cons_type::ConstraintType,
    var_types::Tuple{VariableType, VariableType, VariableType},
    ::Type{T},
) where {T <: PSY.Component}
    time_steps = get_time_steps(container)

    varon = get_variable(container, var_types[1], T)
    varstart = get_variable(container, var_types[2], T)
    varstop = get_variable(container, var_types[3], T)

    device_name_sets = [get_component_name(ic) for ic in initial_duration[:, 1]]
    con_up = add_constraints_container!(
        container,
        cons_type,
        T,
        device_name_sets,
        time_steps;
        meta = "up",
        sparse = true,
    )
    con_down = add_constraints_container!(
        container,
        cons_type,
        T,
        device_name_sets,
        time_steps;
        meta = "dn",
        sparse = true,
    )
    total_time_steps = length(time_steps)
    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            isnothing(get_value(ic)) && continue
            name = get_component_name(ic)
            # Minimum Up-time Constraint
            lhs_on = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            if t in UnitRange{Int}(
                Int(min(duration_data[ix].up, total_time_steps)),
                total_time_steps,
            )
                for i in UnitRange{Int}(Int(t - duration_data[ix].up + 1), t)
                    if i in time_steps
                        JuMP.add_to_expression!(lhs_on, varstart[name, i])
                    end
                end
            elseif t <= max(0, duration_data[ix].up - get_value(ic)) && get_value(ic) > 0
                JuMP.add_to_expression!(lhs_on, 1)
            else
                continue
            end
            con_up[name, t] =
                JuMP.@constraint(get_jump_model(container), lhs_on - varon[name, t] <= 0.0)
        end

        for (ix, ic) in enumerate(initial_duration[:, 2])
            isnothing(get_value(ic)) && continue
            name = get_component_name(ic)
            # Minimum Down-time Constraint
            lhs_off = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            if t in UnitRange{Int}(
                Int(min(duration_data[ix].down, total_time_steps)),
                total_time_steps,
            )
                for i in UnitRange{Int}(Int(t - duration_data[ix].down + 1), t)
                    if i in time_steps
                        JuMP.add_to_expression!(lhs_off, varstop[name, i])
                    end
                end
            elseif t <= max(0, duration_data[ix].down - get_value(ic)) && get_value(ic) > 0
                JuMP.add_to_expression!(lhs_off, 1)
            else
                continue
            end
            con_down[name, t] =
                JuMP.@constraint(get_jump_model(container), lhs_off + varon[name, t] <= 1.0)
        end
    end
    for c in [con_up, con_down]
        # Workaround to remove invalid key combinations
        filter!(x -> !isnothing(x.second), c.data)
    end
    return
end
