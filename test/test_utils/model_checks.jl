const GAEVF = JuMP.GenericAffExpr{Float64, VariableRef}
const GQEVF = JuMP.GenericQuadExpr{Float64, VariableRef}

function moi_tests(
    model::DecisionModel,
    vars::Int,
    interval::Int,
    lessthan::Int,
    greaterthan::Int,
    equalto::Int,
    binary::Bool,
    lessthan_quadratic::Union{Int, Nothing} = nothing,
)
    JuMPmodel = PSI.get_jump_model(model)
    @test JuMP.num_variables(JuMPmodel) == vars
    @test JuMP.num_constraints(JuMPmodel, GAEVF, MOI.Interval{Float64}) == interval
    @test JuMP.num_constraints(JuMPmodel, GAEVF, MOI.LessThan{Float64}) == lessthan
    @test JuMP.num_constraints(JuMPmodel, GAEVF, MOI.GreaterThan{Float64}) == greaterthan
    @test JuMP.num_constraints(JuMPmodel, GAEVF, MOI.EqualTo{Float64}) == equalto
    @test ((JuMP.VariableRef, MOI.ZeroOne) in JuMP.list_of_constraint_types(JuMPmodel)) ==
          binary
    !isnothing(lessthan_quadratic) &&
        @test JuMP.num_constraints(JuMPmodel, GQEVF, MOI.LessThan{Float64}) ==
              lessthan_quadratic
    return
end

function psi_constraint_test(
    model::DecisionModel,
    constraint_keys::Vector{<:PSI.ConstraintKey},
)
    constraints = PSI.get_constraints(model)
    for con in constraint_keys
        if get(constraints, con, nothing) !== nothing
            # Ensure constraint container does not have undefined entries:
            if typeof(constraints[con]) == DenseAxisArray
                @test all(x -> isassigned(constraints[con], x), eachindex(constraints[con]))
            else
                @test true
            end
        else
            @error con
            @test false
        end
    end
    return
end

function psi_aux_variable_test(
    model::DecisionModel,
    constraint_keys::Vector{<:PSI.AuxVarKey},
)
    op_container = PSI.get_optimization_container(model)
    vars = PSI.get_aux_variables(op_container)
    for key in constraint_keys
        @test get(vars, key, nothing) !== nothing
    end
    return
end

function psi_checkbinvar_test(
    model::DecisionModel,
    bin_variable_keys::Vector{<:PSI.VariableKey},
)
    container = PSI.get_optimization_container(model)
    for variable in bin_variable_keys
        for v in PSI.get_variable(container, variable)
            @test JuMP.is_binary(v)
        end
    end
    return
end

function psi_checkobjfun_test(model::DecisionModel, exp_type)
    model = PSI.get_jump_model(model)
    @test JuMP.objective_function_type(model) == exp_type
    return
end

function moi_lbvalue_test(
    model::DecisionModel,
    con_key::PSI.ConstraintKey,
    value::Number,
)
    for con in PSI.get_constraints(model)[con_key]
        @test JuMP.constraint_object(con).set.lower == value
    end
    return
end

function psi_checksolve_test(model::DecisionModel, status)
    model = PSI.get_jump_model(model)
    JuMP.optimize!(model)
    @test termination_status(model) in status
end

function psi_checksolve_test(model::DecisionModel, status, expected_result, tol = 0.0)
    res = solve!(model)
    model = PSI.get_jump_model(model)
    @test termination_status(model) in status
    obj_value = JuMP.objective_value(model)
    @test isapprox(obj_value, expected_result, atol = tol)
end

function psi_ptdf_lmps(res::OptimizationProblemResults, ptdf)
    cp_duals = read_dual(
        res,
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System);
        table_format = TableFormat.WIDE,
    )
    λ = Matrix{Float64}(cp_duals[:, propertynames(cp_duals) .!= :DateTime])

    flow_ub = read_dual(
        res,
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub");
        table_format = TableFormat.WIDE,
    )
    flow_lb = read_dual(
        res,
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb");
        table_format = TableFormat.WIDE,
    )
    arcs = PNM.get_arc_axis(ptdf)
    nr = PNM.get_network_reduction_data(ptdf)
    branch_names = [PSY.get_name(nr.direct_branch_map[arc]) for arc in arcs]
    μ =
        Matrix{Float64}(flow_ub[:, branch_names]) .+
        Matrix{Float64}(flow_lb[:, branch_names])

    buses = get_components(Bus, get_system(res))
    lmps = OrderedDict()
    for bus in buses
        bus_number = get_number(bus)
        ptdf_col = [ptdf[arc, bus_number] for arc in arcs]
        lmps[get_name(bus)] = μ * ptdf_col
    end
    lmp = λ .+ DataFrames.DataFrame(lmps)
    return lmp[!, sort(propertynames(lmp))]
end

function check_variable_unbounded(
    model::DecisionModel,
    ::Type{T},
    ::Type{U},
) where {T <: PSI.VariableType, U <: PSY.Component}
    return check_variable_unbounded(model::DecisionModel, PSI.VariableKey(T, U))
end

function check_variable_unbounded(model::DecisionModel, var_key::PSI.VariableKey)
    psi_cont = PSI.get_optimization_container(model)
    variable = PSI.get_variable(psi_cont, var_key)
    for var in variable
        if JuMP.has_lower_bound(var) || JuMP.has_upper_bound(var)
            return false
        end
    end
    return true
end

function check_variable_bounded(
    model::DecisionModel,
    ::Type{T},
    ::Type{U},
) where {T <: PSI.VariableType, U <: PSY.Component}
    return check_variable_bounded(model, PSI.VariableKey(T, U))
end

function check_variable_bounded(model::DecisionModel, var_key::PSI.VariableKey)
    psi_cont = PSI.get_optimization_container(model)
    variable = PSI.get_variable(psi_cont, var_key)
    for var in variable
        if !JuMP.has_lower_bound(var) || !JuMP.has_upper_bound(var)
            return false
        end
    end
    return true
end

function check_flow_variable_values(
    model::DecisionModel,
    ::Type{T},
    ::Type{U},
    device_name::String,
    limit::Float64,
) where {T <: PSI.VariableType, U <: PSY.Component}
    psi_cont = PSI.get_optimization_container(model)
    variable = PSI.get_variable(psi_cont, T(), U)
    for var in variable[device_name, :]
        if !(PSI.jump_value(var) <= (limit + 1e-2))
            @error "$device_name out of bounds $(PSI.jump_value(var))"
            return false
        end
    end
    return true
end

function check_flow_variable_values(
    model::DecisionModel,
    ::Type{T},
    ::Type{U},
    device_name::String,
    limit::Float64,
) where {T <: PSI.FlowActivePowerVariable, U <: PSY.Component}
    psi_cont = PSI.get_optimization_container(model)
    template = model.template
    device_model = PSI.get_model(template, U)
    dev_formulation = PSI.get_formulation(device_model)
    net_formulation = PSI.get_network_formulation(template)
    if dev_formulation <: Union{PSI.StaticBranch, PSI.StaticBranchUnbounded} &&
       net_formulation <: PSI.PTDFPowerModel
        variable = PSI.get_expression(psi_cont, PSI.PTDFBranchFlow(), U)
    else
        variable = PSI.get_variable(psi_cont, T(), U)
    end
    for var in variable[device_name, :]
        if !(PSI.jump_value(var) <= (limit + 1e-2))
            @error "$device_name out of bounds $(PSI.jump_value(var))"
            return false
        end
    end
    return true
end

function check_flow_variable_values(
    model::DecisionModel,
    ::Type{T},
    ::Type{U},
    device_name::String,
    limit_min::Float64,
    limit_max::Float64,
) where {T <: PSI.VariableType, U <: PSY.Component}
    psi_cont = PSI.get_optimization_container(model)
    variable = PSI.get_variable(psi_cont, T(), U)
    for var in variable[device_name, :]
        if !(PSI.jump_value(var) <= (limit_max + 1e-2)) ||
           !(PSI.jump_value(var) >= (limit_min - 1e-2))
            return false
        end
    end
    return true
end

function check_flow_variable_values(
    model::DecisionModel,
    ::Type{T},
    ::Type{U},
    device_name::String,
    limit_min::Float64,
    limit_max::Float64,
) where {T <: PSI.FlowActivePowerVariable, U <: PSY.Component}
    psi_cont = PSI.get_optimization_container(model)
    template = model.template
    device_model = PSI.get_model(template, U)
    dev_formulation = PSI.get_formulation(device_model)
    net_formulation = PSI.get_network_formulation(template)
    if dev_formulation <: Union{PSI.StaticBranch, PSI.StaticBranchUnbounded} &&
       net_formulation <: PSI.PTDFPowerModel
        variable = PSI.get_expression(psi_cont, PSI.PTDFBranchFlow(), U)
    else
        variable = PSI.get_variable(psi_cont, T(), U)
    end
    for var in variable[device_name, :]
        if !(PSI.jump_value(var) <= (limit_max + 1e-2)) ||
           !(PSI.jump_value(var) >= (limit_min - 1e-2))
            return false
        end
    end
    return true
end

function check_flow_variable_values(
    model::DecisionModel,
    ::Type{T},
    ::Type{U},
    ::Type{V},
    device_name::String,
    limit_min::Float64,
    limit_max::Float64,
) where {T <: PSI.VariableType, U <: PSI.VariableType, V <: PSY.Component}
    psi_cont = PSI.get_optimization_container(model)
    time_steps = PSI.get_time_steps(psi_cont)
    pvariable = PSI.get_variable(psi_cont, T(), V)
    qvariable = PSI.get_variable(psi_cont, U(), V)
    for t in time_steps
        fp = PSI.jump_value(pvariable[device_name, t])
        fq = PSI.jump_value(qvariable[device_name, t])
        flow = sqrt((fp)^2 + (fq)^2)
        if !(flow <= (limit_max + 1e-2)^2) || !(flow >= (limit_min - 1e-2)^2)
            return false
        end
    end
    return true
end

function check_flow_variable_values(
    model::DecisionModel,
    ::Type{T},
    ::Type{U},
    ::Type{V},
    device_name::String,
    limit::Float64,
) where {T <: PSI.VariableType, U <: PSI.VariableType, V <: PSY.Component}
    psi_cont = PSI.get_optimization_container(model)
    time_steps = PSI.get_time_steps(psi_cont)
    pvariable = PSI.get_variable(psi_cont, T(), V)
    qvariable = PSI.get_variable(psi_cont, U(), V)
    for t in time_steps
        fp = PSI.jump_value(pvariable[device_name, t])
        fq = PSI.jump_value(qvariable[device_name, t])
        flow = sqrt((fp)^2 + (fq)^2)
        if !(flow <= (limit + 1e-2)^2)
            return false
        end
    end
    return true
end

function PSI.jump_value(int::Int)
    @warn("This is for testing purposes only.")
    return int
end

function _check_constraint_bounds(bounds::PSI.ConstraintBounds, valid_bounds::NamedTuple)
    @test bounds.coefficient.min == valid_bounds.coefficient.min
    @test bounds.coefficient.max == valid_bounds.coefficient.max
    @test bounds.rhs.min == valid_bounds.rhs.min
    @test bounds.rhs.max == valid_bounds.rhs.max
end

function _check_variable_bounds(bounds::PSI.VariableBounds, valid_bounds::NamedTuple)
    @test bounds.bounds.min == valid_bounds.min
    @test bounds.bounds.max == valid_bounds.max
end

function check_duration_on_initial_conditions_values(
    model,
    ::Type{T},
) where {T <: PSY.Component}
    initial_conditions_data =
        PSI.get_initial_conditions_data(PSI.get_optimization_container(model))
    duration_on_data = PSI.get_initial_condition(
        PSI.get_optimization_container(model),
        InitialTimeDurationOn(),
        T,
    )
    for ic in duration_on_data
        name = PSY.get_name(ic.component)
        on_var = PSI.get_initial_condition_value(initial_conditions_data, OnVariable(), T)[
            name,
            1,
        ]
        duration_on = PSI.jump_value(PSI.get_value(ic))
        if on_var == 1.0 && PSY.get_status(ic.component)
            @test duration_on == PSY.get_time_at_status(ic.component)
        elseif on_var == 1.0 && !PSY.get_status(ic.component)
            @test duration_on == 0.0
        end
    end
end

function check_duration_off_initial_conditions_values(
    model,
    ::Type{T},
) where {T <: PSY.Component}
    initial_conditions_data =
        PSI.get_initial_conditions_data(PSI.get_optimization_container(model))
    duration_off_data = PSI.get_initial_condition(
        PSI.get_optimization_container(model),
        InitialTimeDurationOff(),
        T,
    )
    for ic in duration_off_data
        name = PSY.get_name(ic.component)
        on_var = PSI.get_initial_condition_value(initial_conditions_data, OnVariable(), T)[
            name,
            1,
        ]
        duration_off = PSI.jump_value(PSI.get_value(ic))
        if on_var == 0.0 && !PSY.get_status(ic.component)
            @test duration_off == PSY.get_time_at_status(ic.component)
        elseif on_var == 0.0 && PSY.get_status(ic.component)
            @test duration_off == 0.0
        end
    end
end

function check_energy_initial_conditions_values(model, ::Type{T}) where {T <: PSY.Component}
    ic_data = PSI.get_initial_condition(
        PSI.get_optimization_container(model),
        InitialEnergyLevel(),
        T,
    )
    for ic in ic_data
        d = ic.component
        name = PSY.get_name(ic.component)
        e_value = PSI.jump_value(PSI.get_value(ic))
        @test PSY.get_initial_storage_capacity_level(d) * PSY.get_storage_capacity(d) *
              PSY.get_conversion_factor(d) == e_value
    end
end

function check_energy_initial_conditions_values(model, ::Type{T}) where {T <: PSY.HydroGen}
    ic_data = PSI.get_initial_condition(
        PSI.get_optimization_container(model),
        InitialEnergyLevel(),
        T,
    )
    for ic in ic_data
        name = PSY.get_name(ic.component)
        e_value = PSI.jump_value(PSI.get_value(ic))
        @test PSY.get_initial_storage(ic.component) == e_value
    end
end

function check_status_initial_conditions_values(model, ::Type{T}) where {T <: PSY.Component}
    initial_conditions =
        PSI.get_initial_condition(PSI.get_optimization_container(model), DeviceStatus(), T)
    initial_conditions_data =
        PSI.get_initial_conditions_data(PSI.get_optimization_container(model))
    for ic in initial_conditions
        name = PSY.get_name(ic.component)
        status = PSI.get_initial_condition_value(initial_conditions_data, OnVariable(), T)[
            name,
            1,
        ]
        @test PSI.jump_value(PSI.get_value(ic)) == status
    end
end

function check_active_power_initial_condition_values(
    model,
    ::Type{T},
) where {T <: PSY.Component}
    initial_conditions =
        PSI.get_initial_condition(PSI.get_optimization_container(model), DevicePower(), T)
    initial_conditions_data =
        PSI.get_initial_conditions_data(PSI.get_optimization_container(model))
    for ic in initial_conditions
        name = PSY.get_name(ic.component)
        power = PSI.get_initial_condition_value(
            initial_conditions_data,
            ActivePowerVariable(),
            T,
        )[
            name,
            1,
        ]
        @test PSI.jump_value(PSI.get_value(ic)) == power
    end
end

function check_active_power_abovemin_initial_condition_values(
    model,
    ::Type{T},
) where {T <: PSY.Component}
    initial_conditions = PSI.get_initial_condition(
        PSI.get_optimization_container(model),
        PSI.DeviceAboveMinPower(),
        T,
    )
    initial_conditions_data =
        PSI.get_initial_conditions_data(PSI.get_optimization_container(model))
    for ic in initial_conditions
        name = PSY.get_name(ic.component)
        power = PSI.get_initial_condition_value(
            initial_conditions_data,
            PSI.PowerAboveMinimumVariable(),
            T,
        )[
            name,
            1,
        ]
        @test PSI.jump_value(PSI.get_value(ic)) == power
    end
end

function check_initialization_variable_count(
    model,
    ::S,
    ::Type{T},
) where {S <: PSI.VariableType, T <: PSY.Component}
    container = PSI.get_optimization_container(model)
    initial_conditions_data = PSI.get_initial_conditions_data(container)
    no_component = length(PSY.get_components(PSY.get_available, T, model.sys))
    variable = PSI.get_initial_condition_value(initial_conditions_data, S(), T)
    rows, cols = size(variable)
    @test rows * cols == no_component * PSI.INITIALIZATION_PROBLEM_HORIZON_COUNT
end

function check_variable_count(
    model,
    ::S,
    ::Type{T},
) where {S <: PSI.VariableType, T <: PSY.Component}
    no_component = length(PSY.get_components(PSY.get_available, T, model.sys))
    time_steps = PSI.get_time_steps(PSI.get_optimization_container(model))[end]
    variable = PSI.get_variable(PSI.get_optimization_container(model), S(), T)
    @test length(variable) == no_component * time_steps
end

function check_initialization_constraint_count(
    model,
    ::S,
    ::Type{T};
    filter_func = PSY.get_available,
    meta = PSI.ISOPT.CONTAINER_KEY_EMPTY_META,
) where {S <: PSI.ConstraintType, T <: PSY.Component}
    container =
        ISOPT.get_initial_conditions_model_container(PSI.get_internal(model))
    no_component = length(PSY.get_components(filter_func, T, model.sys))
    time_steps = PSI.get_time_steps(container)[end]
    constraint = PSI.get_constraint(container, S(), T, meta)
    @test length(constraint) == no_component * time_steps
end

function check_constraint_count(
    model,
    ::S,
    ::Type{T};
    filter_func = PSY.get_available,
    meta = PSI.ISOPT.CONTAINER_KEY_EMPTY_META,
) where {S <: PSI.ConstraintType, T <: PSY.Component}
    no_component = length(PSY.get_components(filter_func, T, model.sys))
    time_steps = PSI.get_time_steps(PSI.get_optimization_container(model))[end]
    constraint = PSI.get_constraint(PSI.get_optimization_container(model), S(), T, meta)
    @test length(constraint) == no_component * time_steps
end

function check_constraint_count(
    model,
    ::PSI.RampConstraint,
    ::Type{T},
) where {T <: PSY.Component}
    container = PSI.get_optimization_container(model)
    device_name_set =
        PSY.get_name.(
            PSI._get_ramp_constraint_devices(
                container,
                get_components(PSY.get_available, T, model.sys),
            ),
        )
    check_constraint_count(
        model,
        PSI.RampConstraint(),
        T;
        meta = "up",
        filter_func = x -> x.name in device_name_set,
    )
    check_constraint_count(
        model,
        PSI.RampConstraint(),
        T;
        meta = "dn",
        filter_func = x -> x.name in device_name_set,
    )
    return
end

function check_constraint_count(
    model,
    ::PSI.DurationConstraint,
    ::Type{T},
) where {T <: PSY.Component}
    container = PSI.get_optimization_container(model)
    resolution = PSI.get_resolution(container)
    steps_per_hour = 60 / Dates.value(Dates.Minute(resolution))
    fraction_of_hour = 1 / steps_per_hour
    duration_devices = filter!(
        x -> !(
            PSY.get_time_limits(x).up <= fraction_of_hour &&
            PSY.get_time_limits(x).down <= fraction_of_hour
        ),
        collect(get_components(PSY.get_available, T, model.sys)),
    )
    device_name_set = PSY.get_name.(duration_devices)
    check_constraint_count(
        model,
        PSI.DurationConstraint(),
        T;
        meta = "up",
        filter_func = x -> x.name in device_name_set,
    )
    return check_constraint_count(
        model,
        PSI.DurationConstraint(),
        T;
        meta = "dn",
        filter_func = x -> x.name in device_name_set,
    )
end
