#################################################################################
# Comments
#
# - Ideally the net_injection variables would be bounded.  This can be done using an adhoc data model extension
# - the `instantiate_*_expr_model` functions combine `PM.instantiate_model` and the `build_*` methods
#################################################################################
# Model Definitions

const UNSUPPORTED_POWERMODELS =
    [
        PM.SOCBFPowerModel,
        PM.SOCBFConicPowerModel,
        PM.IVRPowerModel,
        PM.SparseSDPWRMPowerModel,
    ]

function instantiate_nip_expr_model(data::Dict{String, Any}, model_constructor; kwargs...)
    return PM.instantiate_model(data, model_constructor, instantiate_nip_expr; kwargs...)
end

# replicates PM.build_mn_opf
function instantiate_nip_expr(pm::PM.AbstractPowerModel)
    for n in eachindex(PM.nws(pm))
        PM.variable_bus_voltage(pm; nw = n)
        PM.variable_branch_power(pm; nw = n, bounded = false)
        PM.variable_dcline_power(pm; nw = n, bounded = false)

        PM.constraint_model_voltage(pm; nw = n)

        for i in PM.ids(pm, :ref_buses; nw = n)
            PM.constraint_theta_ref(pm, i; nw = n)
        end

        for i in PM.ids(pm, :bus; nw = n)
            constraint_power_balance_ni_expr(pm, i; nw = n)
        end

        for i in PM.ids(pm, :branch; nw = n)
            PM.constraint_ohms_yt_from(pm, i; nw = n)
            PM.constraint_ohms_yt_to(pm, i; nw = n)

            PM.constraint_voltage_angle_difference(pm, i; nw = n)
        end

        for i in PM.ids(pm, :dcline; nw = n)
            PM.constraint_dcline_power_losses(pm, i; nw = n)
        end
    end

    return
end

function instantiate_bfp_expr_model(data::Dict{String, Any}, model_constructor; kwargs...)
    return PM.instantiate_model(data, model_constructor, instantiate_bfp_expr; kwargs...)
end

# replicates PM.build_mn_opf_bf_strg
function instantiate_bfp_expr(pm::PM.AbstractPowerModel)
    for n in eachindex(PM.nws(pm))
        PM.variable_bus_voltage(pm; nw = n)
        PM.variable_branch_power(pm; nw = n, bounded = false)
        PM.variable_dcline_power(pm; nw = n, bounded = false)

        PM.constraint_model_current(pm; nw = n)

        for i in PM.ids(pm, :ref_buses; nw = n)
            PM.constraint_theta_ref(pm, i; nw = n)
        end

        for i in PM.ids(pm, :bus; nw = n)
            constraint_power_balance_ni_expr(pm, i; nw = n)
        end

        for i in PM.ids(pm, :branch; nw = n)
            PM.constraint_power_losses(pm, i; nw = n)
            PM.constraint_voltage_magnitude_difference(pm, i; nw = n)

            PM.constraint_voltage_angle_difference(pm, i; nw = n)
        end

        for i in PM.ids(pm, :dcline; nw = n)
            PM.constraint_dcline_power_losses(pm, i; nw = n)
        end
    end

    return
end

#=
# VI Methdos not supported currently
function instantiate_vip_expr_model(data::Dict{String, Any}, model_constructor; kwargs...)
    throw(error("VI Models not currently supported"))
end
=#

#################################################################################
# Model Extention Functions

function constraint_power_balance_ni_expr(
    pm::PM.AbstractPowerModel,
    i::Int;
    nw::Int = pm.cnw,
)
    if !haskey(PM.con(pm, nw), :power_balance_p)
        PM.con(pm, nw)[:power_balance_p] = Dict{Int, JuMP.ConstraintRef}()
    end
    if !haskey(PM.con(pm, nw), :power_balance_q)
        PM.con(pm, nw)[:power_balance_q] = Dict{Int, JuMP.ConstraintRef}()
    end

    bus_arcs = PM.ref(pm, nw, :bus_arcs, i)
    bus_arcs_dc = PM.ref(pm, nw, :bus_arcs_dc, i)

    inj_p_expr = PM.ref(pm, nw, :bus, i, "inj_p")
    inj_q_expr = PM.ref(pm, nw, :bus, i, "inj_q")

    constraint_power_balance_ni_expr(
        pm,
        nw,
        i,
        bus_arcs,
        bus_arcs_dc,
        inj_p_expr,
        inj_q_expr,
    )

    return
end

function constraint_power_balance_ni_expr(
    pm::PM.AbstractPowerModel,
    n::Int,
    i::Int,
    bus_arcs,
    bus_arcs_dc,
    inj_p_expr,
    inj_q_expr,
)
    p = PM.var(pm, n, :p)
    q = PM.var(pm, n, :q)
    p_dc = PM.var(pm, n, :p_dc)
    q_dc = PM.var(pm, n, :q_dc)

    PM.con(pm, n, :power_balance_p)[i] = JuMP.@constraint(
        pm.model,
        sum(p[a] for a in bus_arcs) + sum(p_dc[a_dc] for a_dc in bus_arcs_dc) == inj_p_expr
    )
    PM.con(pm, n, :power_balance_q)[i] = JuMP.@constraint(
        pm.model,
        sum(q[a] for a in bus_arcs) + sum(q_dc[a_dc] for a_dc in bus_arcs_dc) == inj_q_expr
    )

    return
end

#=
# VI Methdos not supported currently
function constraint_current_balance_ni_expr(
    pm::PM.AbstractPowerModel,
    i::Int;
    nw::Int = pm.cnw,
)
    if !haskey(PM.con(pm, nw), :kcl_cr)
        PM.con(pm, nw)[:kcl_cr] = Dict{Int, JuMP.ConstraintRef}()
    end
    if !haskey(PM.con(pm, nw), :kcl_ci)
        PM.con(pm, nw)[:kcl_ci] = Dict{Int, JuMP.ConstraintRef}()
    end

    bus_arcs = PM.ref(pm, nw, :bus_arcs, i)
    bus_arcs_dc = PM.ref(pm, nw, :bus_arcs_dc, i)

    inj_p_expr = PM.ref(pm, nw, :bus, i, "inj_p")
    inj_q_expr = PM.ref(pm, nw, :bus, i, "inj_q")

    constraint_current_balance_ni_expr(
        pm,
        nw,
        i,
        bus_arcs,
        bus_arcs_dc,
        inj_p_expr,
        inj_q_expr,
    )

    return
end

function constraint_current_balance_ni_expr(
    pm::PM.AbstractPowerModel,
    n::Int,
    i::Int,
    bus_arcs,
    bus_arcs_dc,
    inj_p_expr,
    inj_q_expr,
)
    p = PM.var(pm, n, :p)
    q = PM.var(pm, n, :q)
    p_dc = PM.var(pm, n, :p_dc)
    q_dc = PM.var(pm, n, :q_dc)

    PM.con(pm, n, :power_balance_p)[i] = JuMP.@constraint(
        pm.model,
        sum(p[a] for a in bus_arcs) + sum(p_dc[a_dc] for a_dc in bus_arcs_dc) == inj_p_expr
    )
    PM.con(pm, n, :power_balance_q)[i] = JuMP.@constraint(
        pm.model,
        sum(q[a] for a in bus_arcs) + sum(q_dc[a_dc] for a_dc in bus_arcs_dc) == inj_q_expr
    )

    return
end
=#

"""
active power only models ignore reactive power variables
"""
function variable_reactive_net_injection(pm::PM.AbstractActivePowerModel; kwargs...)
    return
end

function constraint_power_balance_ni_expr(
    pm::PM.AbstractActivePowerModel,
    n::Int,
    i::Int,
    bus_arcs,
    bus_arcs_dc,
    inj_p_expr,
    _,
)
    p = PM.var(pm, n, :p)
    p_dc = PM.var(pm, n, :p_dc)

    PM.con(pm, n, :power_balance_p)[i] = JuMP.@constraint(
        pm.model,
        sum(p[a] for a in bus_arcs) + sum(p_dc[a_dc] for a_dc in bus_arcs_dc) == inj_p_expr
    )

    return
end

function powermodels_network!(
    container::OptimizationContainer,
    system_formulation::Type{S},
    sys::PSY.System,
    template::ProblemTemplate,
    instantiate_model,
) where {S <: PM.AbstractPowerModel}
    time_steps = get_time_steps(container)
    pm_data, PM_map = pass_to_pm(sys, template, time_steps[end])

    network_model = get_network_model(template)
    network_reduction = get_network_reduction(network_model)
    if isempty(network_reduction)
        ac_bus_numbers =
            PSY.get_number.(get_available_components(network_model, PSY.ACBus, sys))
    else
        bus_reduction_map = PNM.get_bus_reduction_map(network_reduction)
        ac_bus_numbers = collect(keys(bus_reduction_map))
    end

    for t in time_steps, bus_no in ac_bus_numbers
        pm_data["nw"]["$(t)"]["bus"]["$bus_no"]["inj_p"] =
            container.expressions[ExpressionKey(ActivePowerBalance, PSY.ACBus)][
                bus_no,
                t,
            ]
        pm_data["nw"]["$(t)"]["bus"]["$bus_no"]["inj_q"] =
            container.expressions[ExpressionKey(ReactivePowerBalance, PSY.ACBus)][
                bus_no,
                t,
            ]
    end

    container.pm =
        instantiate_model(pm_data, system_formulation; jump_model = container.JuMPmodel)
    container.pm.ext[:PMmap] = PM_map

    return
end

function powermodels_network!(
    container::OptimizationContainer,
    system_formulation::Type{S},
    sys::PSY.System,
    template::ProblemTemplate,
    instantiate_model,
) where {S <: PM.AbstractActivePowerModel}
    time_steps = get_time_steps(container)
    pm_data, PM_map = pass_to_pm(sys, template, time_steps[end])

    network_model = get_network_model(template)
    network_reduction = get_network_reduction(network_model)
    if isempty(network_reduction)
        ac_bus_numbers =
            PSY.get_number.(get_available_components(network_model, PSY.ACBus, sys))
    else
        bus_reduction_map = PNM.get_bus_reduction_map(network_reduction)
        ac_bus_numbers = collect(keys(bus_reduction_map))
    end

    for t in time_steps, bus_no in ac_bus_numbers
        pm_data["nw"]["$(t)"]["bus"]["$bus_no"]["inj_p"] =
            container.expressions[ExpressionKey(ActivePowerBalance, PSY.ACBus)][
                bus_no,
                t,
            ]
        # pm_data["nw"]["$(t)"]["bus"]["$(bus.number)"]["inj_q"] = 0.0
    end

    container.pm =
        instantiate_model(
            pm_data,
            system_formulation;
            jump_model = get_jump_model(container),
        )
    container.pm.ext[:PMmap] = PM_map

    return
end

#### PM accessor functions ########

function PMvarmap(::Type{S}) where {S <: PM.AbstractDCPModel}
    pm_variable_map = Dict{Type, Dict{Symbol, Union{VariableType, NamedTuple}}}()

    pm_variable_map[PSY.ACBus] = Dict(:va => VoltageAngle())
    pm_variable_map[PSY.ACTransmission] =
        Dict(:p => (from_to = FlowActivePowerVariable(), to_from = nothing))
    pm_variable_map[PSY.TwoTerminalHVDC] =
        Dict(:p_dc => (from_to = FlowActivePowerVariable(), to_from = nothing))

    return pm_variable_map
end

function PMvarmap(::Type{S}) where {S <: PM.AbstractActivePowerModel}
    pm_variable_map = Dict{Type, Dict{Symbol, Union{VariableType, NamedTuple}}}()

    pm_variable_map[PSY.ACBus] = Dict(:va => VoltageAngle())
    pm_variable_map[PSY.ACTransmission] = Dict(:p => FlowActivePowerFromToVariable())
    pm_variable_map[PSY.TwoTerminalHVDC] = Dict(
        :p_dc => (
            from_to = FlowActivePowerFromToVariable(),
            to_from = FlowActivePowerToFromVariable(),
        ),
    )

    return pm_variable_map
end

function PMvarmap(::Type{S}) where {S <: PM.AbstractPowerModel}
    pm_variable_map = Dict{Type, Dict{Symbol, Union{VariableType, NamedTuple}}}()

    pm_variable_map[PSY.ACBus] = Dict(:va => VoltageAngle(), :vm => VoltageMagnitude())
    pm_variable_map[PSY.ACTransmission] = Dict(
        :p => (
            from_to = FlowActivePowerFromToVariable(),
            to_from = FlowActivePowerToFromVariable(),
        ),
        :q => (
            from_to = FlowReactivePowerFromToVariable(),
            to_from = FlowReactivePowerToFromVariable(),
        ),
    )
    pm_variable_map[PSY.TwoTerminalHVDC] = Dict(
        :p_dc => (from_to = FlowActivePowerVariable(), to_from = nothing),
        :q_dc => (
            from_to = FlowReactivePowerFromToVariable(),
            to_from = FlowReactivePowerToFromVariable(),
        ),
    )

    return pm_variable_map
end

function PMconmap(::Type{S}) where {S <: PM.AbstractActivePowerModel}
    pm_constraint_map = Dict{Type, Dict{Symbol, <:ConstraintType}}()

    pm_constraint_map[PSY.ACBus] = Dict(:power_balance_p => NodalBalanceActiveConstraint())
    return pm_constraint_map
end

function PMconmap(::Type{S}) where {S <: PM.AbstractPowerModel}
    pm_constraint_map = Dict{Type, Dict{Symbol, ConstraintType}}()

    pm_constraint_map[PSY.ACBus] = Dict(
        :power_balance_p => NodalBalanceActiveConstraint(),
        :power_balance_q => NodalBalanceReactiveConstraint(),
    )
    return pm_constraint_map
end

function PMexprmap(::Type{S}) where {S <: PM.AbstractPowerModel}
    pm_expr_map = Dict{
        Type,
        NamedTuple{
            (:pm_expr, :psi_con),
            Tuple{Dict{Symbol, Union{VariableType, NamedTuple}}, Symbol},
        },
    }()

    return pm_expr_map
end

function add_pm_variable_refs!(
    container::OptimizationContainer,
    system_formulation::Type{S},
    ::PSY.System,
    model::NetworkModel,
) where {S <: PM.AbstractPowerModel}
    time_steps = get_time_steps(container)
    bus_dict = container.pm.ext[:PMmap].bus
    ACbranch_dict = container.pm.ext[:PMmap].arcs
    ACbranch_types = PNM.get_ac_transmission_types(model.network_reduction)
    DCbranch_dict = container.pm.ext[:PMmap].arcs_dc
    DCbranch_types = Set(typeof.(values(DCbranch_dict)))

    pm_variable_types = keys(PM.var(container.pm, 1))

    pm_variable_map = PMvarmap(system_formulation)
    bus_names = [PSY.get_name(b) for b in values(bus_dict)]
    for (pm_v, ps_v) in pm_variable_map[PSY.ACBus]
        if pm_v in pm_variable_types
            var_container =
                add_variable_container!(container, ps_v, PSY.ACBus, bus_names, time_steps)
            for t in time_steps, (pm_bus, bus) in bus_dict
                name = PSY.get_name(bus)
                var_container[name, t] = PM.var(container.pm, t, pm_v)[pm_bus] # pm_vars[pm_v][pm_bus]
            end
        end
    end

    add_pm_variable_refs!(
        container,
        model,
        PSY.ACTransmission,
        ACbranch_types,
        ACbranch_dict,
        pm_variable_map,
        pm_variable_types,
        time_steps,
    )
    add_pm_variable_refs!(
        container,
        model,
        PSY.TwoTerminalHVDC,
        DCbranch_types,
        DCbranch_dict,
        pm_variable_map,
        pm_variable_types,
        time_steps,
    )
    return
end

function add_pm_variable_refs!(
    container::OptimizationContainer,
    model::NetworkModel,
    d_class::Type{PSY.ACTransmission},
    device_types::Set,
    pm_map::Dict,
    pm_variable_map::Dict,
    pm_variable_types::Base.KeySet,
    time_steps::UnitRange{Int},
)
    net_reduction_data = get_network_reduction(model)
    reduced_branch_tracker = get_reduced_branch_tracker(model)
    for d_type in device_types, (pm_v, ps_v) in pm_variable_map[d_class]
        if pm_v in pm_variable_types
            for direction in fieldnames(typeof(ps_v))
                var_type = getfield(ps_v, direction)
                var_type === nothing && continue
                branch_names =
                    get_branch_argument_variable_axis(net_reduction_data, d_type)
                var_container = add_variable_container!(
                    container,
                    var_type,
                    d_type,
                    branch_names,
                    time_steps,
                )
                for (name, (arc_tuple, reduction)) in
                    PNM.get_name_to_arc_map(net_reduction_data, d_type)
                    has_entry, tracker_container = search_for_reduced_branch_argument!(
                        reduced_branch_tracker,
                        arc_tuple,
                        typeof(var_type), # TODO: Make the mapping not rely on instances but types
                    )
                    if has_entry
                        @assert !isempty(tracker_container) name arc_tuple reduction
                    end
                    for t in time_steps
                        if !has_entry
                            pm_d = pm_map[arc_tuple]
                            var = PM.var(container.pm, t, pm_v, getfield(pm_d, direction))
                            tracker_container[t] = var
                        end
                        var_container[name, t] = tracker_container[t]
                    end
                end
            end
        end
    end
    return
end

function add_pm_variable_refs!(
    container::OptimizationContainer,
    ::NetworkModel,
    d_class::Type{PSY.TwoTerminalHVDC},
    device_types::Set,
    pm_map::Dict,
    pm_variable_map::Dict,
    pm_variable_types::Base.KeySet,
    time_steps::UnitRange{Int},
)
    for d_type in Set(device_types)
        devices = [d for d in pm_map if typeof(d[2]) == d_type]
        for (pm_v, ps_v) in pm_variable_map[d_class]
            if pm_v in pm_variable_types
                for dir in fieldnames(typeof(ps_v))
                    var_type = getfield(ps_v, dir)
                    var_type === nothing && continue
                    var_container = add_variable_container!(
                        container,
                        var_type,
                        d_type,
                        [PSY.get_name(d[2]) for d in devices],
                        time_steps,
                    )
                    for t in time_steps, (pm_d, d) in devices
                        var = PM.var(container.pm, t, pm_v, getfield(pm_d, dir))
                        var_container[PSY.get_name(d), t] = var
                    end
                end
            end
        end
    end
    return
end

function add_pm_constraint_refs!(
    container::OptimizationContainer,
    system_formulation::Type{S},
    ::PSY.System,
) where {S <: PM.AbstractPowerModel}
    time_steps = get_time_steps(container)
    bus_dict = container.pm.ext[:PMmap].bus

    pm_constraint_names =
        [k for k in keys(PM.con(container.pm, 1)) if !isempty(PM.con(container.pm, 1, k))]

    pm_constraint_map = PMconmap(system_formulation)
    for (pm_v, ps_v) in pm_constraint_map[PSY.ACBus]
        if pm_v in pm_constraint_names
            cons_container = add_constraints_container!(
                container,
                ps_v,
                PSY.ACBus,
                [PSY.get_name(b) for b in values(bus_dict)],
                time_steps,
            )
            for t in time_steps, (pm_bus, bus) in bus_dict
                name = PSY.get_name(bus)
                cons_container[name, t] = PM.con(container.pm, t, pm_v)[pm_bus]
            end
        end
    end
    return
end
