#! format: off
get_multiplier_value(::FromToFlowLimitParameter, d::PSY.AreaInterchange, ::AbstractBranchFormulation) = -1.0 * PSY.get_from_to_flow_limit(d)
get_multiplier_value(::ToFromFlowLimitParameter, d::PSY.AreaInterchange, ::AbstractBranchFormulation) = PSY.get_to_from_flow_limit(d)

get_parameter_multiplier(::FixValueParameter, ::PSY.AreaInterchange, ::AbstractBranchFormulation) = 1.0
get_parameter_multiplier(::LowerBoundValueParameter, ::PSY.AreaInterchange, ::AbstractBranchFormulation) = 1.0
get_parameter_multiplier(::UpperBoundValueParameter, ::PSY.AreaInterchange, ::AbstractBranchFormulation) = 1.0

get_initial_conditions_device_model(
    ::OperationModel,
    model::DeviceModel{PSY.AreaInterchange, T},
) where {T <: AbstractBranchFormulation} = DeviceModel(PSY.AreaInterchange, T)

#! format: on

function get_default_time_series_names(
    ::Type{PSY.AreaInterchange},
    ::Type{V},
) where {V <: AbstractBranchFormulation}
    return Dict{Type{<:TimeSeriesParameter}, String}(
        FromToFlowLimitParameter => "from_to_flow_limit",
        ToFromFlowLimitParameter => "to_from_flow_limit",
    )
end

function get_default_attributes(
    ::Type{PSY.AreaInterchange},
    ::Type{V},
) where {V <: AbstractBranchFormulation}
    return Dict{String, Any}()
end

function add_variables!(
    container::OptimizationContainer,
    ::Type{FlowActivePowerVariable},
    model::NetworkModel{T},
    devices::IS.FlattenIteratorWrapper{PSY.AreaInterchange},
    formulation::AbstractBranchFormulation,
) where {T <: PM.AbstractPowerModel}
    time_steps = get_time_steps(container)

    variable = add_variable_container!(
        container,
        FlowActivePowerVariable(),
        PSY.AreaInterchange,
        PSY.get_name.(devices),
        time_steps,
    )

    for device in devices, t in time_steps
        device_name = get_name(device)
        variable[device_name, t] = JuMP.@variable(
            get_jump_model(container),
            base_name = "FlowActivePowerVariable_AreaInterchange_{$(device_name), $(t)}",
        )
    end
    return
end

function add_variables!(
    container::OptimizationContainer,
    ::Type{FlowActivePowerVariable},
    model::NetworkModel{CopperPlatePowerModel},
    devices::IS.FlattenIteratorWrapper{PSY.AreaInterchange},
    formulation::AbstractBranchFormulation,
)
    @warn(
        "CopperPlatePowerModel ignores AreaInterchanges. Instead use AreaBalancePowerModel."
    )
    return
end

"""
Add flow constraints for area interchanges
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{FlowLimitConstraint},
    devices::IS.FlattenIteratorWrapper{PSY.AreaInterchange},
    model::DeviceModel{PSY.AreaInterchange, StaticBranch},
    ::NetworkModel{T},
) where {T <: PM.AbstractActivePowerModel}
    time_steps = get_time_steps(container)
    device_names = PSY.get_name.(devices)

    con_ub = add_constraints_container!(
        container,
        FlowLimitConstraint(),
        PSY.AreaInterchange,
        device_names,
        time_steps;
        meta = "ub",
    )

    con_lb = add_constraints_container!(
        container,
        FlowLimitConstraint(),
        PSY.AreaInterchange,
        device_names,
        time_steps;
        meta = "lb",
    )

    var_array = get_variable(container, FlowActivePowerVariable(), PSY.AreaInterchange)
    if !all(PSY.has_time_series.(devices))
        for device in devices
            ci_name = PSY.get_name(device)
            to_from_limit = PSY.get_flow_limits(device).to_from
            from_to_limit = PSY.get_flow_limits(device).from_to
            for t in time_steps
                con_lb[ci_name, t] =
                    JuMP.@constraint(
                        get_jump_model(container),
                        var_array[ci_name, t] >= -1.0 * from_to_limit
                    )
                con_ub[ci_name, t] =
                    JuMP.@constraint(
                        get_jump_model(container),
                        var_array[ci_name, t] <= to_from_limit
                    )
            end
        end
    else
        param_container_from_to =
            get_parameter(container, FromToFlowLimitParameter(), PSY.AreaInterchange)
        param_multiplier_from_to = get_parameter_multiplier_array(
            container,
            FromToFlowLimitParameter(),
            PSY.AreaInterchange,
        )
        param_container_to_from =
            get_parameter(container, ToFromFlowLimitParameter(), PSY.AreaInterchange)
        param_multiplier_to_from = get_parameter_multiplier_array(
            container,
            ToFromFlowLimitParameter(),
            PSY.AreaInterchange,
        )
        jump_model = get_jump_model(container)
        for device in devices
            name = PSY.get_name(device)
            param_from_to = get_parameter_column_refs(param_container_from_to, name)
            param_to_from = get_parameter_column_refs(param_container_to_from, name)
            for t in time_steps
                con_lb[name, t] = JuMP.@constraint(
                    jump_model,
                    var_array[name, t] >=
                    param_multiplier_from_to[name, t] * param_from_to[t]
                )
                con_ub[name, t] = JuMP.@constraint(
                    jump_model,
                    var_array[name, t] <=
                    param_multiplier_to_from[name, t] * param_to_from[t]
                )
            end
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{LineFlowBoundConstraint},
    devices::IS.FlattenIteratorWrapper{PSY.AreaInterchange},
    model::DeviceModel{PSY.AreaInterchange, <:AbstractBranchFormulation},
    network_model::NetworkModel{T},
    inter_area_branch_map::Dict{
        Tuple{String, String},
        Dict{DataType, Vector{String}},
    },
) where {T <: AbstractPTDFModel}
    @assert !isempty(inter_area_branch_map)
    time_steps = get_time_steps(container)
    net_reduction_data = get_network_reduction(network_model)
    device_names_with_branches = Vector{String}()
    interchange_direction_branch_map =
        Dict{String, Dict{Float64, Dict{DataType, Vector{String}}}}()
    for area_interchange in devices
        inter_change_name = PSY.get_name(area_interchange)
        area_from_name = PSY.get_name(PSY.get_from_area(area_interchange))
        area_to_name = PSY.get_name(PSY.get_to_area(area_interchange))
        interchange_direction_branch_map[inter_change_name] =
            Dict{Float64, Dict{DataType, Vector{String}}}()
        if haskey(inter_area_branch_map, (area_from_name, area_to_name))
            # 1 is the multiplier
            interchange_direction_branch_map[inter_change_name][1.0] =
                inter_area_branch_map[(area_from_name, area_to_name)]
        end
        if haskey(inter_area_branch_map, (area_to_name, area_from_name))
            # -1 is the multiplier because the direction is reversed
            interchange_direction_branch_map[inter_change_name][-1.0] =
                inter_area_branch_map[(area_to_name, area_from_name)]
        end
        if isempty(interchange_direction_branch_map[inter_change_name])
            @warn(
                "There are no branches modeled in Area InterChange $(summary(area_interchange)) \
          LineFlowBoundConstraint not created"
            )
        else
            push!(device_names_with_branches, inter_change_name)
        end
    end
    con_ub = add_constraints_container!(
        container,
        LineFlowBoundConstraint(),
        PSY.AreaInterchange,
        device_names_with_branches,
        time_steps;
        meta = "ub",
    )

    con_lb = add_constraints_container!(
        container,
        LineFlowBoundConstraint(),
        PSY.AreaInterchange,
        device_names_with_branches,
        time_steps;
        meta = "lb",
    )

    area_ex_var = get_variable(container, FlowActivePowerVariable(), PSY.AreaInterchange)
    jm = get_jump_model(container)
    # The orientation sign depends only on (type, name); memoize it so it is
    # not recomputed (Dict lookup + series scan) once per time step.
    sign_cache = Dict{Tuple{DataType, String}, Float64}()
    for area_interchange in devices
        inter_change_name = PSY.get_name(area_interchange)
        (inter_change_name ∉ device_names_with_branches) && continue
        direction_branch_map = interchange_direction_branch_map[inter_change_name]

        for t in time_steps
            sum_of_flows = JuMP.AffExpr()
            for (mult, inter_area_branches) in direction_branch_map
                for (type, names) in inter_area_branches
                    flow_expr = get_expression(container, PTDFBranchFlow(), type)
                    for name in names
                        # flow_expr is native-oriented; re-apply the member
                        # sign so the interchange total is orientation-invariant.
                        sign = get!(sign_cache, (type, name)) do
                            get_ptdf_orientation_sign(net_reduction_data, type, name)
                        end
                        JuMP.add_to_expression!(
                            sum_of_flows,
                            flow_expr[name, t],
                            mult * sign,
                        )
                    end
                end
            end
            con_ub[inter_change_name, t] =
                JuMP.@constraint(jm, sum_of_flows <= area_ex_var[inter_change_name, t])
            con_lb[inter_change_name, t] =
                JuMP.@constraint(jm, sum_of_flows >= area_ex_var[inter_change_name, t])
        end
    end
    return
end
