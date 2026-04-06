# -----------------------------------------------------
# ------ RATING FUNCTIONS FOR EMERGENCY RATINGS -------
# -----------------------------------------------------
"""
Emergency Min and max limits for Abstract Branch Formulation and Post-Contingency conditions
"""
function get_emergency_min_max_limits(
    double_circuit::PNM.BranchesParallel{<:PSY.ACTransmission},
    constraint_type::Type{<:PostContingencyConstraintType},
    branch_formulation::Type{<:AbstractBranchFormulation},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    equivalent_rating = PNM.get_equivalent_emergency_rating(double_circuit)
    return (min = -1 * equivalent_rating, max = equivalent_rating)
end

"""
Min and max limits for Abstract Branch Formulation and Post-Contingency conditions
"""
function get_emergency_min_max_limits(
    transformer_entry::PNM.ThreeWindingTransformerWinding,
    constraint_type::Type{<:PostContingencyConstraintType},
    branch_formulation::Type{<:AbstractBranchFormulation},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    equivalent_rating = PNM.get_equivalent_emergency_rating(transformer_entry)
    return (min = -1 * equivalent_rating, max = equivalent_rating)
end

"""
Min and max limits for Abstract Branch Formulation and Post-Contingency conditions
"""
function get_emergency_min_max_limits(
    series_chain::PNM.BranchesSeries,
    constraint_type::Type{<:PostContingencyConstraintType},
    branch_formulation::Type{<:AbstractBranchFormulation},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    equivalent_rating = PNM.get_equivalent_emergency_rating(series_chain)
    return (min = -1 * equivalent_rating, max = equivalent_rating)
end

"""
Min and max limits for Abstract Branch Formulation and Post-Contingency conditions
"""
function get_emergency_min_max_limits(
    device::PSY.ACTransmission,
    ::Type{<:PostContingencyConstraintType},
    ::Type{<:AbstractBranchFormulation},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    equivalent_rating = PNM.get_equivalent_emergency_rating(device)
    return (min = -1 * equivalent_rating, max = equivalent_rating)
end

"""
Min and max limits for Abstract Branch Formulation and Post-Contingency conditions
"""
function get_emergency_min_max_limits(
    entry::PSY.PhaseShiftingTransformer,
    ::Type{PhaseAngleControlLimit},
    ::Type{PhaseAngleControl},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    return get_min_max_limits(entry, PhaseAngleControlLimit, PhaseAngleControl)
end

"""
Min and max limits for monitored line
"""
function get_emergency_min_max_limits(
    device::PSY.MonitoredLine,
    ::Type{<:PostContingencyConstraintType},
    ::Type{T},
) where {T <: AbstractBranchFormulation}
    if PSY.get_flow_limits(device).to_from != PSY.get_flow_limits(device).from_to
        @warn(
            "Flow limits in Line $(PSY.get_name(device)) aren't equal. The minimum will be used in formulation $(T)"
        )
    end
    equivalent_rating = PNM.get_equivalent_emergency_rating(device)
    limit = min(
        equivalent_rating,
        PSY.get_flow_limits(device).to_from,
        PSY.get_flow_limits(device).from_to,
    )
    minmax = (min = -1 * limit, max = limit)
    return minmax
end

"""
Add branch post-contingency rate limit constraints for ACBranch considering LODF and Security Constraints
"""
function add_constraints!(
    container::OptimizationContainer,
    sys::PSY.System,
    cons_type::Type{T},
    device_model::DeviceModel{V, U},
    network_model::NetworkModel{X},
) where {
    T <: PostContingencyEmergencyFlowRateConstraint,
    V <: PSY.ACTransmission,
    U <: AbstractSecurityConstrainedStaticBranch,
    X <: AbstractPTDFModel,
}
    time_steps = get_time_steps(container)

    associated_outages = PSY.get_associated_supplemental_attributes(
        sys,
        V;
        attribute_type = PSY.UnplannedOutage,
    )

    net_reduction_data = network_model.network_reduction
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)

    modeled_branch_types = network_model.modeled_ac_branch_types

    branches_names = get_branch_argument_constraint_axis(
        net_reduction_data,
        reduced_branch_tracker,
        modeled_branch_types,
        PostContingencyEmergencyFlowRateConstraint,
    )

    con_lb =
        add_constraints_container!(
            container,
            T(),
            V,
            string.(IS.get_uuid.(associated_outages)),
            branches_names,
            time_steps;
            meta = "lb",
        )

    con_ub =
        add_constraints_container!(
            container,
            T(),
            V,
            string.(IS.get_uuid.(associated_outages)),
            branches_names,
            time_steps;
            meta = "ub",
        )

    expressions = get_expression(container, PostContingencyBranchFlow(), V)

    # Deactivating this since it does not seem that the industry or we have data for this
    #param_keys = get_parameter_keys(container)
    # param_key = ParameterKey(
    #     PostContingencyDynamicBranchRatingTimeSeriesParameter,
    #     typeof(branch),
    # )
    for outage in associated_outages
        outage_id = string(IS.get_uuid(outage))

        for b_type in modeled_branch_types
            if !haskey(
                get_constraint_map_by_type(reduced_branch_tracker)[PostContingencyEmergencyFlowRateConstraint],
                b_type,
            )
                continue
            end
            name_to_arc_map =
                get_constraint_map_by_type(reduced_branch_tracker)[PostContingencyEmergencyFlowRateConstraint][b_type]
            for (name, (arc, reduction)) in name_to_arc_map
                reduction_entry = all_branch_maps_by_type[reduction][b_type][arc]
                limits =
                    get_emergency_min_max_limits(reduction_entry, T, U)
                for t in time_steps
                    con_ub[outage_id, name, t] =
                        JuMP.@constraint(get_jump_model(container),
                            expressions[outage_id, name, t] <=
                            limits.max)
                    con_lb[outage_id, name, t] =
                        JuMP.@constraint(get_jump_model(container),
                            expressions[outage_id, name, t] >=
                            limits.min)
                end
            end
        end
    end
    return
end

function _add_post_contingency_flow_expressions_for_outage!(
    expression_container::DenseAxisArray,
    jump_model::JuMP.Model,
    time_steps::UnitRange{Int},
    modf_matrix::PNM.VirtualMODF,
    contingency_spec::PNM.ContingencySpec,
    contingency_branch_type::Type{<:PSY.ACTransmission},
    nodal_balance_expressions::Matrix{JuMP.AffExpr},
    branch_type_data::Vector{
        Tuple{
            DataType,
            SortedDict{
                String,
                Tuple{Tuple{Int64, Int64}, String},
                Base.Order.ForwardOrdering,
            },
        },
    },
)
    outage_id = string(contingency_spec.uuid)
    for (b_type, name_to_arc_map) in branch_type_data
        @debug "Adding post contingency flow expressions for branch type $b_type caused by contingencies associated with branch type $contingency_branch_type"
        tasks = map(collect(name_to_arc_map)) do pair
            (name, (arc, _)) = pair

            modf_col = modf_matrix[arc, contingency_spec]
            Threads.@spawn _make_flow_expressions!(
                jump_model,
                name,
                time_steps,
                modf_col,
                nodal_balance_expressions,
            )
        end

        for task in tasks
            name, expressions = fetch(task)
            expression_container[outage_id, name, :] .= expressions
        end
    end

    return
end

function add_post_contingency_flow_expressions!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    model::DeviceModel{V, F},
    network_model::NetworkModel{N},
) where {
    T <: PostContingencyBranchFlow,
    V <: PSY.ACTransmission,
    F <: AbstractSecurityConstrainedStaticBranch,
    N <: AbstractPTDFModel,
}
    time_steps = get_time_steps(container)
    modf_matrix = get_MODF_matrix(network_model)
    registered_contingencies = PNM.get_registered_contingencies(modf_matrix)

    net_reduction_data = network_model.network_reduction
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    modeled_branch_types = network_model.modeled_ac_branch_types

    branches_names = get_branch_argument_constraint_axis(
        net_reduction_data,
        reduced_branch_tracker,
        modeled_branch_types,
        PostContingencyEmergencyFlowRateConstraint,
    )

    expression_container = add_expression_container!(
        container,
        T(),
        V,
        string.(collect(keys(registered_contingencies))),
        branches_names,
        time_steps,
    )

    jump_model = get_jump_model(container)
    post_contingency_constraint_map =
        get_constraint_map_by_type(reduced_branch_tracker)[PostContingencyEmergencyFlowRateConstraint]
    branch_type_data = [
        (
            b_type,
            post_contingency_constraint_map[b_type],
        ) for b_type in modeled_branch_types if
        haskey(post_contingency_constraint_map, b_type)
    ]

    for (outage_id, outage_spec) in registered_contingencies
        nodal_balance_expressions = get_expression(
            container,
            ActivePowerBalance(),
            PSY.ACBus,
        )

        _add_post_contingency_flow_expressions_for_outage!(
            expression_container,
            jump_model,
            time_steps,
            modf_matrix,
            outage_spec,
            V,
            nodal_balance_expressions.data,
            branch_type_data,
        )
    end
    return
end

# For DC Power only
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, F},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.ACTransmission, F <: AbstractSecurityConstrainedStaticBranch}
    devices = get_available_components(device_model, sys)
    if get_use_slacks(device_model)
        add_variables!(
            container,
            FlowActivePowerSlackUpperBound,
            network_model,
            devices,
            F(),
        )
        add_variables!(
            container,
            FlowActivePowerSlackLowerBound,
            network_model,
            devices,
            F(),
        )
    end

    if haskey(get_time_series_names(device_model), DynamicBranchRatingTimeSeriesParameter)
        add_parameters!(
            container,
            DynamicBranchRatingTimeSeriesParameter,
            devices,
            device_model,
        )
    end

    # Deactivating this since it does not seem that the industry or we have data for this
    # if haskey(
    #     get_time_series_names(model),
    #     PostContingencyDynamicBranchRatingTimeSeriesParameter,
    # )
    #     add_parameters!(
    #         container,
    #         PostContingencyDynamicBranchRatingTimeSeriesParameter,
    #         devices,
    #         model,
    #     )
    # end

    add_feedforward_arguments!(container, device_model, devices)

    add_expressions!(
        container,
        PTDFBranchFlow,
        devices,
        device_model,
        network_model,
    )

    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{V, F},
    network_model::NetworkModel{X},
) where {
    V <: PSY.ACTransmission,
    F <: AbstractSecurityConstrainedStaticBranch,
    X <: AbstractPTDFModel,
}
    devices = get_available_components(device_model, sys)

    add_constraints!(container, FlowRateConstraint, devices, device_model, network_model)
    add_feedforward_constraints!(container, device_model, devices)
    objective_function!(container, devices, device_model, X)
    add_constraint_dual!(container, sys, device_model)

    associated_outages = PSY.get_associated_supplemental_attributes(
        sys,
        V;
        attribute_type = PSY.UnplannedOutage,
    )

    if isempty(associated_outages)
        @info "No associated outage supplemental attributes found associated with devices: $V. Skipping contingency variable addition for that device type."
        return
    end

    add_post_contingency_flow_expressions!(
        container,
        sys,
        PostContingencyBranchFlow,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        sys,
        PostContingencyEmergencyFlowRateConstraint,
        device_model,
        network_model,
    )

    return
end
