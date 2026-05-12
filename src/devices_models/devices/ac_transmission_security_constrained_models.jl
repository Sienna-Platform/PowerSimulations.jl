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
Pre-allocate a `SparseAxisArray` keyed by
`(outage_id::String, monitored_name::String, t::Int)` holding `JuMP.AffExpr`
zeros for every entry produced by `_resolve_monitored_arcs`. The pre-fill is
required so the parallel PTDF expression build below cannot race on Dict
resize. Registered on `container.expressions` under `ExpressionKey(T, V)`.
"""
function _add_post_contingency_sparse_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{V},
    resolved::Vector{
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}},
    },
    time_steps::UnitRange{Int},
) where {T <: PostContingencyExpressions, V <: PSY.ACTransmission}
    contents = Dict{Tuple{String, String, Int}, JuMP.AffExpr}()
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (_, name, _, _) in entries, t in time_steps
            contents[(outage_id, name, t)] = zero(JuMP.AffExpr)
        end
    end
    expr_container = SparseAxisArray(contents)
    _assign_container!(container.expressions, ExpressionKey(T, V), expr_container)
    return expr_container
end

"""
Register an empty `SparseAxisArray` keyed by
`(outage_id::String, monitored_name::String, t::Int)` for the given constraint
type and meta tag. Entries are populated by `JuMP.@constraint` assignments
during the build. `V` here is the outaged component type the DeviceModel owns;
the container's name axis spans every monitored type associated with those
outages.
"""
function _add_post_contingency_sparse_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{V};
    meta::String,
) where {T <: ConstraintType, V <: PSY.ACTransmission}
    cons_container =
        SparseAxisArray(Dict{Tuple{String, String, Int}, JuMP.ConstraintRef}())
    _assign_container!(container.constraints, ConstraintKey(T, V, meta), cons_container)
    return cons_container
end

"""
For each outage in `device_model.outages`, resolve every monitored component
(across every monitored type) to its arc in the active reduction graph —
using `component_to_reduction_name_map` as a redirect when the monitored name
is an individual component that was reduced into a representative (e.g. `"3"`,
`"3_copy"` → `"3double_circuit"`). Duplicate arcs within an outage are
collapsed per-type. Outages sorted by UUID for deterministic axes.

Template validation is expected to guarantee that every monitored type has an
entry in the reduction maps and every monitored name resolves; missing entries
will raise `KeyError` here.

Returns `Vector{Pair{UUID, Vector{Tuple{Type, String, Tuple{Int, Int}, String}}}}`
where each inner tuple is `(monitored_type, container_name, arc, reduction_kind)`.
The `monitored_type` is needed by callers to fetch the correct post-contingency
flow variable for that component.
"""
function _resolve_monitored_arcs(
    device_model::DeviceModel,
    net_reduction_data::PNM.NetworkReductionData,
)
    name_to_arc_maps = PNM.get_name_to_arc_maps(net_reduction_data)
    c2r_maps = PNM.get_component_to_reduction_name_map(net_reduction_data)
    resolved =
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}}[]
    for (uuid, per_type) in get_outages(device_model)
        kept = Tuple{DataType, String, Tuple{Int, Int}, String}[]
        for (T, names) in per_type
            n2a = name_to_arc_maps[T]
            c2r = get(c2r_maps, T, Dict{String, String}())
            seen = Set{Tuple{Int, Int}}()
            for name in sort!(collect(names))
                container_name = haskey(n2a, name) ? name : c2r[name]
                arc, reduction_kind = n2a[container_name]
                arc in seen && continue
                push!(seen, arc)
                push!(kept, (T, container_name, arc, reduction_kind))
            end
        end
        push!(resolved, uuid => kept)
    end
    sort!(resolved; by = first)
    return resolved
end

"""
Add branch post-contingency rate limit constraints for ACBranch considering MODF and Security Constraints
"""
function add_constraints!(
    container::OptimizationContainer,
    cons_type::Type{T},
    device_model::DeviceModel{V, U},
    network_model::NetworkModel{X},
) where {
    T <: PostContingencyEmergencyFlowRateConstraint,
    V <: PSY.ACTransmission,
    U <: AbstractSecurityConstrainedStaticBranch,
    X <: PM.AbstractPowerModel,
}
    time_steps = get_time_steps(container)

    net_reduction_data = network_model.network_reduction
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    resolved = _resolve_monitored_arcs(device_model, net_reduction_data)

    con_lb = _add_post_contingency_sparse_constraints!(container, T, V; meta = "lb")
    con_ub = _add_post_contingency_sparse_constraints!(container, T, V; meta = "ub")

    expressions = get_expression(container, PostContingencyBranchFlow(), V)
    jump_model = get_jump_model(container)

    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (entry_type, name, arc, reduction_kind) in entries
            reduction_entry = all_branch_maps_by_type[reduction_kind][entry_type][arc]
            limits = get_emergency_min_max_limits(reduction_entry, T, U)
            for t in time_steps
                con_ub[outage_id, name, t] = JuMP.@constraint(
                    jump_model,
                    expressions[outage_id, name, t] <= limits.max,
                )
                con_lb[outage_id, name, t] = JuMP.@constraint(
                    jump_model,
                    expressions[outage_id, name, t] >= limits.min,
                )
            end
        end
    end
    return
end

function _add_post_contingency_flow_expressions_for_outage!(
    expression_container::SparseAxisArray,
    time_steps::UnitRange{Int},
    outage_id::String,
    modf_cols::Dict{Tuple{String, Tuple{Int64, Int64}}, Vector{Float64}},
    nodal_balance_expressions::Matrix{JuMP.AffExpr},
    entries::Vector{Tuple{DataType, String, Tuple{Int, Int}, String}},
)
    # Pure JuMP `AffExpr` build — no libklu in this hot path. Each task only
    # writes to keys (outage_id, name, t) for a single outage_id, so there is
    # no writer-writer aliasing across tasks.
    for (_, name, arc, _) in entries
        modf_col = modf_cols[(outage_id, arc)]
        _, expressions = _make_flow_expressions!(
            name,
            time_steps,
            modf_col,
            nodal_balance_expressions,
        )
        for t in time_steps
            expression_container[outage_id, name, t] = expressions[t]
        end
    end
    return
end

function add_post_contingency_flow_expressions!(
    container::OptimizationContainer,
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
    resolved = _resolve_monitored_arcs(model, net_reduction_data)

    expression_container = _add_post_contingency_sparse_expression!(
        container, T, V, resolved, time_steps,
    )

    nodal_balance_expressions =
        get_expression(container, ActivePowerBalance(), PSY.ACBus).data

    # Serial libklu pass: concurrent libklu calls are unsafe (PNM `_LIBKLU_LOCK`).
    # Each (outage, arc) pair is solved at most once.
    modf_cols = Dict{Tuple{String, Tuple{Int64, Int64}}, Vector{Float64}}()
    for (uuid, entries) in resolved
        outage_spec = registered_contingencies[uuid]
        outage_id = string(uuid)
        for (_, _, arc, _) in entries
            key = (outage_id, arc)
            haskey(modf_cols, key) && continue
            modf_cols[key] = modf_matrix[arc, outage_spec]
        end
    end

    # Parallel pass: pure JuMP `AffExpr` build per outage, no libklu.
    tasks = map(resolved) do (uuid, entries)
        outage_id = string(uuid)
        Threads.@spawn _add_post_contingency_flow_expressions_for_outage!(
            expression_container,
            time_steps,
            outage_id,
            modf_cols,
            nodal_balance_expressions,
            entries,
        )
    end
    foreach(wait, tasks)
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

    add_post_contingency_flow_expressions!(
        container,
        PostContingencyBranchFlow,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        PostContingencyEmergencyFlowRateConstraint,
        device_model,
        network_model,
    )

    return
end

# PTDF needs a PTDFBranchFlow expression here; the lossy AC path doesn't —
# its post-contingency expression is the FromTo flow variable directly.
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, F},
    network_model::NetworkModel{<:PM.AbstractACPModel},
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

    add_feedforward_arguments!(container, device_model, devices)
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
    X <: PM.AbstractACPModel,
}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container, FlowRateConstraintFromTo, devices, device_model, network_model,
    )
    add_constraints!(
        container, FlowRateConstraintToFrom, devices, device_model, network_model,
    )
    add_feedforward_constraints!(container, device_model, devices)
    objective_function!(container, devices, device_model, X)
    add_constraint_dual!(container, sys, device_model)

    add_post_contingency_flow_expressions!(
        container,
        PostContingencyBranchFlow,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        PostContingencyEmergencyFlowRateConstraint,
        device_model,
        network_model,
    )

    return
end

# Lossy AC post-contingency flow expression. Preventive formulation:
# `PostContingencyBranchFlow[outage, name, t]` is the from-to flow variable
# itself, so the per-outage emergency-rate constraint bounds the same variable
# for every outage by the emergency rating. Algebraically redundant across
# outages (same var, same bound) but keeps the (outage, name, t) container
# shape consistent with the PTDF path.
function add_post_contingency_flow_expressions!(
    container::OptimizationContainer,
    ::Type{T},
    model::DeviceModel{V, F},
    network_model::NetworkModel{N},
) where {
    T <: PostContingencyBranchFlow,
    V <: PSY.ACTransmission,
    F <: AbstractSecurityConstrainedStaticBranch,
    N <: PM.AbstractACPModel,
}
    time_steps = get_time_steps(container)
    resolved = _resolve_monitored_arcs(model, network_model.network_reduction)

    expression_container =
        SparseAxisArray(Dict{Tuple{String, String, Int}, JuMP.AffExpr}())
    _assign_container!(
        container.expressions, ExpressionKey(T, V), expression_container,
    )

    flow_vars_by_type = Dict{DataType, Any}()
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (entry_type, name, _, _) in entries
            flow_vars = get!(flow_vars_by_type, entry_type) do
                get_variable(container, FlowActivePowerFromToVariable(), entry_type)
            end
            for t in time_steps
                expression_container[outage_id, name, t] =
                    1.0 * flow_vars[name, t]
            end
        end
    end
    return
end
