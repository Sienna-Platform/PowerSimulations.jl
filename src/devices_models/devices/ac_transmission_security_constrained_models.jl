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
Outages registered on `modf_matrix` that monitor at least one component of
branch type `V`, paired with the sorted set of monitored branch names of that
type. Reads `device_model.outages`, which is populated during template
validation by `_build_device_model_outages!` from the user's explicit
selection (constructor kwarg) or — when no selection was given — by
auto-discovery over the system's `Outage` supplemental attributes (subject to
`"include_planned_outages"`).

Returned pairs are sorted by UUID so expression and constraint containers
have deterministic axes.
"""
function _post_contingency_outage_ids(
    modf_matrix::PNM.VirtualMODF,
    ::Type{V},
    device_model::DeviceModel,
) where {V <: PSY.ACTransmission}
    registered = PNM.get_registered_contingencies(modf_matrix)
    pairs = Pair{Base.UUID, Vector{String}}[]
    for (uuid, per_type) in get_outages(device_model)
        names = get(per_type, V, nothing)
        (names === nothing || isempty(names)) && continue
        if !haskey(registered, uuid)
            @warn "Outage $(uuid) is not registered on the MODF matrix; \
                   skipping its post-contingency constraints." maxlog = 1
            continue
        end
        push!(pairs, uuid => sort!(collect(names)))
    end
    sort!(pairs; by = first)
    return pairs
end

"""
Pre-allocate a `SparseAxisArray` keyed by `(outage_id::String, branch_name::String, t::Int)`
holding `JuMP.AffExpr` zeros for every monitored (outage, branch) pair × time
step. The container is registered on `container.expressions` under
`ExpressionKey(T, V)` and returned to the caller.
"""
function _add_post_contingency_sparse_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{V},
    monitored_pairs::Vector{Pair{Base.UUID, Vector{String}}},
    time_steps::UnitRange{Int},
) where {T <: PostContingencyExpressions, V <: PSY.ACTransmission}
    contents = Dict{Tuple{String, String, Int}, JuMP.AffExpr}()
    for (uuid, names) in monitored_pairs
        outage_id = string(uuid)
        for name in names
            for t in time_steps
                contents[(outage_id, name, t)] = zero(JuMP.AffExpr)
            end
        end
    end
    expr_container = SparseAxisArray(contents)
    expr_key = ExpressionKey(T, V)
    _assign_container!(container.expressions, expr_key, expr_container)
    return expr_container
end

"""
Pre-allocate a `SparseAxisArray` keyed by `(outage_id::String, branch_name::String, t::Int)`
for the given constraint type and meta tag, with each slot initialized to
`nothing` (filled in by `JuMP.@constraint` during the build).
"""
function _add_post_contingency_sparse_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{V},
    monitored_pairs::Vector{Pair{Base.UUID, Vector{String}}},
    time_steps::UnitRange{Int};
    meta::String,
) where {T <: ConstraintType, V <: PSY.ACTransmission}
    contents = Dict{Tuple{String, String, Int}, Union{Nothing, JuMP.ConstraintRef}}()
    for (uuid, names) in monitored_pairs
        outage_id = string(uuid)
        for name in names
            for t in time_steps
                contents[(outage_id, name, t)] = nothing
            end
        end
    end
    cons_container = SparseAxisArray(contents)
    cons_key = ConstraintKey(T, V, meta)
    _assign_container!(container.constraints, cons_key, cons_container)
    return cons_container
end

"""
Resolve each (outage, monitored-branch-name) pair to a unique arc, deduping
within an outage when multiple monitored names alias to the same arc (parallel
circuits). When a monitored name is the individual component (e.g. `"3"`,
`"3_copy"`) but the active reduction maps both to a single representative
(e.g. `"3double_circuit"`), the redirect is taken via
`component_to_reduction_name_map` so the container key is the reduction name —
matching what `name_to_arc_map` stores. Names with no resolution (component
absent from the reduction graph) are dropped silently.

Returns `Vector{Pair{UUID, Vector{Tuple{String, Tuple{Int, Int}, String}}}}`
where each inner tuple is `(container_name, arc, reduction_kind)`.
"""
function _resolve_monitored_arcs(
    monitored_pairs::Vector{Pair{Base.UUID, Vector{String}}},
    name_to_arc_map,
    component_to_reduction_name_map,
)
    resolved = Pair{Base.UUID, Vector{Tuple{String, Tuple{Int, Int}, String}}}[]
    for (uuid, names) in monitored_pairs
        seen = Set{Tuple{Int, Int}}()
        kept = Tuple{String, Tuple{Int, Int}, String}[]
        for name in names
            entry = get(name_to_arc_map, name, nothing)
            container_name = name
            if entry === nothing
                reduction_name = get(component_to_reduction_name_map, name, nothing)
                reduction_name === nothing && continue
                entry = get(name_to_arc_map, reduction_name, nothing)
                entry === nothing && continue
                container_name = reduction_name
            end
            arc = entry[1]
            arc in seen && continue
            push!(seen, arc)
            push!(kept, (container_name, arc, entry[2]))
        end
        isempty(kept) || push!(resolved, uuid => kept)
    end
    return resolved
end

"""
Add branch post-contingency rate limit constraints for ACBranch considering MODF and Security Constraints
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
    modf_matrix = get_MODF_matrix(network_model)

    monitored_pairs = _post_contingency_outage_ids(modf_matrix, V, device_model)
    isempty(monitored_pairs) && return

    net_reduction_data = network_model.network_reduction
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)
    name_to_arc_map = PNM.get_name_to_arc_map(net_reduction_data, V)
    component_to_reduction_name_map =
        get(
            PNM.get_component_to_reduction_name_map(net_reduction_data),
            V,
            Dict{String, String}(),
        )

    resolved = _resolve_monitored_arcs(
        monitored_pairs, name_to_arc_map, component_to_reduction_name_map,
    )
    isempty(resolved) && return

    name_only_pairs =
        [uuid => [t[1] for t in entries] for (uuid, entries) in resolved]
    con_lb = _add_post_contingency_sparse_constraints!(
        container, T, V, name_only_pairs, time_steps; meta = "lb",
    )
    con_ub = _add_post_contingency_sparse_constraints!(
        container, T, V, name_only_pairs, time_steps; meta = "ub",
    )

    expressions = get_expression(container, PostContingencyBranchFlow(), V)
    jump_model = get_jump_model(container)

    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (name, arc, reduction_kind) in entries
            reduction_entry = all_branch_maps_by_type[reduction_kind][V][arc]
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
    entries::Vector{Tuple{String, Tuple{Int, Int}, String}},
)
    # Pure JuMP `AffExpr` build — no libklu in this hot path. Each task only
    # writes to keys (outage_id, name, t) for a single outage_id, so there is
    # no writer-writer aliasing across tasks.
    for (name, arc, _) in entries
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

    monitored_pairs = _post_contingency_outage_ids(modf_matrix, V, model)
    isempty(monitored_pairs) && return

    net_reduction_data = network_model.network_reduction
    name_to_arc_map = PNM.get_name_to_arc_map(net_reduction_data, V)
    component_to_reduction_name_map =
        get(
            PNM.get_component_to_reduction_name_map(net_reduction_data),
            V,
            Dict{String, String}(),
        )
    resolved = _resolve_monitored_arcs(
        monitored_pairs, name_to_arc_map, component_to_reduction_name_map,
    )
    isempty(resolved) && return

    name_only_pairs =
        [uuid => [t[1] for t in entries] for (uuid, entries) in resolved]
    expression_container = _add_post_contingency_sparse_expression!(
        container, T, V, name_only_pairs, time_steps,
    )

    nodal_balance_expressions =
        get_expression(container, ActivePowerBalance(), PSY.ACBus).data

    # Serial libklu pass: concurrent libklu calls are unsafe (PNM `_LIBKLU_LOCK`).
    # Each (outage, arc) pair is solved at most once.
    modf_cols = Dict{Tuple{String, Tuple{Int64, Int64}}, Vector{Float64}}()
    for (uuid, entries) in resolved
        outage_spec = registered_contingencies[uuid]
        outage_id = string(uuid)
        for (_, arc, _) in entries
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
