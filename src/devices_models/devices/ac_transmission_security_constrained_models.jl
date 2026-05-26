# -----------------------------------------------------
# ------ RATING FUNCTIONS FOR EMERGENCY RATINGS -------
# -----------------------------------------------------
"""
Emergency Min and max limits for Abstract Branch Formulation and Post-Contingency conditions.
Covers both `PNM.BranchesParallel` (homogeneous) and `PNM.MixedBranchesParallel`
groups; PNM's `get_equivalent_emergency_rating` aggregates the per-circuit
emergency ratings as a sum-of-max, matching the max-flow capacity of the group.
"""
function get_emergency_min_max_limits(
    double_circuit::PNM.AbstractBranchesParallel,
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
Multi-component outage dedup: when an outage's attached components span more
than one type, every matching SC `DeviceModel` claims it; the first to run
populates the post-contingency expressions/constraints and the rest reuse
those AffExprs/ConstraintRefs by reference. Returns the source
`SparseAxisArray` so the caller indexes `.data[(outage_id, name, t)]` per
time step instead of re-scanning the container Dict per probe.
"""
# Dispatch (not an `isa` branch) skips non-sparse shared-arc containers: only
# the post-contingency `SparseAxisArray`s are keyed by (outage_id, name, t).
_post_contingency_match(c::SparseAxisArray, target::Tuple) = haskey(c.data, target)
_post_contingency_match(::AbstractArray, ::Tuple) = false

# A container is a reusable shared post-contingency source iff it has entry type
# `T`, belongs to a component type *other* than the outaged `V` (a same-`V`
# container is the one currently being built, not a source), and is keyed by
# `target`. Single predicate shared by the expression and constraint scans below.
function _is_shared_post_contingency_source(
    key::OptimizationContainerKey,
    c,
    target::Tuple,
    ::Type{T},
    ::Type{V},
) where {T, V <: PSY.ACTransmission}
    return get_entry_type(key) === T &&
           get_component_type(key) !== V &&
           _post_contingency_match(c, target)
end

# Names of components of type `D` monitored by at least one outage on this
# device model. `PostContingencyBranchRatingTimeSeriesParameter` is only
# meaningful for these (the components whose flows are bounded under
# contingencies), so the parameter is scoped to them at construction.
function _monitored_component_names(device_model::DeviceModel, ::Type{D}) where {D}
    names = Set{String}()
    for (_, per_type) in get_outages(device_model)
        for (mon_type, mon_names) in per_type
            mon_type <: D || continue
            union!(names, mon_names)
        end
    end
    return names
end

# True when a `PostContingencyBranchRatingTimeSeriesParameter` column exists
# for `name` under `entry_type` (i.e. that monitored component carries the
# series). A pure `Bool` predicate so the constraint builder dispatches on a
# boolean rather than a `nothing`-sentinel return; the caller separately gates
# on whether the parameter is configured at all (a loop-invariant check kept
# out of here).
function _has_post_contingency_rate(
    container::OptimizationContainer,
    entry_type::DataType,
    name::String,
)
    has_container_key(
        container,
        PostContingencyBranchRatingTimeSeriesParameter,
        entry_type,
    ) || return false
    param_container = get_parameter(
        container,
        PostContingencyBranchRatingTimeSeriesParameter(),
        entry_type,
    )
    return name in axes(get_multiplier_array(param_container))[1]
end

# The `(parameter column refs, multiplier slice)` for `name`'s post-contingency
# rate limit. Valid only when `_has_post_contingency_rate` is true; always
# returns a concrete tuple (never `nothing`), keeping the limit type stable.
function _post_contingency_rate_columns(
    container::OptimizationContainer,
    entry_type::DataType,
    name::String,
)
    param_container = get_parameter(
        container,
        PostContingencyBranchRatingTimeSeriesParameter(),
        entry_type,
    )
    return get_parameter_column_refs(param_container, name),
    get_multiplier_array(param_container)[name, :]
end

# Reactivated post-contingency branch-rating time series parameter. Scoped to
# the monitored components only: the parameter is exclusively consumed as the
# limit in the post-contingency flow inequalities, which exist only for
# monitored components, so building columns for unmonitored branches would be
# dead weight. The caller gates on the parameter being configured (mirroring
# the `BranchRatingTimeSeriesParameter` block) so the check stays visible.
function _add_post_contingency_branch_rating_parameter!(
    container::OptimizationContainer,
    device_model::DeviceModel{T},
    devices,
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ACTransmission}
    monitored = _monitored_component_names(device_model, T)
    monitored_devices = [d for d in devices if PSY.get_name(d) in monitored]
    isempty(monitored_devices) && return
    # Route through the reduction-aware builder so monitored columns are keyed
    # by the reduced container names and use the emergency multiplier, matching
    # what the post-contingency constraint builder looks up.
    add_branch_parameters!(
        container,
        PostContingencyBranchRatingTimeSeriesParameter,
        monitored_devices,
        device_model,
        network_model,
    )
    return
end

function _find_shared_post_contingency_expression_source(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{V},
    outage_id::String,
    name::String,
    t::Int,
) where {T <: PostContingencyExpressions, V <: PSY.ACTransmission}
    target = (outage_id, name, t)
    for (key, ec) in get_expressions(container)
        _is_shared_post_contingency_source(key, ec, target, T, V) && return ec
    end
    return
end

"""
Constraint counterpart to `_find_shared_post_contingency_expression_source`.
Returns `(lb_source, ub_source)` SparseAxisArrays in one scan; either slot
is `nothing` when no shared container of that meta exists.
"""
function _find_shared_post_contingency_constraint_sources(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{V},
    outage_id::String,
    name::String,
    t::Int,
) where {T <: PostContingencyConstraintType, V <: PSY.ACTransmission}
    target = (outage_id, name, t)
    src_lb = nothing
    src_ub = nothing
    for (key, cc) in get_constraints(container)
        _is_shared_post_contingency_source(key, cc, target, T, V) || continue
        if key.meta == "lb"
            src_lb = cc
        elseif key.meta == "ub"
            src_ub = cc
        end
        !isnothing(src_lb) && !isnothing(src_ub) && break
    end
    return src_lb, src_ub
end

"""
Locate the post-contingency slack variable containers that a shared constraint for
`(outage_id, name, t)` already references, registered by the source DeviceModel under
a component type other than `V`. Returns `(ub_source, lb_source)`; either slot is
`nothing` when the shared constraint was built without that slack (the source model
had `use_slacks = false`). Lets a reusing model alias those refs into its own slack
container so `get_variable`/`has_container_key` stay consistent with the constraints
it reuses, regardless of branch-model build order.
"""
function _find_shared_post_contingency_slack_sources(
    container::OptimizationContainer,
    ::Type{V},
    outage_id::String,
    name::String,
    t::Int,
) where {V <: PSY.ACTransmission}
    target = (outage_id, name, t)
    src_ub = nothing
    src_lb = nothing
    for (key, vc) in get_variables(container)
        get_component_type(key) === V && continue
        entry = get_entry_type(key)
        if entry === PostContingencyFlowActivePowerSlackUpperBound &&
           haskey(vc.data, target)
            src_ub = vc
        elseif entry === PostContingencyFlowActivePowerSlackLowerBound &&
               haskey(vc.data, target)
            src_lb = vc
        end
        !isnothing(src_ub) && !isnothing(src_lb) && break
    end
    return src_ub, src_lb
end

"""
Fast-path precheck: returns `true` iff any container of entry type `T` exists
under a component type other than `V`. The dedup probe scans `container.expressions`
or `container.constraints` linearly per call; on the common single-component-outage
path no other-V container exists yet and every probe returns nothing — so we
short-circuit the entire pre-pass when this returns `false`.
"""
function _has_other_v_container(
    container_dict,
    ::Type{T},
    ::Type{V},
) where {T, V <: PSY.ACTransmission}
    for key in keys(container_dict)
        get_entry_type(key) === T || continue
        get_component_type(key) === V && continue
        return true
    end
    return false
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
is an individual component that was reduced into a representative. Duplicate
arcs within an outage are collapsed per-type. Outages sorted by UUID for
deterministic axes.

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
    component_to_reduction_maps =
        PNM.get_component_to_reduction_name_map(net_reduction_data)
    resolved =
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}}[]
    for (uuid, per_type) in get_outages(device_model)
        kept = Tuple{DataType, String, Tuple{Int, Int}, String}[]
        for (T, names) in per_type
            # Per-type entries may include component types absent from the
            # network-reduction's branch-arc maps (e.g. `PSY.AreaInterchange`
            # under AreaBalance service-side outages). Skip those here so
            # device-side AC SC builders only see branch-arc types.
            haskey(name_to_arc_maps, T) || continue
            name_to_arc = name_to_arc_maps[T]
            component_to_reduction =
                get(component_to_reduction_maps, T, Dict{String, String}())
            seen = Set{Tuple{Int, Int}}()
            for name in sort!(collect(names))
                if haskey(name_to_arc, name)
                    container_name = name
                elseif haskey(component_to_reduction, name)
                    container_name = component_to_reduction[name]
                else
                    error(
                        "Monitored component \"$name\" (type $T) for outage $uuid is " *
                        "absent from both the network-reduction name-to-arc map and the " *
                        "component-to-reduction map. Verify the component exists in the " *
                        "system and is modeled with a security-constrained branch formulation.",
                    )
                end
                arc, reduction_kind = name_to_arc[container_name]
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

# Create a single per-`(outage_id, name, t)` post-contingency flow slack
# variable in `slack_container`, penalize it in the objective at
# `CONSTRAINT_VIOLATION_SLACK_COST`, and return the `VariableRef` so the caller
# can subtract/add it in the post-contingency inequality. Mirrors the
# pre-contingency `FlowActivePowerSlack{Upper,Lower}Bound` treatment in
# `AC_branches.jl`, but keyed per outage so each contingency relaxes
# independently. The penalty is added here (not in `objective_function!`)
# because these slacks are built alongside the constraints in the model stage,
# after the branch objective has already run.
function _make_post_contingency_slack!(
    container::OptimizationContainer,
    jump_model::JuMP.Model,
    slack_container::SparseAxisArray,
    ::Type{S},
    ::Type{V},
    outage_id::String,
    name::String,
    t::Int,
) where {S <: VariableType, V <: PSY.ACTransmission}
    slack = JuMP.@variable(
        jump_model,
        lower_bound = 0.0,
        base_name = "$(S)_$(V)_{$(outage_id), $(name), $(t)}",
    )
    slack_container[outage_id, name, t] = slack
    add_to_objective_invariant_expression!(
        container,
        slack * CONSTRAINT_VIOLATION_SLACK_COST,
    )
    return slack
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
    T <: PostContingencyFlowRateConstraint,
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

    # When `use_slacks` is set, each post-contingency inequality gets its own
    # slack variable so a branch overloaded under different outages relaxes
    # independently. Slacks are accumulated in local sparse containers and only
    # registered if non-empty (in a multi-component-outage reuse, every entry
    # can be shared from another model, leaving nothing to register here).
    use_slacks = get_use_slacks(device_model)
    slack_ub =
        SparseAxisArray(Dict{Tuple{String, String, Int}, JuMP.VariableRef}())
    slack_lb =
        SparseAxisArray(Dict{Tuple{String, String, Int}, JuMP.VariableRef}())

    # Multi-component outage dedup: if another SC DeviceModel already added
    # constraints for `(outage_id, name, t)`, reuse its `ConstraintRef`s
    # instead of issuing duplicate `@constraint`s. Precheck short-circuits
    # the per-entry probe on the single-component-outage hot path.
    has_other_v = _has_other_v_container(get_constraints(container), T, V)
    # Loop-invariant: whether the post-contingency rating time series is
    # configured at all. Checked once here rather than per (outage, entry).
    has_pc_rating = haskey(
        get_time_series_names(device_model),
        PostContingencyBranchRatingTimeSeriesParameter,
    )
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (entry_type, name, arc, reduction_kind) in entries
            if has_other_v
                src_lb, src_ub = _find_shared_post_contingency_constraint_sources(
                    container, T, V, outage_id, name, first(time_steps),
                )
                if !isnothing(src_lb) && !isnothing(src_ub)
                    # Reuse the shared constraint refs verbatim. The shared
                    # constraint already references the source model's slack (if
                    # any); alias those slack refs into this model's container so
                    # `get_variable`/`has_container_key` for `V` stay consistent
                    # with the constraints it reuses.
                    src_slack_ub = nothing
                    src_slack_lb = nothing
                    if use_slacks
                        src_slack_ub, src_slack_lb =
                            _find_shared_post_contingency_slack_sources(
                                container, V, outage_id, name, first(time_steps),
                            )
                    end
                    for t in time_steps
                        con_ub[outage_id, name, t] =
                            src_ub.data[(outage_id, name, t)]
                        con_lb[outage_id, name, t] =
                            src_lb.data[(outage_id, name, t)]
                        isnothing(src_slack_ub) || (
                            slack_ub[outage_id, name, t] =
                                src_slack_ub.data[(outage_id, name, t)]
                        )
                        isnothing(src_slack_lb) || (
                            slack_lb[outage_id, name, t] =
                                src_slack_lb.data[(outage_id, name, t)]
                        )
                    end
                    continue
                end
            end
            reduction_entry = all_branch_maps_by_type[reduction_kind][entry_type][arc]
            # The post-contingency branch-rating time series parameter, when
            # configured and available for this monitored component, replaces
            # the static emergency rating as the inequality limit; otherwise
            # the static emergency rating is used.
            if has_pc_rating && _has_post_contingency_rate(container, entry_type, name)
                param, multiplier =
                    _post_contingency_rate_columns(container, entry_type, name)
                for t in time_steps
                    sub = if use_slacks
                        _make_post_contingency_slack!(
                            container, jump_model, slack_ub,
                            PostContingencyFlowActivePowerSlackUpperBound, V,
                            outage_id, name, t,
                        )
                    else
                        0.0
                    end
                    slb = if use_slacks
                        _make_post_contingency_slack!(
                            container, jump_model, slack_lb,
                            PostContingencyFlowActivePowerSlackLowerBound, V,
                            outage_id, name, t,
                        )
                    else
                        0.0
                    end
                    con_ub[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        expressions[outage_id, name, t] - sub <=
                        param[t] * multiplier[t],
                    )
                    con_lb[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        expressions[outage_id, name, t] + slb >=
                        -1.0 * param[t] * multiplier[t],
                    )
                end
            else
                limits = get_emergency_min_max_limits(reduction_entry, T, U)
                for t in time_steps
                    sub = if use_slacks
                        _make_post_contingency_slack!(
                            container, jump_model, slack_ub,
                            PostContingencyFlowActivePowerSlackUpperBound, V,
                            outage_id, name, t,
                        )
                    else
                        0.0
                    end
                    slb = if use_slacks
                        _make_post_contingency_slack!(
                            container, jump_model, slack_lb,
                            PostContingencyFlowActivePowerSlackLowerBound, V,
                            outage_id, name, t,
                        )
                    else
                        0.0
                    end
                    con_ub[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        expressions[outage_id, name, t] - sub <= limits.max,
                    )
                    con_lb[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        expressions[outage_id, name, t] + slb >= limits.min,
                    )
                end
            end
        end
    end

    if use_slacks
        isempty(slack_ub.data) || _assign_container!(
            container.variables,
            VariableKey(PostContingencyFlowActivePowerSlackUpperBound, V),
            slack_ub,
        )
        isempty(slack_lb.data) || _assign_container!(
            container.variables,
            VariableKey(PostContingencyFlowActivePowerSlackLowerBound, V),
            slack_lb,
        )
    end
    return
end

function _build_post_contingency_flow_expressions_for_outage(
    time_steps::UnitRange{Int},
    outage_id::String,
    modf_cols::Dict{Tuple{String, Tuple{Int64, Int64}}, Vector{Float64}},
    nodal_balance_expressions::Matrix{JuMP.AffExpr},
    entries::Vector{Tuple{DataType, String, Tuple{Int, Int}, String}},
)
    results = Vector{Tuple{String, Vector{JuMP.AffExpr}}}(undef, length(entries))
    for (i, entry) in enumerate(entries)
        (_, name, arc, _) = entry
        modf_col = modf_cols[(outage_id, arc)]
        _, expressions = _make_flow_expressions!(
            name,
            time_steps,
            modf_col,
            nodal_balance_expressions,
        )
        results[i] = (name, expressions)
    end
    return results
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

    # Multi-component outage dedup: an outage attached to N>1 component types
    # is claimed by N SC DeviceModels (see `_build_device_model_outages!`).
    # The first DeviceModel to run populates the entries; subsequent ones
    # reference those AffExprs instead of recomputing. Splits `resolved`
    # into entries already populated (copied here) and entries needing
    # fresh computation downstream.
    fresh_resolved = _copy_existing_post_contingency_expressions!(
        container, T, V, expression_container, resolved, time_steps,
    )
    isempty(fresh_resolved) && return

    nodal_balance_expressions =
        get_expression(container, ActivePowerBalance(), PSY.ACBus).data

    # Serial libklu pass: concurrent libklu calls are unsafe (PNM `_LIBKLU_LOCK`).
    # Each (outage, arc) pair is solved at most once.
    modf_cols = Dict{Tuple{String, Tuple{Int64, Int64}}, Vector{Float64}}()
    for (uuid, entries) in fresh_resolved
        outage_spec = registered_contingencies[uuid]
        outage_id = string(uuid)
        for (_, _, arc, _) in entries
            key = (outage_id, arc)
            haskey(modf_cols, key) && continue
            modf_cols[key] = modf_matrix[arc, outage_spec]
        end
    end

    # Parallel JuMP `AffExpr` build (no libklu): tasks return results, the main
    # thread does the serial writes, so we never rely on SparseAxisArray's Dict
    # being safe under concurrent setindex!. The try/catch surfaces the inner
    # exception, since `build!` otherwise shows only the wrapping
    # `TaskFailedException`. Mirrors the PTDF pattern in `AC_branches.jl`.
    tasks = map(fresh_resolved) do (uuid, entries)
        outage_id = string(uuid)
        Threads.@spawn try
            _build_post_contingency_flow_expressions_for_outage(
                time_steps,
                outage_id,
                modf_cols,
                nodal_balance_expressions,
                entries,
            )
        catch e
            @error "Post-contingency flow-expression task failed" outage_id =
                outage_id exception = (e, catch_backtrace())
            rethrow()
        end
    end
    for (i, task) in enumerate(tasks)
        (uuid, _) = fresh_resolved[i]
        outage_id = string(uuid)
        for (name, expressions) in fetch(task)
            for t in time_steps
                expression_container[outage_id, name, t] = expressions[t]
            end
        end
    end
    return
end

"""
Pre-pass for the multi-component outage dedup: copy already-built entries
into `expression_container` and return the residual `resolved` shape that
still needs fresh computation. Returns `resolved` unchanged when no other-V
container exists (the single-component-outage hot path).
"""
function _copy_existing_post_contingency_expressions!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{V},
    expression_container::SparseAxisArray,
    resolved::Vector{
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}},
    },
    time_steps::UnitRange{Int},
) where {T <: PostContingencyExpressions, V <: PSY.ACTransmission}
    _has_other_v_container(get_expressions(container), T, V) || return resolved

    fresh =
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}}[]
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        unresolved = Tuple{DataType, String, Tuple{Int, Int}, String}[]
        for entry in entries
            (_, name, _, _) = entry
            src_ec = _find_shared_post_contingency_expression_source(
                container, T, V, outage_id, name, first(time_steps),
            )
            if isnothing(src_ec)
                push!(unresolved, entry)
            else
                for t in time_steps
                    expression_container[outage_id, name, t] =
                        src_ec.data[(outage_id, name, t)]
                end
            end
        end
        isempty(unresolved) || push!(fresh, uuid => unresolved)
    end
    return fresh
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

    if haskey(get_time_series_names(device_model), BranchRatingTimeSeriesParameter)
        add_branch_parameters!(
            container,
            BranchRatingTimeSeriesParameter,
            devices,
            device_model,
            network_model,
        )
    end

    if haskey(
        get_time_series_names(device_model),
        PostContingencyBranchRatingTimeSeriesParameter,
    )
        _add_post_contingency_branch_rating_parameter!(
            container,
            device_model,
            devices,
            network_model,
        )
    end

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

    add_post_contingency_flow_expressions!(
        container,
        PostContingencyBranchFlow,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        PostContingencyFlowRateConstraint,
        device_model,
        network_model,
    )

    # Must run after the post-contingency constraints are built so their
    # SparseAxisArray dual containers are registered alongside FlowRateConstraint.
    add_constraint_dual!(container, sys, device_model)

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

    if haskey(get_time_series_names(device_model), BranchRatingTimeSeriesParameter)
        add_branch_parameters!(
            container,
            BranchRatingTimeSeriesParameter,
            devices,
            device_model,
            network_model,
        )
    end

    if haskey(
        get_time_series_names(device_model),
        PostContingencyBranchRatingTimeSeriesParameter,
    )
        _add_post_contingency_branch_rating_parameter!(
            container,
            device_model,
            devices,
            network_model,
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

    add_post_contingency_flow_expressions!(
        container,
        PostContingencyBranchFlow,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        PostContingencyFlowRateConstraint,
        device_model,
        network_model,
    )

    # Must run after the post-contingency constraints are built so their
    # SparseAxisArray dual containers are registered alongside FlowRateConstraint.
    add_constraint_dual!(container, sys, device_model)

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

    has_other_v = _has_other_v_container(get_expressions(container), T, V)
    flow_vars_by_type = Dict{DataType, Any}()
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (entry_type, name, _, _) in entries
            if has_other_v
                src_ec = _find_shared_post_contingency_expression_source(
                    container, T, V, outage_id, name, first(time_steps),
                )
                if !isnothing(src_ec)
                    for t in time_steps
                        expression_container[outage_id, name, t] =
                            src_ec.data[(outage_id, name, t)]
                    end
                    continue
                end
            end
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

# `SecurityConstrainedStaticBranch` is intentionally inert under network models
# that carry no branch-flow representation: NFA, CopperPlate and AreaBalance
# build nothing rather than erroring (mirrors the StaticBranch no-ops). Defined
# on concrete network types to avoid ambiguity with the PTDF/ACP methods.
function construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ArgumentConstructStage,
    ::DeviceModel{T, F},
    ::Union{
        NetworkModel{NFAPowerModel},
        NetworkModel{CopperPlatePowerModel},
        NetworkModel{AreaBalancePowerModel},
    },
) where {T <: PSY.ACTransmission, F <: AbstractSecurityConstrainedStaticBranch}
    @debug "No argument construction for $F under NFA/CopperPlate/AreaBalance; \
            security-constrained branch limits are inert for these network \
            models." _group = LOG_GROUP_BRANCH_CONSTRUCTIONS
    return
end

function construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ModelConstructStage,
    ::DeviceModel{T, F},
    ::Union{
        NetworkModel{NFAPowerModel},
        NetworkModel{CopperPlatePowerModel},
        NetworkModel{AreaBalancePowerModel},
    },
) where {T <: PSY.ACTransmission, F <: AbstractSecurityConstrainedStaticBranch}
    @debug "No model construction for $F under NFA/CopperPlate/AreaBalance; \
            security-constrained branch limits are inert for these network \
            models." _group = LOG_GROUP_BRANCH_CONSTRUCTIONS
    return
end
