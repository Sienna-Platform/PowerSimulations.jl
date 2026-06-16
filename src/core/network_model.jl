const DeviceModelForBranches = DeviceModel{<:PSY.Branch, <:AbstractDeviceFormulation}
const BranchModelContainer = Dict{Symbol, DeviceModelForBranches}

function _check_pm_formulation(::Type{T}) where {T <: PM.AbstractPowerModel}
    if !isconcretetype(T)
        throw(
            ArgumentError(
                "The network model must contain only concrete types, $(T) is an Abstract Type",
            ),
        )
    end
end

_maybe_flatten_pfem(pfem::Vector{PFS.PowerFlowEvaluationModel}) = pfem
_maybe_flatten_pfem(pfem::PFS.PowerFlowEvaluationModel) =
    PFS.flatten_power_flow_evaluation_model(pfem)

"""
Establishes the NetworkModel for a given PowerModels formulation type.

# Arguments
- `::Type{T}` where `T <: PM.AbstractPowerModel`: the power-system formulation type.

# Accepted keyword arguments
- `use_slacks::Bool` = false
    Adds slack buses to the network modeling.
- `PTDF_matrix::Union{PNM.PowerNetworkMatrix, Nothing}` = nothing
    PTDF/VirtualPTDF matrix produced by PowerNetworkMatrices (optional).
- `MODF_matrix::Union{PNM.VirtualMODF, Nothing}` = nothing
    VirtualMODF matrix for security-constrained models (N-k contingencies).
    If `nothing` and the template includes a `SecurityConstrainedStaticBranch`
    formulation, the matrix is constructed from the system during
    `instantiate_network_model!` (same pattern as PTDF).
- `reduce_radial_branches::Bool` = false
    Enable radial branch reduction when building network matrices.
- `reduce_degree_two_branches::Bool` = false
    Enable degree-two branch reduction when building network matrices.
- `duals::Vector{DataType}` = Vector{DataType}()
    Constraint types for which duals should be recorded.
- `power_flow_evaluation::Union{PFS.PowerFlowEvaluationModel, Vector{PFS.PowerFlowEvaluationModel}}`
    Power-flow evaluation model(s). A single model is flattened to a vector internally.

# Notes
- `modeled_ac_branch_types` and `reduced_branch_tracker` are internal fields managed by the model.
- `subsystem` can be set after construction via `set_subsystem!(model, id)`.
- PTDF inputs are validated against the requested reduction flags and may raise
  a ConflictingInputsError if they are inconsistent with `reduce_radial_branches`
  or `reduce_degree_two_branches`.

# Examples
ptdf = PNM.VirtualPTDF(system)
nw = NetworkModel(PTDFPowerModel; PTDF_matrix = ptdf, reduce_radial_branches = true,
                  power_flow_evaluation = PFS.PowerFlowEvaluationModel())
"""
mutable struct NetworkModel{T <: PM.AbstractPowerModel}
    use_slacks::Bool
    PTDF_matrix::Union{Nothing, PNM.PowerNetworkMatrix}
    MODF_matrix::Union{Nothing, PNM.VirtualMODF}
    subnetworks::Dict{Int, Set{Int}}
    bus_area_map::Dict{PSY.ACBus, Int}
    duals::Vector{DataType}
    network_reduction::PNM.NetworkReductionData
    reduce_radial_branches::Bool
    reduce_degree_two_branches::Bool
    power_flow_evaluation::Vector{PFS.PowerFlowEvaluationModel}
    subsystem::Union{Nothing, String}
    hvdc_network_model::Union{Nothing, AbstractHVDCNetworkModel}
    modeled_ac_branch_types::Vector{DataType}
    reduced_branch_tracker::BranchReductionOptimizationTracker

    function NetworkModel(
        ::Type{T};
        use_slacks = false,
        PTDF_matrix = nothing,
        MODF_matrix = nothing,
        reduce_radial_branches = false,
        reduce_degree_two_branches = false,
        duals = Vector{DataType}(),
        power_flow_evaluation::Union{
            PFS.PowerFlowEvaluationModel,
            Vector{PFS.PowerFlowEvaluationModel},
        } = PFS.PowerFlowEvaluationModel[],
        hvdc_network_model = nothing,
    ) where {T <: PM.AbstractPowerModel}
        _check_pm_formulation(T)
        new{T}(
            use_slacks,
            PTDF_matrix,
            MODF_matrix,
            Dict{Int, Set{Int}}(),
            Dict{PSY.ACBus, Int}(),
            duals,
            PNM.NetworkReductionData(),
            reduce_radial_branches,
            reduce_degree_two_branches,
            _maybe_flatten_pfem(power_flow_evaluation),
            nothing,
            hvdc_network_model,
            Vector{DataType}(),
            BranchReductionOptimizationTracker(),
        )
    end
end

get_use_slacks(m::NetworkModel) = m.use_slacks
get_PTDF_matrix(m::NetworkModel) = m.PTDF_matrix
get_MODF_matrix(m::NetworkModel) = m.MODF_matrix
get_reduce_radial_branches(m::NetworkModel) = m.reduce_radial_branches
get_network_reduction(m::NetworkModel) = m.network_reduction
get_duals(m::NetworkModel) = m.duals
get_network_formulation(::NetworkModel{T}) where {T} = T
get_reduced_branch_tracker(m::NetworkModel) = m.reduced_branch_tracker
get_reference_buses(m::NetworkModel{T}) where {T <: PM.AbstractPowerModel} =
    collect(keys(m.subnetworks))
get_subnetworks(m::NetworkModel) = m.subnetworks
get_bus_area_map(m::NetworkModel) = m.bus_area_map
get_power_flow_evaluation(m::NetworkModel) = m.power_flow_evaluation
has_subnetworks(m::NetworkModel) = !isempty(m.bus_area_map)
get_subsystem(m::NetworkModel) = m.subsystem
get_hvdc_network_model(m::NetworkModel) = m.hvdc_network_model

set_subsystem!(m::NetworkModel, id::String) = m.subsystem = id
set_hvdc_network_model!(m::NetworkModel, val::Union{Nothing, AbstractHVDCNetworkModel}) =
    m.hvdc_network_model = val

function add_dual!(model::NetworkModel, dual)
    dual in model.duals && error("dual = $dual is already stored")
    push!(model.duals, dual)
    @debug "Added dual" dual _group = LOG_GROUP_NETWORK_CONSTRUCTION
    return
end

function _template_uses_security_constrained_branch(branch_models::BranchModelContainer)
    for v in values(branch_models)
        if get_formulation(v) <: AbstractSecurityConstrainedStaticBranch
            return true
        end
    end
    return false
end

"""
Drop outages from each SC-branch `DeviceModel` whose UUID isn't registered on
`modf_matrix`; without this they'd `KeyError` downstream in
`add_post_contingency_flow_expressions!`. PNM's `_register_outages!` silently
skips outages it can't convert to a `NetworkModification`.
"""
function _consolidate_device_model_outages_with_modf!(
    branch_models::BranchModelContainer,
    modf_matrix::PNM.VirtualMODF,
)
    registered = PNM.get_registered_contingencies(modf_matrix)
    for m in values(branch_models)
        get_formulation(m) <: AbstractSecurityConstrainedStaticBranch || continue
        for uuid in setdiff(keys(m.outages), keys(registered))
            @warn "Outage $(uuid) (DeviceModel{$(get_component_type(m)), \
                   $(get_formulation(m))}) is not registered on the MODF \
                   matrix and will not contribute any post-contingency \
                   constraints." _group = LOG_GROUP_MODELS_VALIDATION
            delete!(m.outages, uuid)
        end
    end
    return
end

"""
Build the `NetworkReduction` specs for one matrix construction. As of
PowerNetworkMatrices 0.22 the reduction specs are field-less configuration
objects; the set of buses to protect from elimination is supplied separately via
the `irreducible_buses` keyword on the matrix constructor (`Ybus` / `VirtualPTDF`
/ `VirtualMODF`), which seeds it once into the reduction container and shares it
across all reduction stages.
"""
function _build_network_reductions(model::NetworkModel)
    reductions = PNM.NetworkReduction[]
    if model.reduce_radial_branches
        push!(reductions, PNM.RadialReduction())
    end
    if model.reduce_degree_two_branches
        push!(reductions, PNM.DegreeTwoReduction())
    end
    return reductions
end

"""
Buses retained by `nrd` — the keys of the bus reduction map, which PNM
initializes with every available non-isolated bus and prunes as buses are
eliminated (`Ybus` build). This key set is therefore exactly the matrix's bus
dimension.

This is *not* the same as `get_irreducible_buses(nrd)`: that field accumulates
(via `union!`) every bus each reduction stage flagged as protected — including
all injection buses added by a `DegreeTwoReduction`. A protected injection bus
that sits on a radial leaf can still be eliminated by a `RadialReduction` stage,
so `irreducible_buses` is not a subset of the retained keys and must not be
treated as one.
"""
function _retained_buses(nrd::PNM.NetworkReductionData)
    return Set(keys(PNM.get_bus_reduction_map(nrd)))
end

"""
True when two network matrices carry the same reduction, compared by full
`bus_reduction_map` equality (keys *and* values). This is the invariant the
post-contingency builder relies on: it resolves monitored arcs from the reduced
bus/arc numbering and indexes `modf[arc, outage_spec]`, so the PTDF (which
dimensions the nodal balance) and the MODF must agree on the entire map, not just
the retained-bus key set.
"""
function _reductions_match(a::PNM.PowerNetworkMatrix, b::PNM.VirtualMODF)
    return PNM.get_bus_reduction_map(PNM.get_network_reduction_data(a)) ==
           PNM.get_bus_reduction_map(PNM.get_network_reduction_data(b))
end

function _get_filters(branch_models::BranchModelContainer)
    filters = Dict{DataType, Function}()
    for v in values(branch_models)
        filter_func = get_attribute(v, "filter_function")
        if !isnothing(filter_func)
            filters[get_component_type(v)] = filter_func
        end
    end
    return filters
end

"""
Buses that must be preserved through PNM reductions because something pinned to
them is monitored by the simulation. Sources of pinning:

- Branches carrying a `BranchRatingTimeSeriesParameter` — both endpoint buses
  are pinned.
- Outages registered on an SC-formulated branch `DeviceModel` — their
  `monitored_components` and `associated_components` pin buses (for branch
  components both endpoints, for non-branch devices the connecting bus).
  Outages on non-SC devices are not consumed and do not pin anything.

The result is unioned and consumed by `_build_network_reductions` and the
matrix constructors in `instantiate_network_model!`.
"""
function _get_irreducible_buses_due_to_monitored_components(
    sys::PSY.System,
    network_model::NetworkModel,
    branch_models::BranchModelContainer,
)
    @debug "Identifying buses that are irreducible due to monitored components"
    irreducible_buses = Set{Int64}()
    _add_timeseries_irreducible_buses!(irreducible_buses, sys, network_model, branch_models)
    # Outage-monitored components only matter when an SC formulation actually
    # consumes them. The scoping is per-DeviceModel: only outages registered on
    # an SC-formulated branch model pin buses. A system Outage attached solely
    # to a device whose formulation is not security constrained (e.g. a
    # transformer modeled with a non-SC formulation) is ignored, so those
    # branches stay reducible unless monitored by an SC outage.
    _add_outage_monitored_irreducible_buses!(irreducible_buses, sys, branch_models)
    # `model_all_branches` MonitoredLine models pin their lines so zero-impedance
    # ones survive the reduction instead of being merged away.
    _add_model_all_branches_irreducible_buses!(irreducible_buses, branch_models)
    return collect(irreducible_buses)
end

# Pin both endpoint buses of every branch a `model_all_branches` MonitoredLine model
# covers. Dispatch on the model type so it is a no-op for other branch types.
function _add_model_all_branches_irreducible_buses!(
    irreducible_buses::Set{Int64},
    branch_models::BranchModelContainer,
)
    for m in values(branch_models)
        _pin_model_all_branches!(irreducible_buses, m)
    end
    return
end

_pin_model_all_branches!(::Set{Int64}, ::DeviceModel) = nothing

function _pin_model_all_branches!(
    irreducible_buses::Set{Int64},
    m::DeviceModel{PSY.MonitoredLine},
)
    get_attribute(m, MODEL_ALL_BRANCHES_KEY) === true || return
    # The device cache is the modeled set (available + filter_function).
    for branch in get_device_cache(m)
        _push_component_buses!(irreducible_buses, branch)
    end
    return
end

function _add_timeseries_irreducible_buses!(
    irreducible_buses::Set{Int64},
    sys::PSY.System,
    network_model::NetworkModel,
    branch_models::BranchModelContainer,
)
    param_types = Type{<:TimeSeriesParameter}[BranchRatingTimeSeriesParameter]
    # Post-contingency branch ratings only pin buses when an SC formulation
    # actually consumes them. Otherwise a stray PostContingency rating
    # attribute would force a provided PTDF to be discarded and recomputed
    # for every non-SC build.
    if _template_uses_security_constrained_branch(branch_models)
        push!(param_types, PostContingencyBranchRatingTimeSeriesParameter)
    end
    # Resolve the forecast type only when a branch rating time series is
    # actually configured: `get_deterministic_time_series_type` throws for
    # systems without forecast data (e.g. single-step emulation models), and
    # nothing pins buses without a configured rating parameter anyway.
    any(
        pt -> _any_branch_model_configures_param(network_model, branch_models, pt),
        param_types,
    ) || return
    ts_type = get_deterministic_time_series_type(sys)
    for pt in param_types
        _add_rating_timeseries_irreducible_buses!(
            irreducible_buses,
            network_model,
            branch_models,
            pt,
            ts_type,
        )
    end
    return
end

function _any_branch_model_configures_param(
    network_model::NetworkModel,
    branch_models::BranchModelContainer,
    param_type::Type{<:TimeSeriesParameter},
)
    for branch_type in network_model.modeled_ac_branch_types
        device_model = branch_models[nameof(branch_type)]
        haskey(get_time_series_names(device_model), param_type) && return true
    end
    return false
end

function _add_rating_timeseries_irreducible_buses!(
    irreducible_buses::Set{Int64},
    network_model::NetworkModel,
    branch_models::BranchModelContainer,
    param_type::Type{<:TimeSeriesParameter},
    ts_type::Type{<:PSY.AbstractDeterministic},
)
    for branch_type in network_model.modeled_ac_branch_types
        device_model = branch_models[nameof(branch_type)]
        # No rating time series configured for this branch model: nothing pins
        # buses. Same gate as `_any_component_has_branch_rating_ts`.
        haskey(get_time_series_names(device_model), param_type) || continue
        ts_name = get_time_series_names(device_model)[param_type]

        if branch_type <: PSY.ThreeWindingTransformer
            @warn "Branch rating time series for ThreeWindingTransformers are not implemented yet. Skipping it."
            continue
        end

        # Reuse the device cache built during template validation: it is the
        # exact set of modeled components (available + filter_function), so a
        # branch excluded by the device model never pins buses and we avoid a
        # second full PSY component query. The `has_time_series(_, ts_type,
        # ts_name)` test is the same modeled-ts definition validation uses.
        for branch in get_device_cache(device_model)
            PSY.has_time_series(branch, ts_type, ts_name) || continue
            _push_component_buses!(irreducible_buses, branch)
        end
    end
    return
end

function _add_outage_monitored_irreducible_buses!(
    irreducible_buses::Set{Int64},
    sys::PSY.System,
    branch_models::BranchModelContainer,
)
    # Only outages registered on an SC-formulated branch DeviceModel are
    # consumed by post-contingency constraints. `DeviceModel.outages` is
    # populated exclusively when `_formulation_supports_outages` is true, so
    # iterating these keys naturally excludes outages on non-SC devices
    # without traversing every Outage in the system.
    outage_uuids = Set{Base.UUID}()
    for m in values(branch_models)
        get_formulation(m) <: AbstractSecurityConstrainedStaticBranch || continue
        union!(outage_uuids, keys(get_outages(m)))
    end

    for outage_uuid in outage_uuids
        outage = PSY.get_supplemental_attribute(sys, outage_uuid)
        # Monitored-component buses must remain visible so post-contingency
        # flow constraints reference real bus numbers.
        for uuid in PSY.get_monitored_components(outage)
            component = IS.get_component(sys, uuid)
            if isnothing(component)
                throw(
                    IS.ConflictingInputsError(
                        "Monitored component with UUID $(uuid) on outage $(IS.get_uuid(outage)) not found in system. Data requires correction",
                    ),
                )
            end
            _push_component_buses!(irreducible_buses, component)
        end
        # Outaged-component buses must also remain visible: PNM's MODF column
        # for the contingency is keyed by the outaged arc's endpoints. If the
        # network reduction collapses those buses (e.g., a degree-two reduction
        # between them), the contingency arc no longer exists as a discrete
        # element in the reduced topology and the MODF column would be missing
        # or apply to the wrong arc.
        for component in PSY.get_associated_components(sys, outage)
            _push_component_buses!(irreducible_buses, component)
        end
    end
    return
end

function _push_component_buses!(buses::Set{Int64}, branch::PSY.Branch)
    arc = PSY.get_arc(branch)
    push!(buses, PSY.get_number(PSY.get_from(arc)))
    push!(buses, PSY.get_number(PSY.get_to(arc)))
    return
end

function _push_component_buses!(buses::Set{Int64}, branch::PSY.ThreeWindingTransformer)
    for arc in (
        PSY.get_primary_star_arc(branch),
        PSY.get_secondary_star_arc(branch),
        PSY.get_tertiary_star_arc(branch),
    )
        push!(buses, PSY.get_number(PSY.get_from(arc)))
        push!(buses, PSY.get_number(PSY.get_to(arc)))
    end
    return
end

function _push_component_buses!(buses::Set{Int64}, device::PSY.StaticInjection)
    push!(buses, PSY.get_number(PSY.get_bus(device)))
    return
end

# Drop (and warn about) any branch type whose components were all merged away by the
# reduction — e.g. a lone zero-impedance monitored line. Such a type has no surviving
# arc in `name_to_arc_map`, so building its flow vars/constraints would fail. Absence
# from the map alone is not enough: types that never use it (e.g. HVDC) are also
# absent, so we prune only when an endpoint bus was actually removed by the reduction.
# Uncommon; `model_all_branches` keeps such lines instead.
function _prune_fully_reduced_branch_models!(
    network_model::NetworkModel,
    branch_models::BranchModelContainer,
)
    merged_buses = Set{Int64}()
    for removed in values(PNM.get_bus_reduction_map(network_model.network_reduction))
        union!(merged_buses, removed)
    end
    isempty(merged_buses) && return
    name_to_arc_maps = PNM.get_name_to_arc_maps(network_model.network_reduction)
    pruned = DataType[]
    for branch_type in network_model.modeled_ac_branch_types
        survived = get(name_to_arc_maps, branch_type, nothing)
        isnothing(survived) || isempty(survived) || continue
        buses = Set{Int64}()
        for component in get_device_cache(branch_models[nameof(branch_type)])
            _push_component_buses!(buses, component)
        end
        isdisjoint(buses, merged_buses) && continue
        push!(pruned, branch_type)
    end
    for branch_type in pruned
        @warn "All components of branch type $(branch_type) were merged away by the " *
              "network reduction (e.g. a zero-impedance branch merge). The " *
              "$(branch_type) DeviceModel is dropped from the template and will not " *
              "be modeled. Use the `model_all_branches` attribute on a MonitoredLine " *
              "model to retain such branches through the reduction."
        delete!(branch_models, nameof(branch_type))
        filter!(!=(branch_type), network_model.modeled_ac_branch_types)
    end
    return
end

# Warn about individual monitored lines the reduction merged away while their type
# still has surviving members. The whole-type prune above misses this partial case,
# so without a message the dropped line is silently unmodeled. Suggest
# `model_all_branches` to retain it.
function _warn_partially_reduced_monitored_lines!(
    network_model::NetworkModel,
    branch_models::BranchModelContainer,
)
    removed_arcs = PNM.get_removed_arcs(network_model.network_reduction)
    isempty(removed_arcs) && return
    for m in values(branch_models)
        _warn_reduced_monitored_lines!(removed_arcs, m)
    end
    return
end

_warn_reduced_monitored_lines!(removed_arcs::Set{Tuple{Int, Int}}, ::DeviceModel) = nothing

function _warn_reduced_monitored_lines!(removed_arcs::Set{Tuple{Int, Int}}, m::DeviceModel{PSY.MonitoredLine})
    dropped = [
        PSY.get_name(ml) for ml in get_device_cache(m) if
        _branch_arc_removed(ml, removed_arcs)
    ]
    isempty(dropped) && return
    @warn "MonitoredLine(s) $(dropped) were merged away by the network reduction " *
          "(near-zero impedance) and will not be modeled or monitored, though other " *
          "MonitoredLines remain. Set the `model_all_branches` attribute on the " *
          "MonitoredLine DeviceModel to force all monitored lines to be modeled " *
          "through the reduction."
    return
end

function _branch_arc_removed(branch::PSY.Branch, removed_arcs)
    arc = PSY.get_arc(branch)
    from = PSY.get_number(PSY.get_from(arc))
    to = PSY.get_number(PSY.get_to(arc))
    return (from, to) in removed_arcs || (to, from) in removed_arcs
end

function instantiate_network_model!(
    model::NetworkModel{T},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
) where {T <: PM.AbstractPowerModel}
    irreducible_buses =
        _get_irreducible_buses_due_to_monitored_components(sys, model, branch_models)
    if model.reduce_radial_branches && model.reduce_degree_two_branches
        @info "Applying both radial and degree two reductions"
        ybus = PNM.Ybus(
            sys;
            irreducible_buses = irreducible_buses,
            network_reductions = PNM.NetworkReduction[
                PNM.RadialReduction(),
                PNM.DegreeTwoReduction(),
            ],
        )
    elseif model.reduce_radial_branches
        @info "Applying radial reduction"
        if !isempty(irreducible_buses)
            @warn "Irreducible buses identified from monitored components. The reduction of any radial branch between 2 irreducible buses will be ignored"
        end
        ybus = PNM.Ybus(
            sys;
            irreducible_buses = irreducible_buses,
            network_reductions = PNM.NetworkReduction[PNM.RadialReduction()],
        )
    elseif model.reduce_degree_two_branches
        @info "Applying degree two reduction"
        ybus = PNM.Ybus(
            sys;
            irreducible_buses = irreducible_buses,
            network_reductions = PNM.NetworkReduction[PNM.DegreeTwoReduction()],
        )
    else
        ybus = PNM.Ybus(sys)
    end
    # Reuse the Ybus built above (it carries the reduction-aware subnetwork
    # grouping in `subnetwork_axes`) instead of a throwaway PNM.find_subnetworks.
    if isempty(model.subnetworks)
        model.subnetworks = _make_subnetworks_from_subnetwork_axes(ybus)
    end
    PNM.populate_branch_maps_by_type!(
        PNM.get_network_reduction_data(ybus),
        _get_filters(branch_models),
    )
    model.network_reduction = deepcopy(PNM.get_network_reduction_data(ybus))
    empty!(model.reduced_branch_tracker)
    set_number_of_steps!(model.reduced_branch_tracker, number_of_steps)
    return
end

function instantiate_network_model!(
    model::NetworkModel{AreaBalancePowerModel},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
)
    PNM.populate_branch_maps_by_type!(model.network_reduction)
    empty!(model.reduced_branch_tracker)
    set_number_of_steps!(model.reduced_branch_tracker, number_of_steps)
    return
end

function instantiate_network_model!(
    model::NetworkModel{CopperPlatePowerModel},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
)
    if isempty(model.subnetworks)
        model.subnetworks = PNM.find_subnetworks(sys)
    end
    if length(model.subnetworks) > 1
        @debug "System Contains Multiple Subnetworks. Assigning buses to subnetworks."
        model.network_reduction = deepcopy(PNM.get_network_reduction_data(PNM.Ybus(sys)))
        _assign_subnetworks_to_buses(model, sys)
    end
    empty!(model.reduced_branch_tracker)
    set_number_of_steps!(model.reduced_branch_tracker, number_of_steps)
    return
end

function _build_virtual_ptdf(
    model::NetworkModel{<:AbstractPTDFModel},
    sys::PSY.System,
    irreducible_buses::Vector{Int64},
)
    return PNM.VirtualPTDF(
        sys;
        tol = PTDF_ZERO_TOL,
        irreducible_buses = irreducible_buses,
        network_reductions = _build_network_reductions(model),
    )
end

function _build_virtual_modf(
    model::NetworkModel{<:AbstractPTDFModel},
    sys::PSY.System,
    irreducible_buses::Vector{Int64},
)
    return PNM.VirtualMODF(
        sys;
        tol = PTDF_ZERO_TOL,
        irreducible_buses = irreducible_buses,
        network_reductions = _build_network_reductions(model),
    )
end

"""
Keep a provided reduced matrix only when it already retains every bus in
`irreducible_buses` (the buses pinned by monitored components — time-series bounds
and/or outage-monitored devices); otherwise rebuild it with `build` so those buses
survive the reduction. Keeping a sufficient matrix avoids discarding valid user
input. Shared by the PTDF and MODF paths; `label` (`"PTDF"` / `"MODF"`) only names
the matrix in the log messages, and `build` is a zero-arg thunk that recomputes it
with the identified irreducible buses pinned.
"""
function _resolve_reduced_matrix(
    provided::Union{Nothing, PNM.PowerNetworkMatrix},
    label::String,
    irreducible_buses::Vector{Int64},
    build,
)
    if isnothing(provided)
        @info "No $label Matrix provided. Calculating using PowerNetworkMatrices.Virtual$label"
        return build()
    end
    n_missing = length(
        setdiff(
            Set(irreducible_buses),
            _retained_buses(PNM.get_network_reduction_data(provided)),
        ),
    )
    if n_missing > 0
        @warn "Provided $label Matrix does not retain $n_missing bus(es) pinned by \
               monitored components (time-series bounds and/or outage-monitored \
               devices). Recalculating $label Matrix with \
               PowerNetworkMatrices.Virtual$label and the identified irreducible buses."
        return build()
    end
    return provided
end

"""
Validate a kept PTDF matrix's embedded reductions against the model's reduce
flags. A rebuilt matrix satisfies these by construction; the checks matter when a
provided matrix is kept.
"""
function _validate_ptdf_reduction_flags(model::NetworkModel{<:AbstractPTDFModel})
    reductions = PNM.get_reductions(PNM.get_network_reduction_data(model.PTDF_matrix))
    if !model.reduce_radial_branches && PNM.has_radial_reduction(reductions)
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has reduced radial branches and mismatches the network \
                model specification reduce_radial_branches = false. Set the keyword argument \
                reduce_radial_branches = true in your network model",
            ),
        )
    end
    if !model.reduce_degree_two_branches && PNM.has_degree_two_reduction(reductions)
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has reduced degree two branches and mismatches the network \
                model specification reduce_degree_two_branches = false. Set the keyword argument \
                reduce_degree_two_branches = true in your network model",
            ),
        )
    end
    if model.reduce_radial_branches && PNM.has_ward_reduction(reductions)
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has  a ward reduction specified and the keyword argument \\
                reduce_radial_branches = true. Set the keyword argument reduce_radial_branches = false \\
                or provide a modified PTDF Matrix without the Ward reduction.",
            ),
        )
    end
    if model.reduce_radial_branches
        @assert !isempty(PNM.get_network_reduction_data(model.PTDF_matrix))
    end
    return
end

function _set_subnetworks_from_ptdf!(
    model::NetworkModel{<:AbstractPTDFModel},
    sys::PSY.System,
)
    if isempty(model.subnetworks)
        model.subnetworks = _make_subnetworks_from_subnetwork_axes(model.PTDF_matrix)
    end
    if length(model.subnetworks) > 1
        @debug "System Contains Multiple Subnetworks. Assigning buses to subnetworks."
        _assign_subnetworks_to_buses(model, sys)
    end
    return
end

"""
Guarantee the PTDF and MODF share an identical `bus_reduction_map` so the
nodal-balance rows and post-contingency MODF columns are dimensionally aligned.
`VirtualMODF` auto-protects monitored/outaged buses (`VirtualPTDF` does not), so
the MODF's retained set is the maximal one and is taken as the source of truth.
When the two disagree, keep the MODF and rebuild only the PTDF pinning the MODF's
retained buses, which forces the PTDF to reproduce exactly the MODF reduction.
Throw only if the two still diverge, which indicates a PowerNetworkMatrices
reduction defect rather than mismatched inputs.
"""
function _ensure_ptdf_modf_consistent!(
    model::NetworkModel{<:AbstractPTDFModel},
    sys::PSY.System,
)
    _reductions_match(get_PTDF_matrix(model), get_MODF_matrix(model)) && return
    @warn "PTDF and MODF were built with different network reductions. Rebuilding \
           the PTDF to match the MODF's retained buses so the nodal-balance and \
           post-contingency dimensions agree."
    modf_retained =
        collect(_retained_buses(PNM.get_network_reduction_data(get_MODF_matrix(model))))
    model.PTDF_matrix = _build_virtual_ptdf(model, sys, modf_retained)
    _reductions_match(get_PTDF_matrix(model), get_MODF_matrix(model)) && return
    np = length(_retained_buses(PNM.get_network_reduction_data(get_PTDF_matrix(model))))
    nm = length(_retained_buses(PNM.get_network_reduction_data(get_MODF_matrix(model))))
    throw(
        IS.ConflictingInputsError(
            "PTDF and MODF resolved to different network reductions even after \
            rebuilding the PTDF to match the MODF's retained buses (PTDF retains \
            $(np) buses, MODF retains $(nm)). The nodal-balance and post-contingency \
            dimensions cannot agree; this indicates a PowerNetworkMatrices reduction defect.",
        ),
    )
end

function instantiate_network_model!(
    model::NetworkModel{<:AbstractPTDFModel},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
)
    irreducible_buses =
        _get_irreducible_buses_due_to_monitored_components(sys, model, branch_models)
    # Resolve the PTDF first: it dimensions the nodal balance and supplies the
    # active network reduction. The buses to protect from elimination are passed
    # as `irreducible_buses` to each matrix constructor (see
    # `_build_virtual_ptdf` / `_build_virtual_modf`), which seeds them into the
    # reduction container; the reduction specs themselves are field-less config.
    model.PTDF_matrix = _resolve_reduced_matrix(
        get_PTDF_matrix(model),
        "PTDF",
        irreducible_buses,
        () -> _build_virtual_ptdf(model, sys, irreducible_buses),
    )
    _validate_ptdf_reduction_flags(model)
    if _template_uses_security_constrained_branch(branch_models)
        model.MODF_matrix = _resolve_reduced_matrix(
            get_MODF_matrix(model),
            "MODF",
            irreducible_buses,
            () -> _build_virtual_modf(model, sys, irreducible_buses),
        )
        # Both matrices must share one reduction so the nodal-balance rows and the
        # post-contingency MODF columns line up; reconcile (and fail fast) before
        # outage consolidation populates the branch maps.
        _ensure_ptdf_modf_consistent!(model, sys)
        _consolidate_device_model_outages_with_modf!(branch_models, get_MODF_matrix(model))
    end
    PNM.populate_branch_maps_by_type!(
        PNM.get_network_reduction_data(model.PTDF_matrix), _get_filters(branch_models),
    )
    model.network_reduction = deepcopy(PNM.get_network_reduction_data(model.PTDF_matrix))
    # After the reduction is known, before the constructors run.
    _prune_fully_reduced_branch_models!(model, branch_models)
    _warn_partially_reduced_monitored_lines!(model, branch_models)
    _set_subnetworks_from_ptdf!(model, sys)
    empty!(model.reduced_branch_tracker)
    set_number_of_steps!(model.reduced_branch_tracker, number_of_steps)
    return
end

function _make_subnetworks_from_subnetwork_axes(ptdf::PNM.PTDF)
    subnetworks = Dict{Int, Set{Int}}()
    for (ref_bus, ptdf_axes) in ptdf.subnetwork_axes
        subnetworks[ref_bus] = Set(ptdf_axes[1])
    end
    return subnetworks
end

function _make_subnetworks_from_subnetwork_axes(ptdf::PNM.VirtualPTDF)
    subnetworks = Dict{Int, Set{Int}}()
    for (ref_bus, ptdf_axes) in ptdf.subnetwork_axes
        subnetworks[ref_bus] = Set(ptdf_axes[2])
    end
    return subnetworks
end

function _make_subnetworks_from_subnetwork_axes(ybus::PNM.Ybus)
    subnetworks = Dict{Int, Set{Int}}()
    for (ref_bus, ybus_axes) in ybus.subnetwork_axes
        subnetworks[ref_bus] = Set(ybus_axes[1])
    end
    return subnetworks
end

function _assign_subnetworks_to_buses(
    model::NetworkModel{T},
    sys::PSY.System,
) where {T <: Union{CopperPlatePowerModel, AbstractPTDFModel}}
    subnetworks = model.subnetworks
    temp_bus_map = Dict{Int, Int}()
    network_reduction = PSI.get_network_reduction(model)
    for bus in PSI.get_available_components(model, PSY.ACBus, sys)
        bus_no = PSY.get_number(bus)
        mapped_bus_no = PNM.get_mapped_bus_number(network_reduction, bus)
        mapped_bus_no ∈ network_reduction.removed_buses && continue
        if haskey(temp_bus_map, bus_no)
            model.bus_area_map[bus] = temp_bus_map[bus_no]
            continue
        else
            bus_mapped = false
            for (subnet, bus_set) in subnetworks
                if mapped_bus_no ∈ bus_set
                    temp_bus_map[bus_no] = subnet
                    model.bus_area_map[bus] = subnet
                    bus_mapped = true
                    break
                end
            end
        end
        if !bus_mapped
            error(
                "Bus $(PSY.summary(bus)) not mapped to any reference bus: Mapped bus number: $(mapped_bus_no)",
            )
        end
    end
    return
end

_assign_subnetworks_to_buses(
    ::NetworkModel{T},
    ::PSY.System,
) where {T <: PM.AbstractPowerModel} = nothing

function get_reference_bus(
    model::NetworkModel{T},
    b::PSY.ACBus,
)::Int where {T <: PM.AbstractPowerModel}
    if isempty(model.bus_area_map)
        return first(keys(model.subnetworks))
    else
        return model.bus_area_map[b]
    end
end
