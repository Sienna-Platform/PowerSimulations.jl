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
- `subnetworks::Dict{Int, Set{Int}}` = Dict()
    Optional mapping of reference bus → set of mapped buses. If not provided,
    subnetworks are inferred from PTDF/VirtualPTDF or discovered from the system.
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

nw2 = NetworkModel(CopperPlatePowerModel; subnetworks = Dict(1 => Set([1,2,3])))
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
        subnetworks = Dict{Int, Set{Int}}(),
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
            subnetworks,
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

function _build_network_reductions(
    model::NetworkModel,
    irreducible_buses::Vector{Int64},
)
    reductions = PNM.NetworkReduction[]
    if model.reduce_radial_branches
        push!(reductions, PNM.RadialReduction(; irreducible_buses = irreducible_buses))
    end
    if model.reduce_degree_two_branches
        push!(
            reductions,
            PNM.DegreeTwoReduction(; irreducible_buses = irreducible_buses),
        )
    end
    return reductions
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
    return collect(irreducible_buses)
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

function instantiate_network_model!(
    model::NetworkModel{T},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
) where {T <: PM.AbstractPowerModel}
    if isempty(model.subnetworks)
        model.subnetworks = PNM.find_subnetworks(sys)
    end
    irreducible_buses = _get_irreducible_buses_due_to_monitored_components(
        sys,
        model,
        branch_models,
    )
    if model.reduce_radial_branches && model.reduce_degree_two_branches
        @info "Applying both radial and degree two reductions"
        ybus = PNM.Ybus(
            sys;
            network_reductions = PNM.NetworkReduction[
                PNM.RadialReduction(; irreducible_buses = irreducible_buses),
                PNM.DegreeTwoReduction(; irreducible_buses = irreducible_buses),
            ],
        )
    elseif model.reduce_radial_branches
        @info "Applying radial reduction"
        if !isempty(irreducible_buses)
            @warn "Irreducible buses identified from monitored components. The reduction of any radial branch between 2 irreducible buses will be ignored"
        end
        ybus =
            PNM.Ybus(
                sys;
                network_reductions = PNM.NetworkReduction[PNM.RadialReduction(;
                    irreducible_buses = irreducible_buses,
                )],
            )
    elseif model.reduce_degree_two_branches
        @info "Applying degree two reduction"
        ybus = PNM.Ybus(
            sys;
            network_reductions = PNM.NetworkReduction[PNM.DegreeTwoReduction(;
                irreducible_buses = irreducible_buses,
            )],
        )
    else
        ybus = PNM.Ybus(sys)
    end
    model.network_reduction = deepcopy(PNM.get_network_reduction_data(ybus))
    #if !isempty(model.network_reductionget_net_reduction_data)
    # TODO: Network reimplement this when it becomes necessary. We don't have any
    # reductions that are incompatible right now.
    # check_network_reduction_compatibility(T)
    #end
    PNM.populate_branch_maps_by_type!(model.network_reduction, _get_filters(branch_models))
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

# Verify a user-provided MODF Matrix was built with the same network reduction
# as the active reduction (derived from the PTDF Matrix). Equality of the bus
# reduction map is the decisive check: it fixes the reduced bus/arc numbering
# the post-contingency builder uses to index `modf_matrix[arc, outage_spec]`.
function _validate_provided_modf_reduction!(
    modf::PNM.VirtualMODF,
    network_reduction::PNM.NetworkReductionData,
)
    if PNM.get_bus_reduction_map(modf.network_reduction_data) !=
       PNM.get_bus_reduction_map(network_reduction)
        throw(
            IS.ConflictingInputsError(
                "The provided MODF Matrix was built with a different network \
                reduction than the active reduction derived from the PTDF \
                Matrix. Rebuild the MODF with a consistent network reduction, \
                or omit it so it is recalculated automatically.",
            ),
        )
    end
    return
end

function instantiate_network_model!(
    model::NetworkModel{<:AbstractPTDFModel},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
)
    irreducible_buses = _get_irreducible_buses_due_to_monitored_components(
        sys,
        model,
        branch_models,
    )
    if isnothing(get_PTDF_matrix(model)) || !isempty(irreducible_buses)
        if !isnothing(get_PTDF_matrix(model))
            @warn "Provided PTDF Matrix is being ignored since irreducible buses were identified from monitored components (TimeSeriesBounds and/or outage-monitored devices). Recalculating PTDF Matrix with PowerNetworkMatrices.VirtualPTDF and the identified irreducible buses."
        else
            @info "No PTDF Matrix provided. Calculating using PowerNetworkMatrices.VirtualPTDF"
        end

        if model.reduce_radial_branches && model.reduce_degree_two_branches
            @info "Applying both radial and degree two reductions"
            ptdf = PNM.VirtualPTDF(
                sys;
                tol = PTDF_ZERO_TOL,
                network_reductions = PNM.NetworkReduction[
                    PNM.RadialReduction(; irreducible_buses = irreducible_buses),
                    PNM.DegreeTwoReduction(;
                        irreducible_buses = irreducible_buses,
                    ),
                ],
            )
        elseif model.reduce_radial_branches
            @info "Applying radial reduction"
            if !isempty(irreducible_buses)
                @warn "Irreducible buses identified from monitored components. The reduction of any radial branch between 2 irreducible buses will be ignored"
            end
            ptdf = PNM.VirtualPTDF(
                sys;
                tol = PTDF_ZERO_TOL,
                network_reductions = PNM.NetworkReduction[PNM.RadialReduction(;
                    irreducible_buses = irreducible_buses,
                )],
            )
        elseif model.reduce_degree_two_branches
            @info "Applying degree two reduction"
            ptdf = PNM.VirtualPTDF(
                sys;
                tol = PTDF_ZERO_TOL,
                network_reductions = PNM.NetworkReduction[PNM.DegreeTwoReduction(;
                    irreducible_buses = irreducible_buses,
                )],
            )
        else
            ptdf = PNM.VirtualPTDF(sys; tol = PTDF_ZERO_TOL)
        end
        model.PTDF_matrix = ptdf
        model.network_reduction = deepcopy(ptdf.network_reduction_data)
    else
        model.network_reduction = deepcopy(model.PTDF_matrix.network_reduction_data)
    end

    if !model.reduce_radial_branches && PNM.has_radial_reduction(
        PNM.get_reductions(model.PTDF_matrix.network_reduction_data),
    )
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has reduced radial branches and mismatches the network \
                model specification reduce_radial_branches = false. Set the keyword argument \
                reduce_radial_branches = true in your network model"),
        )
    end
    if !model.reduce_degree_two_branches && PNM.has_degree_two_reduction(
        PNM.get_reductions(model.PTDF_matrix.network_reduction_data),
    )
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has reduced degree two branches and mismatches the network \
                model specification reduce_degree_two_branches = false. Set the keyword argument \
                reduce_degree_two_branches = true in your network model"),
        )
    end
    if model.reduce_radial_branches &&
       PNM.has_ward_reduction(PNM.get_reductions(model.PTDF_matrix.network_reduction_data))
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has  a ward reduction specified and the keyword argument \\
                reduce_radial_branches = true. Set the keyword argument reduce_radial_branches = false \\
                or provide a modified PTDF Matrix without the Ward reduction."),
        )
    end

    if model.reduce_radial_branches
        @assert !isempty(model.PTDF_matrix.network_reduction_data)
    end
    model.subnetworks = _make_subnetworks_from_subnetwork_axes(model.PTDF_matrix)
    if length(model.subnetworks) > 1
        @debug "System Contains Multiple Subnetworks. Assigning buses to subnetworks."
        _assign_subnetworks_to_buses(model, sys)
    end
    if _template_uses_security_constrained_branch(branch_models)
        if isnothing(get_MODF_matrix(model))
            @info "No MODF Matrix provided. Calculating using PowerNetworkMatrices.VirtualMODF"
            model.MODF_matrix = PNM.VirtualMODF(
                sys;
                tol = PTDF_ZERO_TOL,
                network_reductions = _build_network_reductions(model, irreducible_buses),
            )
        elseif !isempty(irreducible_buses)
            @warn "Provided MODF Matrix is being ignored since irreducible buses were identified from monitored components (TimeSeriesBounds and/or outage-monitored devices). Recalculating MODF Matrix with PowerNetworkMatrices.VirtualMODF and the identified irreducible buses."
            model.MODF_matrix = PNM.VirtualMODF(
                sys;
                tol = PTDF_ZERO_TOL,
                network_reductions = _build_network_reductions(model, irreducible_buses),
            )
        else
            # The provided MODF is kept verbatim. The post-contingency
            # expression builder resolves monitored arcs from
            # `model.network_reduction` (taken from the PTDF) and then indexes
            # `modf_matrix[arc, outage_spec]`; a MODF built with a different
            # network reduction or bus numbering would silently mis-key or
            # KeyError, so reject the mismatch up front.
            _validate_provided_modf_reduction!(
                get_MODF_matrix(model),
                model.network_reduction,
            )
        end
        _consolidate_device_model_outages_with_modf!(
            branch_models, get_MODF_matrix(model),
        )
    end
    PNM.populate_branch_maps_by_type!(model.network_reduction, _get_filters(branch_models))
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
