function _any_component_has_branch_rating_ts(
    ::Type{P},
    device_model::DeviceModel,
    sys::PSY.System,
) where {P <: AbstractBranchRatingTimeSeriesParameter}
    haskey(get_time_series_names(device_model), P) || return false
    ts_name = get_time_series_names(device_model)[P]
    # Only the modeled forecast matters: operations consume a
    # Deterministic-family forecast, never a bare SingleTimeSeries. Use the
    # same `ts_type` the reduction path resolves so both pathways agree on
    # what "has the branch rating time series" means. Resolved lazily — branch
    # models without the parameter configured returned above and never reach
    # `get_deterministic_time_series_type`.
    ts_type = get_deterministic_time_series_type(sys)
    return any(
        c -> PSY.has_time_series(c, ts_type, ts_name),
        get_device_cache(device_model),
    )
end

# Both `BranchRatingTimeSeriesParameter` and
# `PostContingencyBranchRatingTimeSeriesParameter` are only honored by the
# `StaticBranch` (pre-contingency, PTDF) and
# `AbstractSecurityConstrainedStaticBranch` constructors. Any other
# formulation that carries either series passes validation but never builds a
# usable parameter container, so the series would be silently ignored —
# reject it up front instead.
function _check_branch_rating_time_series_formulation!(
    branch_models::BranchModelContainer,
    sys::PSY.System,
)
    for (_, device_model) in branch_models
        D = get_component_type(device_model)
        B = get_formulation(device_model)
        for P in (
            BranchRatingTimeSeriesParameter,
            PostContingencyBranchRatingTimeSeriesParameter,
        )
            _any_component_has_branch_rating_ts(P, device_model, sys) || continue
            if B <: StaticBranch || B <: AbstractSecurityConstrainedStaticBranch
                continue
            elseif B <: StaticBranchUnbounded
                @warn "$(P) is attached to $(D) components but $(B) does not \
                       enforce flow limits; the branch rating time series will \
                       be ignored for these branches." _group =
                    LOG_GROUP_MODELS_VALIDATION
                continue
            else
                throw(
                    IS.ConflictingInputsError(
                        "$(P) is only supported with the StaticBranch or \
                        AbstractSecurityConstrainedStaticBranch formulations, \
                        but branch type $(D) was configured with $(B). Remove \
                        the branch rating time series from the components or \
                        change the formulation.",
                    ),
                )
            end
        end
    end
    return
end

function _check_security_constrained_three_winding_transformer!(
    branch_models::BranchModelContainer,
)
    for (_, device_model) in branch_models
        D = get_component_type(device_model)
        B = get_formulation(device_model)
        if D <: PSY.ThreeWindingTransformer &&
           B <: AbstractSecurityConstrainedStaticBranch
            throw(
                IS.ConflictingInputsError(
                    "Security-constrained branch formulations are not implemented \
                    yet for $(D), but it was configured with $(B). Use a non \
                    security-constrained formulation (e.g. StaticBranch) for \
                    $(D), or remove it from the template.",
                ),
            )
        end
    end
    return
end

# Whether `SecurityConstrainedStaticBranch` has a `construct_device!` path for
# this network model. PTDF and ACP build full post-contingency limits;
# NFA/CopperPlate/AreaBalance are intentional no-ops. Dispatched (no `isa`):
# the fallback returns `false` so unsupported networks (e.g. DCP, whose
# angle-based path is pending) fail fast at validation instead of hitting a
# `MethodError` during build.
_sc_branch_network_supported(::NetworkModel{<:AbstractPTDFModel}) = true
_sc_branch_network_supported(::NetworkModel{<:PM.AbstractACPModel}) = true
_sc_branch_network_supported(::NetworkModel{NFAPowerModel}) = true
_sc_branch_network_supported(::NetworkModel{CopperPlatePowerModel}) = true
_sc_branch_network_supported(::NetworkModel{AreaBalancePowerModel}) = true
_sc_branch_network_supported(::NetworkModel) = false

function _check_security_constrained_network!(
    branch_models::BranchModelContainer,
    network_model::NetworkModel,
)
    _sc_branch_network_supported(network_model) && return
    for (_, device_model) in branch_models
        B = get_formulation(device_model)
        if B <: AbstractSecurityConstrainedStaticBranch
            throw(
                IS.ConflictingInputsError(
                    "$(B) is not supported with network model \
                    $(get_network_formulation(network_model)). Use a PTDF \
                    (AbstractPTDFModel) or ACP network model. DCP support \
                    (angle-based post-contingency) is pending; NFA, \
                    CopperPlate and AreaBalance are inert for \
                    security-constrained branches.",
                ),
            )
        end
    end
    return
end

function _check_branch_network_compatibility(
    ::NetworkModel{T},
    unmodeled_branch_types::Vector{DataType},
) where {T <: PM.AbstractPowerModel}
    if requires_all_branch_models(T) && !isempty(unmodeled_branch_types)
        for d in unmodeled_branch_types
            @error "The system has a branch branch type $(d) but the DeviceModel is not included in the Template."
        end
        throw(
            IS.ConflictingInputsError(
                "Network model $(T) requires all AC Transmission devices have a model",
            ),
        )
    end
    return
end

function _validate_branch_models(
    ::Type{T},
    model_has_branch_filters::Bool,
) where {T <: PM.AbstractPowerModel}
    if supports_branch_filtering(T) || !model_has_branch_filters
        return
    elseif model_has_branch_filters
        if ignores_branch_filtering(T)
            @warn "Branch filtering is ignored for network model $(T)"
        else
            throw(
                IS.ConflictingInputsError(
                    "Branch filtering is not supported for network model $(T). Remove branch \\
                    filter functions from branch models or use a different network model.",
                ),
            )
        end
    else
        throw(
            IS.ConflictingInputsError(
                "Network model $(T) can't be validated against branch models",
            ),
        )
    end
    return
end

function validate_network_model(network_model::NetworkModel{T},
    unmodeled_branch_types::Vector{DataType},
    model_has_branch_filters::Bool,
) where {T <: PM.AbstractPowerModel}
    _check_branch_network_compatibility(network_model, unmodeled_branch_types)
    _validate_branch_models(T, model_has_branch_filters)
    return
end

function validate_template_impl!(model::OperationModel)
    template = get_template(model)
    settings = get_settings(model)
    if isempty(template)
        error("Template can't be empty for models $(get_problem_type(model))")
    end
    system = get_system(model)
    modeled_types = get_component_types(template)
    system_component_types = PSY.get_existing_component_types(system)
    network_model = get_network_model(template)
    valid_device_types = union(modeled_types, _TEMPLATE_VALIDATION_EXCLUSIONS)
    unmodeled_branch_types = DataType[]

    for m in setdiff(system_component_types, valid_device_types)
        @warn "The template doesn't include models for components of type $(m), consider changing the template" _group =
            LOG_GROUP_MODELS_VALIDATION
        if m <: PSY.ACTransmission
            push!(unmodeled_branch_types, m)
        end
    end

    device_keys_to_delete = Symbol[]
    for (k, device_model) in model.template.devices
        make_device_cache!(device_model, system, get_check_components(settings))
        if isempty(get_device_cache(device_model))
            @info "The system data doesn't include devices of type $(k), consider changing the models in the template" _group =
                LOG_GROUP_MODELS_VALIDATION
            push!(device_keys_to_delete, k)
        end
    end
    for k in device_keys_to_delete
        delete!(model.template.devices, k)
    end

    model_has_branch_filters = false
    branch_keys_to_delete = Symbol[]
    # Rebuilt fresh on every validation pass so it always matches the branch
    # types actually present in the template; otherwise stale entries from an
    # earlier build (e.g. a branch type later pruned for an empty device cache)
    # leak into the PTDF branch_models lookups and raise a KeyError.
    empty!(network_model.modeled_ac_branch_types)
    # Skip PSY.check_component for branches the network never models; the device
    # cache is still built below to prune empty types and populate modeled_ac_branch_types.
    validate_branches =
        get_check_components(settings) &&
        branches_modeled(get_network_formulation(network_model))
    for (k, device_model) in model.template.branches
        make_device_cache!(device_model, system, validate_branches)
        if isempty(get_device_cache(device_model))
            @info "The system data doesn't include Branches of type $(k), consider changing the models in the template" _group =
                LOG_GROUP_MODELS_VALIDATION
            push!(branch_keys_to_delete, k)
        elseif _contributes_to_ac_branch_types(get_component_type(device_model))
            push!(network_model.modeled_ac_branch_types, get_component_type(device_model))
        end
        if !isnothing(get_attribute(device_model, "filter_function"))
            model_has_branch_filters = true
        end
    end
    for k in branch_keys_to_delete
        delete!(model.template.branches, k)
    end
    _check_branch_rating_time_series_formulation!(model.template.branches, system)
    _check_security_constrained_three_winding_transformer!(model.template.branches)
    _check_security_constrained_network!(model.template.branches, network_model)
    validate_network_model(network_model, unmodeled_branch_types, model_has_branch_filters)
    _build_device_model_outages!(template, system)
    _build_service_model_outages!(template, system)
    return
end

"""
Populate `device_model.outages` for every security-constrained (SC) branch
device model in the template, in a single pass over the system's outage
supplemental attributes. `DeviceModel{D, SC}` claims an outage iff `D` is
among the types of the outaged (attached) components — so the OUTAGED
component's type must be covered by an SC `DeviceModel` for the outage to
contribute any constraints. For a multi-component outage tripping components
of N>1 types, every matching SC `DeviceModel` claims it; the post-contingency
build (`add_post_contingency_*`) deduplicates by referencing the first
claimer's expressions/constraints.

The inner dict carries the per-modeled-type breakdown of monitored component
names. Monitored components do NOT need to be security-constrained themselves:
an SC `DeviceModel{Line, SecurityConstrainedStaticBranch}` may claim an outage
whose monitored set includes non-SC `Transformer2W` components, and the
post-contingency build then bounds those Transformer flows under that outage.

Selection semantics:
- If `m.outages` is non-empty when this runs, the user explicitly listed UUIDs
  via the constructor kwarg. Restrict to those UUIDs only; warn for any
  user-listed UUID that produced no `D`-type entry.
- If `m.outages` is empty, auto-discover. Honor `"include_planned_outages"`
  on `m`'s attributes (default `false`) — `PlannedOutage`s are skipped on the
  auto-discover path unless the attribute is `true`. Explicit user selection
  bypasses this filter (listing a planned outage explicitly is intent).

There is no implicit "monitor everything" default. The monitored set is
exactly what each outage lists in its `monitored_components`; an outage with
empty `monitored_components` is treated as "monitor nothing" — a warning is
emitted and it contributes no post-contingency constraints. This is
intentional: defaulting to monitoring every branch under every outage would
silently build an N-1-everything-by-everything problem that is intractable
for realistic systems, so monitoring is strictly opt-in per branch.

A monitored component whose type is not a modeled `PSY.ACTransmission` branch
type (either not in the template, or modeled but not a branch) is reported
once per type with the offending outage UUIDs and is skipped: no
post-contingency variables or constraints are built for that monitored
component under any outage.
"""
function _build_device_model_outages!(
    template::ProblemTemplate,
    sys::PSY.System,
)
    # This code needs to be extended when adding G-1 models to check for injectors too.
    sc_models = _sc_branch_models(template)
    isempty(sc_models) && return

    modeled_types = Set{DataType}(get_component_types(template))
    selection = _take_outage_selection!(sc_models)
    uncovered_types = Dict{DataType, Set{Base.UUID}}()

    for outage in PSY.get_supplemental_attributes(PSY.Outage, sys)
        outage_uuid = IS.get_uuid(outage)
        if isempty(PSY.get_monitored_components(outage))
            @warn "Outage $(outage_uuid) ($(typeof(outage))) has empty \
                   monitored_components; no post-contingency variables or \
                   constraints will be created for this outage." _group =
                LOG_GROUP_MODELS_VALIDATION
            continue
        end

        per_type, uncovered =
            _monitored_components_by_modeled_type(outage, outage_uuid, sys, modeled_types)
        for comp_type in uncovered
            push!(get!(uncovered_types, comp_type, Set{Base.UUID}()), outage_uuid)
        end
        isempty(per_type) && continue

        attached_types = _attached_component_types(outage, sys)
        covered = _assign_outage_to_sc_models!(
            sc_models,
            selection,
            outage,
            outage_uuid,
            per_type,
            attached_types,
        )
        if !covered
            @warn "Outage $(outage_uuid) is attached to component(s) of \
                   type $(collect(attached_types)), but no DeviceModel with \
                   an AbstractSecurityConstrainedStaticBranch formulation \
                   covers those types; it will not contribute any \
                   post-contingency constraints." _group =
                LOG_GROUP_MODELS_VALIDATION
        end
    end

    _warn_uncovered_monitored_types(uncovered_types)
    _warn_unmatched_user_outages(sc_models, selection)
    return
end

# SC branch device models in the template. Extend alongside G-1 injector
# models when those are added.
function _sc_branch_models(template::ProblemTemplate)
    return DeviceModelForBranches[
        m for m in values(get_branch_models(template)) if
        get_formulation(m) <: AbstractSecurityConstrainedStaticBranch
    ]
end

# Per SC-model component type, the user's explicit outage-UUID allow-list from
# the constructor kwarg: a non-empty set restricts auto-discovery to those
# UUIDs; an empty set means auto-discover all. Clears `m.outages` so the main
# pass can repopulate it; the cleared UUIDs survive in the returned map.
function _take_outage_selection!(sc_models::Vector{DeviceModelForBranches})
    selection = Dict{Symbol, Set{Base.UUID}}()
    for m in sc_models
        # `keys(m.outages)` is already empty when the user listed nothing, so
        # this yields the empty (auto-discover) set with no special case.
        selection[nameof(get_component_type(m))] = Set{Base.UUID}(keys(m.outages))
        empty!(m.outages)
    end
    return selection
end

# Monitored-component names grouped by their concrete (modeled) type. Returns
# `(per_type, uncovered)` where `uncovered` is the set of monitored component
# types the template does not model — the caller records the offending outage
# against them. Pure except for the not-in-system warning.
function _monitored_components_by_modeled_type(
    outage::PSY.Outage,
    outage_uuid::Base.UUID,
    sys::PSY.System,
    modeled_types::Set{DataType},
)
    per_type = Dict{DataType, Set{String}}()
    uncovered = Set{DataType}()
    for uuid in PSY.get_monitored_components(outage)
        component = IS.get_component(sys, uuid)
        if isnothing(component)
            @warn "Outage $(outage_uuid) references monitored component \
                   UUID $(uuid) that is not present in the system; \
                   skipping." _group = LOG_GROUP_MODELS_VALIDATION
            continue
        end
        comp_type = typeof(component)
        # Post-contingency flow limits only make sense on branch arcs: the
        # post-contingency builder resolves every `per_type` key through
        # `name_to_arc_maps`, which is keyed by ACTransmission branch types
        # only. `PSY.AreaInterchange` is admitted separately so the
        # AreaBalance service-side builder can pick it up. A monitored
        # component that is neither would `KeyError` there, so route it to
        # the skip path.
        admissible = (comp_type <: PSY.ACTransmission || comp_type <: PSY.AreaInterchange)
        if admissible && comp_type in modeled_types
            push!(get!(per_type, comp_type, Set{String}()), PSY.get_name(component))
        else
            push!(uncovered, comp_type)
        end
    end
    return per_type, uncovered
end

function _attached_component_types(outage::PSY.Outage, sys::PSY.System)
    return Set{DataType}(
        typeof(c) for c in PSY.get_associated_components(sys, outage)
    )
end

# Sibling of `_attached_component_types` returning the actual attached
# `PSY.Component` instances. Used by the SC reserve service per-service
# scoping path to intersect attached injectors with each service's
# contributing devices.
function _attached_components(outage::PSY.Outage, sys::PSY.System)
    return collect(PSY.get_associated_components(sys, outage))
end

# Whether SC model `m` claims `outage`. `sel` is `m`'s component-type slice of
# the user's explicit outage allow-list: non-empty restricts to those UUIDs;
# empty means auto-discover (claim all, skipping `PlannedOutage`s unless the
# model opts in via the `"include_planned_outages"` attribute).
function _sc_model_claims_outage(
    m::DeviceModelForBranches,
    outage::PSY.Outage,
    outage_uuid::Base.UUID,
    sel::Set{Base.UUID},
)
    isempty(sel) || return outage_uuid in sel
    if outage isa PSY.PlannedOutage
        return get_attribute(m, "include_planned_outages")
    end
    return true
end

# Assign `per_type` to every SC model whose component type is among the
# outage's attached types and that claims the outage. Returns whether any SC
# model *covered* an attached type (independent of whether it claimed it); the
# caller warns when nothing covers the outage. `DeviceModel{D, SC}` claims an
# outage iff `D` is among the outaged (attached) component types; multi-type
# outages are claimed by every matching SC model and the post-contingency
# build dedups by referencing the first claimer.
function _assign_outage_to_sc_models!(
    sc_models::Vector{DeviceModelForBranches},
    selection::Dict{Symbol, Set{Base.UUID}},
    outage::PSY.Outage,
    outage_uuid::Base.UUID,
    per_type::Dict{DataType, Set{String}},
    attached_types::Set{DataType},
)
    covered = false
    for m in sc_models
        D = get_component_type(m)
        D in attached_types || continue
        covered = true
        if _sc_model_claims_outage(m, outage, outage_uuid, selection[nameof(D)])
            m.outages[outage_uuid] = per_type
        end
    end
    return covered
end

function _warn_uncovered_monitored_types(
    uncovered_types::Dict{DataType, Set{Base.UUID}},
)
    for (comp_type, offending) in uncovered_types
        @warn "Monitored components of type $(comp_type) appear in outages \
               $(collect(offending)) but $(comp_type) is not a modeled \
               ACTransmission branch type; their post-contingency variables \
               will be skipped." _group =
            LOG_GROUP_MODELS_VALIDATION
    end
    return
end

function _warn_unmatched_user_outages(
    sc_models::Vector{DeviceModelForBranches},
    selection::Dict{Symbol, Set{Base.UUID}},
)
    for m in sc_models
        D = get_component_type(m)
        sel = selection[nameof(D)]
        isempty(sel) && continue
        for uuid in sel
            haskey(m.outages, uuid) && continue
            @warn "Outage $(uuid) listed on DeviceModel{$D, \
                   $(get_formulation(m))} is not attached to a component \
                   of type $D in the system — it will not contribute any \
                   post-contingency constraints." _group =
                LOG_GROUP_MODELS_VALIDATION
        end
    end
    return
end

"""
Populate `service_model.outages` for every security-constrained (SC) reserve
`ServiceModel` in the template. Mirrors `_build_device_model_outages!`, but
keyed off outages whose *attached* component is an injector (typically a
generator) instead of a branch. Monitored components are still branches, and
the inner `Dict{DataType, Set{String}}` is grouped by their modeled branch
type — the post-contingency build resolves the branch type's arc map.

Selection semantics match the device version: a non-empty `m.outages` is
treated as the user's explicit UUID allow-list; an empty `m.outages` triggers
auto-discovery honoring the `"include_planned_outages"` attribute (default
`false`).
"""
function _build_service_model_outages!(
    template::ProblemTemplate,
    sys::PSY.System,
)
    sc_service_models = _sc_reserve_service_models(template)
    isempty(sc_service_models) && return

    modeled_types = Set{DataType}(get_component_types(template))
    selection = _take_service_outage_selection!(sc_service_models)
    uncovered_types = Dict{DataType, Set{Base.UUID}}()

    for outage in PSY.get_supplemental_attributes(PSY.Outage, sys)
        outage_uuid = IS.get_uuid(outage)
        if isempty(PSY.get_monitored_components(outage))
            @warn "Outage $(outage_uuid) ($(typeof(outage))) has empty \
                   monitored_components; no post-contingency variables or \
                   constraints will be created for this outage." _group =
                LOG_GROUP_MODELS_VALIDATION
            continue
        end

        per_type, uncovered = _monitored_components_by_modeled_type(
            outage, outage_uuid, sys, modeled_types,
        )
        for comp_type in uncovered
            push!(get!(uncovered_types, comp_type, Set{Base.UUID}()), outage_uuid)
        end
        isempty(per_type) && continue

        attached_types = _attached_component_types(outage, sys)
        # SC reserve service models claim outages whose attached component is
        # an injector. Skip outages attached only to branches — those belong
        # to N-1 branch-outage SC device models.
        any(t -> t <: PSY.StaticInjection, attached_types) || continue

        covered = _assign_outage_to_sc_service_models!(
            sc_service_models,
            selection,
            outage,
            outage_uuid,
            per_type,
            sys,
        )
        if !covered
            @warn "Outage $(outage_uuid) is attached to injector(s) of \
                   type $(collect(attached_types)), but no ServiceModel with \
                   an AbstractSecurityConstrainedReservesFormulation is \
                   present in the template; it will not contribute any \
                   post-contingency constraints." _group =
                LOG_GROUP_MODELS_VALIDATION
        end
    end

    _warn_uncovered_monitored_types(uncovered_types)
    _warn_unmatched_user_service_outages(sc_service_models, selection)
    return
end

function _sc_reserve_service_models(template::ProblemTemplate)
    return ServiceModel[
        m for m in values(get_service_models(template)) if
        get_formulation(m) <: AbstractSecurityConstrainedReservesFormulation
    ]
end

function _take_service_outage_selection!(sc_service_models::Vector{ServiceModel})
    selection = Dict{Tuple{DataType, String}, Set{Base.UUID}}()
    for m in sc_service_models
        key = (get_component_type(m), get_service_name(m))
        selection[key] = Set{Base.UUID}(keys(m.outages))
        empty!(m.outages)
    end
    return selection
end

function _sc_service_claims_outage(
    m::ServiceModel,
    outage::PSY.Outage,
    outage_uuid::Base.UUID,
    sel::Set{Base.UUID},
)
    isempty(sel) || return outage_uuid in sel
    if outage isa PSY.PlannedOutage
        attr = get_attribute(m, "include_planned_outages")
        return attr === true
    end
    return true
end

function _assign_outage_to_sc_service_models!(
    sc_service_models::Vector{ServiceModel},
    selection::Dict{Tuple{DataType, String}, Set{Base.UUID}},
    outage::PSY.Outage,
    outage_uuid::Base.UUID,
    per_type::Dict{DataType, Set{String}},
    sys::PSY.System,
)
    covered = false
    attached = _attached_components(outage, sys)
    attached_uuids = Set{Base.UUID}(IS.get_uuid(c) for c in attached)
    for m in sc_service_models
        key = (get_component_type(m), get_service_name(m))
        sel = selection[key]
        if isempty(sel)
            # Auto-discovery path: only assign the outage to this service if
            # one of the outage's attached injectors is among the service's
            # contributing devices. The explicit allow-list path below
            # bypasses this filter so users keep full control over which
            # outages a service responds to.
            service = PSY.get_component(
                get_component_type(m),
                sys,
                get_service_name(m),
            )
            service === nothing && continue
            contributing_uuids = Set{Base.UUID}(
                IS.get_uuid(c) for c in PSY.get_contributing_devices(sys, service)
            )
            isempty(intersect(attached_uuids, contributing_uuids)) && continue
        end
        if _sc_service_claims_outage(m, outage, outage_uuid, sel)
            m.outages[outage_uuid] = per_type
            covered = true
        end
    end
    return covered
end

function _warn_unmatched_user_service_outages(
    sc_service_models::Vector{ServiceModel},
    selection::Dict{Tuple{DataType, String}, Set{Base.UUID}},
)
    for m in sc_service_models
        key = (get_component_type(m), get_service_name(m))
        sel = selection[key]
        isempty(sel) && continue
        for uuid in sel
            haskey(m.outages, uuid) && continue
            D = get_component_type(m)
            @warn "Outage $(uuid) listed on ServiceModel{$D, \
                   $(get_formulation(m))} (service_name=$(get_service_name(m))) \
                   was not found among the system's outage supplemental \
                   attributes — it will not contribute any post-contingency \
                   constraints." _group = LOG_GROUP_MODELS_VALIDATION
        end
    end
    return
end
