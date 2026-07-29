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
type (either not in the template, or modeled but not an ACTransmission
subtype — `PSY.AreaInterchange` included, since the device-side
post-contingency build has no mechanism to bound an AreaInterchange under an
outage) is reported once per type with the offending outage UUIDs and is
skipped: no post-contingency variables or constraints are built for that
monitored component under any outage.
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

        per_type, uncovered = _monitored_components_by_modeled_type(
            outage, outage_uuid, sys, modeled_types, DeviceOutageConsumer(),
        )
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

    _warn_uncovered_monitored_types(uncovered_types, DeviceOutageConsumer())
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

# Which consumer is classifying monitored components: the admissible
# monitored types differ (device: ACTransmission only, since the
# device-side post-contingency build resolves monitors through a
# branch-arc map; service: ACTransmission + AreaInterchange, since the
# AreaBalance service-side builder also resolves AreaInterchange monitors as
# system components). Dispatch on the consumer keeps that difference out of
# the shared classifier body.
abstract type MonitoredOutageConsumer end
struct DeviceOutageConsumer <: MonitoredOutageConsumer end
struct ServiceOutageConsumer <: MonitoredOutageConsumer end

_admits_monitored_type(::DeviceOutageConsumer, ::Type{<:PSY.ACTransmission}) = true
_admits_monitored_type(::DeviceOutageConsumer, ::Type) = false
_admits_monitored_type(::ServiceOutageConsumer, ::Type{<:PSY.ACTransmission}) = true
_admits_monitored_type(::ServiceOutageConsumer, ::Type{<:PSY.AreaInterchange}) = true
_admits_monitored_type(::ServiceOutageConsumer, ::Type) = false

_admitted_types_description(::DeviceOutageConsumer) = "a modeled ACTransmission branch type"
_admitted_types_description(::ServiceOutageConsumer) =
    "a modeled ACTransmission branch type or AreaInterchange"

# Monitored-component names grouped by their concrete (modeled) type. Returns
# `(per_type, uncovered)` where `uncovered` is the set of monitored component
# types `consumer` cannot use — the caller records the offending outage
# against them. Pure except for the not-in-system warning.
function _monitored_components_by_modeled_type(
    outage::PSY.Outage,
    outage_uuid::Base.UUID,
    sys::PSY.System,
    modeled_types::Set{DataType},
    consumer::MonitoredOutageConsumer,
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
        if _admits_monitored_type(consumer, comp_type) && comp_type in modeled_types
            push!(get!(per_type, comp_type, Set{String}()), PSY.get_name(component))
        else
            push!(uncovered, comp_type)
        end
    end
    return per_type, uncovered
end

# Whether `model` (a security-constrained `DeviceModel` or `ServiceModel`)
# has opted in to auto-discovering `PlannedOutage`s via the
# `"include_planned_outages"` attribute. Anything other than the literal
# `true` — including a missing key — counts as "not opted in": a
# `ServiceModel`'s default attributes never set this key (it stays `nothing`
# unless the user does), while a security-constrained `DeviceModel`'s
# defaults always set it to `false`; the same null-safe check is correct for
# both.
_includes_planned_outages(model::Union{DeviceModel, ServiceModel}) =
    get_attribute(model, "include_planned_outages") === true

function _attached_component_types(outage::PSY.Outage, sys::PSY.System)
    return Set{DataType}(
        typeof(c) for c in PSY.get_associated_components(sys, outage)
    )
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
        return _includes_planned_outages(m)
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
    consumer::MonitoredOutageConsumer,
)
    for (comp_type, offending) in uncovered_types
        @warn "Monitored components of type $(comp_type) appear in outages \
               $(collect(offending)) but $(comp_type) is not \
               $(_admitted_types_description(consumer)); their \
               post-contingency variables will be skipped." _group =
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
`ServiceModel` in the template, in a single pass over each SC service model.

The user opts a service into responding to a given contingency by attaching
the outage supplemental attribute to the `PSY.Service` instance directly
(`add_supplemental_attribute!(sys, service, outage)`). That attachment is
the sole selection mechanism: a service claims exactly the outages attached
to it, regardless of whether the outaged component is among the service's
contributing devices.

`PlannedOutage`s attached to the service are still gated by the
`"include_planned_outages"` attribute on the SC `ServiceModel` (default
`false`). `UnplannedOutage`s and other `Outage` subtypes are always claimed.

A warning is emitted when an outage is attached to a `PSY.Service` whose
`(component_type, name)` does not correspond to an SC `ServiceModel` in the
template — that outage will not produce any post-contingency reserve
constraints.
"""
function _build_service_model_outages!(
    template::ProblemTemplate,
    sys::PSY.System,
)
    network_model = get_network_model(template)
    sc_service_models = _sc_reserve_service_models(template)
    template_sc_service_keys = Set{Tuple{DataType, String}}(
        (get_component_type(m), get_service_name(m)) for m in sc_service_models
    )
    # Run the orphan-attachment check unconditionally so users still get
    # feedback when they attached outages to a service whose SC `ServiceModel`
    # was never registered (e.g. they forgot the `set_service_model!` call, or
    # registered the service with a non-SC formulation).
    _warn_outages_attached_to_unmodeled_services(sys, template_sc_service_keys)

    isempty(sc_service_models) && return

    modeled_types = Set{DataType}(get_component_types(template))
    uncovered_types = Dict{DataType, Set{Base.UUID}}()

    for m in sc_service_models
        _validate_sc_reserve_direction(m)
        empty!(m.outages)
        D = get_component_type(m)
        service_name = get_service_name(m)
        service = PSY.get_component(D, sys, service_name)
        if isnothing(service)
            @warn "ServiceModel{$D, $(get_formulation(m))} (service_name=\
                   $(service_name)) is in the template but no matching \
                   service exists in the system; it will not contribute any \
                   post-contingency constraints." _group =
                LOG_GROUP_MODELS_VALIDATION
            continue
        end

        for outage in PSY.get_supplemental_attributes(PSY.Outage, service)
            outage_uuid = IS.get_uuid(outage)
            if isempty(PSY.get_monitored_components(outage))
                @warn "Outage $(outage_uuid) ($(typeof(outage))) attached to \
                       service $(service_name) has empty \
                       monitored_components; no post-contingency variables \
                       or constraints will be created for this outage." _group =
                    LOG_GROUP_MODELS_VALIDATION
                continue
            end
            _service_skips_outage(outage, m) && continue

            per_type, uncovered = _monitored_components_by_modeled_type(
                outage, outage_uuid, sys, modeled_types, ServiceOutageConsumer(),
            )
            for comp_type in uncovered
                push!(get!(uncovered_types, comp_type, Set{Base.UUID}()), outage_uuid)
            end
            isempty(per_type) && continue
            m.outages[outage_uuid] = per_type
        end
        _check_monitored_area_interchanges(template, network_model, sys, m)
    end

    _warn_uncovered_monitored_types(uncovered_types, ServiceOutageConsumer())
    return
end

function _sc_reserve_service_models(template::ProblemTemplate)
    return ServiceModel[
        m for m in values(get_service_models(template)) if
        get_formulation(m) <: AbstractSecurityConstrainedReservesFormulation
    ]
end

# Security-constrained reserve formulations currently model deployment as a
# strictly non-negative response to a generation shortfall (an outaged
# generator's power must be replaced upward). `ReserveDown`/`ReserveSymmetric`
# would need direction-consistent multipliers that have not been designed yet,
# so reject them here rather than build a silently wrong-signed model.
_validate_sc_reserve_direction(
    ::ServiceModel{
        <:PSY.Reserve{PSY.ReserveUp},
        <:AbstractSecurityConstrainedReservesFormulation,
    },
) = nothing
function _validate_sc_reserve_direction(
    m::ServiceModel{<:PSY.Reserve{D}, <:AbstractSecurityConstrainedReservesFormulation},
) where {D <: PSY.ReserveDirection}
    throw(
        IS.ConflictingInputsError(
            "ServiceModel{$(get_component_type(m)), $(get_formulation(m))} \
            (service_name=$(get_service_name(m))): security-constrained \
            reserve formulations currently support Reserve{ReserveUp} only; \
            got direction $D.",
        ),
    )
end

# Whether SC service model `m` should skip `outage`. Dispatched on the outage
# subtype to keep `PlannedOutage`/`UnplannedOutage` branching out of the hot
# path. Planned outages are skipped unless the SC service model opts in via
# the `"include_planned_outages"` attribute.
_service_skips_outage(::PSY.Outage, ::ServiceModel) = false
_service_skips_outage(::PSY.PlannedOutage, m::ServiceModel) =
    !_includes_planned_outages(m)

# Only the AreaBalance path builds a per-tie flow-deviation term, so only
# there must a monitored AreaInterchange be available, inside the network
# model's scope, and inside the template's AreaInterchange DeviceModel set:
# those are exactly the ties that get a Δf variable.
_check_monitored_area_interchanges(
    ::ProblemTemplate,
    ::NetworkModel,
    ::PSY.System,
    ::ServiceModel,
) = nothing

const _AREA_INTERCHANGE_MODEL_KEY = Symbol(IS.strip_module_name(PSY.AreaInterchange))

_has_area_interchange_device_model(template::ProblemTemplate) =
    haskey(get_branch_models(template), _AREA_INTERCHANGE_MODEL_KEY)

# Names of the AreaInterchanges the template's AreaInterchange `DeviceModel`
# actually models — the set the pre-contingency `FlowActivePowerVariable`, and
# therefore Δf, is built over. Empty when the template registers no such
# DeviceModel.
function _template_area_interchange_names(template::ProblemTemplate, sys::PSY.System)
    _has_area_interchange_device_model(template) || return Set{String}()
    device_model = get_branch_models(template)[_AREA_INTERCHANGE_MODEL_KEY]
    return Set{String}(
        PSY.get_name(x) for x in get_available_components(device_model, sys)
    )
end

function _check_monitored_area_interchanges(
    template::ProblemTemplate,
    network_model::NetworkModel{<:AreaBalancePowerModel},
    sys::PSY.System,
    m::ServiceModel,
)
    isempty(get_outages(m)) && return
    in_scope = Set{String}(
        PSY.get_name(x) for
        x in get_available_components(network_model, PSY.AreaInterchange, sys)
    )
    modeled = _template_area_interchange_names(template, sys)
    for (outage_uuid, per_type) in get_outages(m)
        haskey(per_type, PSY.AreaInterchange) || continue
        for name in per_type[PSY.AreaInterchange]
            if !(name in in_scope)
                throw(
                    IS.ConflictingInputsError(
                        "Monitored AreaInterchange \"$(name)\" (outage \
                        $(outage_uuid), service $(get_service_name(m))) is not among \
                        the available AreaInterchanges in the network model's scope. \
                        A monitored tie must be available, in the network model's \
                        scope, and included in the template's AreaInterchange \
                        DeviceModel: the post-contingency flow deviation is only \
                        built for those ties.",
                    ),
                )
            end
            name in modeled && continue
            throw(
                IS.ConflictingInputsError(
                    "Monitored AreaInterchange \"$(name)\" (outage $(outage_uuid), \
                    service $(get_service_name(m))) is not modeled by the template's \
                    AreaInterchange DeviceModel (either no DeviceModel is registered \
                    for PSY.AreaInterchange, its \"subsystem\" excludes this tie, or \
                    its \"filter_function\" excludes this tie), so it has no \
                    pre-contingency flow variable. A monitored tie must be \
                    available, in the network model's scope, and included in \
                    the template's AreaInterchange DeviceModel: the post-contingency \
                    flow deviation is only built for those ties.",
                ),
            )
        end
    end
    return
end

# Under a PTDF-based network model an AreaInterchange monitor never resolves
# to a post-contingency flow expression (there is no branch-arc entry for it,
# and the AreaBalance-only Δf mechanism does not apply) — `_resolve_service_
# monitored_arcs` warns and drops it, but only at build time. Warn here too so
# the same user error is reported at validation time instead of surfacing only
# once the model is built; the build-time warning stays as a backstop for
# direct internal-API callers that skip validation.
function _check_monitored_area_interchanges(
    ::ProblemTemplate,
    ::NetworkModel{<:AbstractPTDFModel},
    ::PSY.System,
    m::ServiceModel,
)
    isempty(get_outages(m)) && return
    dropped = Set{String}()
    for (_, per_type) in get_outages(m)
        haskey(per_type, PSY.AreaInterchange) || continue
        union!(dropped, per_type[PSY.AreaInterchange])
    end
    isempty(dropped) && return
    @warn "AreaInterchange monitor(s) $(sort!(collect(dropped))) dropped: no \
           post-contingency flow expression is built for them under a \
           PTDF-based network model. Use AreaBalancePowerModel to monitor \
           AreaInterchange ties." _group = LOG_GROUP_MODELS_VALIDATION
    return
end

function _warn_outages_attached_to_unmodeled_services(
    sys::PSY.System,
    template_sc_service_keys::Set{Tuple{DataType, String}},
)
    for service in PSY.get_components(PSY.Service, sys)
        attached_outages = PSY.get_supplemental_attributes(PSY.Outage, service)
        isempty(attached_outages) && continue
        key = (typeof(service), PSY.get_name(service))
        key in template_sc_service_keys && continue
        for outage in attached_outages
            @warn "Outage $(IS.get_uuid(outage)) is attached to service \
                   $(PSY.get_name(service)) ($(typeof(service))) but the \
                   template does not include a security-constrained \
                   ServiceModel for it; the outage will not contribute any \
                   post-contingency reserve constraints." _group =
                LOG_GROUP_MODELS_VALIDATION
        end
    end
    return
end
