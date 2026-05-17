function _any_component_has_branch_rating_ts(device_model::DeviceModel)
    ts_name = get(
        get_time_series_names(device_model),
        BranchRatingTimeSeriesParameter,
        nothing,
    )
    isnothing(ts_name) && return false
    for c in get_device_cache(device_model)
        if PSY.has_time_series(c, PSY.SingleTimeSeries, ts_name) ||
           PSY.has_time_series(c, PSY.Deterministic, ts_name)
            return true
        end
    end
    return false
end

function _check_branch_rating_time_series_formulation!(branch_models::Dict)
    for (_, device_model) in branch_models
        _any_component_has_branch_rating_ts(device_model) || continue
        D = get_component_type(device_model)
        B = get_formulation(device_model)
        if B <: StaticBranchUnbounded
            @warn "BranchRatingTimeSeriesParameter time series attached to \
                   $(D) components will be ignored: $(B) does not enforce flow \
                   limits."
        elseif B <: StaticBranch || B <: AbstractSecurityConstrainedStaticBranch
            continue
        else
            throw(
                IS.ConflictingInputsError(
                    "BranchRatingTimeSeriesParameter is only supported with the \
                    StaticBranch or AbstractSecurityConstrainedStaticBranch \
                    formulations, but branch type $(D) was configured with $(B). \
                    Remove the branch rating time series from the components or \
                    change the formulation.",
                ),
            )
        end
    end
    return
end

function _check_security_constrained_three_winding_transformer!(branch_models::Dict)
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
    for (k, device_model) in model.template.branches
        make_device_cache!(device_model, system, get_check_components(settings))
        if isempty(get_device_cache(device_model))
            @info "The system data doesn't include Branches of type $(k), consider changing the models in the template" _group =
                LOG_GROUP_MODELS_VALIDATION
            push!(branch_keys_to_delete, k)
        else
            push!(network_model.modeled_ac_branch_types, get_component_type(device_model))
        end
        if !isnothing(get_attribute(device_model, "filter_function"))
            model_has_branch_filters = true
        end
    end
    for k in branch_keys_to_delete
        delete!(model.template.branches, k)
    end
    _check_branch_rating_time_series_formulation!(model.template.branches)
    _check_security_constrained_three_winding_transformer!(model.template.branches)
    validate_network_model(network_model, unmodeled_branch_types, model_has_branch_filters)
    _build_device_model_outages!(template, system)
    return
end

# Multi-dispatch outage subtype tag, avoids inline `isa` in the auto-discover
# path. Default catch-all is `false`; specialized to `true` for `PlannedOutage`.
_is_planned_outage(::PSY.PlannedOutage) = true
_is_planned_outage(::PSY.Outage) = false

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

Empty `monitored_components` on an outage is treated as "monitor nothing" — a
warning is emitted. A monitored component whose type is not modeled by the
template is reported once per type with the offending outage UUIDs and is
silently skipped: no post-contingency variables or constraints are built for
that monitored component under any outage.
"""
function _build_device_model_outages!(
    template::ProblemTemplate,
    sys::PSY.System,
)
    # This code needs to be extended when adding G-1 models to check for injectors too.
    branch_models = get_branch_models(template)
    sc_models = [
        m for m in values(branch_models) if
        get_formulation(m) <: AbstractSecurityConstrainedStaticBranch
    ]
    isempty(sc_models) && return

    modeled_types = Set{DataType}(get_component_types(template))

    selection = Dict{Symbol, Union{Nothing, Set{Base.UUID}}}()
    for m in sc_models
        sym = Symbol(get_component_type(m))
        selection[sym] = if isempty(m.outages)
            nothing
        else
            Set{Base.UUID}(keys(m.outages))
        end
        empty!(m.outages)
    end

    uncovered_types = Dict{DataType, Set{Base.UUID}}()

    for outage in PSY.get_supplemental_attributes(PSY.Outage, sys)
        outage_uuid = IS.get_uuid(outage)
        monitored = PSY.get_monitored_components(outage)
        if isempty(monitored)
            @warn "Outage $(outage_uuid) ($(typeof(outage))) has empty \
                   monitored_components; no post-contingency variables or \
                   constraints will be created for this outage." _group =
                LOG_GROUP_MODELS_VALIDATION
            continue
        end

        per_type = Dict{DataType, Set{String}}()
        for uuid in monitored
            component = IS.get_component(sys, uuid)
            if isnothing(component)
                @warn "Outage $(outage_uuid) references monitored component \
                       UUID $(uuid) that is not present in the system; \
                       skipping." _group = LOG_GROUP_MODELS_VALIDATION
                continue
            end
            comp_type = typeof(component)
            if !(comp_type in modeled_types)
                push!(
                    get!(uncovered_types, comp_type, Set{Base.UUID}()),
                    outage_uuid,
                )
                continue
            end
            push!(get!(per_type, comp_type, Set{String}()), PSY.get_name(component))
        end
        isempty(per_type) && continue

        # DeviceModel{D, SC} claims the outage iff D is among the OUTAGED
        # (attached) component types. Multi-component outages get claimed by
        # every matching SC DeviceModel; the post-contingency build dedups
        # by referencing the first claimer's expressions/constraints rather
        # than recomputing.
        attached_types = Set{DataType}(
            typeof(c) for c in PSY.get_associated_components(sys, outage)
        )
        has_matching_sc_model = false
        for m in sc_models
            D = get_component_type(m)
            D in attached_types || continue
            has_matching_sc_model = true
            sel = selection[Symbol(D)]
            if !isnothing(sel)
                outage_uuid in sel || continue
            else
                include_planned =
                    get_attribute(m, "include_planned_outages") === true
                if _is_planned_outage(outage) && !include_planned
                    continue
                end
            end
            m.outages[outage_uuid] = per_type
        end
        if !has_matching_sc_model
            @warn "Outage $(outage_uuid) is attached to component(s) of \
                   type $(collect(attached_types)), but no DeviceModel with \
                   an AbstractSecurityConstrainedStaticBranch formulation \
                   covers those types; it will not contribute any \
                   post-contingency constraints." _group =
                LOG_GROUP_MODELS_VALIDATION
        end
    end

    for (comp_type, offending) in uncovered_types
        @warn "Monitored components of type $(comp_type) appear in outages \
               $(collect(offending)) but $(comp_type) is not modeled by the \
               template; their post-contingency variables will be skipped." _group =
            LOG_GROUP_MODELS_VALIDATION
    end

    for m in sc_models
        D = get_component_type(m)
        sel = selection[Symbol(D)]
        isnothing(sel) && continue
        for uuid in sel
            if !haskey(m.outages, uuid)
                @warn "Outage $(uuid) listed on DeviceModel{$D, \
                       $(get_formulation(m))} is not attached to a component \
                       of type $D in the system — it will not contribute any \
                       post-contingency constraints." _group =
                    LOG_GROUP_MODELS_VALIDATION
            end
        end
    end
    return
end
