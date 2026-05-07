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
        if get_attribute(device_model, "filter_function") !== nothing
            model_has_branch_filters = true
        end
    end
    for k in branch_keys_to_delete
        delete!(model.template.branches, k)
    end
    validate_network_model(network_model, unmodeled_branch_types, model_has_branch_filters)
    _build_post_contingency_flow_components!(network_model, template, system)
    return
end

"""
Populate `network_model.post_contingency_flow_components` with the per-outage,
per-type set of monitored branch names. Skipped when the template does not use
a security-constrained branch formulation. Empty `monitored_components` on an
outage is treated as "monitor nothing" — a warning is emitted so users notice.
A monitored component whose type is not modeled by the template is reported
once per type with the offending outage UUIDs.
"""
function _build_post_contingency_flow_components!(
    network_model::NetworkModel,
    template::ProblemTemplate,
    sys::PSY.System,
)
    branch_models = get_branch_models(template)
    if !_template_uses_security_constrained_branch(branch_models)
        return
    end

    flowgates = get_post_contingency_flow_components(network_model)
    empty!(flowgates)

    modeled_types = Set{DataType}(get_component_types(template))
    uncovered_types = Dict{DataType, Vector{Base.UUID}}()

    for outage in PSY.get_supplemental_attributes(PSY.Outage, sys)
        outage_uuid = IS.get_uuid(outage)
        monitored = PSY.get_monitored_components(outage)
        if isempty(monitored)
            @warn "Outage $(outage_uuid) ($(typeof(outage))) has empty monitored_components; no post-contingency variables or constraints will be created for this outage." _group =
                LOG_GROUP_MODELS_VALIDATION
            continue
        end

        per_type = Dict{DataType, Set{String}}()
        for uuid in monitored
            component = IS.get_component(sys, uuid)
            if component === nothing
                @warn "Outage $(outage_uuid) references monitored component UUID $(uuid) that is not present in the system; skipping." _group =
                    LOG_GROUP_MODELS_VALIDATION
                continue
            end
            comp_type = typeof(component)
            if !(comp_type in modeled_types)
                push!(get!(uncovered_types, comp_type, Base.UUID[]), outage_uuid)
                continue
            end
            push!(get!(per_type, comp_type, Set{String}()), PSY.get_name(component))
        end

        if !isempty(per_type)
            flowgates[outage_uuid] = per_type
        end
    end

    for (comp_type, offending) in uncovered_types
        unique_outages = unique(offending)
        @warn "Monitored components of type $(comp_type) appear in outages $(unique_outages) but $(comp_type) is not modeled by the template; their post-contingency variables will be skipped." _group =
            LOG_GROUP_MODELS_VALIDATION
    end
    return
end
