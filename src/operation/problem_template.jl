
const DevicesModelContainer = Dict{Symbol, DeviceModel}
const ServicesModelContainer = Dict{Tuple{String, Symbol}, ServiceModel}

abstract type AbstractProblemTemplate end

"""
    ProblemTemplate(::Type{T}) where {T<:PM.AbstractPowerFormulation}

Creates a model reference of the PowerSimulations Optimization Problem.

# Arguments

  - `model::Type{T<:PM.AbstractPowerFormulation}`:

# Example

template = ProblemTemplate(CopperPlatePowerModel)
"""
mutable struct ProblemTemplate <: AbstractProblemTemplate
    network_model::NetworkModel{<:PM.AbstractPowerModel}
    devices::DevicesModelContainer
    branches::BranchModelContainer
    services::ServicesModelContainer
    function ProblemTemplate(network::NetworkModel{T}) where {T <: PM.AbstractPowerModel}
        new(
            network,
            DevicesModelContainer(),
            BranchModelContainer(),
            ServicesModelContainer(),
        )
    end
end

function Base.isempty(template::ProblemTemplate)
    if !isempty(template.devices)
        return false
    elseif !isempty(template.branches)
        return false
    elseif !isempty(template.services)
        return false
    else
        return true
    end
end

ProblemTemplate(::Type{T}) where {T <: PM.AbstractPowerModel} =
    ProblemTemplate(NetworkModel(T))
ProblemTemplate() = ProblemTemplate(CopperPlatePowerModel)

get_device_models(template::ProblemTemplate) = template.devices
get_branch_models(template::ProblemTemplate) = template.branches
get_service_models(template::ProblemTemplate) = template.services
get_network_model(template::ProblemTemplate) = template.network_model
get_network_formulation(template::ProblemTemplate) =
    get_network_formulation(get_network_model(template))
get_hvdc_network_model(template::ProblemTemplate) =
    template.network_model.hvdc_network_model

function get_component_types(template::ProblemTemplate)::Vector{DataType}
    return vcat(
        get_component_type.(values(get_device_models(template))),
        get_component_type.(values(get_branch_models(template))),
        get_component_type.(values(get_service_models(template))),
    )
end

function get_model(template::ProblemTemplate, ::Type{T}) where {T <: PSY.Device}
    if T <: PSY.Branch
        return get(template.branches, Symbol(T), nothing)
    elseif T <: PSY.Device
        return get(template.devices, Symbol(T), nothing)
    else
        error("Component $T not present in the template")
    end
end

function get_model(
    template::ProblemTemplate,
    ::Type{T},
    name::String = NO_SERVICE_NAME_PROVIDED,
) where {T <: PSY.Service}
    if haskey(template.services, (name, Symbol(T)))
        return template.services[(name, Symbol(T))]
    else
        error("Service $T $name not present in the template")
    end
end

# Note to devs. PSY exports set_model! these names are chosen to avoid name clashes

"""
Sets the network model in a template.
"""
function set_network_model!(
    template::ProblemTemplate,
    model::NetworkModel{<:PM.AbstractPowerModel},
)
    template.network_model = model
    return
end

"""
Sets the network model in a template.
"""
function set_hvdc_network_model!(
    template::ProblemTemplate,
    model::Union{Nothing, AbstractHVDCNetworkModel},
)
    set_hvdc_network_model!(template.network_model, model)
    return
end

"""
Sets the network model in a template.
"""
function set_hvdc_network_model!(
    template::ProblemTemplate,
    model::Type{U},
) where {U <: AbstractHVDCNetworkModel}
    set_hvdc_network_model!(template.network_model, model())
    return
end

"""
Sets the device model in a template using the component type and formulation.
Builds a default DeviceModel
"""
function set_device_model!(
    template::ProblemTemplate,
    component_type::Type{<:PSY.Device},
    formulation::Type{<:AbstractDeviceFormulation},
)
    set_device_model!(template, DeviceModel(component_type, formulation))
    return
end

"""
Sets the device model in a template using a DeviceModel instance
"""
function set_device_model!(
    template::ProblemTemplate,
    model::DeviceModel{<:PSY.Device, <:AbstractDeviceFormulation},
)
    _set_model!(template.devices, model)
    return
end

function set_device_model!(
    template::ProblemTemplate,
    model::DeviceModel{<:PSY.Branch, <:AbstractDeviceFormulation},
)
    _set_model!(template.branches, model)
    return
end

"""
Sets the service model in a template using a name and the service type and formulation.
Builds a default ServiceModel with use_service_name set to true.
"""
function set_service_model!(
    template::ProblemTemplate,
    service_name::String,
    service_type::Type{<:PSY.Service},
    formulation::Type{<:AbstractServiceFormulation},
)
    set_service_model!(
        template,
        service_name,
        ServiceModel(service_type, formulation; use_service_name = true),
    )
    return
end

"""
Sets the service model in a template using a ServiceModel instance.
"""
function set_service_model!(
    template::ProblemTemplate,
    service_type::Type{<:PSY.Service},
    formulation::Type{<:AbstractServiceFormulation},
)
    set_service_model!(template, ServiceModel(service_type, formulation))
    return
end

function set_service_model!(
    template::ProblemTemplate,
    service_name::String,
    model::ServiceModel{T, <:AbstractServiceFormulation},
) where {T <: PSY.Service}
    _set_model!(template.services, (service_name, Symbol(T)), model)
    return
end

function set_service_model!(
    template::ProblemTemplate,
    model::ServiceModel{<:PSY.Service, <:AbstractServiceFormulation},
)
    _set_model!(template.services, model)
    return
end

function _add_contributing_device_by_type!(
    service_model::ServiceModel,
    contributing_device::T,
    incompatible_device_types::Set{DataType},
    modeled_devices::Set{DataType},
) where {T <: PSY.Device}
    !PSY.get_available(contributing_device) && return
    if T ∈ incompatible_device_types || T ∉ modeled_devices
        return
    end
    push!(get!(get_contributing_devices_map(service_model), T, T[]), contributing_device)
    return
end

function _populate_contributing_devices!(template::ProblemTemplate, sys::PSY.System)
    service_models = get_service_models(template)
    isempty(service_models) && return

    device_models = get_device_models(template)
    branch_models = get_branch_models(template)
    # Type stability: explicitly type the Set to avoid widening to Set{Type}
    modeled_devices = Set{DataType}(get_component_type(m) for m in values(device_models))
    union!(modeled_devices, (get_component_type(m) for m in values(branch_models)))
    incompatible_device_types = get_incompatible_devices(device_models)
    services_mapping = PSY.get_contributing_device_mapping(sys)
    if isempty(keys(services_mapping))
        @warn "The system doesn't include any services. No services will be modeled, consider removing the service models from the template." _group =
            LOG_GROUP_SERVICE_CONSTUCTORS
        empty!(service_models)
        return
    end
    for (service_key, service_model) in service_models
        @debug "Populating service $(service_key)"
        empty!(get_contributing_devices_map(service_model))
        S = get_component_type(service_model)
        service = PSY.get_component(S, sys, get_service_name(service_model))
        if service === nothing
            @info "The data doesn't include services of type $(S) and name $(get_service_name(service_model)), consider changing the service models" _group =
                LOG_GROUP_SERVICE_CONSTUCTORS
            continue
        end
        service_devices_key = (type = S, name = PSY.get_name(service))
        contributing_devices_ =
            services_mapping[service_devices_key].contributing_devices
        for d in contributing_devices_
            _add_contributing_device_by_type!(
                service_model,
                d,
                incompatible_device_types,
                modeled_devices,
            )
        end
        if isempty(get_contributing_devices_map(service_model))
            @warn "The contributing devices for service $(PSY.get_name(service)) is empty, consider removing the service from the system" _group =
                LOG_GROUP_SERVICE_CONSTUCTORS
            continue
        end
    end
    return
end

function _modify_device_model!(
    devices_template::Dict{Symbol, DeviceModel},
    service_model::ServiceModel{<:PSY.Reserve, <:AbstractReservesFormulation},
    contributing_devices::Vector{<:PSY.Component},
)
    # Type stability: explicitly type the Set to avoid widening
    for dt in Set{DataType}(typeof.(contributing_devices))
        for device_model in values(devices_template)
            # add message here when it exists
            get_component_type(device_model) != dt && continue
            service_model in device_model.services && continue
            # type instability: pushing to vector of abstract type
            push!(device_model.services, service_model)
        end
    end

    return
end

function _modify_device_model!(
    ::Dict{Symbol, DeviceModel},
    ::ServiceModel{<:PSY.ReserveNonSpinning, <:AbstractReservesFormulation},
    ::Vector{<:PSY.Component},
)
    return
end

function _modify_device_model!(
    ::Dict{Symbol, DeviceModel},
    ::ServiceModel{PSY.TransmissionInterface, ConstantMaxInterfaceFlow},
    ::Vector,
)
    return
end

function _modify_device_model!(
    ::Dict{Symbol, DeviceModel},
    ::ServiceModel{PSY.TransmissionInterface, VariableMaxInterfaceFlow},
    ::Vector,
)
    return
end

function _add_services_to_device_model!(template::ProblemTemplate)
    service_models = get_service_models(template)
    devices_template = get_device_models(template)
    for (service_key, service_model) in service_models
        S = get_component_type(service_model)
        (S <: PSY.AGC || S <: PSY.ConstantReserveGroup) && continue
        contributing_devices = get_contributing_devices(service_model)
        isempty(contributing_devices) && continue
        _modify_device_model!(devices_template, service_model, contributing_devices)
    end
    return
end

function _populate_aggregated_service_model!(template::ProblemTemplate, sys::PSY.System)
    services_template = get_service_models(template)
    for (key, service_model) in services_template
        attributes = get_attributes(service_model)
        use_slacks = service_model.use_slacks
        duals = service_model.duals
        if pop!(attributes, "aggregated_service_model", false)
            delete!(services_template, key)
            D = get_component_type(service_model)
            B = get_formulation(service_model)
            for service in get_available_components(service_model, sys)
                new_key = (PSY.get_name(service), Symbol(D))
                if !haskey(services_template, new_key)
                    template.services[new_key] =
                        ServiceModel(
                            D,
                            B,
                            PSY.get_name(service);
                            use_slacks = use_slacks,
                            duals = duals,
                            attributes = attributes,
                        )
                else
                    error("Key $new_key already assigned in ServiceModel")
                end
            end
        end
    end
    return
end

function finalize_template!(template::ProblemTemplate, sys::PSY.System)
    _populate_aggregated_service_model!(template, sys)
    _populate_contributing_devices!(template, sys)
    _add_services_to_device_model!(template)
    return
end
