function _check_service_formulation(
    ::Type{D},
) where {D <: Union{AbstractServiceFormulation, PSY.Service}}
    if !isconcretetype(D)
        throw(
            ArgumentError(
                "The service model must contain only concrete types, $(D) is an Abstract Type",
            ),
        )
    end
end

"""
Establishes the model for a particular services specified by type. Uses the keyword argument
`use_service_name` to assign the model to a service with the same name as the name in the
template. Uses the keyword argument feedforward to enable passing values between operation
model at simulation time

# Arguments

-`::Type{D}`: Power System Service Type
-`::Type{B}`: Abstract Service Formulation

# Accepted Key Words

  - `feedforward::Array{<:AbstractAffectFeedforward}` : use to pass parameters between models
  - `use_service_name::Bool` : use the name as the name for the service
  - `outages::AbstractVector{<:PSY.Outage}` : G-1 contingencies to model when the
    formulation is security-constrained. The constructor stores `IS.get_uuid(outage)`
    of each entry as a key in the model's `outages::Dict{UUID, Dict{DataType, Set{String}}}`
    field with empty inner maps; template validation fills the inner maps with the
    per-type set of monitored component names (e.g., branches) that each outage
    carries. An empty default triggers auto-discovery of every outage in the system
    attached to a contributing device of the service whose formulation supports
    outages. If `B` is not security-constrained, a non-empty value is dropped with a
    warning.

# Example

reserves = ServiceModel(PSY.VariableReserve{PSY.ReserveUp}, RangeReserve)
"""
mutable struct ServiceModel{D <: PSY.Service, B <: AbstractServiceFormulation}
    feedforwards::Vector{<:AbstractAffectFeedforward}
    service_name::String
    use_slacks::Bool
    duals::Vector{DataType}
    time_series_names::Dict{Type{<:TimeSeriesParameter}, String}
    attributes::Dict{String, Any}
    contributing_devices_map::Dict{Type{<:PSY.Component}, Vector{<:PSY.Component}}
    subsystem::Union{Nothing, String}
    outages::Dict{Base.UUID, Dict{DataType, Set{String}}}
    function ServiceModel(
        ::Type{D},
        ::Type{B},
        service_name::String;
        use_slacks = false,
        feedforwards = Vector{AbstractAffectFeedforward}(),
        duals = Vector{DataType}(),
        time_series_names = get_default_time_series_names(D, B),
        attributes = Dict{String, Any}(),
        contributing_devices_map = Dict{Type{<:PSY.Component}, Vector{<:PSY.Component}}(),
        outages::AbstractVector{<:PSY.Outage} = PSY.Outage[],
    ) where {D <: PSY.Service, B <: AbstractServiceFormulation}
        attributes_for_model = get_default_attributes(D, B)
        for (k, v) in attributes
            attributes_for_model[k] = v
        end

        _check_service_formulation(D)
        _check_service_formulation(B)
        outages_field = _add_service_model_outages(D, B, outages)
        new{D, B}(
            feedforwards,
            service_name,
            use_slacks,
            duals,
            time_series_names,
            attributes_for_model,
            contributing_devices_map,
            nothing,
            outages_field,
        )
    end
end

function _add_service_model_outages(
    ::Type{D},
    ::Type{B},
    outages::AbstractVector{<:PSY.Outage},
) where {D <: PSY.Service, B <: AbstractServiceFormulation}
    field = Dict{Base.UUID, Dict{DataType, Set{String}}}()
    isempty(outages) && return field
    if !_formulation_supports_outages(B)
        @warn "ServiceModel{$D, $B}: 'outages' kwarg ignored \u2014 formulation \
               does not support G-1 contingencies."
        return field
    end
    for outage in outages
        field[IS.get_uuid(outage)] = Dict{DataType, Set{String}}()
    end
    return field
end

_formulation_supports_outages(
    ::Type{<:AbstractSecurityConstrainedReservesFormulation},
) = true

get_component_type(
    ::ServiceModel{D, B},
) where {D <: PSY.Service, B <: AbstractServiceFormulation} = D
get_formulation(
    ::ServiceModel{D, B},
) where {D <: PSY.Service, B <: AbstractServiceFormulation} = B
get_feedforwards(m::ServiceModel) = m.feedforwards
get_service_name(m::ServiceModel) = m.service_name
get_use_slacks(m::ServiceModel) = m.use_slacks
get_duals(m::ServiceModel) = m.duals
get_time_series_names(m::ServiceModel) = m.time_series_names
get_attributes(m::ServiceModel) = m.attributes
get_attribute(m::ServiceModel, key::String) = get(m.attributes, key, nothing)
get_contributing_devices_map(m::ServiceModel) = m.contributing_devices_map
get_contributing_devices_map(m::ServiceModel, key) =
    get(m.contributing_devices_map, key, nothing)
get_contributing_devices(m::ServiceModel) =
    [z for x in values(m.contributing_devices_map) for z in x]
get_subsystem(m::ServiceModel) = m.subsystem
get_outages(m::ServiceModel) = m.outages

set_subsystem!(m::ServiceModel, id::String) = m.subsystem = id

function ServiceModel(
    service_type::Type{D},
    formulation_type::Type{B};
    use_slacks = false,
    feedforwards = Vector{AbstractAffectFeedforward}(),
    duals = Vector{DataType}(),
    time_series_names = get_default_time_series_names(D, B),
    attributes = get_default_attributes(D, B),
    outages::AbstractVector{<:PSY.Outage} = PSY.Outage[],
) where {D <: PSY.Service, B <: AbstractServiceFormulation}
    # If more attributes are used later, move free form string to const and organize
    # attributes
    attributes_for_model = get_default_attributes(D, B)
    for (k, v) in attributes
        attributes_for_model[k] = v
    end
    if !haskey(attributes_for_model, "aggregated_service_model")
        push!(attributes_for_model, "aggregated_service_model" => true)
    end
    return ServiceModel(
        service_type,
        formulation_type,
        NO_SERVICE_NAME_PROVIDED;
        use_slacks,
        feedforwards,
        duals,
        time_series_names,
        attributes = attributes_for_model,
        outages,
    )
end

function _set_model!(dict::Dict, key::Tuple{String, Symbol}, model::ServiceModel)
    if haskey(dict, key)
        @warn "Overwriting $(key) existing model"
    end
    dict[key] = model
    return
end

function _set_model!(
    dict::Dict,
    model::ServiceModel{D, B},
) where {D <: PSY.Service, B <: AbstractServiceFormulation}
    _set_model!(dict, (get_service_name(model), Symbol(D)), model)
    return
end
