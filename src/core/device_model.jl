"""
Formulation type to augment the power balance constraint expression with a time series parameter
"""
struct FixedOutput <: AbstractDeviceFormulation end

function _check_device_formulation(
    ::Type{D},
) where {D <: Union{AbstractDeviceFormulation, PSY.Device}}
    if !isconcretetype(D)
        throw(
            ArgumentError(
                "The device model must contain only concrete types, $(D) is an Abstract Type",
            ),
        )
    end
end

"""
    DeviceModel(
        ::Type{D},
        ::Type{B},
        feedforwards::Vector{<:AbstractAffectFeedforward}
        use_slacks::Bool,
        duals::Vector{DataType},
        services::Vector{ServiceModel}
        attributes::Dict{String, Any}
        outages::AbstractVector{<:PSY.Outage}
    )

Establishes the model for a particular device specified by type. Uses the keyword argument
feedforward to enable passing values between operation model at simulation time

# Arguments

  - `::Type{D} where D<:PSY.Device`: Power System Device Type
  - `::Type{B} where B<:AbstractDeviceFormulation`: Abstract Device Formulation
  - `feedforward::Array{<:AbstractAffectFeedforward} = Vector{AbstractAffectFeedforward}()` : use to pass parameters between models
  - `use_slacks::Bool = false` : Add slacks to the device model. Implementation is model dependent and not all models feature slacks
  - `duals::Vector{DataType} = Vector{DataType}()`: use to pass constraint type to calculate the duals. The DataType needs to be a valid ConstraintType
  - `time_series_names::Dict{Type{<:TimeSeriesParameter}, String} = get_default_time_series_names(D, B)` : use to specify time series names associated to the device`
  - `attributes::Dict{String, Any} = get_default_attributes(D, B)` : use to specify attributes to the device
  - `outages::AbstractVector{<:PSY.Outage} = PSY.Outage[]` : N-1 contingencies to model
    when the formulation is security-constrained. The constructor records the user's UUID
    selection; template validation later expands each entry to its per-type set of monitored
    component names. An empty default triggers auto-discovery of every outage in the system
    that monitors at least one component of type `D`. If `B` is not security-constrained,
    a non-empty value is dropped with a warning.

# Example
```julia
thermal_gens = DeviceModel(ThermalStandard, ThermalBasicUnitCommitment)
```
"""
mutable struct DeviceModel{D <: PSY.Device, B <: AbstractDeviceFormulation}
    feedforwards::Vector{<:AbstractAffectFeedforward}
    use_slacks::Bool
    duals::Vector{DataType}
    services::Vector{ServiceModel}
    time_series_names::Dict{Type{<:ParameterType}, String}
    attributes::Dict{String, Any}
    subsystem::Union{Nothing, String}
    events::Dict{EventKey, EventModel}
    outages::Dict{Base.UUID, Dict{DataType, Set{String}}}
    device_cache::Vector{D}
    function DeviceModel(
        ::Type{D},
        ::Type{B};
        feedforwards = Vector{AbstractAffectFeedforward}(),
        use_slacks = false,
        duals = Vector{DataType}(),
        time_series_names = get_default_time_series_names(D, B),
        attributes = Dict{String, Any}(),
        outages::AbstractVector{<:PSY.Outage} = PSY.Outage[],
    ) where {D <: PSY.Device, B <: AbstractDeviceFormulation}
        attributes_ = get_default_attributes(D, B)
        for (k, v) in attributes
            attributes_[k] = v
        end

        _check_device_formulation(D)
        _check_device_formulation(B)
        outages_field = _seed_device_model_outages(D, B, outages)
        new{D, B}(
            feedforwards,
            use_slacks,
            duals,
            Vector{ServiceModel}(),
            time_series_names,
            attributes_,
            nothing,
            Dict{EventKey, EventModel}(),
            outages_field,
            Vector{D}(),
        )
    end
end

function _seed_device_model_outages(
    ::Type{D},
    ::Type{B},
    outages::AbstractVector{<:PSY.Outage},
) where {D <: PSY.Device, B <: AbstractDeviceFormulation}
    field = Dict{Base.UUID, Dict{DataType, Set{String}}}()
    isempty(outages) && return field
    if !_formulation_supports_outages(B)
        @warn "DeviceModel{$D, $B}: 'outages' kwarg ignored — formulation does \
               not support N-1 contingencies."
        return field
    end
    for outage in outages
        field[IS.get_uuid(outage)] = Dict{DataType, Set{String}}()
    end
    return field
end

# Multi-dispatch flag for formulations that consume `DeviceModel.outages`.
# Default: false. Specialized to `true` for security-constrained branch
# formulations in `src/core/formulations.jl`-adjacent code.
_formulation_supports_outages(::Type{<:AbstractDeviceFormulation}) = false
_formulation_supports_outages(::Type{<:AbstractSecurityConstrainedStaticBranch}) = true

get_component_type(
    ::DeviceModel{D, B},
) where {D <: PSY.Device, B <: AbstractDeviceFormulation} = D
get_events(m::DeviceModel) = m.events
get_formulation(
    ::DeviceModel{D, B},
) where {D <: PSY.Device, B <: AbstractDeviceFormulation} = B
get_feedforwards(m::DeviceModel) = m.feedforwards
get_services(m::DeviceModel) = m.services
get_services(::Nothing) = nothing
get_use_slacks(m::DeviceModel) = m.use_slacks
get_duals(m::DeviceModel) = m.duals
get_time_series_names(m::DeviceModel) = m.time_series_names
get_attributes(m::DeviceModel) = m.attributes
get_attribute(::Nothing, ::String) = nothing
get_attribute(m::DeviceModel, key::String) = get(m.attributes, key, nothing)
get_subsystem(m::DeviceModel) = m.subsystem
get_outages(m::DeviceModel) = m.outages
get_device_cache(m::DeviceModel) = m.device_cache

set_subsystem!(m::DeviceModel, id::String) = m.subsystem = id

function _set_model!(
    dict::Dict,
    model::DeviceModel{D, B},
) where {D <: PSY.Device, B <: AbstractDeviceFormulation}
    key = Symbol(D)
    if haskey(dict, key)
        @warn "Overwriting $(D) existing model"
    end
    dict[key] = model
    return
end

has_service_model(model::DeviceModel) = !isempty(get_services(model))

function set_event_model!(
    model::DeviceModel{D, B},
    key::EventKey,
    event_model::EventModel,
) where {D <: PSY.Device, B <: AbstractDeviceFormulation}
    if haskey(model.events, key)
        error("EventModel $key already exists in model for device $D")
    end
    model.events[key] = event_model
    return
end
