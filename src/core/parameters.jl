abstract type ParameterAttributes end

struct NoAttributes end

struct TimeSeriesAttributes{T <: PSY.TimeSeriesData} <: ParameterAttributes
    name::String
    multiplier_id::Base.RefValue{Int}
    component_name_to_ts_uuid::Dict{String, String}
    subsystem::Base.RefValue{String}
end

function TimeSeriesAttributes(
    ::Type{T},
    name::String,
    multiplier_id::Int = 1,
    component_name_to_ts_uuid = Dict{String, String}(),
) where {T <: PSY.TimeSeriesData}
    return TimeSeriesAttributes{T}(
        name,
        Base.RefValue{Int}(multiplier_id),
        component_name_to_ts_uuid,
        Base.RefValue{String}(""),
    )
end

get_time_series_type(::TimeSeriesAttributes{T}) where {T <: PSY.TimeSeriesData} = T
get_time_series_name(attr::TimeSeriesAttributes) = attr.name
get_time_series_multiplier_id(attr::TimeSeriesAttributes) = attr.multiplier_id[]

get_subsystem(attr::TimeSeriesAttributes) = attr.subsystem[]
function set_subsystem!(attr::TimeSeriesAttributes, val::String)
    attr.subsystem[] = val
    return
end
set_subsystem!(::TimeSeriesAttributes, ::Nothing) = nothing

function add_component_name!(attr::TimeSeriesAttributes, name::String, uuid::String)
    if haskey(attr.component_name_to_ts_uuid, name)
        throw(ArgumentError("$name is already stored"))
    end

    attr.component_name_to_ts_uuid[name] = uuid
    return
end

get_component_names(attr::TimeSeriesAttributes) = keys(attr.component_name_to_ts_uuid)
function _get_ts_uuid(attr::TimeSeriesAttributes, name::String)
    if !haskey(attr.component_name_to_ts_uuid, name)
        throw(
            ArgumentError(
                "No time series UUID found for in attributes for component $name: available names are $(keys(attr.component_name_to_ts_uuid))",
            ),
        )
    end
    return attr.component_name_to_ts_uuid[name]
end

struct VariableValueAttributes{T <: OptimizationContainerKey} <: ParameterAttributes
    attribute_key::T
    affected_keys::Set
end

function VariableValueAttributes(key::T) where {T <: OptimizationContainerKey}
    return VariableValueAttributes{T}(key, Set())
end

get_attribute_key(attr::VariableValueAttributes) = attr.attribute_key

struct CostFunctionAttributes{T} <: ParameterAttributes
    variable_types::Tuple{Vararg{Type}}
    sos_status::SOSStatusVariable
    uses_compact_power::Bool
end

get_sos_status(attr::CostFunctionAttributes) = attr.sos_status
get_variable_types(attr::CostFunctionAttributes) = attr.variable_types
get_uses_compact_power(attr::CostFunctionAttributes) = attr.uses_compact_power

struct EventParametersAttributes{T <: PSY.Outage, U <: ParameterType} <: ParameterAttributes
    affected_devices::Vector{<:PSY.Component}
end

function get_param_type(
    ::EventParametersAttributes{T, U},
) where {T <: PSY.Outage, U <: ParameterType}
    return U
end

struct ParameterContainer{T <: AbstractArray, U <: AbstractArray}
    attributes::ParameterAttributes
    parameter_array::T
    multiplier_array::U
end

function ParameterContainer(parameter_array, multiplier_array)
    return ParameterContainer(NoAttributes(), parameter_array, multiplier_array)
end

function calculate_parameter_values(container::ParameterContainer)
    return calculate_parameter_values(
        container.attributes,
        container.parameter_array,
        container.multiplier_array,
    )
end

function calculate_parameter_values(
    attributes::ParameterAttributes,
    param_array::DenseAxisArray,
    multiplier_array::DenseAxisArray,
)
    return get_parameter_values(attributes, param_array, multiplier_array) .*
           multiplier_array
end

function calculate_parameter_values(
    ::ParameterAttributes,
    param_array::SparseAxisArray,
    multiplier_array::SparseAxisArray,
)
    p_array = jump_value.(to_matrix(param_array))
    m_array = to_matrix(multiplier_array)
    return p_array .* m_array
end

function get_parameter_column_refs(container::ParameterContainer, column::AbstractString)
    return get_parameter_column_refs(
        container.attributes,
        container.parameter_array,
        column,
    )
end

function get_parameter_column_refs(::ParameterAttributes, param_array, column)
    return param_array
end

function get_parameter_column_refs(
    attributes::TimeSeriesAttributes{T},
    param_array::DenseAxisArray,
    column,
) where {T <: PSY.TimeSeriesData}
    expand_ixs((_get_ts_uuid(attributes, column),), param_array)
    return param_array[expand_ixs((_get_ts_uuid(attributes, column),), param_array)...]
end

function get_parameter_column_values(container::ParameterContainer, column::AbstractString)
    return jump_value.(get_parameter_column_refs(container, column))
end

function get_parameter_values(container::ParameterContainer)
    return get_parameter_values(
        container.attributes,
        container.parameter_array,
        container.multiplier_array,
    )
end

# TODO: SparseAxisArray versions of these functions

function get_parameter_values(
    ::ParameterAttributes,
    param_array::DenseAxisArray,
    multiplier_array::DenseAxisArray,
)
    return (.*).(jump_value.(param_array), multiplier_array)
end

function get_parameter_values(
    attr::EventParametersAttributes,
    param_array::DenseAxisArray,
    multiplier_array::DenseAxisArray,
)
    return jump_value.(param_array)
end

function get_parameter_values(
    attributes::TimeSeriesAttributes{T},
    param_array::DenseAxisArray,
    multiplier_array::DenseAxisArray,
) where {T <: PSY.TimeSeriesData}
    exploded_param_array = DenseAxisArray{Float64}(undef, axes(multiplier_array)...)
    for name in axes(multiplier_array)[1]
        param_col = param_array[_get_ts_uuid(attributes, name), axes(param_array)[2:end]...]
        device_axes = axes(multiplier_array)[2:end]
        exploded_param_array[name, device_axes...] = jump_value.(param_col)
    end

    return exploded_param_array
end

get_parameter_array(c::ParameterContainer) = c.parameter_array
get_multiplier_array(c::ParameterContainer) = c.multiplier_array
get_attributes(c::ParameterContainer) = c.attributes
Base.length(c::ParameterContainer) = length(c.parameter_array)
Base.size(c::ParameterContainer) = size(c.parameter_array)

function get_column_names(key::ParameterKey, c::ParameterContainer)
    return get_column_names_from_axis_array(key, get_multiplier_array(c))
end

const ValidDataParamEltypes = Union{Float64, Tuple{Vararg{Float64}}}
function _set_parameter!(
    array::AbstractArray{T},
    ::JuMP.Model,
    value::Union{T, AbstractVector{T}},
    ixs::Tuple,
) where {T <: ValidDataParamEltypes}
    assign_maybe_broadcast!(array, value, ixs)
    return
end

function _set_parameter!(
    array::AbstractArray{JuMP.VariableRef},
    model::JuMP.Model,
    value::Union{T, AbstractVector{T}},
    ixs::Tuple,
) where {T <: ValidDataParamEltypes}
    assign_maybe_broadcast!(array, add_jump_parameter.(Ref(model), value), ixs)
    return
end

function _set_parameter!(
    array::SparseAxisArray{Union{Nothing, JuMP.VariableRef}},
    model::JuMP.Model,
    value::Union{T, AbstractVector{T}},
    ixs::Tuple,
) where {T <: ValidDataParamEltypes}
    assign_maybe_broadcast!(array, add_jump_parameter.(Ref(model), value), ixs)
    return
end

function set_multiplier!(container::ParameterContainer, multiplier::Float64, ixs...)
    assign_maybe_broadcast!(get_multiplier_array(container), multiplier, ixs)
    return
end

function set_parameter!(
    container::ParameterContainer,
    jump_model::JuMP.Model,
    parameter::Union{ValidDataParamEltypes, AbstractVector{<:ValidDataParamEltypes}},
    ixs...,
)
    param_array = get_parameter_array(container)
    _set_parameter!(param_array, jump_model, parameter, ixs)
    return
end

# Overload for when a JuMP parameter VariableRef is passed directly (recurrent-solve
# path where a parallel branch type reuses the VariableRef created by the first type).
function set_parameter!(
    container::ParameterContainer,
    ::JuMP.Model,
    parameter::JuMP.VariableRef,
    ixs...,
)
    assign_maybe_broadcast!(get_parameter_array(container), parameter, ixs)
    return
end

"""
Parameter to define active power time series
"""
struct ActivePowerTimeSeriesParameter <: TimeSeriesParameter end

"""
Parameter to define reactive power time series
"""
struct ReactivePowerTimeSeriesParameter <: TimeSeriesParameter end

"""
Parameter to define active power out time series
"""
struct ActivePowerOutTimeSeriesParameter <: TimeSeriesParameter end

"""
Parameter to define active power in time series
"""
struct ActivePowerInTimeSeriesParameter <: TimeSeriesParameter end

"""
Parameter to define requirement time series
"""
struct RequirementTimeSeriesParameter <: TimeSeriesParameter end

"""
Abstract type for dynamic ratings of AC branches
"""
abstract type AbstractDynamicBranchRatingTimeSeriesParameter <: TimeSeriesParameter end

"""
Parameter to define the dynamic rating time series of a branch
"""
struct DynamicBranchRatingTimeSeriesParameter <:
       AbstractDynamicBranchRatingTimeSeriesParameter end

"""
Parameter to define the dynamic ratings time series of an AC branch for post-contingency condition
"""
struct PostContingencyDynamicBranchRatingTimeSeriesParameter <:
       AbstractDynamicBranchRatingTimeSeriesParameter end

"""
Parameter to define Flow From_To limit time series
"""
struct FromToFlowLimitParameter <: TimeSeriesParameter end

"""
Parameter to define Flow To_From limit time series
"""
struct ToFromFlowLimitParameter <: TimeSeriesParameter end

"""
Parameter to define Max Flow limit for interface time series
"""
struct MaxInterfaceFlowLimitParameter <: TimeSeriesParameter end

"""
Parameter to define Min Flow limit for interface time series
"""
struct MinInterfaceFlowLimitParameter <: TimeSeriesParameter end

abstract type VariableValueParameter <: RightHandSideParameter end

"""
Parameter to define variable upper bound
"""
struct UpperBoundValueParameter <: VariableValueParameter end

"""
Parameter to define variable lower bound
"""
struct LowerBoundValueParameter <: VariableValueParameter end

"""
Parameter to define unit commitment status updated from the system state
"""
struct OnStatusParameter <: VariableValueParameter end

"""
Parameter to FixValueParameter
"""
struct FixValueParameter <: VariableValueParameter end

"""
Parameter to define cost function coefficient
"""
struct CostFunctionParameter <: ObjectiveFunctionParameter end

"""
Parameter to define fuel cost time series
"""
struct FuelCostParameter <: ObjectiveFunctionParameter end

"Parameter to define startup cost time series"
struct StartupCostParameter <: ObjectiveFunctionParameter end

"Parameter to define shutdown cost time series"
struct ShutdownCostParameter <: ObjectiveFunctionParameter end

"Parameters to define the cost at the minimum available power"
abstract type AbstractCostAtMinParameter <: ObjectiveFunctionParameter end

"[`AbstractCostAtMinParameter`](@ref) for the incremental case (power source)"
struct IncrementalCostAtMinParameter <: AbstractCostAtMinParameter end

"[`AbstractCostAtMinParameter`](@ref) for the decremental case (power sink)"
struct DecrementalCostAtMinParameter <: AbstractCostAtMinParameter end

"Parameters to define the slopes of a piecewise linear cost function"
abstract type AbstractPiecewiseLinearSlopeParameter <: ObjectiveFunctionParameter end

"[`AbstractPiecewiseLinearSlopeParameter`](@ref) for the incremental case (power source)"
struct IncrementalPiecewiseLinearSlopeParameter <: AbstractPiecewiseLinearSlopeParameter end

"[`AbstractPiecewiseLinearSlopeParameter`](@ref) for the decremental case (power sink)"
struct DecrementalPiecewiseLinearSlopeParameter <: AbstractPiecewiseLinearSlopeParameter end

"Parameters to define the breakpoints of a piecewise linear function"
abstract type AbstractPiecewiseLinearBreakpointParameter <: TimeSeriesParameter end

"[`AbstractPiecewiseLinearBreakpointParameter`](@ref) for the incremental case (power source)"
struct IncrementalPiecewiseLinearBreakpointParameter <:
       AbstractPiecewiseLinearBreakpointParameter end

"[`AbstractPiecewiseLinearBreakpointParameter`](@ref) for the decremental case (power sink)"
struct DecrementalPiecewiseLinearBreakpointParameter <:
       AbstractPiecewiseLinearBreakpointParameter end

abstract type AuxVariableValueParameter <: RightHandSideParameter end

abstract type EventParameter <: ParameterType end

"""
Parameter to define component availability status updated from the system state
"""
struct AvailableStatusParameter <: EventParameter end

"""
Parameter to define active power offset during an event.
"""
struct ActivePowerOffsetParameter <: EventParameter end

"""
Parameter to define reactive power offset during an event.
"""
struct ReactivePowerOffsetParameter <: EventParameter end

"""
Parameter to record that the component changed in the availability status
"""
struct AvailableStatusChangeCountdownParameter <: EventParameter end

should_write_resulting_value(::Type{<:RightHandSideParameter}) = true
should_write_resulting_value(::Type{<:EventParameter}) = true

should_write_resulting_value(::Type{<:FuelCostParameter}) = true
should_write_resulting_value(::Type{<:ShutdownCostParameter}) = true
should_write_resulting_value(::Type{<:AbstractCostAtMinParameter}) = true
should_write_resulting_value(::Type{<:AbstractPiecewiseLinearSlopeParameter}) = true
should_write_resulting_value(::Type{<:AbstractPiecewiseLinearBreakpointParameter}) = true

convert_result_to_natural_units(::Type{DynamicBranchRatingTimeSeriesParameter}) = true
convert_result_to_natural_units(
    ::Type{PostContingencyDynamicBranchRatingTimeSeriesParameter},
) = true
convert_result_to_natural_units(::Type{ActivePowerTimeSeriesParameter}) = true
convert_result_to_natural_units(::Type{ReactivePowerTimeSeriesParameter}) = true
convert_result_to_natural_units(::Type{ActivePowerOutTimeSeriesParameter}) = true
convert_result_to_natural_units(::Type{ActivePowerInTimeSeriesParameter}) = true
convert_result_to_natural_units(::Type{RequirementTimeSeriesParameter}) = true
convert_result_to_natural_units(::Type{UpperBoundValueParameter}) = true
convert_result_to_natural_units(::Type{LowerBoundValueParameter}) = true
