"""
Container for the initial condition data
"""
mutable struct InitialCondition{
    T <: InitialConditionType,
    U <: Union{JuMP.VariableRef, Float64, Nothing},
}
    component::PSY.Component
    value::U
end

function InitialCondition(
    ::Type{T},
    component::PSY.Component,
    value::U,
) where {T <: InitialConditionType, U <: Union{JuMP.VariableRef, Float64}}
    return InitialCondition{T, U}(component, value)
end

function InitialCondition(
    ::InitialConditionKey{T, U},
    component::U,
    value::V,
) where {
    T <: InitialConditionType,
    U <: PSY.Component,
    V <: Union{JuMP.VariableRef, Float64},
}
    return InitialCondition{T, U}(component, value)
end

function get_condition(p::InitialCondition{T, Float64}) where {T <: InitialConditionType}
    return p.value
end

function get_condition(
    p::InitialCondition{T, JuMP.VariableRef},
) where {T <: InitialConditionType}
    return jump_value(p.value)
end

function get_condition(
    ::InitialCondition{T, Nothing},
) where {T <: InitialConditionType}
    return
end

get_component(ic::InitialCondition) = ic.component
get_value(ic::InitialCondition) = ic.value
get_component_name(ic::InitialCondition) = PSY.get_name(ic.component)
get_component_type(ic::InitialCondition) = typeof(ic.component)
get_ic_type(
    ::Type{InitialCondition{T, U}},
) where {T <: InitialConditionType, U <: Union{JuMP.VariableRef, Float64, Nothing}} = T
get_ic_type(
    ::InitialCondition{T, U},
) where {T <: InitialConditionType, U <: Union{JuMP.VariableRef, Float64, Nothing}} = T

"""
Stores data to populate initial conditions before the build call
"""
mutable struct InitialConditionsData
    duals::Dict{ConstraintKey, AbstractArray}
    parameters::Dict{ParameterKey, AbstractArray}
    variables::Dict{VariableKey, AbstractArray}
    aux_variables::Dict{AuxVarKey, AbstractArray}
end

function InitialConditionsData()
    return InitialConditionsData(
        Dict{ConstraintKey, AbstractArray}(),
        Dict{ParameterKey, AbstractArray}(),
        Dict{VariableKey, AbstractArray}(),
        Dict{AuxVarKey, AbstractArray}(),
    )
end

function get_initial_condition_value(
    ic_data::InitialConditionsData,
    ::T,
    ::Type{U},
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    return ic_data.variables[VariableKey(T, U)]
end

function get_initial_condition_value(
    ic_data::InitialConditionsData,
    ::T,
    ::Type{U},
) where {T <: AuxVariableType, U <: Union{PSY.Component, PSY.System}}
    return ic_data.aux_variables[AuxVarKey(T, U)]
end

function get_initial_condition_value(
    ic_data::InitialConditionsData,
    ::T,
    ::Type{U},
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    return ic_data.duals[ConstraintKey(T, U)]
end

function get_initial_condition_value(
    ic_data::InitialConditionsData,
    ::T,
    ::Type{U},
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return ic_data.parameters[ParameterKey(T, U)]
end

function has_initial_condition_value(
    ic_data::InitialConditionsData,
    ::T,
    ::Type{U},
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    return haskey(ic_data.variables, VariableKey(T, U))
end

function has_initial_condition_value(
    ic_data::InitialConditionsData,
    ::T,
    ::Type{U},
) where {T <: AuxVariableType, U <: Union{PSY.Component, PSY.System}}
    return haskey(ic_data.aux_variables, AuxVarKey(T, U))
end

function has_initial_condition_value(
    ic_data::InitialConditionsData,
    ::T,
    ::Type{U},
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    return haskey(ic_data.duals, ConstraintKey(T, U))
end

function has_initial_condition_value(
    ic_data::InitialConditionsData,
    ::T,
    ::Type{U},
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return haskey(ic_data.parameters, ParameterKey(T, U))
end

######################### Initial Conditions Definitions#####################################
struct DevicePower <: InitialConditionType end
struct DeviceAboveMinPower <: InitialConditionType end
struct DeviceStatus <: InitialConditionType end
struct InitialTimeDurationOn <: InitialConditionType end
struct InitialTimeDurationOff <: InitialConditionType end
struct InitialEnergyLevel <: InitialConditionType end
struct AreaControlError <: InitialConditionType end

# Decide whether to run the initial conditions reconciliation algorithm based on the presence of any of these
requires_reconciliation(::Type{<:InitialConditionType}) = false

requires_reconciliation(::Type{InitialTimeDurationOn}) = true
requires_reconciliation(::Type{InitialTimeDurationOff}) = true
requires_reconciliation(::Type{DeviceStatus}) = true
requires_reconciliation(::Type{DevicePower}) = true # to capture a case when device is off in HA but producing power in ED
requires_reconciliation(::Type{DeviceAboveMinPower}) = true # ramping limits may make power differences in thermal compact devices between models infeasible
requires_reconciliation(::Type{InitialEnergyLevel}) = true # large differences in initial storage levels could lead to infeasibilities
# Not requiring reconciliation for AreaControlError
