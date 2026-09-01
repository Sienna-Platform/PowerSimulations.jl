# MIP solver tolerances can leave binary values near their endpoints.
function set_ic_quantity!(
    ic::InitialCondition{DeviceStatus, JuMP.VariableRef},
    var_value::Float64,
)
    @assert isfinite(var_value) ic
    fix_parameter_value(ic.value, round(var_value))
    return
end

function set_ic_quantity!(
    ic::InitialCondition{DeviceStatus, Float64},
    var_value::Float64,
)
    @assert isfinite(var_value) ic
    @debug "Initial condition value set with Float64. Won't update the model until rebuild" _group =
        LOG_GROUP_BUILD_INITIAL_CONDITIONS
    ic.value = round(var_value)
    return
end

"""
Default implementation of set_initial_condition_value
"""
function set_ic_quantity!(
    ic::InitialCondition{T, JuMP.VariableRef},
    var_value::Float64,
) where {T <: InitialConditionType}
    @assert isfinite(var_value) ic
    fix_parameter_value(ic.value, var_value)
    return
end

"""
Default implementation of set_initial_condition_value
"""
function set_ic_quantity!(
    ic::InitialCondition{T, Float64},
    var_value::Float64,
) where {T <: InitialConditionType}
    @assert isfinite(var_value) ic
    @debug "Initial condition value set with Float64. Won't update the model until rebuild" _group =
        LOG_GROUP_BUILD_INITIAL_CONDITIONS
    ic.value = var_value
    return
end

function set_ic_quantity!(
    ::InitialCondition{T, Nothing},
    ::Float64,
) where {T <: InitialConditionType}
    return
end
