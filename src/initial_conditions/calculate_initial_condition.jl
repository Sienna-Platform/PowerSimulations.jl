"""
    set_ic_quantity!(
        ic::InitialCondition{DeviceStatus, JuMP.VariableRef},
        var_value::Float64,
    )

Set the `DeviceStatus` initial condition by rounding the finite solver value
`var_value` before fixing `ic`. This removes small deviations from binary
endpoints caused by MIP solver tolerances.
"""
function set_ic_quantity!(
    ic::InitialCondition{DeviceStatus, JuMP.VariableRef},
    var_value::Float64,
)
    @assert isfinite(var_value) ic
    fix_parameter_value(ic.value, round(var_value))
    return
end

"""
    set_ic_quantity!(
        ic::InitialCondition{DeviceStatus, Float64},
        var_value::Float64,
    )

Set the `DeviceStatus` initial condition by rounding the finite solver value
`var_value` before storing it in `ic`. This removes small deviations from
binary endpoints caused by MIP solver tolerances.
"""
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
