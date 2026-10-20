function IOM.update_initial_conditions!(
    model::IOM.AbstractOptimizationModel,
    state::SimulationState,
    ::InterProblemChronology,
)
    for key in keys(get_initial_conditions(model))
        IOM.update_initial_conditions!(model, key, state)
    end
    return
end

function IOM.update_initial_conditions!(
    ::IOM.AbstractOptimizationModel,
    ::SimulationState,
    ::IntraProblemChronology,
)
    error("Not Implemented yet")
    return
end

function IOM.update_initial_conditions!(
    ics::T,
    state::SimulationState,
    model_resolution::Dates.Millisecond,
) where {
    T <: Union{
        Vector{
            Union{
                InitialCondition{InitialTimeDurationOn, Nothing},
                InitialCondition{InitialTimeDurationOn, Float64},
            },
        },
        Vector{
            Union{
                InitialCondition{InitialTimeDurationOn, Nothing},
                InitialCondition{InitialTimeDurationOn, JuMP.VariableRef},
            },
        },
    },
}
    isempty(ics) && return
    component_type = get_component_type(first(ics))
    key = AuxVarKey(TimeDurationOn, component_type)
    var_val = get_system_state_value(state, key)
    state_resolution = get_data_resolution(get_system_state_data(state, key))
    # The state data is stored in the state resolution (i.e. lowest resolution among all models)
    # so this step scales the data to the model resolution.
    scale = model_resolution / state_resolution
    for ic in ics
        set_ic_quantity!(ic, _get_state_value(var_val, key, get_component_name(ic)) / scale)
    end
    return
end

function IOM.update_initial_conditions!(
    ics::T,
    state::SimulationState,
    model_resolution::Dates.Millisecond,
) where {
    T <: Union{
        Vector{
            Union{
                InitialCondition{InitialTimeDurationOff, Nothing},
                InitialCondition{InitialTimeDurationOff, Float64},
            },
        },
        Vector{
            Union{
                InitialCondition{InitialTimeDurationOff, Nothing},
                InitialCondition{InitialTimeDurationOff, JuMP.VariableRef},
            },
        },
    },
}
    isempty(ics) && return
    component_type = get_component_type(first(ics))
    key = AuxVarKey(TimeDurationOff, component_type)
    var_val = get_system_state_value(state, key)
    state_resolution = get_data_resolution(get_system_state_data(state, key))
    # The state data is stored in the state resolution (i.e. lowest resolution among all models)
    # so this step scales the data to the model resolution.
    scale = model_resolution / state_resolution
    for ic in ics
        isnothing(get_value(ic)) && continue
        set_ic_quantity!(ic, _get_state_value(var_val, key, get_component_name(ic)) / scale)
    end
    return
end

function IOM.update_initial_conditions!(
    ics::T,
    state::SimulationState,
    ::Dates.Millisecond,
) where {
    T <: Union{
        Vector{
            Union{
                InitialCondition{DevicePower, Nothing},
                InitialCondition{DevicePower, Float64},
            },
        },
        Vector{
            Union{
                InitialCondition{DevicePower, Nothing},
                InitialCondition{DevicePower, JuMP.VariableRef},
            },
        },
    },
}
    isempty(ics) && return
    comp_type = get_component_type(first(ics))
    power_key = VariableKey(ActivePowerVariable, comp_type)
    status_key = VariableKey(OnVariable, comp_type)
    power_vals = get_system_state_value(state, power_key)
    for ic in ics
        comp_name = get_component_name(ic)
        comp = IOM.get_component(ic)
        if IOM.is_always_committed(comp)
            status_val = 1.0
        else
            status_val = _get_state_value(
                get_system_state_value(state, status_key),
                status_key,
                comp_name,
            )
        end
        var_val = _get_state_value(power_vals, power_key, comp_name)
        if !isapprox(status_val, 0.0; atol = ABSOLUTE_TOLERANCE)
            (; min, max) = PSY.get_active_power_limits(comp, u"SU")
            if var_val <= max && var_val >= min
                set_ic_quantity!(ic, var_val)
            elseif isapprox(min - var_val, 0.0; atol = ABSOLUTE_TOLERANCE)
                set_ic_quantity!(ic, min)
            elseif isapprox(var_val - max, 0.0; atol = ABSOLUTE_TOLERANCE)
                set_ic_quantity!(ic, max)
            else
                error("Variable value $(var_val) for ActivePowerVariable \\
                      Status value $(status_val) for OnVariable \\
                      $(comp_type)-$(comp_name) is out of bounds [$(min), $(max)].")
            end
        else
            if !isapprox(var_val, 0.0; atol = ABSOLUTE_TOLERANCE)
                error("Status and Power variables don't match for $comp_name. \\
                ActivePowerVariable: $(var_val)\\
                Status value: $(status_val) for OnVariable")
            end
            set_ic_quantity!(ic, 0.0)
        end
    end
    return
end

function IOM.update_initial_conditions!(
    ics::T,
    state::SimulationState,
    ::Dates.Millisecond,
) where {
    T <: Union{
        Vector{
            Union{
                InitialCondition{DeviceStatus, Nothing},
                InitialCondition{DeviceStatus, Float64},
            },
        },
        Vector{
            Union{
                InitialCondition{DeviceStatus, Nothing},
                InitialCondition{DeviceStatus, JuMP.VariableRef},
            },
        },
    },
}
    isempty(ics) && return
    key = VariableKey(OnVariable, get_component_type(first(ics)))
    var_val = get_system_state_value(state, key)
    for ic in ics
        isnothing(get_value(ic)) && continue
        set_ic_quantity!(ic, _get_state_value(var_val, key, get_component_name(ic)))
    end
    return
end

function IOM.update_initial_conditions!(
    ics::T,
    state::SimulationState,
    ::Dates.Millisecond,
) where {
    T <: Union{
        Vector{
            Union{
                InitialCondition{DeviceAboveMinPower, Nothing},
                InitialCondition{DeviceAboveMinPower, Float64},
            },
        },
        Vector{
            Union{
                InitialCondition{DeviceAboveMinPower, Nothing},
                InitialCondition{DeviceAboveMinPower, JuMP.VariableRef},
            },
        },
    },
}
    isempty(ics) && return
    key = VariableKey(PowerAboveMinimumVariable, get_component_type(first(ics)))
    var_val = get_system_state_value(state, key)
    for ic in ics
        set_ic_quantity!(ic, _get_state_value(var_val, key, get_component_name(ic)))
    end
    return
end

function IOM.update_initial_conditions!(
    ics::T,
    state::SimulationState,
    ::Dates.Millisecond,
) where {
    T <: Union{
        Vector{
            Union{
                InitialCondition{InitialEnergyLevel, Nothing},
                InitialCondition{InitialEnergyLevel, Float64},
            },
        },
        Vector{
            Union{
                InitialCondition{InitialEnergyLevel, Nothing},
                InitialCondition{InitialEnergyLevel, JuMP.VariableRef},
            },
        },
    },
}
    isempty(ics) && return
    key = VariableKey(EnergyVariable, get_component_type(first(ics)))
    var_val = get_system_state_value(state, key)
    for ic in ics
        set_ic_quantity!(ic, _get_state_value(var_val, key, get_component_name(ic)))
    end
    return
end

function IOM.update_initial_conditions!(
    ics::T,
    state::SimulationState,
    ::Dates.Millisecond,
) where {
    T <: Union{
        Vector{
            Union{
                InitialCondition{InitialReservoirVolume, Nothing},
                InitialCondition{InitialReservoirVolume, Float64},
            },
        },
        Vector{
            Union{
                InitialCondition{InitialReservoirVolume, Nothing},
                InitialCondition{InitialReservoirVolume, JuMP.VariableRef},
            },
        },
    },
}
    isempty(ics) && return
    key = VariableKey(HydroReservoirVolumeVariable, get_component_type(first(ics)))
    var_val = get_system_state_value(state, key)
    for ic in ics
        set_ic_quantity!(ic, _get_state_value(var_val, key, get_component_name(ic)))
    end
    return
end
