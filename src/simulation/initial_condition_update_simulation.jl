function update_initial_conditions!(
    model::OperationModel,
    state::SimulationState,
    ::InterProblemChronology,
)
    for key in keys(get_initial_conditions(model))
        update_initial_conditions!(model, key, state)
    end
    return
end

function update_initial_conditions!(
    ::OperationModel,
    ::SimulationState,
    ::IntraProblemChronology,
)
    #for key in keys(get_initial_conditions(model))
    #    update_initial_conditions!(model, key, state)
    #end
    error("Not Implemented yet")
    return
end

function update_initial_conditions!(
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
    for ic in ics
        var_val = get_system_state_value(state, TimeDurationOn(), get_component_type(ic))
        state_resolution = get_data_resolution(
            get_system_state_data(state, TimeDurationOn(), get_component_type(ic)),
        )
        # The state data is stored in the state resolution (i.e. lowest resolution among all models)
        # so this step scales the data to the model resolution.
        val = var_val[get_component_name(ic)] / (model_resolution / state_resolution)
        set_ic_quantity!(ic, val)
    end
    return
end

function update_initial_conditions!(
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
    for ic in ics
        isnothing(get_value(ic)) && continue
        var_val = get_system_state_value(state, TimeDurationOff(), get_component_type(ic))
        state_resolution = get_data_resolution(
            get_system_state_data(state, TimeDurationOff(), get_component_type(ic)),
        )
        # The state data is stored in the state resolution (i.e. lowest resolution among all models)
        # so this step scales the data to the model resolution.
        val = var_val[get_component_name(ic)] / (model_resolution / state_resolution)
        set_ic_quantity!(ic, val)
    end
    return
end

function update_initial_conditions!(
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
    for ic in ics
        comp_name = get_component_name(ic)
        comp_type = get_component_type(ic)
        comp = get_component(ic)
        if hasmethod(PSY.get_must_run, Tuple{comp_type}) && PSY.get_must_run(comp)
            status_val = 1.0
        else
            status_val = get_system_state_value(state, OnVariable(), comp_type)[comp_name]
        end
        var_val = get_system_state_value(state, ActivePowerVariable(), comp_type)[comp_name]
        if !isapprox(status_val, 0.0; atol = ABSOLUTE_TOLERANCE)
            min = PSY.get_active_power_limits(comp).min
            max = PSY.get_active_power_limits(comp).max
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

function update_initial_conditions!(
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
    for ic in ics
        isnothing(get_value(ic)) && continue
        var_val = get_system_state_value(state, OnVariable(), get_component_type(ic))
        set_ic_quantity!(ic, var_val[get_component_name(ic)])
    end
    return
end

function update_initial_conditions!(
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
    for ic in ics
        var_val = get_system_state_value(
            state,
            PowerAboveMinimumVariable(),
            get_component_type(ic),
        )
        set_ic_quantity!(ic, var_val[get_component_name(ic)])
    end
    return
end

function update_initial_conditions!(
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
    for ic in ics
        var_val = get_system_state_value(state, EnergyVariable(), get_component_type(ic))
        set_ic_quantity!(ic, var_val[get_component_name(ic)])
    end
    return
end
