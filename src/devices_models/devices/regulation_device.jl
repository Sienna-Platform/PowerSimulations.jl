#! format: off
get_variable_multiplier(_, ::Type{PSY.RegulationDevice{PSY.ThermalStandard}}, ::DeviceLimitedRegulation) = NaN
############################ DeltaActivePowerUpVariable, RegulationDevice ###########################

get_variable_binary(::DeltaActivePowerUpVariable, ::Type{<:PSY.RegulationDevice}, ::AbstractRegulationFormulation) = false
get_variable_lower_bound(::DeltaActivePowerUpVariable, ::PSY.RegulationDevice, ::AbstractRegulationFormulation) = 0.0

############################ DeltaActivePowerDownVariable, RegulationDevice ###########################

get_variable_binary(::DeltaActivePowerDownVariable, ::Type{<:PSY.RegulationDevice}, ::AbstractRegulationFormulation) = false
get_variable_lower_bound(::DeltaActivePowerDownVariable, ::PSY.RegulationDevice, ::AbstractRegulationFormulation) = 0.0

############################ AdditionalDeltaActivePowerUpVariable, RegulationDevice ###########################

get_variable_binary(::AdditionalDeltaActivePowerUpVariable, ::Type{<:PSY.RegulationDevice}, ::AbstractRegulationFormulation) = false
get_variable_lower_bound(::AdditionalDeltaActivePowerUpVariable, ::PSY.RegulationDevice, ::AbstractRegulationFormulation) = 0.0

############################ AdditionalDeltaActivePowerDownVariable, RegulationDevice ###########################

get_variable_binary(::AdditionalDeltaActivePowerDownVariable, ::Type{<:PSY.RegulationDevice}, ::AbstractRegulationFormulation) = false
get_variable_lower_bound(::AdditionalDeltaActivePowerDownVariable, ::PSY.RegulationDevice, ::AbstractRegulationFormulation) = 0.0

get_multiplier_value(::ActivePowerTimeSeriesParameter, d::PSY.RegulationDevice, _)  = PSY.get_max_active_power(d)

########################Objective Function##################################################
proportional_cost(::PSY.OperationalCost, ::AdditionalDeltaActivePowerUpVariable, d::PSY.RegulationDevice, ::AbstractRegulationFormulation) = isapprox(PSY.get_participation_factor(d).up, 0.0; atol=1e-2) ? SERVICES_SLACK_COST : 1 / PSY.get_participation_factor(d).up
proportional_cost(::PSY.OperationalCost, ::AdditionalDeltaActivePowerDownVariable, d::PSY.RegulationDevice, ::AbstractRegulationFormulation) = isapprox(PSY.get_participation_factor(d).dn, 0.0; atol=1e-2) ? SERVICES_SLACK_COST : 1 / PSY.get_participation_factor(d).dn

objective_function_multiplier(::VariableType, ::AbstractRegulationFormulation)=OBJECTIVE_FUNCTION_POSITIVE

#! format: on

function get_default_time_series_names(
    ::Type{<:PSY.RegulationDevice{T}},
    ::Type{<:AbstractRegulationFormulation},
) where {T <: PSY.StaticInjection}
    return Dict{Type{<:TimeSeriesParameter}, String}(
        ActivePowerTimeSeriesParameter => "max_active_power",
    )
end

function get_default_attributes(
    ::Type{<:PSY.RegulationDevice{T}},
    ::Type{<:AbstractRegulationFormulation},
) where {T <: PSY.StaticInjection}
    return Dict{String, Any}()
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{S},
    ::Type{DeltaActivePowerUpVariable},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, DeviceLimitedRegulation},
    ::NetworkModel{AreaBalancePowerModel},
) where {
    S <: RegulationLimitsConstraint,
    T <: PSY.RegulationDevice{U},
} where {U <: PSY.StaticInjection}
    var_up = get_variable(container, DeltaActivePowerUpVariable(), T)
    var_dn = get_variable(container, DeltaActivePowerDownVariable(), T)
    base_points_param = get_parameter(container, ActivePowerTimeSeriesParameter(), T)
    multiplier = get_multiplier_array(base_points_param)

    names = [PSY.get_name(g) for g in devices]
    time_steps = get_time_steps(container)

    container_up =
        add_constraints_container!(container, S(), U, names, time_steps; meta = "up")
    container_dn =
        add_constraints_container!(container, S(), U, names, time_steps; meta = "dn")

    for d in devices
        name = PSY.get_name(d)
        limits = PSY.get_active_power_limits(d)
        param = get_parameter_column_refs(base_points_param, name)
        for t in time_steps
            container_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                var_up[name, t] <= limits.max - param[t] * multiplier[name, t]
            )
            container_dn[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                var_dn[name, t] <= param[t] * multiplier[name, t] - limits.min
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{S},
    ::Type{DeltaActivePowerUpVariable},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, ReserveLimitedRegulation},
    ::NetworkModel{AreaBalancePowerModel},
) where {
    S <: RegulationLimitsConstraint,
    T <: PSY.RegulationDevice{U},
} where {U <: PSY.StaticInjection}
    var_up = get_variable(container, DeltaActivePowerUpVariable(), T)
    var_dn = get_variable(container, DeltaActivePowerDownVariable(), T)

    names = [PSY.get_name(g) for g in devices]
    time_steps = get_time_steps(container)

    container_up =
        add_constraints_container!(container, S(), U, names, time_steps; meta = "up")
    container_dn =
        add_constraints_container!(container, S(), U, names, time_steps; meta = "dn")

    for d in devices
        name = PSY.get_name(d)
        limit_up = PSY.get_reserve_limit_up(d)
        limit_dn = PSY.get_reserve_limit_dn(d)
        for t in time_steps
            container_up[name, t] =
                JuMP.@constraint(container.JuMPmodel, var_up[name, t] <= limit_up)
            container_dn[name, t] =
                JuMP.@constraint(container.JuMPmodel, var_dn[name, t] <= limit_dn)
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{S},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, DeviceLimitedRegulation},
    ::NetworkModel{AreaBalancePowerModel},
) where {
    S <: RampLimitConstraint,
    T <: PSY.RegulationDevice{U},
} where {U <: PSY.StaticInjection}
    R_up = get_variable(container, DeltaActivePowerUpVariable(), T)
    R_dn = get_variable(container, DeltaActivePowerDownVariable(), T)

    resolution = Dates.value(Dates.Minute(get_resolution(container)))
    names = [PSY.get_name(g) for g in devices]
    time_steps = get_time_steps(container)

    container_up =
        add_constraints_container!(container, S(), U, names, time_steps; meta = "up")
    container_dn =
        add_constraints_container!(container, S(), U, names, time_steps; meta = "dn")

    for d in devices
        ramp_limits = PSY.get_ramp_limits(d)
        isnothing(ramp_limits) && continue
        scaling_factor = resolution * SECONDS_IN_MINUTE
        name = PSY.get_name(d)
        for t in time_steps
            container_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                R_up[name, t] <= ramp_limits.up * scaling_factor
            )
            container_dn[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                R_dn[name, t] <= ramp_limits.down * scaling_factor
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{S},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:AbstractRegulationFormulation},
    ::NetworkModel{AreaBalancePowerModel},
) where {
    S <: ParticipationAssignmentConstraint,
    T <: PSY.RegulationDevice{U},
} where {U <: PSY.StaticInjection}
    time_steps = get_time_steps(container)
    R_up = get_variable(container, DeltaActivePowerUpVariable(), T)
    R_dn = get_variable(container, DeltaActivePowerDownVariable(), T)
    R_up_emergency = get_variable(container, AdditionalDeltaActivePowerUpVariable(), T)
    R_dn_emergency = get_variable(container, AdditionalDeltaActivePowerUpVariable(), T)
    area_reserve_up = get_variable(container, DeltaActivePowerUpVariable(), PSY.AGC)
    area_reserve_dn = get_variable(container, DeltaActivePowerDownVariable(), PSY.AGC)

    component_names = PSY.get_name.(devices)
    participation_assignment_up = add_constraints_container!(
        container,
        S(),
        T,
        component_names,
        time_steps;
        meta = "up",
    )
    participation_assignment_dn = add_constraints_container!(
        container,
        S(),
        T,
        component_names,
        time_steps;
        meta = "dn",
    )

    expr_up = get_expression(container, EmergencyUp(), PSY.Area)
    expr_dn = get_expression(container, EmergencyDown(), PSY.Area)
    for d in devices
        name = PSY.get_name(d)
        services = PSY.get_services(d)
        if length(services) > 1
            device_agc = (a for a in PSY.get_services(d) if isa(a, PSY.AGC))
            agc_name = PSY.get_name.(device_agc)[1]
            area_name = PSY.get_name.(PSY.get_area.(device_agc))[1]
        else
            device_agc = first(services)
            agc_name = PSY.get_name(device_agc)
            area_name = PSY.get_name(PSY.get_area(device_agc))
        end
        p_factor = PSY.get_participation_factor(d)
        for t in time_steps
            participation_assignment_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                R_up[name, t] ==
                (p_factor.up * area_reserve_up[agc_name, t]) + R_up_emergency[name, t]
            )
            participation_assignment_dn[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                R_dn[name, t] ==
                (p_factor.dn * area_reserve_dn[agc_name, t]) + R_dn_emergency[name, t]
            )
            JuMP.add_to_expression!(expr_up[area_name, t], -1.0, R_up_emergency[name, t])
            JuMP.add_to_expression!(expr_dn[area_name, t], -1.0, R_dn_emergency[name, t])
        end
    end

    return
end

function objective_function!(
    container::OptimizationContainer,
    devs::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, U},
) where {
    T <: PSY.RegulationDevice{V},
    U <: AbstractRegulationFormulation,
} where {V <: PSY.StaticInjection}
    add_proportional_cost!(container, AdditionalDeltaActivePowerUpVariable(), devs, U())
    add_proportional_cost!(container, AdditionalDeltaActivePowerDownVariable(), devs, U())
    return
end
