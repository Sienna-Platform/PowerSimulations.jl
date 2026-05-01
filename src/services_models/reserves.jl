#! format: off
############################### Reserve Variables #########################################

get_variable_multiplier(_, ::Type{<:PSY.Reserve}, ::AbstractReservesFormulation) = NaN
############################### PostContingencyActivePowerReserveDeploymentVariable, Reserve #########################################
get_variable_binary(::PostContingencyActivePowerReserveDeploymentVariable, ::Type{<:PSY.Reserve}, ::AbstractSecurityConstrainedReservesFormulation) = false
function get_variable_upper_bound(::PostContingencyActivePowerReserveDeploymentVariable, r::PSY.Reserve, d::PSY.Device, ::AbstractSecurityConstrainedReservesFormulation)
    return  PSY.get_max_active_power(d)
end
get_variable_lower_bound(::PostContingencyActivePowerReserveDeploymentVariable, ::PSY.Reserve, ::PSY.Device, _) = 0.0
get_variable_warm_start_value(::PostContingencyActivePowerReserveDeploymentVariable, d::PSY.Reserve, ::AbstractSecurityConstrainedReservesFormulation) = 0.0
get_variable_multiplier(::AbstractContingencyVariableType, ::Type{<:PSY.Reserve{PSY.ReserveDown}}, ::AbstractSecurityConstrainedReservesFormulation) = -1.0
get_variable_multiplier(::AbstractContingencyVariableType, ::Type{<:PSY.Reserve{PSY.ReserveUp}}, ::AbstractSecurityConstrainedReservesFormulation) = 1.0
get_variable_multiplier(::VariableType, ::Type{<:PSY.Generator}, ::AbstractSecurityConstrainedReservesFormulation) = -1.0

############################### ActivePowerReserveVariable, Reserve #########################################
get_variable_binary(::ActivePowerReserveVariable, ::Type{<:PSY.Reserve}, ::AbstractReservesFormulation) = false
function get_variable_upper_bound(::ActivePowerReserveVariable, r::PSY.Reserve, d::PSY.Device, ::AbstractReservesFormulation)
    return PSY.get_max_output_fraction(r) * PSY.get_max_active_power(d)
end
get_variable_upper_bound(::ActivePowerReserveVariable, r::PSY.ReserveDemandCurve, d::PSY.Device, ::AbstractReservesFormulation) = PSY.get_max_active_power(d)
get_variable_lower_bound(::ActivePowerReserveVariable, ::PSY.Reserve, ::PSY.Device, _) = 0.0

############################### ActivePowerReserveVariable, ReserveNonSpinning #########################################
get_variable_binary(::ActivePowerReserveVariable, ::Type{<:PSY.ReserveNonSpinning}, ::AbstractReservesFormulation) = false
function get_variable_upper_bound(::ActivePowerReserveVariable, r::PSY.ReserveNonSpinning, d::PSY.Device, ::AbstractReservesFormulation)
    return PSY.get_max_output_fraction(r) * PSY.get_max_active_power(d)
end
get_variable_lower_bound(::ActivePowerReserveVariable, ::PSY.ReserveNonSpinning, ::PSY.Device, _) = 0.0

############################### ServiceRequirementVariable, ReserveDemandCurve ################################

get_variable_binary(::ServiceRequirementVariable, ::Type{<:PSY.ReserveDemandCurve}, ::AbstractReservesFormulation) = false
get_variable_upper_bound(::ServiceRequirementVariable, ::PSY.ReserveDemandCurve, d::PSY.Component, ::AbstractReservesFormulation) = PSY.get_max_active_power(d)
get_variable_lower_bound(::ServiceRequirementVariable, ::PSY.ReserveDemandCurve, ::PSY.Component, ::AbstractReservesFormulation) = 0.0

get_multiplier_value(::RequirementTimeSeriesParameter, d::PSY.Reserve, ::AbstractReservesFormulation) = PSY.get_requirement(d)
get_multiplier_value(::RequirementTimeSeriesParameter, d::PSY.ReserveNonSpinning, ::AbstractReservesFormulation) = PSY.get_requirement(d)

get_parameter_multiplier(::VariableValueParameter, d::Type{<:PSY.AbstractReserve}, ::AbstractReservesFormulation) = 1.0
get_initial_parameter_value(::VariableValueParameter, d::Type{<:PSY.AbstractReserve}, ::AbstractReservesFormulation) = 0.0

objective_function_multiplier(::ServiceRequirementVariable, ::StepwiseCostReserve) = -1.0
sos_status(::PSY.ReserveDemandCurve, ::StepwiseCostReserve)=SOSStatusVariable.NO_VARIABLE
uses_compact_power(::PSY.ReserveDemandCurve, ::StepwiseCostReserve)=false
#! format: on

function get_initial_conditions_service_model(
    ::OperationModel,
    ::ServiceModel{T, D},
) where {T <: PSY.Reserve, D <: AbstractReservesFormulation}
    return ServiceModel(T, D)
end

function get_initial_conditions_service_model(
    ::OperationModel,
    ::ServiceModel{T, D},
) where {T <: PSY.VariableReserveNonSpinning, D <: AbstractReservesFormulation}
    return ServiceModel(T, D)
end

function get_default_time_series_names(
    ::Type{<:PSY.Reserve},
    ::Type{T},
) where {T <: Union{RangeReserve, RampReserve}}
    return Dict{Type{<:TimeSeriesParameter}, String}(
        RequirementTimeSeriesParameter => "requirement",
    )
end

function get_default_time_series_names(
    ::Type{<:PSY.ReserveNonSpinning},
    ::Type{NonSpinningReserve},
)
    return Dict{Type{<:TimeSeriesParameter}, String}(
        RequirementTimeSeriesParameter => "requirement",
    )
end

function get_default_time_series_names(
    ::Type{T},
    ::Type{<:AbstractReservesFormulation},
) where {T <: PSY.Reserve}
    return Dict{Type{<:TimeSeriesParameter}, String}()
end

function get_default_attributes(
    ::Type{<:PSY.Reserve},
    ::Type{<:AbstractReservesFormulation},
)
    return Dict{String, Any}()
end

function get_default_attributes(
    ::Type{<:PSY.ReserveNonSpinning},
    ::Type{<:AbstractReservesFormulation},
)
    return Dict{String, Any}()
end

"""
Add variables for ServiceRequirementVariable for StepWiseCostReserve
"""
function add_variable!(
    container::OptimizationContainer,
    variable_type::T,
    service::D,
    formulation,
) where {
    T <: ServiceRequirementVariable,
    D <: PSY.ReserveDemandCurve,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    variable = add_variable_container!(
        container,
        variable_type,
        D,
        [service_name],
        time_steps;
        meta = service_name,
    )

    for t in time_steps
        variable[service_name, t] = JuMP.@variable(
            get_jump_model(container),
            base_name = "$(T)_$(D)_$(service_name)_{$(service_name), $(t)}",
            lower_bound = 0.0,
        )
    end

    return
end

function _sum_reserve_variables(
    vars::AbstractArray{<:JuMP.AbstractVariableRef},
    extra::Int,
)
    acc = get_hinted_aff_expr(length(vars) + extra)
    for v in vars
        JuMP.add_to_expression!(acc, v)
    end
    return acc
end

################################## Reserve Requirement Constraint ##########################
function add_constraints!(
    container::OptimizationContainer,
    T::Type{RequirementConstraint},
    service::SR,
    ::U,
    model::ServiceModel{SR, V},
) where {
    SR <: PSY.AbstractReserve,
    V <: AbstractReservesFormulation,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Component}
    parameters = built_for_recurrent_solves(container)

    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    # TODO: Add a method for services that handles this better
    constraint = add_constraints_container!(
        container,
        T(),
        SR,
        [service_name],
        time_steps;
        meta = service_name,
    )
    reserve_variable =
        get_variable(container, ActivePowerReserveVariable(), SR, service_name)
    use_slacks = get_use_slacks(model)

    ts_vector = get_time_series(
        container,
        service,
        "requirement";
        interval = get_interval(get_settings(container)),
    )

    use_slacks && (slack_vars = reserve_slacks!(container, service))
    requirement = PSY.get_requirement(service)
    jump_model = get_jump_model(container)
    extra = use_slacks ? 1 : 0
    if built_for_recurrent_solves(container)
        param_container =
            get_parameter(container, RequirementTimeSeriesParameter(), SR, service_name)
        param = get_parameter_column_refs(param_container, service_name)
        for t in time_steps
            resource_expression =
                _sum_reserve_variables(@view(reserve_variable[:, t]), extra)
            use_slacks &&
                JuMP.add_to_expression!(resource_expression, slack_vars[t])
            constraint[service_name, t] =
                JuMP.@constraint(jump_model, resource_expression >= param[t] * requirement)
        end
    else
        for t in time_steps
            resource_expression =
                _sum_reserve_variables(@view(reserve_variable[:, t]), extra)
            use_slacks &&
                JuMP.add_to_expression!(resource_expression, slack_vars[t])
            constraint[service_name, t] = JuMP.@constraint(
                jump_model,
                resource_expression >= ts_vector[t] * requirement
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{ParticipationFractionConstraint},
    service::SR,
    contributing_devices::U,
    ::ServiceModel{SR, V},
) where {
    SR <: PSY.AbstractReserve,
    V <: AbstractReservesFormulation,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Device}
    max_participation_factor = PSY.get_max_participation_factor(service)

    if max_participation_factor >= 1.0
        return
    end

    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    cons = add_constraints_container!(
        container,
        T(),
        SR,
        [PSY.get_name(d) for d in contributing_devices],
        time_steps;
        meta = service_name,
    )
    var_r = get_variable(container, ActivePowerReserveVariable(), SR, service_name)
    jump_model = get_jump_model(container)
    requirement = PSY.get_requirement(service)
    ts_vector = get_time_series(
        container,
        service,
        "requirement";
        interval = get_interval(get_settings(container)),
    )
    param_container =
        get_parameter(container, RequirementTimeSeriesParameter(), SR, service_name)
    param = get_parameter_column_refs(param_container, service_name)
    for t in time_steps, d in contributing_devices
        name = PSY.get_name(d)
        if built_for_recurrent_solves(container)
            cons[name, t] =
                JuMP.@constraint(
                    jump_model,
                    var_r[name, t] <= (requirement * max_participation_factor) * param[t]
                )
        else
            cons[name, t] = JuMP.@constraint(
                jump_model,
                var_r[name, t] <= (requirement * max_participation_factor) * ts_vector[t]
            )
        end
    end

    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{RequirementConstraint},
    service::SR,
    ::U,
    model::ServiceModel{SR, V},
) where {
    SR <: PSY.ConstantReserve,
    V <: AbstractReservesFormulation,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Component}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    # TODO: The constraint addition is still not clean enough
    constraint = add_constraints_container!(
        container,
        T(),
        SR,
        [service_name],
        time_steps;
        meta = service_name,
    )
    reserve_variable =
        get_variable(container, ActivePowerReserveVariable(), SR, service_name)
    use_slacks = get_use_slacks(model)
    use_slacks && (slack_vars = reserve_slacks!(container, service))

    requirement = PSY.get_requirement(service)
    jump_model = get_jump_model(container)
    extra = use_slacks ? 1 : 0
    for t in time_steps
        resource_expression =
            _sum_reserve_variables(@view(reserve_variable[:, t]), extra)
        use_slacks && JuMP.add_to_expression!(resource_expression, slack_vars[t])
        constraint[service_name, t] =
            JuMP.@constraint(jump_model, resource_expression >= requirement)
    end

    return
end

function objective_function!(
    container::OptimizationContainer,
    service::SR,
    ::ServiceModel{SR, T},
) where {SR <: PSY.AbstractReserve, T <: AbstractReservesFormulation}
    add_proportional_cost!(container, ActivePowerReserveVariable(), service, T())
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{RequirementConstraint},
    service::SR,
    ::U,
    ::ServiceModel{SR, StepwiseCostReserve},
) where {
    SR <: PSY.ReserveDemandCurve,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Component}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    constraint = add_constraints_container!(
        container,
        T(),
        SR,
        [service_name],
        time_steps;
        meta = service_name,
    )
    reserve_variable =
        get_variable(container, ActivePowerReserveVariable(), SR, service_name)
    requirement_variable =
        get_variable(container, ServiceRequirementVariable(), SR, service_name)
    jump_model = get_jump_model(container)
    for t in time_steps
        constraint[service_name, t] = JuMP.@constraint(
            jump_model,
            sum(@view reserve_variable[:, t]) >= requirement_variable[service_name, t]
        )
    end

    return
end

_get_ramp_limits(::PSY.Component) = nothing
_get_ramp_limits(d::PSY.ThermalGen) = PSY.get_ramp_limits(d)
_get_ramp_limits(d::PSY.HydroGen) = PSY.get_ramp_limits(d)

function _get_ramp_constraint_contributing_devices(
    service::PSY.Reserve,
    contributing_devices::Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
) where {D <: PSY.Component}
    time_frame = PSY.get_time_frame(service)
    filtered_device = Vector{D}()
    for d in contributing_devices
        ramp_limits = _get_ramp_limits(d)
        if ramp_limits !== nothing
            p_lims = PSY.get_active_power_limits(d)
            max_rate = abs(p_lims.min - p_lims.max) / time_frame
            if (ramp_limits.up >= max_rate) & (ramp_limits.down >= max_rate)
                @debug "Generator $(name) has a nonbinding ramp limits. Constraints Skipped"
                continue
            else
                push!(filtered_device, d)
            end
        end
    end
    return filtered_device
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{RampConstraint},
    service::SR,
    contributing_devices::Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    ::ServiceModel{SR, V},
) where {
    SR <: PSY.Reserve{PSY.ReserveUp},
    V <: AbstractReservesFormulation,
    D <: PSY.Component,
}
    ramp_devices = _get_ramp_constraint_contributing_devices(service, contributing_devices)
    service_name = PSY.get_name(service)
    if !isempty(ramp_devices)
        jump_model = get_jump_model(container)
        time_steps = get_time_steps(container)
        time_frame = PSY.get_time_frame(service)
        variable = get_variable(container, ActivePowerReserveVariable(), SR, service_name)
        device_name_set = [PSY.get_name(d) for d in ramp_devices]
        con_up = add_constraints_container!(
            container,
            T(),
            SR,
            device_name_set,
            time_steps;
            meta = service_name,
        )
        for d in ramp_devices, t in time_steps
            name = PSY.get_name(d)
            ramp_limits = PSY.get_ramp_limits(d)
            con_up[name, t] = JuMP.@constraint(
                jump_model,
                variable[name, t] <= ramp_limits.up * time_frame
            )
        end
    else
        @warn "Data doesn't contain contributing devices with ramp limits for service $service_name, consider adjusting your formulation"
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{RampConstraint},
    service::SR,
    contributing_devices::Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    ::ServiceModel{SR, V},
) where {
    SR <: PSY.Reserve{PSY.ReserveDown},
    V <: AbstractReservesFormulation,
    D <: PSY.Component,
}
    ramp_devices = _get_ramp_constraint_contributing_devices(service, contributing_devices)
    service_name = PSY.get_name(service)
    if !isempty(ramp_devices)
        jump_model = get_jump_model(container)
        time_steps = get_time_steps(container)
        time_frame = PSY.get_time_frame(service)
        variable = get_variable(container, ActivePowerReserveVariable(), SR, service_name)
        device_name_set = [PSY.get_name(d) for d in ramp_devices]
        con_down = add_constraints_container!(
            container,
            T(),
            SR,
            device_name_set,
            time_steps;
            meta = service_name,
        )
        for d in ramp_devices, t in time_steps
            name = PSY.get_name(d)
            ramp_limits = PSY.get_ramp_limits(d)
            con_down[name, t] = JuMP.@constraint(
                jump_model,
                variable[name, t] <= ramp_limits.down * time_frame
            )
        end
    else
        @warn "Data doesn't contain contributing devices with ramp limits for service $service_name, consider adjusting your formulation"
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{ReservePowerConstraint},
    service::SR,
    contributing_devices::U,
    ::ServiceModel{SR, V},
) where {
    SR <: PSY.VariableReserveNonSpinning,
    V <: AbstractReservesFormulation,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Component}
    time_steps = get_time_steps(container)
    resolution = get_resolution(container)
    if resolution > Dates.Minute(1)
        minutes_per_period = Dates.value(Dates.Minute(resolution))
    else
        @warn("Not all formulations support under 1-minute resolutions. Exercise caution.")
        minutes_per_period = Dates.value(Dates.Second(resolution)) / 60
    end
    service_name = PSY.get_name(service)
    cons = add_constraints_container!(
        container,
        T(),
        SR,
        [PSY.get_name(d) for d in contributing_devices],
        time_steps;
        meta = service_name,
    )
    var_r = get_variable(container, ActivePowerReserveVariable(), SR, service_name)
    reserve_response_time = PSY.get_time_frame(service)
    jump_model = get_jump_model(container)
    for d in contributing_devices
        component_type = typeof(d)
        name = PSY.get_name(d)
        varstatus = get_variable(container, OnVariable(), component_type)
        startup_time = PSY.get_time_limits(d).down
        ramp_limits = _get_ramp_limits(d)
        if reserve_response_time > startup_time
            reserve_limit =
                PSY.get_active_power_limits(d).min +
                (reserve_response_time - startup_time) * minutes_per_period * ramp_limits.up
        else
            reserve_limit = 0.0
        end
        for t in time_steps
            cons[name, t] = JuMP.@constraint(
                jump_model,
                var_r[name, t] <= (1 - varstatus[name, t]) * reserve_limit
            )
        end
    end
    return
end

function objective_function!(
    container::OptimizationContainer,
    service::PSY.ReserveDemandCurve{T},
    ::ServiceModel{PSY.ReserveDemandCurve{T}, SR},
) where {T <: PSY.ReserveDirection, SR <: StepwiseCostReserve}
    add_variable_cost!(container, ServiceRequirementVariable(), service, SR())
    return
end

function add_variable_cost!(
    container::OptimizationContainer,
    ::U,
    service::T,
    ::V,
) where {T <: PSY.ReserveDemandCurve, U <: VariableType, V <: StepwiseCostReserve}
    _add_variable_cost_to_objective!(container, U(), service, V())
    return
end

function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Reserve,
    ::U,
) where {T <: VariableType, U <: StepwiseCostReserve}
    component_name = PSY.get_name(component)
    @debug "PWL Variable Cost" _group = LOG_GROUP_COST_FUNCTIONS component_name
    # If array is full of tuples with zeros return 0.0
    time_steps = get_time_steps(container)
    variable_cost = PSY.get_variable(component)
    if variable_cost isa Nothing
        error("ReserveDemandCurve $(component.name) does not have cost data.")
    elseif typeof(variable_cost) <: PSY.TimeSeriesKey
        error(
            "Timeseries curve for ReserveDemandCurve $(component.name) is not supported yet.",
        )
    end

    pwl_cost_expressions =
        _add_pwl_term!(container, component, variable_cost, T(), U())
    for t in time_steps
        add_to_expression!(
            container,
            ProductionCostExpression,
            pwl_cost_expressions[t],
            component,
            t,
        )
        add_to_objective_invariant_expression!(container, pwl_cost_expressions[t])
    end
    return
end

function add_proportional_cost!(
    container::OptimizationContainer,
    ::U,
    service::T,
    ::V,
) where {
    T <: Union{PSY.Reserve, PSY.ReserveNonSpinning},
    U <: ActivePowerReserveVariable,
    V <: AbstractReservesFormulation,
}
    base_p = get_base_power(container)
    reserve_variable = get_variable(container, U(), T, PSY.get_name(service))
    for index in Iterators.product(axes(reserve_variable)...)
        add_to_objective_invariant_expression!(
            container,
            # possibly decouple
            DEFAULT_RESERVE_COST / base_p * reserve_variable[index...],
        )
    end
    return
end
