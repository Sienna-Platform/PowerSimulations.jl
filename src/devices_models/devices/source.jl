#! format: off

requires_initialization(::ImportExportSourceModel) = false


get_variable_multiplier(::ActivePowerOutVariable, ::Type{<:PSY.Source}, ::AbstractSourceFormulation) = 1.0
get_variable_multiplier(::ActivePowerInVariable, ::Type{<:PSY.Source}, ::AbstractSourceFormulation) = -1.0
get_variable_multiplier(::ReactivePowerVariable, ::Type{<:PSY.Source}, ::AbstractSourceFormulation) = 1.0
############## ActivePowerVariables, Source ####################
get_variable_binary(::ActivePowerInVariable, ::Type{<:PSY.Source}, ::AbstractSourceFormulation) = false
get_variable_binary(::ActivePowerOutVariable, ::Type{<:PSY.Source}, ::AbstractSourceFormulation) = false
get_variable_lower_bound(::ActivePowerInVariable, d::PSY.Source, ::AbstractSourceFormulation) = 0.0
get_variable_lower_bound(::ActivePowerOutVariable, d::PSY.Source, ::AbstractSourceFormulation) = 0.0
get_variable_upper_bound(::ActivePowerInVariable, d::PSY.Source, ::AbstractSourceFormulation) = -PSY.get_active_power_limits(d).min
get_variable_upper_bound(::ActivePowerOutVariable, d::PSY.Source, ::AbstractSourceFormulation) = PSY.get_active_power_limits(d).max

############## ReactivePowerVariable, Source ####################
get_variable_binary(::ReactivePowerVariable, ::Type{<:PSY.Source}, ::AbstractSourceFormulation) = false
get_variable_lower_bound(::ReactivePowerVariable, d::PSY.Source, ::AbstractSourceFormulation) = PSY.get_reactive_power_limits(d).min
get_variable_upper_bound(::ReactivePowerVariable, d::PSY.Source, ::AbstractSourceFormulation) = PSY.get_reactive_power_limits(d).max

get_multiplier_value(::ActivePowerTimeSeriesParameter, d::PSY.Source, ::AbstractSourceFormulation) = PSY.get_active_power_limits(d).max
get_multiplier_value(::ActivePowerOutTimeSeriesParameter, d::PSY.Source, ::AbstractSourceFormulation) = PSY.get_active_power_limits(d).max
get_multiplier_value(::ActivePowerInTimeSeriesParameter, d::PSY.Source, ::AbstractSourceFormulation) = PSY.get_active_power_limits(d).max
get_multiplier_value(::ActivePowerTimeSeriesParameter, d::PSY.Source, ::FixedOutput) = PSY.get_active_power_limits(d).max
get_multiplier_value(::ActivePowerOutTimeSeriesParameter, d::PSY.Source, ::FixedOutput) = PSY.get_active_power_limits(d).max
get_multiplier_value(::ActivePowerInTimeSeriesParameter, d::PSY.Source, ::FixedOutput) = PSY.get_active_power_limits(d).min
# This additional method definition is used to avoid ambiguity with the method defined in default_interface_methods.jl
get_multiplier_value(::AbstractPiecewiseLinearBreakpointParameter, d::PSY.Source, ::AbstractSourceFormulation) = 1.0

############## ReservationVariable, Source ####################
get_variable_binary(::ReservationVariable, ::Type{<:PSY.Source}, ::ImportExportSourceModel) = true


#! format: on
function get_default_time_series_names(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.Source, V <: Union{FixedOutput, AbstractSourceFormulation}}
    return Dict{Type{<:TimeSeriesParameter}, String}(
        ActivePowerOutTimeSeriesParameter => "max_active_power_out",
        ActivePowerInTimeSeriesParameter => "max_active_power_in",
    )
end

function get_default_attributes(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.Source, V <: Union{FixedOutput, AbstractSourceFormulation}}
    return Dict{String, Any}(
        "reservation" => true,
    )
end

function get_min_max_limits(
    device,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{<:AbstractSourceFormulation},
)
    return PSY.get_active_power_limits(device)
end

function get_min_max_limits(
    device,
    ::Type{InputActivePowerVariableLimitsConstraint},
    ::Type{<:AbstractSourceFormulation},
)
    return PSY.get_active_power_limits(device)  # TODO do we need a new field in PSY for this -- input active power limits?
end

function get_min_max_limits(
    device,
    ::Type{ReactivePowerVariableLimitsConstraint},
    ::Type{<:AbstractSourceFormulation},
)
    return PSY.get_reactive_power_limits(device)
end

##### Constraints ######

function add_constraints!(
    container::OptimizationContainer,
    T::Type{<:PowerVariableLimitsConstraint},
    U::Type{<:Union{VariableType, ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.Source,
    W <: AbstractSourceFormulation,
    X <: PM.AbstractPowerModel,
}
    if get_attribute(model, "reservation")
        add_reserve_range_constraints!(container, T, U, devices, model, X)
    else
        add_range_constraints!(container, T, U, devices, model, X)
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{ImportExportBudgetConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    network_model::NetworkModel{X},
) where {
    U <: PSY.Source,
    V <: AbstractSourceFormulation,
    X <: PM.AbstractPowerModel,
}
    time_steps = get_time_steps(container)
    resolution = get_resolution(container)
    resolution_in_hours = Dates.Hour(resolution).value
    hours_in_horizon = length(time_steps) * resolution_in_hours
    p_out = get_variable(container, ActivePowerOutVariable(), U)
    p_in = get_variable(container, ActivePowerInVariable(), U)
    names = PSY.get_name.(devices)
    constraint_export =
        add_constraints_container!(
            container,
            ImportExportBudgetConstraint(),
            U,
            names;
            meta = "export",
        )
    constraint_import = add_constraints_container!(
        container,
        ImportExportBudgetConstraint(),
        U,
        names;
        meta = "import",
    )

    for d in devices
        name = PSY.get_name(d)
        op_cost = PSY.get_operation_cost(d)
        week_import_limit = PSY.get_energy_import_weekly_limit(op_cost)
        week_export_limit = PSY.get_energy_export_weekly_limit(op_cost)
        constraint_import[name] = JuMP.@constraint(
            get_jump_model(container),
            resolution_in_hours * sum(p_out[name, t] for t in time_steps) <=
            week_import_limit * (hours_in_horizon / HOURS_IN_WEEK)
        )
        constraint_export[name] = JuMP.@constraint(
            get_jump_model(container),
            resolution_in_hours * sum(p_in[name, t] for t in time_steps) <=
            week_export_limit * (hours_in_horizon / HOURS_IN_WEEK)
        )
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{ActivePowerOutVariableTimeSeriesLimitsConstraint},
    U::Type{<:Union{ActivePowerOutVariable, ActivePowerRangeExpressionUB}},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.Source,
    W <: AbstractSourceFormulation,
    X <: PM.AbstractPowerModel,
}
    add_parameterized_upper_bound_range_constraints(
        container,
        ActivePowerOutVariableTimeSeriesLimitsConstraint,
        U,
        ActivePowerOutTimeSeriesParameter,
        devices,
        model,
        X,
    )
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{ActivePowerInVariableTimeSeriesLimitsConstraint},
    U::Type{<:Union{ActivePowerInVariable, ActivePowerRangeExpressionUB}},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.Source,
    W <: AbstractSourceFormulation,
    X <: PM.AbstractPowerModel,
}
    add_parameterized_upper_bound_range_constraints(
        container,
        ActivePowerInVariableTimeSeriesLimitsConstraint,
        U,
        ActivePowerInTimeSeriesParameter,
        devices,
        model,
        X,
    )
    return
end

function PSI.objective_function!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    ::PSI.DeviceModel{T, U},
    ::Type{V},
) where {T <: PSY.Source, U <: AbstractSourceFormulation, V <: PM.AbstractPowerModel}
    PSI.add_variable_cost!(container, PSI.ActivePowerOutVariable(), devices, U())
    PSI.add_variable_cost!(container, PSI.ActivePowerInVariable(), devices, U())
    return
end

get_initial_conditions_device_model(
    ::OperationModel,
    model::DeviceModel{PSY.Source, T},
) where {T <: AbstractSourceFormulation} = DeviceModel(PSY.Source, T)
