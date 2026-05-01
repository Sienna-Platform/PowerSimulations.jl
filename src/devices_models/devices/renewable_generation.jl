#! format: off
get_variable_multiplier(_, ::Type{<:PSY.RenewableGen}, ::AbstractRenewableFormulation) = 1.0
get_expression_type_for_reserve(::ActivePowerReserveVariable, ::Type{<:PSY.RenewableGen}, ::Type{<:PSY.Reserve{PSY.ReserveUp}}) = ActivePowerRangeExpressionUB
get_expression_type_for_reserve(::ActivePowerReserveVariable, ::Type{<:PSY.RenewableGen}, ::Type{<:PSY.Reserve{PSY.ReserveDown}}) = ActivePowerRangeExpressionLB
########################### Parameter related set functions ################################
get_parameter_multiplier(::VariableValueParameter, d::PSY.RenewableGen, ::AbstractRenewableFormulation) = 1.0

########################### ActivePowerVariable, RenewableGen #################################

get_variable_binary(::ActivePowerVariable, ::Type{<:PSY.RenewableGen}, ::AbstractRenewableFormulation) = false
get_min_max_limits(d::PSY.RenewableGen, ::Type{ActivePowerVariableLimitsConstraint}, ::Type{<:AbstractRenewableFormulation}) = (min = 0.0, max = PSY.get_max_active_power(d))
get_variable_lower_bound(::ActivePowerVariable, d::PSY.RenewableGen, ::AbstractRenewableFormulation) = 0.0
get_variable_upper_bound(::ActivePowerVariable, d::PSY.RenewableGen, ::AbstractRenewableFormulation) = PSY.get_max_active_power(d)

########################### ReactivePowerVariable, RenewableGen #################################

get_variable_binary(::ReactivePowerVariable, ::Type{<:PSY.RenewableGen}, ::AbstractRenewableFormulation) = false

get_multiplier_value(::TimeSeriesParameter, d::PSY.RenewableGen, ::FixedOutput) = PSY.get_max_active_power(d)
get_multiplier_value(::TimeSeriesParameter, d::PSY.RenewableGen, ::AbstractRenewableFormulation) = PSY.get_max_active_power(d)

# To avoid ambiguity with default_interface_methods.jl:
get_multiplier_value(::AbstractPiecewiseLinearBreakpointParameter, ::PSY.RenewableGen, ::FixedOutput) = 1.0
get_multiplier_value(::AbstractPiecewiseLinearBreakpointParameter, ::PSY.RenewableGen, ::AbstractRenewableFormulation) = 1.0

########################Objective Function##################################################
objective_function_multiplier(::ActivePowerVariable, ::AbstractRenewableDispatchFormulation)=OBJECTIVE_FUNCTION_NEGATIVE

variable_cost(::Nothing, ::ActivePowerVariable, ::PSY.RenewableDispatch, ::AbstractRenewableDispatchFormulation)=0.0
variable_cost(cost::PSY.OperationalCost, ::ActivePowerVariable, ::PSY.RenewableDispatch, ::AbstractRenewableDispatchFormulation)=PSY.get_variable(cost)

#! format: on

get_initial_conditions_device_model(
    ::OperationModel,
    ::DeviceModel{T, <:AbstractRenewableFormulation},
) where {T <: PSY.RenewableGen} = DeviceModel(T, RenewableFullDispatch)

get_initial_conditions_device_model(
    ::OperationModel,
    ::DeviceModel{T, FixedOutput},
) where {T <: PSY.RenewableGen} = DeviceModel(T, FixedOutput)

function get_min_max_limits(
    device,
    ::Type{ReactivePowerVariableLimitsConstraint},
    ::Type{<:AbstractRenewableFormulation},
)
    return PSY.get_reactive_power_limits(device)
end

function get_default_time_series_names(
    ::Type{<:PSY.RenewableGen},
    ::Type{<:Union{FixedOutput, AbstractRenewableFormulation}},
)
    return Dict{Type{<:TimeSeriesParameter}, String}(
        ActivePowerTimeSeriesParameter => "max_active_power",
        ReactivePowerTimeSeriesParameter => "max_active_power",
    )
end

function get_default_attributes(
    ::Type{<:PSY.RenewableGen},
    ::Type{<:Union{FixedOutput, AbstractRenewableFormulation}},
)
    return Dict{String, Any}()
end

####################################### Reactive Power constraint_infos #########################

function add_constraints!(
    container::OptimizationContainer,
    T::Type{<:ReactivePowerVariableLimitsConstraint},
    U::Type{<:ReactivePowerVariable},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {V <: PSY.RenewableGen, W <: AbstractDeviceFormulation, X <: PM.AbstractPowerModel}
    add_range_constraints!(container, T, U, devices, model, X)
    return
end

"""
Reactive Power Constraints on Renewable Gen Constant power_factor
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{<:ReactivePowerVariableLimitsConstraint},
    ::Type{<:ReactivePowerVariable},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.RenewableGen,
    W <: RenewableConstantPowerFactor,
    X <: PM.AbstractPowerModel,
}
    names = PSY.get_name.(devices)
    time_steps = get_time_steps(container)
    p_var = get_variable(container, ActivePowerVariable(), V)
    q_var = get_variable(container, ReactivePowerVariable(), V)
    jump_model = get_jump_model(container)
    constraint =
        add_constraints_container!(container, EqualityConstraint(), V, names, time_steps)
    for t in time_steps, d in devices
        name = PSY.get_name(d)
        pf = sin(acos(PSY.get_power_factor(d)))
        constraint[name, t] =
            JuMP.@constraint(jump_model, q_var[name, t] == p_var[name, t] * pf)
    end
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{ActivePowerVariableLimitsConstraint},
    U::Type{<:Union{VariableType, ActivePowerRangeExpressionUB}},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.RenewableGen,
    W <: AbstractRenewableDispatchFormulation,
    X <: PM.AbstractPowerModel,
}
    add_parameterized_upper_bound_range_constraints(
        container,
        ActivePowerVariableTimeSeriesLimitsConstraint,
        U,
        ActivePowerTimeSeriesParameter,
        devices,
        model,
        X,
    )
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{ActivePowerVariableLimitsConstraint},
    U::Type{ActivePowerRangeExpressionLB},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.RenewableGen,
    W <: AbstractRenewableDispatchFormulation,
    X <: PM.AbstractPowerModel,
}
    add_range_constraints!(
        container,
        T,
        U,
        devices,
        model,
        X,
    )
    return
end

##################################### renewable generation cost ############################
function objective_function!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, U},
    ::Type{<:PM.AbstractPowerModel},
) where {T <: PSY.RenewableGen, U <: AbstractRenewableDispatchFormulation}
    add_variable_cost!(container, ActivePowerVariable(), devices, U())
    add_curtailment_cost!(container, ActivePowerVariable(), devices, U())
    return
end
