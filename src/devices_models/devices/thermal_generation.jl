#! format: off

requires_initialization(::AbstractThermalFormulation) = false
requires_initialization(::AbstractThermalUnitCommitment) = true
requires_initialization(::ThermalStandardDispatch) = true
requires_initialization(::ThermalBasicCompactUnitCommitment) = false
requires_initialization(::ThermalBasicUnitCommitment) = false

get_variable_multiplier(_, ::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = 1.0
get_variable_multiplier(::OnVariable, d::PSY.ThermalGen, ::Union{AbstractCompactUnitCommitment, ThermalCompactDispatch}) = PSY.get_active_power_limits(d).min
get_expression_type_for_reserve(::ActivePowerReserveVariable, ::Type{<:PSY.ThermalGen}, ::Type{<:PSY.Reserve{PSY.ReserveUp}}) = ActivePowerRangeExpressionUB
get_expression_type_for_reserve(::ActivePowerReserveVariable, ::Type{<:PSY.ThermalGen}, ::Type{<:PSY.Reserve{PSY.ReserveDown}}) = ActivePowerRangeExpressionLB

############## ActivePowerVariable, ThermalGen ####################
get_variable_binary(::ActivePowerVariable, ::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = false
get_variable_warm_start_value(::ActivePowerVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_active_power(d)
get_variable_lower_bound(::ActivePowerVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_must_run(d) ? PSY.get_active_power_limits(d).min : 0.0
get_variable_upper_bound(::ActivePowerVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_active_power_limits(d).max
get_variable_lower_bound(::ActivePowerVariable, d::PSY.ThermalGen, ::ThermalDispatchNoMin) = 0.0

############## PowerAboveMinimumVariable, ThermalGen ####################
get_variable_binary(::PowerAboveMinimumVariable, ::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = false
get_variable_warm_start_value(::PowerAboveMinimumVariable, d::PSY.ThermalGen, ::AbstractCompactUnitCommitment) = max(0.0, PSY.get_active_power(d) - PSY.get_active_power_limits(d).min)
get_variable_lower_bound(::PowerAboveMinimumVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 0.0
get_variable_upper_bound(::PowerAboveMinimumVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_active_power_limits(d).max - PSY.get_active_power_limits(d).min

############## ReactivePowerVariable, ThermalGen ####################
get_variable_binary(::ReactivePowerVariable, ::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = false
get_variable_warm_start_value(::ReactivePowerVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_reactive_power(d)
get_variable_lower_bound(::ReactivePowerVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_reactive_power_limits(d).min
get_variable_upper_bound(::ReactivePowerVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_reactive_power_limits(d).max

############## OnVariable, ThermalGen ####################
get_variable_binary(::OnVariable, ::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = true
get_variable_warm_start_value(::OnVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_status(d) ? 1.0 : 0.0
get_variable_lower_bound(::OnVariable, d::PSY.ThermalGen, ::AbstractThermalUnitCommitment) = PSY.get_must_run(d) ? 1.0 : 0.0

############## StopVariable, ThermalGen ####################
get_variable_binary(::StopVariable, ::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = true
get_variable_lower_bound(::StopVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 0.0
get_variable_upper_bound(::StopVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 1.0

############## StartVariable, ThermalGen ####################
get_variable_binary(::StartVariable, d::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = true
get_variable_lower_bound(::StartVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 0.0
get_variable_upper_bound(::StartVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 1.0

############## ColdStartVariable, WarmStartVariable, HotStartVariable ############
get_variable_binary(::Union{ColdStartVariable, WarmStartVariable, HotStartVariable}, ::Type{PSY.ThermalMultiStart}, ::AbstractThermalFormulation) = true

############## SlackVariables, ThermalGen ####################
# LB Slack #
get_variable_binary(::RateofChangeConstraintSlackDown, ::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = false
get_variable_lower_bound(::RateofChangeConstraintSlackDown, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 0.0
# UB Slack #
get_variable_binary(::RateofChangeConstraintSlackUp, ::Type{<:PSY.ThermalGen}, ::AbstractThermalFormulation) = false
get_variable_lower_bound(::RateofChangeConstraintSlackUp, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 0.0

########################### Parameter related set functions ################################
get_multiplier_value(::ActivePowerTimeSeriesParameter, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_max_active_power(d)
get_multiplier_value(::FuelCostParameter, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 1.0
get_parameter_multiplier(::VariableValueParameter, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 1.0
get_initial_parameter_value(::VariableValueParameter, d::PSY.ThermalGen, ::AbstractThermalFormulation) = 1.0
get_expression_multiplier(::OnStatusParameter, ::ActivePowerRangeExpressionUB, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_active_power_limits(d).max
get_expression_multiplier(::OnStatusParameter, ::ActivePowerRangeExpressionLB, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_active_power_limits(d).min
get_expression_multiplier(::OnStatusParameter, ::ActivePowerRangeExpressionUB, d::PSY.ThermalGen, ::AbstractCompactUnitCommitment) = PSY.get_active_power_limits(d).max - PSY.get_active_power_limits(d).min
get_expression_multiplier(::OnStatusParameter, ::ActivePowerRangeExpressionLB, d::PSY.ThermalGen, ::AbstractCompactUnitCommitment) = 0.0
get_expression_multiplier(::OnStatusParameter, ::ActivePowerRangeExpressionUB, d::PSY.ThermalGen, ::ThermalCompactDispatch) = PSY.get_active_power_limits(d).max - PSY.get_active_power_limits(d).min
get_expression_multiplier(::OnStatusParameter, ::ActivePowerRangeExpressionLB, d::PSY.ThermalGen, ::ThermalCompactDispatch) = 0.0
get_expression_multiplier(::OnStatusParameter, ::ActivePowerBalance, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_active_power_limits(d).min

#################### Initial Conditions for models ###############
initial_condition_default(::DeviceStatus, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_status(d) ? 1.0 : 0.0
initial_condition_variable(::DeviceStatus, d::PSY.ThermalGen, ::AbstractThermalFormulation) = OnVariable()
initial_condition_default(::DevicePower, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_active_power(d)
initial_condition_variable(::DevicePower, d::PSY.ThermalGen, ::AbstractThermalFormulation) = ActivePowerVariable()
initial_condition_default(::DeviceAboveMinPower, d::PSY.ThermalGen, ::AbstractThermalFormulation) = max(0.0, PSY.get_active_power(d) - PSY.get_active_power_limits(d).min)
initial_condition_variable(::DeviceAboveMinPower, d::PSY.ThermalGen, ::AbstractCompactUnitCommitment) = PowerAboveMinimumVariable()
initial_condition_variable(::DeviceAboveMinPower, d::PSY.ThermalGen, ::ThermalCompactDispatch) = PowerAboveMinimumVariable()
initial_condition_default(::InitialTimeDurationOn, d::PSY.ThermalGen, ::AbstractThermalFormulation) = PSY.get_status(d) ? PSY.get_time_at_status(d) : 0.0
initial_condition_variable(::InitialTimeDurationOn, d::PSY.ThermalGen, ::AbstractThermalFormulation) = OnVariable()
initial_condition_default(::InitialTimeDurationOff, d::PSY.ThermalGen, ::AbstractThermalFormulation) = !PSY.get_status(d) ? PSY.get_time_at_status(d) : 0.0
initial_condition_variable(::InitialTimeDurationOff, d::PSY.ThermalGen, ::AbstractThermalFormulation) = OnVariable()

########################Objective Function##################################################
# TODO: Decide what is the cost for OnVariable, if fixed or constant term in variable
function proportional_cost(container::OptimizationContainer, cost::PSY.ThermalGenerationCost, S::OnVariable, T::PSY.ThermalGen, U::AbstractThermalFormulation, t::Int)
    return onvar_cost(container, cost, S, T, U, t) + PSY.get_constant_term(PSY.get_vom_cost(PSY.get_variable(cost))) + PSY.get_fixed(cost)
end
is_time_variant_term(::OptimizationContainer, ::PSY.ThermalGenerationCost, ::OnVariable, ::PSY.ThermalGen, ::AbstractThermalFormulation, t::Int) = false

proportional_cost(container::OptimizationContainer, cost::PSY.MarketBidCost, ::OnVariable, comp::PSY.ThermalGen, ::AbstractThermalFormulation, t::Int) =
    _lookup_maybe_time_variant_param(container, comp, t,
    Val(is_time_variant(PSY.get_incremental_initial_input(cost))),
    PSY.get_initial_input ∘ PSY.get_incremental_offer_curves ∘ PSY.get_operation_cost,
    IncrementalCostAtMinParameter())
is_time_variant_term(::OptimizationContainer, cost::PSY.MarketBidCost, ::OnVariable, ::PSY.ThermalGen, ::AbstractThermalFormulation, t::Int) =
    is_time_variant(PSY.get_incremental_initial_input(cost))

proportional_cost(::Union{PSY.MarketBidCost, PSY.ThermalGenerationCost}, ::Union{RateofChangeConstraintSlackUp, RateofChangeConstraintSlackDown}, ::PSY.ThermalGen, ::AbstractThermalFormulation) = CONSTRAINT_VIOLATION_SLACK_COST


has_multistart_variables(::PSY.ThermalGen, ::AbstractThermalFormulation)=false
has_multistart_variables(::PSY.ThermalMultiStart, ::ThermalMultiStartUnitCommitment)=true

objective_function_multiplier(::VariableType, ::AbstractThermalFormulation)=OBJECTIVE_FUNCTION_POSITIVE

sos_status(::PSY.ThermalGen, ::AbstractThermalDispatchFormulation)=SOSStatusVariable.NO_VARIABLE
sos_status(::PSY.ThermalGen, ::AbstractThermalUnitCommitment)=SOSStatusVariable.VARIABLE
sos_status(::PSY.ThermalMultiStart, ::AbstractStandardUnitCommitment)=SOSStatusVariable.VARIABLE
sos_status(::PSY.ThermalMultiStart, ::ThermalMultiStartUnitCommitment)=SOSStatusVariable.VARIABLE

# Startup cost interpretations!
# Validators: check that the types match (formulation is optional) and redirect to the simpler methods
start_up_cost(cost, ::PSY.ThermalGen, ::T, ::Union{AbstractThermalFormulation, Nothing} = nothing) where {T <: StartVariable} =
    start_up_cost(cost, T())
start_up_cost(cost, ::PSY.ThermalMultiStart, ::T, ::ThermalMultiStartUnitCommitment = ThermalMultiStartUnitCommitment()) where {T <: MultiStartVariable} =
    start_up_cost(cost, T())

# Implementations: given a single number, tuple, or StartUpStages and a variable, do the right thing
# Single number to anything
start_up_cost(cost::Float64, ::StartVariable) = cost
# TODO in the case where we have a single number startup cost and we're modeling a multi-start, do we set all the values to that number?
start_up_cost(cost::Float64, ::T) where {T <: MultiStartVariable} =
    start_up_cost((hot = cost, warm = cost, cold = cost), T())

# 3-tuple to anything
start_up_cost(cost::NTuple{3, Float64}, ::T) where {T <: VariableType} =
    start_up_cost(StartUpStages(cost), T())

# `StartUpStages` to anything
start_up_cost(cost::StartUpStages, ::ColdStartVariable) = cost.cold
start_up_cost(cost::StartUpStages, ::WarmStartVariable) = cost.warm
start_up_cost(cost::StartUpStages, ::HotStartVariable) = cost.hot
# TODO in the opposite case, do we want to get the maximum or the hot?
start_up_cost(cost::StartUpStages, ::StartVariable) = maximum(cost)

uses_compact_power(::PSY.ThermalGen, ::AbstractThermalFormulation)=false
uses_compact_power(::PSY.ThermalGen, ::AbstractCompactUnitCommitment )=true
uses_compact_power(::PSY.ThermalGen, ::ThermalCompactDispatch)=true

variable_cost(cost::PSY.OperationalCost, ::ActivePowerVariable, ::PSY.ThermalGen, ::AbstractThermalFormulation)=PSY.get_variable(cost)
variable_cost(cost::PSY.OperationalCost, ::PowerAboveMinimumVariable, ::PSY.ThermalGen, ::AbstractThermalFormulation)=PSY.get_variable(cost)

"""
Theoretical Cost at power output zero. Mathematically is the intercept with the y-axis
"""
function onvar_cost(container::OptimizationContainer, cost::PSY.ThermalGenerationCost, ::OnVariable, d::PSY.ThermalGen, ::AbstractThermalFormulation, t::Int)
    return _onvar_cost(container, PSY.get_variable(cost), d, t)
end

function _onvar_cost(::OptimizationContainer, cost_function::PSY.FuelCurve{PSY.PiecewisePointCurve}, d::PSY.ThermalGen, ::Int)
    # OnVariableCost is included in the Point itself for PiecewisePointCurve
    return 0.0
end

function _onvar_cost(::OptimizationContainer, cost_function::PSY.FuelCurve{PSY.PiecewiseIncrementalCurve}, d::PSY.ThermalGen, ::Int)
    # Input at min is used to transform to InputOutputCurve
    return 0.0
end

function _onvar_cost(::OptimizationContainer, cost_function::PSY.FuelCurve{PSY.PiecewiseAverageCurve}, d::PSY.ThermalGen, ::Int)
    # Converted to InputOutputCurve, OnVariableCost handled in transformation
    return 0.0
end

# this one implementation is thermal-specific, and requires the component.
function _onvar_cost(container::OptimizationContainer, cost_function::Union{PSY.FuelCurve{PSY.LinearCurve}, PSY.FuelCurve{PSY.QuadraticCurve}}, d::T, t::Int) where {T <: PSY.ThermalGen}
    value_curve = PSY.get_value_curve(cost_function)
    cost_component = PSY.get_function_data(value_curve)
    # In Unit/h
    constant_term = PSY.get_constant_term(cost_component)
    fuel_cost = PSY.get_fuel_cost(cost_function)
    if typeof(fuel_cost) <: Float64
        return constant_term * fuel_cost
    else
        parameter_array = get_parameter_array(container, FuelCostParameter(), T)
        parameter_multiplier =
            get_parameter_multiplier_array(container, FuelCostParameter(), T)
        name = PSY.get_name(d)
        return constant_term * parameter_array[name, t] * parameter_multiplier[name, t]
    end
end

#! format: on
function get_initial_conditions_device_model(
    model::OperationModel,
    ::DeviceModel{T, D},
) where {T <: PSY.ThermalGen, D <: AbstractThermalDispatchFormulation}
    if supports_milp(get_optimization_container(model))
        return DeviceModel(T, ThermalBasicUnitCommitment)
    else
        return DeviceModel(T, ThermalBasicDispatch)
    end
end

function get_initial_conditions_device_model(
    ::OperationModel,
    ::DeviceModel{T, D},
) where {T <: PSY.ThermalGen, D <: ThermalDispatchNoMin}
    return DeviceModel(T, ThermalDispatchNoMin)
end

function get_initial_conditions_device_model(
    ::OperationModel,
    ::DeviceModel{T, D},
) where {T <: PSY.ThermalGen, D <: AbstractThermalUnitCommitment}
    return DeviceModel(T, ThermalBasicUnitCommitment)
end

function get_initial_conditions_device_model(
    ::OperationModel,
    ::DeviceModel{T, D},
) where {T <: PSY.ThermalGen, D <: AbstractCompactUnitCommitment}
    return DeviceModel(T, ThermalBasicCompactUnitCommitment)
end

function get_default_time_series_names(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.ThermalGen, V <: Union{FixedOutput, AbstractThermalFormulation}}
    return Dict{Any, String}(
        FuelCostParameter => "fuel_cost",
    )
end

function get_default_attributes(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.ThermalGen, V <: Union{FixedOutput, AbstractThermalFormulation}}
    return Dict{String, Any}()
end

######## THERMAL GENERATION CONSTRAINTS ############

# active power limits of generators when there are no CommitmentVariables
"""
Min and max active power limits of generators for thermal dispatch formulations
"""
function get_min_max_limits(
    device,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{<:AbstractThermalDispatchFormulation},
)
    return PSY.get_active_power_limits(device)
end

# active power limits of generators when there are CommitmentVariables
"""
Min and max active power limits of generators for thermal unit commitment formulations
"""
function get_min_max_limits(
    device,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{<:AbstractThermalUnitCommitment},
)
    return PSY.get_active_power_limits(device)
end

"""
Range constraints for thermal compact dispatch
"""
function add_constraints!(
    container::OptimizationContainer,
    T::Type{<:PowerVariableLimitsConstraint},
    U::Type{<:Union{PowerAboveMinimumVariable, ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {V <: PSY.ThermalGen, W <: ThermalCompactDispatch, X <: PM.AbstractPowerModel}
    if !has_semicontinuous_feedforward(model, PowerAboveMinimumVariable)
        add_range_constraints!(container, T, U, devices, model, X)
    end
    return
end

"""
Min and max active power limits of generators for thermal dispatch compact formulations
"""
function get_min_max_limits(
    device,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{ThermalCompactDispatch},
)
    return (
        min = 0.0,
        max = PSY.get_active_power_limits(device).max -
              PSY.get_active_power_limits(device).min,
    )
end

"""
Min and max active power limits of generators for thermal dispatch no minimum formulations
"""
function get_min_max_limits(
    device,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{ThermalDispatchNoMin},
)
    return (min = 0.0, max = PSY.get_active_power_limits(device).max)
end

"""
Semicontinuous range constraints for thermal dispatch formulations
"""
function add_constraints!(
    container::OptimizationContainer,
    T::Type{<:PowerVariableLimitsConstraint},
    U::Type{<:Union{VariableType, ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.ThermalGen,
    W <: AbstractThermalDispatchFormulation,
    X <: PM.AbstractPowerModel,
}
    if !has_semicontinuous_feedforward(model, U)
        add_range_constraints!(container, T, U, devices, model, X)
    end
    return
end

"""
Min and max active power limits for multi-start unit commitment formulations
"""
function get_min_max_limits(
    device,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{ThermalMultiStartUnitCommitment},
) #  -> Union{Nothing, NamedTuple{(:startup, :shutdown), Tuple{Float64, Float64}}}
    return (
        min = 0.0,
        max = PSY.get_active_power_limits(device).max -
              PSY.get_active_power_limits(device).min,
    )
end

"""
Adds a variable to the optimization model for the OnVariable of Thermal Units
"""
function add_variable!(
    container::OptimizationContainer,
    variable_type::T,
    devices::U,
    formulation::AbstractThermalFormulation,
) where {
    T <: Union{OnVariable, StartVariable, StopVariable},
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.ThermalGen}
    @assert !isempty(devices)
    time_steps = get_time_steps(container)
    settings = get_settings(container)
    binary = get_variable_binary(variable_type, D, formulation)

    variable = add_variable_container!(
        container,
        variable_type,
        D,
        [PSY.get_name(d) for d in devices if !PSY.get_must_run(d)],
        time_steps,
    )

    for d in devices
        if PSY.get_must_run(d)
            continue
        end
        name = PSY.get_name(d)
        for t in time_steps
            variable[name, t] = JuMP.@variable(
                get_jump_model(container),
                base_name = "$(T)_$(D)_{$(name), $(t)}",
                binary = binary
            )
            if get_warm_start(settings)
                init = get_variable_warm_start_value(variable_type, d, formulation)
                !isnothing(init) && JuMP.set_start_value(variable[name, t], init)
            end
        end
    end

    return
end

"""
Semicontinuous range constraints for unit commitment formulations
"""
function add_constraints!(
    container::OptimizationContainer,
    T::Type{<:PowerVariableLimitsConstraint},
    U::Type{<:Union{VariableType, ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.ThermalGen,
    W <: AbstractThermalUnitCommitment,
    X <: PM.AbstractPowerModel,
}
    add_semicontinuous_range_constraints!(container, T, U, devices, model, X)
    return
end

"""
Startup and shutdown active power limits for Compact Unit Commitment
"""
function get_startup_shutdown_limits(
    device::PSY.ThermalMultiStart,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{ThermalMultiStartUnitCommitment},
)
    startup_shutdown = PSY.get_power_trajectory(device)
    if isnothing(startup_shutdown)
        @warn(
            "Generator $(summary(device)) has a Nothing startup_shutdown property. Using active power limits."
        )
        return (
            startup = PSY.get_active_power_limits(device).max,
            shutdown = PSY.get_active_power_limits(device).max,
        )
    end
    return startup_shutdown
end

"""
Min and Max active power limits for Compact Unit Commitment
"""
function get_min_max_limits(
    device,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{<:AbstractCompactUnitCommitment},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    return (
        min = 0,
        max = PSY.get_active_power_limits(device).max -
              PSY.get_active_power_limits(device).min,
    )
end

"""
Startup shutdown limits for Compact Unit Commitment
"""
function get_startup_shutdown_limits(
    device,
    ::Type{ActivePowerVariableLimitsConstraint},
    ::Type{<:AbstractCompactUnitCommitment},
)
    return (
        startup = PSY.get_active_power_limits(device).max,
        shutdown = PSY.get_active_power_limits(device).max,
    )
end

function _get_data_for_range_ic(
    initial_conditions_power::Vector{<:InitialCondition},
    initial_conditions_status::Vector{<:InitialCondition},
)
    lenght_devices_power = length(initial_conditions_power)
    lenght_devices_status = length(initial_conditions_status)
    IS.@assert_op lenght_devices_power == lenght_devices_status
    ini_conds = Matrix{InitialCondition}(undef, lenght_devices_power, 2)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_power)
        g = get_component(ic)
        IS.@assert_op g == get_component(initial_conditions_status[ix])
        idx += 1
        ini_conds[idx, 1] = ic
        ini_conds[idx, 2] = initial_conditions_status[ix]
    end
    return ini_conds
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{ActivePowerVariableTimeSeriesLimitsConstraint},
    U::Type{<:Union{ActivePowerVariable, ActivePowerRangeExpressionUB}},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.ThermalGen,
    W <: AbstractThermalUnitCommitment,
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

"""
This function adds range constraint for the first time period. Constraint (10) from PGLIB formulation
"""
function add_constraints!(
    container::OptimizationContainer,
    T::Type{<:ActivePowerVariableLimitsConstraint},
    U::Type{<:VariableType},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.ThermalMultiStart,
    W <: ThermalMultiStartUnitCommitment,
    X <: PM.AbstractPowerModel,
}
    time_steps = get_time_steps(container)
    constraint_type = T()
    variable_type = U()
    component_type = V
    varp = get_variable(container, variable_type, component_type)
    varstatus = get_variable(container, OnVariable(), component_type)
    varon = get_variable(container, StartVariable(), component_type)
    varoff = get_variable(container, StopVariable(), component_type)

    names = [PSY.get_name(x) for x in devices]
    con_on = add_constraints_container!(
        container,
        constraint_type,
        component_type,
        names,
        time_steps;
        meta = "on",
    )
    con_off = add_constraints_container!(
        container,
        constraint_type,
        component_type,
        names,
        time_steps[1:(end - 1)];
        meta = "off",
    )
    con_lb = add_constraints_container!(
        container,
        constraint_type,
        component_type,
        names,
        time_steps;
        meta = "lb",
    )

    for device in devices
        name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W) # depends on constraint type and formulation type
        startup_shutdown_limits = get_startup_shutdown_limits(device, T, W)

        if JuMP.has_lower_bound(varp[name, t])
            JuMP.set_lower_bound(varp[name, t], 0.0)
        end
        for t in time_steps
            con_on[name, t] = JuMP.@constraint(
                get_jump_model(container),
                varp[name, t] <=
                (limits.max - limits.min) * varstatus[name, t] -
                max(limits.max - startup_shutdown_limits.startup, 0.0) * varon[name, t]
            )

            con_lb[name, t] =
                JuMP.@constraint(get_jump_model(container), varp[name, t] >= 0.0)

            if t != length(time_steps)
                con_off[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    varp[name, t] <=
                    (limits.max - limits.min) * varstatus[name, t] -
                    max(limits.max - startup_shutdown_limits.shutdown, 0.0) *
                    varoff[name, t + 1]
                )
            end
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{<:ActivePowerVariableLimitsConstraint},
    U::Type{ActivePowerRangeExpressionLB},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.ThermalMultiStart,
    W <: ThermalMultiStartUnitCommitment,
    X <: PM.AbstractPowerModel,
}
    time_steps = get_time_steps(container)
    constraint_type = T()
    expression_type = U()
    component_type = V
    expression_products = get_expression(container, expression_type, component_type)
    varp = get_variable(container, PowerAboveMinimumVariable(), component_type)

    names = [PSY.get_name(x) for x in devices]
    con_lb = add_constraints_container!(
        container,
        constraint_type,
        component_type,
        names,
        time_steps;
        meta = "lb",
    )

    for device in devices
        name = PSY.get_name(device)
        for t in time_steps
            if JuMP.has_lower_bound(varp[name, t])
                JuMP.set_lower_bound(varp[name, t], 0.0)
            end
            con_lb[name, t] =
                JuMP.@constraint(
                    get_jump_model(container),
                    expression_products[name, t] >= 0
                )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{<:ActivePowerVariableLimitsConstraint},
    U::Type{ActivePowerRangeExpressionUB},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    ::NetworkModel{X},
) where {
    V <: PSY.ThermalMultiStart,
    W <: ThermalMultiStartUnitCommitment,
    X <: PM.AbstractPowerModel,
}
    time_steps = get_time_steps(container)
    constraint_type = T()
    expression_type = U()
    component_type = V
    expression_products = get_expression(container, expression_type, component_type)
    varstatus = get_variable(container, OnVariable(), component_type)
    varon = get_variable(container, StartVariable(), component_type)
    varoff = get_variable(container, StopVariable(), component_type)
    varp = get_variable(container, PowerAboveMinimumVariable(), component_type)

    names = [PSY.get_name(x) for x in devices]
    con_on = add_constraints_container!(
        container,
        constraint_type,
        component_type,
        names,
        time_steps;
        meta = "ubon",
    )
    con_off = add_constraints_container!(
        container,
        constraint_type,
        component_type,
        names,
        time_steps[1:(end - 1)];
        meta = "uboff",
    )

    for device in devices
        name = PSY.get_name(device)
        limits = get_min_max_limits(device, T, W) # depends on constraint type and formulation type
        startup_shutdown_limits = get_startup_shutdown_limits(device, T, W)
        @assert !isnothing(startup_shutdown_limits) "$(name)"
        for t in time_steps
            if JuMP.has_lower_bound(varp[name, t])
                JuMP.set_lower_bound(varp[name, t], 0.0)
            end
            con_on[name, t] = JuMP.@constraint(
                get_jump_model(container),
                expression_products[name, t] <=
                (limits.max - limits.min) * varstatus[name, t] -
                max(limits.max - startup_shutdown_limits.startup, 0) * varon[name, t]
            )
            if t != length(time_steps)
                con_off[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    expression_products[name, t] <=
                    (limits.max - limits.min) * varstatus[name, t] -
                    max(limits.max - startup_shutdown_limits.shutdown, 0) *
                    varoff[name, t + 1]
                )
            end
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{ActiveRangeICConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, S},
    network_model::NetworkModel{X},
) where {
    T <: PSY.ThermalGen,
    S <: AbstractCompactUnitCommitment,
    X <: PM.AbstractPowerModel,
}
    initial_conditions_power = get_initial_condition(container, DeviceAboveMinPower(), T)
    initial_conditions_status = get_initial_condition(container, DeviceStatus(), T)
    ini_conds = _get_data_for_range_ic(initial_conditions_power, initial_conditions_status)

    if !isempty(ini_conds)
        varstop = get_variable(container, StopVariable(), T)
        device_name_set = PSY.get_name.(devices)
        con = add_constraints_container!(
            container,
            ActiveRangeICConstraint(),
            T,
            device_name_set,
        )

        for (ix, ic) in enumerate(ini_conds[:, 1])
            name = get_component_name(ic)
            device = get_component(ic)
            limits = PSY.get_active_power_limits(device)
            lag_ramp_limits = PSY.get_power_trajectory(device)
            val = max(limits.max - lag_ramp_limits.shutdown, 0)
            con[name] = JuMP.@constraint(
                get_jump_model(container),
                val * varstop[name, 1] <=
                ini_conds[ix, 2].value * (limits.max - limits.min) - get_value(ic)
            )
        end
    else
        @warn "Data doesn't contain generators with ramp limits, consider adjusting your formulation"
    end
    return
end

"""
Reactive power limits of generators for all dispatch formulations
"""
function get_min_max_limits(
    device,
    ::Type{ReactivePowerVariableLimitsConstraint},
    ::Type{<:AbstractThermalDispatchFormulation},
)
    return PSY.get_reactive_power_limits(device)
end

"""
Reactive power limits of generators when there CommitmentVariables
"""
function get_min_max_limits(
    device,
    ::Type{ReactivePowerVariableLimitsConstraint},
    ::Type{<:AbstractThermalUnitCommitment},
)
    return PSY.get_reactive_power_limits(device)
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{CommitmentConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    network_model::NetworkModel{X},
) where {
    U <: PSY.ThermalGen,
    V <: AbstractThermalUnitCommitment,
    X <: PM.AbstractPowerModel,
}
    time_steps = get_time_steps(container)
    varstart = get_variable(container, StartVariable(), U)
    varstop = get_variable(container, StopVariable(), U)
    varon = get_variable(container, OnVariable(), U)
    names = axes(varstart, 1)
    initial_conditions = get_initial_condition(container, DeviceStatus(), U)
    constraint =
        add_constraints_container!(container, CommitmentConstraint(), U, names, time_steps)
    aux_constraint = add_constraints_container!(
        container,
        CommitmentConstraint(),
        U,
        names,
        time_steps;
        meta = "aux",
    )

    for ic in initial_conditions
        name = PSY.get_name(get_component(ic))
        if !PSY.get_must_run(get_component(ic))
            constraint[name, 1] = JuMP.@constraint(
                get_jump_model(container),
                varon[name, 1] == get_value(ic) + varstart[name, 1] - varstop[name, 1]
            )
            aux_constraint[name, 1] = JuMP.@constraint(
                get_jump_model(container),
                varstart[name, 1] + varstop[name, 1] <= 1.0
            )
        end
    end

    for ic in initial_conditions
        if PSY.get_must_run(get_component(ic))
            continue
        else
            name = get_component_name(ic)
            for t in time_steps[2:end]
                constraint[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    varon[name, t] ==
                    varon[name, t - 1] + varstart[name, t] - varstop[name, t]
                )
                aux_constraint[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    varstart[name, t] + varstop[name, t] <= 1.0
                )
            end
        end
    end
    return
end

########################## Make initial Conditions for a Model #############################
function initial_conditions!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::AbstractThermalUnitCommitment,
) where {T <: PSY.ThermalGen}
    add_initial_condition!(container, devices, formulation, DeviceStatus())
    add_initial_condition!(container, devices, formulation, DevicePower())
    add_initial_condition!(container, devices, formulation, InitialTimeDurationOn())
    add_initial_condition!(container, devices, formulation, InitialTimeDurationOff())

    return
end

function initial_conditions!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::AbstractCompactUnitCommitment,
) where {T <: PSY.ThermalGen}
    add_initial_condition!(container, devices, formulation, DeviceStatus())
    add_initial_condition!(container, devices, formulation, DeviceAboveMinPower())
    add_initial_condition!(container, devices, formulation, InitialTimeDurationOn())
    add_initial_condition!(container, devices, formulation, InitialTimeDurationOff())

    return
end

function initial_conditions!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::Union{ThermalBasicUnitCommitment, ThermalBasicCompactUnitCommitment},
) where {T <: PSY.ThermalGen}
    add_initial_condition!(container, devices, formulation, DeviceStatus())
    add_initial_condition!(container, devices, formulation, InitialTimeDurationOn())
    add_initial_condition!(container, devices, formulation, InitialTimeDurationOff())
    return
end

function initial_conditions!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::AbstractThermalDispatchFormulation,
) where {T <: PSY.ThermalGen}
    add_initial_condition!(container, devices, formulation, DevicePower())
    return
end

function initial_conditions!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::ThermalCompactDispatch,
) where {T <: PSY.ThermalGen}
    add_initial_condition!(container, devices, formulation, DeviceAboveMinPower())
    return
end
############################ Auxiliary Variables Calculation ################################
function calculate_aux_variable_value!(
    container::OptimizationContainer,
    ::AuxVarKey{TimeDurationOn, T},
    ::PSY.System,
) where {T <: PSY.ThermalGen}
    on_variable_results = get_variable(container, OnVariable(), T)
    aux_variable_container = get_aux_variable(container, TimeDurationOn(), T)
    ini_cond = get_initial_condition(container, InitialTimeDurationOn(), T)

    time_steps = get_time_steps(container)

    for ix in eachindex(JuMP.axes(aux_variable_container)[1])
        # if its nothing it means the thermal unit was on must run
        # so there is nothing to do but to add the total number of time steps
        # to the count
        if isnothing(get_value(ini_cond[ix]))
            sum_on_var = time_steps[end]
        else
            on_var_name = get_component_name(ini_cond[ix])
            ini_cond_value = get_condition(ini_cond[ix])
            # On Var doesn't exist for a unit that has must_run = true
            on_var = jump_value.(on_variable_results[on_var_name, :])
            aux_variable_container.data[ix, :] .= ini_cond_value
            sum_on_var = sum(on_var)
        end
        if sum_on_var == time_steps[end] # Unit was always on
            aux_variable_container.data[ix, :] += time_steps
        elseif sum_on_var == 0.0 # Unit was always off
            aux_variable_container.data[ix, :] .= 0.0
        else
            previous_condition = ini_cond_value
            for (t, v) in enumerate(on_var)
                if v < 0.99 # Unit turn off
                    time_value = 0.0
                elseif isapprox(v, 1.0; atol = ABSOLUTE_TOLERANCE) # Unit is on
                    time_value = previous_condition + 1.0
                else
                    error("Binary condition returned $v")
                end
                previous_condition = aux_variable_container.data[ix, t] = time_value
            end
        end
    end

    return
end

function calculate_aux_variable_value!(
    container::OptimizationContainer,
    ::AuxVarKey{TimeDurationOff, T},
    ::PSY.System,
) where {T <: PSY.ThermalGen}
    on_variable_results = get_variable(container, OnVariable(), T)
    aux_variable_container = get_aux_variable(container, TimeDurationOff(), T)
    ini_cond = get_initial_condition(container, InitialTimeDurationOff(), T)

    time_steps = get_time_steps(container)
    for ix in eachindex(JuMP.axes(aux_variable_container)[1])
        # if its nothing it means the thermal unit was on must_run = true
        # so there is nothing to do but continue
        if isnothing(get_value(ini_cond[ix]))
            sum_on_var = 0.0
        else
            on_var_name = get_component_name(ini_cond[ix])
            # On Var doesn't exist for a unit that has must run
            on_var = jump_value.(on_variable_results[on_var_name, :])
            ini_cond_value = get_condition(ini_cond[ix])
            aux_variable_container.data[ix, :] .= ini_cond_value
            sum_on_var = sum(on_var)
        end
        if sum_on_var == time_steps[end] # Unit was always on
            aux_variable_container.data[ix, :] .= 0.0
        elseif sum_on_var == 0.0 # Unit was always off
            aux_variable_container.data[ix, :] += time_steps
        else
            previous_condition = ini_cond_value
            for (t, v) in enumerate(on_var)
                if v < 0.99 # Unit turn off
                    time_value = previous_condition + 1.0
                elseif isapprox(v, 1.0; atol = ABSOLUTE_TOLERANCE) # Unit is on
                    time_value = 0.0
                else
                    error("Binary condition returned $v")
                end
                previous_condition = aux_variable_container.data[ix, t] = time_value
            end
        end
    end

    return
end

function calculate_aux_variable_value!(
    container::OptimizationContainer,
    ::AuxVarKey{PowerOutput, T},
    system::PSY.System,
) where {T <: PSY.ThermalGen}
    time_steps = get_time_steps(container)
    if has_container_key(container, OnVariable, T)
        on_variable_results = get_variable(container, OnVariable(), T)
    elseif has_container_key(container, OnStatusParameter, T)
        on_variable_results = get_parameter_array(container, OnStatusParameter(), T)
    else
        error(
            "$T formulation is NOT supported without a Feedforward for CommitmentDecisions,
      please consider changing your simulation setup or adding a SemiContinuousFeedforward.",
        )
    end
    p_variable_results = get_variable(container, PowerAboveMinimumVariable(), T)
    device_name = axes(p_variable_results, 1)
    aux_variable_container = get_aux_variable(container, PowerOutput(), T)
    for d_name in device_name
        d = PSY.get_component(T, system, d_name)
        name = PSY.get_name(d)
        min = PSY.get_active_power_limits(d).min
        for t in time_steps
            aux_variable_container[name, t] =
                jump_value(on_variable_results[name, t]) * min +
                jump_value(p_variable_results[name, t])
        end
    end

    return
end
########################### Ramp/Rate of Change Constraints ################################
"""
This function gets the data for the generators for ramping constraints of thermal generators
"""
_get_initial_condition_type(
    ::Type{RampConstraint},
    ::Type{<:PSY.ThermalGen},
    ::Type{<:AbstractThermalFormulation},
) = DevicePower
_get_initial_condition_type(
    ::Type{RampConstraint},
    ::Type{<:PSY.ThermalGen},
    ::Type{<:AbstractCompactUnitCommitment},
) = DeviceAboveMinPower
_get_initial_condition_type(
    ::Type{RampConstraint},
    ::Type{<:PSY.ThermalGen},
    ::Type{ThermalCompactDispatch},
) = DeviceAboveMinPower

"""
This function adds the ramping limits of generators when there are CommitmentVariables
"""
function add_constraints!(
    container::OptimizationContainer,
    T::Type{RampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    ::NetworkModel{W},
) where {
    U <: PSY.ThermalGen,
    V <: AbstractThermalUnitCommitment,
    W <: PM.AbstractPowerModel,
}
    add_semicontinuous_ramp_constraints!(
        container,
        T,
        ActivePowerVariable,
        devices,
        model,
        W,
    )
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{RampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    ::NetworkModel{W},
) where {
    U <: PSY.ThermalGen,
    V <: AbstractCompactUnitCommitment,
    W <: PM.AbstractPowerModel,
}
    add_semicontinuous_ramp_constraints!(
        container,
        T,
        PowerAboveMinimumVariable,
        devices,
        model,
        W,
    )
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{RampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, ThermalCompactDispatch},
    ::NetworkModel{V},
) where {U <: PSY.ThermalGen, V <: PM.AbstractPowerModel}
    add_linear_ramp_constraints!(container, T, PowerAboveMinimumVariable, devices, model, V)
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{RampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    ::NetworkModel{W},
) where {
    U <: PSY.ThermalGen,
    V <: AbstractThermalDispatchFormulation,
    W <: PM.AbstractPowerModel,
}
    add_linear_ramp_constraints!(container, T, ActivePowerVariable, devices, model, W)
    return
end

function add_constraints!(
    container::OptimizationContainer,
    T::Type{RampConstraint},
    devices::IS.FlattenIteratorWrapper{PSY.ThermalMultiStart},
    model::DeviceModel{PSY.ThermalMultiStart, ThermalMultiStartUnitCommitment},
    ::NetworkModel{U},
) where {U <: PM.AbstractPowerModel}
    add_linear_ramp_constraints!(container, T, PowerAboveMinimumVariable, devices, model, U)
    return
end

########################### start up trajectory constraints ######################################

function _convert_hours_to_timesteps(
    start_times_hr::StartUpStages,
    resolution::Dates.TimePeriod,
)
    _start_times_ts = (
        round((hr * MINUTES_IN_HOUR) / Dates.value(Dates.Minute(resolution)), RoundUp) for
        hr in start_times_hr
    )
    start_times_ts = StartUpStages(_start_times_ts)
    return start_times_ts
end

@doc raw"""
Constructs contraints for different types of starts based on generator down-time

# Equations
for t in time_limits[s+1]:T

``` var_starts[name, s, t] <= sum( var_stop[name, t-i] for i in time_limits[s]:(time_limits[s+1]-1)  ```

# LaTeX

``  δ^{s}(t)  \leq \sum_{i=TS^{s}_{g}}^{TS^{s+1}_{g}} x^{stop}(t-i) ``
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{StartupTimeLimitTemperatureConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, ThermalMultiStartUnitCommitment},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalMultiStart}
    resolution = get_resolution(container)
    time_steps = get_time_steps(container)
    start_vars = [
        get_variable(container, HotStartVariable(), T),
        get_variable(container, WarmStartVariable(), T),
    ]
    varstop = get_variable(container, StopVariable(), T)

    names = PSY.get_name.(devices)

    con = [
        add_constraints_container!(
            container,
            StartupTimeLimitTemperatureConstraint(),
            T,
            names,
            time_steps;
            sparse = true,
            meta = "hot",
        ),
        add_constraints_container!(
            container,
            StartupTimeLimitTemperatureConstraint(),
            T,
            names,
            time_steps;
            sparse = true,
            meta = "warm",
        ),
    ]

    for t in time_steps, d in devices
        name = PSY.get_name(d)
        startup_types = PSY.get_start_types(d)
        time_limits = _convert_hours_to_timesteps(PSY.get_start_time_limits(d), resolution)
        for ix in 1:(startup_types - 1)
            if t >= time_limits[ix + 1]
                con[ix][name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    start_vars[ix][name, t] <= sum(
                        varstop[name, t - i] for i in UnitRange{Int}(
                            Int(time_limits[ix]),
                            Int(time_limits[ix + 1] - 1),
                        )
                    )
                )
            end
        end
    end
    for c in con
        # Workaround to remove invalid key combinations
        filter!(x -> !isnothing(x.second), c.data)
    end
    return
end

@doc raw"""

Constructs contraints that restricts devices to one type of start at a time

# Equations

``` sum(var_starts[name, s, t] for s in starts) = var_start[name, t]  ```

# LaTeX

``  \sum^{S_g}_{s=1} δ^{s}(t)  \eq  x^{start}(t) ``

"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{StartTypeConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, ThermalMultiStartUnitCommitment},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalMultiStart}
    time_steps = get_time_steps(container)
    varstart = get_variable(container, StartVariable(), T)
    start_vars = [
        get_variable(container, HotStartVariable(), T),
        get_variable(container, WarmStartVariable(), T),
        get_variable(container, ColdStartVariable(), T),
    ]

    device_name_set = PSY.get_name.(devices)
    con = add_constraints_container!(
        container,
        StartTypeConstraint(),
        T,
        device_name_set,
        time_steps,
    )

    for t in time_steps, d in devices
        name = PSY.get_name(d)
        startup_types = PSY.get_start_types(d)
        con[name, t] = JuMP.@constraint(
            get_jump_model(container),
            varstart[name, t] == sum(start_vars[ix][name, t] for ix in 1:(startup_types))
        )
    end
    return
end

@doc raw"""
Constructs contraints that restricts devices to one type of start at a time

# Equations
ub:
``` (time_limits[st+1]-1)*δ^{s}(t) + (1 - δ^{s}(t)) * M_VALUE >= sum(1-varbin[name, i]) for i in 1:t) + initial_condition_offtime  ```
lb:
``` (time_limits[st]-1)*δ^{s}(t) =< sum(1-varbin[name, i]) for i in 1:t) + initial_condition_offtime  ```

# LaTeX

`` TS^{s+1}_{g} δ^{s}(t) + (1-δ^{s}(t)) M_VALUE   \geq  \sum^{t}_{i=1} x^{status}(i)  +  DT_{g}^{0}  \forall t in \{1, \ldots,  TS^{s+1}_{g}``

`` TS^{s}_{g} δ^{s}(t) \leq  \sum^{t}_{i=1} x^{status}(i)  +  DT_{g}^{0}  \forall t in \{1, \ldots,  TS^{s+1}_{g}``

"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{StartupInitialConditionConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, ThermalMultiStartUnitCommitment},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalMultiStart}
    resolution = get_resolution(container)
    initial_conditions_offtime =
        get_initial_condition(container, InitialTimeDurationOff(), PSY.ThermalMultiStart)

    time_steps = get_time_steps(container)
    device_name_set = [get_component_name(ic) for ic in initial_conditions_offtime]
    varbin = get_variable(container, OnVariable(), T)
    varstarts = [
        get_variable(container, HotStartVariable(), T),
        get_variable(container, WarmStartVariable(), T),
    ]

    con_ub = add_constraints_container!(
        container,
        StartupInitialConditionConstraint(),
        T,
        device_name_set,
        time_steps,
        1:(MAX_START_STAGES - 1);
        sparse = true,
        meta = "ub",
    )
    con_lb = add_constraints_container!(
        container,
        StartupInitialConditionConstraint(),
        T,
        device_name_set,
        time_steps,
        1:(MAX_START_STAGES - 1);
        sparse = true,
        meta = "lb",
    )

    for t in time_steps, (ix, ic) in enumerate(initial_conditions_offtime)
        name = PSY.get_name(get_component(ic))
        startup_types = PSY.get_start_types(get_component(ic))
        time_limits = _convert_hours_to_timesteps(
            PSY.get_start_time_limits(get_component(ic)),
            resolution,
        )
        ic = initial_conditions_offtime[ix]
        for st in 1:(startup_types - 1)
            var = varstarts[st]
            if t < (time_limits[st + 1] - 1)
                con_ub[name, t, st] = JuMP.@constraint(
                    get_jump_model(container),
                    (time_limits[st + 1] - 1) * var[name, t] +
                    (1 - var[name, t]) * M_VALUE >=
                    sum((1 - varbin[name, i]) for i in 1:t) + get_value(ic)
                )
                con_lb[name, t, st] = JuMP.@constraint(
                    get_jump_model(container),
                    time_limits[st] * var[name, t] <=
                    sum((1 - varbin[name, i]) for i in 1:t) + get_value(ic)
                )
            end
        end
    end
    for c in [con_ub, con_lb]
        # Workaround to remove invalid key combinations
        filter!(x -> !isnothing(x.second), c.data)
    end
    return
end

########################### time duration constraints ######################################
"""
If the fraction of hours that a generator has a duration constraint is less than
the fraction of hours that a single time_step represents then it is not binding.
"""
function _get_data_for_tdc(
    initial_conditions_on::Vector{T},
    initial_conditions_off::Vector{U},
    resolution::Dates.TimePeriod,
) where {T <: InitialCondition, U <: InitialCondition}
    steps_per_hour = 60 / Dates.value(Dates.Minute(resolution))
    fraction_of_hour = 1 / steps_per_hour
    lenght_devices_on = length(initial_conditions_on)
    lenght_devices_off = length(initial_conditions_off)
    IS.@assert_op lenght_devices_off == lenght_devices_on
    time_params = Vector{UpDown}(undef, lenght_devices_on)
    ini_conds = Matrix{InitialCondition}(undef, lenght_devices_on, 2)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_on)
        g = get_component(ic)
        IS.@assert_op g == get_component(initial_conditions_off[ix])
        time_limits = PSY.get_time_limits(g)
        name = PSY.get_name(g)
        if !isnothing(time_limits)
            if (time_limits.up <= fraction_of_hour) & (time_limits.down <= fraction_of_hour)
                @debug "Generator $(name) has a nonbinding time limits. Constraints Skipped"
                continue
            else
                idx += 1
            end
            ini_conds[idx, 1] = ic
            ini_conds[idx, 2] = initial_conditions_off[ix]
            up_val = round(time_limits.up * steps_per_hour, RoundUp)
            down_val = round(time_limits.down * steps_per_hour, RoundUp)
            time_params[idx] = (up = up_val, down = down_val)
        end
    end
    if idx < lenght_devices_on
        ini_conds = ini_conds[1:idx, :]
        deleteat!(time_params, (idx + 1):lenght_devices_on)
    end
    return ini_conds, time_params
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{DurationConstraint},
    ::IS.FlattenIteratorWrapper{U},
    ::DeviceModel{U, V},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: AbstractThermalUnitCommitment}
    parameters = built_for_recurrent_solves(container)
    resolution = get_resolution(container)
    # Use getter functions that don't require creating the keys here
    initial_conditions_on = get_initial_condition(container, InitialTimeDurationOn(), U)
    initial_conditions_off = get_initial_condition(container, InitialTimeDurationOff(), U)
    ini_conds, time_params =
        _get_data_for_tdc(initial_conditions_on, initial_conditions_off, resolution)
    if !(isempty(ini_conds))
        if parameters
            device_duration_parameters!(
                container,
                time_params,
                ini_conds,
                DurationConstraint(),
                (OnVariable(), StartVariable(), StopVariable()),
                U,
            )
        else
            device_duration_retrospective!(
                container,
                time_params,
                ini_conds,
                DurationConstraint(),
                (OnVariable(), StartVariable(), StopVariable()),
                U,
            )
        end
    else
        @warn "Data doesn't contain generators with time-up/down limits, consider adjusting your formulation"
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{DurationConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, ThermalMultiStartUnitCommitment},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen}
    parameters = built_for_recurrent_solves(container)
    resolution = get_resolution(container)
    initial_conditions_on = get_initial_condition(container, InitialTimeDurationOn(), U)
    initial_conditions_off = get_initial_condition(container, InitialTimeDurationOff(), U)
    ini_conds, time_params =
        _get_data_for_tdc(initial_conditions_on, initial_conditions_off, resolution)
    if !(isempty(ini_conds))
        if parameters
            device_duration_parameters!(
                container,
                time_params,
                ini_conds,
                DurationConstraint(),
                (OnVariable(), StartVariable(), StopVariable()),
                U,
            )
        else
            device_duration_compact_retrospective!(
                container,
                time_params,
                ini_conds,
                DurationConstraint(),
                (OnVariable(), StartVariable(), StopVariable()),
                U,
            )
        end
    else
        @warn "Data doesn't contain generators with time-up/down limits, consider adjusting your formulation"
    end
    return
end

########################### Objective Function Calls#############################################
# These functions are custom implementations of the cost data. In the file objective_functions.jl there are default implementations. Define these only if needed.

function objective_function!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, U},
    ::Type{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen, U <: AbstractThermalUnitCommitment}
    add_variable_cost!(container, ActivePowerVariable(), devices, U())
    add_start_up_cost!(container, StartVariable(), devices, U())
    add_shut_down_cost!(container, StopVariable(), devices, U())
    add_proportional_cost!(container, OnVariable(), devices, U())
    if get_use_slacks(device_model)
        add_proportional_cost!(container, RateofChangeConstraintSlackUp(), devices, U())
        add_proportional_cost!(container, RateofChangeConstraintSlackDown(), devices, U())
    end
    return
end

function objective_function!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, U},
    ::Type{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen, U <: AbstractCompactUnitCommitment}
    add_variable_cost!(container, PowerAboveMinimumVariable(), devices, U())
    add_start_up_cost!(container, StartVariable(), devices, U())
    add_shut_down_cost!(container, StopVariable(), devices, U())
    add_proportional_cost!(container, OnVariable(), devices, U())
    if get_use_slacks(device_model)
        add_proportional_cost!(container, RateofChangeConstraintSlackUp(), devices, U())
        add_proportional_cost!(container, RateofChangeConstraintSlackDown(), devices, U())
    end
    return
end

function objective_function!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{PSY.ThermalMultiStart},
    device_model::DeviceModel{PSY.ThermalMultiStart, U},
    ::Type{<:PM.AbstractPowerModel},
) where {U <: ThermalMultiStartUnitCommitment}
    add_variable_cost!(container, PowerAboveMinimumVariable(), devices, U())
    for var_type in MULTI_START_VARIABLES
        add_start_up_cost!(container, var_type(), devices, U())
    end
    add_shut_down_cost!(container, StopVariable(), devices, U())
    add_proportional_cost!(container, OnVariable(), devices, U())
    if get_use_slacks(device_model)
        add_proportional_cost!(container, RateofChangeConstraintSlackUp(), devices, U())
        add_proportional_cost!(container, RateofChangeConstraintSlackDown(), devices, U())
    end
    return
end

function objective_function!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, U},
    ::Type{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen, U <: AbstractThermalDispatchFormulation}
    add_variable_cost!(container, ActivePowerVariable(), devices, U())
    if get_use_slacks(device_model)
        add_proportional_cost!(container, RateofChangeConstraintSlackUp(), devices, U())
        add_proportional_cost!(container, RateofChangeConstraintSlackDown(), devices, U())
    end
    return
end

function objective_function!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, U},
    ::Type{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen, U <: ThermalCompactDispatch}
    add_variable_cost!(container, PowerAboveMinimumVariable(), devices, U())
    if get_use_slacks(device_model)
        add_proportional_cost!(container, RateofChangeConstraintSlackUp(), devices, U())
        add_proportional_cost!(container, RateofChangeConstraintSlackDown(), devices, U())
    end
    return
end

function objective_function!(
    ::OptimizationContainer,
    ::IS.FlattenIteratorWrapper{PSY.ThermalMultiStart},
    ::DeviceModel{PSY.ThermalMultiStart, ThermalDispatchNoMin},
    ::Type{<:PM.AbstractPowerModel},
)
    throw(
        IS.ConflictingInputsError(
            "ThermalDispatchNoMin cost function is not compatible with ThermalMultiStart Devices.",
        ),
    )
end
