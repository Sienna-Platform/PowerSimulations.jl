_system_expression_type(::Type{PTDFPowerModel}) = PSY.System
_system_expression_type(::Type{CopperPlatePowerModel}) = PSY.System
_system_expression_type(::Type{AreaPTDFPowerModel}) = PSY.Area

function _ref_index(network_model::NetworkModel{<:PM.AbstractPowerModel}, bus::PSY.ACBus)
    return get_reference_bus(network_model, bus)
end

function _ref_index(::NetworkModel{AreaPTDFPowerModel}, device_bus::PSY.ACBus)
    return PSY.get_name(PSY.get_area(device_bus))
end

_get_variable_if_exists(::PSY.MarketBidCost) = nothing
_get_variable_if_exists(cost::PSY.OperationalCost) = PSY.get_variable(cost)

function add_expressions!(
    container::OptimizationContainer,
    ::Type{T},
    devices::U,
    model::DeviceModel{D, W},
) where {
    T <: ExpressionType,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    add_expression_container!(container, T(), D, names, time_steps)
    return
end

function add_expressions!(
    container::OptimizationContainer,
    ::Type{T},
    devices::U,
    model::DeviceModel{D, W},
) where {
    T <: FuelConsumptionExpression,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: AbstractDeviceFormulation,
} where {D <: PSY.Component}
    time_steps = get_time_steps(container)
    names = String[]
    found_quad_fuel_functions = false
    for d in devices
        op_cost = PSY.get_operation_cost(d)
        fuel_curve = _get_variable_if_exists(op_cost)
        if fuel_curve isa PSY.FuelCurve
            push!(names, PSY.get_name(d))
            if !found_quad_fuel_functions
                found_quad_fuel_functions =
                    PSY.get_value_curve(fuel_curve) isa PSY.QuadraticCurve
            end
        end
    end

    if !isempty(names)
        expr_type = found_quad_fuel_functions ? JuMP.QuadExpr : GAE
        add_expression_container!(
            container,
            T(),
            D,
            names,
            time_steps;
            expr_type = expr_type,
        )
    end
    return
end

function add_expressions!(
    container::OptimizationContainer,
    ::Type{T},
    devices::U,
    model::ServiceModel{V, W},
) where {
    T <: ExpressionType,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    V <: PSY.Reserve,
    W <: AbstractReservesFormulation,
} where {D <: PSY.Component}
    time_steps = get_time_steps(container)
    @assert length(devices) == 1
    add_expression_container!(
        container,
        T(),
        D,
        PSY.get_name.(devices),
        time_steps;
        meta = get_service_name(model),
    )
    return
end

# Note: add_to_jump_expression! are used to control depending on the parameter type used
# on the simulation.
function _add_to_jump_expression!(
    expression::T,
    var::JuMP.VariableRef,
    multiplier::Float64,
) where {T <: JuMP.AbstractJuMPScalar}
    JuMP.add_to_expression!(expression, multiplier, var)
    return
end

function _add_to_jump_expression!(
    expression::T,
    value::Float64,
) where {T <: JuMP.AbstractJuMPScalar}
    JuMP.add_to_expression!(expression, value)
    return
end

function _add_to_jump_expression!(
    expression::T,
    var::JuMP.VariableRef,
    multiplier::Float64,
    constant::Float64,
) where {T <: JuMP.AbstractJuMPScalar}
    _add_to_jump_expression!(expression, constant)
    _add_to_jump_expression!(expression, var, multiplier)
    return
end

function _add_to_jump_expression!(
    expression::T,
    parameter::Float64,
    multiplier::Float64,
) where {T <: JuMP.AbstractJuMPScalar}
    _add_to_jump_expression!(expression, parameter * multiplier)
    return
end

"""
Default implementation to add parameters to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: TimeSeriesParameter,
    V <: PSY.Device,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    param_container = get_parameter(container, U(), V)
    multiplier = get_multiplier_array(param_container)
    network_reduction = get_network_reduction(network_model)
    for d in devices, t in get_time_steps(container)
        bus_no = PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(d))
        name = PSY.get_name(d)
        _add_to_jump_expression!(
            get_expression(container, T(), PSY.ACBus)[bus_no, t],
            get_parameter_column_refs(param_container, name)[t],
            multiplier[name, t],
        )
    end
    return
end

"""
Generic electric load implementation to add parameters to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: TimeSeriesParameter,
    V <: PSY.ElectricLoad,
    W <: AbstractLoadFormulation,
    X <: PM.AbstractPowerModel,
}
    param_container = get_parameter(container, U(), V)
    multiplier = get_multiplier_array(param_container)
    network_reduction = get_network_reduction(network_model)
    ts_name = get_time_series_names(model)[U]
    ts_type = get_default_time_series_type(container)
    for d in devices
        bus_no = PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(d))
        name = PSY.get_name(d)
        has_ts = PSY.has_time_series(d, ts_type, ts_name)
        if !has_ts
            @warn "Device $(name) does not have time series of type $(ts_type) with name $(ts_name). Using default value of 1.0 for all time steps."
        end
        for t in get_time_steps(container)
            if has_ts
                param_value = get_parameter_column_refs(param_container, name)[t]
                mult = multiplier[name, t]
            else
                param_value = 1.0
                mult = get_multiplier_value(U(), d, W())
            end
            _add_to_jump_expression!(
                get_expression(container, T(), PSY.ACBus)[bus_no, t],
                param_value,
                mult,
            )
        end
    end
    return
end

"""
Motor load implementation to add constant power to ActivePowerBalance expression
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerTimeSeriesParameter,
    V <: PSY.MotorLoad,
    W <: StaticPowerLoad,
    X <: PM.AbstractPowerModel,
}
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no = PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                get_expression(container, T(), PSY.ACBus)[bus_no, t],
                PSY.get_active_power(d),
                -1.0,
            )
        end
    end
    return
end

"""
Motor load implementation to add constant power to ActivePowerBalance expression for AreaBalancePowerModel
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerTimeSeriesParameter,
    V <: PSY.MotorLoad,
    W <: StaticPowerLoad,
}
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus = PSY.get_bus(d)
        area_name = PSY.get_name(PSY.get_area(bus))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                get_expression(container, T(), PSY.Area)[area_name, t],
                PSY.get_active_power(d),
                -1.0,
            )
        end
    end
    return
end

"""
Motor load implementation to add constant power to ActivePowerBalance expression
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ReactivePowerBalance,
    U <: ReactivePowerTimeSeriesParameter,
    V <: PSY.MotorLoad,
    W <: StaticPowerLoad,
    X <: PM.ACPPowerModel,
}
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no = PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                get_expression(container, T(), PSY.ACBus)[bus_no, t],
                PSY.get_reactive_power(d),
                -1.0,
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: SystemBalanceExpressions,
    U <: TimeSeriesParameter,
    V <: PSY.Device,
    W <: AbstractDeviceFormulation,
}
    param_container = get_parameter(container, U(), V)
    multiplier = get_multiplier_array(param_container)
    for d in devices, t in get_time_steps(container)
        bus = PSY.get_bus(d)
        area_name = PSY.get_name(PSY.get_area(bus))
        name = PSY.get_name(d)
        _add_to_jump_expression!(
            get_expression(container, T(), PSY.Area)[area_name, t],
            get_parameter_column_refs(param_container, name)[t],
            multiplier[name, t],
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: SystemBalanceExpressions,
    U <: TimeSeriesParameter,
    V <: PSY.ElectricLoad,
    W <: AbstractLoadFormulation,
}
    param_container = get_parameter(container, U(), V)
    multiplier = get_multiplier_array(param_container)
    ts_name = get_time_series_names(model)[U]
    ts_type = get_default_time_series_type(container)
    for d in devices
        bus = PSY.get_bus(d)
        area_name = PSY.get_name(PSY.get_area(bus))
        name = PSY.get_name(d)
        has_ts = PSY.has_time_series(d, ts_type, ts_name)
        if !has_ts
            @warn "Device $(name) does not have time series of type $(ts_type) with name $(ts_name). Using default value of 1.0 for all time steps."
        end
        for t in get_time_steps(container)
            if has_ts
                param_value = get_parameter_column_refs(param_container, name)[t]
                mult = multiplier[name, t]
            else
                param_value = 1.0
                mult = get_multiplier_value(U(), d, W())
            end
            _add_to_jump_expression!(
                get_expression(container, T(), PSY.Area)[area_name, t],
                param_value,
                mult,
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: OnStatusParameter,
    V <: PSY.ThermalGen,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    parameter = get_parameter_array(container, U(), V)
    network_reduction = get_network_reduction(network_model)
    for d in devices, t in get_time_steps(container)
        bus_no = PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(d))
        name = PSY.get_name(d)
        mult = get_expression_multiplier(U(), T(), d, W())
        _add_to_jump_expression!(
            get_expression(container, T(), PSY.ACBus)[bus_no, t],
            parameter[name, t],
            mult,
        )
    end
    return
end

"""
Default implementation to add device variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: VariableType,
    V <: PSY.StaticInjection,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        bus_no = PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[bus_no, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: SystemBalanceExpressions,
    U <: VariableType,
    V <: PSY.StaticInjection,
    W <: AbstractDeviceFormulation,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.Area)
    for d in devices, t in get_time_steps(container)
        name = PSY.get_name(d)
        bus = PSY.get_bus(d)
        area_name = PSY.get_name(PSY.get_area(bus))
        _add_to_jump_expression!(
            expression[area_name, t],
            variable[name, t],
            get_variable_multiplier(U(), V, W()),
        )
    end
    return
end

"""
Default implementation to add branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: HVDCLosses,
    V <: PSY.TwoTerminalHVDC,
    W <: HVDCTwoTerminalDispatch,
    X <: Union{PTDFPowerModel, CopperPlatePowerModel},
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.System)
    for d in devices
        name = PSY.get_name(d)
        device_bus_from = PSY.get_arc(d).from
        device_bus_to = PSY.get_arc(d).to
        ref_bus_from = get_reference_bus(network_model, device_bus_from)
        ref_bus_to = get_reference_bus(network_model, device_bus_to)
        if ref_bus_from == ref_bus_to
            for t in get_time_steps(container)
                _add_to_jump_expression!(
                    expression[ref_bus_from, t],
                    variable[name, t],
                    get_variable_multiplier(U(), d, W()),
                )
            end
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: HVDCLosses,
    V <: PSY.TwoTerminalHVDC,
    W <: HVDCTwoTerminalDispatch,
    X <:
    Union{AreaPTDFPowerModel, AreaBalancePowerModel},
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.Area)
    for d in devices
        name = PSY.get_name(d)
        device_bus_from = PSY.get_arc(d).from
        area_name = PSY.get_name(PSY.get_area(device_bus_from))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[area_name, t],
                variable[name, t],
                get_variable_multiplier(U(), d, W()),
            )
        end
    end
    return
end

"""
Default implementation to add branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{Union{PTDFPowerModel}},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerToFromVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: AbstractTwoTerminalDCLineFormulation,
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    sys_expr = get_expression(container, T(), PSY.System)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_to = PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).to)
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_to, t], flow_variable, -1.0)
            if ref_bus_from != ref_bus_to
                _add_to_jump_expression!(sys_expr[ref_bus_to, t], flow_variable, -1.0)
            end
        end
    end
    return
end

"""
Default implementation to add branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerFromToVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: AbstractTwoTerminalDCLineFormulation,
    X <: AbstractPTDFModel,
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    sys_expr = get_expression(container, T(), _system_expression_type(X))
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_from =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_from, t], flow_variable, -1.0)
            if ref_bus_from != ref_bus_to
                _add_to_jump_expression!(sys_expr[ref_bus_from, t], flow_variable, -1.0)
            end
        end
    end
    return
end

"""
PWL implementation to add FromTo branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: HVDCActivePowerReceivedFromVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: HVDCTwoTerminalPiecewiseLoss,
    X <: AbstractPTDFModel,
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    sys_expr = get_expression(container, T(), _system_expression_type(X))
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_from =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_from, t], flow_variable, 1.0)
            if ref_bus_from != ref_bus_to
                _add_to_jump_expression!(sys_expr[ref_bus_from, t], flow_variable, 1.0)
            end
        end
    end
    return
end

"""
HVDC LCC implementation to add ActivePowerBalance expression for HVDCActivePowerReceivedFromVariable variable
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,                        # expression
    U <: HVDCActivePowerReceivedFromVariable,       # variable
    V <: PSY.TwoTerminalHVDC,                      # power system type
    W <: HVDCTwoTerminalLCC,                        # formulation
    X <: ACPPowerModel,                             # network model
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_from =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).from)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_from, t], flow_variable, -1.0)
        end
    end
    return
end

"""
HVDC LCC implementation to add ActivePowerBalance expression for HVDCActivePowerReceivedToVariable variable
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: HVDCActivePowerReceivedToVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: HVDCTwoTerminalLCC,
    X <: ACPPowerModel,
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_to =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).to)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_to, t], flow_variable, 1.0)
        end
    end
    return
end

"""
HVDC LCC implementation to add ReactivePowerBalance expression for HVDCReactivePowerReceivedFromVariable variable
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ReactivePowerBalance,                        # expression
    U <: HVDCReactivePowerReceivedFromVariable,     # variable
    V <: PSY.TwoTerminalHVDC,                      # power system type
    W <: HVDCTwoTerminalLCC,                        # formulation
    X <: ACPPowerModel,                             # network model
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_from =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).from)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_from, t], flow_variable, -1.0)
        end
    end
    return
end

"""
HVDC LCC implementation to add ReactivePowerBalance expression for HVDCReactivePowerReceivedToVariable variable
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ReactivePowerBalance,
    U <: HVDCReactivePowerReceivedToVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: HVDCTwoTerminalLCC,
    X <: ACPPowerModel,
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_to =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).to)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_to, t], flow_variable, -1.0)
        end
    end
    return
end

"""
PWL implementation to add FromTo branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: HVDCActivePowerReceivedToVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: HVDCTwoTerminalPiecewiseLoss,
    X <: AbstractPTDFModel,
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    sys_expr = get_expression(container, T(), _system_expression_type(X))
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_to =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).to)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_to, t], flow_variable, 1.0)
            if ref_bus_from != ref_bus_to
                _add_to_jump_expression!(sys_expr[ref_bus_to, t], flow_variable, 1.0)
            end
        end
    end
    return
end

"""
Default implementation to add branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerToFromVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: AbstractTwoTerminalDCLineFormulation,
    X <: CopperPlatePowerModel,
}
    if has_subnetworks(network_model)
        var = get_variable(container, U(), V)
        sys_expr = get_expression(container, T(), PSY.System)
        for d in devices
            ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
            ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
            for t in get_time_steps(container)
                flow_variable = var[PSY.get_name(d), t]
                if ref_bus_from != ref_bus_to
                    _add_to_jump_expression!(sys_expr[ref_bus_to, t], flow_variable, 1.0)
                end
            end
        end
    end
    return
end

"""
Default implementation to add branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerFromToVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: AbstractTwoTerminalDCLineFormulation,
    X <: CopperPlatePowerModel,
}
    if has_subnetworks(network_model)
        var = get_variable(container, U(), V)
        sys_expr = get_expression(container, T(), PSY.System)
        for d in devices
            ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
            ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
            for t in get_time_steps(container)
                flow_variable = var[PSY.get_name(d), t]
                if ref_bus_from != ref_bus_to
                    _add_to_jump_expression!(sys_expr[ref_bus_to, t], flow_variable, -1.0)
                end
            end
        end
    end
    return
end

"""
Default implementation to add branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerFromToVariable,
    V <: PSY.Branch,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        bus_no_ = PSY.get_number(PSY.get_arc(d).from)
        bus_no = PNM.get_mapped_bus_number(network_reduction, bus_no_)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[bus_no, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
        end
    end
    return
end

"""
Default implementation to add branch variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerToFromVariable,
    V <: PSY.ACBranch,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        bus_no_ = PSY.get_number(PSY.get_arc(d).to)
        bus_no = PNM.get_mapped_bus_number(network_reduction, bus_no_)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[bus_no, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, HVDCTwoTerminalDispatch},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerToFromVariable,
    V <: PSY.TwoTerminalHVDC,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.Area)
    for d in devices
        name = PSY.get_name(d)
        area_name = PSY.get_name(PSY.get_area(PSY.get_arc(d).to))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[area_name, t],
                variable[name, t],
                get_variable_multiplier(U(), V, HVDCTwoTerminalDispatch()),
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: SystemBalanceExpressions,
    U <: OnVariable,
    V <: PSY.ThermalGen,
    W <: AbstractCompactUnitCommitment,
}
    _add_to_expression!(
        container,
        T,
        U,
        devices,
        device_model,
        network_model,
    )
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: OnVariable,
    V <: PSY.ThermalGen,
    W <: AbstractCompactUnitCommitment,
    X <: PM.AbstractPowerModel,
}
    _add_to_expression!(
        container,
        T,
        U,
        devices,
        device_model,
        network_model,
    )
    return
end

function _add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: OnVariable,
    V <: PSY.ThermalGen,
    W <: AbstractCompactUnitCommitment,
    X <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        bus_no_ = PSY.get_number(PSY.get_bus(d))
        bus_no = PNM.get_mapped_bus_number(network_reduction, bus_no_)
        for t in get_time_steps(container)
            if PSY.get_must_run(d)
                _add_to_jump_expression!(
                    expression[bus_no, t],
                    get_variable_multiplier(U(), d, W()),
                )
            else
                _add_to_jump_expression!(
                    expression[bus_no, t],
                    variable[name, t],
                    get_variable_multiplier(U(), d, W()),
                )
            end
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: SystemBalanceExpressions,
    U <: OnVariable,
    V <: PSY.ThermalGen,
    W <: Union{AbstractCompactUnitCommitment, ThermalCompactDispatch},
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.Area)
    for d in devices
        name = PSY.get_name(d)
        bus = PSY.get_bus(d)
        area_name = PSY.get_name(PSY.get_area(bus))
        for t in get_time_steps(container)
            if PSY.get_must_run(d)
                _add_to_jump_expression!(
                    expression[area_name, t],
                    get_variable_multiplier(U(), d, W()),
                )
            else
                _add_to_jump_expression!(
                    expression[area_name, t],
                    variable[name, t],
                    get_variable_multiplier(U(), d, W()),
                )
            end
        end
    end
    return
end

"""
Default implementation to add parameters to Copperplate SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: SystemBalanceExpressions,
    U <: TimeSeriesParameter,
    V <: PSY.StaticInjection,
    W <: AbstractDeviceFormulation,
}
    param_container = get_parameter(container, U(), V)
    multiplier = get_multiplier_array(param_container)
    expression = get_expression(container, T(), PSY.System)
    for d in devices
        device_bus = PSY.get_bus(d)
        ref_bus = get_reference_bus(network_model, device_bus)
        name = PSY.get_name(d)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[ref_bus, t],
                get_parameter_column_refs(param_container, name)[t],
                multiplier[name, t],
            )
        end
    end
end

"""
Electric Load implementation to add parameters to Copperplate SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: SystemBalanceExpressions,
    U <: TimeSeriesParameter,
    V <: PSY.ElectricLoad,
    W <: AbstractLoadFormulation,
}
    param_container = get_parameter(container, U(), V)
    multiplier = get_multiplier_array(param_container)
    expression = get_expression(container, T(), PSY.System)
    ts_name = get_time_series_names(device_model)[U]
    ts_type = get_default_time_series_type(container)
    for d in devices
        device_bus = PSY.get_bus(d)
        ref_bus = get_reference_bus(network_model, device_bus)
        name = PSY.get_name(d)
        has_ts = PSY.has_time_series(d, ts_type, ts_name)
        if !has_ts
            @warn "Device $(name) does not have time series of type $(ts_type) with name $(ts_name). Using default value of 1.0 for all time steps."
        end
        for t in get_time_steps(container)
            if has_ts
                param_value = get_parameter_column_refs(param_container, name)[t]
                mult = multiplier[name, t]
            else
                param_value = 1.0
                mult = get_multiplier_value(U(), d, W())
            end
            _add_to_jump_expression!(
                expression[ref_bus, t],
                param_value,
                mult,
            )
        end
    end
    return
end

"""
Motor load implementation to add parameters to SystemBalanceExpressions CopperPlate
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerTimeSeriesParameter,
    V <: PSY.MotorLoad,
    W <: StaticPowerLoad,
}
    expression = get_expression(container, T(), PSY.System)
    for d in devices
        device_bus = PSY.get_bus(d)
        ref_bus = get_reference_bus(network_model, device_bus)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[ref_bus, t],
                PSY.get_active_power(d),
                -1.0,
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: OnStatusParameter,
    V <: PSY.ThermalGen,
    W <: AbstractDeviceFormulation,
}
    parameter = get_parameter_array(container, U(), V)
    expression = get_expression(container, T(), PSY.System)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        ref_bus = get_reference_bus(network_model, device_bus)
        for t in get_time_steps(container)
            mult = get_expression_multiplier(U(), T(), d, W())
            _add_to_jump_expression!(expression[ref_bus, t], parameter[name, t], mult)
        end
    end
    return
end

"""
Default implementation to add variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: VariableType,
    V <: PSY.StaticInjection,
    W <: AbstractDeviceFormulation,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.System)
    for d in devices
        device_bus = PSY.get_bus(d)
        ref_bus = get_reference_bus(network_model, device_bus)
        name = PSY.get_name(d)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[ref_bus, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: OnVariable,
    V <: PSY.ThermalGen,
    W <: AbstractCompactUnitCommitment,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.System)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        ref_bus = get_reference_bus(network_model, device_bus)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[ref_bus, t],
                variable[name, t],
                get_variable_multiplier(U(), d, W()),
            )
        end
    end
    return
end

"""
Default implementation to add parameters to PTDF SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: TimeSeriesParameter,
    V <: PSY.StaticInjection,
    W <: AbstractDeviceFormulation,
    X <: AbstractPTDFModel,
}
    param_container = get_parameter(container, U(), V)
    multiplier = get_multiplier_array(param_container)
    sys_expr = get_expression(container, T(), _system_expression_type(X))
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        bus_no = PNM.get_mapped_bus_number(network_reduction, PSY.get_number(device_bus))
        ref_index = _ref_index(network_model, device_bus)
        param = get_parameter_column_refs(param_container, name)
        for t in get_time_steps(container)
            _add_to_jump_expression!(sys_expr[ref_index, t], param[t], multiplier[name, t])
            _add_to_jump_expression!(nodal_expr[bus_no, t], param[t], multiplier[name, t])
        end
    end
    return
end

"""
Electric Load implementation to add parameters to PTDF SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: TimeSeriesParameter,
    V <: PSY.ElectricLoad,
    W <: AbstractLoadFormulation,
    X <: AbstractPTDFModel,
}
    param_container = get_parameter(container, U(), V)
    multiplier = get_multiplier_array(param_container)
    sys_expr = get_expression(container, T(), _system_expression_type(X))
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    ts_name = get_time_series_names(device_model)[U]
    ts_type = get_default_time_series_type(container)
    for d in devices
        name = PSY.get_name(d)
        has_ts = PSY.has_time_series(d, ts_type, ts_name)
        if !has_ts
            @warn "Device $(name) does not have time series of type $(ts_type) with name $(ts_name). Using default value of 1.0 for all time steps."
        end
        device_bus = PSY.get_bus(d)
        bus_no_ = PSY.get_number(device_bus)
        bus_no = PNM.get_mapped_bus_number(network_reduction, bus_no_)
        ref_index = _ref_index(network_model, device_bus)
        for t in get_time_steps(container)
            if has_ts
                param = get_parameter_column_refs(param_container, name)[t]
                mult = multiplier[name, t]
            else
                param = 1.0
                mult = get_multiplier_value(U(), d, W())
            end
            _add_to_jump_expression!(sys_expr[ref_index, t], param, mult)
            _add_to_jump_expression!(nodal_expr[bus_no, t], param, mult)
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: OnStatusParameter,
    V <: PSY.ThermalGen,
    W <: AbstractDeviceFormulation,
    X <: AbstractPTDFModel,
}
    parameter = get_parameter_array(container, U(), V)
    sys_expr = get_expression(container, T(), PSY.System)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        bus_no_ = PSY.get_number(PSY.get_bus(d))
        bus_no = PNM.get_mapped_bus_number(network_reduction, bus_no_)
        mult = get_expression_multiplier(U(), T(), d, W())
        device_bus = PSY.get_bus(d)
        ref_index = _ref_index(network_model, device_bus)
        for t in get_time_steps(container)
            _add_to_jump_expression!(sys_expr[ref_index, t], parameter[name, t], mult)
            _add_to_jump_expression!(nodal_expr[bus_no, t], parameter[name, t], mult)
        end
    end
    return
end

"""
Default implementation to add variables to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: VariableType,
    V <: PSY.StaticInjection,
    W <: AbstractDeviceFormulation,
    X <: PTDFPowerModel,
}
    variable = get_variable(container, U(), V)
    sys_expr = get_expression(container, T(), PSY.System)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        bus_no = PNM.get_mapped_bus_number(network_reduction, device_bus)
        ref_index = _ref_index(network_model, device_bus)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                sys_expr[ref_index, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
            _add_to_jump_expression!(
                nodal_expr[bus_no, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
        end
    end
    return
end

"""
Motor Load implementation to add constant motor power to PTDF SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerTimeSeriesParameter,
    V <: PSY.MotorLoad,
    W <: StaticPowerLoad,
    X <: AbstractPTDFModel,
}
    sys_expr = get_expression(container, T(), PSY.System)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        device_bus = PSY.get_bus(d)
        bus_no = PNM.get_mapped_bus_number(network_reduction, device_bus)
        ref_index = _ref_index(network_model, device_bus)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                sys_expr[ref_index, t],
                PSY.get_active_power(d),
                -1.0,
            )
            _add_to_jump_expression!(
                nodal_expr[bus_no, t],
                PSY.get_active_power(d),
                -1.0,
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.StaticInjection,
    W <: AbstractDeviceFormulation,
    X <: AreaPTDFPowerModel,
}
    variable = get_variable(container, U(), V)
    area_expr = get_expression(container, T(), PSY.Area)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        area_name = PSY.get_name(PSY.get_area(device_bus))
        bus_no = PNM.get_mapped_bus_number(network_reduction, device_bus)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                area_expr[area_name, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
            _add_to_jump_expression!(
                nodal_expr[bus_no, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
        end
    end
    return
end

# The on variables are included in the system balance expressions becuase they
# are multiplied by the Pmin and the active power is not the total active power
# but the power above minimum.
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: OnVariable,
    V <: PSY.ThermalGen,
    W <: AbstractCompactUnitCommitment,
    X <: PTDFPowerModel,
}
    variable = get_variable(container, U(), V)
    sys_expr = get_expression(container, T(), _system_expression_type(PTDFPowerModel))
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        bus_no = PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(d))
        ref_index = _ref_index(network_model, PSY.get_bus(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                sys_expr[ref_index, t],
                variable[name, t],
                get_variable_multiplier(U(), d, W()),
            )
            _add_to_jump_expression!(
                nodal_expr[bus_no, t],
                variable[name, t],
                get_variable_multiplier(U(), d, W()),
            )
        end
    end
    return
end

"""
Implementation of add_to_expression! for lossless branch/network models
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerVariable,
    V <: PSY.ACBranch,
    W <: AbstractBranchFormulation,
    X <: PM.AbstractActivePowerModel,
}
    var = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_from =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).from)
        bus_no_to = PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).to)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(
                expression[bus_no_from, t],
                flow_variable,
                -1.0,
            )
            _add_to_jump_expression!(
                expression[bus_no_to, t],
                flow_variable,
                1.0,
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{ActivePowerBalance},
    ::Type{FlowActivePowerVariable},
    devices::IS.FlattenIteratorWrapper{PSY.AreaInterchange},
    ::DeviceModel{PSY.AreaInterchange, W},
    network_model::NetworkModel{U},
) where {
    W <: AbstractBranchFormulation,
    U <: Union{AreaBalancePowerModel, AreaPTDFPowerModel},
}
    flow_variable = get_variable(container, FlowActivePowerVariable(), PSY.AreaInterchange)
    expression = get_expression(container, ActivePowerBalance(), PSY.Area)
    for d in devices
        area_from_name = PSY.get_name(PSY.get_from_area(d))
        area_to_name = PSY.get_name(PSY.get_to_area(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression[area_from_name, t],
                flow_variable[PSY.get_name(d), t],
                -1.0,
            )
            _add_to_jump_expression!(
                expression[area_to_name, t],
                flow_variable[PSY.get_name(d), t],
                1.0,
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{ActivePowerBalance},
    ::Type{FlowActivePowerVariable},
    devices::IS.FlattenIteratorWrapper{PSY.AreaInterchange},
    ::DeviceModel{PSY.AreaInterchange, W},
    network_model::NetworkModel{U},
) where {
    W <: AbstractBranchFormulation,
    U <: PM.AbstractActivePowerModel,
}
    @debug "AreaInterchanges do not contribute to ActivePowerBalance expressions in non-area models."
    return
end

"""
Implementation of add_to_expression! for lossless branch/network models
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerVariable,
    V <: PSY.TwoTerminalHVDC,
    W <: AbstractBranchFormulation,
    X <: PTDFPowerModel,
}
    var = get_variable(container, U(), V)
    nodal_expr = get_expression(container, T(), PSY.ACBus)
    sys_expr = get_expression(container, T(), PSY.System)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_from =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).from)
        bus_no_to = PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).to)
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(nodal_expr[bus_no_from, t], flow_variable, -1.0)
            _add_to_jump_expression!(nodal_expr[bus_no_to, t], flow_variable, 1.0)
            if ref_bus_from != ref_bus_to
                _add_to_jump_expression!(sys_expr[ref_bus_from, t], flow_variable, -1.0)
                _add_to_jump_expression!(sys_expr[ref_bus_to, t], flow_variable, 1.0)
            end
        end
    end
    return
end

"""
Implementation of add_to_expression! for lossless branch/network models
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: FlowActivePowerVariable,
    V <: PSY.ACBranch,
    W <: AbstractBranchFormulation,
}
    inter_network_branches = V[]
    for d in devices
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        if ref_bus_from != ref_bus_to
            push!(inter_network_branches, d)
        end
    end
    if !isempty(inter_network_branches)
        var = get_variable(container, U(), V)
        sys_expr = get_expression(container, T(), PSY.System)
        for d in devices
            ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
            ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
            if ref_bus_from == ref_bus_to
                continue
            end
            for t in get_time_steps(container)
                flow_variable = var[PSY.get_name(d), t]
                _add_to_jump_expression!(sys_expr[ref_bus_from, t], flow_variable, -1.0)
                _add_to_jump_expression!(sys_expr[ref_bus_to, t], flow_variable, 1.0)
            end
        end
    end
    return
end

"""
Implementation of add_to_expression! for lossless branch/network models
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{PSY.PhaseShiftingTransformer},
    ::DeviceModel{PSY.PhaseShiftingTransformer, V},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: ActivePowerBalance, U <: PhaseShifterAngle, V <: PhaseAngleControl}
    var = get_variable(container, U(), PSY.PhaseShiftingTransformer)
    expression = get_expression(container, T(), PSY.ACBus)
    network_reduction = get_network_reduction(network_model)
    for d in devices
        bus_no_from =
            PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).from)
        bus_no_to = PNM.get_mapped_bus_number(network_reduction, PSY.get_arc(d).to)
        for t in get_time_steps(container)
            flow_variable = var[PSY.get_name(d), t]
            _add_to_jump_expression!(
                expression[bus_no_from, t],
                flow_variable,
                -get_variable_multiplier(U(), d, V()),
            )
            _add_to_jump_expression!(
                expression[bus_no_to, t],
                flow_variable,
                get_variable_multiplier(U(), d, V()),
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: Union{ActivePowerRangeExpressionUB, ActivePowerRangeExpressionLB},
    U <: VariableType,
    V <: PSY.Device,
    W <: AbstractDeviceFormulation,
    X <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), V)
    if !has_container_key(container, T, V)
        add_expressions!(container, T, devices, model)
    end
    expression = get_expression(container, T(), V)
    for d in devices, t in get_time_steps(container)
        name = PSY.get_name(d)
        _add_to_jump_expression!(expression[name, t], variable[name, t], 1.0)
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::ServiceModel{X, W},
) where {
    T <: ActivePowerRangeExpressionUB,
    U <: VariableType,
    V <: PSY.Component,
    X <: PSY.Reserve{PSY.ReserveUp},
    W <: AbstractReservesFormulation,
}
    service_name = get_service_name(model)
    variable = get_variable(container, U(), X, service_name)
    if !has_container_key(container, T, V)
        add_expressions!(container, T, devices, model)
    end
    expression = get_expression(container, T(), V)
    for d in devices, t in get_time_steps(container)
        name = PSY.get_name(d)
        _add_to_jump_expression!(expression[name, t], variable[name, t], 1.0)
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{InterfaceTotalFlow},
    ::Type{T},
    service::PSY.TransmissionInterface,
    model::ServiceModel{PSY.TransmissionInterface, U},
) where {
    T <: Union{InterfaceFlowSlackUp, InterfaceFlowSlackDown},
    U <: Union{ConstantMaxInterfaceFlow, VariableMaxInterfaceFlow},
}
    expression = get_expression(container, InterfaceTotalFlow(), PSY.TransmissionInterface)
    service_name = PSY.get_name(service)
    variable = get_variable(container, T(), PSY.TransmissionInterface, service_name)
    for t in get_time_steps(container)
        _add_to_jump_expression!(
            expression[service_name, t],
            variable[t],
            get_variable_multiplier(T(), service, U()),
        )
    end
    return
end

function _handle_nodal_or_zonal_interfaces(
    br_type::Type{V},
    net_reduction_data::PNM.NetworkReductionData,
    direction_map::Dict{String, Int},
    contributing_devices::Vector{V},
    variable::JuMPVariableArray,
    expression::DenseAxisArray, # There is no good type for a DenseAxisArray slice
) where {V <: PSY.ACTransmission}
    all_branch_maps_by_type = net_reduction_data.all_branch_maps_by_type
    for (name, (arc, reduction)) in
        PNM.get_name_to_arc_map(net_reduction_data, br_type)
        reduction_entry = all_branch_maps_by_type[reduction][br_type][arc]
        if _reduced_entry_in_interface(reduction_entry, contributing_devices)
            if isempty(direction_map)
                direction = 1.0
            else
                direction = _get_direction(
                    arc,
                    reduction_entry,
                    direction_map,
                    net_reduction_data,
                )
            end
            for t in axes(variable, 2)
                _add_to_jump_expression!(
                    expression[t],
                    variable[name, t],
                    Float64(direction),
                )
            end
        end
    end
    return
end

function _handle_nodal_or_zonal_interfaces(
    ::Type{PSY.AreaInterchange},
    net_reduction_data::PNM.NetworkReductionData,
    direction_map::Dict{String, Int},
    contributing_devices::Vector{PSY.AreaInterchange},
    variable::JuMPVariableArray,
    expression::DenseAxisArray, # There is no good type for a DenseAxisArray slice
)
    for device in contributing_devices
        name = PSY.get_name(device)
        if isempty(direction_map)
            direction = 1.0
        else
            direction = direction_map[name]
        end
        for t in axes(variable, 2)
            _add_to_jump_expression!(
                expression[t],
                variable[name, t],
                Float64(direction),
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{InterfaceTotalFlow},
    ::Type{FlowActivePowerVariable},
    service::PSY.TransmissionInterface,
    model::ServiceModel{PSY.TransmissionInterface, V},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {V <: Union{ConstantMaxInterfaceFlow, VariableMaxInterfaceFlow}}
    net_reduction_data = get_network_reduction(network_model)
    expression = get_expression(container, InterfaceTotalFlow(), PSY.TransmissionInterface)
    service_name = get_service_name(model)
    direction_map = PSY.get_direction_mapping(service)
    contributing_devices_map = get_contributing_devices_map(model)
    for (br_type, contributing_devices) in contributing_devices_map
        variable = get_variable(container, FlowActivePowerVariable(), br_type)
        _handle_nodal_or_zonal_interfaces(
            br_type,
            net_reduction_data,
            direction_map,
            contributing_devices,
            variable,
            expression[service_name, :],
        )
    end
    return
end

function _is_interchanges_interfaces(
    contributing_devices_map::Dict{Type{<:PSY.Component}, Vector{<:PSY.Component}},
)
    if PSY.AreaInterchange ∈ keys(contributing_devices_map)
        @assert length(keys(contributing_devices_map)) == 1
        return true
    end
    return false
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{InterfaceTotalFlow},
    ::Type{FlowActivePowerVariable},
    service::PSY.TransmissionInterface,
    model::ServiceModel{PSY.TransmissionInterface, V},
    network_model::NetworkModel{AreaPTDFPowerModel},
) where {V <: Union{ConstantMaxInterfaceFlow, VariableMaxInterfaceFlow}}
    net_reduction_data = get_network_reduction(network_model)
    expression = get_expression(container, InterfaceTotalFlow(), PSY.TransmissionInterface)
    service_name = get_service_name(model)
    direction_map = PSY.get_direction_mapping(service)
    contributing_devices_map = get_contributing_devices_map(model)
    # Ignore interfaces over lines for AreaPTDFModel
    if !_is_interchanges_interfaces(contributing_devices_map)
        return
    end
    variable = get_variable(container, FlowActivePowerVariable(), PSY.AreaInterchange)
    _handle_nodal_or_zonal_interfaces(
        PSY.AreaInterchange,
        net_reduction_data,
        direction_map,
        contributing_devices_map[PSY.AreaInterchange],
        variable,
        expression[service_name, :],
    )
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{InterfaceTotalFlow},
    ::Type{PTDFBranchFlow},
    service::PSY.TransmissionInterface,
    model::ServiceModel{PSY.TransmissionInterface, V},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {V <: Union{ConstantMaxInterfaceFlow, VariableMaxInterfaceFlow}}
    net_reduction_data = get_network_reduction(network_model)
    expression = get_expression(container, InterfaceTotalFlow(), PSY.TransmissionInterface)
    service_name = get_service_name(model)
    direction_map = PSY.get_direction_mapping(service)
    contributing_devices_map = get_contributing_devices_map(model)
    # Interfaces over interchanges
    if _is_interchanges_interfaces(contributing_devices_map)
        return
    end

    for (br_type, contributing_devices) in contributing_devices_map
        flow_expression = get_expression(container, PTDFBranchFlow(), br_type)
        all_branch_maps_by_type = net_reduction_data.all_branch_maps_by_type
        for (name, (arc, reduction)) in PNM.get_name_to_arc_map(net_reduction_data, br_type)
            reduction_entry = all_branch_maps_by_type[reduction][br_type][arc]
            if _reduced_entry_in_interface(reduction_entry, contributing_devices)
                if isempty(direction_map)
                    direction = 1.0
                else
                    direction = _get_direction(
                        arc,
                        reduction_entry,
                        direction_map,
                        net_reduction_data,
                    )
                end
                for t in axes(flow_expression, 2)
                    JuMP.add_to_expression!(
                        expression[service_name, t],
                        flow_expression[name, t],
                        Float64(direction),
                    )
                end
            end
        end
    end
    return
end

function _get_direction(
    ::Tuple{Int, Int},
    reduction_entry::PSY.ACTransmission,
    direction_map::Dict{String, Int},
    ::PNM.NetworkReductionData,
)
    name = PSY.get_name(reduction_entry)
    if !haskey(direction_map, name)
        @warn "Direction not found for $(summary(reduction_entry)). Will use the default from -> to direction"
        return 1.0
    else
        return direction_map[name]
    end
end

function _get_direction(
    arc_tuple::Tuple{Int, Int},
    reduction_entry::PNM.BranchesParallel,
    direction_map::Dict{String, Int},
    net_reduction_data::PNM.NetworkReductionData,
)
    # Loops through parallel branches twice, but there are relatively few parallel branches per reduction entry:
    directions = [
        _get_direction(arc_tuple, x, direction_map, net_reduction_data) for
        x in reduction_entry
    ]
    if allequal(directions)
        return first(directions)
    end
    throw(
        ArgumentError(
            "The interface direction mapping contains a double circuit with opposite directions. Modify the data to have consistent directions for double circuits.",
        ),
    )
end

function _get_direction(
    arc_tuple::Tuple{Int, Int},
    reduction_entry::PNM.BranchesSeries,
    direction_map::Dict{String, Int},
    net_reduction_data::PNM.NetworkReductionData,
)
    # direction of segments from the user provided mapping:
    mapping_directions = [
        _get_direction(arc_tuple, x, direction_map, net_reduction_data) for
        x in reduction_entry
    ]
    # direction of segments relative to the reduced degree two chain:
    _, segment_orientations =
        PNM._get_chain_data(arc_tuple, reduction_entry, net_reduction_data)
    segment_directions = [x == :FromTo ? 1.0 : -1.0 for x in segment_orientations]
    net_directions = mapping_directions .* segment_directions
    if allequal(net_directions)
        return first(net_directions)
    else
        throw(
            ArgumentError(
                "The interface direction mapping for degree two chain with arc $(arc_tuple) is inconsistent. Check the mapping entries and the orientation of the segment arcs within the chain.",
            ),
        )
    end
end

# These checks can be moved to happen at the service template check level
function _reduced_entry_in_interface(
    reduction_entry::PSY.ACTransmission,
    contributing_devices::Vector{<:PSY.ACTransmission},
)
    reduction_entry_name = PSY.get_name(reduction_entry)
    # This is compared by name given that the reduction data uses copies of the devices
    # so, simple comparisons will not work
    for device in contributing_devices
        device_name = PSY.get_name(device)
        if reduction_entry_name == device_name
            return true
        end
    end
    return false
end

function _reduced_entry_in_interface(
    reduction_entry::PNM.BranchesParallel,
    contributing_devices::Vector{<:PSY.ACTransmission},
)
    in_interface = [
        _reduced_entry_in_interface(x, contributing_devices) for
        x in reduction_entry
    ]

    if !allequal(in_interface)
        branch_names = PSY.get_name.(reduction_entry)
        throw(
            ArgumentError(
                "An interface is specified with only part of a double-circuit that has been reduced. 
                Branches: $(branch_names[in_interface]) are in the interface and branches: $(branch_names[.!in_interface]) are not.
                Modify the data to include all of or none of the parallel segements.",
            ),
        )
    end
    return first(in_interface)
end

function _reduced_entry_in_interface(
    reduction_entry::PNM.BranchesSeries,
    contributing_devices::Vector{<:PSY.ACTransmission},
)
    in_interface = [
        _reduced_entry_in_interface(x, contributing_devices) for
        x in reduction_entry
    ]

    if !allequal(in_interface)
        throw(
            ArgumentError(
                "An interface is specified with only portion of a degree two chain reduction that has been reduced. Modify the data to include all segments of the reduced chain",
            ),
        )
    end
    return first(in_interface)
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::ServiceModel{X, W},
) where {
    T <: ActivePowerRangeExpressionLB,
    U <: VariableType,
    V <: PSY.Component,
    X <: PSY.Reserve{PSY.ReserveDown},
    W <: AbstractReservesFormulation,
}
    service_name = get_service_name(model)
    variable = get_variable(container, U(), X, service_name)
    if !has_container_key(container, T, V)
        add_expressions!(container, T, devices, model)
    end
    expression = get_expression(container, T(), V)
    for d in devices, t in get_time_steps(container)
        name = PSY.get_name(d)
        _add_to_jump_expression!(expression[name, t], variable[name, t], -1.0)
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::U,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {
    T <: Union{ActivePowerRangeExpressionUB, ActivePowerRangeExpressionLB},
    U <: OnStatusParameter,
    V <: PSY.Device,
    W <: AbstractDeviceFormulation,
}
    parameter_array = get_parameter_array(container, U(), V)
    if !has_container_key(container, T, V)
        add_expressions!(container, T, devices, model)
    end
    expression = get_expression(container, T(), V)
    for d in devices
        mult = get_expression_multiplier(U(), T(), d, W())
        for t in get_time_steps(container)
            name = PSY.get_name(d)
            _add_to_jump_expression!(
                expression[name, t],
                parameter_array[name, t],
                -mult,
                mult,
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::U,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {
    T <: Union{ActivePowerRangeExpressionUB, ActivePowerRangeExpressionLB},
    U <: OnStatusParameter,
    V <: PSY.ThermalGen,
    W <: AbstractThermalDispatchFormulation,
}
    parameter_array = get_parameter_array(container, U(), V)
    if !has_container_key(container, T, V)
        add_expressions!(container, T, devices, model)
    end
    expression = get_expression(container, T(), V)
    for d in devices
        if PSY.get_must_run(d)
            continue
        end
        mult = get_expression_multiplier(U(), T(), d, W())
        for t in get_time_steps(container)
            name = PSY.get_name(d)
            _add_to_jump_expression!(expression[name, t], parameter_array[name, t], -mult)
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{U},
    model::ServiceModel{V, W},
    devices_template::Dict{Symbol, DeviceModel},
) where {U <: VariableType, V <: PSY.Reserve, W <: AbstractReservesFormulation}
    contributing_devices_map = get_contributing_devices_map(model)
    for (device_type, devices) in contributing_devices_map
        device_model = get(devices_template, Symbol(device_type), nothing)
        device_model === nothing && continue
        expression_type = get_expression_type_for_reserve(U(), device_type, V)
        add_to_expression!(container, expression_type, U, devices, model)
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    ::PSY.System,
    network_model::NetworkModel{W},
) where {
    T <: ActivePowerBalance,
    U <: Union{SystemBalanceSlackUp, SystemBalanceSlackDown},
    W <: Union{CopperPlatePowerModel, PTDFPowerModel},
}
    variable = get_variable(container, U(), PSY.System)
    expression = get_expression(container, T(), _system_expression_type(W))
    reference_buses = get_reference_buses(network_model)
    for t in get_time_steps(container), n in reference_buses
        _add_to_jump_expression!(
            expression[n, t],
            variable[n, t],
            get_variable_multiplier(U(), PSY.System, W()),
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    sys::PSY.System,
    network_model::NetworkModel{V},
) where {
    T <: ActivePowerBalance,
    U <: Union{SystemBalanceSlackUp, SystemBalanceSlackDown},
    V <: AreaPTDFPowerModel,
}
    variable =
        get_variable(container, U(), _system_expression_type(AreaPTDFPowerModel))
    expression =
        get_expression(container, T(), _system_expression_type(AreaPTDFPowerModel))
    areas = get_available_components(network_model, PSY.Area, sys)
    for t in get_time_steps(container), n in PSY.get_name.(areas)
        _add_to_jump_expression!(
            expression[n, t],
            variable[n, t],
            get_variable_multiplier(U(), PSY.Area, AreaPTDFPowerModel()),
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    sys::PSY.System,
    ::NetworkModel{AreaBalancePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: Union{SystemBalanceSlackUp, SystemBalanceSlackDown},
}
    variable = get_variable(container, U(), PSY.Area)
    expression = get_expression(container, T(), PSY.Area)
    @assert_op length(axes(variable, 1)) == length(axes(expression, 1))
    for t in get_time_steps(container), n in axes(expression, 1)
        _add_to_jump_expression!(
            expression[n, t],
            variable[n, t],
            get_variable_multiplier(U(), PSY.Area, AreaBalancePowerModel),
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    sys::PSY.System,
    ::NetworkModel{W},
) where {
    T <: ActivePowerBalance,
    U <: Union{SystemBalanceSlackUp, SystemBalanceSlackDown},
    W <: PM.AbstractActivePowerModel,
}
    variable = get_variable(container, U(), PSY.ACBus)
    expression = get_expression(container, T(), PSY.ACBus)
    @assert_op length(axes(variable, 1)) == length(axes(expression, 1))
    # We uses axis here to avoid double addition of the slacks to the aggregated buses
    for t in get_time_steps(container), n in axes(expression, 1)
        _add_to_jump_expression!(
            expression[n, t],
            variable[n, t],
            get_variable_multiplier(U(), PSY.ACBus, W),
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    sys::PSY.System,
    ::NetworkModel{W},
) where {
    T <: ActivePowerBalance,
    U <: Union{SystemBalanceSlackUp, SystemBalanceSlackDown},
    W <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), PSY.ACBus, "P")
    expression = get_expression(container, T(), PSY.ACBus)
    # We uses axis here to avoid double addition of the slacks to the aggregated buses
    for t in get_time_steps(container), n in axes(expression, 1)
        _add_to_jump_expression!(
            expression[n, t],
            variable[n, t],
            get_variable_multiplier(U(), PSY.ACBus, W),
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    sys::PSY.System,
    ::NetworkModel{W},
) where {
    T <: ReactivePowerBalance,
    U <: Union{SystemBalanceSlackUp, SystemBalanceSlackDown},
    W <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), PSY.ACBus, "Q")
    expression = get_expression(container, T(), PSY.ACBus)
    # We uses axis here to avoid double addition of the slacks to the aggregated buses
    for t in get_time_steps(container), n in axes(expression, 1)
        _add_to_jump_expression!(
            expression[n, t],
            variable[n, t],
            get_variable_multiplier(U(), PSY.ACBus, W),
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{S},
    cost_expression::JuMPOrFloat,
    component::T,
    time_period::Int,
) where {S <: Union{CostExpressions, FuelConsumptionExpression}, T <: PSY.Component}
    if has_container_key(container, S, T)
        device_cost_expression = get_expression(container, S(), T)
        component_name = PSY.get_name(component)
        JuMP.add_to_expression!(
            device_cost_expression[component_name, time_period],
            cost_expression,
        )
    end
    return
end

"""
Specialized `add_to_expression!` for `ConstituentCostExpression` subtypes. In addition to
adding to the constituent expression, this method automatically propagates the cost to
`ProductionCostExpression`, so callers do not need to add to both.
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{S},
    cost_expression::JuMPOrFloat,
    component::T,
    time_period::Int,
) where {S <: ConstituentCostExpression, T <: PSY.Component}
    if has_container_key(container, S, T)
        device_cost_expression = get_expression(container, S(), T)
        component_name = PSY.get_name(component)
        JuMP.add_to_expression!(
            device_cost_expression[component_name, time_period],
            cost_expression,
        )
    end
    if has_container_key(container, ProductionCostExpression, T)
        prod_cost_expression = get_expression(container, ProductionCostExpression(), T)
        JuMP.add_to_expression!(
            prod_cost_expression[PSY.get_name(component), time_period],
            cost_expression,
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{S},
    cost_expression::JuMP.AbstractJuMPScalar,
    component::T,
    time_period::Int,
) where {S <: CostExpressions, T <: PSY.ReserveDemandCurve}
    if has_container_key(container, S, T, PSY.get_name(component))
        device_cost_expression = get_expression(container, S(), T, PSY.get_name(component))
        component_name = PSY.get_name(component)
        JuMP.add_to_expression!(
            device_cost_expression[component_name, time_period],
            cost_expression,
        )
    end
    return
end

# Disambiguate: ConstituentCostExpression ∩ ReserveDemandCurve — no ProductionCostExpression
# propagation since reserves don't have a ProductionCostExpression container.
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{S},
    cost_expression::JuMP.AbstractJuMPScalar,
    component::T,
    time_period::Int,
) where {S <: ConstituentCostExpression, T <: PSY.ReserveDemandCurve}
    if has_container_key(container, S, T, PSY.get_name(component))
        device_cost_expression = get_expression(container, S(), T, PSY.get_name(component))
        component_name = PSY.get_name(component)
        JuMP.add_to_expression!(
            device_cost_expression[component_name, time_period],
            cost_expression,
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {
    T <: FuelConsumptionExpression,
    U <: ActivePowerVariable,
    V <: PSY.ThermalGen,
    W <: AbstractDeviceFormulation,
}
    variable = get_variable(container, U(), V)
    time_steps = get_time_steps(container)
    base_power = get_base_power(container)
    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR
    for d in devices
        op_cost = PSY.get_operation_cost(d)
        var_cost = _get_variable_if_exists(op_cost)
        if !(var_cost isa PSY.FuelCurve)
            continue
        end
        expression = get_expression(container, T(), V)
        name = PSY.get_name(d)
        device_base_power = PSY.get_base_power(d)
        value_curve = PSY.get_value_curve(var_cost)
        if value_curve isa PSY.LinearCurve
            power_units = PSY.get_power_units(var_cost)
            proportional_term = PSY.get_proportional_term(value_curve)
            prop_term_per_unit = get_proportional_cost_per_system_unit(
                proportional_term,
                power_units,
                base_power,
                device_base_power,
            )
            for t in time_steps
                JuMP.add_to_expression!(
                    expression[name, t],
                    prop_term_per_unit * dt,
                    variable[name, t],
                )
            end
        elseif value_curve isa PSY.QuadraticCurve
            power_units = PSY.get_power_units(var_cost)
            proportional_term = PSY.get_proportional_term(value_curve)
            quadratic_term = PSY.get_quadratic_term(value_curve)
            prop_term_per_unit = get_proportional_cost_per_system_unit(
                proportional_term,
                power_units,
                base_power,
                device_base_power,
            )
            quad_term_per_unit = get_quadratic_cost_per_system_unit(
                quadratic_term,
                power_units,
                base_power,
                device_base_power,
            )
            for t in time_steps
                fuel_expr =
                    (
                        variable[name, t] .^ 2 * quad_term_per_unit +
                        variable[name, t] * prop_term_per_unit
                    ) * dt
                JuMP.add_to_expression!(
                    expression[name, t],
                    fuel_expr,
                )
            end
        end
    end
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {
    T <: FuelConsumptionExpression,
    U <: PowerAboveMinimumVariable,
    V <: PSY.ThermalGen,
    W <: AbstractDeviceFormulation,
}
    variable = get_variable(container, U(), V)
    time_steps = get_time_steps(container)
    base_power = get_base_power(container)
    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR
    for d in devices
        op_cost = PSY.get_operation_cost(d)
        var_cost = _get_variable_if_exists(op_cost)
        if !(var_cost isa PSY.FuelCurve)
            continue
        end
        expression = get_expression(container, T(), V)
        name = PSY.get_name(d)
        device_base_power = PSY.get_base_power(d)
        value_curve = PSY.get_value_curve(var_cost)
        P_min = PSY.get_active_power_limits(d).min
        if value_curve isa PSY.LinearCurve
            power_units = PSY.get_power_units(var_cost)
            proportional_term = PSY.get_proportional_term(value_curve)
            prop_term_per_unit = get_proportional_cost_per_system_unit(
                proportional_term,
                power_units,
                base_power,
                device_base_power,
            )
            for t in time_steps
                sos_status = _get_sos_value(container, W, d)
                if sos_status == SOSStatusVariable.NO_VARIABLE
                    JuMP.add_to_expression!(
                        expression[name, t],
                        P_min * prop_term_per_unit * dt,
                    )
                elseif sos_status == SOSStatusVariable.PARAMETER
                    param = get_default_on_parameter(d)
                    bin = get_parameter(container, param, V).parameter_array[name, t]
                    JuMP.add_to_expression!(
                        expression[name, t],
                        P_min * prop_term_per_unit * dt,
                        bin,
                    )
                elseif sos_status == SOSStatusVariable.VARIABLE
                    var = get_default_on_variable(d)
                    bin = get_variable(container, var, V)[name, t]
                    JuMP.add_to_expression!(
                        expression[name, t],
                        P_min * prop_term_per_unit * dt,
                        bin,
                    )
                else
                    @assert false
                end
                JuMP.add_to_expression!(
                    expression[name, t],
                    prop_term_per_unit * dt,
                    variable[name, t],
                )
            end
        elseif value_curve isa PSY.QuadraticCurve
            error("Quadratic Curves are not accepted with Compact Formulation: $W")
        end
    end
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::U,
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
) where {
    T <: NetActivePower,
    U <: Union{ActivePowerInVariable, ActivePowerOutVariable},
    V <: PSY.Source,
    W <: AbstractSourceFormulation,
}
    expression = get_expression(container, T(), V)
    variable = get_variable(container, U(), V)
    mult = get_variable_multiplier(U(), V, W())
    for d in devices
        name = PSY.get_name(d)
        for t in get_time_steps(container)
            JuMP.add_to_expression!(
                expression[name, t],
                variable[name, t] * mult,
            )
        end
    end
    return
end

##################################
##### Cost Expression Setup ######
##################################

"""
Adds all cost expression containers appropriate for the given device type and formulation
in a single pass through the device list.

The default method adds only `ProductionCostExpression`. Overloads for `PSY.ThermalGen`
and `PSY.RenewableGen` add their full set of constituent cost expressions, which then
automatically propagate into `ProductionCostExpression` via the `ConstituentCostExpression`
dispatch on `add_to_expression!`.
"""
function add_cost_expressions!(
    container::OptimizationContainer,
    devices::U,
    model::DeviceModel{D, W},
) where {D <: PSY.Component, U, W <: AbstractDeviceFormulation}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    add_expression_container!(container, ProductionCostExpression(), D, names, time_steps)
    return
end

function add_cost_expressions!(
    container::OptimizationContainer,
    devices::U,
    model::DeviceModel{D, W},
) where {D <: PSY.ThermalGen, U, W <: AbstractThermalFormulation}
    time_steps = get_time_steps(container)
    n_devices = length(devices)
    all_names = Vector{String}(undef, n_devices)
    fuel_names = sizehint!(String[], n_devices)
    has_quad_fuel = false
    for (i, d) in enumerate(devices)
        name = PSY.get_name(d)
        all_names[i] = name
        fuel_curve = _get_variable_if_exists(PSY.get_operation_cost(d))
        if fuel_curve isa PSY.FuelCurve
            push!(fuel_names, name)
            if !has_quad_fuel
                has_quad_fuel = PSY.get_value_curve(fuel_curve) isa PSY.QuadraticCurve
            end
        end
    end
    if !isempty(fuel_names)
        fuel_expr_type = has_quad_fuel ? JuMP.QuadExpr : GAE
        add_expression_container!(
            container, FuelConsumptionExpression(), D, fuel_names, time_steps;
            expr_type = fuel_expr_type,
        )
    end
    add_expression_container!(
        container,
        ProductionCostExpression(),
        D,
        all_names,
        time_steps,
    )
    add_expression_container!(container, FuelCostExpression(), D, all_names, time_steps)
    add_expression_container!(container, StartUpCostExpression(), D, all_names, time_steps)
    add_expression_container!(container, ShutDownCostExpression(), D, all_names, time_steps)
    add_expression_container!(container, FixedCostExpression(), D, all_names, time_steps)
    add_expression_container!(container, VOMCostExpression(), D, all_names, time_steps)
    return
end

function add_cost_expressions!(
    container::OptimizationContainer,
    devices::U,
    model::DeviceModel{D, W},
) where {D <: PSY.RenewableGen, U, W <: AbstractRenewableDispatchFormulation}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    add_expression_container!(container, ProductionCostExpression(), D, names, time_steps)
    add_expression_container!(container, FixedCostExpression(), D, names, time_steps)
    add_expression_container!(container, CurtailmentCostExpression(), D, names, time_steps)
    add_expression_container!(container, VOMCostExpression(), D, names, time_steps)
    return
end

#=
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    areas::IS.FlattenIteratorWrapper{V},
    model::ServiceModel{PSY.AGC, W},
) where {
    T <: Union{EmergencyUp, EmergencyDown},
    U <:
    Union{AdditionalDeltaActivePowerUpVariable, AdditionalDeltaActivePowerDownVariable},
    V <: PSY.Area,
    W <: AbstractServiceFormulation,
}
    names = PSY.get_name.(areas)
    time_steps = get_time_steps(container)
    if !has_container_key(container, T, V)
        expression = add_expression_container!(container, T(), V, names, time_steps)
    end
    expression = get_expression(container, T(), V)
    variable = get_variable(container, U(), V)
    for n in names, t in time_steps
        _add_to_jump_expression!(expression[n, t], variable[n, t], 1.0)
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    services::IS.FlattenIteratorWrapper{V},
    model::ServiceModel{V, W},
) where {
    T <: RawACE,
    U <: SteadyStateFrequencyDeviation,
    V <: PSY.AGC,
    W <: AbstractServiceFormulation,
}
    names = PSY.get_name.(services)
    time_steps = get_time_steps(container)
    if !has_container_key(container, T, V)
        expression = add_expression_container!(container, T(), PSY.AGC, names, time_steps)
    end
    expression = get_expression(container, T(), PSY.AGC)
    variable = get_variable(container, U(), PSY.AGC)
    for s in services, t in time_steps
        name = PSY.get_name(s)
        _add_to_jump_expression!(
            expression[name, t],
            variable[t],
            get_variable_multiplier(U(), s, W()),
        )
    end
    return
end
=#
