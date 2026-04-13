#! format: off
get_variable_binary(::ActivePowerVariable, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
get_variable_warm_start_value(::ActivePowerVariable, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_active_power(d)
get_variable_lower_bound(::ActivePowerVariable, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_active_power_limits(d).min
get_variable_upper_bound(::ActivePowerVariable, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_active_power_limits(d).max
get_variable_multiplier(_, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = 1.0


function _get_flow_bounds(d::PSY.TModelHVDCLine)
    check_hvdc_line_limits_consistency(d)
    from_min = PSY.get_active_power_limits_from(d).min
    to_min = PSY.get_active_power_limits_to(d).min
    from_max = PSY.get_active_power_limits_from(d).max
    to_max = PSY.get_active_power_limits_to(d).max

    if from_min >= 0.0 && to_min >= 0.0
        min_rate = min(from_min, to_min)
    elseif from_min <= 0.0 && to_min <= 0.0
        min_rate = max(from_min, to_min)
    elseif from_min <= 0.0 && to_min >= 0.0
        min_rate = from_min
    elseif to_min <= 0.0 && from_min >= 0.0
        min_rate = to_min
    else
        @assert false
    end

    if from_max >= 0.0 && to_max >= 0.0
        max_rate = min(from_max, to_max)
    elseif from_max <= 0.0 && to_max <= 0.0
        max_rate = max(from_max, to_max)
    elseif from_max <= 0.0 && to_max >= 0.0
        max_rate = from_max
    elseif from_max >= 0.0 && to_max <= 0.0
        max_rate = to_max
    else
        @assert false
    end

    return min_rate, max_rate
end


get_variable_binary(::FlowActivePowerVariable, ::Type{PSY.TModelHVDCLine}, ::AbstractBranchFormulation) = false
get_variable_binary(::DCLineCurrent, ::Type{PSY.TModelHVDCLine}, ::AbstractBranchFormulation) = false
get_variable_warm_start_value(::FlowActivePowerVariable, d::PSY.TModelHVDCLine, ::AbstractBranchFormulation) = PSY.get_active_power_flow(d)
get_variable_lower_bound(::FlowActivePowerVariable, d::PSY.TModelHVDCLine, ::AbstractBranchFormulation) = _get_flow_bounds(d)[1]
get_variable_upper_bound(::FlowActivePowerVariable, d::PSY.TModelHVDCLine, ::AbstractBranchFormulation) = _get_flow_bounds(d)[2]

# This is an approximation for DC lines since the actual current limit depends on the voltage, that is a variable in the optimization problem
function get_variable_lower_bound(::DCLineCurrent, d::PSY.TModelHVDCLine, ::AbstractBranchFormulation)
    p_min_flow = _get_flow_bounds(d)[1]
    arc = PSY.get_arc(d)
    bus_from = arc.from
    bus_to = arc.to
    max_v = max(PSY.get_magnitude(bus_from), PSY.get_magnitude(bus_to))
    return p_min_flow / max_v
end
# This is an approximation for DC lines since the actual current limit depends on the voltage, that is a variable in the optimization problem
function get_variable_upper_bound(::DCLineCurrent, d::PSY.TModelHVDCLine, ::AbstractBranchFormulation)
    p_max_flow = _get_flow_bounds(d)[2]
    arc = PSY.get_arc(d)
    bus_from = arc.from
    bus_to = arc.to
    max_v = max(PSY.get_magnitude(bus_from), PSY.get_magnitude(bus_to))
    return p_max_flow / max_v
end
get_variable_multiplier(_, ::Type{PSY.TModelHVDCLine}, ::AbstractBranchFormulation) = 1.0

requires_initialization(::AbstractConverterFormulation) = false
requires_initialization(::LosslessLine) = false

function get_initial_conditions_device_model(
    ::OperationModel,
    model::DeviceModel{PSY.InterconnectingConverter, <:AbstractConverterFormulation},
)
    return model
end

function get_initial_conditions_device_model(
    ::OperationModel,
    model::DeviceModel{PSY.TModelHVDCLine, D},
) where {D <: AbstractDCLineFormulation}
    return model
end


function get_default_time_series_names(
    ::Type{PSY.InterconnectingConverter},
    ::Type{<:AbstractConverterFormulation},
)
    return Dict{Type{<:TimeSeriesParameter}, String}()
end

function get_default_time_series_names(
    ::Type{PSY.TModelHVDCLine},
    ::Type{<:AbstractBranchFormulation},
)
    return Dict{Type{<:TimeSeriesParameter}, String}()
end

function get_default_attributes(
    ::Type{PSY.InterconnectingConverter},
    ::Type{<:AbstractConverterFormulation},
)
    return Dict{String, Any}()
end

function get_default_attributes(
    ::Type{PSY.TModelHVDCLine},
    ::Type{<:AbstractBranchFormulation},
)
    return Dict{String, Any}()
end


############################################
######## Quadratic Converter Model #########
############################################

## Binaries ###
get_variable_binary(::ConverterDCPower, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
get_variable_binary(::ConverterPowerDirection, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = true
get_variable_binary(::ConverterCurrent, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
get_variable_binary(::ConverterPositiveCurrent, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
get_variable_binary(::ConverterNegativeCurrent, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
get_variable_binary(::ConverterCurrentDirection, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = true
get_variable_binary(::SquaredConverterCurrent, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
get_variable_binary(::SquaredDCVoltage, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
get_variable_binary(::AuxBilinearConverterVariable, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
get_variable_binary(::AuxBilinearSquaredConverterVariable, ::Type{PSY.InterconnectingConverter}, ::AbstractConverterFormulation) = false
function get_variable_binary(
    ::W,
    ::Type{PSY.InterconnectingConverter},
    ::AbstractConverterFormulation
) where W <: InterpolationVariableType
    return false
end
function get_variable_binary(
    ::W,
    ::Type{PSY.InterconnectingConverter},
    ::AbstractConverterFormulation
) where W <: BinaryInterpolationVariableType
    return true
end


### Warm Start ###
get_variable_warm_start_value(::ConverterCurrent, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_dc_current(d)

### Lower Bounds ###
get_variable_lower_bound(::ConverterDCPower, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_active_power_limits(d).min
get_variable_lower_bound(::ConverterCurrent, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = -PSY.get_max_dc_current(d)
get_variable_lower_bound(::SquaredConverterCurrent, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = 0.0
get_variable_lower_bound(::SquaredDCVoltage, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_voltage_limits(d.dc_bus).min^2
get_variable_lower_bound(::InterpolationVariableType, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = 0.0
get_variable_lower_bound(::ConverterPositiveCurrent, d::PSY.InterconnectingConverter,::AbstractConverterFormulation) = 0.0
get_variable_lower_bound(::ConverterNegativeCurrent, d::PSY.InterconnectingConverter,::AbstractConverterFormulation) = 0.0

### Upper Bounds ###
get_variable_upper_bound(::ConverterDCPower, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_active_power_limits(d).max
get_variable_upper_bound(::ConverterCurrent, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_max_dc_current(d)
get_variable_upper_bound(::SquaredConverterCurrent, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_max_dc_current(d)^2
get_variable_upper_bound(::SquaredDCVoltage, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = PSY.get_voltage_limits(d.dc_bus).max^2
get_variable_upper_bound(::InterpolationVariableType, d::PSY.InterconnectingConverter, ::AbstractConverterFormulation) = 1.0
get_variable_upper_bound(::ConverterPositiveCurrent, d::PSY.InterconnectingConverter,::AbstractConverterFormulation) = PSY.get_max_dc_current(d)
get_variable_upper_bound(::ConverterNegativeCurrent, d::PSY.InterconnectingConverter,::AbstractConverterFormulation) = PSY.get_max_dc_current(d)


function get_default_attributes(
    ::Type{PSY.InterconnectingConverter},
    ::Type{QuadraticLossConverter},
)
    return Dict{String, Any}(
        "voltage_segments" => 3,
        "current_segments" => 6,
        "bilinear_segments" => 10,
        "use_linear_loss" => true,
    )
end

#! format: on

############################################
############## Expressions #################
############################################

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: Union{ActivePowerBalance, DCCurrentBalance},
    U <: Union{FlowActivePowerVariable, DCLineCurrent},
    V <: PSY.TModelHVDCLine,
    W <: AbstractDCLineFormulation,
    X <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), V)
    expression = get_expression(container, T(), PSY.DCBus)
    for d in devices
        arc = PSY.get_arc(d)
        to_bus_number = PSY.get_number(PSY.get_to(arc))
        from_bus_number = PSY.get_number(PSY.get_from(arc))
        for t in get_time_steps(container)
            name = PSY.get_name(d)
            _add_to_jump_expression!(
                expression[to_bus_number, t],
                variable[name, t],
                1.0,
            )
            _add_to_jump_expression!(
                expression[from_bus_number, t],
                variable[name, t],
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
    V <: PSY.InterconnectingConverter,
    W <: AbstractConverterFormulation,
    X <: AreaPTDFPowerModel,
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
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.InterconnectingConverter,
    W <: AbstractConverterFormulation,
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
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.InterconnectingConverter,
    W <: AbstractConverterFormulation,
    X <: PM.AbstractPowerModel,
}
    variable = get_variable(container, U(), V)
    expression_dc = get_expression(container, T(), PSY.DCBus)
    expression_ac = get_expression(container, T(), PSY.ACBus)
    for d in devices, t in get_time_steps(container)
        name = PSY.get_name(d)
        bus_number_dc = PSY.get_number(PSY.get_dc_bus(d))
        bus_number_ac = PSY.get_number(PSY.get_bus(d))
        _add_to_jump_expression!(
            expression_ac[bus_number_ac, t],
            variable[name, t],
            1.0,
        )
        _add_to_jump_expression!(
            expression_dc[bus_number_dc, t],
            variable[name, t],
            -1.0,
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{AreaPTDFPowerModel},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.InterconnectingConverter,
    W <: AbstractConverterFormulation,
}
    error("AreaPTDFPowerModel doesn't support InterconnectingConverter")
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{PTDFPowerModel},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.InterconnectingConverter,
    W <: AbstractConverterFormulation,
}
    variable = get_variable(container, U(), V)
    expression_dc = get_expression(container, T(), PSY.DCBus)
    expression_ac = get_expression(container, T(), PSY.ACBus)
    sys_expr = get_expression(container, T(), PSY.System)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        bus_number_ac = PSY.get_number(device_bus)
        ref_bus = get_reference_bus(network_model, device_bus)
        bus_number_dc = PSY.get_number(PSY.get_dc_bus(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                sys_expr[ref_bus, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
            _add_to_jump_expression!(
                expression_ac[bus_number_ac, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
            _add_to_jump_expression!(
                expression_dc[bus_number_dc, t],
                variable[name, t],
                -1.0,
            )
        end
    end
    return
end

function add_to_expression!(
    ::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    ::DeviceModel{V, W},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.InterconnectingConverter,
    W <: AbstractConverterFormulation,
}
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    model::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.InterconnectingConverter,
    W <: AbstractConverterFormulation,
}
    variable = get_variable(container, U(), V)
    expression_dc = get_expression(container, T(), PSY.DCBus)
    sys_expr = get_expression(container, T(), PSY.System)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        ref_bus = get_reference_bus(network_model, device_bus)
        bus_number_dc = PSY.get_number(PSY.get_dc_bus(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                sys_expr[ref_bus, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
            _add_to_jump_expression!(
                expression_dc[bus_number_dc, t],
                variable[name, t],
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
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.InterconnectingConverter,
    W <: QuadraticLossConverter,
}
    variable = get_variable(container, U(), V)
    sys_expr = get_expression(container, T(), PSY.System)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        ref_bus = get_reference_bus(network_model, device_bus)
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                sys_expr[ref_bus, t],
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
    model::DeviceModel{V, W},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: DCCurrentBalance,
    U <: ConverterCurrent,
    V <: PSY.InterconnectingConverter,
    W <: QuadraticLossConverter,
}
    variable = get_variable(container, U(), V)
    expression_dc = get_expression(container, T(), PSY.DCBus)
    for d in devices
        name = PSY.get_name(d)
        bus_number_dc = PSY.get_number(PSY.get_dc_bus(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                expression_dc[bus_number_dc, t],
                variable[name, t],
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
    network_model::NetworkModel{X},
) where {
    T <: ActivePowerBalance,
    U <: ActivePowerVariable,
    V <: PSY.InterconnectingConverter,
    W <: AbstractConverterFormulation,
    X <: PTDFPowerModel,
}
    variable = get_variable(container, U(), V)
    expression_dc = get_expression(container, T(), PSY.DCBus)
    expression_ac = get_expression(container, T(), PSY.ACBus)
    sys_expr = get_expression(container, T(), PSY.System)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        bus_number_ac = PSY.get_number(device_bus)
        ref_bus = get_reference_bus(network_model, device_bus)
        bus_number_dc = PSY.get_number(PSY.get_dc_bus(d))
        for t in get_time_steps(container)
            _add_to_jump_expression!(
                sys_expr[ref_bus, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
            _add_to_jump_expression!(
                expression_ac[bus_number_ac, t],
                variable[name, t],
                get_variable_multiplier(U(), V, W()),
            )
            _add_to_jump_expression!(
                expression_dc[bus_number_dc, t],
                variable[name, t],
                -1.0,
            )
        end
    end

    return
end

############################################
############## Constraints #################
############################################

############## HVDC Lines ##################
function add_constraints!(
    container::OptimizationContainer,
    ::Type{DCLineCurrentConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, U},
    network_model::NetworkModel{V},
) where {T <: PSY.TModelHVDCLine, U <: DCLossyLine, V <: PM.AbstractPowerModel}
    variable = get_variable(container, DCLineCurrent(), T)
    dc_voltage = get_variable(container, DCVoltage(), PSY.DCBus)
    time_steps = get_time_steps(container)
    constraints = add_constraints_container!(
        container,
        DCLineCurrentConstraint(),
        T,
        PSY.get_name.(devices),
        time_steps,
    )

    for d in devices
        arc = PSY.get_arc(d)
        from_bus_name = PSY.get_name(arc.from)
        to_bus_name = PSY.get_name(arc.to)
        name = PSY.get_name(d)
        r = PSY.get_r(d)
        if iszero(r)
            for t in time_steps
                constraints[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    dc_voltage[from_bus_name, t] == dc_voltage[to_bus_name, t]
                )
            end
        else
            for t in get_time_steps(container)
                constraints[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    variable[name, t] ==
                    (dc_voltage[from_bus_name, t] - dc_voltage[to_bus_name, t]) / r
                )
            end
        end
    end
    return
end

############## Converters ##################
function add_constraints!(
    container::OptimizationContainer,
    ::Type{ConverterPowerCalculationConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    network_model::NetworkModel{X},
) where {
    U <: PSY.InterconnectingConverter,
    V <: QuadraticLossConverter,
    X <: PM.AbstractActivePowerModel,
}
    time_steps = get_time_steps(container)
    varcurrent = get_variable(container, ConverterCurrent(), U)
    var_dcvoltage = get_variable(container, DCVoltage(), PSY.DCBus)
    var_sq_current = get_variable(container, SquaredConverterCurrent(), U)
    var_sq_voltage = get_variable(container, SquaredDCVoltage(), U)
    var_bilinear = get_variable(container, AuxBilinearConverterVariable(), U)
    var_sq_bilinear = get_variable(container, AuxBilinearSquaredConverterVariable(), U)
    var_dc_power = get_variable(container, ConverterDCPower(), U)
    ipc_names = axes(varcurrent, 1)
    constraint =
        add_constraints_container!(
            container,
            ConverterPowerCalculationConstraint(),
            U,
            ipc_names,
            time_steps,
        )
    constraint_aux =
        add_constraints_container!(
            container,
            ConverterPowerCalculationConstraint(),
            U,
            ipc_names,
            time_steps;
            meta = "aux",
        )

    for device in devices
        name = PSY.get_name(device)
        dc_bus_name = PSY.get_name(PSY.get_dc_bus(device))
        for t in time_steps
            # p_dc = v_dc * i_dc = 0.5 * (bilinear - v_dc^2 - i_dc^2)
            constraint[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var_dc_power[name, t] ==
                0.5 * (
                    var_sq_bilinear[name, t] - var_sq_voltage[name, t] -
                    var_sq_current[name, t]
                )
            )
            constraint_aux[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var_bilinear[name, t] ==
                var_dcvoltage[dc_bus_name, t] + varcurrent[name, t]
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{ConverterMcCormickEnvelopes},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    network_model::NetworkModel{X},
) where {
    U <: PSY.InterconnectingConverter,
    V <: QuadraticLossConverter,
    X <: PM.AbstractActivePowerModel,
}
    time_steps = get_time_steps(container)
    varcurrent = get_variable(container, ConverterCurrent(), U)
    var_dcvoltage = get_variable(container, DCVoltage(), PSY.DCBus)
    var_dc_power = get_variable(container, ConverterDCPower(), U)
    ipc_names = axes(varcurrent, 1)
    constraint1_under =
        add_constraints_container!(
            container,
            ConverterMcCormickEnvelopes(),
            U,
            ipc_names,
            time_steps;
            meta = "under_1",
        )
    constraint2_under =
        add_constraints_container!(
            container,
            ConverterMcCormickEnvelopes(),
            U,
            ipc_names,
            time_steps;
            meta = "under_2",
        )
    constraint1_over =
        add_constraints_container!(
            container,
            ConverterMcCormickEnvelopes(),
            U,
            ipc_names,
            time_steps;
            meta = "over_1",
        )
    constraint2_over =
        add_constraints_container!(
            container,
            ConverterMcCormickEnvelopes(),
            U,
            ipc_names,
            time_steps;
            meta = "over_2",
        )

    for device in devices
        name = PSY.get_name(device)
        dc_bus = PSY.get_dc_bus(device)
        dc_bus_name = PSY.get_name(dc_bus)
        V_min, V_max = PSY.get_voltage_limits(dc_bus)
        I_max = PSY.get_max_dc_current(device)
        I_min = -I_max
        for t in time_steps
            constraint1_under[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var_dc_power[name, t] >=
                V_min * varcurrent[name, t] + var_dcvoltage[dc_bus_name, t] * I_min -
                I_min * V_min
            )
            constraint2_under[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var_dc_power[name, t] >=
                V_max * varcurrent[name, t] + var_dcvoltage[dc_bus_name, t] * I_max -
                I_max * V_max
            )
            constraint1_over[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var_dc_power[name, t] <=
                V_max * varcurrent[name, t] + var_dcvoltage[dc_bus_name, t] * I_min -
                I_min * V_max
            )
            constraint2_over[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var_dc_power[name, t] <=
                V_min * varcurrent[name, t] + var_dcvoltage[dc_bus_name, t] * I_max -
                I_max * V_min
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{ConverterLossConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    network_model::NetworkModel{X},
) where {
    U <: PSY.InterconnectingConverter,
    V <: QuadraticLossConverter,
    X <: PM.AbstractActivePowerModel,
}
    time_steps = get_time_steps(container)
    var_sq_current = get_variable(container, SquaredConverterCurrent(), U)
    var_ac_power = get_variable(container, ActivePowerVariable(), U)
    var_dc_power = get_variable(container, ConverterDCPower(), U)
    ipc_names = axes(var_sq_current, 1)
    constraint =
        add_constraints_container!(
            container,
            ConverterLossConstraint(),
            U,
            ipc_names,
            time_steps,
        )

    use_linear_loss = PSI.get_attribute(model, "use_linear_loss")
    if use_linear_loss
        pos_current = get_variable(container, ConverterPositiveCurrent(), U)
        neg_current = get_variable(container, ConverterNegativeCurrent(), U)
    end

    for device in devices
        name = PSY.get_name(device)
        loss_function = PSY.get_loss_function(device)
        if isa(loss_function, PSY.QuadraticCurve)
            a = PSY.get_quadratic_term(loss_function)
            b = PSY.get_proportional_term(loss_function)
            c = PSY.get_constant_term(loss_function)
        else
            a = 0.0
            b = PSY.get_proportional_term(loss_function)
            c = PSY.get_constant_term(loss_function)
        end
        for t in time_steps
            if use_linear_loss
                loss =
                    a * var_sq_current[name, t] +
                    b * (pos_current[name, t] + neg_current[name, t]) + c
            else
                loss = a * var_sq_current[name, t] + c
            end
            constraint[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var_ac_power[name, t] == var_dc_power[name, t] - loss
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    ::DeviceModel{U, V},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: CurrentAbsoluteValueConstraint,
    U <: PSY.InterconnectingConverter,
    V <: QuadraticLossConverter,
}
    time_steps = get_time_steps(container)
    names = [PSY.get_name(d) for d in devices]
    JuMPmodel = get_jump_model(container)
    # current vars #
    current_var = get_variable(container, ConverterCurrent(), U) # From direction
    current_var_pos = get_variable(container, ConverterPositiveCurrent(), U) # From direction
    current_var_neg = get_variable(container, ConverterNegativeCurrent(), U) # From direction
    current_dir = get_variable(container, ConverterCurrentDirection(), U)

    constraint =
        add_constraints_container!(
            container,
            CurrentAbsoluteValueConstraint(),
            U,
            names,
            time_steps,
        )
    constraint_pos_ub =
        add_constraints_container!(
            container,
            CurrentAbsoluteValueConstraint(),
            U,
            names,
            time_steps;
            meta = "pos_ub",
        )
    constraint_neg_ub =
        add_constraints_container!(
            container,
            CurrentAbsoluteValueConstraint(),
            U,
            names,
            time_steps;
            meta = "neg_ub",
        )

    for d in devices
        name = PSY.get_name(d)
        I_max = PSY.get_max_dc_current(d)
        for t in time_steps
            constraint[name, t] = JuMP.@constraint(
                JuMPmodel,
                current_var[name, t] == current_var_pos[name, t] - current_var_neg[name, t]
            )
            constraint_pos_ub[name, t] = JuMP.@constraint(
                JuMPmodel,
                current_var_pos[name, t] <= I_max * current_dir[name, t]
            )
            constraint_neg_ub[name, t] = JuMP.@constraint(
                JuMPmodel,
                current_var_neg[name, t] <= I_max * (1 - current_dir[name, t])
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: InterpolationVoltageConstraints,
    U <: PSY.InterconnectingConverter,
    V <: QuadraticLossConverter,
}
    dic_var_bkpts = Dict{String, Vector{Float64}}()
    dic_function_bkpts = Dict{String, Vector{Float64}}()
    num_segments = get_attribute(model, "voltage_segments")
    for d in devices
        name = PSY.get_name(d)
        vmin, vmax = PSY.get_voltage_limits(d.dc_bus)
        var_bkpts, function_bkpts =
            _get_breakpoints_for_pwl_function(vmin, vmax, x -> x^2; num_segments)
        dic_var_bkpts[name] = var_bkpts
        dic_function_bkpts[name] = function_bkpts
    end

    _add_generic_incremental_interpolation_constraint!(
        container,
        DCVoltage(),
        SquaredDCVoltage(),
        InterpolationSquaredVoltageVariable(),
        InterpolationBinarySquaredVoltageVariable(),
        InterpolationVoltageConstraints(),
        devices,
        dic_var_bkpts,
        dic_function_bkpts,
    )
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: InterpolationCurrentConstraints,
    U <: PSY.InterconnectingConverter,
    V <: QuadraticLossConverter,
}
    dic_var_bkpts = Dict{String, Vector{Float64}}()
    dic_function_bkpts = Dict{String, Vector{Float64}}()
    num_segments = get_attribute(model, "current_segments")
    for d in devices
        name = PSY.get_name(d)
        Imax = PSY.get_max_dc_current(d)
        Imin = -Imax
        var_bkpts, function_bkpts =
            _get_breakpoints_for_pwl_function(Imin, Imax, x -> x^2; num_segments)
        dic_var_bkpts[name] = var_bkpts
        dic_function_bkpts[name] = function_bkpts
    end

    _add_generic_incremental_interpolation_constraint!(
        container,
        ConverterCurrent(),
        SquaredConverterCurrent(),
        InterpolationSquaredCurrentVariable(),
        InterpolationBinarySquaredCurrentVariable(),
        InterpolationCurrentConstraints(),
        devices,
        dic_var_bkpts,
        dic_function_bkpts,
    )
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: InterpolationBilinearConstraints,
    U <: PSY.InterconnectingConverter,
    V <: QuadraticLossConverter,
}
    dic_var_bkpts = Dict{String, Vector{Float64}}()
    dic_function_bkpts = Dict{String, Vector{Float64}}()
    num_segments = get_attribute(model, "bilinear_segments")
    for d in devices
        name = PSY.get_name(d)
        vmin, vmax = PSY.get_voltage_limits(d.dc_bus)
        Imax = PSY.get_max_dc_current(d)
        Imin = -Imax
        γ_min = vmin * Imin
        γ_max = vmax * Imax
        var_bkpts, function_bkpts =
            _get_breakpoints_for_pwl_function(γ_min, γ_max, x -> x^2; num_segments)
        dic_var_bkpts[name] = var_bkpts
        dic_function_bkpts[name] = function_bkpts
    end

    _add_generic_incremental_interpolation_constraint!(
        container,
        AuxBilinearConverterVariable(),
        AuxBilinearSquaredConverterVariable(),
        InterpolationSquaredBilinearVariable(),
        InterpolationBinarySquaredBilinearVariable(),
        InterpolationBilinearConstraints(),
        devices,
        dic_var_bkpts,
        dic_function_bkpts,
    )
    return
end

############################################
########### Objective Function #############
############################################

function objective_function!(
    ::OptimizationContainer,
    ::IS.FlattenIteratorWrapper{PSY.InterconnectingConverter},
    ::DeviceModel{PSY.InterconnectingConverter, D},
    ::Type{<:PM.AbstractPowerModel},
) where {D <: AbstractConverterFormulation}
    return
end
