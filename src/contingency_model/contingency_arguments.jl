function add_event_arguments!(
    container::OptimizationContainer,
    devices::T,
    device_model::DeviceModel{U, V},
    network_model::NetworkModel,
) where {
    T <: Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    V <: AbstractDeviceFormulation,
} where {U <: PSY.StaticInjection}
    for (key, event_model) in get_events(device_model)
        event_type = get_entry_type(key)
        devices_with_attributes =
            [d for d in devices if PSY.has_supplemental_attributes(d, event_type)]
        isempty(devices_with_attributes) &&
            error("no devices found with a supplemental attribute for event $event_type")
        for p_type in [AvailableStatusChangeCountdownParameter, AvailableStatusParameter]
            add_parameters!(
                container,
                p_type,
                devices_with_attributes,
                device_model,
                event_model,
            )
        end
    end

    return
end

function add_event_arguments!(
    container::OptimizationContainer,
    devices::T,
    device_model::DeviceModel{U, V},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    V <: Union{StaticPowerLoad, PowerLoadDispatch, PowerLoadInterruption},
} where {U <: PSY.PowerLoad}
    for (key, event_model) in get_events(device_model)
        event_type = get_entry_type(key)
        devices_with_attributes =
            [d for d in devices if PSY.has_supplemental_attributes(d, event_type)]
        isempty(devices_with_attributes) &&
            error("no devices found with a supplemental attribute for event $event_type")
        for p_type in [AvailableStatusChangeCountdownParameter, AvailableStatusParameter]
            add_parameters!(
                container,
                p_type,
                devices_with_attributes,
                device_model,
                event_model,
            )
        end
        add_parameters!(
            container,
            ActivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            event_model,
        )
        add_to_expression!(
            container,
            ActivePowerBalance,
            ActivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            network_model,
        )
    end

    return
end

function add_event_arguments!(
    container::OptimizationContainer,
    devices::T,
    device_model::DeviceModel{U, V},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    V <: Union{StaticPowerLoad, PowerLoadDispatch, PowerLoadInterruption},
} where {U <: PSY.PowerLoad}
    for (key, event_model) in get_events(device_model)
        event_type = get_entry_type(key)
        devices_with_attributes =
            [d for d in devices if PSY.has_supplemental_attributes(d, event_type)]
        isempty(devices_with_attributes) &&
            error("no devices found with a supplemental attribute for event $event_type")
        for p_type in [AvailableStatusChangeCountdownParameter, AvailableStatusParameter]
            add_parameters!(
                container,
                p_type,
                devices_with_attributes,
                device_model,
                event_model,
            )
        end
        add_parameters!(
            container,
            ActivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            event_model,
        )
        add_to_expression!(
            container,
            ActivePowerBalance,
            ActivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            network_model,
        )
        add_parameters!(
            container,
            ReactivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            event_model,
        )
        add_to_expression!(
            container,
            ReactivePowerBalance,
            ReactivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            network_model,
        )
    end

    return
end

function add_event_arguments!(
    container::OptimizationContainer,
    devices::T,
    device_model::DeviceModel{U, FixedOutput},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
} where {U <: PSY.StaticInjection}
    for (key, event_model) in get_events(device_model)
        event_type = get_entry_type(key)
        devices_with_attributes =
            [d for d in devices if PSY.has_supplemental_attributes(d, event_type)]
        isempty(devices_with_attributes) &&
            error("no devices found with a supplemental attribute for event $event_type")
        for p_type in [AvailableStatusChangeCountdownParameter, AvailableStatusParameter]
            add_parameters!(
                container,
                p_type,
                devices_with_attributes,
                device_model,
                event_model,
            )
        end
        add_parameters!(
            container,
            ActivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            event_model,
        )
        add_to_expression!(
            container,
            ActivePowerBalance,
            ActivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            network_model,
        )
    end
    return
end

function add_event_arguments!(
    container::OptimizationContainer,
    devices::T,
    device_model::DeviceModel{U, FixedOutput},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
} where {U <: PSY.StaticInjection}
    for (key, event_model) in get_events(device_model)
        event_type = get_entry_type(key)
        devices_with_attributes =
            [d for d in devices if PSY.has_supplemental_attributes(d, event_type)]
        isempty(devices_with_attributes) &&
            error("no devices found with a supplemental attribute for event $event_type")
        for p_type in [AvailableStatusChangeCountdownParameter, AvailableStatusParameter]
            add_parameters!(
                container,
                p_type,
                devices_with_attributes,
                device_model,
                event_model,
            )
        end
        add_parameters!(
            container,
            ActivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            event_model,
        )
        add_to_expression!(
            container,
            ActivePowerBalance,
            ActivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            network_model,
        )
        add_parameters!(
            container,
            ReactivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            event_model,
        )
        add_to_expression!(
            container,
            ReactivePowerBalance,
            ReactivePowerOffsetParameter,
            devices_with_attributes,
            device_model,
            network_model,
        )
    end
    return
end

function _add_parameters!(
    container,
    ::T,
    devices::Vector{U},
    device_model::DeviceModel{U, W},
    event_model::EventModel{V, X},
) where {
    T <: EventParameter,
    U <: PSY.Component,
    V <: PSY.Contingency,
    W <: AbstractDeviceFormulation,
    X <: AbstractEventCondition,
}
    @debug "adding" T U V _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    time_steps = get_time_steps(container)
    parameter_container = add_param_container!(
        container,
        T(),
        U,
        V,
        PSY.get_name.(devices),
        time_steps,
    )

    jump_model = get_jump_model(container)
    multiplier_array = get_multiplier_array(parameter_container)
    parent_param = get_parameter_array_data(parameter_container)

    for (i, d) in enumerate(devices)
        ini_val = get_initial_parameter_value(T(), d, event_model)
        name = PSY.get_name(d)
        assign_maybe_broadcast!(
            multiplier_array,
            get_parameter_multiplier(T(), d, event_model),
            (name,),
        )
        for t in time_steps
            _set_parameter_at!(parent_param, jump_model, ini_val, i, t)
        end
    end
    return
end

"""
Default implementation to add parameters to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: EventParameter,
    V <: PSY.StaticInjection,
    W <: AbstractDeviceFormulation,
    X <: CopperPlatePowerModel,
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
                get_parameter_array(param_container)[name, t],
                multiplier[name, t],
            )
        end
    end
    return
end

"""
Default implementation to add parameters to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    device_model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: EventParameter,
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
        bus_no_ = PSY.get_number(device_bus)
        bus_no = PNM.get_mapped_bus_number(network_reduction, bus_no_)
        ref_index = _ref_index(network_model, device_bus)
        for t in get_time_steps(container)
            param = get_parameter_array(param_container)[name, t]
            _add_to_jump_expression!(sys_expr[ref_index, t], param, multiplier[name, t])
            _add_to_jump_expression!(nodal_expr[bus_no, t], param, multiplier[name, t])
        end
    end
    return
end

"""
Default implementation to add parameters to SystemBalanceExpressions
"""
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
    network_model::NetworkModel{X},
) where {
    T <: SystemBalanceExpressions,
    U <: EventParameter,
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
            get_parameter_array(param_container)[name, t],
            multiplier[name, t],
        )
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    model::DeviceModel{V, W},
    network_model::NetworkModel{AreaBalancePowerModel},
) where {
    T <: SystemBalanceExpressions,
    U <: EventParameter,
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
            get_parameter_array(param_container)[name, t],
            multiplier[name, t],
        )
    end
    return
end
