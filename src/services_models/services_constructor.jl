function get_incompatible_devices(devices_template::Dict)
    incompatible_device_types = Set{DataType}()
    for model in values(devices_template)
        formulation = get_formulation(model)
        if formulation == FixedOutput
            if !isempty(get_services(model))
                @info "$(formulation) for $(get_component_type(model)) is not compatible with the provision of reserve services"
            end
            push!(incompatible_device_types, get_component_type(model))
        end
    end
    return incompatible_device_types
end

function construct_services!(
    container::OptimizationContainer,
    sys::PSY.System,
    stage::ArgumentConstructStage,
    services_template::ServicesModelContainer,
    devices_template::DevicesModelContainer,
    network_model::NetworkModel{<:PM.AbstractPowerModel},
)
    isempty(services_template) && return
    incompatible_device_types = get_incompatible_devices(devices_template)

    groupservices = Any[]

    for (key, service_model) in services_template
        if get_formulation(service_model) === GroupReserve # group service needs to be constructed last
            push!(groupservices, key)  
            continue
        end
        isempty(get_contributing_devices(service_model)) && continue
        construct_service!(
            container,
            sys,
            stage,
            service_model,
            devices_template,
            incompatible_device_types,
            network_model,
        )
    end
    for key in groupservices
        construct_service!(
            container,
            sys,
            stage,
            services_template[key],
            devices_template,
            incompatible_device_types,
            network_model,
        )
    end
    return
end

function construct_services!(
    container::OptimizationContainer,
    sys::PSY.System,
    stage::ModelConstructStage,
    services_template::ServicesModelContainer,
    devices_template::DevicesModelContainer,
    network_model::NetworkModel{<:PM.AbstractPowerModel},
)
    isempty(services_template) && return
    incompatible_device_types = get_incompatible_devices(devices_template)

    groupservices = Any[]
    for (key, service_model) in services_template
        if get_formulation(service_model) === GroupReserve # group service needs to be constructed last
            push!(groupservices, key)  
            continue
        end
        isempty(get_contributing_devices_map(service_model)) && continue
        construct_service!(
            container,
            sys,
            stage,
            service_model,
            devices_template,
            incompatible_device_types,
            network_model,
        )
    end
    for key in groupservices
        construct_service!(
            container,
            sys,
            stage,
            services_template[key],
            devices_template,
            incompatible_device_types,
            network_model,
        )
    end
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, RangeReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.Reserve}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    add_parameters!(container, RequirementTimeSeriesParameter, service, model)
    contributing_devices = get_contributing_devices(model)

    add_variables!(
        container,
        ActivePowerReserveVariable,
        service,
        contributing_devices,
        RangeReserve(),
    )
    add_to_expression!(container, ActivePowerReserveVariable, model, devices_template)
    add_feedforward_arguments!(container, model, service)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, RangeReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.Reserve}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    add_constraints!(container, RequirementConstraint, service, contributing_devices, model)
    add_constraints!(
        container,
        ParticipationFractionConstraint,
        service,
        contributing_devices,
        model,
    )
    objective_function!(container, service, model)

    add_feedforward_constraints!(container, model, service)

    add_constraint_dual!(container, sys, model)

    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, RangeReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.ConstantReserve}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    add_variables!(
        container,
        ActivePowerReserveVariable,
        service,
        contributing_devices,
        RangeReserve(),
    )
    add_to_expression!(container, ActivePowerReserveVariable, model, devices_template)
    add_feedforward_arguments!(container, model, service)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, RangeReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.ConstantReserve}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    add_constraints!(container, RequirementConstraint, service, contributing_devices, model)
    add_constraints!(
        container,
        ParticipationFractionConstraint,
        service,
        contributing_devices,
        model,
    )
    objective_function!(container, service, model)

    add_feedforward_constraints!(container, model, service)

    add_constraint_dual!(container, sys, model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, StepwiseCostReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.Reserve}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)
    add_variable!(container, ServiceRequirementVariable(), service, StepwiseCostReserve())
    add_variables!(
        container,
        ActivePowerReserveVariable,
        service,
        contributing_devices,
        StepwiseCostReserve(),
    )
    add_to_expression!(container, ActivePowerReserveVariable, model, devices_template)
    add_expressions!(container, ProductionCostExpression, [service], model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, StepwiseCostReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.Reserve}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    add_constraints!(container, RequirementConstraint, service, contributing_devices, model)

    objective_function!(container, service, model)

    add_feedforward_constraints!(container, model, service)

    add_constraint_dual!(container, sys, model)
    return
end

#=
function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{S, T},
    devices_template::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {S <: PSY.AGC, T <: AbstractAGCFormulation}
    services = get_available_components(model, sys)
    agc_areas = PSY.get_area.(services)
    areas = PSY.get_components(PSY.Area, sys)
    if !isempty(setdiff(areas, agc_areas))
        throw(
            IS.ConflictingInputsError(
                "All area must have an AGC service assigned in order to model the System's Frequency regulation",
            ),
        )
    end

    add_variables!(container, SteadyStateFrequencyDeviation)
    add_variables!(container, AreaMismatchVariable, services, T())
    add_variables!(container, SmoothACE, services, T())
    add_variables!(container, LiftVariable, services, T())
    add_variables!(container, ActivePowerVariable, areas, T())
    add_variables!(container, DeltaActivePowerUpVariable, services, T())
    add_variables!(container, DeltaActivePowerDownVariable, services, T())
    add_variables!(container, AdditionalDeltaActivePowerUpVariable, areas, T())
    add_variables!(container, AdditionalDeltaActivePowerDownVariable, areas, T())

    add_initial_condition!(container, services, T(), AreaControlError())

    add_to_expression!(
        container,
        EmergencyUp,
        AdditionalDeltaActivePowerUpVariable,
        areas,
        model,
    )

    add_to_expression!(
        container,
        EmergencyDown,
        AdditionalDeltaActivePowerDownVariable,
        areas,
        model,
    )

    add_to_expression!(container, RawACE, SteadyStateFrequencyDeviation, services, model)

    add_feedforward_arguments!(container, model, services)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{S, T},
    devices_template::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {S <: PSY.AGC, T <: AbstractAGCFormulation}
    areas = PSY.get_components(PSY.Area, sys)
    services = get_available_components(model, sys)

    add_constraints!(container, AbsoluteValueConstraint, LiftVariable, services, model)
    add_constraints!(
        container,
        FrequencyResponseConstraint,
        SteadyStateFrequencyDeviation,
        services,
        model,
        sys,
    )
    add_constraints!(
        container,
        SACEPIDAreaConstraint,
        SteadyStateFrequencyDeviation,
        services,
        model,
        sys,
    )
    add_constraints!(container, BalanceAuxConstraint, SmoothACE, services, model, sys)

    add_feedforward_constraints!(container, model, services)

    add_constraint_dual!(container, sys, model)

    objective_function!(container, services, model)
    return
end
=#

"""
    Constructs a service for ConstantReserveGroup.
"""
function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, GroupReserve},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.ConstantReserveGroup}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_services = PSY.get_contributing_services(service)
    # check if variables exist
    check_activeservice_variables(container, contributing_services)

    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, GroupReserve},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.ConstantReserveGroup}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_services = PSY.get_contributing_services(service)

    add_constraints!(
        container,
        RequirementConstraint,
        service,
        contributing_services,
        model,
    )

    add_constraint_dual!(container, sys, model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, RampReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.Reserve}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)
    add_parameters!(container, RequirementTimeSeriesParameter, service, model)

    add_variables!(
        container,
        ActivePowerReserveVariable,
        service,
        contributing_devices,
        RampReserve(),
    )
    add_to_expression!(container, ActivePowerReserveVariable, model, devices_template)
    add_feedforward_arguments!(container, model, service)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, RampReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.Reserve}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    add_constraints!(container, RequirementConstraint, service, contributing_devices, model)
    add_constraints!(container, RampConstraint, service, contributing_devices, model)
    add_constraints!(
        container,
        ParticipationFractionConstraint,
        service,
        contributing_devices,
        model,
    )

    objective_function!(container, service, model)

    add_feedforward_constraints!(container, model, service)

    add_constraint_dual!(container, sys, model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, NonSpinningReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.ReserveNonSpinning}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)
    add_parameters!(container, RequirementTimeSeriesParameter, service, model)

    add_variables!(
        container,
        ActivePowerReserveVariable,
        service,
        contributing_devices,
        NonSpinningReserve(),
    )
    add_feedforward_arguments!(container, model, service)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, NonSpinningReserve},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.ReserveNonSpinning}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    add_constraints!(container, RequirementConstraint, service, contributing_devices, model)
    add_constraints!(
        container,
        ReservePowerConstraint,
        service,
        contributing_devices,
        model,
    )

    add_constraints!(
        container,
        ParticipationFractionConstraint,
        service,
        contributing_devices,
        model,
    )

    objective_function!(container, service, model)

    add_feedforward_constraints!(container, model, service)

    add_constraint_dual!(container, sys, model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{T, ConstantMaxInterfaceFlow},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.TransmissionInterface}
    interfaces = get_available_components(model, sys)
    interface = PSY.get_component(T, sys, get_service_name(model))
    if get_use_slacks(model)
        # Adding the slacks can be done in a cleaner fashion
        @assert PSY.get_available(interface)
        transmission_interface_slacks!(container, interface)
    end
    # Lazy container addition for the expressions.
    lazy_container_addition!(
        container,
        InterfaceTotalFlow(),
        T,
        PSY.get_name.(interfaces),
        get_time_steps(container),
    )

    add_feedforward_arguments!(container, model, interface)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{PSY.TransmissionInterface, ConstantMaxInterfaceFlow},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{AreaBalancePowerModel},
)
    interfaces = get_available_components(model, sys)
    interface = PSY.get_component(PSY.TransmissionInterface, sys, get_service_name(model))
    if get_use_slacks(model)
        # Adding the slacks can be done in a cleaner fashion
        @assert PSY.get_available(interface)
        transmission_interface_slacks!(container, interface)
    end
    # Lazy container addition for the expressions.
    lazy_container_addition!(
        container,
        InterfaceTotalFlow(),
        PSY.TransmissionInterface,
        PSY.get_name.(interfaces),
        get_time_steps(container),
    )
    @warn "AreaBalancePowerModel doesn't model individual line flows and it ignores the flows on AC Transmission Devices"
    add_feedforward_arguments!(container, model, interface)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{PSY.TransmissionInterface, ConstantMaxInterfaceFlow},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    name = get_service_name(model)
    service = PSY.get_component(PSY.TransmissionInterface, sys, name)
    !PSY.get_available(service) && return

    add_to_expression!(
        container,
        InterfaceTotalFlow,
        FlowActivePowerVariable,
        service,
        model,
        network_model,
    )

    if get_use_slacks(model)
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackUp,
            service,
            model,
        )
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackDown,
            service,
            model,
        )
    end

    add_constraints!(container, InterfaceFlowLimit, service, model)
    add_feedforward_constraints!(container, model, service)
    add_constraint_dual!(container, sys, model)
    objective_function!(container, service, model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{PSY.TransmissionInterface, ConstantMaxInterfaceFlow},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{PTDFPowerModel},
)
    name = get_service_name(model)
    service = PSY.get_component(PSY.TransmissionInterface, sys, name)
    !PSY.get_available(service) && return

    add_to_expression!(
        container,
        InterfaceTotalFlow,
        PTDFBranchFlow,
        service,
        model,
        network_model,
    )

    if get_use_slacks(model)
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackUp,
            service,
            model,
        )
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackDown,
            service,
            model,
        )
    end

    add_constraints!(container, InterfaceFlowLimit, service, model)
    add_feedforward_constraints!(container, model, service)
    add_constraint_dual!(container, sys, model)
    objective_function!(container, service, model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{PSY.TransmissionInterface, ConstantMaxInterfaceFlow},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{AreaPTDFPowerModel},
)
    name = get_service_name(model)
    service = PSY.get_component(PSY.TransmissionInterface, sys, name)
    !PSY.get_available(service) && return

    # This function makes interfaces for the AC Branches
    add_to_expression!(
        container,
        InterfaceTotalFlow,
        PTDFBranchFlow,
        service,
        model,
        network_model,
    )

    # This function makes interfaces for the interchanges
    add_to_expression!(
        container,
        InterfaceTotalFlow,
        FlowActivePowerVariable,
        service,
        model,
        network_model,
    )

    if get_use_slacks(model)
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackUp,
            service,
            model,
        )
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackDown,
            service,
            model,
        )
    end

    add_constraints!(container, InterfaceFlowLimit, service, model)
    add_feedforward_constraints!(container, model, service)
    add_constraint_dual!(container, sys, model)
    objective_function!(container, service, model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{PSY.TransmissionInterface, VariableMaxInterfaceFlow},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{<:AbstractPTDFModel},
)
    name = get_service_name(model)
    service = PSY.get_component(PSY.TransmissionInterface, sys, name)
    !PSY.get_available(service) && return

    # This function makes interfaces for the AC Branches
    add_to_expression!(
        container,
        InterfaceTotalFlow,
        PTDFBranchFlow,
        service,
        model,
        network_model,
    )

    if get_use_slacks(model)
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackUp,
            service,
            model,
        )
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackDown,
            service,
            model,
        )
    end

    add_constraints!(container, InterfaceFlowLimit, service, model)
    add_feedforward_constraints!(container, model, service)
    add_constraint_dual!(container, sys, model)
    objective_function!(container, service, model)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{PSY.TransmissionInterface, U},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{T},
) where {
    T <: PM.AbstractPowerModel,
    U <: Union{ConstantMaxInterfaceFlow, VariableMaxInterfaceFlow},
}
    error("TransmissionInterface models not implemented for PowerModel of type $T")
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{PSY.TransmissionInterface, VariableMaxInterfaceFlow},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
)
    interfaces = get_available_components(model, sys)
    if get_use_slacks(model)
        # Adding the slacks can be done in a cleaner fashion
        interface =
            PSY.get_component(PSY.TransmissionInterface, sys, get_service_name(model))
        @assert PSY.get_available(interface)
        transmission_interface_slacks!(container, interface)
    end
    # Lazy container addition for the expressions.
    lazy_container_addition!(
        container,
        InterfaceTotalFlow(),
        PSY.TransmissionInterface,
        PSY.get_name.(interfaces),
        get_time_steps(container),
    )
    has_ts = PSY.has_time_series.(interfaces)
    if any(has_ts) && !all(has_ts)
        error(
            "Not all TransmissionInterfaces devices have time series. Check data to complete (or remove) time series.",
        )
    end
    if all(has_ts)
        for device in interfaces
            name = PSY.get_name(device)
            num_ts = length(unique(PSY.get_name.(PSY.get_time_series_keys(device))))
            if num_ts < 2
                error(
                    "TransmissionInterface $name has less than two time series. It is required to add both min_flow and max_flow time series.",
                )
            end
            add_parameters!(container, MinInterfaceFlowLimitParameter, device, model)
            add_parameters!(container, MaxInterfaceFlowLimitParameter, device, model)
        end
    end
    interface = PSY.get_component(PSY.TransmissionInterface, sys, get_service_name(model))
    add_feedforward_arguments!(container, model, interface)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{PSY.TransmissionInterface, U},
    devices_template::Dict{Symbol, DeviceModel},
    incompatible_device_types::Set{<:DataType},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {U <: Union{ConstantMaxInterfaceFlow, VariableMaxInterfaceFlow}}
    name = get_service_name(model)
    service = PSY.get_component(PSY.TransmissionInterface, sys, name)
    !PSY.get_available(service) && return

    add_to_expression!(
        container,
        InterfaceTotalFlow,
        FlowActivePowerVariable,
        service,
        model,
        network_model,
    )

    if get_use_slacks(model)
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackUp,
            service,
            model,
        )
        add_to_expression!(
            container,
            InterfaceTotalFlow,
            InterfaceFlowSlackDown,
            service,
            model,
        )
    end

    add_constraints!(container, InterfaceFlowLimit, service, model)
    add_feedforward_constraints!(container, model, service)
    add_constraint_dual!(container, sys, model)
    objective_function!(container, service, model)
    return
end
