# TODO: Security constrained models implement the correct functions for the model
function has_security_arguments(device_model::DeviceModel)::Bool
    return true
end

# TODO: Security constrained models implement the correct functions for the model
function has_security_model(device_model::DeviceModel)::Bool
    return true
end

@inline function _handle_common_thermal_parameters!(
    container::OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T},
) where {T <: PSY.ThermalGen}
    if haskey(get_time_series_names(model), FuelCostParameter)
        add_parameters!(container, FuelCostParameter, devices, model)
    end

    process_market_bid_parameters!(container, devices, model)
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, FixedOutput},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)
    add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerTimeSeriesParameter,
        devices,
        device_model,
        network_model,
    )
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ModelConstructStage,
    ::DeviceModel{<:PSY.ThermalGen, FixedOutput},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
)
    # FixedOutput doesn't add any constraints to the model. This function covers
    # AbstractPowerModel and AbtractActivePowerModel
    return
end

"""
This function creates the arguments for the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PSY.ThermalGen,
    D <: AbstractStandardUnitCommitment,
}
    devices = get_available_components(device_model, sys)

    add_variables!(container, ActivePowerVariable, devices, D())
    add_variables!(container, ReactivePowerVariable, devices, D())
    add_variables!(container, OnVariable, devices, D())
    add_variables!(container, StartVariable, devices, D())
    add_variables!(container, StopVariable, devices, D())

    add_variables!(container, TimeDurationOn, devices, D())
    add_variables!(container, TimeDurationOff, devices, D())

    initial_conditions!(container, devices, D())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ReactivePowerBalance,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        ActivePowerVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(container, RateofChangeConstraintSlackUp, devices, D())
        add_variables!(container, RateofChangeConstraintSlackDown, devices, D())
    end
    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

"""
This function creates the constraints for the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, U},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PSY.ThermalGen,
    U <: AbstractStandardUnitCommitment,
}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    add_constraints!(container, RampConstraint, devices, device_model, network_model)
    add_constraints!(container, DurationConstraint, devices, device_model, network_model)

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)
    add_event_constraints!(container, devices, device_model, network_model)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )

    if has_security_arguments(device_model)
        # TODO: SecurityConstrainedModels Implemenation of G-1
        #add_constraints!(
        #    container,
        #    SecurityConstraint,
        #    devices,
        #    device_model,
        #    network_model,
        #)
    end

    add_constraint_dual!(container, sys, device_model)
    return
end

"""
This function creates the arguments model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen, D <: AbstractStandardUnitCommitment}
    devices = get_available_components(device_model, sys)

    add_variables!(container, ActivePowerVariable, devices, D())
    add_variables!(container, OnVariable, devices, D())
    add_variables!(container, StartVariable, devices, D())
    add_variables!(container, StopVariable, devices, D())

    add_variables!(container, TimeDurationOn, devices, D())
    add_variables!(container, TimeDurationOff, devices, D())

    initial_conditions!(container, devices, D())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        ActivePowerVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(container, RateofChangeConstraintSlackUp, devices, D())
        add_variables!(container, RateofChangeConstraintSlackDown, devices, D())
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

"""
This function creates the constraints for the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, <:AbstractStandardUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    add_constraints!(container, RampConstraint, devices, device_model, network_model)
    add_constraints!(container, DurationConstraint, devices, device_model, network_model)
    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalBasicUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(container, ActivePowerVariable, devices, ThermalBasicUnitCommitment())
    add_variables!(container, ReactivePowerVariable, devices, ThermalBasicUnitCommitment())
    add_variables!(container, OnVariable, devices, ThermalBasicUnitCommitment())
    add_variables!(container, StartVariable, devices, ThermalBasicUnitCommitment())
    add_variables!(container, StopVariable, devices, ThermalBasicUnitCommitment())

    add_variables!(container, TimeDurationOn, devices, ThermalBasicUnitCommitment())
    add_variables!(container, TimeDurationOff, devices, ThermalBasicUnitCommitment())
    initial_conditions!(container, devices, ThermalBasicUnitCommitment())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ReactivePowerBalance,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        ActivePowerVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalBasicUnitCommitment(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalBasicUnitCommitment(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalBasicUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

"""
This function creates the arguments for the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalBasicUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(container, ActivePowerVariable, devices, ThermalBasicUnitCommitment())
    add_variables!(container, OnVariable, devices, ThermalBasicUnitCommitment())
    add_variables!(container, StartVariable, devices, ThermalBasicUnitCommitment())
    add_variables!(container, StopVariable, devices, ThermalBasicUnitCommitment())

    add_variables!(container, TimeDurationOn, devices, ThermalBasicUnitCommitment())
    add_variables!(container, TimeDurationOff, devices, ThermalBasicUnitCommitment())
    initial_conditions!(container, devices, ThermalBasicUnitCommitment())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        ActivePowerVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalBasicUnitCommitment(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalBasicUnitCommitment(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

"""
This function creates the constraints for the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalBasicUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalStandardDispatch},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(container, ActivePowerVariable, devices, ThermalStandardDispatch())
    add_variables!(container, ReactivePowerVariable, devices, ThermalStandardDispatch())

    initial_conditions!(container, devices, ThermalStandardDispatch())

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ReactivePowerBalance,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        ActivePowerVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalStandardDispatch(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalStandardDispatch(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalStandardDispatch},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, RampConstraint, devices, device_model, network_model)

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

"""
This function creates the arguments for the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalStandardDispatch},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(container, ActivePowerVariable, devices, ThermalStandardDispatch())

    initial_conditions!(container, devices, ThermalStandardDispatch())

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        ActivePowerVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalStandardDispatch(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalStandardDispatch(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

"""
This function creates the constraints for the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalStandardDispatch},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(container, RampConstraint, devices, device_model, network_model)

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PSY.ThermalGen,
    D <: AbstractThermalDispatchFormulation,
}
    devices = get_available_components(device_model, sys)

    add_variables!(container, ActivePowerVariable, devices, D())
    add_variables!(container, ReactivePowerVariable, devices, D())

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ReactivePowerBalance,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        ActivePowerVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(container, RateofChangeConstraintSlackUp, devices, D())
        add_variables!(container, RateofChangeConstraintSlackDown, devices, D())
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, <:AbstractThermalDispatchFormulation},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: PSY.ThermalGen,
    D <: AbstractThermalDispatchFormulation,
}
    devices = get_available_components(device_model, sys)

    add_variables!(container, ActivePowerVariable, devices, D())

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        ActivePowerVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(container, RateofChangeConstraintSlackUp, devices, D())
        add_variables!(container, RateofChangeConstraintSlackDown, devices, D())
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, <:AbstractThermalDispatchFormulation},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_feedforward_constraints!(container, device_model, devices)
    add_event_constraints!(container, devices, device_model, network_model)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{PSY.ThermalMultiStart, ThermalMultiStartUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
)
    devices = get_available_components(device_model, sys)

    add_variables!(
        container,
        PowerAboveMinimumVariable,
        devices,
        ThermalMultiStartUnitCommitment(),
    )
    add_variables!(
        container,
        ReactivePowerVariable,
        devices,
        ThermalMultiStartUnitCommitment(),
    )
    add_variables!(container, OnVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, StopVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, StartVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, ColdStartVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, WarmStartVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, HotStartVariable, devices, ThermalMultiStartUnitCommitment())

    add_variables!(container, TimeDurationOn, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, TimeDurationOff, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, PowerOutput, devices, ThermalMultiStartUnitCommitment())

    initial_conditions!(container, devices, ThermalMultiStartUnitCommitment())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        OnVariable,
        devices,
        device_model,
        network_model,
    )

    add_to_expression!(
        container,
        ReactivePowerBalance,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        PowerAboveMinimumVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalMultiStartUnitCommitment(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalMultiStartUnitCommitment(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.ThermalMultiStart, ThermalMultiStartUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
)
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    add_constraints!(container, RampConstraint, devices, device_model, network_model)
    add_constraints!(container, DurationConstraint, devices, device_model, network_model)
    add_constraints!(
        container,
        StartupTimeLimitTemperatureConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, StartTypeConstraint, devices, device_model, network_model)
    add_constraints!(
        container,
        StartupInitialConditionConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActiveRangeICConstraint,
        devices,
        device_model,
        network_model,
    )

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{PSY.ThermalMultiStart, ThermalMultiStartUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(device_model, sys)

    add_variables!(
        container,
        PowerAboveMinimumVariable,
        devices,
        ThermalMultiStartUnitCommitment(),
    )
    add_variables!(container, OnVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, StopVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, StartVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, ColdStartVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, WarmStartVariable, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, HotStartVariable, devices, ThermalMultiStartUnitCommitment())

    add_variables!(container, TimeDurationOn, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, TimeDurationOff, devices, ThermalMultiStartUnitCommitment())
    add_variables!(container, PowerOutput, devices, ThermalMultiStartUnitCommitment())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )

    add_to_expression!(
        container,
        ActivePowerBalance,
        OnVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        PowerAboveMinimumVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalMultiStartUnitCommitment(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalMultiStartUnitCommitment(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.ThermalMultiStart, ThermalMultiStartUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(device_model, sys)

    initial_conditions!(container, devices, ThermalMultiStartUnitCommitment())

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    add_constraints!(container, RampConstraint, devices, device_model, network_model)
    add_constraints!(container, DurationConstraint, devices, device_model, network_model)
    add_constraints!(
        container,
        StartupTimeLimitTemperatureConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, StartTypeConstraint, devices, device_model, network_model)
    add_constraints!(
        container,
        StartupInitialConditionConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActiveRangeICConstraint,
        devices,
        device_model,
        network_model,
    )

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalCompactUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(
        container,
        PowerAboveMinimumVariable,
        devices,
        ThermalCompactUnitCommitment(),
    )
    add_variables!(
        container,
        ReactivePowerVariable,
        devices,
        ThermalCompactUnitCommitment(),
    )
    add_variables!(container, OnVariable, devices, ThermalCompactUnitCommitment())
    add_variables!(container, StartVariable, devices, ThermalCompactUnitCommitment())
    add_variables!(container, StopVariable, devices, ThermalCompactUnitCommitment())

    add_variables!(container, TimeDurationOn, devices, ThermalCompactUnitCommitment())
    add_variables!(container, TimeDurationOff, devices, ThermalCompactUnitCommitment())
    add_variables!(container, PowerOutput, devices, ThermalCompactUnitCommitment())

    initial_conditions!(container, devices, ThermalCompactUnitCommitment())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        OnVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        PowerAboveMinimumVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalCompactUnitCommitment(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalCompactUnitCommitment(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalCompactUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    add_constraints!(container, RampConstraint, devices, device_model, network_model)
    add_constraints!(container, DurationConstraint, devices, device_model, network_model)
    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalCompactUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(
        container,
        PowerAboveMinimumVariable,
        devices,
        ThermalCompactUnitCommitment(),
    )
    add_variables!(container, OnVariable, devices, ThermalCompactUnitCommitment())
    add_variables!(container, StartVariable, devices, ThermalCompactUnitCommitment())
    add_variables!(container, StopVariable, devices, ThermalCompactUnitCommitment())

    add_variables!(container, TimeDurationOn, devices, ThermalCompactUnitCommitment())
    add_variables!(container, TimeDurationOff, devices, ThermalCompactUnitCommitment())
    add_variables!(container, PowerOutput, devices, ThermalCompactUnitCommitment())

    initial_conditions!(container, devices, ThermalCompactUnitCommitment())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        OnVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        PowerAboveMinimumVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalCompactUnitCommitment(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalCompactUnitCommitment(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalCompactUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    add_constraints!(container, RampConstraint, devices, device_model, network_model)
    add_constraints!(container, DurationConstraint, devices, device_model, network_model)
    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalBasicCompactUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(
        container,
        PowerAboveMinimumVariable,
        devices,
        ThermalBasicCompactUnitCommitment(),
    )
    add_variables!(
        container,
        ReactivePowerVariable,
        devices,
        ThermalBasicCompactUnitCommitment(),
    )
    add_variables!(container, OnVariable, devices, ThermalBasicCompactUnitCommitment())
    add_variables!(container, StartVariable, devices, ThermalBasicCompactUnitCommitment())
    add_variables!(container, StopVariable, devices, ThermalBasicCompactUnitCommitment())

    add_variables!(container, PowerOutput, devices, ThermalBasicCompactUnitCommitment())
    add_variables!(container, TimeDurationOn, devices, ThermalBasicCompactUnitCommitment())
    add_variables!(container, TimeDurationOff, devices, ThermalBasicCompactUnitCommitment())

    initial_conditions!(container, devices, ThermalBasicCompactUnitCommitment())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        OnVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        PowerAboveMinimumVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalBasicCompactUnitCommitment(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalBasicCompactUnitCommitment(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalBasicCompactUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalBasicCompactUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(
        container,
        PowerAboveMinimumVariable,
        devices,
        ThermalBasicCompactUnitCommitment(),
    )
    add_variables!(container, OnVariable, devices, ThermalBasicCompactUnitCommitment())
    add_variables!(container, StartVariable, devices, ThermalBasicCompactUnitCommitment())
    add_variables!(container, StopVariable, devices, ThermalBasicCompactUnitCommitment())

    add_variables!(container, PowerOutput, devices, ThermalBasicCompactUnitCommitment())
    add_variables!(container, TimeDurationOn, devices, ThermalBasicCompactUnitCommitment())
    add_variables!(container, TimeDurationOff, devices, ThermalBasicCompactUnitCommitment())
    initial_conditions!(container, devices, ThermalBasicCompactUnitCommitment())

    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, device_model)
    end

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        OnVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        PowerAboveMinimumVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalBasicCompactUnitCommitment(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalBasicCompactUnitCommitment(),
        )
    end
    add_feedforward_arguments!(container, device_model, devices)
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalBasicCompactUnitCommitment},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(container, CommitmentConstraint, devices, device_model, network_model)
    if haskey(get_time_series_names(device_model), ActivePowerTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalCompactDispatch},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(container, PowerAboveMinimumVariable, devices, ThermalCompactDispatch())
    add_variables!(container, ReactivePowerVariable, devices, ThermalCompactDispatch())

    add_variables!(container, PowerOutput, devices, ThermalCompactDispatch())

    add_parameters!(container, OnStatusParameter, devices, device_model)

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_feedforward_arguments!(container, device_model, devices)

    initial_conditions!(container, devices, ThermalCompactDispatch())

    add_to_expression!(
        container,
        ActivePowerBalance,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ReactivePowerBalance,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        OnStatusParameter,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        PowerAboveMinimumVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalCompactDispatch(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalCompactDispatch(),
        )
    end
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalCompactDispatch},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, RampConstraint, devices, device_model, network_model)

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )
    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, ThermalCompactDispatch},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_variables!(container, PowerAboveMinimumVariable, devices, ThermalCompactDispatch())

    add_variables!(container, PowerOutput, devices, ThermalCompactDispatch())

    add_parameters!(container, OnStatusParameter, devices, device_model)

    _handle_common_thermal_parameters!(container, devices, device_model)

    add_feedforward_arguments!(container, device_model, devices)

    add_to_expression!(
        container,
        ActivePowerBalance,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )

    add_to_expression!(
        container,
        ActivePowerBalance,
        OnStatusParameter,
        devices,
        device_model,
        network_model,
    )

    initial_conditions!(container, devices, ThermalCompactDispatch())

    add_cost_expressions!(container, devices, device_model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        PowerAboveMinimumVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        FuelConsumptionExpression,
        PowerAboveMinimumVariable,
        devices,
        device_model,
    )
    if get_use_slacks(device_model)
        add_variables!(
            container,
            RateofChangeConstraintSlackUp,
            devices,
            ThermalCompactDispatch(),
        )
        add_variables!(
            container,
            RateofChangeConstraintSlackDown,
            devices,
            ThermalCompactDispatch(),
        )
    end
    add_event_arguments!(container, devices, device_model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, ThermalCompactDispatch},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ThermalGen}
    devices = get_available_components(device_model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        device_model,
        network_model,
    )

    add_constraints!(container, RampConstraint, devices, device_model, network_model)

    add_feedforward_constraints!(container, device_model, devices)

    objective_function!(
        container,
        devices,
        device_model,
        get_network_formulation(network_model),
    )

    add_event_constraints!(container, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end
