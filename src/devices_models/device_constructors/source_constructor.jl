"""
This function creates the arguments for the model for an import/export formulation for Source devices
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PSY.Source,
    D <: ImportExportSourceModel,
}
    devices = get_available_components(model, sys)

    add_variables!(container, ActivePowerInVariable, devices, D())
    add_variables!(container, ActivePowerOutVariable, devices, D())
    add_variables!(container, ReactivePowerVariable, devices, D())
    add_expressions!(container, NetActivePower, devices, model)

    if haskey(get_time_series_names(model), ActivePowerOutTimeSeriesParameter)
        add_parameters!(container, ActivePowerOutTimeSeriesParameter, devices, model)
    end
    if haskey(get_time_series_names(model), ActivePowerInTimeSeriesParameter)
        add_parameters!(container, ActivePowerInTimeSeriesParameter, devices, model)
    end

    if get_attribute(model, "reservation")
        add_variables!(container, ReservationVariable, devices, D())
    end

    process_import_export_parameters!(container, devices, model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerInVariable,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerOutVariable,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        ReactivePowerBalance,
        ReactivePowerVariable,
        devices,
        model,
        network_model,
    )

    add_expressions!(container, ProductionCostExpression, devices, model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerOutVariable,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerOutVariable,
        devices,
        model,
        network_model,
    )

    add_feedforward_arguments!(container, model, devices)
    return
end

"""
This function creates the constraints for the model for an import/export formulation for Source devices
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PSY.Source,
    D <: ImportExportSourceModel,
}
    devices = get_available_components(model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        InputActivePowerVariableLimitsConstraint,
        ActivePowerInVariable,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        model,
        network_model,
    )
    add_constraints!(container, ImportExportBudgetConstraint, devices, model, network_model)

    if haskey(get_time_series_names(model), ActivePowerOutTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerOutVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            model,
            network_model,
        )
    end
    if haskey(get_time_series_names(model), ActivePowerInTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerInVariableTimeSeriesLimitsConstraint,
            ActivePowerInVariable,
            devices,
            model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, model, devices)

    objective_function!(container, devices, model, get_network_formulation(network_model))
    add_constraint_dual!(container, sys, model)
    return
end

"""
This function creates the arguments for the model for an import/export formulation for Source devices
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: PSY.Source,
    D <: ImportExportSourceModel,
}
    devices = get_available_components(model, sys)

    add_variables!(container, ActivePowerInVariable, devices, D())
    add_variables!(container, ActivePowerOutVariable, devices, D())
    add_expressions!(container, NetActivePower, devices, model)

    if haskey(get_time_series_names(model), ActivePowerOutTimeSeriesParameter)
        add_parameters!(container, ActivePowerOutTimeSeriesParameter, devices, model)
    end
    if haskey(get_time_series_names(model), ActivePowerInTimeSeriesParameter)
        add_parameters!(container, ActivePowerInTimeSeriesParameter, devices, model)
    end

    if get_attribute(model, "reservation")
        add_variables!(container, ReservationVariable, devices, D())
    end

    process_import_export_parameters!(container, devices, model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerInVariable,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerOutVariable,
        devices,
        model,
        network_model,
    )

    add_to_expression!(
        container,
        NetActivePower,
        ActivePowerInVariable(),
        devices,
        model,
    )
    add_to_expression!(
        container,
        NetActivePower,
        ActivePowerOutVariable(),
        devices,
        model,
    )

    add_expressions!(container, ProductionCostExpression, devices, model)

    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        ActivePowerOutVariable,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        ActivePowerOutVariable,
        devices,
        model,
        network_model,
    )

    add_feedforward_arguments!(container, model, devices)
    return
end

"""
This function creates the constraints for the model for an import/export formulation for Source devices
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: PSY.Source,
    D <: ImportExportSourceModel,
}
    devices = get_available_components(model, sys)

    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionLB,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        ActivePowerVariableLimitsConstraint,
        ActivePowerRangeExpressionUB,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        InputActivePowerVariableLimitsConstraint,
        ActivePowerInVariable,
        devices,
        model,
        network_model,
    )

    add_constraints!(container, ImportExportBudgetConstraint, devices, model, network_model)

    if haskey(get_time_series_names(model), ActivePowerOutTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerOutVariableTimeSeriesLimitsConstraint,
            ActivePowerRangeExpressionUB,
            devices,
            model,
            network_model,
        )
    end
    if haskey(get_time_series_names(model), ActivePowerInTimeSeriesParameter)
        add_constraints!(
            container,
            ActivePowerInVariableTimeSeriesLimitsConstraint,
            ActivePowerInVariable,
            devices,
            model,
            network_model,
        )
    end

    add_feedforward_constraints!(container, model, devices)

    objective_function!(container, devices, model, get_network_formulation(network_model))
    add_constraint_dual!(container, sys, model)
    return
end

"""
This function creates the arguments for the model for a FixedOutput formulation for Source devices
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: PSY.Source,
    D <: FixedOutput,
}
    devices = get_available_components(model, sys)
    add_parameters!(container, ActivePowerInTimeSeriesParameter, devices, model)
    add_parameters!(container, ActivePowerOutTimeSeriesParameter, devices, model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerInTimeSeriesParameter,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerOutTimeSeriesParameter,
        devices,
        model,
        network_model,
    )
    add_event_arguments!(container, devices, model, network_model)
    return
end

"""
This function creates the constraints for the model for a FixedOutput formulation for Source devices (no constraints added for FixedOutput)
"""
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{T, D},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: PSY.Source,
    D <: FixedOutput,
}
    # FixedOutput doesn't add any constraints to the model.
    return
end
