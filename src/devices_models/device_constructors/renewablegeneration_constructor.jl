function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{R, D},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {
    R <: PSY.RenewableGen,
    D <: AbstractRenewableDispatchFormulation,
}
    devices = get_available_components(model, sys)

    add_variables!(container, ActivePowerVariable, devices, D())
    add_variables!(container, ReactivePowerVariable, devices, D())
    if haskey(get_time_series_names(model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, model)
    end
    if haskey(get_time_series_names(model), ReactivePowerTimeSeriesParameter)
        add_parameters!(container, ReactivePowerTimeSeriesParameter, devices, model)
    end
    process_market_bid_parameters!(container, devices, model)

    add_cost_expressions!(container, devices, model)

    # Expression
    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
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
    if has_service_model(model)
        add_to_expression!(
            container,
            ActivePowerRangeExpressionLB,
            ActivePowerVariable,
            devices,
            model,
            network_model,
        )
        add_to_expression!(
            container,
            ActivePowerRangeExpressionUB,
            ActivePowerVariable,
            devices,
            model,
            network_model,
        )
    end
    add_feedforward_arguments!(container, model, devices)
    add_event_arguments!(container, devices, model, network_model)
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{R, <:AbstractRenewableDispatchFormulation},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {R <: PSY.RenewableGen}
    devices = get_available_components(model, sys)

    if has_service_model(model)
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
    else
        add_constraints!(
            container,
            ActivePowerVariableLimitsConstraint,
            ActivePowerVariable,
            devices,
            model,
            network_model,
        )
    end

    add_constraints!(
        container,
        ReactivePowerVariableLimitsConstraint,
        ReactivePowerVariable,
        devices,
        model,
        network_model,
    )
    add_feedforward_constraints!(container, model, devices)

    objective_function!(container, devices, model, get_network_formulation(network_model))

    add_event_constraints!(container, devices, model, network_model)
    add_constraint_dual!(container, sys, model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{R, D},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    R <: PSY.RenewableGen,
    D <: AbstractRenewableDispatchFormulation,
}
    devices = get_available_components(model, sys)

    add_variables!(container, ActivePowerVariable, devices, D())
    # this is always true!! see get_default_time_series_names in renewable_generation.jl
    # and line 62 of device_model.jl
    if haskey(get_time_series_names(model), ActivePowerTimeSeriesParameter)
        add_parameters!(container, ActivePowerTimeSeriesParameter, devices, model)
    end
    process_market_bid_parameters!(container, devices, model)

    add_cost_expressions!(container, devices, model)

    # Expression
    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        model,
        network_model,
    )
    if has_service_model(model)
        add_to_expression!(
            container,
            ActivePowerRangeExpressionLB,
            ActivePowerVariable,
            devices,
            model,
            network_model,
        )
        add_to_expression!(
            container,
            ActivePowerRangeExpressionUB,
            ActivePowerVariable,
            devices,
            model,
            network_model,
        )
    end
    add_feedforward_arguments!(container, model, devices)
    add_event_arguments!(container, devices, model, network_model)
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{R, <:AbstractRenewableDispatchFormulation},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {R <: PSY.RenewableGen}
    devices = get_available_components(model, sys)

    if has_service_model(model)
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
    else
        add_constraints!(
            container,
            ActivePowerVariableLimitsConstraint,
            ActivePowerVariable,
            devices,
            model,
            network_model,
        )
    end
    add_feedforward_constraints!(container, model, devices)

    objective_function!(container, devices, model, get_network_formulation(network_model))

    add_event_constraints!(container, devices, model, network_model)
    add_constraint_dual!(container, sys, model)

    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{R, FixedOutput},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {R <: PSY.RenewableGen}
    devices = get_available_components(model, sys)

    add_parameters!(container, ActivePowerTimeSeriesParameter, devices, model)
    add_parameters!(container, ReactivePowerTimeSeriesParameter, devices, model)
    process_market_bid_parameters!(container, devices, model)

    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerTimeSeriesParameter,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        ReactivePowerBalance,
        ReactivePowerTimeSeriesParameter,
        devices,
        model,
        network_model,
    )
    add_event_arguments!(container, devices, model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{R, FixedOutput},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {R <: PSY.RenewableGen}
    devices = get_available_components(model, sys)

    add_parameters!(container, ActivePowerTimeSeriesParameter, devices, model)
    add_to_expression!(
        container,
        ActivePowerBalance,
        ActivePowerTimeSeriesParameter,
        devices,
        model,
        network_model,
    )
    process_market_bid_parameters!(container, devices, model)
    add_event_arguments!(container, devices, model, network_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{<:PSY.RenewableGen, FixedOutput},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
)
    # FixedOutput doesn't add any constraints to the model. This function covers
    # AbstractPowerModel and AbtractActivePowerModel
    return
end
