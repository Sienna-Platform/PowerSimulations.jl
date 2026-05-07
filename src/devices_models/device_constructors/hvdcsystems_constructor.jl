function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{PSY.InterconnectingConverter, LosslessConverter},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(
        model,
        sys,
    )
    add_variables!(container, ActivePowerVariable, devices, LosslessConverter())
    add_to_expression!(
        container,
        sys,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        model,
        network_model,
    )
    add_feedforward_arguments!(container, model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{PSY.InterconnectingConverter, LosslessConverter},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(
        model,
        sys,
    )
    add_feedforward_constraints!(container, model, devices)
    objective_function!(container, devices, model, get_network_formulation(network_model))
    add_constraint_dual!(container, sys, model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{PSY.InterconnectingConverter, QuadraticLossConverter},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(
        model,
        sys,
    )
    #####################
    ##### Variables #####
    #####################

    # Add Power Variable
    add_variables!(container, ActivePowerVariable, devices, QuadraticLossConverter()) # p_c^{ac}
    add_variables!(container, ConverterDCPower, devices, QuadraticLossConverter()) # p_c
    # Add Current Variables: i, i+, i-
    add_variables!(container, ConverterCurrent, devices, QuadraticLossConverter()) # i
    add_variables!(container, SquaredConverterCurrent, devices, QuadraticLossConverter()) # i^sq
    use_linear_loss = PSI.get_attribute(model, "use_linear_loss")
    if use_linear_loss
        add_variables!(
            container,
            ConverterPositiveCurrent,
            devices,
            QuadraticLossConverter(),
        ) # i^+
        add_variables!(
            container,
            ConverterNegativeCurrent,
            devices,
            QuadraticLossConverter(),
        ) # i^-
        add_variables!(
            container,
            ConverterCurrentDirection,
            devices,
            QuadraticLossConverter(),
        ) # ν
    end
    # Add Voltage Variables: v^sq
    add_variables!(container, SquaredDCVoltage, devices, QuadraticLossConverter())
    # Add Bilinear Variables: γ, γ^{sq}
    add_variables!(
        container,
        AuxBilinearConverterVariable,
        devices,
        QuadraticLossConverter(),
    ) # γ
    add_variables!(
        container,
        AuxBilinearSquaredConverterVariable,
        devices,
        QuadraticLossConverter(),
    ) # γ^{sq}

    #### Add Interpolation Variables ####

    v_segments = PSI.get_attribute(model, "voltage_segments")
    i_segments = PSI.get_attribute(model, "current_segments")
    γ_segments = PSI.get_attribute(model, "bilinear_segments")

    vars_vector = [
        # Voltage v #
        (InterpolationSquaredVoltageVariable, v_segments), # δ^v
        (InterpolationBinarySquaredVoltageVariable, v_segments), # z^v
        # Current i #
        (InterpolationSquaredCurrentVariable, i_segments), # δ^i
        (InterpolationBinarySquaredCurrentVariable, i_segments), # z^i
        # Bilinear γ #
        (InterpolationSquaredBilinearVariable, γ_segments), # δ^γ
        (InterpolationBinarySquaredBilinearVariable, γ_segments), # z^γ
    ]

    for (T, len_segments) in vars_vector
        add_sparse_pwl_interpolation_variables!(
            container,
            T(),
            devices,
            model,
            len_segments,
        )
    end

    #####################
    #### Expressions ####
    #####################

    add_to_expression!(
        container,
        sys,
        ActivePowerBalance,
        ActivePowerVariable,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        sys,
        DCCurrentBalance,
        ConverterCurrent,
        devices,
        model,
        network_model,
    )
    add_feedforward_arguments!(container, model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{PSY.InterconnectingConverter, QuadraticLossConverter},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(
        model,
        sys,
    )

    add_constraints!(
        container,
        ConverterPowerCalculationConstraint,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        ConverterMcCormickEnvelopes,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        ConverterLossConstraint,
        devices,
        model,
        network_model,
    )
    use_linear_loss = PSI.get_attribute(model, "use_linear_loss")
    if use_linear_loss
        add_constraints!(
            container,
            CurrentAbsoluteValueConstraint,
            devices,
            model,
            network_model,
        )
    end
    add_constraints!(
        container,
        InterpolationVoltageConstraints,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        InterpolationCurrentConstraints,
        devices,
        model,
        network_model,
    )
    add_constraints!(
        container,
        InterpolationBilinearConstraints,
        devices,
        model,
        network_model,
    )

    add_feedforward_constraints!(container, model, devices)
    objective_function!(container, devices, model, get_network_formulation(network_model))
    #add_constraint_dual!(container, sys, model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{PSY.TModelHVDCLine, LosslessLine},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(
        model,
        sys,
    )
    add_variables!(container, FlowActivePowerVariable, devices, LosslessLine())
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerVariable,
        devices,
        model,
        network_model,
    )
    add_feedforward_arguments!(container, model, devices)
    return
end

function construct_device!(
    ::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{PSY.TModelHVDCLine, LosslessLine},
    ::NetworkModel{<:PM.AbstractActivePowerModel},
)
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{PSY.TModelHVDCLine, DCLossyLine},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(
        model,
        sys,
    )

    add_variables!(container, DCLineCurrent, devices, DCLossyLine())
    add_to_expression!(
        container,
        DCCurrentBalance,
        DCLineCurrent,
        devices,
        model,
        network_model,
    )
    add_feedforward_arguments!(container, model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::DeviceModel{PSY.TModelHVDCLine, DCLossyLine},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = get_available_components(
        model,
        sys,
    )
    add_constraints!(
        container,
        DCLineCurrentConstraint,
        devices,
        model,
        network_model,
    )
end
