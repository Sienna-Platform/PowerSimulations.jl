################################# Generic AC Branch  Models ################################
# These 3 methods are defined on concrete formulations of the branches to avoid ambiguity
function construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ArgumentConstructStage,
    ::DeviceModel{T, StaticBranch},
    ::Union{
        NetworkModel{CopperPlatePowerModel},
        NetworkModel{AreaBalancePowerModel},
    },
) where {T <: PSY.ACTransmission}
    @debug "No argument construction needed for CopperPlatePowerModel or AreaBalancePowerModel and DeviceModel{$T, StaticBranch}" _group =
        LOG_GROUP_BRANCH_CONSTRUCTIONS
    return
end

function construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ModelConstructStage,
    ::DeviceModel{T, StaticBranch},
    ::Union{
        NetworkModel{CopperPlatePowerModel},
        NetworkModel{AreaBalancePowerModel},
    },
) where {T <: PSY.ACTransmission}
    @debug "No model construction needed for CopperPlatePowerModel or AreaBalancePowerModel and DeviceModel{$T, StaticBranch}" _group =
        LOG_GROUP_BRANCH_CONSTRUCTIONS
    return
end

function construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ArgumentConstructStage,
    ::DeviceModel{T, StaticBranchBounds},
    ::Union{
        NetworkModel{CopperPlatePowerModel},
        NetworkModel{AreaBalancePowerModel},
    },
) where {T <: PSY.ACTransmission}
    @debug "No argument construction needed for CopperPlatePowerModel or AreaBalancePowerModel and DeviceModel{$T, StaticBranchBounds}" _group =
        LOG_GROUP_BRANCH_CONSTRUCTIONS
    return
end

function construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ModelConstructStage,
    ::DeviceModel{T, StaticBranchBounds},
    ::Union{
        NetworkModel{CopperPlatePowerModel},
        NetworkModel{AreaBalancePowerModel},
    },
) where {T <: PSY.ACTransmission}
    @debug "No model construction needed for CopperPlatePowerModel or AreaBalancePowerModel and DeviceModel{$T, StaticBranchBounds}" _group =
        LOG_GROUP_BRANCH_CONSTRUCTIONS
    return
end

construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ArgumentConstructStage,
    ::DeviceModel{<:PSY.ACTransmission, StaticBranchUnbounded},
    ::NetworkModel{<:PM.AbstractPowerModel},
) = nothing

construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ModelConstructStage,
    ::DeviceModel{<:PSY.ACTransmission, StaticBranchUnbounded},
    ::NetworkModel{<:PM.AbstractPowerModel},
) = nothing

# For DC Power only. Implements constraints
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, StaticBranch},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)
    if get_use_slacks(device_model)
        add_variables!(
            container,
            FlowActivePowerSlackUpperBound,
            network_model,
            devices,
            StaticBranch(),
        )
        add_variables!(
            container,
            FlowActivePowerSlackLowerBound,
            network_model,
            devices,
            StaticBranch(),
        )
    end
    add_feedforward_arguments!(container, device_model, devices)
    return
end

# For DC Power only. Implements constraints
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, StaticBranch},
    network_model::NetworkModel{U},
) where {T <: PSY.ACTransmission, U <: PM.AbstractActivePowerModel}
    @debug "construct_device" _group = LOG_GROUP_BRANCH_CONSTRUCTIONS

    devices = get_available_components(device_model, sys)
    add_constraints!(container, FlowRateConstraint, devices, device_model, network_model)
    add_feedforward_constraints!(container, device_model, devices)
    objective_function!(container, devices, device_model, U)
    add_constraint_dual!(container, sys, device_model)
    return
end

# For DC Power only
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, StaticBranch},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)
    if get_use_slacks(device_model)
        add_variables!(
            container,
            FlowActivePowerSlackUpperBound,
            network_model,
            devices,
            StaticBranch(),
        )
        add_variables!(
            container,
            FlowActivePowerSlackLowerBound,
            network_model,
            devices,
            StaticBranch(),
        )
    end

    add_branch_parameters!(
        container,
        DynamicBranchRatingTimeSeriesParameter,
        devices,
        device_model,
        network_model,
    )

    if haskey(
        get_time_series_names(device_model),
        PostContingencyDynamicBranchRatingTimeSeriesParameter,
    )
        add_branch_parameters!(
            container,
            PostContingencyDynamicBranchRatingTimeSeriesParameter,
            devices,
            device_model,
            network_model,
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, StaticBranch},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)

    # The order of these methods is important. The add_expressions! must be before the constraints
    add_expressions!(
        container,
        PTDFBranchFlow,
        devices,
        device_model,
        network_model,
    )
    if haskey(get_time_series_names(device_model), DynamicBranchRatingTimeSeriesParameter)
        add_flow_rate_constraint_with_parameters!(
            container,
            FlowRateConstraint,
            devices,
            device_model,
            network_model,
        )
    else
        add_constraints!(
            container,
            FlowRateConstraint,
            devices,
            device_model,
            network_model,
        )
    end
    add_feedforward_constraints!(container, device_model, devices)
    objective_function!(container, devices, device_model, PTDFPowerModel)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, StaticBranchBounds},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)

    add_variables!(
        container,
        FlowActivePowerVariable,
        network_model,
        devices,
        StaticBranchBounds(),
    )

    if get_use_slacks(device_model)
        add_variables!(
            container,
            FlowActivePowerSlackUpperBound,
            network_model,
            devices,
            StaticBranch(),
        )
        add_variables!(
            container,
            FlowActivePowerSlackLowerBound,
            network_model,
            devices,
            StaticBranch(),
        )
    end

    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, StaticBranchBounds},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)
    # The order of these methods is important. The add_expressions! must be before the constraints
    add_expressions!(
        container,
        PTDFBranchFlow,
        devices,
        device_model,
        network_model,
    )

    branch_rate_bounds!(container, device_model, network_model)
    add_constraints!(container, NetworkFlowConstraint, devices, device_model, network_model)
    add_feedforward_constraints!(container, device_model, devices)
    objective_function!(container, devices, device_model, PTDFPowerModel)
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, StaticBranchUnbounded},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, StaticBranchUnbounded},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)
    # The order of these methods is important. The add_expressions! must be before the constraints
    add_expressions!(
        container,
        PTDFBranchFlow,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_constraints!(container, device_model, devices)
    add_constraints!(container, NetworkFlowConstraint, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    return
end

# For AC Power only. Implements Bounds on the active power and rating constraints on the aparent power
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, StaticBranch},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)

    if get_use_slacks(device_model)
        # Only one slack is needed for this formulations in AC
        add_variables!(
            container,
            FlowActivePowerSlackUpperBound,
            network_model,
            devices,
            StaticBranch(),
        )
    end
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, StaticBranch},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)
    add_feedforward_constraints!(container, device_model, devices)
    add_constraints!(
        container,
        FlowRateConstraintFromTo,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        FlowRateConstraintToFrom,
        devices,
        device_model,
        network_model,
    )
    add_constraint_dual!(container, sys, device_model)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, StaticBranchBounds},
    ::NetworkModel{U},
) where {T <: PSY.ACTransmission, U <: PM.AbstractPowerModel}
    if get_use_slacks(device_model)
        throw(
            ArgumentError(
                "StaticBranchBounds formulation and $U is not compatible with the use of slacks",
            ),
        )
    end
    devices = get_available_components(device_model, sys)
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, StaticBranchBounds},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)
    branch_rate_bounds!(container, device_model, network_model)
    add_constraints!(
        container,
        FlowRateConstraintFromTo,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        FlowRateConstraintToFrom,
        devices,
        device_model,
        network_model,
    )
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, StaticBranchBounds},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ACTransmission}
    devices = get_available_components(device_model, sys)
    branch_rate_bounds!(container, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

################################### TwoTerminal HVDC Line Models ###################################
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLossless},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    if has_subnetworks(network_model)
        devices = get_available_components(device_model, sys)
        add_variables!(
            container,
            FlowActivePowerVariable,
            network_model,
            devices,
            HVDCTwoTerminalLossless(),
        )
        add_to_expression!(
            container,
            ActivePowerBalance,
            FlowActivePowerVariable,
            devices,
            device_model,
            network_model,
        )
        add_feedforward_arguments!(container, device_model, devices)
    end
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLossless},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    if has_subnetworks(network_model)
        devices =
            get_available_components(device_model, sys)
        add_constraints!(
            container,
            FlowRateConstraint,
            devices,
            device_model,
            network_model,
        )
        add_constraint_dual!(container, sys, device_model)
        add_feedforward_constraints!(container, device_model, devices)
    end
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalUnbounded},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{<:PSY.TwoTerminalHVDC, HVDCTwoTerminalUnbounded},
    ::NetworkModel{<:PM.AbstractPowerModel},
)
    devices = get_available_components(device_model, sys)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalUnbounded},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_variables!(container, FlowActivePowerVariable, devices, HVDCTwoTerminalUnbounded())
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_arguments!(container, devicemodel, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{<:PSY.TwoTerminalHVDC, HVDCTwoTerminalUnbounded},
    ::NetworkModel{CopperPlatePowerModel},
)
    devices = get_available_components(device_model, sys)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalDispatch},
    ::NetworkModel{AreaBalancePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    @warn "AreaBalancePowerModel doesn't model individual line flows for $T. Arguments not built"
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalDispatch},
    ::NetworkModel{AreaBalancePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    @warn "AreaBalancePowerModel doesn't model individual line flows for $T. Model not built"
    return
end

# Repeated method to avoid ambiguity between HVDCTwoTerminalUnbounded, HVDCTwoTerminalLossless and HVDCTwoTerminalDispatch
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalUnbounded},
    ::NetworkModel{AreaBalancePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    @warn "AreaBalancePowerModel doesn't model individual line flows for $T. Arguments not built"
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalUnbounded},
    ::NetworkModel{AreaBalancePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    @warn "AreaBalancePowerModel doesn't model individual line flows for $T. Model not built"
    return
end

# Repeated method to avoid ambiguity between HVDCTwoTerminalUnbounded, HVDCTwoTerminalLossless and HVDCTwoTerminalDispatch
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLossless},
    ::NetworkModel{AreaBalancePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    @warn "AreaBalancePowerModel doesn't model individual line flows for $T. Arguments not built"
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLossless},
    ::NetworkModel{AreaBalancePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    @warn "AreaBalancePowerModel doesn't model individual line flows for $T. Model not built"
    return
end

# Repeated method to avoid ambiguity between HVDCTwoTerminalUnbounded and HVDCTwoTerminalLossless
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalUnbounded},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_variables!(container, FlowActivePowerVariable, devices, HVDCTwoTerminalUnbounded())
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_arguments!(container, device_model, devices)
    return
end

# Repeated method to avoid ambiguity between HVDCTwoTerminalUnbounded and HVDCTwoTerminalLossless
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{<:PSY.TwoTerminalHVDC, HVDCTwoTerminalUnbounded},
    network_model::NetworkModel{<:AbstractPTDFModel},
)
    devices = get_available_components(device_model, sys)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLossless},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLossless},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_constraints!(container, FlowRateConstraint, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

# Repeated method to avoid ambiguity between HVDCTwoTerminalUnbounded and HVDCTwoTerminalLossless
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLossless},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_variables!(container, FlowActivePowerVariable, devices, HVDCTwoTerminalLossless())
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_arguments!(container, device_model, devices)
    return
end

# Repeated method to avoid ambiguity between HVDCTwoTerminalUnbounded and HVDCTwoTerminalLossless
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLossless},
    network_model::NetworkModel{PTDFPowerModel},
) where {
    T <: PSY.TwoTerminalHVDC,
}
    devices = get_available_components(device_model, sys)
    add_constraints!(container, FlowRateConstraint, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalDispatch},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_variables!(
        container,
        FlowActivePowerToFromVariable,
        devices,
        HVDCTwoTerminalDispatch(),
    )
    add_variables!(
        container,
        FlowActivePowerFromToVariable,
        devices,
        HVDCTwoTerminalDispatch(),
    )
    add_variables!(container, HVDCLosses, devices, HVDCTwoTerminalDispatch())
    add_variables!(container, HVDCFlowDirectionVariable, devices, HVDCTwoTerminalDispatch())
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerToFromVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerFromToVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        HVDCLosses,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalDispatch},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_constraints!(
        container,
        FlowRateConstraintFromTo,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        FlowRateConstraintToFrom,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, HVDCPowerBalance, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalDispatch},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_variables!(
        container,
        FlowActivePowerToFromVariable,
        devices,
        HVDCTwoTerminalDispatch(),
    )
    add_variables!(
        container,
        FlowActivePowerFromToVariable,
        devices,
        HVDCTwoTerminalDispatch(),
    )
    add_variables!(container, HVDCFlowDirectionVariable, devices, HVDCTwoTerminalDispatch())
    add_variables!(container, HVDCLosses, devices, HVDCTwoTerminalDispatch())
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerToFromVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerFromToVariable,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalDispatch},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    @warn "CopperPlatePowerModel models with HVDC ignores inter-area losses"
    add_constraints!(
        container,
        FlowRateConstraintFromTo,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        FlowRateConstraintToFrom,
        devices,
        device_model,
        network_model,
    )
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, U},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {
    T <: PSY.TwoTerminalHVDC,
    U <: HVDCTwoTerminalPiecewiseLoss,
}
    devices = get_available_components(device_model, sys)
    add_variables!(
        container,
        HVDCActivePowerReceivedFromVariable,
        devices,
        HVDCTwoTerminalPiecewiseLoss(),
    )
    add_variables!(
        container,
        HVDCActivePowerReceivedToVariable,
        devices,
        HVDCTwoTerminalPiecewiseLoss(),
    )
    _add_sparse_pwl_loss_variables!(container, devices, device_model)
    add_to_expression!(
        container,
        ActivePowerBalance,
        HVDCActivePowerReceivedFromVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        HVDCActivePowerReceivedToVariable,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, U},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {
    T <: PSY.TwoTerminalHVDC,
    U <: HVDCTwoTerminalPiecewiseLoss,
}
    devices = get_available_components(device_model, sys)
    add_constraints!(
        container,
        FlowRateConstraintFromTo,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        FlowRateConstraintToFrom,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCFlowCalculationConstraint,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_constraints!(container, device_model, devices)
    return
end

# TODO: Other models besides PTDF
#=
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::DeviceModel{T, HVDCTwoTerminalPiecewiseLoss},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices =
        get_available_components(model, sys)
    add_variables!(
        container,
        FlowActivePowerToFromVariable,
        devices,
        HVDCTwoTerminalDispatch(),
    )
    add_variables!(
        container,
        FlowActivePowerFromToVariable,
        devices,
        HVDCTwoTerminalDispatch(),
    )
    add_variables!(container, HVDCFlowDirectionVariable, devices, HVDCTwoTerminalDispatch())
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerToFromVariable,
        devices,
        model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerFromToVariable,
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
    model::DeviceModel{T, HVDCTwoTerminalPiecewiseLoss},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices =
        get_available_components(model, sys)
    @warn "CopperPlatePowerModel models with HVDC ignores inter-area losses"
    add_constraints!(container, FlowRateConstraintFromTo, devices, model, network_model)
    add_constraints!(container, FlowRateConstraintToFrom, devices, model, network_model)
    add_constraint_dual!(container, sys, model)
    return
end
=#

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalDispatch},
    network_model::NetworkModel{<:PM.AbstractActivePowerModel},
) where {T <: PSY.TwoTerminalHVDC}
    devices = get_available_components(device_model, sys)
    add_constraints!(
        container,
        FlowRateConstraintFromTo,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        FlowRateConstraintToFrom,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, HVDCPowerBalance, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

############################# NEW LCC HVDC NON-LINEAR MODEL #############################

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLCC},
    network_model::NetworkModel{<:PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    devices = get_available_components(device_model, sys)

    # Variables
    add_variables!(
        container,
        HVDCActivePowerReceivedFromVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCActivePowerReceivedToVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCReactivePowerReceivedFromVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCReactivePowerReceivedToVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCRectifierDelayAngleVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCInverterExtinctionAngleVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCRectifierPowerFactorAngleVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCInverterPowerFactorAngleVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCRectifierOverlapAngleVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCInverterOverlapAngleVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCRectifierDCVoltageVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCInverterDCVoltageVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCRectifierACCurrentVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCInverterACCurrentVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        DCLineCurrentFlowVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCRectifierTapSettingVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )
    add_variables!(
        container,
        HVDCInverterTapSettingVariable,
        devices,
        HVDCTwoTerminalLCC(),
    )

    # Expressions
    add_to_expression!(
        container,
        ActivePowerBalance,
        HVDCActivePowerReceivedFromVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        HVDCActivePowerReceivedToVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ReactivePowerBalance,
        HVDCReactivePowerReceivedFromVariable,
        devices,
        device_model,
        network_model,
    )
    add_to_expression!(
        container,
        ReactivePowerBalance,
        HVDCReactivePowerReceivedToVariable,
        devices,
        device_model,
        network_model,
    )

    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    ::OptimizationContainer,
    ::PSY.System,
    ::ArgumentConstructStage,
    ::DeviceModel{T, HVDCTwoTerminalLCC},
    ::NetworkModel{N},
) where {T <: PSY.TwoTerminalLCCLine, N <: PM.AbstractPowerModel}
    throw(
        ArgumentError(
            "HVDCTwoTerminalLCC formulation requires ACPPowerModel network. " *
            "Got $N. Use HVDCTwoTerminalLossless, HVDCTwoTerminalDispatch, or " *
            "HVDCTwoTerminalPiecewiseLoss for DC/PTDF networks.",
        ),
    )
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{T, HVDCTwoTerminalLCC},
    network_model::NetworkModel{<:PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    devices = get_available_components(device_model, sys)
    add_constraints!(
        container,
        HVDCRectifierDCLineVoltageConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCInverterDCLineVoltageConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCRectifierOverlapAngleConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCInverterOverlapAngleConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCRectifierPowerFactorAngleConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCInverterPowerFactorAngleConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCRectifierACCurrentFlowConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCInverterACCurrentFlowConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCRectifierPowerCalculationConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCInverterPowerCalculationConstraint,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(
        container,
        HVDCTransmissionDCLineConstraint,
        devices,
        device_model,
        network_model,
    )
    return
end

############################# Phase Shifter Transformer Models #############################

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{PSY.PhaseShiftingTransformer, PhaseAngleControl},
    network_model::NetworkModel{PM.DCPPowerModel},
)
    devices = get_available_components(device_model, sys)
    add_variables!(container, FlowActivePowerVariable, devices, PhaseAngleControl())
    add_variables!(container, PhaseShifterAngle, devices, PhaseAngleControl())
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{PSY.PhaseShiftingTransformer, PhaseAngleControl},
    network_model::NetworkModel{<:AbstractPTDFModel},
)
    devices = get_available_components(device_model, sys)
    add_variables!(container, FlowActivePowerVariable, devices, PhaseAngleControl())
    add_variables!(container, PhaseShifterAngle, devices, PhaseAngleControl())
    add_to_expression!(
        container,
        ActivePowerBalance,
        PhaseShifterAngle,
        devices,
        device_model,
        network_model,
    )
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.PhaseShiftingTransformer, PhaseAngleControl},
    network_model::NetworkModel{PM.DCPPowerModel},
)
    devices = get_available_components(device_model, sys)
    add_constraints!(container, FlowLimitConstraint, devices, device_model, network_model)
    add_constraints!(
        container,
        PhaseAngleControlLimit,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, NetworkFlowConstraint, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.PhaseShiftingTransformer, PhaseAngleControl},
    network_model::NetworkModel{<:AbstractPTDFModel},
)
    devices = get_available_components(device_model, sys)
    add_constraints!(container, FlowLimitConstraint, devices, device_model, network_model)
    add_constraints!(
        container,
        PhaseAngleControlLimit,
        devices,
        device_model,
        network_model,
    )
    add_constraints!(container, NetworkFlowConstraint, devices, device_model, network_model)
    add_constraint_dual!(container, sys, device_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

################################# AreaInterchange Models ################################
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{PSY.AreaInterchange, U},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {U <: Union{StaticBranchUnbounded, StaticBranch}}
    devices = get_available_components(device_model, sys)
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.AreaInterchange, StaticBranchUnbounded},
    network_model::NetworkModel{T},
) where {T <: PM.AbstractActivePowerModel}
    devices = get_available_components(device_model, sys)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    device_model::DeviceModel{PSY.AreaInterchange, T},
    network_model::NetworkModel{U},
) where {
    T <: Union{StaticBranchUnbounded, StaticBranch},
    U <: PM.AbstractActivePowerModel,
}
    devices = get_available_components(device_model, sys)
    has_ts = PSY.has_time_series.(devices)
    if get_use_slacks(device_model)
        add_variables!(
            container,
            FlowActivePowerSlackUpperBound,
            network_model,
            devices,
            T(),
        )
        add_variables!(
            container,
            FlowActivePowerSlackLowerBound,
            network_model,
            devices,
            T(),
        )
    end
    if any(has_ts) && !all(has_ts)
        error(
            "Not all AreaInterchange devices have time series. Check data to complete (or remove) time series.",
        )
    end
    add_variables!(
        container,
        FlowActivePowerVariable,
        network_model,
        devices,
        T(),
    )
    add_to_expression!(
        container,
        ActivePowerBalance,
        FlowActivePowerVariable,
        devices,
        device_model,
        network_model,
    )
    if all(has_ts)
        for device in devices
            name = PSY.get_name(device)
            num_ts = length(unique(PSY.get_name.(PSY.get_time_series_keys(device))))
            if num_ts < 2
                error(
                    "AreaInterchange $name has less than two time series. It is required to add both from_to and to_from time series.",
                )
            end
        end
        add_parameters!(container, FromToFlowLimitParameter, devices, device_model)
        add_parameters!(container, ToFromFlowLimitParameter, devices, device_model)
    end
    add_feedforward_arguments!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.AreaInterchange, StaticBranch},
    network_model::NetworkModel{T},
) where {T <: PM.AbstractActivePowerModel}
    devices = get_available_components(device_model, sys)
    add_constraints!(container, FlowLimitConstraint, devices, device_model, network_model)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function _get_branch_map(network_model::NetworkModel)
    @assert !isempty(network_model.modeled_ac_branch_types)
    net_reduction_data = get_network_reduction(network_model)
    all_branch_maps_by_type = net_reduction_data.all_branch_maps_by_type
    inter_area_branch_map =
    # This method uses ACBranch to support HVDC
        Dict{Tuple{String, String}, Dict{DataType, Vector{String}}}()
    name_to_arc_maps = PNM.get_name_to_arc_maps(net_reduction_data)
    for br_type in network_model.modeled_ac_branch_types
        !haskey(name_to_arc_maps, br_type) && continue
        name_to_arc_map = PNM.get_name_to_arc_map(net_reduction_data, br_type)
        for (name, (arc, reduction)) in name_to_arc_map
            reduction_entry = all_branch_maps_by_type[reduction][br_type][arc]
            area_from, area_to = _get_area_from_to(reduction_entry)
            if area_from != area_to
                branch_typed_dict = get!(
                    inter_area_branch_map,
                    (PSY.get_name(area_from), PSY.get_name(area_to)),
                    Dict{DataType, Vector{String}}(),
                )
                _add_to_branch_map!(branch_typed_dict, reduction_entry, name)
            end
        end
    end
    return inter_area_branch_map
end

function _add_to_branch_map!(
    branch_typed_dict::Dict{DataType, Vector{String}},
    ::T,
    name::String,
) where {T <: PSY.ACBranch}
    if !haskey(branch_typed_dict, T)
        branch_typed_dict[T] = [name]
    else
        push!(branch_typed_dict[T], name)
    end
end

function _add_to_branch_map!(
    branch_typed_dict::Dict{DataType, Vector{String}},
    reduction_entry::Union{PNM.BranchesParallel, PNM.BranchesSeries},
    name::String,
)
    _add_to_branch_map!(branch_typed_dict, first(reduction_entry), name)
end

# This method uses ACBranch to support 2T - HVDC
function _get_area_from_to(reduction_entry::PSY.ACBranch)
    area_from = PSY.get_area(PSY.get_arc(reduction_entry).from)
    area_to = PSY.get_area(PSY.get_arc(reduction_entry).to)
    return area_from, area_to
end

function _get_area_from_to(reduction_entry::PNM.ThreeWindingTransformerWinding)
    tfw = PNM.get_transformer(reduction_entry)
    winding_int = PNM.get_winding_number(reduction_entry)
    if winding_int == 1
        area_from = PSY.get_area(PSY.get_primary_star_arc(tfw).from)
        area_to = PSY.get_area(PSY.get_primary_star_arc(tfw).to)
    elseif winding_int == 2
        area_from = PSY.get_area(PSY.get_secondary_star_arc(tfw).from)
        area_to = PSY.get_area(PSY.get_secondary_star_arc(tfw).to_index)
    elseif winding_int == 3
        area_from = PSY.get_area(PSY.get_tertiary_star_arc(tfw).from)
        area_to = PSY.get_area(PSY.get_tertiary_star_arc(tfw).to)
    else
        @assert false "Winding number $winding_int is not valid for three-winding transformer"
    end
    return area_from, area_to
end

function _get_area_from_to(reduction_entry::PNM.BranchesParallel)
    return _get_area_from_to(first(reduction_entry))
end

function _get_area_from_to(reduction_entry::PNM.BranchesSeries)
    area_froms = [_get_area_from_to(x)[1] for x in reduction_entry]
    area_tos = [_get_area_from_to(x)[2] for x in reduction_entry]
    all_areas = vcat(area_froms, area_tos)
    if length(unique(all_areas)) > 1
        error(
            "Inter-area line found as part of a degree two chain reduction; this feature is not supported",
        )
    end
    return first(all_areas), first(all_areas)
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.AreaInterchange, StaticBranch},
    network_model::NetworkModel{T},
) where {T <: PSI.AbstractPTDFModel}
    devices = get_available_components(device_model, sys)
    add_constraints!(container, FlowLimitConstraint, devices, device_model, network_model)
    # Not ideal to do this here, but it is a not terrible workaround
    # The area interchanges are like a services/device mix.
    # Doesn't include the possibility of Multi-terminal HVDC
    inter_area_branch_map = _get_branch_map(network_model)

    add_constraints!(
        container,
        LineFlowBoundConstraint,
        devices,
        device_model,
        network_model,
        inter_area_branch_map,
    )
    add_feedforward_constraints!(container, device_model, devices)
    return
end

function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.AreaInterchange, StaticBranchUnbounded},
    network_model::NetworkModel{AreaBalancePowerModel},
)
    devices = get_available_components(device_model, sys)
    add_feedforward_constraints!(container, device_model, devices)
    return
end

#TODO Check if for SCUC AreaPTDF needs something else
function construct_device!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    device_model::DeviceModel{PSY.AreaInterchange, StaticBranchUnbounded},
    network_model::NetworkModel{AreaPTDFPowerModel},
)
    devices = get_available_components(device_model, sys)
    inter_area_branch_map = _get_branch_map(network_model)
    # Not ideal to do this here, but it is a not terrible workaround
    # The area interchanges are like a services/device mix.
    # Doesn't include the possibility of Multi-terminal HVDC
    add_constraints!(
        container,
        LineFlowBoundConstraint,
        devices,
        device_model,
        network_model,
        inter_area_branch_map,
    )
    add_feedforward_constraints!(container, device_model, devices)
    return
end
