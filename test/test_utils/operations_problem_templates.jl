const NETWORKS_FOR_TESTING = [
    (CopperPlateNetworkModel, HiGHS_optimizer),
    (PTDFNetworkModel, HiGHS_optimizer),
]

function get_thermal_standard_uc_template()
    template = PowerOperationsProblemTemplate(CopperPlateNetworkModel)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
    return template
end

function get_thermal_dispatch_template_network(network = CopperPlateNetworkModel)
    template = PowerOperationsProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, MonitoredLine, StaticBranchBounds)
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, TwoWindingTransformer, StaticBranch)
    set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)
    return template
end

function get_template_basic_uc_simulation()
    template = PowerOperationsProblemTemplate(CopperPlateNetworkModel)
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, InterruptiblePowerLoad, StaticPowerLoad)
    set_device_model!(template, HydroTurbine, HydroTurbineEnergyDispatch)
    set_device_model!(template, HydroReservoir, HydroEnergyModelReservoir)
    return template
end

function get_template_standard_uc_simulation()
    template = get_template_basic_uc_simulation()
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
    return template
end

function get_template_nomin_ed_simulation(network = CopperPlateNetworkModel)
    template = PowerOperationsProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, InterruptiblePowerLoad, PowerLoadDispatch)
    set_device_model!(template, HydroTurbine, HydroTurbineEnergyDispatch)
    set_device_model!(template, HydroReservoir, HydroEnergyModelReservoir)
    return template
end

function get_template_hydro_st_uc(network = CopperPlateNetworkModel)
    template = PowerOperationsProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment),
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch),
    set_device_model!(template, PowerLoad, StaticPowerLoad),
    set_device_model!(template, InterruptiblePowerLoad, PowerLoadDispatch),
    set_device_model!(template, HydroTurbine, HydroTurbineEnergyDispatch)
    set_device_model!(template, HydroReservoir, HydroEnergyModelReservoir)
    return template
end

function get_template_hydro_st_ed(network = CopperPlateNetworkModel, duals = [])
    template = PowerOperationsProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, InterruptiblePowerLoad, PowerLoadDispatch)
    set_device_model!(template, HydroTurbine, HydroTurbineEnergyDispatch)
    set_device_model!(template, HydroReservoir, HydroEnergyModelReservoir)
    return template
end

function get_template_dispatch_with_network(network = PTDFNetworkModel)
    template = PowerOperationsProblemTemplate(network)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, TwoWindingTransformer, StaticBranchBounds)
    set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)
    return template
end
