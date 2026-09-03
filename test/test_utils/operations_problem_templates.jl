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

# Test fixtures mirroring PSI's now-removed `template_unit_commitment`/
# `template_economic_dispatch` defaults; not a supported API.
function test_template_unit_commitment(network = CopperPlateNetworkModel)
    template = PowerOperationsProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, RenewableNonDispatch, FixedOutput)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, InterruptiblePowerLoad, PowerLoadInterruption)
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, TwoWindingTransformer, StaticBranch)
    set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalDispatch)
    set_service_model!(template, OnlineReserve{ReserveUp}, RangeReserve)
    set_service_model!(template, OnlineReserve{ReserveDown}, RangeReserve)
    return template
end

function test_template_economic_dispatch(network = CopperPlateNetworkModel)
    template = test_template_unit_commitment(network)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    return template
end
