const NETWORKS_FOR_TESTING = [
    (PM.ACPPowerModel, fast_ipopt_optimizer),
    (PM.ACRPowerModel, fast_ipopt_optimizer),
    (PM.ACTPowerModel, fast_ipopt_optimizer),
    #(PM.IVRPowerModel, fast_ipopt_optimizer), #instantiate_ivp_expr_model not implemented
    (PM.DCPPowerModel, fast_ipopt_optimizer),
    (PM.DCMPPowerModel, fast_ipopt_optimizer),
    (PM.NFAPowerModel, fast_ipopt_optimizer),
    (PM.DCPLLPowerModel, fast_ipopt_optimizer),
    (PM.LPACCPowerModel, fast_ipopt_optimizer),
    (PM.SOCWRPowerModel, fast_ipopt_optimizer),
    (PM.SOCWRConicPowerModel, scs_solver),
    (PM.QCRMPowerModel, fast_ipopt_optimizer),
    (PM.QCLSPowerModel, fast_ipopt_optimizer),
    #(PM.SOCBFPowerModel, fast_ipopt_optimizer), # not implemented
    (PM.BFAPowerModel, fast_ipopt_optimizer),
    #(PM.SOCBFConicPowerModel, fast_ipopt_optimizer), # not implemented
    (PM.SDPWRMPowerModel, scs_solver),
]

function get_thermal_standard_uc_template()
    template = ProblemTemplate(CopperPlatePowerModel)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
    return template
end

function get_thermal_dispatch_template_network(network = CopperPlatePowerModel)
    template = ProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, MonitoredLine, StaticBranchBounds)
    set_device_model!(
        template,
        DeviceModel(Line, StaticBranch; attributes = PARALLEL_RATING),
    )
    set_device_model!(template, Transformer2W, StaticBranch)
    set_device_model!(template, TapTransformer, StaticBranch)
    set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)
    return template
end

function get_template_basic_uc_simulation()
    template = ProblemTemplate(CopperPlatePowerModel)
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

function get_template_nomin_ed_simulation(network = CopperPlatePowerModel)
    template = ProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, InterruptiblePowerLoad, PowerLoadDispatch)
    set_device_model!(template, HydroTurbine, HydroTurbineEnergyDispatch)
    set_device_model!(template, HydroReservoir, HydroEnergyModelReservoir)
    return template
end

function get_template_hydro_st_uc(network = CopperPlatePowerModel)
    template = ProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment),
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch),
    set_device_model!(template, PowerLoad, StaticPowerLoad),
    set_device_model!(template, InterruptiblePowerLoad, PowerLoadDispatch),
    set_device_model!(template, HydroTurbine, HydroTurbineEnergyDispatch)
    set_device_model!(template, HydroReservoir, HydroEnergyModelReservoir)
    return template
end

function get_template_hydro_st_ed(network = CopperPlatePowerModel, duals = [])
    template = ProblemTemplate(network)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, InterruptiblePowerLoad, PowerLoadDispatch)
    set_device_model!(template, HydroTurbine, HydroTurbineEnergyDispatch)
    set_device_model!(template, HydroReservoir, HydroEnergyModelReservoir)
    return template
end

function get_template_dispatch_with_network(network = PTDFPowerModel)
    template = ProblemTemplate(network)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(
        template,
        DeviceModel(Line, StaticBranch; attributes = PARALLEL_RATING),
    )
    set_device_model!(template, Transformer2W, StaticBranchBounds)
    set_device_model!(template, TapTransformer, StaticBranchBounds)
    set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)
    return template
end
