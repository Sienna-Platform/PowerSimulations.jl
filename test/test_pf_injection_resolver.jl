# Pins every (category, entry, component) -> PFContribution mapping so a new device can't
# silently desync the OPF->PowerFlows injection-sign path.
@testset "pf_contribution resolver table" begin
    P = PSI.PFContribution
    FAPV = PSI.FlowActivePowerVariable
    FToF = PSI.FlowActivePowerToFromVariable
    # singleton quantity/role tags (replace the former Symbol fields)
    ACT, REAC = PSI.PFActiveQuantity(), PSI.PFReactiveQuantity()
    ANG, MAG = PSI.PFAngleQuantity(), PSI.PFMagnitudeQuantity()
    INJ, WD, HVDC, NONE =
        PSI.PFInjectionRole(), PSI.PFWithdrawalRole(), PSI.PFHVDCNetRole(), PSI.PFNoRole()

    # --- generic injectors / loads (variable entries) ---
    @test PSI.pf_contribution(
        Val(:active_power),
        PSI.ActivePowerVariable,
        PSY.ThermalStandard,
    ) ==
          P(ACT, INJ, 1.0, false)
    @test PSI.pf_contribution(Val(:active_power), PSI.ActivePowerVariable, PSY.PowerLoad) ==
          P(ACT, WD, -1.0, false)
    @test PSI.pf_contribution(
        Val(:active_power_out),
        PSI.ActivePowerOutVariable,
        PSY.ThermalStandard,
    ) ==
          P(ACT, INJ, 1.0, true)
    @test PSI.pf_contribution(
        Val(:active_power_in),
        PSI.ActivePowerInVariable,
        PSY.ThermalStandard,
    ) ==
          P(ACT, INJ, -1.0, true)
    @test PSI.pf_contribution(
        Val(:reactive_power),
        PSI.ReactivePowerVariable,
        PSY.ThermalStandard,
    ) ==
          P(REAC, INJ, 1.0, false)
    @test PSI.pf_contribution(
        Val(:reactive_power),
        PSI.ReactivePowerVariable,
        PSY.PowerLoad,
    ) ==
          P(REAC, WD, -1.0, false)

    # --- voltages assign (no direction) ---
    @test PSI.pf_contribution(Val(:voltage_angle_opf), PSI.VoltageAngle, PSY.ACBus) ==
          P(ANG, NONE, 1.0, false)
    @test PSI.pf_contribution(
        Val(:voltage_magnitude_opf),
        PSI.VoltageMagnitude,
        PSY.ACBus,
    ) ==
          P(MAG, NONE, 1.0, false)

    # --- parameters are pre-signed: in/out collapse to +1 (the #1631 fix) ---
    @test PSI.pf_contribution(
        Val(:active_power_in),
        PSI.ActivePowerInTimeSeriesParameter,
        PSY.ThermalStandard,
    ) ==
          P(ACT, INJ, 1.0, true)
    @test PSI.pf_contribution(
        Val(:active_power),
        PSI.ActivePowerTimeSeriesParameter,
        PSY.PowerLoad,
    ) ==
          P(ACT, WD, -1.0, false)

    # --- HVDC re-targets to :hvdc_net (PowerFlows' bus_hvdc_net_power); same signs as injection.
    #     single var to_from is +1, directional to_from is -1, from_to is -1 ---
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_to_from),
        FAPV,
        PSY.TwoTerminalGenericHVDCLine,
    ) ==
          P(ACT, HVDC, 1.0, false)
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_to_from),
        FToF,
        PSY.TwoTerminalGenericHVDCLine,
    ) ==
          P(ACT, HVDC, -1.0, false)
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_from_to),
        FAPV,
        PSY.TwoTerminalGenericHVDCLine,
    ) ==
          P(ACT, HVDC, -1.0, false)

    # --- PhaseShiftingTransformer: from_to -1, to_from +1 ---
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_to_from),
        FAPV,
        PSY.PhaseShiftingTransformer,
    ) ==
          P(ACT, INJ, 1.0, false)
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_from_to),
        FAPV,
        PSY.PhaseShiftingTransformer,
    ) ==
          P(ACT, INJ, -1.0, false)
end
