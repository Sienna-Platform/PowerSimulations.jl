# Unit test for the PSI->PowerFlows injection-sign resolver (`pf_contribution`). This is the
# single source of truth for how a mapped optimization value is written into the power-flow
# state; the sign must equal the OPF nodal-balance multiplier. The table below pins every
# (category, entry, component) mapping in use so a new device cannot silently desync the path.
@testset "pf_contribution resolver table" begin
    P = PSI.PFContribution
    FAPV = PSI.FlowActivePowerVariable
    FToF = PSI.FlowActivePowerToFromVariable

    # --- generic injectors / loads (variable entries) ---
    @test PSI.pf_contribution(
        Val(:active_power),
        PSI.ActivePowerVariable,
        PSY.ThermalStandard,
    ) ==
          P(:active, :injection, 1.0, false)
    @test PSI.pf_contribution(Val(:active_power), PSI.ActivePowerVariable, PSY.PowerLoad) ==
          P(:active, :withdrawal, -1.0, false)
    @test PSI.pf_contribution(
        Val(:active_power_out),
        PSI.ActivePowerOutVariable,
        PSY.ThermalStandard,
    ) ==
          P(:active, :injection, 1.0, true)
    @test PSI.pf_contribution(
        Val(:active_power_in),
        PSI.ActivePowerInVariable,
        PSY.ThermalStandard,
    ) ==
          P(:active, :injection, -1.0, true)
    @test PSI.pf_contribution(
        Val(:reactive_power),
        PSI.ReactivePowerVariable,
        PSY.ThermalStandard,
    ) ==
          P(:reactive, :injection, 1.0, false)
    @test PSI.pf_contribution(
        Val(:reactive_power),
        PSI.ReactivePowerVariable,
        PSY.PowerLoad,
    ) ==
          P(:reactive, :withdrawal, -1.0, false)

    # --- voltages assign (no direction) ---
    @test PSI.pf_contribution(Val(:voltage_angle_opf), PSI.VoltageAngle, PSY.ACBus) ==
          P(:angle, :voltage, 1.0, false)
    @test PSI.pf_contribution(
        Val(:voltage_magnitude_opf),
        PSI.VoltageMagnitude,
        PSY.ACBus,
    ) ==
          P(:magnitude, :voltage, 1.0, false)

    # --- parameters are pre-signed: in/out collapse to +1 (the #1631 fix) ---
    @test PSI.pf_contribution(
        Val(:active_power_in),
        PSI.ActivePowerInTimeSeriesParameter,
        PSY.ThermalStandard,
    ) ==
          P(:active, :injection, 1.0, true)
    @test PSI.pf_contribution(
        Val(:active_power),
        PSI.ActivePowerTimeSeriesParameter,
        PSY.PowerLoad,
    ) ==
          P(:active, :withdrawal, -1.0, false)

    # --- HVDC re-targets to :hvdc_net (PowerFlows' bus_hvdc_net_power); same signs as injection.
    #     single var to_from is +1, directional to_from is -1, from_to is -1 ---
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_to_from),
        FAPV,
        PSY.TwoTerminalGenericHVDCLine,
    ) ==
          P(:active, :hvdc_net, 1.0, false)
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_to_from),
        FToF,
        PSY.TwoTerminalGenericHVDCLine,
    ) ==
          P(:active, :hvdc_net, -1.0, false)
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_from_to),
        FAPV,
        PSY.TwoTerminalGenericHVDCLine,
    ) ==
          P(:active, :hvdc_net, -1.0, false)

    # --- PhaseShiftingTransformer: from_to -1, to_from +1 ---
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_to_from),
        FAPV,
        PSY.PhaseShiftingTransformer,
    ) ==
          P(:active, :injection, 1.0, false)
    @test PSI.pf_contribution(
        Val(:active_power_hvdc_pst_from_to),
        FAPV,
        PSY.PhaseShiftingTransformer,
    ) ==
          P(:active, :injection, -1.0, false)
end
