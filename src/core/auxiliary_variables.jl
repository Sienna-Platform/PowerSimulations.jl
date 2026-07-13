"""
Auxiliary Variable for Thermal Generation Models to keep track of time elapsed on
"""
struct TimeDurationOn <: AuxVariableType end

"""
Auxiliary Variable for Thermal Generation Models to keep track of time elapsed off
"""
struct TimeDurationOff <: AuxVariableType end

"""
Auxiliary Variable for Thermal Generation Models that solve for power above min
"""
struct PowerOutput <: AuxVariableType end

"""
Auxiliary Variable of DC Current Variables for DC Lines formulations
Docs abbreviation: ``p_l^{loss}``
"""
struct DCLineLosses <: AuxVariableType end

"""
Auxiliary Variables that are calculated using a `PowerFlowEvaluationModel`
"""
abstract type PowerFlowAuxVariableType <: AuxVariableType end

"""
Auxiliary Variable for the bus angle results from power flow evaluation
"""
struct PowerFlowVoltageAngle <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the bus voltage magnitued results from power flow evaluation
"""
struct PowerFlowVoltageMagnitude <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for line power flow results (directional flows and losses) from power flow
evaluation. Every branch-indexed aux variable is a subtype; the generic arc-flow
`calculate_aux_variable_value!` method dispatches on this.
"""
abstract type BranchFlowAuxVariableType <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the line reactive flow in the from -> to direction from power flow evaluation
"""
struct PowerFlowBranchReactivePowerFromTo <: BranchFlowAuxVariableType end

"""
Auxiliary Variable for the line reactive flow in the to -> from direction from power flow evaluation
"""
struct PowerFlowBranchReactivePowerToFrom <: BranchFlowAuxVariableType end

"""
Auxiliary Variable for the line active flow in the from -> to direction from power flow evaluation
"""
struct PowerFlowBranchActivePowerFromTo <: BranchFlowAuxVariableType end

"""
Auxiliary Variable for the line active flow in the to -> from direction from power flow evaluation
"""
struct PowerFlowBranchActivePowerToFrom <: BranchFlowAuxVariableType end

"""
Auxiliary Variable for the loss factors from AC power flow evaluation that are calculated using the Jacobian matrix
"""
struct PowerFlowLossFactors <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the voltage stability factors from AC power flow evaluation that are calculated using the Jacobian matrix
"""
struct PowerFlowVoltageStabilityFactors <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the active power loss on a line from AC power flow evaluation.
"""
struct PowerFlowBranchActivePowerLoss <: BranchFlowAuxVariableType end

# TODO reactive loss?

"""
Auxiliary Variable for the solved tap ratio (unitless) of each `TapTransformer` enrolled in
power-flow discrete control, per power-flow time step.
"""
struct PowerFlowTapRatio <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the solved susceptance (p.u. admittance) of each `SwitchedAdmittance`
enrolled in power-flow discrete control, per power-flow time step.
"""
struct PowerFlowSwitchedShuntSusceptance <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the reactive power (p.u.) delivered by each `FACTSControlDevice` in
power-flow discrete control, per power-flow time step.
"""
struct PowerFlowFACTSReactivePower <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the net HVDC active power (p.u.) at each bus as used by the power flow
evaluation, per time step. Registered only when the system carries HVDC components.

Indexed by **bus**, not by HVDC component — which is why it is not a
`PowerFlowHVDCAuxVariableType` despite the name. It is registered through `bus_aux_vars`
on an `(ACBus, bus_number)` axis and filled from `PowerFlows.get_bus_hvdc_net_power`, whereas
every `PowerFlowHVDCAuxVariableType` is per-component and read from `PowerFlows.get_hvdc_results`.
"""
struct PowerFlowHVDCNetPower <: PowerFlowAuxVariableType end

"""
Parent of the per-component HVDC quantities solved by the AC power flow (VSC/LCC lines, MTDC
converters and DC branches), read from `PowerFlows.get_hvdc_results`.
"""
abstract type PowerFlowHVDCAuxVariableType <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the active power (p.u.) entering an HVDC line at its from terminal, from
the AC power flow solution.
"""
struct PowerFlowHVDCActivePowerFromTo <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the active power (p.u.) entering an HVDC line at its to terminal, from
the AC power flow solution.
"""
struct PowerFlowHVDCActivePowerToFrom <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the reactive power (p.u.) at an HVDC line's from terminal, from the AC
power flow solution.
"""
struct PowerFlowHVDCReactivePowerFromTo <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the reactive power (p.u.) at an HVDC line's to terminal, from the AC
power flow solution.
"""
struct PowerFlowHVDCReactivePowerToFrom <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the total active power loss (p.u.; converters plus DC line) of an HVDC
line or DC branch, from the AC power flow solution.
"""
struct PowerFlowHVDCActivePowerLoss <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the DC current (p.u.) of a VSC HVDC line or MTDC DC branch, from the AC
power flow solution.
"""
struct PowerFlowHVDCDCCurrent <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the DC-side voltage (p.u.) at a VSC HVDC line's from converter, from the
AC power flow solution.
"""
struct PowerFlowHVDCDCVoltageFrom <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the DC-side voltage (p.u.) at a VSC HVDC line's to converter, from the
AC power flow solution.
"""
struct PowerFlowHVDCDCVoltageTo <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the solved rectifier transformer tap (p.u.) of an LCC HVDC line.
"""
struct PowerFlowLCCRectifierTap <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the solved inverter transformer tap (p.u.) of an LCC HVDC line.
"""
struct PowerFlowLCCInverterTap <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the solved rectifier delay (firing) angle (rad) of an LCC HVDC line.
"""
struct PowerFlowLCCRectifierDelayAngle <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the solved inverter extinction angle (rad) of an LCC HVDC line.
"""
struct PowerFlowLCCInverterExtinctionAngle <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the DC-side power (p.u.) drawn by an MTDC `InterconnectingConverter`
(AC power plus converter loss), from the AC power flow solution.
"""
struct PowerFlowConverterDCPower <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the reactive power (p.u.) of an MTDC `InterconnectingConverter`, from
the AC power flow solution.
"""
struct PowerFlowConverterReactivePower <: PowerFlowHVDCAuxVariableType end

"""
Auxiliary Variable for the DC node voltage (p.u.) at an MTDC `InterconnectingConverter`, from
the AC power flow solution.
"""
struct PowerFlowConverterDCVoltage <: PowerFlowHVDCAuxVariableType end

convert_result_to_natural_units(::Type{PowerOutput}) = true
# Every branch flow quantity is a power; the HVDC group is NOT uniform (taps, angles,
# DC currents and DC voltages are not powers), so those stay enumerated below.
convert_result_to_natural_units(::Type{<:BranchFlowAuxVariableType}) = true
convert_result_to_natural_units(::Type{PowerFlowFACTSReactivePower}) = true
convert_result_to_natural_units(::Type{PowerFlowHVDCNetPower}) = true
convert_result_to_natural_units(::Type{PowerFlowHVDCActivePowerFromTo}) = true
convert_result_to_natural_units(::Type{PowerFlowHVDCActivePowerToFrom}) = true
convert_result_to_natural_units(::Type{PowerFlowHVDCReactivePowerFromTo}) = true
convert_result_to_natural_units(::Type{PowerFlowHVDCReactivePowerToFrom}) = true
convert_result_to_natural_units(::Type{PowerFlowHVDCActivePowerLoss}) = true
convert_result_to_natural_units(::Type{PowerFlowConverterDCPower}) = true
convert_result_to_natural_units(::Type{PowerFlowConverterReactivePower}) = true

"Whether the auxiliary variable is calculated using a `PowerFlowEvaluationModel`"
is_from_power_flow(::Type{<:AuxVariableType}) = false
is_from_power_flow(::Type{<:PowerFlowAuxVariableType}) = true
