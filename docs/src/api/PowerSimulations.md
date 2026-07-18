```@meta
CurrentModule = PowerSimulations
DocTestSetup  = quote
    using PowerSimulations
end
```

# API Reference

```@contents
Pages = ["PowerSimulations.md"]
Depth = 3
```

```@raw html
&nbsp;
&nbsp;
```

## Device Models

List of structures and methods for Device models

```@docs
DeviceModel
```

### Formulations

Refer to the [Formulations Page](@ref formulation_library) for each Abstract Device Formulation.

HVDC formulations will be moved to its own section in future releases

### HVDC Formulations

```@docs
TransportHVDCNetworkModel
VoltageDispatchHVDCNetworkModel
HVDCTwoTerminalLCC
```

### Security-Constrained Branch Formulations

```@docs
SecurityConstrainedStaticBranch
```

### Converter Formulations

```@docs
QuadraticLossConverter
```

### DC Lines Formulations

```@docs
DCLossyLine
```

### Synchronous Condenser Formulations

```@docs
SynchronousCondenserBasicDispatch
```

### Problem Templates

```@autodocs
Modules = [PowerSimulations]
Pages   = ["problem_template.jl",
            "operation_problem_templates.jl",
           ]
Order = [:type, :function]
Public = true
Private = false
```

```@raw html
&nbsp;
&nbsp;
```

* * *

## Decision Models

```@autodocs
Modules = [PowerSimulations]
Pages   = ["decision_model.jl",
           ]
Order = [:type, :function]
Public = true
Private = false
```

```@raw html
&nbsp;
```

```@docs
GenericOpProblem
```

```@raw html
&nbsp;
&nbsp;
```

* * *

## Emulation Models

```@docs
EmulationModel
EmulationModel(::Type{M} where {M <: EmulationProblem}, ::ProblemTemplate, ::PSY.System, ::Union{Nothing, JuMP.Model})
build!(::EmulationModel)
run!(::EmulationModel)
solve!(::Int, ::EmulationModel{<:EmulationProblem}, ::Dates.DateTime, ::SimulationStore)
```

```@raw html
&nbsp;
&nbsp;
```

* * *

## Service Models

List of structures and methods for Service models

```@docs
ServiceModel
```

```@raw html
&nbsp;
&nbsp;
```

* * *

## Simulation Models

```@docs
InitialCondition
SimulationModels
SimulationSequence
Simulation
build!(::Simulation)
execute!(::Simulation)
```

```@autodocs
Modules = [PowerSimulations]
Pages   = ["simulation_partitions.jl",
           ]
Order = [:type, :function]
Public = true
Private = false
```

```@raw html
&nbsp;
&nbsp;
```

## Chronology Models

```@autodocs
Modules = [PowerSimulations]
Pages   = ["initial_condition_chronologies.jl",
           ]
Order = [:type, :function]
Public = true
Private = false
```

* * *

## Variables

For a list of variables for each device refer to its Formulations page.

### Common Variables

```@docs
ActivePowerVariable
ReactivePowerVariable
PiecewiseLinearCostVariable
RateofChangeConstraintSlackUp
RateofChangeConstraintSlackDown
PostContingencyActivePowerChangeVariable
```

### Thermal Unit Variables

```@docs
OnVariable
StartVariable
StopVariable
HotStartVariable
WarmStartVariable
ColdStartVariable
PowerAboveMinimumVariable
```

### Storage Unit Variables

```@docs
ReservationVariable
EnergyVariable
ActivePowerOutVariable
ActivePowerInVariable
```

### Load Variables

```@docs
ShiftUpActivePowerVariable
ShiftDownActivePowerVariable
```

### Branches and Network Variables

```@docs
FlowActivePowerVariable
FlowActivePowerSlackUpperBound
FlowActivePowerSlackLowerBound
PostContingencyFlowActivePowerSlackUpperBound
PostContingencyFlowActivePowerSlackLowerBound
FlowActivePowerFromToVariable
FlowActivePowerToFromVariable
FlowReactivePowerFromToVariable
FlowReactivePowerToFromVariable
PhaseShifterAngle
HVDCLosses
HVDCFlowDirectionVariable
VoltageMagnitude
VoltageAngle
```

### Two Terminal and Multi-Terminal HVDC Variables

```@docs
InterpolationBinarySquaredCurrentVariable
SquaredDCVoltage
DCLineCurrent
InterpolationSquaredVoltageVariable
InterpolationBinarySquaredVoltageVariable
AuxBilinearConverterVariable
AuxBilinearSquaredConverterVariable
InterpolationSquaredBilinearVariable
InterpolationBinarySquaredBilinearVariable
InterpolationSquaredCurrentVariable
DCVoltage
ConverterCurrent
SquaredConverterCurrent
ConverterPositiveCurrent
ConverterNegativeCurrent
ConverterPowerDirection
```

### Services Variables

```@docs
ActivePowerReserveVariable
ServiceRequirementVariable
SystemBalanceSlackUp
SystemBalanceSlackDown
ReserveRequirementSlack
InterfaceFlowSlackUp
InterfaceFlowSlackDown
PostContingencyActivePowerReserveDeploymentVariable
```

### Feedforward Variables

```@docs
UpperBoundFeedForwardSlack
LowerBoundFeedForwardSlack
```

```@raw html
&nbsp;
&nbsp;
```

* * *

## Auxiliary Variables

Auxiliary variables recovered from a power flow evaluation all descend from
`PowerFlowAuxVariableType`. The tree below groups them by the **axis they are indexed on**,
which is what determines how each one is registered and filled — note that
`PowerFlowHVDCNetPower` is a *bus* quantity despite its name, while the
`PowerFlowHVDCAuxVariableType` group is per *HVDC component*:

```
PowerFlowAuxVariableType
├── bus-indexed  (ACBus, bus_number)
│   ├── PowerFlowVoltageAngle
│   ├── PowerFlowVoltageMagnitude
│   ├── PowerFlowLossFactors                  (only if calculate_loss_factors)
│   ├── PowerFlowVoltageStabilityFactors      (only if calculate_voltage_stability_factors)
│   └── PowerFlowHVDCNetPower                 (only if the system carries HVDC)
│
├── BranchFlowAuxVariableType                 branch-indexed (branch type, name)
│   ├── PowerFlowBranchActivePowerFromTo
│   ├── PowerFlowBranchActivePowerToFrom
│   ├── PowerFlowBranchReactivePowerFromTo
│   ├── PowerFlowBranchReactivePowerToFrom
│   └── PowerFlowBranchActivePowerLoss
│
├── device-control  (device type, name)       enrolled in power-flow discrete control
│   ├── PowerFlowTapRatio                     TapTransformer
│   ├── PowerFlowSwitchedShuntSusceptance     SwitchedAdmittance
│   └── PowerFlowFACTSReactivePower           FACTSControlDevice
│
└── PowerFlowHVDCAuxVariableType              HVDC-component-indexed (component type, name)
    ├── PowerFlowHVDCActivePowerFromTo        LCC, VSC
    ├── PowerFlowHVDCActivePowerToFrom        LCC, VSC
    ├── PowerFlowHVDCReactivePowerFromTo      LCC, VSC
    ├── PowerFlowHVDCReactivePowerToFrom      LCC, VSC
    ├── PowerFlowHVDCActivePowerLoss          LCC, VSC, TModelHVDCLine
    ├── PowerFlowHVDCDCCurrent                VSC, TModelHVDCLine
    ├── PowerFlowHVDCDCVoltageFrom            VSC
    ├── PowerFlowHVDCDCVoltageTo              VSC
    ├── PowerFlowLCCRectifierTap              LCC
    ├── PowerFlowLCCInverterTap               LCC
    ├── PowerFlowLCCRectifierDelayAngle       LCC
    ├── PowerFlowLCCInverterExtinctionAngle   LCC
    ├── PowerFlowConverterDCPower             InterconnectingConverter
    ├── PowerFlowConverterReactivePower       InterconnectingConverter
    └── PowerFlowConverterDCVoltage           InterconnectingConverter
```

Every branch flow quantity is a power and is exported in natural units. The HVDC group is
**not** uniform: the power quantities convert to natural units, while the taps, delay/extinction
angles, DC currents and DC voltages are per-unit or radian quantities and are exported as-is.

### Thermal Unit Auxiliary Variables

```@docs
TimeDurationOn
TimeDurationOff
PowerOutput
```

### Bus Auxiliary Variables

```@docs
PowerFlowVoltageAngle
PowerFlowVoltageMagnitude
PowerFlowLossFactors
PowerFlowVoltageStabilityFactors
PowerFlowHVDCNetPower
```

### Branch Auxiliary Variables

```@docs
PowerFlowBranchReactivePowerFromTo
PowerFlowBranchReactivePowerToFrom
PowerFlowBranchActivePowerFromTo
PowerFlowBranchActivePowerToFrom
PowerFlowBranchActivePowerLoss
```

### Device Control Auxiliary Variables

```@docs
PowerFlowTapRatio
PowerFlowSwitchedShuntSusceptance
PowerFlowFACTSReactivePower
```

### HVDC Auxiliary Variables

```@docs
PowerFlowHVDCActivePowerFromTo
PowerFlowHVDCActivePowerToFrom
PowerFlowHVDCReactivePowerFromTo
PowerFlowHVDCReactivePowerToFrom
PowerFlowHVDCActivePowerLoss
PowerFlowHVDCDCCurrent
PowerFlowHVDCDCVoltageFrom
PowerFlowHVDCDCVoltageTo
PowerFlowLCCRectifierTap
PowerFlowLCCInverterTap
PowerFlowLCCRectifierDelayAngle
PowerFlowLCCInverterExtinctionAngle
PowerFlowConverterDCPower
PowerFlowConverterReactivePower
PowerFlowConverterDCVoltage
```

```@raw html
&nbsp;
&nbsp;
```

* * *

## Constraints

### Common Constraints

```@docs
PiecewiseLinearCostConstraint

```

### Network Constraints

```@docs
CopperPlateBalanceConstraint
NodalBalanceActiveConstraint
NodalBalanceReactiveConstraint
AreaParticipationAssignmentConstraint
```

### Power Variable Limit Constraints

```@docs
ActivePowerVariableLimitsConstraint
ReactivePowerVariableLimitsConstraint
ActivePowerVariableTimeSeriesLimitsConstraint
InputActivePowerVariableLimitsConstraint
OutputActivePowerVariableLimitsConstraint
ActivePowerInVariableTimeSeriesLimitsConstraint
ActivePowerOutVariableTimeSeriesLimitsConstraint
```

### Services Constraints

```@docs
RequirementConstraint
ParticipationFractionConstraint
ReservePowerConstraint
```

### Thermal Unit Constraints

```@docs
ActiveRangeICConstraint
CommitmentConstraint
DurationConstraint
RampConstraint
StartupInitialConditionConstraint
StartupTimeLimitTemperatureConstraint
```

### Renewable Unit Constraints

```@docs
EqualityConstraint
```

## Source Constraints

```@docs
ImportExportBudgetConstraint
```

## Load Constraints

```@docs
ShiftDownActivePowerVariableLimitsConstraint
NonAnticipativityConstraint
ShiftUpActivePowerVariableLimitsConstraint
RealizedShiftedLoadMinimumBoundConstraint
ShiftedActivePowerBalanceConstraint
```

### Branches Constraints

```@docs
FlowLimitConstraint
FlowRateConstraint
FlowRateConstraintFromTo
FlowRateConstraintToFrom
HVDCPowerBalance
NetworkFlowConstraint
PhaseAngleControlLimit
```

### Two Terminal and Multi-Terminal HVDC Constraints

```@docs
ConverterLossConstraint
InterpolationVoltageConstraints
InterpolationCurrentConstraints
InterpolationBilinearConstraints
CurrentAbsoluteValueConstraint
ConverterPowerCalculationConstraint
ConverterMcCormickEnvelopes
DCLineCurrentConstraint
DCCurrentBalance
```

### Contingency Constraints

```@docs
PostContingencyGenerationBalanceConstraint
PostContingencyActivePowerVariableLimitsConstraint
PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint
PostContingencyFlowRateConstraint
```

### Market Bid Cost Constraints

```@docs
PiecewiseLinearBlockIncrementalOfferConstraint
PiecewiseLinearBlockDecrementalOfferConstraint
```

### Feedforward Constraints

```@docs
FeedforwardSemiContinuousConstraint
FeedforwardUpperBoundConstraint
FeedforwardLowerBoundConstraint
```

```@raw html
&nbsp;
&nbsp;
```

* * *

## Parameters

### Time Series Parameters

```@docs
ActivePowerTimeSeriesParameter
ReactivePowerTimeSeriesParameter
RequirementTimeSeriesParameter
ReactivePowerOffsetParameter
ActivePowerOutTimeSeriesParameter
ActivePowerInTimeSeriesParameter
FuelCostParameter
FromToFlowLimitParameter
ToFromFlowLimitParameter
```

### Variable Value Parameters

```@docs
UpperBoundValueParameter
LowerBoundValueParameter
OnStatusParameter
FixValueParameter
```

### Objective Function Parameters

```@docs
CostFunctionParameter
```

### Events Parameters

```@docs
AvailableStatusChangeCountdownParameter
AvailableStatusParameter
ActivePowerOffsetParameter
BranchRatingTimeSeriesParameter
PostContingencyBranchRatingTimeSeriesParameter
```

## Results

### Acessing Optimization Model

```@autodocs
Modules = [PowerSimulations]
Pages   = ["optimization_container.jl",
            "optimization_debugging.jl"
           ]
Order = [:type, :function]
Public = true
Private = false
```

### Accessing Problem Results

```@autodocs
Modules = [PowerSimulations]
Pages   = ["operation/problem_results.jl",
           ]
Order = [:type, :function]
Public = true
Private = false
```

### Accessing Simulation Results

```@autodocs
Modules = [PowerSimulations]
Pages   = ["simulation_results.jl",
            "simulation_problem_results.jl",
            "simulation_partition_results.jl",
            "hdf_simulation_store.jl"
           ]
Order = [:type, :function]
Public = true
Private = false
```

## Simulation Recorder

```@autodocs
Modules = [PowerSimulations]
Pages   = ["utils/recorder_events.jl",
           ]
Order = [:type, :function]
Public = true
Private = false
```
