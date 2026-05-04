# PowerSimulations.jl

Power system optimization and simulation framework. Builds and solves large-scale optimization problems for operations modeling across multiple time scales (planning, day-ahead, real-time). Julia compat: `^1.10`.

> **General Sienna Programming Practices:** For performance requirements, code conventions, documentation practices, and contribution workflows that apply across all Sienna packages, see [Sienna.md](Sienna.md). Always load [Sienna.md](Sienna.md) before any change or code execution.

## Core Architecture

### Operation Models

The central abstraction is `OperationModel`, with two concrete types:

  - **`DecisionModel{M <: DecisionProblem}`** — Solves optimization problems over a specified horizon (e.g., 24h unit commitment, 1h economic dispatch). Contains a `ProblemTemplate`, an `OptimizationContainer` (JuMP model wrapper), and a `System` from PowerSystems.jl.
  - **`EmulationModel{M <: EmulationProblem}`** — Simulates real-time operation with a single time-step horizon. Used for AGC, reserve deployment, and similar fast-timescale problems.

Built-in problem types: `GenericOpProblem`, `UnitCommitmentProblem`, `EconomicDispatchProblem`, `AGCReserveDeployment`.

### ProblemTemplate

Defines what a model contains — its network representation and which device/service formulations to use:

```julia
template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
```

### Device, Service, and Network Models

These types bind a PowerSystems component type to a formulation:

  - **`DeviceModel{D <: PSY.Device, B <: AbstractDeviceFormulation}`** — Specifies how a device type is modeled. The formulation determines which variables, constraints, and parameters are added. Also carries feedforward specifications, time series mappings, and attributes.
  - **`ServiceModel{D <: PSY.Service, B <: AbstractServiceFormulation}`** — Same pattern for ancillary services (reserves, AGC).
  - **`NetworkModel{T <: PM.AbstractPowerModel}`** — Specifies the power flow formulation. Options include `CopperPlatePowerModel` (single node), `PTDFPowerModel` (linearized with PTDF matrix), `AreaBalancePowerModel` (zonal), and full AC/DC from PowerModels.jl.

### Formulation Hierarchy

Formulations are organized by device category. The formulation type controls what gets built:

  - **Thermal**: `ThermalBasicUnitCommitment`, `ThermalStandardUnitCommitment`, `ThermalBasicDispatch`, `ThermalCompactUnitCommitment`, etc. UC formulations add binary on/off variables and min up/down time constraints; dispatch formulations use continuous variables only.
  - **Renewable**: `RenewableFullDispatch`, `RenewableConstantPowerFactor`
  - **Load**: `StaticPowerLoad`, `PowerLoadInterruption`, `PowerLoadDispatch`
  - **Storage**: `BookKeeping`, `BatteryAncillaryServices`
  - **Branches**: `StaticBranch`, `StaticBranchBounds`, `StaticBranchUnbounded`, `HVDCTwoTerminalDispatch`

### OptimizationContainer

Wraps the JuMP model and holds all optimization artifacts in typed containers:

  - **Variables** — decision variables indexed by device and time
  - **Constraints** — constraint references
  - **Parameters** — time-varying data (time series, feedforward values) stored as parameter containers
  - **Expressions** — reusable expressions (e.g., nodal balance) that multiple devices contribute to
  - **Objective function** — cost components

## Simulation Architecture

### Simulation

Orchestrates multi-model runs across time. A `Simulation` contains:

  - **`SimulationModels`** — Container holding a vector of `DecisionModel`s and an optional `EmulationModel`
  - **`SimulationSequence`** — Defines execution order, feedforward connections between models, and initial condition chronologies
  - **`SimulationState`** — Tracks evolving state across the simulation timeline

### SimulationState

Maintains state that flows between models and across time steps:

```
SimulationState
├── current_time::Ref{DateTime}          # Current simulation clock
├── last_decision_model::Ref{Symbol}     # Which model ran last
├── decision_states::DatasetContainer    # Outputs from decision models
└── system_states::DatasetContainer      # Actual system state evolution
```

After each model solves, its results update the relevant datasets in `SimulationState`. The next model in sequence reads from these datasets via feedforwards and initial conditions.

### Feedforward Mechanism

Feedforwards transfer values between models in a simulation sequence. They parameterize a downstream model using results from an upstream model:

  - **`UpperBoundFeedforward`** — Constrains variables with upper bounds from source
  - **`LowerBoundFeedforward`** — Constrains variables with lower bounds from source
  - **`SemiContinuousFeedforward`** — Passes binary on/off status
  - **`FixValueFeedforward`** — Fixes variable values from source results

Each feedforward specifies a source model, source variable, and affected component/variable in the target model.

### Initial Conditions

State carried between time steps within or across models:

  - `DevicePower` — Previous generation level
  - `DeviceStatus` — On/off status
  - `InitialTimeDurationOn/Off` — Time in current state
  - `InitialEnergyLevel` — Storage state-of-charge
  - `AreaControlError` — AGC error state

Chronologies control how initial conditions are sourced: `InterProblemChronology` (from a different model's results) or `IntraProblemChronology` (from the same model's previous solve).

### Simulation Execution Loop

 1. Read current state from `SimulationState`
 2. Update feedforward parameters in the current model from upstream results
 3. Update initial conditions from state
 4. Solve the model (`JuMP.optimize!`)
 5. Write results to `SimulationState` and results store (HDF5 or in-memory)
 6. Advance to next model in sequence; repeat

## Directory Structure

```
src/
├── core/                          # Core types: OptimizationContainer, DeviceModel,
│                                  #   NetworkModel, ServiceModel, formulations,
│                                  #   variable/constraint/parameter type definitions
├── operation/                     # DecisionModel, EmulationModel, ProblemTemplate,
│                                  #   built-in problem templates, model build/solve logic
├── simulation/                    # Simulation, SimulationModels, SimulationSequence,
│                                  #   SimulationState, results storage (HDF5, in-memory)
├── devices_models/
│   ├── devices/                   # Per-device-type implementations (thermal, renewable,
│   │                              #   loads, branches, HVDC, storage)
│   └── device_constructors/       # Build functions that add variables, constraints,
│                                  #   parameters to OptimizationContainer per formulation
├── network_models/                # CopperPlate, PTDF, AreaBalance, PowerModels interface
├── services_models/               # Reserve and transmission interface implementations
├── feedforward/                   # Feedforward types, argument setup, constraint builders
├── initial_conditions/            # IC types, chronologies, update logic
└── parameters/                    # Parameter update mechanisms for time series and state
```

## Build Flow

When `build!(model, system)` is called:

 1. Template specifies device models, service models, and network model
 2. For each `DeviceModel`, the formulation type dispatches to device-specific constructors that add variables, constraints, parameters, and expressions to the `OptimizationContainer`
 3. Network model adds power balance and flow constraints; devices contribute to shared nodal balance expressions
 4. Service models add reserve variables and participation constraints
 5. Feedforwards (if in simulation context) add linking constraints/parameters
 6. Objective function assembled from cost components
 7. Result: a complete JuMP optimization model ready to solve
