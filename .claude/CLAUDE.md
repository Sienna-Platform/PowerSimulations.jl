# PowerSimulations.jl — Claude Guide

Platform-wide Sienna conventions (performance, type stability, formatter, environments, code style) live in `.claude/Sienna.md` — read it too. This file is repo-specific and does not restate them.

Power-system optimization and simulation framework. Builds and solves large-scale optimization (JuMP) problems for operations modeling across multiple time scales (planning, day-ahead, real-time). Package version `0.36.x`; Julia compat `^1.10`.

## Where it sits in the Sienna stack

- **Upstream deps:** `InfrastructureSystems` (IS — container, time series, serialization, `@assert_op`), `PowerSystems` (PSY — `System`, component/device/service types), `PowerNetworkMatrices` (PNM — PTDF/LODF/`VirtualPTDF`/`VirtualMODF`, network reduction), `PowerFlows` (PF — power-flow-in-the-loop), `PowerModels` (PM — AC/DC network formulation abstract types), `JuMP`/`MathOptInterface` (the optimizer interface), `HDF5` (results store).
- **Downstream:** PowerAnalytics / PowerGraphics / PowerSimulationsDynamics consume PSI results and `System` copies. Changes to results storage and serialization have downstream blast radius.
- `InfrastructureOptimizationModels` and `PowerOperationsModels` are NOT current dependencies (not in `Project.toml`) — do not assume coupling to them.

### Version pairing (non-obvious, keep coupled)
- PF and PNM are version-coupled: do not mix incompatible majors. Current compat: `PowerFlows ^0.21.1`, `PowerNetworkMatrices ^0.24`, `PowerSystems ^5.11`, `PowerModels ^0.21.5`, `JuMP ^1.28`, `InfrastructureSystems ^3.5`. When bumping PF, bump PNM in lockstep and re-run the full suite.

## Core Architecture

### Operation Models
Central abstraction `OperationModel`, two concrete types:
- **`DecisionModel{M <: DecisionProblem}`** — optimization over a horizon (e.g. 24h UC, 1h ED). Holds a `ProblemTemplate`, an `OptimizationContainer` (JuMP wrapper), and a PSY `System`.
- **`EmulationModel{M <: EmulationProblem}`** — single-time-step real-time emulation (AGC, reserve deployment).

Built-in problem types: `GenericOpProblem`, `UnitCommitmentProblem`, `EconomicDispatchProblem`, `AGCReserveDeployment`.

### ProblemTemplate
Defines network representation + device/service/event formulations:
```julia
template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
```

### Device, Service, Network, Event Models
- **`DeviceModel{D <: PSY.Device, B <: AbstractDeviceFormulation}`** — binds a device type to a formulation; carries feedforwards, time-series mappings, attributes.
- **`ServiceModel{D <: PSY.Service, B <: AbstractServiceFormulation}`** — same pattern for ancillary services.
- **`NetworkModel{T <: PM.AbstractPowerModel}`** — power-flow formulation: `CopperPlatePowerModel`, `PTDFPowerModel`/`AbstractPTDFModel`, `AreaBalancePowerModel`, `AreaPTDFPowerModel`, full AC/DC from PowerModels.
- **`EventModel`** (`set_event_model!`) — outage/contingency events; see `src/core/event_model.jl`, `event_keys.jl`, and the `contingency_model/` directory.

### Formulation Hierarchy (formulation type controls what is built)
- **Thermal:** `ThermalBasicUnitCommitment`, `ThermalStandardUnitCommitment`, `ThermalCompactUnitCommitment`, `ThermalBasicDispatch`, etc. (UC = binary on/off + min up/down; dispatch = continuous only)
- **Renewable:** `RenewableFullDispatch`, `RenewableConstantPowerFactor`
- **Load:** `StaticPowerLoad`, `PowerLoadInterruption`, `PowerLoadDispatch`
- **Storage:** `BookKeeping`, `BatteryAncillaryServices`
- **Branches:** `StaticBranch`, `StaticBranchBounds`, `StaticBranchUnbounded`, `HVDCTwoTerminalDispatch`, `SecurityConstrainedStaticBranch`

### OptimizationContainer
Wraps the JuMP model; holds Variables, Constraints, Parameters (time series + feedforward), Expressions (shared nodal/area balance), and the objective, all in typed containers keyed by `OptimizationContainerKey`.

## Simulation Architecture

- **`Simulation`** orchestrates multi-model runs across time.
- **`SimulationModels`** — vector of `DecisionModel`s + optional `EmulationModel`.
- **`SimulationSequence`** — execution order, feedforward connections, initial-condition chronologies.
- **`SimulationState`** — `current_time`, `last_decision_model`, `decision_states::DatasetContainer`, `system_states::DatasetContainer`.

### Feedforwards
`UpperBoundFeedforward`, `LowerBoundFeedforward`, `SemiContinuousFeedforward`, `FixValueFeedforward` — parameterize a downstream model with upstream results (source model, source variable, affected target component/variable).

### Initial Conditions
`DevicePower`, `DeviceStatus`, `InitialTimeDurationOn/Off`, `InitialEnergyLevel`, `AreaControlError`. Chronologies: `InterProblemChronology` (other model's results) / `IntraProblemChronology` (same model's previous solve).

### Execution loop
read state → update feedforward params → update initial conditions → `JuMP.optimize!` → write results to `SimulationState` + store (HDF5 or in-memory) → advance.

## src/ layout

```
src/
├── PowerSimulations.jl            # main module: include order + ALL exports
├── core/                          # OptimizationContainer, DeviceModel, NetworkModel,
│                                  #   ServiceModel, EventModel, formulations, variable/
│                                  #   constraint/parameter/expression keys, settings,
│                                  #   datasets, network_reductions, power_flow_data_wrapper
├── operation/                     # DecisionModel, EmulationModel, ProblemTemplate,
│                                  #   built-in templates, template_validation, build/solve
├── simulation/                    # Simulation, SimulationModels, SimulationSequence,
│                                  #   SimulationState, results storage (HDF5 / in-memory)
├── devices_models/
│   ├── devices/                   # per-device impls (AC_branches, TwoTerminalDC_branches,
│   │                              #   thermal/renewable/loads/HVDC/source, area_interchange,
│   │                              #   ac_transmission_security_constrained_models)
│   ├── devices/common/            # shared device helpers
│   └── device_constructors/       # construct_device! build functions per formulation
├── network_models/                # CopperPlate, PTDF, AreaBalance, PowerModels interface,
│                                  #   power_flow_evaluation (PF-in-the-loop), HVDC constructors
├── services_models/               # reserve + transmission interface formulations
├── feedforward/                   # feedforward types, argument setup, constraint builders
├── initial_conditions/            # IC types, chronologies, update logic
├── contingency_model/             # contingency.jl, contingency_arguments.jl,
│                                  #   contingency_constraints.jl (N-1 / SCUC support)
├── parameters/                    # parameter update mechanisms (time series + state)
└── utils/
```

## Build flow (`build!(model, system)`)
1. Template specifies device/service/network/event models.
2. Each `DeviceModel` dispatches on its formulation to `construct_device!`, which adds variables, constraints, parameters, expressions to the `OptimizationContainer` in two stages: **ArgumentConstructStage** (variables, parameters, expressions) then **ModelConstructStage** (constraints). Each `DeviceModel` checks its own `haskey(get_time_series_names(device_model), <ParameterType>)` in each stage — mirror ThermalGeneration.
3. Network model adds power-balance/flow constraints; devices contribute to shared nodal/area balance expressions.
4. Service models add reserve variables/participation constraints.
5. Feedforwards (simulation context) add linking constraints/parameters.
6. Objective assembled from cost components.

## Running tests, docs, formatter (verified commands for THIS repo)

```sh
# Formatter (run after every change; this is the project script)
julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'
#   (script self-activates scripts/formatter and instantiates; formats ./src ./test ./docs)

# Full test suite (test env)
julia --project=test test/runtests.jl

# A single test file by name (runner uses @includetests ARGS)
julia --project=test test/runtests.jl test_network_constructors

# Instantiate test env
julia --project=test -e 'using Pkg; Pkg.instantiate()'

# Build docs
julia --project=docs docs/make.jl
```
- Test runner is the classic InfrastructureSystems `@includetests ARGS` runner (`test/runtests.jl`), plus `Aqua` (`test_unbound_args`, ambiguity, etc.). Test files are named `test_*.jl`; deps live in `test/Project.toml`.
- See `.claude/Sienna.md` for the `--project=test` rule and PSB shared-state gotchas; see the `sienna-test-environment` skill for in-isolation-vs-`runtests` differences.

## Optimization Model Construction Conventions

### `add_*!()` methods must not return collections
Methods that create variables, constraints, or expressions (`add_variables!`, `add_constraints!`, `add_expressions!`, etc.) must always end with a bare `return` (i.e., return `nothing`). They must never return dicts or collections of JuMP objects. Instead, instantiate the appropriate container via `add_*_container!` and store all created objects there.

### Inline expressions when possible
Expression construction should be inlined at the point of use. Only store an expression in a container when it is intended to be reused across multiple constraints or objective terms. Avoid creating expression containers solely as intermediate computation steps.

## Package-specific conventions, invariants, gotchas

### Never modify `src/core/optimization_container.jl`
The harness layer (template iteration over branches/devices, build orchestration) is off-limits — central pipeline, outsized blast radius, breaks invariants other code relies on (e.g. `AreaInterchange` depends on natural template iteration order under `AreaPTDFPowerModel`). If a fix looks like it needs reordering template iteration, claiming arcs, or cross-`DeviceModel` coordination at the container level: STOP and push the fix down into the specific `construct_device!` / `add_parameters!` / `add_constraints!` chain in `src/devices_models/`. Cross-type coordination, when genuinely needed, is data-driven (e.g. iterate a `name_to_arc_map[T]` so a time-series `DeviceModel` sees every arc its branch type participates in).

### Branch dedup / parallel groups — do NOT make `get_branch_argument_constraint_axis` time-series-aware
A mixed-type parallel branch group collapsed to one reduced arc routes its time-varying limit through parameter addition (reduction-aware routing), NOT through dedup. Two prior attempts to make dedup TS-aware both regressed the green "BranchesParallel of different types (MonitoredLine with BranchRatingTimeSeriesParameter)" testset. The correct layer is parameter addition; do not re-attempt a dedup change.

### Branch-rating time-series multiplier is type-aware
`_resolve_branch_multiplier` (`src/devices_models/devices/AC_branches.jl`) is a dispatch set, not an `isa` branch: parallel groups use `PNM.get_sum_of_max_rating` / `get_equivalent_emergency_rating`; non-parallel series/3W use `PNM.get_equivalent_rating` / `get_equivalent_emergency_rating` (matches the static path). All PNM reduction wrappers are `<: PSY.ACTransmission`. Branch-rating TS is only supported with `StaticBranch` / `AbstractSecurityConstrainedStaticBranch` (enforced in `template_validation.jl`). **PSI JuMP parameter objects can never be squared in a constraint expression** — AC `FlowRateConstraintFromTo/ToFrom` use `<= param[name,t]*mult[name,t]`; the resolved AC RHS already equals `(DC RHS)^2` via the multiplier, so do not re-introduce `(param*mult)^2`.

### Security-constrained branch network-model support
`SecurityConstrainedStaticBranch` must WORK for PTDF (`AbstractPTDFModel`), ACP (`PM.AbstractACPModel`), and DCP (`DCPPowerModel`) — SC/MODF is DC-linearized so DCP is valid. NFA and CopperPlate must be deliberate NO-OP `construct_device!` (build nothing, do not error, not `ConflictingInputsError`/MethodError) — no meaningful branch-flow representation means SC is silently inert by design (this is the one intentional no-op, distinct from the forbidden silent data-error skip).

### Parameter multiplier `fill!` optimizations need a manual audit
Whether a parameter type has a uniform multiplier across all `(device, time)` cells is NOT statically derivable — it depends on which `get_parameter_multiplier(::T, ::D, ::W)` overloads return a constant vs per-device value, spread across many formulation files. Never claim a multiplier is uniform without a user audit; frame `fill!` proposals as "for uniform-multiplier types" and ask which qualify. Safe generic alternatives: integer-indexed writes to `.data`, or hoisting `get_multiplier_array` out of loops. Hot paths: `_add_parameters!` in `src/parameters/add_parameters.jl`, `src/contingency_model/contingency_arguments.jl`.

### No silent absence-sentinel skips
Do not add `isnothing(x) && continue` / `x === nothing && continue` guards that hide malformed-data bugs; let the next call (`add_time_series!`, `error()`, a MethodError) surface the data error. When triaging bot review comments, mark "add a nothing-skip" suggestions as invalid.

### Results time-series recovery (downstream coupling)
PSI 0.34+ stopped serializing a full `System` copy, so `populate_system=true` can return a `System` with 0 time series, breaking PowerAnalytics/PowerGraphics. `TimeSeriesAttributes.component_name_to_ts_uuid` (`src/core/parameters.jl`) is populated via `add_component_name!` (`src/parameters/add_parameters.jl`) but historically not serialized — relevant when touching results/serialization (`_serialize_systems_to_json` in `simulation.jl`).

## Auto-generated / do-not-hand-edit
- Variable/constraint/parameter/expression type definitions and their export lists are concentrated in `src/core/` and `src/PowerSimulations.jl`. Respect include order: new constants/types must be defined in files included before files that reference them — check `src/PowerSimulations.jl` for include order before placing definitions.
- Do not edit any auto-generated files directly (see Sienna.md).

## Default branch
`main` (not `master`). Diff/PR/review against `origin/main`. Per global rules: never `git commit`; leave changes unstaged.
