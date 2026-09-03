# [PowerSimulations.jl Modeling Structure](@id psi_structure)

PowerSimulations orchestrates a sequence of power-system optimization problems and controls
how they connect. It does not build the problems themselves.

```
IS ──▶ IOM ──▶ POM ──▶ PSI ──▶ PowerAnalytics / PowerGraphics
        ▲       ▲
       PSY ──▶ PNM, PF, PSB
```

  - **`InfrastructureOptimizationModels.jl`** (IOM) defines the domain-neutral optimization
    core: `OptimizationContainer`, `DecisionModel`, `EmulationModel`, settings, per-model
    stores, and results types.
  - **`PowerOperationsModels.jl`** (POM) defines every power formulation: device and service
    models via `DeviceModel` and `ServiceModel`, network models, `PowerOperationsProblemTemplate`,
    and the `build!`/`solve!` chain for a single model.
  - **`PowerSimulations.jl`** (PSI) takes built `DecisionModel`s and `EmulationModel`s and
    sequences them: it decides execution order, moves data between models with
    `SimulationSequence` and feedforwards, tracks `SimulationState` across solves, and stores
    and reads back results.

A `Simulation` is built from:

  - a [`SimulationModels`](@ref) — the `DecisionModel`s (and optional `EmulationModel`) to run,
  - a [`SimulationSequence`](@ref) — execution order, feedforwards, and initial-condition
    chronologies, and
  - the number of steps and an output folder.

`build!(sim)` builds every model and wires the sequence; `execute!(sim)` runs it, writing
results to a `SimulationStore` as it goes.

!!! question "What is the difference between a Model and a Problem?"
    
    A "Problem" is an abstract mathematical description of how to represent power system
    behavior; a "Model" is that Problem applied to a `System`. Building `DeviceModel`s,
    `ServiceModel`s, and `PowerOperationsProblemTemplate`s is POM's job — see the
    [`PowerOperationsModels.jl`](https://github.com/Sienna-Platform/PowerOperationsModels.jl)
    documentation.
