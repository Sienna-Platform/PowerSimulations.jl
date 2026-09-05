# PowerSimulations.jl

```@meta
CurrentModule = PowerSimulations
```

## Overview

`PowerSimulations.jl` (PSI) is the **simulation orchestration** package of the Sienna psy6
line. It runs optimization models in a loop over time, keeps simulation state, updates
parameters and initial conditions between solves, stores results, and reads them back. It does
not build optimization models.

Model building belongs to two upstream packages:

  - [`InfrastructureOptimizationModels.jl`](https://github.com/Sienna-Platform/InfrastructureOptimizationModels.jl) (IOM) — the domain-neutral optimization
    core: `OptimizationContainer`, `DecisionModel`, `EmulationModel`, settings, per-model
    stores, datasets, results types, objective functions.
  - [`PowerOperationsModels.jl`](https://sienna-platform.github.io/PowerOperationsModels.jl/dev/) (POM) — power formulations: every device, service,
    network, HVDC, storage, and hydro formulation, `PowerOperationsProblemTemplate`, the
    problem-type chain, per-model `build!`/`solve!`/`run!`, feedforward types, parameter
    types, and power-flow-in-the-loop.

`using PowerSimulations` re-exports the full IOM and POM public API, so a script needs only
one `using` statement.

```
IS ──▶ IOM ──▶ POM ──▶ PSI ──▶ PowerAnalytics / PowerGraphics
        ▲       ▲
       PSY ──▶ PNM, PF, PSB
```

PSI owns:

  - `Simulation`, `SimulationModels`, `SimulationSequence` — multi-model orchestration,
    execution order, and feedforward attachment.
  - `SimulationState` — the state passed between solves.
  - Parameter and initial-condition updates between solves.
  - `HdfSimulationStore` / `InMemorySimulationStore`, results, realized results, partitions,
    and recorder events.

If a change needs `SimulationState`, a `SimulationStore`, or knowledge of more than one
model, it belongs in PSI. If it adds a variable, constraint, parameter, or expression to a
container, it belongs in POM (or IOM, if domain-neutral).

`PowerSimulations.jl` is part of the National Renewable Energy Laboratory's
[Sienna ecosystem](https://sienna-platform.github.io/Sienna/), an open source framework for
power system modeling, simulation, and optimization.

## Installation and Quick Links

  - [Sienna installation page](https://sienna-platform.github.io/Sienna/SiennaDocs/docs/build/how-to/install/):
    Instructions to install `PowerSimulations.jl` and other Sienna\Ops packages
  - [`JuMP.jl` solver's page](https://jump.dev/JuMP.jl/stable/installation/#Install-a-solver): An appropriate optimization solver is required for running `PowerSimulations.jl` models. Refer to this page to select and install a solver for your application.
  - [Sienna Documentation Hub](https://sienna-platform.github.io/Sienna/SiennaDocs/docs/build/index.html):
    Links to other Sienna packages' documentation

## How To Use This Documentation

  - **Tutorials** - Detailed walk-throughs to help you *learn* how to use
    `PowerSimulations.jl`
  - **How to...** - Directions to help *guide* your work for a particular task
  - **Explanation** - Additional details and background information to help you *understand*
    `PowerSimulations.jl`, its structure, and how it works behind the scenes
  - **Reference** - Technical references and API for a quick *look-up* during your work

For device, service, and network formulations, see the
[`PowerOperationsModels.jl`](https://sienna-platform.github.io/PowerOperationsModels.jl/dev/)
documentation.

`PowerSimulations.jl` strives to follow the [Diataxis](https://diataxis.fr/) documentation
framework.
