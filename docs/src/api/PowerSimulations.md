```@meta
CurrentModule = PowerSimulations
DocTestSetup  = quote
    using PowerSimulations
end
```

# API Reference

This page documents PSI's own API: simulation orchestration, outputs, and stores. For device,
service, and network formulations, problem templates, and single-model `build!`/`solve!`, see
the [`PowerOperationsModels.jl`](https://sienna-platform.github.io/PowerOperationsModels.jl/dev/)
documentation. For the optimization core (`OptimizationContainer`, datasets, per-model stores),
see `InfrastructureOptimizationModels.jl`.

```@contents
Pages = ["PowerSimulations.md"]
Depth = 3
```

## Problem Types

```@autodocs
Modules = [PowerSimulations]
Pages   = ["operation_model_types.jl"]
Order = [:type, :function]
Public = true
Private = false
```

### Problem Templates

```@autodocs
Modules = [PowerSimulations]
Pages   = ["operation_problem_templates.jl"]
Order = [:type, :function]
Public = true
Private = false
```

* * *

## Simulation Entry Points

```@autodocs
Modules = [PowerSimulations]
Pages   = ["decision_model.jl", "emulation_model.jl", "operation_model_simulation_interface.jl"]
Order = [:type, :function]
Public = true
Private = false
```

## Parameter Updates

```@autodocs
Modules = [PowerSimulations]
Pages   = ["update_container_parameter_values.jl", "update_parameters.jl", "update_cost_parameters.jl"]
Order = [:type, :function]
Public = true
Private = false
```

* * *

## Simulation Models

```@autodocs
Modules = [PowerSimulations]
Pages   = ["simulation_models.jl"]
Order = [:type, :function]
Public = true
Private = false
```

## Simulation Sequence

```@autodocs
Modules = [PowerSimulations]
Pages   = ["simulation_sequence.jl"]
Order = [:type, :function]
Public = true
Private = false
```

## Chronology Models

```@autodocs
Modules = [PowerSimulations]
Pages   = ["initial_condition_chronologies.jl"]
Order = [:type, :function]
Public = true
Private = false
```

## Simulation

```@autodocs
Modules = [PowerSimulations]
Pages   = ["simulation.jl"]
Order = [:type, :function]
Public = true
Private = false
```

## Simulation Partitions

```@autodocs
Modules = [PowerSimulations]
Pages   = ["simulation_partitions.jl", "simulation_partition_outputs.jl"]
Order = [:type, :function]
Public = true
Private = false
```

* * *

## Outputs

### Accessing Simulation Outputs

```@autodocs
Modules = [PowerSimulations]
Pages   = ["simulation_outputs.jl",
            "simulation_problem_outputs.jl",
            "decision_model_simulation_outputs.jl",
            "emulation_model_simulation_outputs.jl",
            "realized_meta.jl",
           ]
Order = [:type, :function]
Public = true
Private = false
```

### Exporting Outputs

```@autodocs
Modules = [PowerSimulations]
Pages   = ["simulation_outputs_export.jl"]
Order = [:type, :function]
Public = true
Private = false
```

* * *

## Simulation Stores

```@autodocs
Modules = [PowerSimulations]
Pages   = ["abstract_simulation_store.jl",
            "hdf_simulation_store.jl",
            "in_memory_simulation_store.jl",
            "simulation_store_common.jl",
            "simulation_store_params.jl",
            "simulation_store_requirements.jl",
           ]
Order = [:type, :function]
Public = true
Private = false
```

* * *

## Simulation Recorder

```@autodocs
Modules = [PowerSimulations]
Pages   = ["recorder_events.jl"]
Order = [:type, :function]
Public = true
Private = false
```
