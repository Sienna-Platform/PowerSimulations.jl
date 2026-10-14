# Simulation Recorder

PowerSimulations.jl records structured events while a simulation runs. Use them to follow a
running simulation, or to inspect afterwards what each model execution did.

## Recorder files

Every simulation writes two recorders to `<simulation directory>/recorder/`:

| File                    | Events                                                                |
|:----------------------- |:--------------------------------------------------------------------- |
| `simulation_status.log` | `SimulationStepEvent`, `ProblemExecutionEvent`                        |
| `execution.log`         | `ParameterUpdateEvent`, `InitialConditionUpdateEvent`, `StateUpdateEvent` |

Each event is one line of JSON. The recorder flushes the file after every event, so the file
on disk is always current while the simulation runs. Every line has a `name` (the event type)
and a `timestamp` (the wall-clock time when the event was written), followed by the event's
fields. Type names are written without their module prefix, for example `ThermalStandard`.
A missing numeric value is written as `null`.

### `ProblemExecutionEvent`

Each model execution writes a `"start"` line and then either a `"done"` line or, if the
execution throws, a `"failed"` line before the error propagates.

| Field                | Meaning                                                                                     |
|:-------------------- |:------------------------------------------------------------------------------------------- |
| `simulation_time`    | Simulation time of the execution                                                            |
| `step`               | Simulation step                                                                             |
| `model_name`         | Name of the model                                                                           |
| `status`             | `"start"`, `"done"`, or `"failed"`                                                          |
| `termination_status` | Solver termination status, for example `OPTIMAL` or `TIME_LIMIT`; `OPTIMIZE_NOT_CALLED` on `"start"` |
| `objective_value`    | Objective value of a successful solve; `null` otherwise                                     |
| `solve_time_s`       | Wall time of the last `optimize!` call, in seconds; `null` on `"start"`                     |
| `exec_time_s`        | Wall time of the whole execution (update, solve, state update), in seconds; `null` on `"start"` |

The error message of a failed execution is in `logs/simulation.log`.

### `InitialConditionUpdateEvent`

One event per initial-condition key per model execution. The fields `device_names`,
`new_values`, and `previous_values` hold one entry per device.

## Following a running simulation

Because every line is flushed, standard tools can follow the status file while the simulation
runs. For example, with [`jq`](https://jqlang.org/):

```sh
cd <simulation directory>/recorder

# Stream every model execution as it starts and finishes
tail -f simulation_status.log

# The last completed execution
grep '"status":"done"' simulation_status.log | tail -1 | jq .

# Executions that did not solve to optimality
jq -c 'select(.name == "ProblemExecutionEvent" and .status != "start" and .termination_status != "OPTIMAL")' simulation_status.log

# Total solve time per model
jq -s 'map(select(.status == "done")) | group_by(.model_name) | map({model: .[0].model_name, solve_time_s: (map(.solve_time_s) | add)})' simulation_status.log
```

## Reading events in Julia

Assume that a simulation ran in `./output/aggregation/1` and that these setup commands ran:

```julia
using PowerSimulations
import PowerSimulations as PSI
```

[`list_simulation_events`](@ref) returns the events of one type. Filter them by step, by
model, or with a function that accepts an event and returns `true` or `false`:

```julia
events = list_simulation_events(PSI.ProblemExecutionEvent, "./output/aggregation/1")
failed = list_simulation_events(
    PSI.ProblemExecutionEvent,
    "./output/aggregation/1",
    x -> x.status == "failed",
)
uc_updates = list_simulation_events(
    PSI.InitialConditionUpdateEvent,
    "./output/aggregation/1";
    step = 2,
    model_name = "UC",
)
```

[`show_simulation_events`](@ref) prints the same events as a table. Initial-condition events
print one row per device. Pass `wall_time = true` to include the `timestamp` column:

```julia
show_simulation_events(PSI.SimulationStepEvent, "./output/aggregation/1")
show_simulation_events(
    PSI.InitialConditionUpdateEvent,
    "./output/aggregation/1";
    step = 2,
    model_name = "UC",
    wall_time = true,
)
```
