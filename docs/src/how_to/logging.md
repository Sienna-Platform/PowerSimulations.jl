# Logging

`PowerSimulations.jl` will output many log messages when building systems and
running simulations. You may want to customize what gets logged to the console
and, optionally, a file.

By default all log messages of level `Logging.Info` or higher will get
displayed to the console.  When you run a simulation a simulation-specific
logger will take over and log its messages to a file in the `logs` directory in
the simulation output directory. When finished it will relinquish control back
to the global logger.

## Configuring the global logger

To configure the global logger in a Jupyter Notebook or REPL you may configure
your own logger with the Julia Logging standard library or use the convenience
function provided by PowerSimulations.  This example will log messages of level
`Logging.Error` to console and `Logging.Info` and higher to the file
`power-simulations.log` in the current directory.

```julia
import Logging
using PowerSimulations
logger = configure_logging(;
    console_level = Logging.Error,
    file_level = Logging.Info,
    filename = "power-simulations.log",
)
```

## Configuring the simulation logger

You can configure the logging level used by the simulation logger when you call
`build!(simulation)`.  Here is an example that increases logging verbosity:

```julia
import Logging
using PowerSimulations
simulation = Simulation(...)
build!(simulation; console_level = Logging.Info, file_level = Logging.Debug)
```

The log file will be located at `<your-output-path>/<simulation-name>/<run-output-dir>/logs/simulation.log`.

## Log groups

Debug messages in PSI's own code are tagged with one of these log groups:

  - `LOG_GROUP_SIMULATION_STORE` — the `HdfSimulationStore`/`InMemorySimulationStore` cache and flush path
  - `LOG_GROUP_OUTPUTS` — result reads and writes
  - `LOG_GROUP_MODEL_STORE` — per-model store access during a simulation
  - `LOG_GROUP_BUILD_INITIAL_CONDITIONS` — initial-condition setup between solves

Model-building log groups (variables, constraints, parameters) belong to
`PowerOperationsModels.jl` and `InfrastructureOptimizationModels.jl`.

## Solver logs

You can configure logging for the solver you use.  Refer to the solver
documentation.  PowerSimulations does not redirect or intercept prints to
`stdout` or `stderr` from other libraries.

## Recorder events

PowerSimulations uses the `InfrastructureSystems.Recorder` to store simulation
events in a log file.  Refer to this [link](./simulation_recorder.md) for more
information.
