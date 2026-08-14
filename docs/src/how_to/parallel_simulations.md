# Parallel Simulations

This section contains instructions to:

  - [Run a Simulation in Parallel on a local computer](@ref)
  - [Run a Simulation in Parallel on an HPC](@ref)

## Run a Simulation in Parallel on a local computer

This page describes how to split a simulation into partitions, run each partition in parallel,
and then join the results.

### Setup

Create a Julia script to build and run simulations. It must meet the requirements below.
A full example is in the PowerSimulations repository in `test/run_partitioned_simulation.jl`.

  - Call `using PowerSimulations`.

  - Implement a build function that matches the signature below.
    It must construct a `Simulation`, call `build!`, and then return the `Simulation` instance.
    It must throw an exception if the build fails.

```
function build_simulation(
    output_dir::AbstractString,
    simulation_name::AbstractString,
    partitions::SimulationPartitions,
    index::Union{Nothing, Integer}=nothing,
)
```

Here is example code to construct the `Simulation` with these parameters:

```
    sim = Simulation(
        name=simulation_name,
        steps=partitions.num_steps,
        models=models,
        sequence=sequence,
        simulation_folder=output_dir,
    )
    status = build!(sim; partitions=partitions, index=index, serialize=isnothing(index))
    if status != PSI.SimulationBuildStatus.BUILT
        error("Failed to build simulation: status=$status")
    end
```

  - Implement an execute function that matches the signature below. It must throw an exception
    if the execute fails.

```
function execute_simulation(sim, args...; kwargs...)
    status = execute!(sim)
    if status != PSI.RunStatus.SUCCESSFULLY_FINALIZED
        error("Simulation failed to execute: status=$status")
    end
end
```

### Execution

After loading your script, call the function `run_parallel_simulation` as shown below.

This example splits a year-long simulation into weekly partitions for a total of 53 individual
jobs and then runs them four at a time.

```
julia> include("my_simulation.jl")
julia> run_parallel_simulation(
        build_simulation,
        execute_simulation,
        script="my_simulation.jl",
        output_dir="my_simulation_output",
        name="my_simulation",
        num_steps=365,
        period=7,
        num_overlap_steps=1,
        num_parallel_processes=4,
        exeflags="--project=<path-to-your-julia-environment>",
    )
```

The final results will be in `./my_simulation_otuput/my_simulation`

Note the log files and results for each partition are located in
`./my_simulation_otuput/my_simulation/simulation_partitions`

### Failed partitions

If any partition fails, the join step throws an exception and records a failed status in
`./my_simulation_output/my_simulation/results/status.json`. Each failed partition, along
with the simulation steps that it covers, is logged in
`./my_simulation_output/my_simulation/logs/join_partitioned_simulation.log`.

If you want to inspect the results of the successful partitions, join them by skipping the
failed partitions, as shown below. The status of the simulation remains a failure and the
results of the steps that were covered by the failed partitions are invalid.

```
julia> PowerSimulations.join_simulation("my_simulation_output/my_simulation"; skip_failures=true)
julia> results = SimulationResults("my_simulation_output/my_simulation"; ignore_status=true)
```

## Run a Simulation in Parallel on an HPC

This page describes how to split a simulation into partitions, run each partition in parallel
on HPC compute nodes, and then join the results.

These steps can be used on a local computer or any HPC supported by the submission software.
Some steps may be specific to NREL's HPC `Eagle` cluster.

*Note*: Some instructions are preliminary and will change if functionality is moved
to a new Julia package.

### Setup

 1. Create a conda environment and install the Python package `NREL-jade`:
    https://nrel.github.io/jade/installation.html. The rest of this page assumes that
    the environment is called `jade`.
 2. Activate the environment with `conda activate jade`.
 3. Locate the path to that conda environment. It will likely be `~/.conda-envs/jade` or
    `~/.conda/envs/jade`.
 4. Load the Julia environment that you use to run simulations. Add the packages `Conda` and
    `PyCall`.
 5. Setup Conda to use the existing `jade` environment by running these commands:

```
julia> run(`conda create -n conda_jl python conda`)
julia> ENV["CONDA_JL_HOME"] = joinpath(ENV["HOME"], ".conda-envs", "jade")  # change this to your path
pkg> build Conda
```

 6. Copy the code below into a Julia file called `configure_parallel_simulation.jl`.
    This is an interface to Jade through PyCall. It will be used to create a Jade configuration.
    (It may eventually be moved to a separate package.)

```
function configure_parallel_simulation(
    script::AbstractString,
    num_steps::Integer,
    num_period_steps::Integer;
    num_overlap_steps::Integer=0,
    project_path=nothing,
    simulation_name="simulation",
    config_file="config.json",
    force=false,
)
    partitions = SimulationPartitions(num_steps, num_period_steps, num_overlap_steps)
    jgc = pyimport("jade.extensions.generic_command")
    julia_cmd = isnothing(project_path) ? "julia" : "julia --project=$project_path"
    setup_command = "$julia_cmd $script setup --simulation-name=$simulation_name " *
    "--num-steps=$num_steps --num-period-steps=$num_period_steps " *
    "--num-overlap-steps=$num_overlap_steps"
    teardown_command = "$julia_cmd $script join --simulation-name=$simulation_name"
    config = jgc.GenericCommandConfiguration(
        setup_command=setup_command,
        teardown_command=teardown_command,
    )

    for i in 1:get_num_partitions(partitions)
        cmd = "$julia_cmd $script execute --simulation-name=$simulation_name --index=$i"
        job = jgc.GenericCommandParameters(command=cmd, name="execute-$i")
        config.add_job(job)
    end

    config.dump(config_file, indent=2)
    println("Created Jade configuration in $config_file. " *
            "Run 'jade submit-jobs [options] $config_file' to execute them.")
end
```

 7. Create a Julia script to build and run simulations. It must meet the requirements below.
    A full example is in the PowerSimulations repository in `test/run_partitioned_simulation.jl`.

  - Call `using PowerSimulations`.

  - Implement a build function that matches the signature below.
    It must construct a `Simulation`, call `build!`, and then return the `Simulation` instance.
    It must throw an exception if the build fails.

```
function build_simulation(
    output_dir::AbstractString,
    simulation_name::AbstractString,
    partitions::SimulationPartitions,
    index::Union{Nothing, Integer}=nothing,
)
```

Here is example code to construct the `Simulation` with these parameters:

```
    sim = Simulation(
        name=simulation_name,
        steps=partitions.num_steps,
        models=models,
        sequence=sequence,
        simulation_folder=output_dir,
    )
    status = build!(sim; partitions=partitions, index=index, serialize=isnothing(index))
    if status != PSI.SimulationBuildStatus.BUILT
        error("Failed to build simulation: status=$status")
    end
```

  - Implement an execute function that matches the signature below. It must throw an exception
    if the execute fails.

```
function execute_simulation(sim, args...; kwargs...)
    status = execute!(sim)
    if status != PSI.RunStatus.SUCCESSFULLY_FINALIZED
        error("Simulation failed to execute: status=$status")
    end
end
```

  - Make the script runnable as a CLI command by including the following code at the bottom of the
    file.

```
function main()
    process_simulation_partition_cli_args(build_simulation, execute_simulation, ARGS...)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
```

### Execution

 1. Create a Jade configuration that defines the partitioned simulation jobs. Load your Julia
    environment.
    
    This example splits a year-long simulation into weekly partitions for a total of 53 individual
    jobs.

```
julia> include("configure_parallel_simulation.jl")
julia> num_steps = 365
julia> period = 7
julia> num_overlap_steps = 1
julia> configure_parallel_simulation(
    "my_simulation.jl",  # this is your build/execute script
    num_steps,
    period,
    num_overlap_steps=1,
    project_path=".",  # This optionally specifies the Julia project environment to load.
)
Created Jade configuration in config.json. Run 'jade submit-jobs [options] config.json' to execute them.
```

Exit Julia.

 2. View the configuration for accuracy.

```
$ jade config show config.json
```

 3. Start an interactive session on a debug node. *Do not submit the jobs on a login node!* The submission
    step will run a full build of the simulation and that may consume too many CPU and memory resources
    for the login node.

```
$ salloc -t 01:00:00 -N1 --account=<your-account> --partition=debug
```

 4. Follow the instructions at https://nrel.github.io/jade/tutorial.html to submit the jobs.
    The example below will configure Jade to run each partition on its own compute node. Depending on
    the compute and memory constraints of your simulation, you may be able to pack more jobs on each
    node.
    
    Adjust the walltime as necessary.

```
$ jade config hpc -c hpc_config.toml -t slurm  --walltime=04:00:00 -a <your-account>
$ jade submit-jobs config.json --per-node-batch-size=1 -o output
```

If you are unsure about how much memory and CPU resources your simulation consumes, add these options:

```
$ jade submit-jobs config.json --per-node-batch-size=1 -o output --resource-monitor-type periodic --resource-monitor-interval 3
```

Jade will create HTML plots of the resource utilization in `output/stats`. You may be able to customize
`--per-node-batch-size` and `--num-processes` to finish the simulations more quickly.

 5. Jade will run a final command to join the simulation partitions into one unified file. You can load the
    results as you normally would.

```
julia> results = SimulationResults("<output-dir>/job-outputs/<simulation-name>")
```

Note the log files and results for each partition are located in
`<output-dir>/job-outputs/<simulation-name>/simulation_partitions`

If any partition fails, the join command fails and records a failed status in
`<output-dir>/job-outputs/<simulation-name>/results/status.json`. Each failed partition,
along with the simulation steps that it covers, is logged in
`<output-dir>/job-outputs/<simulation-name>/logs/join_partitioned_simulation.log`.
Re-run the command with the `--skip-failures` option to skip the failed partitions and merge
the results of the successful ones. The status of the simulation remains a failure, so the
results must be loaded with `SimulationResults(path; ignore_status=true)`.

```
$ julia --project=<path> my_simulation.jl join --simulation-name=<simulation-name> --output-dir=<output-dir>/job-outputs --skip-failures
```
