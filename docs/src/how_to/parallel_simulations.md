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

The final results will be in `./my_simulation_output/my_simulation`

Note the log files and results for each partition are located in
`./my_simulation_output/my_simulation/simulation_partitions`

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
Some steps may be specific to NLR's HPC `Kestrel` cluster.

### Setup

 1. Install torc per these
    [instructions](https://natlabrockies.github.io/torc/latest/getting-started/installation.html)
    If you are at NLR, follow
    [these](https://natlabrockies.github.io/torc/latest/getting-started/installation.html#site-specific-installations)
    instead. Connect to the shared torc server.

 2. Copy the code below into a Julia file called `configure_parallel_simulation.jl`. Modify the
    runtime parameters as needed, especially the CPU and memory resource requirements.
    Your Julia environment must include `JSON` and `PowerSimulations`.

```
using JSON
using PowerSimulations

"""
    configure_parallel_simulation(;
        script::AbstractString,
        num_steps::Integer,
        num_period_steps::Integer,
        output_dir::AbstractString,
        account::AbstractString,
        num_overlap_steps::Integer = 1,
        project_path = ".",
        simulation_name = "simulation",
        num_work_nodes = nothing,
        workflow_name::AbstractString = "sienna",
        workflow_description::AbstractString = "Partitioned simulation",
        enable_time_series_monitoring = false,
    )

Write a torc workflow spec to `workflow.json` that runs a partitioned simulation on Slurm.

The workflow has one `build` job, one `job_<i>` job per partition, and a `join` job. It
schedules two Slurm stages: a single-node allocation on the `debug` partition for the build,
and many single-node allocations for the partition jobs and the join job. The second stage is
requested only after the build completes.

# Arguments

  - `script::AbstractString`: Julia script that builds and executes the simulation. It must
    call `process_simulation_partition_cli_args`.
  - `num_steps::Integer`: Total number of simulation steps.
  - `num_period_steps::Integer`: Number of steps in each partition.
  - `output_dir::AbstractString`: Directory where the simulation writes its outputs.
  - `account::AbstractString`: Slurm account charged for all allocations.
  - `num_overlap_steps::Integer`: Number of steps by which adjacent partitions overlap.
  - `project_path`: Julia project environment passed to `julia --project`. Set to `nothing`
    to run `julia` without `--project`.
  - `simulation_name`: Name of the simulation.
  - `num_work_nodes`: Number of single-node allocations for the partition jobs. Defaults to
    one per partition.
  - `workflow_name::AbstractString`: Name of the torc workflow.
  - `workflow_description::AbstractString`: Description of the torc workflow.
  - `enable_time_series_monitoring`: If `true`, record time-series resource utilization for
    each job and for each compute node. View the results with `torc plot-resources`.
"""
function configure_parallel_simulation(;
    script::AbstractString,
    num_steps::Integer,
    num_period_steps::Integer,
    output_dir::AbstractString,
    account::AbstractString,
    num_overlap_steps::Integer = 1,
    project_path = ".",
    simulation_name = "simulation",
    num_work_nodes = nothing,
    workflow_name::AbstractString = "sienna",
    workflow_description::AbstractString = "Partitioned simulation",
    enable_time_series_monitoring = false,
)
    partitions = SimulationPartitions(num_steps, num_period_steps, num_overlap_steps)
    num_partitions = get_num_partitions(partitions)
    # One node per allocation. Defaults to one allocation per partition.
    num_work_nodes = isnothing(num_work_nodes) ? num_partitions : num_work_nodes
    julia_cmd = isnothing(project_path) ? "julia" : "julia --project=$project_path"
    f1 = Dict(
        "name" => "run_script",
        "path" => script,
    )
    rr_small = Dict(
        "name" => "small",
        "num_cpus" => 1,
        "memory" => "50g",
        "runtime" => "PT30M",
    )
    rr_medium = Dict(
        "name" => "medium",
        "num_cpus" => 8,
        "memory" => "200g",
        "runtime" => "PT3H",
    )
    build_command =
        "$julia_cmd $script setup --simulation-name=$simulation_name " *
        "--num-steps=$num_steps --num-period-steps=$num_period_steps " *
        "--num-overlap-steps=$num_overlap_steps --output-dir=$output_dir"
    work_command = "$julia_cmd $script execute --simulation-name=$simulation_name --index={i} --output-dir=$output_dir"
    join_command = "$julia_cmd $script join --simulation-name=$simulation_name --output-dir=$output_dir"
    jobs = [
        Dict(
            "name" => "build",
            "command" => build_command,
            "resource_requirements" => "small",
            "scheduler" => "build",
        ),
        Dict(
            "name" => "job_{i}",
            "command" => work_command,
            "use_parameters" => ["i"],
            "depends_on" => ["build"],
            "resource_requirements" => "medium",
            "scheduler" => "work",
        ),
        Dict(
            "name" => "join",
            "command" => join_command,
            "resource_requirements" => "small",
            "depends_on_regexes" => ["job_\\d+"],
            "scheduler" => "work",
        )
    ]
    slurm_schedulers = [
        # The build step is short and serial; the debug partition starts quickly.
        Dict(
            "name" => "build",
            "account" => account,
            "partition" => "debug",
            "nodes" => 1,
            "walltime" => "01:00:00",
        ),
        # Walltime must cover one "medium" job plus the "small" join job. Torc does not start
        # a job on an allocation whose remaining walltime is shorter than the job's runtime.
        # Four hours fits the common "short" partition.
        Dict(
            "name" => "work",
            "account" => account,
            "nodes" => 1,
            "walltime" => "04:00:00",
        ),
    ]
    actions = [
        # Stage 1: the build allocation is requested as soon as the workflow starts.
        Dict(
            "trigger_type" => "on_jobs_ready",
            "action_type" => "schedule_nodes",
            "jobs" => ["build"],
            "scheduler" => "build",
            "scheduler_type" => "slurm",
            "num_allocations" => 1,
        ),
        # Stage 2: many single-node allocations, requested only after the build completes.
        # The join job runs on whichever of these allocations is still alive at the end.
        Dict(
            "trigger_type" => "on_jobs_ready",
            "action_type" => "schedule_nodes",
            "job_name_regexes" => ["job_\\d+"],
            "scheduler" => "work",
            "scheduler_type" => "slurm",
            "num_allocations" => num_work_nodes,
        ),
    ]
    workflow = Dict(
        "name" => workflow_name,
        "description" => workflow_description,
        "files" => [f1],
        "jobs" => jobs,
        "resource_requirements" => [rr_small, rr_medium],
        "slurm_schedulers" => slurm_schedulers,
        "actions" => actions,
        "parameters" => Dict("i" => "1:$num_partitions"),
    )
    if enable_time_series_monitoring
        # Records per-job and per-node samples for `torc plot-resources`.
        workflow["resource_monitor"] = Dict(
            "jobs" => Dict("enabled" => true, "granularity" => "time_series"),
            "compute_node" => Dict("enabled" => true, "granularity" => "time_series"),
        )
    end

    # TODO: add a post-processing job that loads results.

    filename = "workflow.json"
    open(filename, "w") do io
        JSON.print(io, workflow, 2)
    end
    println("Created torc workflow spec in $filename")
end
```

 3. Create a Julia script to build and run simulations. It must meet the requirements below.
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

 1. Create a torc workflow that defines the partitioned simulation jobs. Load your Julia
    environment.
    
    This example splits a year-long simulation into weekly partitions for a total of 53 individual
    jobs.

```
julia> include("configure_parallel_simulation.jl")
julia> configure_parallel_simulation(;
           script = "my_simulation.jl",  # this is your build/execute script
           num_steps = 365,
           num_period_steps = 7,
           output_dir = "my_simulation_output",
           account = "<your-hpc-account>",
           num_overlap_steps = 1,
           project_path = ".",  # This optionally specifies the Julia project environment to load.
       )
Created torc workflow spec in workflow.json.
```

Exit Julia.

 2. Open the spec in an editor and check for accuracy.

 3. Submit the workflow.

```
torc submit workflow.json
```

 4. Torc will run a final command to join the simulation partitions into one unified file. You can
    load the results as you normally would.

```
julia> results = SimulationResults("<output-dir>/<simulation-name>")
```

Note the log files and results for each partition are located in
`<output-dir>/<simulation-name>/simulation_partitions`

If any partition fails, the join command fails and records a failed status in
`<output-dir>/<simulation-name>/results/status.json`. Each failed partition,
along with the simulation steps that it covers, is logged in
`<output-dir>/<simulation-name>/logs/join_partitioned_simulation.log`.
Re-run the command with the `--skip-failures` option to skip the failed partitions and merge
the results of the successful ones. The status of the simulation remains a failure, so the
results must be loaded with `SimulationResults(path; ignore_status=true)`.

```
$ julia --project=<path> my_simulation.jl join --simulation-name=<simulation-name> --output-dir=<output-dir> --skip-failures
```
