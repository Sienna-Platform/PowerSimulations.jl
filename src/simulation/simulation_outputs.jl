function check_folder_integrity(folder::String)
    folder_files = readdir(folder)
    alien_files = setdiff(folder_files, KNOWN_SIMULATION_PATHS)
    alien_files = filter(x -> !any(occursin.(IGNORABLE_FILES, x)), alien_files)
    if isempty(alien_files)
        return true
    else
        @warn "Unrecognized simulation files: $(sort(alien_files))"
    end
    if STORE_DIR ∉ folder_files
        error("The file path doesn't contain any data_store folder")
    end
    return false
end

struct SimulationOutputs
    path::String
    params::SimulationStoreParams
    decision_problem_outputs::Dict{
        String,
        SimulationProblemOutputs{DecisionModelSimulationOutputs},
    }
    emulation_problem_outputs::SimulationProblemOutputs{EmulationModelSimulationOutputs}
    store::Union{Nothing, SimulationStore}
end

function SimulationOutputs(path::AbstractString, execution = nothing; ignore_status = false)
    # This method maintains compatibility with the old interface as long as there is only
    # one simulation name.
    unique_names = Set{String}()
    for name in readdir(path)
        m = match(r"(.*)-\d+$", name)
        if isnothing(m)
            push!(unique_names, name)
        else
            push!(unique_names, m.captures[1])
        end
    end

    if length(unique_names) == 1
        name = first(unique_names)
        return SimulationOutputs(path, name, execution; ignore_status = ignore_status)
    end

    if STORE_DIR in readdir(path)
        return SimulationOutputs(
            dirname(path),
            basename(path),
            execution;
            ignore_status = ignore_status,
        )
    end

    error(
        "Found more than one simulation name in $path. Please call the constructor that includes 'name.'",
    )
end

"""
Construct SimulationOutputs from a simulation output directory.

# Arguments

  - `path::AbstractString`: Simulation output directory
  - `name::AbstractString`: Simulation name
  - `execution::AbstractString`: Execution number. Default is the most recent.
  - `ignore_status::Bool`: If true, return outputs even if the simulation failed.
"""
_emulation_system(::Nothing) = nothing
_emulation_system(model::EmulationModel) = get_system(model)

function SimulationOutputs(
    path::AbstractString,
    name::AbstractString,
    execution = nothing;
    ignore_status = false,
)
    if isnothing(execution)
        execution = _get_most_recent_execution(path, name)
    end
    if execution == 1
        execution_path = joinpath(path, name)
    else
        execution_path = joinpath(path, "$name-$execution")
    end
    if !isdir(execution_path)
        error("No valid simulation in $execution_path: execution = $execution")
    end

    @info "Loading simulation results from $execution_path"
    status = deserialize_status(joinpath(execution_path, OUTPUTS_DIR))
    _check_status(status, ignore_status)

    if !check_folder_integrity(execution_path)
        @warn "The results folder $(execution_path) is not consistent with the default folder structure. " *
              "This can lead to errors or unwanted results."
    end

    simulation_store_path = joinpath(execution_path, STORE_DIR)
    check_file_integrity(simulation_store_path)

    return open_store(HdfSimulationStore, simulation_store_path, "r") do store
        decision_problem_outputs =
            Dict{String, SimulationProblemOutputs{DecisionModelSimulationOutputs}}()
        sim_params = get_params(store)
        container_key_lookup = get_container_key_lookup(store)
        # Shared across every decision and emulation result below (R30/R31): one System
        # loaded through any of them becomes readable by all the others via
        # `_register_borrowed_stores!`.
        system_registry = Dict{Base.UUID, POM.ParameterTimeSeriesStore}()
        for (name, problem_params) in sim_params.decision_models_params
            name = string(name)
            problem_output = SimulationProblemOutputs(
                DecisionModel,
                store,
                name,
                problem_params,
                sim_params,
                execution_path,
                container_key_lookup;
                system = nothing,
                system_registry = system_registry,
            )
            decision_problem_outputs[name] = problem_output
        end

        em_params = get_emulation_model_params(sim_params)
        emulation_output = SimulationProblemOutputs(
            EmulationModel,
            store,
            string(first(keys(sim_params.emulation_model_params))),
            em_params,
            sim_params,
            execution_path,
            container_key_lookup;
            system = nothing,
            system_registry = system_registry,
        )

        return SimulationOutputs(
            execution_path,
            sim_params,
            decision_problem_outputs,
            emulation_output,
            nothing,
        )
    end
end

"""
Construct SimulationOutputs from a simulation.
"""
function SimulationOutputs(sim::Simulation; ignore_status = false, kwargs...)
    _check_status(get_simulation_status(sim), ignore_status)
    store = get_simulation_store(sim)
    execution_path = get_simulation_dir(sim)
    decision_problem_outputs =
        Dict{String, SimulationProblemOutputs{DecisionModelSimulationOutputs}}()
    sim_params = get_params(store)
    models = get_models(sim)
    container_key_lookup = get_container_key_lookup(store)
    # Shared across every decision and emulation result below (R30/R31): one System
    # loaded through any of them becomes readable by all the others via
    # `_register_borrowed_stores!`. Constructed empty here -- the `system` kwarg below is
    # each model's live, in-memory System, which predates `finalize_parameters!` and is not
    # registered (see `set_system!`); only a later `get_system!` reload populates this.
    system_registry = Dict{Base.UUID, POM.ParameterTimeSeriesStore}()
    for (name, problem_params) in sim_params.decision_models_params
        model = get_simulation_model(models, name)
        name = string(name)
        problem_output = SimulationProblemOutputs(
            DecisionModel,
            store,
            name,
            problem_params,
            sim_params,
            execution_path,
            container_key_lookup;
            system = get_system(model),
            system_registry = system_registry,
        )
        decision_problem_outputs[name] = problem_output
    end

    emulation_model = get_emulation_model(models)
    emulation_outputs = SimulationProblemOutputs(
        EmulationModel,
        store,
        string(first(keys(sim_params.emulation_model_params))),
        first(values(sim_params.emulation_model_params)),
        sim_params,
        execution_path,
        container_key_lookup;
        system = _emulation_system(emulation_model),
        system_registry = system_registry,
    )

    return SimulationOutputs(
        execution_path,
        sim_params,
        decision_problem_outputs,
        emulation_outputs,
        store,
    )
end

"""
    Base.empty!(res::SimulationOutputs)

Empty the [`SimulationOutputs`](@ref)
"""
function Base.empty!(res::SimulationOutputs)
    foreach(empty!, values(res.decision_problem_outputs))
    empty!(res.emulation_problem_outputs)
end

Base.isempty(res::SimulationOutputs) = all(isempty, values(res.decision_problem_outputs))
Base.length(res::SimulationOutputs) =
    mapreduce(length, +, values(res.decision_problem_outputs))

"""
Return SimulationProblemOutputs corresponding to a SimulationOutputs

# Arguments
 - `sim_outputs::PSI.SimulationOutputs`: the simulation outputs to read from
 - `problem::String`: the name of the problem (e.g., "UC", "ED")
 - `populate_system::Bool = true`: whether to set the outputs' system as if using
   [`get_system!`](@ref)
 - `populate_units::Union{IS.UnitSystem.Value, String, Nothing} = nothing`: unsupported;
   PowerSystems (psy6) has no system-wide unit base, so passing a non-`nothing`
   value throws (requires `populate_system=true`)
"""
function get_decision_problem_outputs(
    results::SimulationOutputs,
    problem::String;
    populate_system::Bool = false,
    populate_units::Union{IS.UnitSystem.Value, String, Nothing} = nothing,
)
    if !haskey(results.decision_problem_outputs, problem)
        throw(IS.InvalidValue("$problem is not stored"))
    end

    results = results.decision_problem_outputs[problem]
    _populate_system_in_outputs!(results, populate_system, populate_units)

    return results
end

"""
Return SimulationProblemOutputs corresponding to a SimulationOutputs

# Arguments
 - `sim_outputs::PSI.SimulationOutputs`: the simulation outputs to read from
 - `populate_system::Bool = true`: whether to set the outputs' system as if using
   [`get_system!`](@ref)
 - `populate_units::Union{IS.UnitSystem.Value, String, Nothing} = nothing`: unsupported;
   PowerSystems (psy6) has no system-wide unit base, so passing a non-`nothing`
   value throws (requires `populate_system=true`)
"""
function get_emulation_problem_outputs(
    results::SimulationOutputs;
    populate_system::Bool = false,
    populate_units::Union{IS.UnitSystem.Value, String, Nothing} = nothing,
)
    results = results.emulation_problem_outputs
    _populate_system_in_outputs!(results, populate_system, populate_units)
    return results
end

function _populate_system_in_outputs!(
    results::SimulationProblemOutputs,
    populate_system::Bool,
    populate_units::Union{IS.UnitSystem.Value, String, Nothing},
)
    if populate_system
        try
            get_system!(results)
        catch e
            error("Can't find the system file or retrieve the system error=$e")
        end

        # PowerSystems (psy6) removed the system-wide unit-base mode this used to set via
        # `set_units_base_system!`; getters now take an explicit unit system (PSY.SU/DU/NU)
        # per call. Error loudly rather than silently ignoring a caller's request.
        if !isnothing(populate_units)
            error(
                "populate_units is not supported: PowerSystems no longer has a system-wide " *
                "unit base. Pass the desired unit system explicitly to each accessor instead " *
                "(e.g. PSY.get_rating(component, PSY.SU)).",
            )
        end

    else
        (isnothing(populate_units)) ||
            throw(
                ArgumentError(
                    "populate_units=$populate_units is unaccepted when populate_system=$populate_system",
                ),
            )
    end
    return
end

"""
Return the problem names in the simulation.
"""
list_decision_problems(results::SimulationOutputs) =
    collect(keys(results.decision_problem_outputs))

"""
Export outputs to files in the outputs directory.

# Arguments

  - `results::SimulationOutputs`: simulation outputs
  - `exports`: SimulationOutputsExport or anything that can be passed to its constructor.
    (such as Dict or path to JSON file)

An example JSON file demonstrating possible options is below. Note that `start_time`,
`end_time`, `path`, and `format` are optional.

```
{
  "decision_models": [
    {
      "name": "ED",
      "variables": [
        "P__ThermalStandard",
      ],
      "parameters": [
        "all"
      ]
    },
    {
      "name": "UC",
      "variables": [
        "On__ThermalStandard"
      ],
      "parameters": [
        "all"
      ],
      "duals": [
        "all"
      ]
    }
  ],
  "start_time": "2020-01-01T04:00:00",
  "end_time": null,
  "path": null,
  "format": "csv"
}

```
"""
function IOM.export_outputs(results::SimulationOutputs, exports)
    _export_outputs_with_store(results, exports, results.store)
    return
end

_export_outputs_with_store(results, exports, store::InMemorySimulationStore) =
    IOM.export_outputs(results, exports, store)
function _export_outputs_with_store(results, exports, ::Union{Nothing, HdfSimulationStore})
    _open_outputs_store(results.path) do store
        IOM.export_outputs(results, exports, store)
    end
    return
end

function IOM.export_outputs(results::SimulationOutputs, exports, store::SimulationStore)
    exports = _as_outputs_export(exports, results.params)
    file_type = get_export_file_type(exports)

    for problem_outputs in values(results.decision_problem_outputs)
        problem_exports = get_problem_exports(exports, problem_outputs.problem)
        if isnothing(exports.path)
            path = problem_outputs.output_dir
        else
            path = exports.path
        end
        for timestamp in get_timestamps(problem_outputs)
            !should_export(exports, timestamp) && continue
            for (folder, list_names, should_export_fn, read_fn) in (
                ("variables", list_variable_names, should_export_variable, read_variable),
                (
                    "aux_variables",
                    list_aux_variable_names,
                    should_export_aux_variable,
                    read_aux_variable,
                ),
                (
                    "parameters",
                    list_parameter_names,
                    should_export_parameter,
                    read_parameter,
                ),
                ("duals", list_dual_names, should_export_dual, read_dual),
                (
                    "expression",
                    list_expression_names,
                    should_export_expression,
                    read_expression,
                ),
            )
                export_path = mkpath(joinpath(path, problem_outputs.problem, folder))
                for name in list_names(problem_outputs)
                    should_export_fn(problem_exports, name) || continue
                    dfs = read_fn(
                        problem_outputs,
                        name;
                        start_time = timestamp,
                        len = 1,
                        store = store,
                    )
                    export_output(file_type, export_path, name, timestamp, dfs[timestamp])
                end
            end
        end

        if problem_exports.optimizer_stats
            export_path = joinpath(path, problem_outputs.problem, "optimizer_stats.csv")
            df = read_optimizer_stats(problem_outputs; store = store)
            export_output(file_type, export_path, df)
        end
    end
    return
end

function _check_status(status::RunStatus.Value, ignore_status)
    status == RunStatus.SUCCESSFULLY_FINALIZED && return

    if ignore_status
        @warn "Simulation was not successful: $status. Results may not be valid."
    else
        error(
            "Simulation was not successful: status = $status. Set ignore_status = true to override.",
        )
    end
    return
end
