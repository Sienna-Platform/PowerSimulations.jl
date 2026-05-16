function check_folder_integrity(folder::String)
    folder_files = readdir(folder)
    alien_files = setdiff(folder_files, KNOWN_SIMULATION_PATHS)
    alien_files = filter(x -> !any(occursin.(IGNORABLE_FILES, x)), alien_files)
    if isempty(alien_files)
        return true
    else
        @warn "Unrecognized simulation files: $(sort(alien_files))"
    end
    if "data_store" ∉ folder_files
        error("The file path doesn't contain any data_store folder")
    end
    return false
end

struct SimulationResults
    path::String
    params::SimulationStoreParams
    decision_problem_results::Dict{
        String,
        SimulationProblemResults{DecisionModelSimulationResults},
    }
    emulation_problem_results::SimulationProblemResults{EmulationModelSimulationResults}
    store::Union{Nothing, SimulationStore}
end

function SimulationResults(path::AbstractString, execution = nothing; ignore_status = false)
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
        return SimulationResults(path, name, execution; ignore_status = ignore_status)
    end

    if "data_store" in readdir(path)
        return SimulationResults(
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
Construct SimulationResults from a simulation output directory.

# Arguments

  - `path::AbstractString`: Simulation output directory
  - `name::AbstractString`: Simulation name
  - `execution::AbstractString`: Execution number. Default is the most recent.
  - `ignore_status::Bool`: If true, return results even if the simulation failed.
"""
function SimulationResults(
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
    status = deserialize_status(joinpath(execution_path, RESULTS_DIR))
    _check_status(status, ignore_status)

    if !check_folder_integrity(execution_path)
        @warn "The results folder $(execution_path) is not consistent with the default folder structure. " *
              "This can lead to errors or unwanted results."
    end

    simulation_store_path = joinpath(execution_path, "data_store")
    check_file_integrity(simulation_store_path)

    return open_store(HdfSimulationStore, simulation_store_path, "r") do store
        decision_problem_results =
            Dict{String, SimulationProblemResults{DecisionModelSimulationResults}}()
        sim_params = get_params(store)
        container_key_lookup = get_container_key_lookup(store)
        for (name, problem_params) in sim_params.decision_models_params
            name = string(name)
            problem_result = SimulationProblemResults(
                DecisionModel,
                store,
                name,
                problem_params,
                sim_params,
                execution_path,
                container_key_lookup,
            )
            decision_problem_results[name] = problem_result
        end

        emulation_result = SimulationProblemResults(
            EmulationModel,
            store,
            string(first(keys(sim_params.emulation_model_params))),
            first(values(sim_params.emulation_model_params)),
            sim_params,
            execution_path,
            container_key_lookup,
        )

        return SimulationResults(
            execution_path,
            sim_params,
            decision_problem_results,
            emulation_result,
            nothing,
        )
    end
end

"""
Construct SimulationResults from a simulation.
"""
function SimulationResults(sim::Simulation; ignore_status = false, kwargs...)
    _check_status(get_simulation_status(sim), ignore_status)
    store = get_simulation_store(sim)
    execution_path = get_simulation_dir(sim)
    decision_problem_results =
        Dict{String, SimulationProblemResults{DecisionModelSimulationResults}}()
    sim_params = get_params(store)
    models = get_models(sim)
    container_key_lookup = get_container_key_lookup(store)
    for (name, problem_params) in sim_params.decision_models_params
        model = get_simulation_model(models, name)
        name = string(name)
        problem_result = SimulationProblemResults(
            DecisionModel,
            store,
            name,
            problem_params,
            sim_params,
            execution_path,
            container_key_lookup;
            system = get_system(model),
        )
        decision_problem_results[name] = problem_result
    end

    emulation_model = get_emulation_model(models)
    emulation_results = SimulationProblemResults(
        EmulationModel,
        store,
        string(first(keys(sim_params.emulation_model_params))),
        first(values(sim_params.emulation_model_params)),
        sim_params,
        execution_path,
        container_key_lookup;
        system = isnothing(emulation_model) ? nothing : get_system(emulation_model),
    )

    return SimulationResults(
        execution_path,
        sim_params,
        decision_problem_results,
        emulation_results,
        store,
    )
end

"""
    Base.empty!(res::SimulationResults)

Empty the [`SimulationResults`](@ref)
"""
function Base.empty!(res::SimulationResults)
    foreach(empty!, values(res.decision_problem_results))
    empty!(res.emulation_problem_results)
end

Base.isempty(res::SimulationResults) = all(isempty, values(res.decision_problem_results))
Base.length(res::SimulationResults) =
    mapreduce(length, +, values(res.decision_problem_results))
get_exports_folder(x::SimulationResults) = joinpath(x.path, "exports")

"""
Return SimulationProblemResults corresponding to a SimulationResults

# Arguments
 - `sim_results::PSI.SimulationResults`: the simulation results to read from
 - `problem::String`: the name of the problem (e.g., "UC", "ED")
 - `populate_system::Bool = true`: whether to set the results' system as if using
   [`get_system!`](@ref)
 - `populate_units::Union{IS.UnitSystem, String, Nothing} = IS.UnitSystem.NATURAL_UNITS`:
   the units system with which to populate the results' system, if any (requires
   `populate_system=true`)
"""
function get_decision_problem_results(
    results::SimulationResults,
    problem::String;
    populate_system::Bool = false,
    populate_units::Union{IS.UnitSystem, String, Nothing} = nothing,
)
    if !haskey(results.decision_problem_results, problem)
        throw(IS.InvalidValue("$problem is not stored"))
    end

    results = results.decision_problem_results[problem]
    _populate_system_in_results!(results, populate_system, populate_units)

    return results
end

"""
Return SimulationProblemResults corresponding to a SimulationResults

# Arguments
 - `sim_results::PSI.SimulationResults`: the simulation results to read from
 - `populate_system::Bool = true`: whether to set the results' system as if using
   [`get_system!`](@ref)
 - `populate_units::Union{IS.UnitSystem, String, Nothing} = IS.UnitSystem.NATURAL_UNITS`:
   the units system with which to populate the results' system, if any (requires
   `populate_system=true`)
"""
function get_emulation_problem_results(
    results::SimulationResults;
    populate_system::Bool = false,
    populate_units::Union{IS.UnitSystem, String, Nothing} = nothing,
)
    results = results.emulation_problem_results
    _populate_system_in_results!(results, populate_system, populate_units)
    return results
end

function _populate_system_in_results!(
    results::SimulationProblemResults,
    populate_system::Bool,
    populate_units::Union{IS.UnitSystem, String, Nothing},
)
    if populate_system
        try
            get_system!(results)
        catch e
            error("Can't find the system file or retrieve the system error=$e")
        end

        if !isnothing(populate_units)
            PSY.set_units_base_system!(PSI.get_system(results), populate_units)
        else
            PSY.set_units_base_system!(PSI.get_system(results), IS.UnitSystem.NATURAL_UNITS)
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
list_decision_problems(results::SimulationResults) =
    collect(keys(results.decision_problem_results))

"""
Export results to files in the results directory.

# Arguments

  - `results::SimulationResults`: simulation results
  - `exports`: SimulationResultsExport or anything that can be passed to its constructor.
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
function export_results(results::SimulationResults, exports)
    if results.store isa InMemorySimulationStore
        export_results(results, exports, results.store)
    else
        simulation_store_path = joinpath(results.path, "data_store")
        open_store(HdfSimulationStore, simulation_store_path, "r") do store
            export_results(results, exports, store)
        end
    end
    return
end

function export_results(results::SimulationResults, exports, store::SimulationStore)
    if !(exports isa SimulationResultsExport)
        exports = SimulationResultsExport(exports, results.params)
    end

    file_type = get_export_file_type(exports)

    for problem_results in values(results.decision_problem_results)
        problem_exports = get_problem_exports(exports, problem_results.problem)
        path =
            isnothing(exports.path) ? problem_results.results_output_folder : exports.path
        for timestamp in get_timestamps(problem_results)
            !should_export(exports, timestamp) && continue

            export_path = mkpath(joinpath(path, problem_results.problem, "variables"))
            for name in list_variable_names(problem_results)
                if should_export_variable(problem_exports, name)
                    dfs = read_variable(
                        problem_results,
                        name;
                        initial_time = timestamp,
                        count = 1,
                        store = store,
                    )
                    ISOPT.export_result(
                        file_type,
                        export_path,
                        name,
                        timestamp,
                        dfs[timestamp],
                    )
                end
            end

            export_path = mkpath(joinpath(path, problem_results.problem, "aux_variables"))
            for name in list_aux_variable_names(problem_results)
                if should_export_aux_variable(problem_exports, name)
                    dfs = read_aux_variable(
                        problem_results,
                        name;
                        initial_time = timestamp,
                        count = 1,
                        store = store,
                    )
                    ISOPT.export_result(
                        file_type,
                        export_path,
                        name,
                        timestamp,
                        dfs[timestamp],
                    )
                end
            end

            export_path = mkpath(joinpath(path, problem_results.problem, "parameters"))
            for name in list_parameter_names(problem_results)
                if should_export_parameter(problem_exports, name)
                    dfs = read_parameter(
                        problem_results,
                        name;
                        initial_time = timestamp,
                        count = 1,
                        store = store,
                    )
                    ISOPT.export_result(
                        file_type,
                        export_path,
                        name,
                        timestamp,
                        dfs[timestamp],
                    )
                end
            end

            export_path = mkpath(joinpath(path, problem_results.problem, "duals"))
            for name in list_dual_names(problem_results)
                if should_export_dual(problem_exports, name)
                    dfs = read_dual(
                        problem_results,
                        name;
                        initial_time = timestamp,
                        count = 1,
                        store = store,
                    )
                    ISOPT.export_result(
                        file_type,
                        export_path,
                        name,
                        timestamp,
                        dfs[timestamp],
                    )
                end
            end
        end

        export_path = mkpath(joinpath(path, problem_results.problem, "expression"))
        for name in list_expression_names(problem_results)
            if should_export_expression(problem_exports, name)
                dfs = read_expression(
                    problem_results,
                    name;
                    initial_time = timestamp,
                    count = 1,
                    store = store,
                )
                ISOPT.export_result(
                    file_type,
                    export_path,
                    name,
                    timestamp,
                    dfs[timestamp],
                )
            end
        end

        if problem_exports.optimizer_stats
            export_path = joinpath(path, problem_results.problem, "optimizer_stats.csv")
            df = read_optimizer_stats(problem_results; store = store)
            ISOPT.export_result(file_type, export_path, df)
        end
    end
    return
end

function _check_status(status::RunStatus, ignore_status)
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
