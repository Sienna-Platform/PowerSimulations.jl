const _TEMP_WRITE_POSITION = "__write_position__"

"""
Handles merging of simulation partitions
"""
struct SimulationPartitionResults
    "Directory of main simulation"
    path::String
    "User-defined simulation name"
    simulation_name::String
    "Defines how the simulation is split into partitions"
    partitions::SimulationPartitions
    "Cache of datasets"
    datasets::Dict{String, HDF5.Dataset}
end

function SimulationPartitionResults(path::AbstractString)
    config_file = joinpath(path, "simulation_partitions", "config.json")
    config = open(config_file, "r") do io
        JSON3.read(io, Dict)
    end
    partitions = IS.deserialize(SimulationPartitions, config)
    return SimulationPartitionResults(
        path,
        basename(path),
        partitions,
        Dict{String, HDF5.Dataset}(),
    )
end

"""
Combine all partition simulation files and return the status of the joined simulation.

Throw an exception if any partition job failed, unless `skip_failures` is `true`.

# Arguments

  - `path::AbstractString`: Directory of the main simulation.
  - `skip_failures::Bool`: If `true`, log and skip the store files of the partition jobs
    that failed and merge the results of the successful jobs. The status of the joined
    simulation is `RunStatus.FAILED` whenever any partition job failed, regardless of this
    setting.
"""
function join_simulation(path::AbstractString; skip_failures = false)
    results = SimulationPartitionResults(path)
    return join_simulation(results; skip_failures = skip_failures)
end

function join_simulation(results::SimulationPartitionResults; skip_failures = false)
    failed_partitions = _check_jobs(results)
    if !isempty(failed_partitions) && !skip_failures
        _serialize_failed_status(results)
        error(
            "These partition jobs were not successful: $(first.(failed_partitions)). " *
            "Refer to the log messages above for the affected simulation steps. " *
            "Pass skip_failures = true (--skip-failures on the command line) to skip the " *
            "failed jobs and merge the results of the successful jobs.",
        )
    end

    not_merged = try
        _merge_store_files!(results, Set{Int}(first.(failed_partitions)), skip_failures)
    catch
        _serialize_failed_status(results)
        rethrow()
    end

    status = if isempty(not_merged)
        RunStatus.SUCCESSFULLY_FINALIZED
    else
        RunStatus.FAILED
    end
    _complete(results, status)
    return status
end

function _partition_path(x::SimulationPartitionResults, i)
    partition_path = joinpath(x.path, "simulation_partitions", string(i))
    execution_no = _get_most_recent_execution(partition_path, x.simulation_name)
    if execution_no == 1
        execution_path = joinpath(partition_path, x.simulation_name)
    else
        execution_path = joinpath(partition_path, "$(x.simulation_name)-$execution_no")
    end
    return execution_path
end

_store_subpath() = joinpath("data_store", "simulation_store.h5")
_store_path(x::SimulationPartitionResults) = joinpath(x.path, _store_subpath())

"""
Return the absolute range of simulation steps that the partition with the given index
contributes to the merged store (excludes overlap steps).
"""
function _valid_step_range(x::SimulationPartitionResults, index::Int)
    step_range = get_absolute_step_range(x.partitions, index)
    first_step = step_range[get_valid_step_offset(x.partitions, index)]
    return first_step:(first_step + get_valid_step_length(x.partitions, index) - 1)
end

"""
Return the indexes and statuses of the partition jobs that were not successful. Log an
error message for each one of them.
"""
function _check_jobs(results::SimulationPartitionResults)
    failed_jobs = Vector{Tuple{Int, RunStatus}}()
    for i in 1:get_num_partitions(results.partitions)
        status = try
            deserialize_status(joinpath(_partition_path(results, i), RESULTS_DIR))
        catch e
            @error "Failed to read the status of partition job index = $i" exception =
                (e, catch_backtrace())
            RunStatus.FAILED
        end
        if status != RunStatus.SUCCESSFULLY_FINALIZED
            @error "Partition job index = $i was not successful: status = $status. " *
                   "Results for steps = $(_valid_step_range(results, i)) will be invalid " *
                   "in the merged store."
            push!(failed_jobs, (i, status))
        end
    end

    return failed_jobs
end

"""
Merge the store files of all partitions into the main store file and return the indexes of
the partitions that were not merged.

Partitions in `skip_indexes` are never merged. If `skip_failures` is `true`, log and skip
the partitions whose store files cannot be read; otherwise, propagate the exception.
"""
function _merge_store_files!(
    results::SimulationPartitionResults,
    skip_indexes::Set{Int},
    skip_failures::Bool,
)
    not_merged = Int[]
    HDF5.h5open(_store_path(results), "r+") do dst
        try
            for i in 1:get_num_partitions(results.partitions)
                if i in skip_indexes
                    @warn "Skip the store file of the failed partition job index = $i. " *
                          "Results for steps = $(_valid_step_range(results, i)) will be " *
                          "invalid in the merged store."
                    push!(not_merged, i)
                    continue
                end
                try
                    HDF5.h5open(
                        joinpath(_partition_path(results, i), _store_subpath()),
                        "r",
                    ) do src
                        _copy_datasets!(results, i, src, dst)
                    end
                catch e
                    (!skip_failures || e isa InterruptException) && rethrow()
                    push!(not_merged, i)
                    @error "Failed to merge the store file of partition job index = $i. " *
                           "The file is missing or corrupted. Results for " *
                           "steps = $(_valid_step_range(results, i)) will be invalid in " *
                           "the merged store." exception = (e, catch_backtrace())
                end
            end

            if isempty(not_merged)
                for dataset in values(results.datasets)
                    if occursin("decision_models", HDF5.name(dataset))
                        IS.@assert_op HDF5.attrs(dataset)[_TEMP_WRITE_POSITION] ==
                                      size(dataset)[end] + 1
                    else
                        IS.@assert_op HDF5.attrs(dataset)[_TEMP_WRITE_POSITION] ==
                                      size(dataset)[1] + 1
                    end
                end
            end
        finally
            # This runs on the error path as well, where the attribute may not be set for
            # every cached dataset. Deleting a missing attribute would mask the exception.
            for dataset in values(results.datasets)
                if haskey(HDF5.attrs(dataset), _TEMP_WRITE_POSITION)
                    delete!(HDF5.attrs(dataset), _TEMP_WRITE_POSITION)
                end
            end
        end
    end
    return not_merged
end

function _copy_datasets!(
    results::SimulationPartitionResults,
    index::Int,
    src::HDF5.File,
    dst::HDF5.File,
)
    output_types = string.(STORE_CONTAINERS)

    function process_dataset(src_dataset, merge_func)
        if !endswith(HDF5.name(src_dataset), "__columns")
            name = HDF5.name(src_dataset)
            dst_dataset = dst[name]
            if !haskey(results.datasets, name)
                results.datasets[name] = dst_dataset
                HDF5.attrs(dst_dataset)[_TEMP_WRITE_POSITION] = 1
            end
            merge_func(results, index, src_dataset, dst_dataset)
        end
    end

    for src_group in src["simulation/decision_models"]
        for output_type in output_types
            for src_dataset in src_group[output_type]
                process_dataset(src_dataset, _merge_dataset_rows!)
            end
        end
        process_dataset(src_group["optimizer_stats"], _merge_dataset_rows!)
    end

    for output_type in output_types
        for src_dataset in src["simulation/emulation_model/$output_type"]
            process_dataset(src_dataset, _merge_dataset_columns!)
        end
    end
end

function _merge_dataset_columns!(results::SimulationPartitionResults, index, src, dst)
    num_columns = size(src)[1]
    step_range = get_absolute_step_range(results.partitions, index)
    IS.@assert_op num_columns % length(step_range) == 0
    num_columns_per_step = num_columns ÷ length(step_range)
    skip_offset = get_valid_step_offset(results.partitions, index) - 1
    src_start = 1 + num_columns_per_step * skip_offset
    len = get_valid_step_length(results.partitions, index) * num_columns_per_step
    src_end = src_start + len - 1

    IS.@assert_op ndims(src) == ndims(dst)
    # Compute the destination offset from the partition's absolute step range rather
    # than from the running write position so that a skipped (corrupted) partition
    # does not shift the data of subsequent partitions.
    dst_start = (first(_valid_step_range(results, index)) - 1) * num_columns_per_step + 1
    if ndims(src) == 2
        IS.@assert_op size(src)[2] == size(dst)[2]
        dst_end = dst_start + len - 1
        dst[dst_start:dst_end, :] = src[src_start:src_end, :]
    else
        error("Unsupported dataset ndims: $(ndims(src))")
    end

    HDF5.attrs(dst)[_TEMP_WRITE_POSITION] = dst_end + 1
    return
end

function _merge_dataset_rows!(results::SimulationPartitionResults, index, src, dst)
    num_rows = size(src)[end]
    step_range = get_absolute_step_range(results.partitions, index)
    IS.@assert_op num_rows % length(step_range) == 0
    num_rows_per_step = num_rows ÷ length(step_range)
    skip_offset = get_valid_step_offset(results.partitions, index) - 1
    src_start = 1 + num_rows_per_step * skip_offset
    len = get_valid_step_length(results.partitions, index) * num_rows_per_step
    src_end = src_start + len - 1

    IS.@assert_op ndims(src) == ndims(dst)
    # See the comment about the destination offset in _merge_dataset_columns!.
    dst_start = (first(_valid_step_range(results, index)) - 1) * num_rows_per_step + 1
    if ndims(src) == 2
        IS.@assert_op size(src)[1] == size(dst)[1]
        dst_end = dst_start + len - 1
        dst[:, dst_start:dst_end] = src[:, src_start:src_end]
    elseif ndims(src) == 3
        IS.@assert_op size(src)[1] == size(dst)[1]
        IS.@assert_op size(src)[2] == size(dst)[2]
        dst_end = dst_start + len - 1
        IS.@assert_op dst_end <= size(dst)[3]
        dst[:, :, dst_start:dst_end] = src[:, :, src_start:src_end]
    else
        error("Unsupported dataset ndims: $(ndims(src))")
    end

    HDF5.attrs(dst)[_TEMP_WRITE_POSITION] = dst_end + 1
    return
end

function _complete(results::SimulationPartitionResults, status)
    serialize_status(status, joinpath(results.path, RESULTS_DIR))
    store_path = _store_path(results)
    IS.compute_file_hash(dirname(store_path), basename(store_path))
    return
end

"""
Record a failure in the status file of the joined simulation. Unlike [`_complete`](@ref),
this does not compute the hash of the store file because the store was not merged.
"""
function _serialize_failed_status(results::SimulationPartitionResults)
    serialize_status(RunStatus.FAILED, joinpath(results.path, RESULTS_DIR))
    return
end
