# Failure-handling contract for partitioned simulations (run_parallel_simulation and
# join_simulation):
#
# 1. If any partition job fails, status.json of the joined simulation records
#    RunStatus.FAILED.
# 2. The exception that caused a failure always reaches the caller. Status recording and
#    cleanup inside catch blocks are best-effort: guarded by one try/catch that logs and
#    continues, and never allowed to replace the original exception.
# 3. A missing partition status.json means that the partition job failed. Any other error
#    while reading partition results (permissions, parse errors, wrong paths) indicates a
#    problem with this process or environment and propagates instead of being reclassified
#    as a partition failure. The one exception: with skip_failures = true, a partition
#    store file that cannot be opened is skipped, because recovering from corrupted
#    partition outputs is the purpose of that flag.
# 4. InterruptException is a user action, not a failure: it propagates without recording
#    RunStatus.FAILED.
# 5. The datasets of a partition store must exactly match the merged store; a mismatch in
#    either direction is an error.
# 6. Failure paths are guarded exactly one level deep. If a guarded best-effort step
#    itself fails, that is logged and otherwise out of scope.

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
end

function SimulationPartitionResults(path::AbstractString)
    config_file = joinpath(path, "simulation_partitions", "config.json")
    config = open(config_file, "r") do io
        JSON3.read(io, Dict)
    end
    partitions = IS.deserialize(SimulationPartitions, config)
    return SimulationPartitionResults(path, basename(path), partitions)
end

"""
Combine all partition simulation files and return the status of the joined simulation.

Throw an exception if any partition job failed, unless `skip_failures` is `true`.

# Arguments

  - `path::AbstractString`: Directory of the main simulation.
  - `skip_failures::Bool`: If `true`, log and skip the store files of the partition jobs
    that failed or cannot be opened and merge the results of the successful jobs. The
    status of the joined simulation is `RunStatus.FAILED` whenever any partition job
    failed, regardless of this setting.
"""
function join_simulation(path::AbstractString; skip_failures = false)
    results = SimulationPartitionResults(path)
    return join_simulation(results; skip_failures = skip_failures)
end

function join_simulation(results::SimulationPartitionResults; skip_failures = false)
    failed_partitions = _check_jobs(results)
    if !isempty(failed_partitions) && !skip_failures
        _try_serialize_failed_status(joinpath(results.path, RESULTS_DIR))
        error(
            "These partition jobs were not successful: $failed_partitions. " *
            "Refer to the log messages above for the affected simulation steps. " *
            "Pass skip_failures = true (--skip-failures on the command line) to skip the " *
            "failed jobs and merge the results of the successful jobs.",
        )
    end

    not_merged = try
        _merge_store_files!(results, Set(failed_partitions), skip_failures)
    catch
        # Best effort to keep the outputs usable: record the failure and, because the
        # merge may have partially completed, recompute the store file hash so that
        # SimulationResults(path; ignore_status = true) still works. Nothing here may
        # mask the original exception.
        try
            _complete(results, RunStatus.FAILED)
        catch cleanup_e
            @error "Failed to finalize the results of the failed join" exception =
                (cleanup_e, catch_backtrace())
        end
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
Return the indexes of the partition jobs that were not successful. Log an error message
for each one of them.
"""
function _check_jobs(results::SimulationPartitionResults)
    failed_jobs = Int[]
    for i in 1:get_num_partitions(results.partitions)
        status_dir = joinpath(_partition_path(results, i), RESULTS_DIR)
        # A missing status file means that the job died before recording its status. Any
        # other error reading a status (permissions, parse errors) indicates a problem
        # with this process or environment, not with the partition job, and propagates
        # instead of marking the partition failed.
        if isfile(_status_file_path(status_dir))
            status = deserialize_status(status_dir)
        else
            @error "Partition job index = $i did not record a status; it may have died " *
                   "before completing."
            status = RunStatus.FAILED
        end
        if status != RunStatus.SUCCESSFULLY_FINALIZED
            @error "Partition job index = $i was not successful: status = $status. " *
                   "Results for steps = $(_valid_step_range(results, i)) will be invalid " *
                   "in the merged store."
            push!(failed_jobs, i)
        end
    end

    return failed_jobs
end

"""
Merge the store files of all partitions into the main store file and return the indexes of
the partitions that were not merged.

Partitions in `skip_indexes` are never merged. If `skip_failures` is `true`, log and skip
the partitions whose store files cannot be opened; otherwise, propagate the exception.
Errors raised after a store file has been opened always propagate.
"""
function _merge_store_files!(
    results::SimulationPartitionResults,
    skip_indexes::Set{Int},
    skip_failures::Bool,
)
    not_merged = Int[]
    HDF5.h5open(_store_path(results), "r+") do dst
        for i in 1:get_num_partitions(results.partitions)
            if i in skip_indexes
                @warn "Skip the store file of the failed partition job index = $i. " *
                      "Results for steps = $(_valid_step_range(results, i)) will be " *
                      "invalid in the merged store."
                push!(not_merged, i)
                continue
            end
            src = try
                HDF5.h5open(joinpath(_partition_path(results, i), _store_subpath()), "r")
            catch e
                (!skip_failures || e isa InterruptException) && rethrow()
                push!(not_merged, i)
                @error "Failed to open the store file of partition job index = $i. " *
                       "The file is missing or corrupted. Results for " *
                       "steps = $(_valid_step_range(results, i)) will be invalid in " *
                       "the merged store." exception = (e, catch_backtrace())
                continue
            end
            try
                _copy_datasets!(results, i, src, dst)
            finally
                close(src)
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

    # A dataset present in only one of the stores means that the partition was built
    # differently than the main simulation; merging would silently produce incomplete
    # results in one direction and leave a region of the destination unwritten in the
    # other.
    function check_matching_names(group_path)
        src_names = sort!(keys(src[group_path]))
        dst_names = sort!(keys(dst[group_path]))
        if src_names != dst_names
            error(
                "The partition store and the merged store have different contents at " *
                "$group_path: partition = $src_names, merged = $dst_names. The " *
                "partition was likely built with different inputs than the main " *
                "simulation.",
            )
        end
    end

    function process_dataset(dst_dataset, merge_func)
        name = HDF5.name(dst_dataset)
        if !endswith(name, "__columns")
            merge_func(results, index, src[name], dst_dataset)
        end
    end

    check_matching_names("simulation/decision_models")
    for dst_group in dst["simulation/decision_models"]
        group_name = HDF5.name(dst_group)
        check_matching_names(group_name)
        for output_type in output_types
            check_matching_names("$group_name/$output_type")
            for dst_dataset in dst_group[output_type]
                process_dataset(dst_dataset, _merge_dataset_rows!)
            end
        end
        process_dataset(dst_group["optimizer_stats"], _merge_dataset_rows!)
    end

    for output_type in output_types
        check_matching_names("simulation/emulation_model/$output_type")
        for dst_dataset in dst["simulation/emulation_model/$output_type"]
            process_dataset(dst_dataset, _merge_dataset_columns!)
        end
    end
end

"""
Return the ranges of the source and destination datasets in the step dimension for merging
the partition with the given index. `src_size` and `dst_size` are the sizes of the
datasets in that dimension.
"""
function _merge_ranges(
    results::SimulationPartitionResults,
    index::Int,
    src_size::Int,
    dst_size::Int,
)
    step_range = get_absolute_step_range(results.partitions, index)
    IS.@assert_op src_size % length(step_range) == 0
    per_step = src_size ÷ length(step_range)
    # Guarantees that the absolute destination offsets computed below are in bounds and
    # consistent across all partitions.
    IS.@assert_op per_step * results.partitions.num_steps == dst_size
    # Compute both offsets from the same absolute range of valid steps so that they
    # cannot drift apart, and compute the destination offset from absolute steps rather
    # than from a running write position so that a skipped (corrupted) partition does
    # not shift the data of subsequent partitions.
    valid_range = _valid_step_range(results, index)
    len = length(valid_range) * per_step
    src_start = 1 + per_step * (first(valid_range) - first(step_range))
    dst_start = (first(valid_range) - 1) * per_step + 1
    return (src_start:(src_start + len - 1), dst_start:(dst_start + len - 1))
end

# Emulation model datasets grow along the first dimension; decision model datasets grow
# along the last dimension.
_merge_dataset_columns!(results::SimulationPartitionResults, index, src, dst) =
    _merge_dataset!(results, index, src, dst, 1, (2,))
_merge_dataset_rows!(results::SimulationPartitionResults, index, src, dst) =
    _merge_dataset!(results, index, src, dst, ndims(dst), (2, 3))

function _merge_dataset!(
    results::SimulationPartitionResults,
    index,
    src,
    dst,
    step_dim::Int,
    supported_ndims,
)
    IS.@assert_op ndims(src) == ndims(dst)
    ndims(dst) in supported_ndims || error("Unsupported dataset ndims: $(ndims(dst))")
    for dim in 1:ndims(dst)
        dim == step_dim && continue
        IS.@assert_op size(src)[dim] == size(dst)[dim]
    end
    src_range, dst_range =
        _merge_ranges(results, index, size(src)[step_dim], size(dst)[step_dim])
    src_indexes = ntuple(d -> d == step_dim ? src_range : Colon(), ndims(dst))
    dst_indexes = ntuple(d -> d == step_dim ? dst_range : Colon(), ndims(dst))
    dst[dst_indexes...] = src[src_indexes...]
    return
end

function _complete(results::SimulationPartitionResults, status)
    serialize_status(status, joinpath(results.path, RESULTS_DIR))
    store_path = _store_path(results)
    # The store may not exist if the merge failed before it could be opened.
    isfile(store_path) && IS.compute_file_hash(dirname(store_path), basename(store_path))
    return
end
