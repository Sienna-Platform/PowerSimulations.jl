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

_store_subpath() = joinpath(STORE_DIR, "simulation_store.h5")
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
    _merge_parameter_stores!(results, not_merged)
    return not_merged
end

function _copy_datasets!(
    results::SimulationPartitionResults,
    index::Int,
    src::HDF5.File,
    dst::HDF5.File,
)
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
        for output_type in STORE_CONTAINERS
            _copy_decision_model_container_datasets!(
                Val(output_type),
                group_name,
                dst_group,
                check_matching_names,
                process_dataset,
            )
        end
        process_dataset(dst_group["optimizer_stats"], _merge_dataset_rows!)
    end

    for output_type in STORE_CONTAINERS
        _copy_emulation_model_container_datasets!(
            Val(output_type),
            dst,
            check_matching_names,
            process_dataset,
        )
    end
    return nothing
end

"""
Parameters have no HDF5-backed dataset to copy: decision-model and emulation-model
parameters live in each bundle's InfraStore sidecar instead of an HDF5 dataset (Task 8+9),
merged separately by [`_merge_parameter_stores!`](@ref). Mirrors the same
`Val(STORE_CONTAINER_PARAMETERS)` skip `initialize_problem_storage!` already applies in
`hdf_simulation_store.jl`.
"""
_copy_decision_model_container_datasets!(
    ::Val{STORE_CONTAINER_PARAMETERS},
    ::String,
    dst_group,
    check_matching_names,
    process_dataset,
) = nothing

function _copy_decision_model_container_datasets!(
    ::Val{T},
    group_name::String,
    dst_group,
    check_matching_names,
    process_dataset,
) where {T}
    output_type = string(T)
    check_matching_names("$group_name/$output_type")
    for dst_dataset in dst_group[output_type]
        process_dataset(dst_dataset, _merge_dataset_rows!)
    end
    return nothing
end

_copy_emulation_model_container_datasets!(
    ::Val{STORE_CONTAINER_PARAMETERS},
    ::HDF5.File,
    check_matching_names,
    process_dataset,
) = nothing

function _copy_emulation_model_container_datasets!(
    ::Val{T},
    dst::HDF5.File,
    check_matching_names,
    process_dataset,
) where {T}
    output_type = string(T)
    check_matching_names("simulation/emulation_model/$output_type")
    for dst_dataset in dst["simulation/emulation_model/$output_type"]
        process_dataset(dst_dataset, _merge_dataset_columns!)
    end
    return nothing
end

"""
Merge every merged partition's realized parameter rows into the main simulation's bundle
sidecars. `_copy_datasets!` never touches parameters -- they live in each model's bundle
InfraStore, not an HDF5 dataset (Task 8+9) -- and the main simulation folder of a partitioned
run never executes, so its bundle sidecars start with zero parameter rows. This reads every
merged partition's bundle sidecar for every parameter key the model wrote, accumulates the
disjoint per-step windows/values across partitions (each partition covers its own steps), and
writes the combined result into the main bundle once per model. A skipped or failed partition
(`not_merged`, already warned about by `_merge_store_files!`) contributes nothing, exactly
like its region of the HDF5 store is left invalid.
"""
function _merge_parameter_stores!(
    results::SimulationPartitionResults,
    not_merged::Vector{Int},
)
    merged_indexes = setdiff(1:get_num_partitions(results.partitions), not_merged)
    isempty(merged_indexes) && return nothing

    open_store(HdfSimulationStore, joinpath(results.path, STORE_DIR), "r") do dst_store
        for model_name in keys(dst_store.params.decision_models_params)
            _merge_decision_model_parameters!(
                results,
                dst_store,
                model_name,
                merged_indexes,
            )
        end
        _merge_emulation_model_parameters!(results, dst_store, merged_indexes)
    end
    return nothing
end

"""
The bundle sidecar directory a partition's own build wrote for `model_name`, resolved from
that partition's own `HdfSimulationStore`. Each partition builds its own `System`, so its
bundle's uuid need not match the main folder's.
"""
function _partition_decision_bundle_dir(
    results::SimulationPartitionResults,
    index::Int,
    model_name::Symbol,
)
    partition_dir = joinpath(_partition_path(results, index), STORE_DIR)
    return open_store(HdfSimulationStore, partition_dir, "r") do src_store
        params = get_decision_model_params(src_store, model_name)
        return _bundle_dir(src_store, model_name, get_system_uuid(params))
    end
end

"""
The `(slice_id, read_features)` units to merge `key` under: one `("", base_features)` for a
2-D parameter (no `"axis2"` filter), or one `(label2, base_features + "axis2" => label2)` per
axis-2 slice for a 3-D parameter, discovered from `pstore` via `POM.parameter_slice_labels`.
`slice_id` is `""` only for the 2-D case (a 3-D parameter's own axis-2 labels are never empty
strings — `string(label2)` on a real axis label), so it doubles as the accumulator key without
a separate `Union{Nothing,...}` tag.
"""
function _decision_merge_units(
    pstore,
    key::IOM.ParameterKey,
    base_features::Dict{String, Any},
)::Vector{Tuple{String, Dict{String, Any}}}
    slice_labels = POM.parameter_slice_labels(pstore, key; extra_features = base_features)
    isempty(slice_labels) && return [("", base_features)]
    return [
        (label2, merge(base_features, Dict{String, Any}("axis2" => label2))) for
        label2 in slice_labels
    ]
end

"""
Merge one decision model's parameter windows across every partition in `merged_indexes` into
the main bundle, one source sidecar open at a time (InfraStore allows only one open handle
per store file per process) and the destination sidecar opened once, after every partition has
been read.

A 3-D (`axis2`-sliced) parameter is merged one slice at a time (`_decision_merge_units`):
`POM.read_parameter_windows` matches rows by a *subset* of features, so reading a key with
only the `"model"` feature would silently pull back every slice's rows and collapse them onto
the same axis-1 label (they share a name; only the `"axis2"` feature tells them apart) --
adding the slice's own `"axis2"` feature to both the read and the write keeps each slice in
its own accumulator entry and its own destination row.
"""
function _merge_decision_model_parameters!(
    results::SimulationPartitionResults,
    dst_store::HdfSimulationStore,
    model_name::Symbol,
    merged_indexes::Vector{Int},
)
    keys_to_merge =
        list_decision_model_keys(dst_store, model_name, STORE_CONTAINER_PARAMETERS)
    isempty(keys_to_merge) && return nothing

    base_features = Dict{String, Any}("model" => string(model_name))
    accumulated = Dict{
        Tuple{IOM.ParameterKey, String},
        Dict{String, Dict{Dates.DateTime, Vector{Float64}}},
    }()
    for index in merged_indexes
        bundle_dir = _partition_decision_bundle_dir(results, index, model_name)
        pstore = POM.open_parameter_store(joinpath(bundle_dir, PSY.TIME_SERIES_FILE))
        try
            for key in keys_to_merge
                for (slice_id, read_features) in
                    _decision_merge_units(pstore, key, base_features)
                    windows = POM.read_parameter_windows(
                        pstore,
                        key;
                        extra_features = read_features,
                    )
                    dest = get!(
                        accumulated,
                        (key, slice_id),
                        Dict{String, Dict{Dates.DateTime, Vector{Float64}}}(),
                    )
                    for (label, by_time) in windows
                        merge!(
                            get!(dest, label, Dict{Dates.DateTime, Vector{Float64}}()),
                            by_time,
                        )
                    end
                end
            end
        finally
            POM.close_parameter_store!(pstore)
        end
    end

    dst_params = get_decision_model_params(dst_store, model_name)
    dst_bundle_dir = _bundle_dir(dst_store, model_name, get_system_uuid(dst_params))
    pstore =
        POM.open_parameter_store_writable(joinpath(dst_bundle_dir, PSY.TIME_SERIES_FILE))
    try
        for ((key, slice_id), merged_by_label) in accumulated
            if isempty(slice_id)
                write_features = base_features
            else
                write_features =
                    merge(base_features, Dict{String, Any}("axis2" => slice_id))
            end
            new_by_label = _new_windows_by_label(
                merged_by_label,
                _existing_decision_windows(pstore, key, write_features),
            )
            isempty(new_by_label) && continue
            POM.write_parameter_windows!(
                pstore,
                key,
                _windows_from_labels(new_by_label),
                get_resolution(dst_params),
                get_interval(dst_params);
                extra_features = write_features,
            )
        end
    finally
        POM.close_parameter_store!(pstore)
    end
    return nothing
end

"""
The destination bundle's already-merged windows for `key`, or an empty `Dict` when this key
has never been merged into it before. `join_simulation` can be called more than once against
the same output directory (a failed partition retried, a `--skip-failures` re-run after a
corrupted store is fixed, ...), so a re-merge must add only what is not already there instead
of raising a duplicate-time-series error the second time. An empty result is how
`POM.read_parameter_windows` itself represents "no data for this key" -- it documents and
raises this on purpose rather than returning something falsy -- so this catches only that
exact, well-defined condition and re-raises anything else.
"""
function _existing_decision_windows(
    pstore,
    key::IOM.ParameterKey,
    extra_features::Dict{String, Any},
)::Dict{String, Dict{Dates.DateTime, Vector{Float64}}}
    try
        return POM.read_parameter_windows(pstore, key; extra_features = extra_features)
    catch e
        e isa ErrorException && startswith(e.msg, "no parameter windows found for") &&
            return Dict{String, Dict{Dates.DateTime, Vector{Float64}}}()
        rethrow()
    end
end

"""
`merged_by_label` restricted to the initial times not already present in
`existing_by_label` -- every label shares the same set of initial times (windows are always
written together, one `POM.write_parameter_windows!` call per key covering every label), so
one shared time set is enough to filter every label consistently.
"""
function _new_windows_by_label(
    merged_by_label::Dict{String, Dict{Dates.DateTime, Vector{Float64}}},
    existing_by_label::Dict{String, Dict{Dates.DateTime, Vector{Float64}}},
)::Dict{String, Dict{Dates.DateTime, Vector{Float64}}}
    isempty(existing_by_label) && return merged_by_label
    existing_times = Set(keys(first(values(existing_by_label))))
    result = Dict{String, Dict{Dates.DateTime, Vector{Float64}}}()
    for (label, by_time) in merged_by_label
        new_by_time = filter(p -> !(p.first in existing_times), by_time)
        isempty(new_by_time) || (result[label] = new_by_time)
    end
    return result
end

"""
Rebuild `Dict{DateTime, DenseAxisArray{Float64,2}}` windows (labels x steps) from parameter
rows read back per label, sorted labels for a deterministic column order -- the same
sorted-label fallback the read side uses (`_dm_parameter_labels`), since a parameter has no
persisted column order.
"""
function _windows_from_labels(
    merged_by_label::Dict{String, Dict{Dates.DateTime, Vector{Float64}}},
)::Dict{Dates.DateTime, DenseAxisArray{Float64, 2}}
    labels = sort!(collect(keys(merged_by_label)))
    times = sort!(collect(keys(merged_by_label[first(labels)])))
    windows = Dict{Dates.DateTime, DenseAxisArray{Float64, 2}}()
    for t in times
        steps = length(merged_by_label[first(labels)][t])
        data = Matrix{Float64}(undef, length(labels), steps)
        for (i, label) in enumerate(labels)
            data[i, :] = merged_by_label[label][t]
        end
        windows[t] = DenseAxisArray(data, labels, 1:steps)
    end
    return windows
end

"""
Merge the emulation model's parameter series across every partition in `merged_indexes` into
the main bundle, the same shape as [`_merge_decision_model_parameters!`](@ref): one source
sidecar open at a time, the destination sidecar opened once at the end. Resolves each
partition's bundle by system uuid ([`_emulation_bundle_dir`](@ref)), since the emulation
model entry never has a `problems/<name>/` directory of its own.
"""
function _merge_emulation_model_parameters!(
    results::SimulationPartitionResults,
    dst_store::HdfSimulationStore,
    merged_indexes::Vector{Int},
)
    keys_to_merge = list_emulation_model_keys(dst_store, STORE_CONTAINER_PARAMETERS)
    isempty(keys_to_merge) && return nothing

    em_model_name = _em_model_name(dst_store)
    extra_features = Dict{String, Any}("model" => string(em_model_name))
    accumulated = Dict{IOM.ParameterKey, Dict{String, Dict{Dates.DateTime, Float64}}}(
        key => Dict{String, Dict{Dates.DateTime, Float64}}() for key in keys_to_merge
    )
    for index in merged_indexes
        partition_dir = joinpath(_partition_path(results, index), STORE_DIR)
        bundle_dir = open_store(HdfSimulationStore, partition_dir, "r") do src_store
            return _emulation_bundle_dir(src_store)
        end
        pstore = POM.open_parameter_store(joinpath(bundle_dir, PSY.TIME_SERIES_FILE))
        try
            for key in keys_to_merge
                series =
                    POM.read_parameter_array(pstore, key; extra_features = extra_features)
                for (label, ts) in series
                    dest = get!(accumulated[key], label, Dict{Dates.DateTime, Float64}())
                    for (t, v) in zip(IS.TimeSeries.timestamp(ts), IS.TimeSeries.values(ts))
                        dest[t] = v
                    end
                end
            end
        finally
            POM.close_parameter_store!(pstore)
        end
    end

    em_resolution = get_resolution(get_emulation_model_params(dst_store))
    dst_bundle_dir = _emulation_bundle_dir(dst_store)
    pstore =
        POM.open_parameter_store_writable(joinpath(dst_bundle_dir, PSY.TIME_SERIES_FILE))
    try
        for key in keys_to_merge
            new_by_label = _new_series_by_label(
                accumulated[key],
                _existing_em_series(pstore, key, extra_features),
            )
            isempty(new_by_label) && continue
            labels = sort!(collect(keys(new_by_label)))
            timestamps = sort!(collect(keys(new_by_label[first(labels)])))
            data = Matrix{Float64}(undef, length(labels), length(timestamps))
            for (i, label) in enumerate(labels), (j, t) in enumerate(timestamps)
                data[i, j] = new_by_label[label][t]
            end
            array = DenseAxisArray(data, labels, 1:length(timestamps))
            POM.write_parameter_array!(
                pstore,
                key,
                array,
                timestamps,
                em_resolution;
                extra_features = extra_features,
            )
        end
    finally
        POM.close_parameter_store!(pstore)
    end
    return nothing
end

"""
The destination bundle's already-merged series for `key`, or an empty `Dict` when this key
has never been merged into it before. See [`_existing_decision_windows`](@ref) for why this
catches only `POM.read_parameter_array`'s own well-defined "no data" error.
"""
function _existing_em_series(
    pstore,
    key::IOM.ParameterKey,
    extra_features::Dict{String, Any},
)::Dict{String, IS.TimeSeries.TimeArray}
    try
        return POM.read_parameter_array(pstore, key; extra_features = extra_features)
    catch e
        e isa ErrorException && startswith(e.msg, "no parameter arrays found for") &&
            return Dict{String, IS.TimeSeries.TimeArray}()
        rethrow()
    end
end

"""
`merged_by_label` restricted to the timestamps not already present in `existing_by_label` --
every label shares the same set of timestamps (an emulation-model parameter array is always
written together, one `POM.write_parameter_array!` call per key covering every label), so one
shared timestamp set is enough to filter every label consistently.
"""
function _new_series_by_label(
    merged_by_label::Dict{String, Dict{Dates.DateTime, Float64}},
    existing_by_label::Dict{String, IS.TimeSeries.TimeArray},
)::Dict{String, Dict{Dates.DateTime, Float64}}
    isempty(existing_by_label) && return merged_by_label
    existing_times = Set(IS.TimeSeries.timestamp(first(values(existing_by_label))))
    result = Dict{String, Dict{Dates.DateTime, Float64}}()
    for (label, by_time) in merged_by_label
        new_by_time = filter(p -> !(p.first in existing_times), by_time)
        isempty(new_by_time) || (result[label] = new_by_time)
    end
    return result
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
