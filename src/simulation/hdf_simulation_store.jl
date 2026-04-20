const HDF_FILENAME = "simulation_store.h5"
const HDF_SIMULATION_ROOT_PATH = "simulation"
const EMULATION_MODEL_PATH = "$HDF_SIMULATION_ROOT_PATH/emulation_model"
const OPTIMIZER_STATS_PATH = "optimizer_stats"
const SERIALIZED_KEYS_PATH = "serialized_keys"

# This only applies if chunks are enabled, and that will only likely happen if we enable
# compression.
# The optimal number of chunks to store in memory will vary widely.
# The HDF docs recommend keeping chunk byte sizes between 10 KiB - 1 MiB.
# We want to make it big enough to compress duplicate values.
# The downside to making this larger is that any read causes the
# entire chunk to be read.
# If one variable has 10,000 components and each value is a Float64 then one row would
# consume 10,000 * 8 = 78 KiB
DEFAULT_MAX_CHUNK_BYTES = 128 * KiB

"""
Stores simulation data in an HDF file.
"""
mutable struct HdfSimulationStore <: SimulationStore
    file::HDF5.File
    params::SimulationStoreParams
    # The key order is the problem execution order.
    dm_data::OrderedDict{Symbol, DatasetContainer{HDF5Dataset}}
    em_data::DatasetContainer{HDF5Dataset}
    # The key is the problem name.
    optimizer_stats_datasets::Dict{Symbol, HDF5.Dataset}
    optimizer_stats_write_index::Dict{Symbol, Int}
    cache::OptimizationOutputCaches
end

get_initial_time(store::HdfSimulationStore) = get_initial_time(store.params)

function HdfSimulationStore(file_path::AbstractString, mode::AbstractString)
    if !(mode in ("w", "r", "rw"))
        throw(IS.ConflictingInputsError("mode can only be 'w', 'r', or 'rw'"))
    end

    if !isfile(file_path) && mode in ("r", "rw")
        throw(IS.ConflictingInputsError("$file_path does not exist"))
    end

    if isfile(file_path) && mode == "w"
        throw(IS.ConflictingInputsError("$file_path already exists"))
    end

    hdf5_mode = mode == "rw" ? "r+" : mode
    file = HDF5.h5open(file_path, hdf5_mode)
    if mode == "w"
        HDF5.create_group(file, HDF_SIMULATION_ROOT_PATH)
        @debug "Created store" file_path
    end

    store = HdfSimulationStore(
        file,
        SimulationStoreParams(),
        OrderedDict{Symbol, DatasetContainer{HDF5Dataset}}(),
        DatasetContainer{HDF5Dataset}(),
        Dict{Symbol, HDF5.Dataset}(),
        Dict{Symbol, Int}(),
        OptimizationOutputCaches(),
    )
    mode in ("r", "rw") && _deserialize_attributes!(store)

    finalizer(_check_state, store)
    return store
end

"""
Construct and open an HdfSimulationStore.

When reading or writing results in a program you should use the method that accepts a
function in order to guarantee that the file handle gets closed.

# Arguments

  - `directory::AbstractString`: Directory containing the store file
  - `mode::AbstractString`: Mode to use to open the store file
  - `filename::AbstractString`: Base name of the store file

# Examples

```julia
# Assumes a simulation has been executed in the './rts' directory with these parameters.
path = "./rts"
problem = :ED
var_name = :P__ThermalStandard
timestamp = DateTime("2020-01-01T05:00:00")
store = open_store(HdfSimulationStore, path)
df = PowerSimulations.read_result(DataFrame, store, model, :variables, var_name, timestamp)
```
"""
function open_store(
    ::Type{HdfSimulationStore},
    directory::AbstractString,
    mode = "r";
    filename = HDF_FILENAME,
)
    return HdfSimulationStore(joinpath(directory, filename), mode)
end

function open_store(
    func::Function,
    ::Type{HdfSimulationStore},
    directory::AbstractString,
    mode = "r";
    filename = HDF_FILENAME,
)
    store = nothing
    try
        store = HdfSimulationStore(joinpath(directory, filename), mode)
        return func(store)
    finally
        if store !== nothing
            close(store)
        end
    end
end

function Base.close(store::HdfSimulationStore)
    flush(store)
    HDF5.close(store.file)
    empty!(store.cache)
    @debug "Close store file handle" store.file
end

function Base.isopen(store::HdfSimulationStore)
    return store.file === nothing ? false : HDF5.isopen(store.file)
end

function Base.flush(store::HdfSimulationStore)
    for (key, output_cache) in store.cache.data
        _flush_data!(output_cache, store, key, false)
        @assert !has_dirty(output_cache) "$key has dirty cache after flushing"
    end

    flush(store.file)
    @debug "Flush store"
    return
end

get_params(store::HdfSimulationStore) = store.params

function set_cache_flush_rules!(store::HdfSimulationStore, flush_rules::CacheFlushRules)
    new_cache = OptimizationOutputCaches(flush_rules)
    for (key, output_cache) in store.cache.data
        new_cache.data[key] = output_cache
    end
    store.cache = new_cache
    @debug "Updated store cache rules" get_min_flush_size(store.cache) get_max_size(
        store.cache,
    )
    return
end

function get_decision_model_params(store::HdfSimulationStore, model_name::Symbol)
    return get_decision_model_params(get_params(store), model_name)
end

function get_emulation_model_params(store::HdfSimulationStore)
    return get_emulation_model_params(get_params(store))
end

function get_container_key_lookup(store::HdfSimulationStore)
    function _get_lookup()
        root = _get_root(store)
        buf = IOBuffer(root[SERIALIZED_KEYS_PATH][:])
        return Serialization.deserialize(buf)
    end
    isopen(store) && return _get_lookup()

    store.file = HDF5.h5open(store.file.filename, "r")
    try
        return _get_lookup()
    finally
        HDF5.close(store.file)
    end
end

"""
Return the problem names in order of execution.
"""
list_decision_models(store::HdfSimulationStore) = keys(get_dm_data(store))

"""
Return the fields stored for the `problem` and `container_type` (duals/parameters/variables).
"""
function list_decision_model_keys(
    store::HdfSimulationStore,
    model::Symbol,
    container_type::Symbol,
)
    container = getfield(get_dm_data(store)[model], container_type)
    return collect(keys(container))
end

function list_emulation_model_keys(store::HdfSimulationStore, container_type::Symbol)
    container = getfield(get_em_data(store), container_type)
    return collect(keys(container))
end

function write_optimizer_stats!(
    store::HdfSimulationStore,
    model::OperationModel,
    ::DecisionModelIndexType,
)
    stats = get_optimizer_stats(model)
    model_name = get_name(model)
    dataset = _get_dataset(OptimizerStats, store, model_name)

    # Uncomment for performance measures of HDF Store
    #TimerOutputs.@timeit RUN_SIMULATION_TIMER "Write optimizer stats" begin
    dataset[:, store.optimizer_stats_write_index[model_name]] = to_matrix(stats)
    #end

    store.optimizer_stats_write_index[model_name] += 1
    return
end

function write_optimizer_stats!(
    store::HdfSimulationStore,
    model::OperationModel,
    ::EmulationModelIndexType,
)
    return
end

"""
Read the optimizer stats for a problem execution.
"""
function read_optimizer_stats(
    store::HdfSimulationStore,
    simulation_step::Int,
    model_name::Symbol,
    execution_index::Int,
)
    optimizer_stats_write_index =
        (simulation_step - 1) *
        store.params.decision_models_params[model_name].num_executions + execution_index
    dataset = _get_dataset(OptimizerStats, store, model_name)
    return OptimizerStats(dataset[:, optimizer_stats_write_index])
end

"""
Return the optimizer stats for a problem as a DataFrame.
"""
function read_optimizer_stats(store::HdfSimulationStore, model_name)
    dataset = _get_dataset(OptimizerStats, store, model_name)
    data = permutedims(dataset[:, :])
    stats = [IS.to_namedtuple(OptimizerStats(data[i, :])) for i in axes(data)[1]]
    return DataFrames.DataFrame(stats)
end

function initialize_problem_storage!(
    store::HdfSimulationStore,
    params::SimulationStoreParams,
    dm_problem_reqs::Dict{Symbol, SimulationModelStoreRequirements},
    em_problem_reqs::SimulationModelStoreRequirements,
    flush_rules::CacheFlushRules,
)
    store.params = params
    root = store.file[HDF_SIMULATION_ROOT_PATH]
    problems_group = _get_group_or_create(root, "decision_models")
    store.cache = OptimizationOutputCaches(flush_rules)
    @info "Initialize store cache" get_min_flush_size(store.cache) get_max_size(store.cache)
    initial_time = get_initial_time(store)
    container_key_lookup = Dict{String, OptimizationContainerKey}()
    for (problem, problem_params) in store.params.decision_models_params
        get_dm_data(store)[problem] = DatasetContainer{HDF5Dataset}()
        problem_group = _get_group_or_create(problems_group, string(problem))
        for type in STORE_CONTAINERS
            group = _get_group_or_create(problem_group, string(type))
            for (key, reqs) in getfield(dm_problem_reqs[problem], type)
                !should_write_resulting_value(key) && continue
                name = encode_key_as_string(key)
                dataset = _create_dataset(group, name, reqs)
                # Columns can't be stored in attributes because they might be larger than
                # the max size of 64 KiB.
                col = _make_column_name(name)
                if length(reqs["columns"]) == 1
                    HDF5.write_dataset(group, col, string.(reqs["columns"][1]))
                else
                    col_vals = vcat(reqs["columns"]...)
                    HDF5.write_dataset(group, col, string.(col_vals))
                end
                column_dataset = group[col]
                datasets = getfield(get_dm_data(store)[problem], type)
                column_lengths = reqs["dims"][2:(end - 1)]
                datasets[key] = HDF5Dataset{length(column_lengths)}(
                    dataset,
                    column_dataset,
                    column_lengths,
                    get_resolution(problem_params),
                    initial_time,
                )
                add_output_cache!(
                    store.cache,
                    problem,
                    key,
                    get_rule(flush_rules, problem, key),
                )
                container_key_lookup[encode_key_as_string(key)] = key
            end
        end

        num_stats = params.num_steps * params.decision_models_params[problem].num_executions
        columns = fieldnames(OptimizerStats)
        num_columns = length(columns)
        dataset = HDF5.create_dataset(
            problem_group,
            OPTIMIZER_STATS_PATH,
            HDF5.datatype(Float64),
            HDF5.dataspace((num_columns, num_stats)),
        )
        HDF5.attributes(dataset)["columns"] = [string(x) for x in columns]
        store.optimizer_stats_datasets[problem] = dataset
        store.optimizer_stats_write_index[problem] = 1
        @debug "Initialized optimizer_stats_datasets $problem ($num_columns, $num_stats)"
    end

    emulation_group = _get_group_or_create(root, "emulation_model")
    for emulation_params in values(store.params.emulation_model_params)
        for type in STORE_CONTAINERS
            group = _get_group_or_create(emulation_group, string(type))
            for (key, reqs) in getfield(em_problem_reqs, type)
                name = encode_key_as_string(key)
                dataset = _create_dataset(group, name, reqs)
                # Columns can't be stored in attributes because they might be larger than
                # the max size of 64 KiB.
                col = _make_column_name(name)
                if length(reqs["columns"]) == 1
                    HDF5.write_dataset(group, col, string.(reqs["columns"][1]))
                else
                    col_vals = vcat(reqs["columns"]...)
                    HDF5.write_dataset(group, col, string.(col_vals))
                end
                column_dataset = group[col]
                datasets = getfield(store.em_data, type)
                column_lengths = reqs["dims"][2:end]
                datasets[key] = HDF5Dataset{length(column_lengths)}(
                    dataset,
                    column_dataset,
                    column_lengths,
                    get_resolution(emulation_params),
                    initial_time,
                )
                container_key_lookup[encode_key_as_string(key)] = key
            end
        end
    end
    buf = IOBuffer()
    Serialization.serialize(buf, container_key_lookup)
    seek(buf, 0)
    root[SERIALIZED_KEYS_PATH] = buf.data

    # This has to run after problem groups are created.
    _serialize_attributes(store)
    return
end

log_cache_hit_percentages(x::HdfSimulationStore) = log_cache_hit_percentages(x.cache)

function _make_dataframe(data::Matrix{Float64}, columns::Tuple{Vector{String}})
    return DataFrames.DataFrame(data, columns[1]; copycols = false)
end

"""
Return DataFrame, DenseAxisArray, or Array for a model result at a timestamp.
"""
function read_result(
    ::Type{DataFrames.DataFrame},
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::Union{DecisionModelIndexType, EmulationModelIndexType},
)
    data, columns = _read_data_columns(store, model_name, key, index)
    return _make_dataframe(data, columns)
end

function _make_denseaxisarray(
    data::Matrix{Float64},
    columns::Tuple{Vector{String}},
)
    return DenseAxisArray(permutedims(data), columns[1], 1:size(data)[1])
end

function _make_denseaxisarray(
    data::Matrix{Float64},
    columns::NTuple{2, <:Any},
)
    # Handle 2D data with 2 column axes (e.g., from reshaped 3D emulation data)
    return DenseAxisArray(
        permutedims(data),
        columns[1],
        columns[2],
    )
end

function _make_denseaxisarray(
    data::Array{Float64, 3},
    columns::NTuple{2, <:Any},
)
    return DenseAxisArray(
        permutedims(data, (2, 3, 1)),
        columns[1],
        columns[2],
        1:size(data)[1],
    )
end

function read_result(
    ::Type{DenseAxisArray},
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::Union{DecisionModelIndexType, EmulationModelIndexType},
)
    if is_cached(store.cache, model_name, key, index)
        data = read_result(store.cache, model_name, key, index)
        columns = get_column_names(store, DecisionModelIndexType, model_name, key)
    else
        data, columns = _read_result(store, model_name, key, index)
    end
    return _make_denseaxisarray(data, columns)
end

function read_result(
    ::Type{Array},
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::Union{DecisionModelIndexType, EmulationModelIndexType},
)
    if is_cached(store.cache, model_name, key, index)
        data = read_result(store.cache, model_name, key, index)
    else
        data, _ = _read_result(store, model_name, key, index)
    end

    return data
end

function read_results(
    store::HdfSimulationStore,
    key::OptimizationContainerKey;
    index::Union{Nothing, EmulationModelIndexType} = nothing,
    len::Union{Nothing, Int} = nothing,
)
    dataset = _get_em_dataset(store, key)
    num_dims = ndims(dataset.values)
    if num_dims == 2
        if isnothing(index)
            @assert_op(isnothing(len))
            data = dataset.values[:, :]
        elseif isnothing(len)
            data = dataset.values[index:end, :]
        else
            data = dataset.values[index:(index + len - 1), :]
        end
        columns = get_column_names(key, dataset)
        return DenseAxisArray(permutedims(data), columns..., 1:size(data)[1])
    elseif num_dims == 3
        if isnothing(index)
            @assert_op(isnothing(len))
            data = dataset.values[:, :, :]
        elseif isnothing(len)
            data = dataset.values[index:end, :, :]
        else
            data = dataset.values[index:(index + len - 1), :, :]
        end
        columns = get_column_names(key, dataset)
        return DenseAxisArray(permutedims(data, (2, 3, 1)), columns..., 1:size(data)[1])
    else
        error("Unsupported number of dimensions for emulation dataset: $num_dims")
    end
end

function get_column_names(
    store::HdfSimulationStore,
    ::Type{DecisionModelIndexType},
    model_name::Symbol,
    key::OptimizationContainerKey,
)
    !isopen(store) && throw(ArgumentError("store must be opened prior to reading"))
    dataset = _get_dm_dataset(store, model_name, key)
    return get_column_names(key, dataset)
end

function get_column_names(
    store::HdfSimulationStore,
    ::Type{EmulationModelIndexType},
    key::OptimizationContainerKey,
)
    !isopen(store) && throw(ArgumentError("store must be opened prior to reading"))
    dataset = _get_em_dataset(store, key)
    return get_column_names(key, dataset)
end

function get_number_of_dimensions(
    store::HdfSimulationStore,
    i::Type{DecisionModelIndexType},
    model_name::Symbol,
    key::OptimizationContainerKey,
)
    return length(get_column_names(store, i, model_name, key))
end

function get_number_of_dimensions(
    store::HdfSimulationStore,
    i::Type{EmulationModelIndexType},
    key::OptimizationContainerKey,
)
    return length(get_column_names(store, i, key))
end

function get_emulation_model_dataset_size(
    store::HdfSimulationStore,
    key::OptimizationContainerKey,
)
    dataset = _get_em_dataset(store, key)
    return size(dataset.values)[1]
end

function _read_result(
    store::HdfSimulationStore,
    ::Symbol,
    key::OptimizationContainerKey,
    index::EmulationModelIndexType,
)
    !isopen(store) && throw(ArgumentError("store must be opened prior to reading"))
    model_params = get_emulation_model_params(store)

    if index > model_params.num_executions
        throw(
            ArgumentError(
                "index = $index cannot be larger than $(model_params.num_executions)",
            ),
        )
    end

    dataset = _get_em_dataset(store, key)
    dset = dataset.values
    # Uncomment for performance checking
    #TimerOutputs.@timeit RUN_SIMULATION_TIMER "Read dataset" begin
    num_dims = ndims(dset)
    if num_dims == 2
        data = dset[index, :]
    elseif num_dims == 3
        data = dset[index, :, :]
    else
        error("Unsupported number of dimensions for emulation dataset: $num_dims")
    end
    #end
    columns = get_column_names(key, dataset)
    data = ndims(data) == 1 ? permutedims(data) : data
    return data, columns
end

function _read_result(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
)
    simulation_step, execution_index = _get_indices(store, model_name, index)
    return _read_result(store, model_name, key, simulation_step, execution_index)
end

function _read_result(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    simulation_step::Int,
    execution_index::Int,
)
    !isopen(store) && throw(ArgumentError("store must be opened prior to reading"))

    model_params = get_decision_model_params(store, model_name)
    num_executions = model_params.num_executions
    if execution_index > num_executions
        throw(
            ArgumentError(
                "execution_index = $execution_index cannot be larger than $num_executions",
            ),
        )
    end

    dataset = _get_dm_dataset(store, model_name, key)
    dset = dataset.values
    row_index = (simulation_step - 1) * num_executions + execution_index
    columns = get_column_names(key, dataset)

    # Uncomment for performance checking
    #TimerOutputs.@timeit RUN_SIMULATION_TIMER "Read dataset" begin
    num_dims = ndims(dset)
    if num_dims == 3
        data = dset[:, :, row_index]
    elseif num_dims == 4
        data = dset[:, :, :, row_index]
    else
        error("unsupported dims: $num_dims")
    end
    #end

    return data, columns
end

"""
Write a decision model result for a timestamp to the store.
"""
function write_result!(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    ::Dates.DateTime,
    data::DenseAxisArray{Float64, N, <:NTuple{N, Any}},
) where {N}
    output_cache = get_output_cache(store.cache, model_name, key)
    cur_size = get_size(store.cache)
    add_result!(output_cache, index, to_matrix(data), is_full(store.cache, cur_size))

    if get_dirty_size(output_cache) >= get_min_flush_size(store.cache)
        discard = !should_keep_in_cache(output_cache)

        # PERF: A potentially significant performance improvement would be to queue several
        # flushes and submit them in parallel.
        size_flushed = _flush_data!(output_cache, store, model_name, key, discard)

        @debug "flushed data" LOG_GROUP_SIMULATION_STORE key size_flushed discard cur_size
    end

    # Disabled because this is currently a noop.
    #if is_full(store.cache)
    #    _flush_data!(store.cache, store)
    #end

    @debug "write_result" get_size(store.cache) encode_key_as_string(key)
    return
end

"""
Write a decision model result for a timestamp to the store.
"""
function write_result!(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    ::Dates.DateTime,
    data::DenseAxisArray{Float64, 3, <:NTuple{3, Any}},
)
    output_cache = get_output_cache(store.cache, model_name, key)
    cur_size = get_size(store.cache)

    add_result!(
        output_cache,
        index,
        permutedims(data.data, (3, 1, 2)),
        is_full(store.cache, cur_size),
    )

    if get_dirty_size(output_cache) >= get_min_flush_size(store.cache)
        discard = !should_keep_in_cache(output_cache)

        # PERF: A potentially significant performance improvement would be to queue several
        # flushes and submit them in parallel.
        size_flushed = _flush_data!(output_cache, store, model_name, key, discard)

        @debug "flushed data" LOG_GROUP_SIMULATION_STORE key size_flushed discard cur_size
    end

    # Disabled because this is currently a noop.
    #if is_full(store.cache)
    #    _flush_data!(store.cache, store)
    #end

    @debug "write_result" get_size(store.cache) encode_key_as_string(key)
    return
end

"""
Write an emulation model result for an execution index value and the timestamp of the update
"""
function write_result!(
    store::HdfSimulationStore,
    ::Symbol,
    key::OptimizationContainerKey,
    index::EmulationModelIndexType,
    simulation_time::Dates.DateTime,
    array::DenseAxisArray{Float64, 2},
)
    # TODO: This is a temporary fix.
    # Not sure why the special case for this dimension size is needed.
    # It fails with the key = InfrastructureSystems.Optimization.ParameterKey{OnStatusParameter, ThermalStandard}("")
    # The array size is 5 x 1
    data = size(array, 2) == 1 ? reshape(array.data, length(array.data)) : array.data
    dataset = _get_em_dataset(store, key)
    _write_dataset!(dataset.values, data, index)
    set_last_recorded_row!(dataset, index)
    set_update_timestamp!(dataset, simulation_time)
    return
end

function write_result!(
    store::HdfSimulationStore,
    ::Symbol,
    key::OptimizationContainerKey,
    index::EmulationModelIndexType,
    simulation_time::Dates.DateTime,
    array::DenseAxisArray{Float64, 3},
)
    # Handle 3D arrays by reshaping if the last dimension is 1
    # This mirrors the 2D case above where size(array, 2) == 1 triggers a reshape
    if size(array, 3) == 1
        data = reshape(array.data, size(array, 1), size(array, 2))
    else
        data = array.data
    end
    dataset = _get_em_dataset(store, key)
    _write_dataset!(dataset.values, data, index)
    set_last_recorded_row!(dataset, index)
    set_update_timestamp!(dataset, simulation_time)
    return
end

function write_result!(
    store::HdfSimulationStore,
    ::Symbol,
    key::OptimizationContainerKey,
    index::EmulationModelIndexType,
    simulation_time::Dates.DateTime,
    array::DenseAxisArray{Float64},
)
    dataset = _get_em_dataset(store, key)
    _write_dataset!(dataset.values, array.data, index)
    set_last_recorded_row!(dataset, index)
    set_update_timestamp!(dataset, simulation_time)
    return
end

function serialize_system!(store::HdfSimulationStore, sys::PSY.System)
    root = store.file[HDF_SIMULATION_ROOT_PATH]
    systems_group = _get_group_or_create(root, "systems")
    uuid = string(IS.get_uuid(sys))
    if haskey(systems_group, uuid)
        @debug "System with UUID = $uuid is already stored" _group =
            LOG_GROUP_SIMULATION_STORE
        return
    end

    json_text = PSY.to_json(sys)
    systems_group[uuid] = json_text
    return
end

function write_system_json!(store::HdfSimulationStore, uuid::String, json_text::String)
    root = store.file[HDF_SIMULATION_ROOT_PATH]
    systems_group = _get_group_or_create(root, "systems")
    if !haskey(systems_group, uuid)
        systems_group[uuid] = json_text
    end
    return
end

function deserialize_system(store::HdfSimulationStore, uuid::Base.UUID)
    root = store.file[HDF_SIMULATION_ROOT_PATH]
    systems_group = _get_group_or_create(root, "systems")
    uuid_str = string(uuid)
    if !haskey(systems_group, uuid_str)
        error("No system with UUID $uuid_str is stored")
    end

    json_text = HDF5.read(systems_group[uuid_str])
    return PSY.from_json(json_text, PSY.System)
end

function _check_state(store::HdfSimulationStore)
    if has_dirty(store.cache)
        error("BUG!!! dirty cache is present at shutdown: $(store.file)")
    end
end

function _compute_chunk_count(dims, dtype; max_chunk_bytes = DEFAULT_MAX_CHUNK_BYTES)
    bytes_per_element = sizeof(dtype)

    if length(dims) == 2
        size_row = dims[1] * bytes_per_element
    elseif length(dims) == 3
        size_row = dims[1] * dims[2] * bytes_per_element
    elseif length(dims) == 4
        size_row = dims[1] * dims[2] * dims[3] * bytes_per_element
    else
        error("unsupported dims = $dims")
    end

    chunk_count = minimum((trunc(max_chunk_bytes / size_row), dims[end]))
    if chunk_count == 0
        error(
            "HDF Max Chunk Bytes is smaller than the size of a row. Please increase it. " *
            "max_chunk_bytes=$max_chunk_bytes dims=$dims " *
            "size_row=$size_row",
        )
    end

    chunk_dims = [x for x in dims]
    chunk_dims[end] = chunk_count
    return chunk_dims
end

function _create_dataset(group, name, reqs)
    dataset = HDF5.create_dataset(
        group,
        name,
        HDF5.datatype(Float64),
        HDF5.dataspace(reqs["dims"]),
        # We are choosing to optimize read performance in the first implementation.
        # Compression would slow that down.
        #chunk = _compute_chunk_count(reqs["dims"], Float64),
        #shuffle = (),
        #deflate = 3,
    )
    @debug "Created dataset for" group name size(dataset)
    return dataset
end

function _deserialize_attributes!(store::HdfSimulationStore)
    container_key_lookup = get_container_key_lookup(store)
    group = store.file["simulation"]
    initial_time = Dates.DateTime(HDF5.read(HDF5.attributes(group)["initial_time"]))
    step_resolution =
        Dates.Millisecond(HDF5.read(HDF5.attributes(group)["step_resolution_ms"]))
    num_steps = HDF5.read(HDF5.attributes(group)["num_steps"])
    store.params = SimulationStoreParams(initial_time, step_resolution, num_steps)
    empty!(get_dm_data(store))
    for model in HDF5.read(HDF5.attributes(group)["problem_order"])
        problem_group = store.file["simulation/decision_models/$model"]
        # Fall back on old key for backwards compatibility
        horizon_count = HDF5.read(
            if haskey(HDF5.attributes(problem_group), "horizon_count")
                HDF5.attributes(problem_group)["horizon_count"]
            else
                HDF5.attributes(problem_group)["horizon"]
            end)
        model_name = Symbol(model)
        store.params.decision_models_params[model_name] = ModelStoreParams(
            HDF5.read(HDF5.attributes(problem_group)["num_executions"]),
            horizon_count,
            Dates.Millisecond(HDF5.read(HDF5.attributes(problem_group)["interval_ms"])),
            Dates.Millisecond(HDF5.read(HDF5.attributes(problem_group)["resolution_ms"])),
            HDF5.read(HDF5.attributes(problem_group)["base_power"]),
            Base.UUID(HDF5.read(HDF5.attributes(problem_group)["system_uuid"])),
        )
        get_dm_data(store)[model_name] = DatasetContainer{HDF5Dataset}()
        for type in STORE_CONTAINERS
            group = problem_group[string(type)]
            for name in keys(group)
                if !endswith(name, "columns")
                    dataset = group[name]
                    column_dataset = group[_make_column_name(name)]
                    resolution =
                        get_resolution(get_decision_model_params(store, model_name))
                    column_lengths = size(dataset)[2:(end - 1)]
                    item = HDF5Dataset{length(column_lengths)}(
                        dataset,
                        column_dataset,
                        column_lengths,
                        resolution,
                        initial_time,
                    )
                    container_key = container_key_lookup[name]
                    getfield(get_dm_data(store)[model_name], type)[container_key] = item
                    add_output_cache!(
                        store.cache,
                        model_name,
                        container_key,
                        CacheFlushRule(),
                    )
                end
            end
        end

        store.optimizer_stats_datasets[model_name] = problem_group[OPTIMIZER_STATS_PATH]
        store.optimizer_stats_write_index[model_name] = 1
    end

    em_group = _get_emulation_model_path(store)
    # Fall back on old key for backwards compatibility
    horizon_count = HDF5.read(
        if haskey(HDF5.attributes(em_group), "horizon_count")
            HDF5.attributes(em_group)["horizon_count"]
        else
            HDF5.attributes(em_group)["horizon"]
        end)
    model_name = Symbol(HDF5.read(HDF5.attributes(em_group)["name"]))
    resolution = Dates.Millisecond(HDF5.read(HDF5.attributes(em_group)["resolution_ms"]))
    store.params.emulation_model_params[model_name] = ModelStoreParams(
        HDF5.read(HDF5.attributes(em_group)["num_executions"]),
        horizon_count,
        Dates.Millisecond(HDF5.read(HDF5.attributes(em_group)["interval_ms"])),
        resolution,
        HDF5.read(HDF5.attributes(em_group)["base_power"]),
        Base.UUID(HDF5.read(HDF5.attributes(em_group)["system_uuid"])),
    )
    for type in STORE_CONTAINERS
        group = em_group[string(type)]
        for name in keys(group)
            if !endswith(name, "columns")
                dataset = group[name]
                column_dataset = group[_make_column_name(name)]
                column_lengths = size(dataset)[2:end]
                item = HDF5Dataset{length(column_lengths)}(
                    dataset,
                    column_dataset,
                    column_lengths,
                    resolution,
                    initial_time,
                )
                container_key = container_key_lookup[name]
                getfield(store.em_data, type)[container_key] = item
                add_output_cache!(store.cache, model_name, container_key, CacheFlushRule())
            end
        end
    end
    # TODO: optimizer stats are not being written for EM.

    @debug "deserialized store params and datasets" store.params
end

function _serialize_attributes(store::HdfSimulationStore)
    params = store.params
    group = store.file["simulation"]
    HDF5.attributes(group)["problem_order"] =
        [string(k) for k in keys(params.decision_models_params)]
    HDF5.attributes(group)["initial_time"] = string(params.initial_time)
    HDF5.attributes(group)["step_resolution_ms"] =
        Dates.Millisecond(params.step_resolution).value
    HDF5.attributes(group)["num_steps"] = params.num_steps

    for problem in keys(params.decision_models_params)
        problem_group = store.file["simulation/decision_models/$problem"]
        HDF5.attributes(problem_group)["num_executions"] =
            params.decision_models_params[problem].num_executions
        HDF5.attributes(problem_group)["horizon_count"] =
            params.decision_models_params[problem].horizon_count
        HDF5.attributes(problem_group)["resolution_ms"] =
            Dates.Millisecond(params.decision_models_params[problem].resolution).value
        HDF5.attributes(problem_group)["interval_ms"] =
            Dates.Millisecond(params.decision_models_params[problem].interval).value
        HDF5.attributes(problem_group)["base_power"] =
            params.decision_models_params[problem].base_power
        HDF5.attributes(problem_group)["system_uuid"] =
            string(params.decision_models_params[problem].system_uuid)
    end

    if !isempty(params.emulation_model_params)
        em_params = first(values(params.emulation_model_params))
        emulation_group = store.file["simulation/emulation_model"]
        HDF5.attributes(emulation_group)["name"] =
            string(first(keys(params.emulation_model_params)))
        HDF5.attributes(emulation_group)["num_executions"] = em_params.num_executions
        HDF5.attributes(emulation_group)["horizon_count"] = em_params.horizon_count
        HDF5.attributes(emulation_group)["resolution_ms"] =
            Dates.Millisecond(em_params.resolution).value
        HDF5.attributes(emulation_group)["interval_ms"] =
            Dates.Millisecond(em_params.interval).value
        HDF5.attributes(emulation_group)["base_power"] = em_params.base_power
        HDF5.attributes(emulation_group)["system_uuid"] = string(em_params.system_uuid)
    end
    return
end

function _flush_data!(
    cache::OptimizationOutputCache,
    store::HdfSimulationStore,
    model_name,
    key::OptimizationContainerKey,
    discard,
)
    return _flush_data!(cache, store, OptimizationResultCacheKey(model_name, key), discard)
end

function _flush_data!(
    cache::OptimizationOutputCache,
    store::HdfSimulationStore,
    cache_key::OptimizationResultCacheKey,
    discard::Bool,
)
    !has_dirty(cache) && return 0
    dataset = _get_dm_dataset(store, cache_key)
    timestamps, data = get_dirty_data_to_flush!(cache)
    num_results = length(timestamps)
    @assert_op num_results == size(data)[end]
    end_index = dataset.write_index + length(timestamps) - 1
    write_range = (dataset.write_index):end_index
    # Enable only for development and benchmarking
    #TimerOutputs.@timeit RUN_SIMULATION_TIMER "Write $(key.key) array to HDF" begin
    _write_dataset!(dataset.values, data, write_range)
    #end

    discard && discard_results!(cache, timestamps)

    dataset.write_index += num_results
    size_flushed = cache.size_per_entry * num_results

    @debug "Flushed cache results to HDF5" LOG_GROUP_SIMULATION_STORE cache_key size_flushed num_results get_size(
        store.cache,
    )
    return size_flushed
end

function _get_dataset(::Type{OptimizerStats}, store::HdfSimulationStore, model_name)
    return store.optimizer_stats_datasets[model_name]
end

function _get_dataset(::Type{OptimizerStats}, store::HdfSimulationStore)
    return store.optimizer_stats_datasets
end

function _get_em_dataset(store::HdfSimulationStore, key::OptimizationContainerKey)
    return getfield(get_em_data(store), get_store_container_type(key))[key]
end

function _get_dm_dataset(store::HdfSimulationStore, model_name::Symbol)
    return get_dm_data(store)[model_name]
end

function _get_dm_dataset(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
)
    return getfield(get_dm_data(store)[model_name], get_store_container_type(key))[key]
end

function _get_dm_dataset(store::HdfSimulationStore, key::OptimizationResultCacheKey)
    return _get_dm_dataset(store, key.model, key.key)
end

function _get_group_or_create(parent, group_name)
    if haskey(parent, group_name)
        group = parent[group_name]
    else
        group = HDF5.create_group(parent, group_name)
        @debug "Created group" group
    end

    return group
end

_make_column_name(name) = string(name) * "__columns"

function _get_indices(store::HdfSimulationStore, model_name::Symbol, timestamp)
    time_diff = Dates.Millisecond(timestamp - store.params.initial_time)
    step = time_diff ÷ store.params.step_resolution + 1
    if step > store.params.num_steps
        throw(
            ArgumentError("timestamp = $timestamp is beyond the simulation: step = $step"),
        )
    end
    problem_params = store.params.decision_models_params[model_name]
    initial_time = store.params.initial_time + (step - 1) * store.params.step_resolution
    time_diff = timestamp - initial_time
    if time_diff % problem_params.interval != Dates.Millisecond(0)
        throw(ArgumentError("timestamp = $timestamp is not a valid problem timestamp"))
    end
    execution_index = time_diff ÷ problem_params.interval + 1
    return step, execution_index
end

_get_root(store::HdfSimulationStore) = store.file[HDF_SIMULATION_ROOT_PATH]
_get_emulation_model_path(store::HdfSimulationStore) = store.file[EMULATION_MODEL_PATH]

function _read_column_names(::Type{OptimizerStats}, store::HdfSimulationStore)
    dataset = _get_dataset(OptimizerStats, store)
    return HDF5.read(HDF5.attributes(dataset), "columns")
end

function _read_data_columns(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
)
    if is_cached(store.cache, model_name, key, index)
        data = read_result(store.cache, model_name, key, index)
        column_dataset = _get_dm_dataset(store, model_name, key).column_dataset
        if ndims(column_dataset) == 1
            columns = (column_dataset[:],)
        elseif ndims(column_dataset) == 2
            columns = (column_dataset[:, 1], column_dataset[:, 2])
        else
            error("Datasets with $(ndims(column_dataset)) columns not supported")
        end
    else
        data, columns = _read_result(store, model_name, key, index)
    end
    return data, columns
end

function _read_data_columns(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::EmulationModelIndexType,
)
    # TODO: Enable once the cache is in use for em_data
    #if is_cached(store.cache, model_name, key, index)
    # data = read_result(store.cache, model_name, key, index)
    #columns = _get_em_dataset(store, model_name, key).column_dataset[:]
    #else
    #    data, columns = _read_result(store, model_name, key, index)
    #end

    return _read_result(store, model_name, key, index)
end

function _read_length(::Type{OptimizerStats}, store::HdfSimulationStore)
    dataset = _get_dataset(OptimizerStats, store)
    return HDF5.read(HDF5.attributes(dataset), "columns")
end

# Specific data set writing function that writes decision model data. It dispatches on the index type of the dataset as a range
function _write_dataset!(
    dataset::HDF5.Dataset,
    array::Array{Float64, 3},
    row_range::UnitRange{Int64},
)
    dataset[:, :, row_range] = array
    @debug "wrote dm dataset" dataset row_range
    return
end

function _write_dataset!(
    dataset::HDF5.Dataset,
    array::Array{Float64, 4},
    row_range::UnitRange{Int64},
)
    dataset[:, :, :, row_range] = array
    @debug "wrote dm dataset" dataset row_range
    return
end

# Specific data set writing function that writes emulation model data. It dispatches on the index type of the dataset
function _write_dataset!(
    dataset::HDF5.Dataset,
    array::Vector{Float64},
    index::EmulationModelIndexType,
)
    assign_maybe_broadcast!(dataset, array, (index,))
    @debug "wrote em dataset" dataset index
    return
end

function _write_dataset!(
    dataset::HDF5.Dataset,
    array::Matrix{Float64},
    index::EmulationModelIndexType,
)
    dataset[index, :, :] = array
    @debug "wrote em dataset" dataset index
    return
end

function _write_dataset!(
    dataset::HDF5.Dataset,
    array::Array{Float64, 3},
    index::EmulationModelIndexType,
)
    dataset[index, :, :, :] = array
    @debug "wrote em dataset" dataset index
    return
end

# TODO DT: this looked wrong. Was it tested?
function _write_dataset!(
    dataset::HDF5.Dataset,
    array::Array{Float64, 4},
    index::EmulationModelIndexType,
)
    dataset[index, :, :, :] = array
    @debug "wrote em dataset" dataset index
    return
end
