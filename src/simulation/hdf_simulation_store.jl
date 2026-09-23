const HDF_FILENAME = "simulation_store.h5"
const HDF_SIMULATION_ROOT_PATH = "simulation"
const EMULATION_MODEL_PATH = "$HDF_SIMULATION_ROOT_PATH/emulation_model"
const OPTIMIZER_STATS_PATH = "optimizer_stats"
const SERIALIZED_KEYS_PATH = "serialized_keys"
const PARAMETER_KEYS_PATH = "parameter_keys"

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
    # Realized parameter slabs, buffered until finalize (Task 1: `:buffer`); see finalize_parameters!.
    dm_parameter_windows::Dict{
        Symbol,
        Dict{IOM.ParameterKey, Dict{Dates.DateTime, DenseAxisArray{Float64}}},
    }
    em_parameter_values::Dict{
        IOM.ParameterKey,
        OrderedDict{Dates.DateTime, DenseAxisArray{Float64}},
    }
    # Raw (unmultiplied) time-series parameter values, per execution, recast at finalize into
    # component-owned input series so the bundle's System can rebuild the model. Result rows
    # keep the multiplied values under the synthetic owner. See `buffer_parameter_inputs!`.
    dm_input_windows::Dict{
        Symbol,
        Dict{IOM.ParameterKey, Dict{Dates.DateTime, DenseAxisArray{Float64, 2}}},
    }
    em_input_values::Dict{
        IOM.ParameterKey,
        OrderedDict{Dates.DateTime, DenseAxisArray{Float64, 2}},
    }
    input_descriptors::Dict{Tuple{Symbol, IOM.ParameterKey}, POM.InputSeriesDescriptor}
    # Keys already warned about by the 3-D `_buffer_input_values!` arm, so a multi-execution
    # simulation warns once per (model, key) instead of once per execution.
    warned_3d_input_keys::Set{Tuple{Symbol, IOM.ParameterKey}}
    # Parameter keys have no HDF5-backed dataset to enumerate `keys(...)` from (unlike every
    # other container type), so `list_decision_model_keys`/`list_emulation_model_keys` read
    # this registry instead. Populated once, at `initialize_problem_storage!`; the per-step
    # decision-state sync (`_update_simulation_state_parameters!`) depends on it.
    dm_parameter_keys::Dict{Symbol, Vector{IOM.ParameterKey}}
    em_parameter_keys::Vector{IOM.ParameterKey}
    # Parameter rows read back from a model's bundle sidecar, filled on first use per
    # (model, key)/key so a sidecar is opened at most once per key. See
    # `_bundle_parameter_windows`/`_bundle_parameter_array`.
    parameter_read_cache::Dict{
        Tuple{Symbol, IOM.ParameterKey},
        Dict{String, Dict{Dates.DateTime, Vector{Float64}}},
    }
    # The 3-D counterpart of `parameter_read_cache`: one axis-2 slice's rows nested inside
    # another. See `_bundle_parameter_windows_3d`.
    parameter_read_cache_3d::Dict{
        Tuple{Symbol, IOM.ParameterKey},
        Dict{String, Dict{String, Dict{Dates.DateTime, Vector{Float64}}}},
    }
    em_parameter_read_cache::Dict{IOM.ParameterKey, Dict{String, IS.TimeSeries.TimeArray}}
    # A caller's own already-open `System` for a model's bundle (R30), keyed by system uuid.
    # Merged in from a `SimulationProblemResults`'s shared `system_registry` by
    # `_register_borrowed_stores!` (simulation_problem_results.jl) before a store read -- so
    # this includes systems loaded through any sibling result from the same SimulationResults,
    # e.g. the Emulator borrowing a decision model's bundle (R31). When present for a uuid,
    # the parameter read path reads through it instead of opening a second handle to the same
    # sidecar (InfraStore allows only one) -- and never closes it, since it is owned by the
    # caller's `System`, not by this store. Empty by default: a store with no caller-registered
    # System behaves exactly as before.
    borrowed_parameter_stores::Dict{Base.UUID, POM.ParameterTimeSeriesStore}
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

    if mode == "rw"
        hdf5_mode = "r+"
    else
        hdf5_mode = mode
    end
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
        Dict{
            Symbol,
            Dict{IOM.ParameterKey, Dict{Dates.DateTime, DenseAxisArray{Float64}}},
        }(),
        Dict{IOM.ParameterKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}(),
        Dict{
            Symbol,
            Dict{IOM.ParameterKey, Dict{Dates.DateTime, DenseAxisArray{Float64, 2}}},
        }(),
        Dict{IOM.ParameterKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64, 2}}}(),
        Dict{Tuple{Symbol, IOM.ParameterKey}, POM.InputSeriesDescriptor}(),
        Set{Tuple{Symbol, IOM.ParameterKey}}(),
        Dict{Symbol, Vector{IOM.ParameterKey}}(),
        IOM.ParameterKey[],
        Dict{
            Tuple{Symbol, IOM.ParameterKey},
            Dict{String, Dict{Dates.DateTime, Vector{Float64}}},
        }(),
        Dict{
            Tuple{Symbol, IOM.ParameterKey},
            Dict{String, Dict{String, Dict{Dates.DateTime, Vector{Float64}}}},
        }(),
        Dict{IOM.ParameterKey, Dict{String, IS.TimeSeries.TimeArray}}(),
        Dict{Base.UUID, POM.ParameterTimeSeriesStore}(),
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
        if !isnothing(store)
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
    return HDF5.isopen(store.file)
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
Parameter keys have no HDF5-backed dataset to read `keys(...)` from; they come from
`dm_parameter_keys` instead (populated at `initialize_problem_storage!`).
"""
function list_decision_model_keys(
    store::HdfSimulationStore,
    model::Symbol,
    container_type::Symbol,
)
    return _list_decision_model_keys(store, model, Val(container_type))
end

_list_decision_model_keys(
    store::HdfSimulationStore,
    model::Symbol,
    ::Val{STORE_CONTAINER_PARAMETERS},
) =
    get(store.dm_parameter_keys, model, IOM.ParameterKey[])

function _list_decision_model_keys(
    store::HdfSimulationStore,
    model::Symbol,
    ::Val{T},
) where {T}
    container = getfield(get_dm_data(store)[model], T)
    return collect(keys(container))
end

function list_emulation_model_keys(store::HdfSimulationStore, container_type::Symbol)
    return _list_emulation_model_keys(store, Val(container_type))
end

_list_emulation_model_keys(store::HdfSimulationStore, ::Val{STORE_CONTAINER_PARAMETERS}) =
    store.em_parameter_keys

function _list_emulation_model_keys(store::HdfSimulationStore, ::Val{T}) where {T}
    container = getfield(get_em_data(store), T)
    return collect(keys(container))
end

function write_optimizer_stats!(
    store::HdfSimulationStore,
    model::IOM.AbstractOptimizationModel,
    ::DecisionModelIndexType,
)
    stats = get_optimizer_stats(model)
    model_name = get_name(model)
    dataset = _get_dataset(OptimizerStats, store, model_name)

    # Uncomment for performance measures of HDF Store
    dataset[:, store.optimizer_stats_write_index[model_name]] = to_matrix(stats)

    store.optimizer_stats_write_index[model_name] += 1
    return
end

function write_optimizer_stats!(
    store::HdfSimulationStore,
    model::IOM.AbstractOptimizationModel,
    ::EmulationModelIndexType,
)
    return
end

"""
Read the optimizer stats for a problem execution.
"""
function IOM.read_optimizer_stats(
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
function IOM.read_optimizer_stats(store::HdfSimulationStore, model_name)
    dataset = _get_dataset(OptimizerStats, store, model_name)
    data = permutedims(dataset[:, :])
    stats = [IS.to_namedtuple(OptimizerStats(data[i, :])) for i in axes(data)[1]]
    return DataFrames.DataFrame(stats)
end

"""
Register a decision model's parameter keys in the lookup and in `dm_parameter_keys`, and
create nothing else: no HDF5 group, no dataset, no output cache. Parameter values are buffered
in memory (`write_result!`) and materialized into the bundle's InfraStore at
`finalize_parameters!`.
"""
function _initialize_decision_model_container!(
    ::Val{STORE_CONTAINER_PARAMETERS},
    problem_group,
    dm_reqs::SimulationModelStoreRequirements,
    store::HdfSimulationStore,
    problem::Symbol,
    initial_time::Dates.DateTime,
    problem_params::ModelStoreParams,
    flush_rules::CacheFlushRules,
    container_key_lookup::Dict{String, OptimizationContainerKey},
)
    problem_keys = IOM.ParameterKey[]
    for (key, _) in getfield(dm_reqs, STORE_CONTAINER_PARAMETERS)
        !should_write_resulting_value(key) && continue
        container_key_lookup[encode_key_as_string(key)] = key
        push!(problem_keys, key)
    end
    store.dm_parameter_keys[problem] = problem_keys
    return nothing
end

function _initialize_decision_model_container!(
    ::Val{T},
    problem_group,
    dm_reqs::SimulationModelStoreRequirements,
    store::HdfSimulationStore,
    problem::Symbol,
    initial_time::Dates.DateTime,
    problem_params::ModelStoreParams,
    flush_rules::CacheFlushRules,
    container_key_lookup::Dict{String, OptimizationContainerKey},
) where {T}
    group = _get_group_or_create(problem_group, string(T))
    for (key, reqs) in getfield(dm_reqs, T)
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
        datasets = getfield(get_dm_data(store)[problem], T)
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
        container_key_lookup[name] = key
    end
    return nothing
end

"""
Register an emulation model's parameter keys in the lookup and in `em_parameter_keys`, and
create nothing else. See [`_initialize_decision_model_container!`](@ref) for the
decision-model equivalent.
"""
function _initialize_emulation_model_container!(
    ::Val{STORE_CONTAINER_PARAMETERS},
    emulation_group,
    em_reqs::SimulationModelStoreRequirements,
    store::HdfSimulationStore,
    initial_time::Dates.DateTime,
    emulation_params::ModelStoreParams,
    container_key_lookup::Dict{String, OptimizationContainerKey},
)
    for (key, _) in getfield(em_reqs, STORE_CONTAINER_PARAMETERS)
        container_key_lookup[encode_key_as_string(key)] = key
        push!(store.em_parameter_keys, key)
    end
    return nothing
end

function _initialize_emulation_model_container!(
    ::Val{T},
    emulation_group,
    em_reqs::SimulationModelStoreRequirements,
    store::HdfSimulationStore,
    initial_time::Dates.DateTime,
    emulation_params::ModelStoreParams,
    container_key_lookup::Dict{String, OptimizationContainerKey},
) where {T}
    group = _get_group_or_create(emulation_group, string(T))
    for (key, reqs) in getfield(em_reqs, T)
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
        datasets = getfield(store.em_data, T)
        column_lengths = reqs["dims"][2:end]
        datasets[key] = HDF5Dataset{length(column_lengths)}(
            dataset,
            column_dataset,
            column_lengths,
            get_resolution(emulation_params),
            initial_time,
        )
        container_key_lookup[name] = key
    end
    return nothing
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
            _initialize_decision_model_container!(
                Val(type),
                problem_group,
                dm_problem_reqs[problem],
                store,
                problem,
                initial_time,
                problem_params,
                flush_rules,
                container_key_lookup,
            )
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
            _initialize_emulation_model_container!(
                Val(type),
                emulation_group,
                em_problem_reqs,
                store,
                initial_time,
                emulation_params,
                container_key_lookup,
            )
        end
    end
    buf = IOBuffer()
    Serialization.serialize(buf, container_key_lookup)
    seek(buf, 0)
    root[SERIALIZED_KEYS_PATH] = buf.data

    # `execute!` reopens the store fresh (`"rw"`, a new `HdfSimulationStore` struct), so this
    # registry must round-trip through the file like `container_key_lookup` above, or the
    # live per-step decision-state sync (`_update_simulation_state_parameters!`) sees no
    # parameter keys for any model once the run starts.
    param_keys_buf = IOBuffer()
    Serialization.serialize(
        param_keys_buf,
        (store.dm_parameter_keys, store.em_parameter_keys),
    )
    seek(param_keys_buf, 0)
    root[PARAMETER_KEYS_PATH] = param_keys_buf.data

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

"""
A `ParameterKey` is never cached — `write_result!` bypasses `add_output_cache!` entirely for
parameters (buffered or bundle-backed instead) — so the generic 4-arg `is_cached` must not
run for one: it wraps `(model_name, key)` into an `OptimizationResultCacheKey` and looks it up
in `cache.data`, a `KeyError` for a key nobody ever registered there. This dispatches on the
key before that wrap happens, for every `read_result(Array/DenseAxisArray, ...)` call.
"""
is_cached(::OptimizationOutputCaches, ::Symbol, ::IOM.ParameterKey, ::Any) = false

"""
The decision model's buffered windows for `key`, or an empty `Dict` when nothing has been
buffered for it (either it was never written this run, or `finalize_parameters!` already
emptied the buffer into the bundle).
"""
function _dm_buffered_windows(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
)::Dict{Dates.DateTime, DenseAxisArray{Float64}}
    haskey(store.dm_parameter_windows, model_name) ||
        return Dict{Dates.DateTime, DenseAxisArray{Float64}}()
    model_windows = store.dm_parameter_windows[model_name]
    haskey(model_windows, key) && return model_windows[key]
    return Dict{Dates.DateTime, DenseAxisArray{Float64}}()
end

"""
Whether the registered borrowed store for `uuid`, if any, is genuinely backed by
`sidecar_path` — the exact bundle sidecar a read for this model would otherwise open.
`SimulationProblemResults.system` is set unconditionally at construction
(`SimulationResults(sim::Simulation)` passes each decision model's own live, in-memory
`System`), which predates `finalize_parameters!`'s writes and is backed by a different store
than the bundle file entirely; registering it on every non-`nothing` `get_system(res)` (R30)
would otherwise silently misdirect a parameter read to a store that never has the row. Path
comparison is what makes only a genuine `get_system!`-reloaded `System` (backed by
`PSY.from_file` on this exact sidecar) usable this way -- the live in-memory case simply
does not match, so the caller falls through to opening the bundle sidecar directly, which is
always safe (nothing else has that specific file open).
"""
function _has_borrowed_store(
    store::HdfSimulationStore,
    uuid::Base.UUID,
    sidecar_path::AbstractString,
)::Bool
    haskey(store.borrowed_parameter_stores, uuid) || return false
    borrowed = store.borrowed_parameter_stores[uuid]
    return IS.get_file_path(borrowed.store) == abspath(sidecar_path)
end

"""
Run `f` on the parameter store backing `sidecar_path`: a borrowed view of an already-open
`System` when one is registered for `uuid` and genuinely backed by that file (never closed
here, since it is owned by the caller's `System`), else a fresh open that is closed
afterwards. InfraStore allows only one open handle per store file per process, so reusing a
borrowed store -- instead of opening a second handle -- is what lets a read succeed while a
caller's `System` is still open.
"""
function _with_parameter_store(
    f,
    store::HdfSimulationStore,
    uuid::Base.UUID,
    sidecar_path::AbstractString,
)
    if _has_borrowed_store(store, uuid, sidecar_path)
        return f(store.borrowed_parameter_stores[uuid])
    end
    pstore = POM.open_parameter_store(sidecar_path)
    try
        return f(pstore)
    finally
        POM.close_parameter_store!(pstore)
    end
end

"""
The bundle sidecar's parameter windows for one decision model's key: read through a
borrowed, already-open `System` store when one is registered for this model's system uuid
and genuinely backed by this sidecar, or a fresh open-read-close otherwise
(`_with_parameter_store`). Cached per `(model, key)` so a sidecar is read at most once
across a `SimulationProblemResults` session, whichever source served it.
"""
function _bundle_parameter_windows(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
)::Dict{String, Dict{Dates.DateTime, Vector{Float64}}}
    cache_key = (model_name, key)
    haskey(store.parameter_read_cache, cache_key) &&
        return store.parameter_read_cache[cache_key]
    params = get_decision_model_params(store, model_name)
    uuid = get_system_uuid(params)
    bundle_dir = _bundle_dir(store, model_name, uuid)
    sidecar_path = joinpath(bundle_dir, PSY.TIME_SERIES_FILE)
    extra_features = Dict{String, Any}("model" => string(model_name))
    windows = _with_parameter_store(store, uuid, sidecar_path) do pstore
        POM.read_parameter_windows(pstore, key; extra_features = extra_features)
    end
    store.parameter_read_cache[cache_key] = windows
    return windows
end

"""
Distinguishes a 2-D decision-model parameter window (`(label, time)`, one series per axis-1
label) from a 3-D one (`(label, label2, time)`, written sliced per axis-2 label — see
`_write_parameter_windows!`). Whichever source serves a window (buffer or bundle) is
dispatched through this same trait so the assembly logic for each shape is written once.
Selected from the bundle's discovered `"axis2"` slice list (`_dm_parameter_slice_labels`):
empty means 2-D, the only place this check is made.
"""
abstract type _ParameterShape end
struct _TwoD <: _ParameterShape end
struct _ThreeD <: _ParameterShape end

function _dm_parameter_shape(slice_labels::Vector{String})
    isempty(slice_labels) && return _TwoD()
    return _ThreeD()
end

"""
The distinct `"axis2"` values `key`'s bundle rows carry for this model
(`POM.parameter_slice_labels`, already sorted) — empty for a 2-D parameter. A discovery-only
open (through a borrowed store when one is registered, else open-close, same as
`_bundle_parameter_windows`); the rows themselves are read and cached separately
(`_bundle_parameter_windows`/`_bundle_parameter_windows_3d`). Bundle-only: a buffered (mid-run)
window's shape is read directly off its own array dimensionality instead
(`_dm_buffered_columns`/`_dm_window_result`), never through this.
"""
function _dm_parameter_slice_labels(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
)::Vector{String}
    params = get_decision_model_params(store, model_name)
    uuid = get_system_uuid(params)
    bundle_dir = _bundle_dir(store, model_name, uuid)
    sidecar_path = joinpath(bundle_dir, PSY.TIME_SERIES_FILE)
    base_features = Dict{String, Any}("model" => string(model_name))
    return _with_parameter_store(store, uuid, sidecar_path) do pstore
        POM.parameter_slice_labels(pstore, key; extra_features = base_features)
    end
end

"""
The bundle sidecar's 3-D parameter windows for one decision model's key: one
`POM.read_parameter_windows` call per `"axis2"` slice (mirrors the write side's own
per-slice writes, `_write_parameter_windows!`, and the partition merge's read,
`_decision_merge_units` in `simulation_partition_results.jl`), keyed by slice label, then
axis-1 label, then initial time. Read through a borrowed store or a fresh open-read-close,
same as `_bundle_parameter_windows`. Cached per `(model, key)`, keyed by the slice list that
discovered it, so a sidecar's slices are read at most once across a
`SimulationProblemResults` session.
"""
function _bundle_parameter_windows_3d(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    slice_labels::Vector{String},
)::Dict{String, Dict{String, Dict{Dates.DateTime, Vector{Float64}}}}
    cache_key = (model_name, key)
    haskey(store.parameter_read_cache_3d, cache_key) &&
        return store.parameter_read_cache_3d[cache_key]
    params = get_decision_model_params(store, model_name)
    uuid = get_system_uuid(params)
    bundle_dir = _bundle_dir(store, model_name, uuid)
    sidecar_path = joinpath(bundle_dir, PSY.TIME_SERIES_FILE)
    base_features = Dict{String, Any}("model" => string(model_name))
    rows = _with_parameter_store(store, uuid, sidecar_path) do pstore
        Dict{String, Dict{String, Dict{Dates.DateTime, Vector{Float64}}}}(
            label2 => POM.read_parameter_windows(
                pstore,
                key;
                extra_features = merge(
                    base_features,
                    Dict{String, Any}("axis2" => label2),
                ),
            ) for label2 in slice_labels
        )
    end
    store.parameter_read_cache_3d[cache_key] = rows
    return rows
end

"""
The column axes for a decision-model parameter: `(labels,)` for a 2-D parameter, or
`(labels, labels2)` for a 3-D one. Same source precedence as `_read_result` (buffer while
mid-run, bundle once `finalize_parameters!` empties it), and the same sorted-label
fallback in both cases: unlike every other container type, a parameter has no HDF5 column
dataset persisting its array's original label order
(`_initialize_decision_model_container!`'s `STORE_CONTAINER_PARAMETERS` method creates no
dataset at all), so a caller never sees the column order change depending on when it reads.
"""
function _dm_parameter_columns(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
)::Tuple
    buffered = _dm_buffered_windows(store, model_name, key)
    !isempty(buffered) && return _dm_buffered_columns(first(values(buffered)))
    slice_labels = _dm_parameter_slice_labels(store, model_name, key)
    return _dm_bundle_columns(
        _dm_parameter_shape(slice_labels),
        store,
        model_name,
        key,
        slice_labels,
    )
end

_dm_buffered_columns(window::DenseAxisArray{Float64, 2}) =
    (sort!(collect(axes(window, 1))),)
function _dm_buffered_columns(window::DenseAxisArray{Float64, 3})
    return (sort!(collect(axes(window, 1))), sort!(collect(axes(window, 2))))
end

function _dm_bundle_columns(
    ::_TwoD,
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    ::Vector{String},
)
    return (sort!(collect(keys(_bundle_parameter_windows(store, model_name, key)))),)
end

function _dm_bundle_columns(
    ::_ThreeD,
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    slice_labels::Vector{String},
)
    rows = _bundle_parameter_windows_3d(store, model_name, key, slice_labels)
    labels = sort!(collect(keys(rows[first(slice_labels)])))
    return (labels, slice_labels)
end

"""
`(data, columns)` for one buffered decision-model parameter window: `data` is
`(horizon, num_labels)` for a 2-D window (`columns = (labels,)`), or `time × label × label2`
for a 3-D window (`columns = (labels, labels2)`) — the shapes `_make_denseaxisarray`'s
`Matrix`/2-column and `Array{Float64,3}`/2-column methods expect. The 3-D axis order mirrors
the write side: axis-1 label, axis-2 label2, axis-3 time (`_write_parameter_windows!`). Both
label axes sorted, same reason as `_dm_parameter_columns`.
"""
function _dm_window_result(::IOM.ParameterKey, window::DenseAxisArray{Float64, 2})
    labels = sort!(collect(axes(window, 1)))
    data = reduce(hcat, (window[label, :] for label in labels))
    return data, (labels,)
end

function _dm_window_result(::IOM.ParameterKey, window::DenseAxisArray{Float64, 3})
    labels = sort!(collect(axes(window, 1)))
    labels2 = sort!(collect(axes(window, 2)))
    mats = map(labels2) do label2
        reduce(hcat, (window[label, label2, :] for label in labels))
    end
    data = cat(mats...; dims = 3)
    return data, (labels, labels2)
end

function _dm_window_result(
    key::IOM.ParameterKey,
    ::DenseAxisArray{Float64, N},
) where {N}
    error(
        "reading a $(N)-D parameter window for $key is not supported by this read path",
    )
end

"""
`(data, columns)` for one decision-model parameter's realized window at execution
`initial_time`, from whichever source is in play: the buffer while mid-run (before
`finalize_parameters!` empties it into the bundle), or the bundle's InfraStore once the run
has finished. Dispatches 2-D vs 3-D through `_dm_window_result` (buffer, off the array's own
dimensionality) or `_dm_bundle_result` (bundle, off the discovered `"axis2"` slice list) —
`_read_result` itself makes only the buffer-vs-bundle choice, never a shape choice. Errors,
naming the model, key and time, when the requested window is missing from whichever source
is in play — never a `NaN`-filled fallback. `read_result(Array/DenseAxisArray/DataFrame, ...)`
and `_read_data_columns` for a `DecisionModelIndexType` all funnel through `_read_result`
(mirroring the `OptimizationContainerKey` methods above), so overriding it here is the
narrowest point that keeps every caller unchanged.
"""
function _read_result(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    initial_time::DecisionModelIndexType,
)
    buffered = _dm_buffered_windows(store, model_name, key)
    if !isempty(buffered)
        haskey(buffered, initial_time) || error(
            "no buffered parameter window for model $model_name, key $key, time $initial_time",
        )
        return _dm_window_result(key, buffered[initial_time])
    end
    return _dm_bundle_result(store, model_name, key, initial_time)
end

function _dm_bundle_result(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    initial_time::Dates.DateTime,
)
    slice_labels = _dm_parameter_slice_labels(store, model_name, key)
    return _dm_bundle_result(
        _dm_parameter_shape(slice_labels),
        store,
        model_name,
        key,
        initial_time,
        slice_labels,
    )
end

function _dm_bundle_result(
    ::_TwoD,
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    initial_time::Dates.DateTime,
    ::Vector{String},
)
    rows = _bundle_parameter_windows(store, model_name, key)
    labels = sort!(collect(keys(rows)))
    for label in labels
        haskey(rows[label], initial_time) || error(
            "no parameter window for model $model_name, key $key, time $initial_time",
        )
    end
    data = reduce(hcat, (rows[label][initial_time] for label in labels))
    return data, (labels,)
end

function _dm_bundle_result(
    ::_ThreeD,
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    initial_time::Dates.DateTime,
    slice_labels::Vector{String},
)
    slices = _bundle_parameter_windows_3d(store, model_name, key, slice_labels)
    labels = sort!(collect(keys(slices[first(slice_labels)])))
    mats = map(slice_labels) do label2
        rows = slices[label2]
        for label in labels
            haskey(rows, label) && haskey(rows[label], initial_time) || error(
                "no parameter window for model $model_name, key $key, axis2 $label2, " *
                "label $label, time $initial_time",
            )
        end
        reduce(hcat, (rows[label][initial_time] for label in labels))
    end
    data = cat(mats...; dims = 3)
    return data, (labels, slice_labels)
end

function get_column_names(
    store::HdfSimulationStore,
    ::Type{DecisionModelIndexType},
    model_name::Symbol,
    key::IOM.ParameterKey,
)
    !isopen(store) && throw(ArgumentError("store must be opened prior to reading"))
    return _dm_parameter_columns(store, model_name, key)
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

function get_number_of_dimensions(
    store::HdfSimulationStore,
    i::Type{DecisionModelIndexType},
    model_name::Symbol,
    key::OptimizationContainerKey,
)
    return length(get_column_names(store, i, model_name, key))
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
    num_dims = ndims(dset)
    if num_dims == 2
        data = dset[index, :]
    elseif num_dims == 3
        data = dset[index, :, :]
    else
        error("Unsupported number of dimensions for emulation dataset: $num_dims")
    end
    columns = get_column_names(key, dataset)
    if ndims(data) == 1
        data = permutedims(data)
    end
    return data, columns
end

"""
The only emulation model in a store's params (`get_emulation_model_params` already asserts
there is exactly one).
"""
_em_model_name(store::HdfSimulationStore) = first(keys(store.params.emulation_model_params))

"""
The emulation model's bundle sidecar's realized series for one parameter key: read through a
borrowed store or a fresh open-read-close, same as `_bundle_parameter_windows`
(`_with_parameter_store`).
"""
function _bundle_parameter_array(
    store::HdfSimulationStore,
    key::IOM.ParameterKey,
)::Dict{String, IS.TimeSeries.TimeArray}
    haskey(store.em_parameter_read_cache, key) && return store.em_parameter_read_cache[key]
    model_name = _em_model_name(store)
    uuid = get_system_uuid(get_emulation_model_params(store))
    bundle_dir = _emulation_bundle_dir(store)
    sidecar_path = joinpath(bundle_dir, PSY.TIME_SERIES_FILE)
    extra_features = Dict{String, Any}("model" => string(model_name))
    series = _with_parameter_store(store, uuid, sidecar_path) do pstore
        POM.read_parameter_array(pstore, key; extra_features = extra_features)
    end
    store.em_parameter_read_cache[key] = series
    return series
end

"""
One label's value out of a buffered emulation-model parameter array. The array written by
`_update_system_state!`'s per-step `Emulator` collection (`get_decision_state_value`, which
drops the time axis) is `DenseAxisArray{Float64,1}`; a real `POM.EmulationModel`'s own
parameter write is `DenseAxisArray{Float64,2}` with a length-1 second axis (mirroring the
non-parameter `EmulationModelIndexType` `write_result!` methods above). Both are handled by
dispatch; a genuine 3-D emulation-model parameter has no test coverage and errors here.
"""
_em_parameter_value(window::DenseAxisArray{Float64, 1}, label::AbstractString) =
    window[label]
_em_parameter_value(window::DenseAxisArray{Float64, 2}, label::AbstractString) =
    only(window[label, :])
function _em_parameter_value(
    window::DenseAxisArray{Float64, N},
    label::AbstractString,
) where {N}
    error(
        "reading a $(N)-D emulation-model parameter value is not supported by this read path",
    )
end

"""
An emulation-model parameter's realized values, one vector per axis-1 label (sorted, for
the same reason `_dm_parameter_labels` sorts: no persisted column order exists for a
parameter), each vector ordered by execution. Served from the buffer while mid-run, from the
bundle's InfraStore once `finalize_parameters!` has emptied it.
"""
function _em_parameter_series(
    store::HdfSimulationStore,
    key::IOM.ParameterKey,
)::Tuple{Vector{String}, Dict{String, Vector{Float64}}}
    if haskey(store.em_parameter_values, key) && !isempty(store.em_parameter_values[key])
        buffered = store.em_parameter_values[key]
        timestamps = collect(keys(buffered))
        labels = sort!(collect(axes(first(values(buffered)), 1)))
        series = Dict{String, Vector{Float64}}(
            label => [_em_parameter_value(buffered[t], label) for t in timestamps] for
            label in labels
        )
        return labels, series
    end
    rows = _bundle_parameter_array(store, key)
    labels = sort!(collect(keys(rows)))
    series = Dict{String, Vector{Float64}}(
        label => IS.TimeSeries.values(rows[label]) for label in labels
    )
    return labels, series
end

"""
Number of executions already buffered for an emulation-model parameter (R31): the buffer's
own equivalent of `get_last_recorded_row(em_store, key)`, which indexes `em_store` by key
and has no entry for a `ParameterKey`. Used by `simulation.jl`'s `_write_state_rows!`
override for `HdfSimulationStore`.
"""
function _last_em_parameter_row(store::HdfSimulationStore, key::IOM.ParameterKey)
    haskey(store.em_parameter_values, key) || return 0
    return length(store.em_parameter_values[key])
end

"""
Timestamp of the most recently buffered execution for an emulation-model parameter (R31):
the buffer's own equivalent of `get_last_updated_timestamp(em_store, key)`. `em_parameter_values`
is keyed by an `OrderedDict`, so its last key is the most recently buffered timestamp.
"""
function _last_em_parameter_update_time(store::HdfSimulationStore, key::IOM.ParameterKey)
    haskey(store.em_parameter_values, key) || return UNSET_INI_TIME
    values_by_time = store.em_parameter_values[key]
    isempty(values_by_time) && return UNSET_INI_TIME
    return collect(keys(values_by_time))[end]
end

function get_emulation_model_dataset_size(store::HdfSimulationStore, key::IOM.ParameterKey)
    _, series = _em_parameter_series(store, key)
    return length(first(values(series)))
end

function read_results(
    store::HdfSimulationStore,
    key::IOM.ParameterKey;
    index::Union{Nothing, EmulationModelIndexType} = nothing,
    len::Union{Nothing, Int} = nothing,
)
    labels, series = _em_parameter_series(store, key)
    num_executions = length(first(values(series)))
    if isnothing(index)
        @assert_op(isnothing(len))
        first_index = 1
        last_index = num_executions
    elseif isnothing(len)
        first_index = index
        last_index = num_executions
    else
        first_index = index
        last_index = index + len - 1
    end
    last_index > num_executions && throw(
        ArgumentError(
            "index = $index, len = $len exceeds the number of executions " *
            "($num_executions) for $key",
        ),
    )
    data = reduce(hcat, (series[label][first_index:last_index] for label in labels))
    return DenseAxisArray(permutedims(data), labels, 1:size(data, 1))
end

"""
Feeds every `read_result`/`read_result(Array/DataFrame/DenseAxisArray, ...)` variant that
takes an `EmulationModelIndexType`: they all funnel through `_read_result`, so overriding it
here (rather than each of them) is the narrowest point that keeps every caller unchanged.
"""
function _read_result(
    store::HdfSimulationStore,
    ::Symbol,
    key::IOM.ParameterKey,
    index::EmulationModelIndexType,
)
    array = read_results(store, key; index = index, len = 1)
    return permutedims(array.data), (collect(axes(array, 1)),)
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
    num_dims = ndims(dset)
    if num_dims == 3
        data = dset[:, :, row_index]
    elseif num_dims == 4
        data = dset[:, :, :, row_index]
    else
        error("unsupported dims: $num_dims")
    end

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

    @debug "write_result" get_size(store.cache) encode_key_as_string(key)
    return
end

"""
Write a decision-model result whose container is a `SparseAxisArray`. The
sparse container is flattened to a `(horizon × n_cols)` `Matrix{Float64}` via
`to_matrix`, where columns are the unique non-time tuple keys (e.g. for
post-contingency flows: `(outage_id, branch_name)`). Cache and HDF5 dataset
shapes match the 3D dense path: `(horizon, n_cols, num_results)`.
"""
function write_result!(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    ::Dates.DateTime,
    data::SparseAxisArray{Float64},
)
    output_cache = get_output_cache(store.cache, model_name, key)
    cur_size = get_size(store.cache)
    add_result!(output_cache, index, to_matrix(data), is_full(store.cache, cur_size))

    if get_dirty_size(output_cache) >= get_min_flush_size(store.cache)
        discard = !should_keep_in_cache(output_cache)
        size_flushed = _flush_data!(output_cache, store, model_name, key, discard)
        @debug "flushed data" LOG_GROUP_SIMULATION_STORE key size_flushed discard cur_size
    end

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
    if size(array, 2) == 1
        data = reshape(array.data, length(array.data))
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

"""
Buffer a decision-model parameter window in memory, keyed by `index` — the execution's initial
time, the same value `write_results!` passes as both `index` and the update timestamp
(`decision_model.jl`'s `write_results!(store, model, start_time, start_time; ...)`). Never
reaches a dataset: materialized into the bundle's InfraStore only at `finalize_parameters!`.
"""
function _buffer_dm_parameter_window!(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    initial_time::Dates.DateTime,
    data::DenseAxisArray{Float64},
)
    model_windows = get!(
        () -> Dict{IOM.ParameterKey, Dict{Dates.DateTime, DenseAxisArray{Float64}}}(),
        store.dm_parameter_windows,
        model_name,
    )
    windows =
        get!(() -> Dict{Dates.DateTime, DenseAxisArray{Float64}}(), model_windows, key)
    windows[initial_time] = data
    return nothing
end

function write_result!(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    index::DecisionModelIndexType,
    ::Dates.DateTime,
    data::DenseAxisArray{Float64, 2, <:NTuple{2, Any}},
)
    _buffer_dm_parameter_window!(store, model_name, key, index, data)
    return
end

function write_result!(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::IOM.ParameterKey,
    index::DecisionModelIndexType,
    ::Dates.DateTime,
    data::DenseAxisArray{Float64, 3, <:NTuple{3, Any}},
)
    _buffer_dm_parameter_window!(store, model_name, key, index, data)
    return
end

function write_result!(
    store::HdfSimulationStore,
    ::Symbol,
    key::IOM.ParameterKey,
    ::DecisionModelIndexType,
    ::Dates.DateTime,
    ::SparseAxisArray{Float64},
)
    error("sparse parameter arrays are not stored: $key")
end

"""
Buffer an emulation-model parameter value in memory, keyed by `simulation_time`. Materialized
into the bundle's InfraStore only at `finalize_parameters!`.
"""
function _buffer_em_parameter!(
    store::HdfSimulationStore,
    key::IOM.ParameterKey,
    simulation_time::Dates.DateTime,
    array::DenseAxisArray{Float64},
)
    values = get!(
        () -> OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}(),
        store.em_parameter_values,
        key,
    )
    values[simulation_time] = array
    return nothing
end

function write_result!(
    store::HdfSimulationStore,
    ::Symbol,
    key::IOM.ParameterKey,
    ::EmulationModelIndexType,
    simulation_time::Dates.DateTime,
    array::DenseAxisArray{Float64, 2},
)
    _buffer_em_parameter!(store, key, simulation_time, array)
    return
end

function write_result!(
    store::HdfSimulationStore,
    ::Symbol,
    key::IOM.ParameterKey,
    ::EmulationModelIndexType,
    simulation_time::Dates.DateTime,
    array::DenseAxisArray{Float64, 3},
)
    _buffer_em_parameter!(store, key, simulation_time, array)
    return
end

function write_result!(
    store::HdfSimulationStore,
    ::Symbol,
    key::IOM.ParameterKey,
    ::EmulationModelIndexType,
    simulation_time::Dates.DateTime,
    array::DenseAxisArray{Float64},
)
    _buffer_em_parameter!(store, key, simulation_time, array)
    return
end

"""
Buffer the raw values of every time-series parameter this execution read, plus (once per model
and key) where they belong: the series name and type from the container's `TimeSeriesAttributes`
and each label's owner in the model's System. `write_result!` receives the multiplied values, so
this is the only place the raw half is still available.
"""
function buffer_parameter_inputs!(
    store::HdfSimulationStore,
    model::IOM.AbstractOptimizationModel,
    index::DecisionModelIndexType,
    ::Dates.DateTime,
)
    _buffer_parameter_inputs!(store, model, index)
    return nothing
end

function buffer_parameter_inputs!(
    store::HdfSimulationStore,
    model::IOM.AbstractOptimizationModel,
    ::EmulationModelIndexType,
    simulation_time::Dates.DateTime,
)
    _buffer_parameter_inputs!(store, model, simulation_time)
    return nothing
end

function _buffer_parameter_inputs!(
    store::HdfSimulationStore,
    model::IOM.AbstractOptimizationModel,
    time_key::Dates.DateTime,
)
    model_name = get_name(model)
    sys = get_system(model)
    for (key, pc) in get_parameters(get_optimization_container(model))
        POM.is_input_parameter(key, pc) || continue
        get!(store.input_descriptors, (model_name, key)) do
            POM.input_series_descriptor(sys, key, pc)
        end
        _buffer_input_values!(
            store,
            model,
            model_name,
            key,
            time_key,
            IOM.get_parameter_values(pc),
        )
    end
    return nothing
end

function _buffer_input_values!(
    store::HdfSimulationStore,
    ::DecisionModel,
    model_name::Symbol,
    key::IOM.ParameterKey,
    initial_time::Dates.DateTime,
    raw::DenseAxisArray{Float64, 2},
)
    by_key = get!(
        () ->
            Dict{IOM.ParameterKey, Dict{Dates.DateTime, DenseAxisArray{Float64, 2}}}(),
        store.dm_input_windows,
        model_name,
    )
    windows = get!(() -> Dict{Dates.DateTime, DenseAxisArray{Float64, 2}}(), by_key, key)
    windows[initial_time] = raw
    return nothing
end

function _buffer_input_values!(
    store::HdfSimulationStore,
    ::EmulationModel,
    ::Symbol,
    key::IOM.ParameterKey,
    simulation_time::Dates.DateTime,
    raw::DenseAxisArray{Float64, 2},
)
    values = get!(
        () -> OrderedDict{Dates.DateTime, DenseAxisArray{Float64, 2}}(),
        store.em_input_values,
        key,
    )
    values[simulation_time] = raw
    return nothing
end

# A 3-D time-series parameter has no component-series shape; its multiplied values stay in
# the result rows (POM's `write_model_inputs!` makes the same choice). Warns once per
# (model, key), not once per execution.
function _buffer_input_values!(
    store::HdfSimulationStore,
    ::IOM.AbstractOptimizationModel,
    model_name::Symbol,
    key::IOM.ParameterKey,
    ::Dates.DateTime,
    ::DenseAxisArray{Float64, 3},
)
    warn_key = (model_name, key)
    if warn_key ∉ store.warned_3d_input_keys
        push!(store.warned_3d_input_keys, warn_key)
        @warn "$(encode_key_as_string(key)) is a 3-D time-series parameter; not recast into the bundle"
    end
    return nothing
end

_simulation_folder(store::HdfSimulationStore) = dirname(dirname(store.file.filename))

_problem_dir(store::HdfSimulationStore, model_name::Symbol) =
    joinpath(_simulation_folder(store), "problems", string(model_name))

function _bundle_dir(store::HdfSimulationStore, model_name::Symbol, system_uuid::Base.UUID)
    bundle_dir = joinpath(
        _problem_dir(store, model_name),
        IOM.make_system_dirname(system_uuid),
    )
    !isdir(bundle_dir) && error(
        "no system bundle found for model $model_name at $bundle_dir; it must exist from build",
    )
    return bundle_dir
end

"""
The bundle directory an emulation model's parameters live in (R31, Task 10b). Two cases:

  - A real `POM.EmulationModel` has its own `System` — a different uuid from every decision
    model — and `_write_system_bundles!` (Task 11) writes it a bundle at
    `problems/<em_model_name>/system-<uuid>` like any other model. Tried first: if that
    directory exists, it is the answer, regardless of what any decision model's uuid is.
  - The synthetic "Emulator" aggregator (`_initialize_problem_storage!`) has no `System` of
    its own; it borrows a decision model's, and never gets a `problems/Emulator/` directory
    written. Falls back to the first decision model whose params carry the same uuid
    (`store.params.decision_models_params`).

Errors, naming the uuid and both places searched, if neither resolves.
"""
function _emulation_bundle_dir(store::HdfSimulationStore)
    model_name = _em_model_name(store)
    uuid = get_system_uuid(get_emulation_model_params(store))
    own_dir = joinpath(_problem_dir(store, model_name), IOM.make_system_dirname(uuid))
    isdir(own_dir) && return own_dir
    for (dm_name, params) in store.params.decision_models_params
        get_system_uuid(params) == uuid && return _bundle_dir(store, dm_name, uuid)
    end
    error(
        "no bundle carries system uuid $uuid for the emulation model $model_name: " *
        "checked its own bundle at $own_dir and every decision model's bundle",
    )
end

"""
Write one decision-model parameter's realized windows into `pstore`, one `Deterministic` per
axis-1 label. A 3-D window set has no `Deterministic` counterpart, so it is sliced per axis-2
label into 2-D windows, each written with `"axis2"` added to `extra_features` (mirrors how POM's
3-D `write_parameter_array!` names that feature; time is the last axis in both).

`windows`' declared value type is the loosely-typed `DenseAxisArray{Float64}` buffered by
`write_result!` (any `N`), so dispatch reads the concrete dimensionality off one representative
window rather than off the `Dict`'s own (non-concrete) value type parameter.
"""
function _write_parameter_windows!(
    pstore,
    key::IOM.ParameterKey,
    windows::AbstractDict{Dates.DateTime, DenseAxisArray{Float64}},
    resolution::Dates.Period,
    interval::Dates.Period,
    model_name::Symbol,
)
    return _write_parameter_windows!(
        pstore,
        key,
        first(values(windows)),
        windows,
        resolution,
        interval,
        model_name,
    )
end

function _write_parameter_windows!(
    pstore,
    key::IOM.ParameterKey,
    ::DenseAxisArray{Float64, 2},
    windows::AbstractDict{Dates.DateTime, DenseAxisArray{Float64}},
    resolution::Dates.Period,
    interval::Dates.Period,
    model_name::Symbol,
)
    concrete_windows = Dict{Dates.DateTime, DenseAxisArray{Float64, 2}}(
        initial_time => window for (initial_time, window) in windows
    )
    POM.write_parameter_windows!(
        pstore,
        key,
        concrete_windows,
        resolution,
        interval;
        extra_features = Dict{String, Any}("model" => string(model_name)),
    )
    return nothing
end

function _write_parameter_windows!(
    pstore,
    key::IOM.ParameterKey,
    ::DenseAxisArray{Float64, 3},
    windows::AbstractDict{Dates.DateTime, DenseAxisArray{Float64}},
    resolution::Dates.Period,
    interval::Dates.Period,
    model_name::Symbol,
)
    labels2 = axes(first(values(windows)), 2)
    for label2 in labels2
        sliced = Dict{Dates.DateTime, DenseAxisArray{Float64, 2}}(
            initial_time => window[:, label2, :] for (initial_time, window) in windows
        )
        POM.write_parameter_windows!(
            pstore,
            key,
            sliced,
            resolution,
            interval;
            extra_features = Dict{String, Any}(
                "model" => string(model_name),
                "axis2" => string(label2),
            ),
        )
    end
    return nothing
end

function _finalize_decision_model_parameters!(
    store::HdfSimulationStore,
    model_name::Symbol,
    windows_by_key::Dict{IOM.ParameterKey, Dict{Dates.DateTime, DenseAxisArray{Float64}}},
)
    isempty(windows_by_key) && return nothing
    params = get_decision_model_params(store, model_name)
    bundle_dir = _bundle_dir(store, model_name, get_system_uuid(params))
    pstore = POM.open_parameter_store_writable(joinpath(bundle_dir, PSY.TIME_SERIES_FILE))
    try
        for (key, windows) in windows_by_key
            _write_parameter_windows!(
                pstore,
                key,
                windows,
                get_resolution(params),
                get_interval(params),
                model_name,
            )
        end
    finally
        POM.close_parameter_store!(pstore)
    end
    return nothing
end

function _finalize_emulation_model_parameters!(
    store::HdfSimulationStore,
    model_name::Symbol,
    values_by_key::Dict{
        IOM.ParameterKey,
        OrderedDict{Dates.DateTime, DenseAxisArray{Float64}},
    },
)
    isempty(values_by_key) && return nothing
    bundle_dir = _emulation_bundle_dir(store)
    pstore = POM.open_parameter_store_writable(joinpath(bundle_dir, PSY.TIME_SERIES_FILE))
    resolution = get_resolution(get_emulation_model_params(store))
    try
        for (key, values_by_time) in values_by_key
            timestamps = collect(keys(values_by_time))
            labels = axes(first(values(values_by_time)), 1)
            matrix = reduce(
                hcat,
                [vec(values_by_time[t].data) for t in timestamps],
            )
            array = DenseAxisArray(matrix, labels, 1:length(timestamps))
            try
                POM.write_parameter_array!(
                    pstore,
                    key,
                    array,
                    timestamps,
                    resolution;
                    extra_features = Dict{String, Any}("model" => string(model_name)),
                )
            catch e
                error(
                    "emulation model $model_name could not write parameter array for " *
                    "$key ($(length(timestamps)) time step(s)): $(sprint(showerror, e))",
                )
            end
        end
    finally
        POM.close_parameter_store!(pstore)
    end
    return nothing
end

function _finalize_decision_model_inputs!(
    store::HdfSimulationStore,
    model_name::Symbol,
    windows_by_key::Dict{
        IOM.ParameterKey,
        Dict{Dates.DateTime, DenseAxisArray{Float64, 2}},
    },
)
    isempty(windows_by_key) && return nothing
    params = get_decision_model_params(store, model_name)
    bundle_dir = _bundle_dir(store, model_name, get_system_uuid(params))
    pstore = POM.open_parameter_store_writable(joinpath(bundle_dir, PSY.TIME_SERIES_FILE))
    try
        for (key, windows) in windows_by_key
            POM.write_input_forecasts!(
                pstore,
                store.input_descriptors[(model_name, key)],
                windows,
                get_resolution(params),
                get_interval(params),
            )
        end
    finally
        POM.close_parameter_store!(pstore)
    end
    return nothing
end

function _finalize_emulation_model_inputs!(
    store::HdfSimulationStore,
    model_name::Symbol,
    values_by_key::Dict{
        IOM.ParameterKey,
        OrderedDict{Dates.DateTime, DenseAxisArray{Float64, 2}},
    },
)
    isempty(values_by_key) && return nothing
    bundle_dir = _emulation_bundle_dir(store)
    pstore = POM.open_parameter_store_writable(joinpath(bundle_dir, PSY.TIME_SERIES_FILE))
    resolution = get_resolution(get_emulation_model_params(store))
    try
        for (key, values_by_time) in values_by_key
            timestamps = collect(keys(values_by_time))
            labels = axes(first(values(values_by_time)), 1)
            matrix = reduce(hcat, [vec(values_by_time[t].data) for t in timestamps])
            array = DenseAxisArray(matrix, labels, 1:length(timestamps))
            POM.write_input_series!(
                pstore, store.input_descriptors[(model_name, key)], array, timestamps,
                resolution,
            )
        end
    finally
        POM.close_parameter_store!(pstore)
    end
    return nothing
end

"""
Materialize the buffered parameter slabs into each model's bundle store. Runs once, when the
simulation finishes writing results, before the store closes. Decision-model parameters become
forecast windows (one per execution); emulation-model parameters become one series per label
over the executions. The raw input windows recast the same execution's values into
component-owned series so the bundle's System can rebuild the model.
"""
function finalize_parameters!(store::HdfSimulationStore)
    for (model_name, windows_by_key) in store.dm_parameter_windows
        _finalize_decision_model_parameters!(store, model_name, windows_by_key)
    end
    empty!(store.dm_parameter_windows)

    if !isempty(store.em_parameter_values)
        em_model_name = first(keys(store.params.emulation_model_params))
        _finalize_emulation_model_parameters!(
            store,
            em_model_name,
            store.em_parameter_values,
        )
    end
    empty!(store.em_parameter_values)

    for (model_name, windows_by_key) in store.dm_input_windows
        _finalize_decision_model_inputs!(store, model_name, windows_by_key)
    end
    empty!(store.dm_input_windows)
    if !isempty(store.em_input_values)
        em_model_name = first(keys(store.params.emulation_model_params))
        _finalize_emulation_model_inputs!(store, em_model_name, store.em_input_values)
    end
    empty!(store.em_input_values)
    return nothing
end

"""
Sync an emulation model's just-written parameter value into `state`, mid-simulation. Mirrors
`update_system_state!(state, key::OptimizationContainerKey, store::SimulationStore, model_name,
simulation_time)` (`simulation_state.jl`), which reads the value back from the emulation
model's HDF5 dataset via `get_last_recorded_row(em_data, key)`/`read_result` — both unavailable
for a `ParameterKey`, since it has no dataset. Reads the just-buffered value directly instead
(`em_parameter_values`, the same buffer `finalize_parameters!` later writes into the bundle);
only valid mid-run, before that buffer is emptied. `HdfSimulationStore` in the store position
(rather than the generic `SimulationStore`) keeps `InMemorySimulationStore` on the original
method: it never moved parameters out of its normal per-key storage (see R19), so that path
still works there unchanged. No test system in this repo has a `POM.EmulationModel` with
parameters, so this method is implemented but untested (see report).
"""
function update_system_state!(
    state::DatasetContainer{InMemoryDataset},
    key::IOM.ParameterKey,
    store::HdfSimulationStore,
    ::Symbol,
    simulation_time::Dates.DateTime,
)
    values_by_time = get(store.em_parameter_values, key, nothing)
    isnothing(values_by_time) &&
        error("no buffered emulation-model parameter values for $key")
    _, array = last(values_by_time)
    dataset = get_dataset(state, key)
    set_update_timestamp!(dataset, simulation_time)
    set_dataset_values!(state, key, 1, array)
    set_last_recorded_row!(dataset, 1)
    return nothing
end

# Resolves the ambiguity between the method above (any `ParameterKey`, `HdfSimulationStore`)
# and the `EventParameter` no-op in simulation_state.jl (any `SimulationStore`, `EventParameter`
# key): neither is more specific than the other once both a narrower key and a narrower store
# are in play at once, so an event parameter read against an `HdfSimulationStore` needs its own,
# most-specific method. See the no-op's own comment for why it stays a no-op.
function update_system_state!(
    ::DatasetContainer{InMemoryDataset},
    ::ParameterKey{T, U},
    ::HdfSimulationStore,
    ::Symbol,
    ::Dates.DateTime,
) where {T <: EventParameter, U <: PSY.Component}
    return nothing
end

function _check_state(store::HdfSimulationStore)
    if has_dirty(store.cache)
        error("BUG!!! dirty cache is present at shutdown: $(store.file)")
    end
end

function _create_dataset(group, name, reqs)
    # No chunking or compression: read performance is the priority. A row that is never
    # written must read back as NaN, not as the HDF5 default of 0.0.
    dataset = HDF5.create_dataset(
        group,
        name,
        HDF5.datatype(Float64),
        HDF5.dataspace(reqs["dims"]);
        fill_value = NaN,
    )
    @debug "Created dataset for" group name size(dataset)
    return dataset
end

"""
On deserialize, a decision model's parameter keys have no dataset to reconstruct: buffered
parameter windows are never persisted to the HDF5 store (see `finalize_parameters!`), so there
is nothing under `"parameters"` to read back here.
"""
function _deserialize_decision_model_container!(
    ::Val{STORE_CONTAINER_PARAMETERS},
    problem_group,
    store::HdfSimulationStore,
    model_name::Symbol,
    initial_time::Dates.DateTime,
    container_key_lookup::Dict{String, OptimizationContainerKey},
)
    return nothing
end

function _deserialize_decision_model_container!(
    ::Val{T},
    problem_group,
    store::HdfSimulationStore,
    model_name::Symbol,
    initial_time::Dates.DateTime,
    container_key_lookup::Dict{String, OptimizationContainerKey},
) where {T}
    group = problem_group[string(T)]
    for name in keys(group)
        if !endswith(name, "columns")
            dataset = group[name]
            column_dataset = group[_make_column_name(name)]
            resolution = get_resolution(get_decision_model_params(store, model_name))
            column_lengths = size(dataset)[2:(end - 1)]
            item = HDF5Dataset{length(column_lengths)}(
                dataset,
                column_dataset,
                column_lengths,
                resolution,
                initial_time,
            )
            container_key = container_key_lookup[name]
            getfield(get_dm_data(store)[model_name], T)[container_key] = item
            add_output_cache!(store.cache, model_name, container_key, CacheFlushRule())
        end
    end
    return nothing
end

"""
On deserialize, an emulation model's parameter keys have no dataset to reconstruct. See
[`_deserialize_decision_model_container!`](@ref).
"""
function _deserialize_emulation_model_container!(
    ::Val{STORE_CONTAINER_PARAMETERS},
    em_group,
    store::HdfSimulationStore,
    model_name::Symbol,
    initial_time::Dates.DateTime,
    resolution::Dates.Period,
    container_key_lookup::Dict{String, OptimizationContainerKey},
)
    return nothing
end

function _deserialize_emulation_model_container!(
    ::Val{T},
    em_group,
    store::HdfSimulationStore,
    model_name::Symbol,
    initial_time::Dates.DateTime,
    resolution::Dates.Period,
    container_key_lookup::Dict{String, OptimizationContainerKey},
) where {T}
    group = em_group[string(T)]
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
            getfield(store.em_data, T)[container_key] = item
            add_output_cache!(store.cache, model_name, container_key, CacheFlushRule())
        end
    end
    return nothing
end

function _deserialize_attributes!(store::HdfSimulationStore)
    container_key_lookup = get_container_key_lookup(store)
    param_keys_buf = IOBuffer(_get_root(store)[PARAMETER_KEYS_PATH][:])
    store.dm_parameter_keys, store.em_parameter_keys =
        Serialization.deserialize(param_keys_buf)
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
            _deserialize_decision_model_container!(
                Val(type),
                problem_group,
                store,
                model_name,
                initial_time,
                container_key_lookup,
            )
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
        _deserialize_emulation_model_container!(
            Val(type),
            em_group,
            store,
            model_name,
            initial_time,
            resolution,
            container_key_lookup,
        )
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
    _write_dataset!(dataset.values, data, write_range)

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

function _read_data_columns(
    store::HdfSimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
)
    if is_cached(store.cache, model_name, key, index)
        data = read_result(store.cache, model_name, key, index)
        columns = get_column_names(key, _get_dm_dataset(store, model_name, key))
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
    return _read_result(store, model_name, key, index)
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
