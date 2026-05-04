"""
Stores simulation data in memory
"""
mutable struct InMemorySimulationStore <: SimulationStore
    params::SimulationStoreParams
    dm_data::OrderedDict{Symbol, DecisionModelStore}
    em_data::EmulationModelStore
    container_key_lookup::Dict{String, OptimizationContainerKey}
end

function InMemorySimulationStore()
    return InMemorySimulationStore(
        SimulationStoreParams(),
        OrderedDict{Symbol, DecisionModelStore}(),
        EmulationModelStore(),
        Dict{String, OptimizationContainerKey}(),
    )
end

function get_number_of_dimensions(
    store::InMemorySimulationStore,
    i::Type{EmulationModelIndexType},
    key::OptimizationContainerKey,
)
    return length(get_column_names(store, i, key))
end

function get_number_of_dimensions(
    store::InMemorySimulationStore,
    i::Type{DecisionModelIndexType},
    model_name::Symbol,
    key::OptimizationContainerKey,
)
    return length(get_column_names(store, i, model_name, key))
end

function open_store(
    func::Function,
    ::Type{InMemorySimulationStore},
    directory::AbstractString,  # Unused. Need to match the interface.
    mode = nothing,
    filename = nothing,
)
    store = InMemorySimulationStore()
    return func(store)
end

function Base.empty!(store::InMemorySimulationStore)
    for val in values(get_dm_data(store))
        empty!(val)
    end
    empty!(get_em_data(store))
    @debug "Emptied the store" _group = LOG_GROUP_SIMULATION_STORE
    return
end

Base.isopen(::InMemorySimulationStore) = true
Base.close(::InMemorySimulationStore) = nothing
Base.flush(::InMemorySimulationStore) = nothing
get_params(store::InMemorySimulationStore) = store.params

function get_decision_model_params(store::InMemorySimulationStore, model_name::Symbol)
    return get_params(store).decision_models_params[model_name]
end

get_container_key_lookup(store::InMemorySimulationStore) = store.container_key_lookup

list_decision_models(x::InMemorySimulationStore) = collect(keys(x.dm_data))
log_cache_hit_percentages(::InMemorySimulationStore) = nothing

function list_decision_model_keys(
    store::InMemorySimulationStore,
    model_name::Symbol,
    container_type::Symbol,
)
    return ISOPT.list_fields(
        _get_model_results(store, model_name),
        container_type,
    )
end

function list_emulation_model_keys(store::InMemorySimulationStore, container_type::Symbol)
    return ISOPT.list_fields(store.em_data, container_type)
end

function write_result!(
    store::InMemorySimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    array,
)
    write_result!(
        get_dm_data(store)[model_name],
        model_name,
        key,
        index,
        update_timestamp,
        array,
    )
    return
end

function write_result!(
    store::InMemorySimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::EmulationModelIndexType,
    update_timestamp::Dates.DateTime,
    array,
)
    write_result!(get_em_data(store), model_name, key, index, update_timestamp, array)
    return
end

function read_optimizer_stats(store::InMemorySimulationStore, model_name)
    # TODO EmulationModel: this interface is TBD
    return read_optimizer_stats(get_dm_data(store)[model_name])
end

function initialize_problem_storage!(
    store::InMemorySimulationStore,
    params::SimulationStoreParams,
    dm_problem_reqs::Dict{Symbol, SimulationModelStoreRequirements},
    em_problem_reqs::SimulationModelStoreRequirements,
    ::CacheFlushRules,
)
    store.params = params
    for problem in keys(store.params.decision_models_params)
        get_dm_data(store)[problem] = DecisionModelStore()
        for type in STORE_CONTAINERS
            for (key, _) in getfield(dm_problem_reqs[problem], type)
                container = getfield(get_dm_data(store)[problem], type)
                container[key] = OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}()
                store.container_key_lookup[encode_key_as_string(key)] = key
                @debug "Added $type $key in $problem" _group = LOG_GROUP_SIMULATION_STORE
            end
        end
    end

    for type in STORE_CONTAINERS
        for (key, reqs) in getfield(em_problem_reqs, type)
            container = get_data_field(get_em_data(store), type)
            container[key] = InMemoryDataset(
                fill!(
                    DenseAxisArray{Float64}(undef, reqs["columns"]..., 1:reqs["dims"][1]),
                    NaN,
                ),
            )
            store.container_key_lookup[encode_key_as_string(key)] = key
            @debug "Added $type $key in emulation store" _group = LOG_GROUP_SIMULATION_STORE
        end
    end

    return
end

function get_column_names(
    store::InMemorySimulationStore,
    ::Type{DecisionModelIndexType},
    model_name::Symbol,
    key::OptimizationContainerKey,
)
    return get_column_names(get_dm_data(store)[model_name], key)
end

function get_column_names(
    store::InMemorySimulationStore,
    ::Type{EmulationModelIndexType},
    model_name::Symbol,
    key::OptimizationContainerKey,
)
    return get_column_names(get_em_data(store)[model_name], key)
end

function read_result(
    ::Type{DenseAxisArray},
    store::InMemorySimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
)
    return read_results(get_dm_data(store)[model_name], key; index = index)
end

function read_result(
    ::Type{Array},
    store::InMemorySimulationStore,
    model_name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
)
    return permutedims(
        read_results(get_dm_data(store)[model_name], key; index = index).data,
    )
end

function read_result(
    ::Type{DenseAxisArray},
    store::InMemorySimulationStore,
    ::Symbol,
    key::OptimizationContainerKey,
    index::EmulationModelIndexType,
)
    return read_results(get_em_data(store), key; index = index)
end

function read_results(
    store::InMemorySimulationStore,
    key::OptimizationContainerKey;
    index::EmulationModelIndexType = nothing,
    len::Int = nothing,
)
    return read_results(get_em_data(store), key; index = index, len = len)
end

function get_emulation_model_dataset_size(
    store::InMemorySimulationStore,
    key::OptimizationContainerKey,
)
    return get_dataset_size(get_em_data(store), key)[2]
end

# Note that this function is not type-stable.
function _get_model_results(store::InMemorySimulationStore, model_name::Symbol)
    if model_name in keys(get_dm_data(store))
        results = get_dm_data(store)
    else
        # TODO EmulationModel: this interface is TBD
        error("model name $model_name is not stored")
    end

    return results[model_name]
end

function write_optimizer_stats!(
    store::InMemorySimulationStore,
    model::DecisionModel,
    index::DecisionModelIndexType,
)
    stats = get_optimizer_stats(model)
    dm_data = get_dm_data(store)
    write_optimizer_stats!(dm_data[get_name(model)], stats, index)
    read_optimizer_stats(dm_data[get_name(model)])
    return
end

function write_optimizer_stats!(
    store::InMemorySimulationStore,
    model::EmulationModel,
    index::EmulationModelIndexType,
)
    stats = get_optimizer_stats(model)
    em_data = get_em_data(store)
    write_optimizer_stats!(em_data, stats, index)
    return
end

serialize_system!(::InMemorySimulationStore, ::PSY.System) = nothing
write_system_json!(::InMemorySimulationStore, ::String, ::String) = nothing
