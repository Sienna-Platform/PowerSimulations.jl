"""
Stores results data for one DecisionModel
"""
mutable struct DecisionModelStore <: ISOPT.AbstractModelStore
    # All DenseAxisArrays have axes (column names, row indexes)
    duals::Dict{ConstraintKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}
    parameters::Dict{ParameterKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}
    variables::Dict{VariableKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}
    aux_variables::Dict{AuxVarKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}
    expressions::Dict{ExpressionKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}
    optimizer_stats::OrderedDict{Dates.DateTime, OptimizerStats}
end

function DecisionModelStore()
    return DecisionModelStore(
        Dict{ConstraintKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}(),
        Dict{ParameterKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}(),
        Dict{VariableKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}(),
        Dict{AuxVarKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}(),
        Dict{ExpressionKey, OrderedDict{Dates.DateTime, DenseAxisArray{Float64}}}(),
        OrderedDict{Dates.DateTime, OptimizerStats}(),
    )
end

function initialize_storage!(
    store::DecisionModelStore,
    container::ISOPT.AbstractOptimizationContainer,
    params::ModelStoreParams,
)
    num_of_executions = get_num_executions(params)
    if length(get_time_steps(container)) < 1
        error("The time step count in the optimization container is not defined")
    end
    time_steps_count = get_time_steps(container)[end]
    initial_time = get_initial_time(container)
    model_interval = get_interval(params)
    for type in STORE_CONTAINERS
        field_containers = getfield(container, type)
        results_container = getfield(store, type)
        for (key, field_container) in field_containers
            !should_write_resulting_value(key) && continue
            @debug "Adding $(encode_key_as_string(key)) to DecisionModelStore" _group =
                LOG_GROUP_MODEL_STORE
            column_names = get_column_names(container, type, field_container, key)
            data = OrderedDict{
                Dates.DateTime,
                DenseAxisArray{Float64, length(column_names) + 1},
            }()
            for timestamp in
                range(initial_time; step = model_interval, length = num_of_executions)
                data[timestamp] = fill!(
                    DenseAxisArray{Float64}(undef, column_names..., 1:time_steps_count),
                    NaN,
                )
            end
            results_container[key] = data
        end
    end
    return
end

function write_result!(
    store::DecisionModelStore,
    name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    array::DenseAxisArray{T, 2, <:Tuple{Vector{String}, UnitRange}},
) where {T}
    container = getfield(store, get_store_container_type(key))
    container[key][index] = array
    return
end

function write_result!(
    store::DecisionModelStore,
    name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    array::DenseAxisArray{T, 2, <:Tuple{Vector{Int}, UnitRange}},
) where {T}
    columns = get_column_names_from_axis_array(array)
    container = getfield(store, get_store_container_type(key))
    container[key][index] = DenseAxisArray(array.data, columns..., 1:size(array, 2))
    return
end

function write_result!(
    store::DecisionModelStore,
    name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    array::DenseAxisArray{T, 1, <:Tuple{Vector{String}}},
) where {T}
    container = getfield(store, get_store_container_type(key))
    container[key][index] = array
    return
end

function write_result!(
    store::DecisionModelStore,
    name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    array::DenseAxisArray{T, 1, <:Tuple{UnitRange}},
) where {T}
    container = getfield(store, get_store_container_type(key))
    container[key][index] = array
    return
end

function write_result!(
    store::DecisionModelStore,
    name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    array::DenseAxisArray{T, 3, <:Tuple{Vector{String}, Vector{String}, UnitRange{Int}}},
) where {T}
    container = getfield(store, get_store_container_type(key))
    container[key][index] = array
    return
end

# Sparse expressions (e.g., post-contingency flows keyed by
# `(outage_id, branch_name, t)`) are pre-allocated as 2D dense storage with
# the non-time tuple flattened into encoded `"a__b"` columns by
# `get_column_names_from_axis_array(::SparseAxisArray)`. `to_matrix` returns
# `(n_time, n_cols)`; transpose to match the `(cols, time)` layout the dense
# storage expects.
function write_result!(
    store::DecisionModelStore,
    name::Symbol,
    key::OptimizationContainerKey,
    index::DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    array::SparseAxisArray{T},
) where {T}
    columns = get_column_names_from_axis_array(array)[1]
    matrix = to_matrix(array)
    container = getfield(store, get_store_container_type(key))
    container[key][index] =
        DenseAxisArray(permutedims(matrix), columns, 1:size(matrix, 1))
    return
end

function read_results(
    store::DecisionModelStore,
    key::OptimizationContainerKey;
    index::Union{DecisionModelIndexType, Nothing} = nothing,
)
    container = getfield(store, get_store_container_type(key))
    data = container[key]
    if isnothing(index)
        @assert_op length(data) == 1
        index = first(keys(data))
    end

    # Return a copy because callers may mutate it.
    return deepcopy(data[index])
end

function write_optimizer_stats!(
    store::DecisionModelStore,
    stats::OptimizerStats,
    index::DecisionModelIndexType,
)
    # TODO: This check is incompatible with test calls to psi_checksolve_test
    # Overwriting should not be allowed in normal operation.
    # if index in keys(store.optimizer_stats)
    #     error("Bug: Overwriting optimizer stats for index = $index")
    # end
    store.optimizer_stats[index] = stats
    return
end

function read_optimizer_stats(store::DecisionModelStore)
    stats = [IS.to_namedtuple(x) for x in values(store.optimizer_stats)]
    df = DataFrames.DataFrame(stats)
    DataFrames.insertcols!(df, 1, :DateTime => keys(store.optimizer_stats))
    return df
end

function get_column_names(store::DecisionModelStore, key::OptimizationContainerKey)
    container = getfield(store, get_store_container_type(key))
    return get_column_names_from_axis_array(key, first(values(container[key])))
end
