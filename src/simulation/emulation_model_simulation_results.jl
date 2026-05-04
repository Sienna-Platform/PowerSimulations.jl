struct EmulationModelSimulationResults <: OperationModelSimulationResults
    variables::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    duals::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    parameters::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    aux_variables::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    expressions::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    container_key_lookup::Dict{String, OptimizationContainerKey}
end

function SimulationProblemResults(
    ::Type{EmulationModel},
    store::SimulationStore,
    model_name::AbstractString,
    problem_params::ModelStoreParams,
    sim_params::SimulationStoreParams,
    path,
    container_key_lookup;
    kwargs...,
)
    return SimulationProblemResults{EmulationModelSimulationResults}(
        store,
        model_name,
        problem_params,
        sim_params,
        path,
        EmulationModelSimulationResults(
            Dict(
                x => DataFrames.DataFrame() for
                x in list_emulation_model_keys(store, STORE_CONTAINER_VARIABLES)
            ),
            Dict(
                x => DataFrames.DataFrame() for
                x in list_emulation_model_keys(store, STORE_CONTAINER_DUALS)
            ),
            Dict(
                x => DataFrames.DataFrame() for
                x in list_emulation_model_keys(store, STORE_CONTAINER_PARAMETERS)
            ),
            Dict(
                x => DataFrames.DataFrame() for
                x in list_emulation_model_keys(store, STORE_CONTAINER_AUX_VARIABLES)
            ),
            Dict(
                x => DataFrames.DataFrame() for
                x in list_emulation_model_keys(store, STORE_CONTAINER_EXPRESSIONS)
            ),
            container_key_lookup,
        );
        kwargs...,
    )
end

list_aux_variable_keys(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    collect(keys(res.values.aux_variables))
list_dual_keys(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    collect(keys(res.values.duals))
list_expression_keys(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    collect(keys(res.values.expressions))
list_parameter_keys(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    collect(keys(res.values.parameters))
list_variable_keys(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    collect(keys(res.values.variables))

get_cached_aux_variables(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    res.values.aux_variables
get_cached_duals(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    res.values.duals
get_cached_expressions(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    res.values.expressions
get_cached_parameters(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    res.values.parameters
get_cached_variables(res::SimulationProblemResults{EmulationModelSimulationResults}) =
    res.values.variables

function _list_containers(res::SimulationProblemResults)
    return (getfield(res.values, x) for x in get_container_fields(res))
end

function Base.empty!(res::SimulationProblemResults{EmulationModelSimulationResults})
    for container in _list_containers(res)
        for df in values(container)
            empty!(df)
        end
    end
end

function Base.isempty(res::SimulationProblemResults{EmulationModelSimulationResults})
    for container in _list_containers(res)
        for df in values(container)
            if !isempty(df)
                return false
            end
        end
    end

    return true
end

function Base.length(res::SimulationProblemResults{EmulationModelSimulationResults})
    count_not_empty = 0
    for container in _list_containers(res)
        for df in values(container)
            if !isempty(df)
                count_not_empty += 1
            end
        end
    end

    return count_not_empty
end

function _get_store_value(
    res::SimulationProblemResults{EmulationModelSimulationResults},
    container_keys::Vector{<:OptimizationContainerKey},
    ::Nothing;
    start_time = nothing,
    len = nothing,
    table_format = TableFormat.LONG,
)
    simulation_store_path = joinpath(get_execution_path(res), "data_store")
    return open_store(HdfSimulationStore, simulation_store_path, "r") do store
        _get_store_value(
            res,
            container_keys,
            store;
            start_time = start_time,
            len = len,
            table_format = table_format,
        )
    end
end

function _get_store_value(
    res::SimulationProblemResults{EmulationModelSimulationResults},
    container_keys::Vector{<:OptimizationContainerKey},
    store::SimulationStore;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Nothing, Int} = nothing,
    table_format = TableFormat.LONG,
)
    base_power = res.base_power
    results = Dict{OptimizationContainerKey, DataFrames.DataFrame}()
    for key in container_keys
        start_time, _len, resolution = _check_offsets(res, key, store, start_time, len)
        start_index = (start_time - first(res.timestamps)) ÷ resolution + 1
        array = read_results(store, key; index = start_index, len = _len)
        if convert_result_to_natural_units(key)
            array.data .*= base_power
        end
        # PERF: this is a double-permutedims with HDF
        # We could make an optimized version of this that reads Arrays
        # like decision_model_simulation_results
        timestamps = range(start_time; length = _len, step = res.resolution)
        results[key] = to_results_dataframe(array, timestamps, Val(table_format))
    end

    return results
end

function _check_offsets(
    res::SimulationProblemResults{EmulationModelSimulationResults},
    key,
    store,
    start_time,
    len,
)
    dataset_size = get_emulation_model_dataset_size(store, key)
    resolution =
        (last(res.timestamps) - first(res.timestamps) + res.resolution) ÷ dataset_size
    if isnothing(start_time)
        start_time = first(res.timestamps)
    elseif start_time < first(res.timestamps) || start_time > last(res.timestamps)
        throw(
            IS.InvalidValue(
                "start_time = $start_time is not in the results range $(res.timestamps)",
            ),
        )
    elseif (start_time - first(res.timestamps)) % resolution != Dates.Millisecond(0)
        throw(
            IS.InvalidValue(
                "start_time = $start_time is not a multiple of resolution = $resolution",
            ),
        )
    end

    if isnothing(len)
        len = (last(res.timestamps) + resolution - start_time) ÷ resolution
    elseif start_time + resolution * len > last(res.timestamps) + res.resolution
        throw(
            IS.InvalidValue(
                "len = $len resolution = $resolution exceeds the results range $(res.timestamps)",
            ),
        )
    end

    return start_time, len, resolution
end

function _read_results(
    res::SimulationProblemResults{EmulationModelSimulationResults},
    result_keys,
    store;
    start_time = nothing,
    len = nothing,
    table_format = TableFormat.LONG,
)
    isempty(result_keys) && return Dict{OptimizationContainerKey, DataFrames.DataFrame}()
    _store = try_resolve_store(store, res.store)
    existing_keys = list_result_keys(res, first(result_keys))
    ISOPT._validate_keys(existing_keys, result_keys)
    cached_results = Dict(
        k => v for
        (k, v) in get_cached_results(res, eltype(result_keys)) if !isempty(v)
    )
    if isempty(setdiff(result_keys, keys(cached_results)))
        @debug "reading aux_variables from SimulationsResults"
        vals = Dict(k => cached_results[k] for k in result_keys)
        if table_format == TableFormat.WIDE
            for (k, v) in vals
                if :name2 in DataFrames.propertynames(v)
                    error(
                        "TableFormat.WIDE is not supported when the data has three dimensions.",
                    )
                end
            end
            vals = Dict(
                k => DataFrames.unstack(v, :DateTime, :name, :value) for (k, v) in vals
            )
        end
    else
        @debug "reading aux_variables from data store"
        vals =
            _get_store_value(
                res,
                result_keys,
                _store;
                start_time = start_time,
                len = len,
                table_format = table_format,
            )
    end
    return vals
end

function read_results_with_keys(
    res::SimulationProblemResults{EmulationModelSimulationResults},
    result_keys::Vector{<:OptimizationContainerKey};
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Nothing, Int} = nothing,
    table_format = TableFormat.LONG,
)
    return _read_results(
        res,
        result_keys,
        nothing;
        start_time = start_time,
        len = len,
        table_format = table_format,
    )
end

"""
Load the simulation results into memory for repeated reads. This is useful when loading
results from remote locations over network connections.

For each variable/parameter/dual, etc., each element must be the name encoded as a string,
like `"ActivePowerVariable__ThermalStandard"`` or a Tuple with its constituent types, like
`(ActivePowerVariable, ThermalStandard)`.

# Arguments

  - `aux_variables::Vector{Union{String, Tuple}}`: Optional list of aux variables to load.
  - `duals::Vector{Union{String, Tuple}}`: Optional list of duals to load.
  - `expressions::Vector{Union{String, Tuple}}`: Optional list of expressions to load.
  - `parameters::Vector{Union{String, Tuple}}`: Optional list of parameters to load.
  - `variables::Vector{Union{String, Tuple}}`: Optional list of variables to load.
"""
function load_results!(
    res::SimulationProblemResults{EmulationModelSimulationResults};
    aux_variables = Vector{Tuple}(),
    duals = Vector{Tuple}(),
    expressions = Vector{Tuple}(),
    parameters = Vector{Tuple}(),
    variables = Vector{Tuple}(),
)
    # TODO: consider extending this to support start_time and len
    aux_variable_keys = [_deserialize_key(AuxVarKey, res, x...) for x in aux_variables]
    dual_keys = [_deserialize_key(ConstraintKey, res, x...) for x in duals]
    expression_keys = [_deserialize_key(ExpressionKey, res, x...) for x in expressions]
    parameter_keys = [_deserialize_key(ParameterKey, res, x...) for x in parameters]
    variable_keys = [_deserialize_key(VariableKey, res, x...) for x in variables]
    function merge_results(store)
        merge!(get_cached_aux_variables(res), _read_results(res, aux_variable_keys, store))
        merge!(get_cached_duals(res), _read_results(res, dual_keys, store))
        merge!(get_cached_expressions(res), _read_results(res, expression_keys, store))
        merge!(get_cached_parameters(res), _read_results(res, parameter_keys, store))
        merge!(get_cached_variables(res), _read_results(res, variable_keys, store))
    end

    if res.store isa InMemorySimulationStore
        merge_results(res.store)
    else
        simulation_store_path = joinpath(res.execution_path, "data_store")
        open_store(HdfSimulationStore, simulation_store_path, "r") do store
            merge_results(store)
        end
    end

    return
end

# TODO: These aren't being written to the store.
function _read_optimizer_stats(
    res::SimulationProblemResults{EmulationModelSimulationResults},
    store::SimulationStore,
)
    return
end
