struct EmulationModelSimulationOutputs <: OperationModelSimulationOutputs
    variables::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    duals::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    parameters::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    aux_variables::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    expressions::Dict{OptimizationContainerKey, DataFrames.DataFrame}
    container_key_lookup::Dict{String, OptimizationContainerKey}
end

function SimulationProblemOutputs(
    ::Type{EmulationModel},
    store::SimulationStore,
    model_name::AbstractString,
    problem_params::ModelStoreParams,
    sim_params::SimulationStoreParams,
    path,
    container_key_lookup;
    kwargs...,
)
    return SimulationProblemOutputs{EmulationModelSimulationOutputs}(
        store,
        model_name,
        problem_params,
        sim_params,
        path,
        EmulationModelSimulationOutputs(
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

IOM.list_aux_variable_keys(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    collect(keys(res.values.aux_variables))
IOM.list_dual_keys(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    collect(keys(res.values.duals))
IOM.list_expression_keys(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    collect(keys(res.values.expressions))
IOM.list_parameter_keys(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    collect(keys(res.values.parameters))
IOM.list_variable_keys(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    collect(keys(res.values.variables))

get_cached_aux_variables(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    res.values.aux_variables
get_cached_duals(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    res.values.duals
get_cached_expressions(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    res.values.expressions
get_cached_parameters(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    res.values.parameters
get_cached_variables(res::SimulationProblemOutputs{EmulationModelSimulationOutputs}) =
    res.values.variables

function _list_containers(res::SimulationProblemOutputs)
    return (getfield(res.values, x) for x in get_container_fields(res))
end

function Base.empty!(res::SimulationProblemOutputs{EmulationModelSimulationOutputs})
    for container in _list_containers(res)
        for df in values(container)
            empty!(df)
        end
    end
end

function Base.isempty(res::SimulationProblemOutputs{EmulationModelSimulationOutputs})
    for container in _list_containers(res)
        for df in values(container)
            if !isempty(df)
                return false
            end
        end
    end

    return true
end

function Base.length(res::SimulationProblemOutputs{EmulationModelSimulationOutputs})
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
    res::SimulationProblemOutputs{EmulationModelSimulationOutputs},
    container_keys::Vector{<:OptimizationContainerKey},
    ::Nothing;
    start_time = nothing,
    len = nothing,
    table_format = TableFormat.LONG,
)
    return _open_outputs_store(get_execution_path(res)) do store
        _register_borrowed_stores!(store, res)
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
    res::SimulationProblemOutputs{EmulationModelSimulationOutputs},
    container_keys::Vector{<:OptimizationContainerKey},
    store::SimulationStore;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Nothing, Int} = nothing,
    table_format = TableFormat.LONG,
)
    base_power = res.base_power
    outputs = Dict{OptimizationContainerKey, DataFrames.DataFrame}()
    for key in container_keys
        start_time, _len, resolution = _check_offsets(res, key, store, start_time, len)
        start_index = (start_time - first(res.timestamps)) ÷ resolution + 1
        array = read_outputs(store, key; index = start_index, len = _len)
        if convert_output_to_natural_units(key)
            array.data .*= base_power
        end
        # PERF: this is a double-permutedims with HDF
        # We could make an optimized version of this that reads Arrays
        # like decision_model_simulation_outputs
        timestamps = range(start_time; length = _len, step = res.resolution)
        outputs[key] = to_outputs_dataframe(array, timestamps, Val(table_format))
    end

    return outputs
end

function _check_offsets(
    res::SimulationProblemOutputs{EmulationModelSimulationOutputs},
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
                "start_time = $start_time is not in the outputs range $(res.timestamps)",
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
                "len = $len resolution = $resolution exceeds the outputs range $(res.timestamps)",
            ),
        )
    end

    return start_time, len, resolution
end

function _read_outputs(
    res::SimulationProblemOutputs{EmulationModelSimulationOutputs},
    output_keys,
    store;
    start_time = nothing,
    len = nothing,
    table_format = TableFormat.LONG,
)
    isempty(output_keys) && return Dict{OptimizationContainerKey, DataFrames.DataFrame}()
    _store = try_resolve_store(store, res.store)
    existing_keys = list_output_keys(res, first(output_keys))
    # _validate_keys is unexported; mirrors the call in
    # IOM.optimization_problem_outputs.jl's `_read_outputs`.
    _validate_keys(existing_keys, output_keys)
    cached_outputs = Dict(
        k => v for
        (k, v) in get_cached_outputs(res, eltype(output_keys)) if !isempty(v)
    )
    if isempty(setdiff(output_keys, keys(cached_outputs)))
        @debug "reading aux_variables from SimulationsOutputs"
        vals = Dict(k => cached_outputs[k] for k in output_keys)
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
                output_keys,
                _store;
                start_time = start_time,
                len = len,
                table_format = table_format,
            )
    end
    return vals
end

function IOM.read_outputs_with_keys(
    res::SimulationProblemOutputs{EmulationModelSimulationOutputs},
    output_keys::Vector{<:OptimizationContainerKey};
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Nothing, Int} = nothing,
    table_format = TableFormat.LONG,
)
    return _read_outputs(
        res,
        output_keys,
        nothing;
        start_time = start_time,
        len = len,
        table_format = table_format,
    )
end

"""
Load the simulation outputs into memory for repeated reads. This is useful when loading
outputs from remote locations over network connections.

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
_with_outputs_store(f, store::InMemorySimulationStore, ::AbstractString) = f(store)
_with_outputs_store(f, ::Nothing, execution_path::AbstractString) =
    _open_outputs_store(f, execution_path)

function load_outputs!(
    res::SimulationProblemOutputs{EmulationModelSimulationOutputs};
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
    function merge_outputs(store)
        _register_borrowed_stores!(store, res)
        merge!(get_cached_aux_variables(res), _read_outputs(res, aux_variable_keys, store))
        merge!(get_cached_duals(res), _read_outputs(res, dual_keys, store))
        merge!(get_cached_expressions(res), _read_outputs(res, expression_keys, store))
        merge!(get_cached_parameters(res), _read_outputs(res, parameter_keys, store))
        merge!(get_cached_variables(res), _read_outputs(res, variable_keys, store))
    end

    _with_outputs_store(merge_outputs, res.store, res.execution_path)

    return
end

# TODO: These aren't being written to the store.
function _read_optimizer_stats(
    res::SimulationProblemOutputs{EmulationModelSimulationOutputs},
    store::SimulationStore,
)
    return
end
