struct DecisionModelSimulationOutputs <: OperationModelSimulationOutputs
    variables::OutputsByKeyAndTime
    duals::OutputsByKeyAndTime
    parameters::OutputsByKeyAndTime
    aux_variables::OutputsByKeyAndTime
    expressions::OutputsByKeyAndTime
    forecast_horizon::Int
    container_key_lookup::Dict{String, OptimizationContainerKey}
end

function SimulationProblemOutputs(
    ::Type{DecisionModel},
    store::SimulationStore,
    model_name::AbstractString,
    problem_params::ModelStoreParams,
    sim_params::SimulationStoreParams,
    path,
    container_key_lookup;
    kwargs...,
)
    name = Symbol(model_name)
    return SimulationProblemOutputs{DecisionModelSimulationOutputs}(
        store,
        model_name,
        problem_params,
        sim_params,
        path,
        DecisionModelSimulationOutputs(
            OutputsByKeyAndTime(
                list_decision_model_keys(store, name, STORE_CONTAINER_VARIABLES),
            ),
            OutputsByKeyAndTime(
                list_decision_model_keys(store, name, STORE_CONTAINER_DUALS),
            ),
            OutputsByKeyAndTime(
                list_decision_model_keys(store, name, STORE_CONTAINER_PARAMETERS),
            ),
            OutputsByKeyAndTime(
                list_decision_model_keys(store, name, STORE_CONTAINER_AUX_VARIABLES),
            ),
            OutputsByKeyAndTime(
                list_decision_model_keys(store, name, STORE_CONTAINER_EXPRESSIONS),
            ),
            get_horizon_count(problem_params),
            container_key_lookup,
        );
        kwargs...,
    )
end

function _list_containers(res::SimulationProblemOutputs{DecisionModelSimulationOutputs})
    return (getfield(res.values, x).cached_outputs for x in get_container_fields(res))
end

function Base.empty!(res::SimulationProblemOutputs{DecisionModelSimulationOutputs})
    foreach(empty!, _list_containers(res))
    empty!(get_outputs_timestamps(res))
end

function Base.isempty(res::SimulationProblemOutputs{DecisionModelSimulationOutputs})
    all(isempty, _list_containers(res))
end

# This returns the number of timestamps stored in all containers.
function Base.length(res::SimulationProblemOutputs{DecisionModelSimulationOutputs})
    return mapreduce(length, +, (y for x in _list_containers(res) for y in values(x)))
end

IOM.list_aux_variable_keys(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.aux_variables.output_keys[:]
IOM.list_dual_keys(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.duals.output_keys[:]
IOM.list_expression_keys(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.expressions.output_keys[:]
IOM.list_parameter_keys(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.parameters.output_keys[:]
IOM.list_variable_keys(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.variables.output_keys[:]

get_cached_aux_variables(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.aux_variables.cached_outputs
get_cached_duals(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.duals.cached_outputs
get_cached_expressions(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.expressions.cached_outputs
get_cached_parameters(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.parameters.cached_outputs
get_cached_variables(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}) =
    res.values.variables.cached_outputs

function IOM.get_forecast_horizon(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
)
    return res.values.forecast_horizon
end

function _get_store_value(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    container_keys::Vector{<:OptimizationContainerKey},
    timestamps,
    ::Nothing,
)
    return _open_outputs_store(get_execution_path(res)) do store
        _register_borrowed_stores!(store, res)
        _get_store_value(res, container_keys, timestamps, store)
    end
end

function _get_store_value(
    sim_outputs::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    container_keys::Vector{<:OptimizationContainerKey},
    timestamps::Vector{Dates.DateTime},
    store::SimulationStore,
)
    outputs_by_key = Dict{OptimizationContainerKey, OutputsByTime}()
    model_name = Symbol(get_model_name(sim_outputs))
    for ckey in container_keys
        n_dims = get_number_of_dimensions(store, DecisionModelIndexType, model_name, ckey)
        container_type = DenseAxisArray{Float64, n_dims + 1}
        outputs_by_key[ckey] = _get_store_value(container_type,
            sim_outputs,
            ckey,
            timestamps, store)
    end
    return outputs_by_key
end

function _get_store_value(
    ::Type{T},
    sim_outputs::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    key::OptimizationContainerKey,
    timestamps::Vector{Dates.DateTime},
    store::SimulationStore,
) where {N, T <: DenseAxisArray{Float64, N}}
    resolution = get_resolution(sim_outputs)
    horizon = get_forecast_horizon(sim_outputs)
    base_power = get_model_base_power(sim_outputs)
    model_name = Symbol(get_model_name(sim_outputs))
    outputs_by_time = OutputsByTime(
        key,
        SortedDict{Dates.DateTime, T}(),
        resolution,
        get_column_names(store, DecisionModelIndexType, model_name, key),
    )
    array_size::Union{Nothing, NTuple{N, Int}} = nothing
    for ts in timestamps
        array = read_output(DenseAxisArray, store, model_name, key, ts)
        if isnothing(array_size)
            array_size = size(array)
        elseif size(array) != array_size
            error(
                "Arrays for $(encode_key_as_string(key)) at different timestamps have different sizes",
            )
        end
        if convert_output_to_natural_units(key)
            array.data .*= base_power
        end
        # The last axis is time.
        if array_size[end] != horizon
            @warn "$(encode_key_as_string(key)) has a different horizon than the " *
                  "problem specification. Can't assign timestamps to the resulting DataFrame."
            outputs_by_time.resolution = Dates.Period(Dates.Millisecond(0))
        end
        outputs_by_time[ts] = array
    end

    return outputs_by_time
end

function IOM._process_timestamps(
    res::SimulationProblemOutputs,
    initial_time::Union{Nothing, Dates.DateTime},
    count::Union{Nothing, Int},
)
    if isnothing(initial_time)
        initial_time = first(get_timestamps(res))
    end

    if initial_time ∉ res.timestamps
        invalid_timestamps = [initial_time]
    else
        if isnothing(count)
            requested_range = [v for v in res.timestamps if v >= initial_time]
        else
            requested_range =
                collect(range(initial_time; length = count, step = get_interval(res)))
        end
        invalid_timestamps = [v for v in requested_range if v ∉ res.timestamps]
    end
    if !isempty(invalid_timestamps)
        @error "Timestamps $(invalid_timestamps) not stored" get_timestamps(res)
        throw(IS.InvalidValue("Timestamps not stored"))
    end
    return requested_range
end

function _read_outputs(
    ::Type{DataFrame},
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    output_keys,
    timestamps::Vector{Dates.DateTime},
    store::Union{Nothing, <:SimulationStore};
    cols::Union{Colon, Vector{String}} = (:),
    table_format::TableFormat.Value = TableFormat.LONG,
)
    vals = _read_outputs(res, output_keys, timestamps, store)
    converted_vals = Dict{OptimizationContainerKey, OutputsByTime{DataFrame}}()
    for (output_key, output_data) in vals
        inner_converted = SortedDict{Dates.DateTime, DataFrame}()
        for (date_key, inner_data) in output_data
            extra = ntuple(_ -> (:), ndims(inner_data) - 1)
            inner_converted[date_key] =
                to_outputs_dataframe(inner_data[cols, extra...], nothing, Val(table_format))
        end
        # `to_outputs_dataframe` reshapes the raw component axis into either a fixed
        # 3-column long table or a wide table with one column per component plus
        # `:DateTime`; `IOM.OutputsByTime` validates against the actual resulting
        # DataFrame's columns, not the pre-reshape component axis.
        _cols = (String.(names(first(values(inner_converted)))),)
        converted_vals[output_key] = OutputsByTime(
            output_data.key,
            inner_converted,
            output_data.resolution,
            _cols)
    end
    return converted_vals
end

function _read_outputs(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    output_keys,
    timestamps::Vector{Dates.DateTime},
    store::Union{Nothing, <:SimulationStore},
)
    isempty(output_keys) &&
        return Dict{OptimizationContainerKey, OutputsByTime{DenseAxisArray{Float64, 2}}}()

    _store = try_resolve_store(store, res.store)
    existing_keys = list_output_keys(res, first(output_keys))
    _validate_keys(existing_keys, output_keys)
    cached_outputs = get_cached_outputs(res, eltype(output_keys))
    if _are_outputs_cached(res, output_keys, timestamps, keys(cached_outputs))
        @debug "reading outputs from SimulationsOutputs cache"  # NOTE tests match on this
        vals = Dict(k => cached_outputs[k] for k in output_keys)
        # Cached data may contain more timestamps than we need, remove these if so
        (timestamps == get_outputs_timestamps(res)) && return vals
        filtered_vals = Dict{keytype(vals), valtype(vals)}()
        for (output_key, output_data) in vals
            inner_converted = filter((((k, v),) -> k in timestamps), output_data.data)
            filtered_vals[output_key] = OutputsByTime(
                output_data.key,
                inner_converted,
                output_data.resolution,
                output_data.column_names)
        end
        return filtered_vals
    else
        @debug "reading outputs from data store"  # NOTE tests match on this
        vals = _get_store_value(res, output_keys, timestamps, _store)
    end
    return vals
end

"""
Return the values for the requested variable. It keeps requests when performing multiple retrievals.

# Arguments

  - `args`: Can be a string returned from [`list_variable_names`](@ref) or args that can be
    splatted into a VariableKey.
  - `start_time::Dates.DateTime` : initial of the requested outputs
  - `len::Int`: Number of outputs
  - `store::SimulationStore`: a store that has been opened for reading
  - `table_format::TableFormat.Value`: Format of the table to be returned. Default is
    `TableFormat.LONG` where the columns are `DateTime`, `name`, and `value` when the data
    has two dimensions and `DateTime`, `name`, `name2`, and `value` when the data has three
    dimensions.
    Set to it `TableFormat.WIDE` to pivot the names as columns.
    Note: `TableFormat.WIDE` is not supported when the data has three dimensions.

# Examples

```julia
IOM.read_variable(outputs, ActivePowerVariable, ThermalStandard)
IOM.read_variable(outputs, "ActivePowerVariable__ThermalStandard")
IOM.read_variable(outputs, "ActivePowerVariable__ThermalStandard", table_format = TableFormat.WIDE)
```
"""
function IOM.read_variable(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    args...;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
    store = nothing,
    table_format::TableFormat.Value = TableFormat.LONG,
)
    key = _deserialize_key(VariableKey, res, args...)
    timestamps = _process_timestamps(res, start_time, len)
    return make_dataframes(
        _read_outputs(res, [key], timestamps, store)[key];
        table_format = table_format,
    )
end

"""
Return the values for the requested dual. It keeps requests when performing multiple retrievals.

# Arguments

  - `args`: Can be a string returned from [`list_dual_names`](@ref) or args that can be
    splatted into a ConstraintKey.
  - `start_time::Dates.DateTime` : initial of the requested outputs
  - `len::Int`: Number of outputs
  - `store::SimulationStore`: a store that has been opened for reading
  - `table_format::TableFormat.Value`: Format of the table to be returned. Default is
    `TableFormat.LONG` where the columns are `DateTime`, `name`, and `value` when the data
    has two dimensions and `DateTime`, `name`, `name2`, and `value` when the data has three
    dimensions.
    Set to it `TableFormat.WIDE` to pivot the names as columns.
    Note: `TableFormat.WIDE` is not supported when the data has three dimensions.
"""
function IOM.read_dual(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    args...;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
    store = nothing,
    table_format::TableFormat.Value = TableFormat.LONG,
)
    key = _deserialize_key(ConstraintKey, res, args...)
    timestamps = _process_timestamps(res, start_time, len)
    return make_dataframes(
        _read_outputs(res, [key], timestamps, store)[key];
        table_format = table_format,
    )
end

"""
Return the values for the requested parameter. It keeps requests when performing multiple retrievals.

# Arguments

  - `args`: Can be a string returned from [`list_parameter_names`](@ref) or args that can be
    splatted into a ParameterKey.
  - `start_time::Dates.DateTime` : initial of the requested outputs
  - `len::Int`: Number of outputs
  - `table_format::TableFormat.Value`: Format of the table to be returned. Default is
    `TableFormat.LONG` where the columns are `DateTime`, `name`, and `value` when the data
    has two dimensions and `DateTime`, `name`, `name2`, and `value` when the data has three
    dimensions.
    Set to it `TableFormat.WIDE` to pivot the names as columns.
    Note: `TableFormat.WIDE` is not supported when the data has three dimensions.
"""
function IOM.read_parameter(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    args...;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
    store = nothing,
    table_format::TableFormat.Value = TableFormat.LONG,
)
    key = _deserialize_key(ParameterKey, res, args...)
    timestamps = _process_timestamps(res, start_time, len)
    return make_dataframes(
        _read_outputs(res, [key], timestamps, store)[key];
        table_format = table_format,
    )
end

"""
Return the values for the requested auxillary variables. It keeps requests when performing multiple retrievals.

# Arguments

  - `args`: Can be a string returned from [`list_aux_variable_names`](@ref) or args that can be
    splatted into a AuxVarKey.
  - `start_time::Dates.DateTime` : initial of the requested outputs
  - `len::Int`: Number of outputs
"""
function IOM.read_aux_variable(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    args...;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
    store = nothing,
    table_format::TableFormat.Value = TableFormat.LONG,
)
    key = _deserialize_key(AuxVarKey, res, args...)
    timestamps = _process_timestamps(res, start_time, len)
    return make_dataframes(
        _read_outputs(res, [key], timestamps, store)[key];
        table_format = table_format,
    )
end

"""
Return the values for the requested auxillary variables. It keeps requests when performing multiple retrievals.

# Arguments

  - `args`: Can be a string returned from [`list_expression_names`](@ref) or args that can be
    splatted into a ExpressionKey.
  - `start_time::Dates.DateTime` : initial of the requested outputs
  - `len::Int`: Number of outputs
"""
function IOM.read_expression(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    args...;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
    store = nothing,
    table_format::TableFormat.Value = TableFormat.LONG,
)
    key = _deserialize_key(ExpressionKey, res, args...)
    timestamps = _process_timestamps(res, start_time, len)
    return make_dataframes(
        _read_outputs(res, [key], timestamps, store)[key];
        table_format = table_format,
    )
end

function IOM.get_realized_timestamps(
    res::SimulationProblemOutputs;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
)
    timestamps = get_timestamps(res)
    resolution = get_resolution(res)
    interval = get_interval(res)
    horizon = get_forecast_horizon(res)
    if isnothing(start_time)
        start_time = first(timestamps)
    end
    end_time =
        if isnothing(len)
            last(timestamps) + interval - resolution
        else
            start_time + (len - 1) * resolution
        end

    requested_range = start_time:resolution:end_time
    available_range =
        first(timestamps):resolution:(last(timestamps) + (horizon - 1) * resolution)
    invalid_timestamps = setdiff(requested_range, available_range)

    if !isempty(invalid_timestamps)
        msg = "Requested time does not match available outputs"
        @error msg
        throw(IS.InvalidValue(msg))
    end

    return requested_range
end

"""
High-level function to read a DataFrame of outputs.

# Arguments

  - `res`: the outputs to read.
  - `output_keys::Vector{<:OptimizationContainerKey}`: the keys to read. Output will be a
    `Dict{OptimizationContainerKey, DataFrame}` with these as the keys
  - `start_time::Union{Nothing, Dates.DateTime} = nothing`: the time at which the resulting
    time series should begin; `nothing` indicates the first time in the outputs
  - `len::Union{Int, Nothing} = nothing`: the number of steps in the resulting time series;
    `nothing` indicates up to the end of the outputs
  - `cols::Union{Colon, Vector{String}} = (:)`: which columns to fetch; defaults to `:`,
    i.e., all the columns
"""
function IOM.read_outputs_with_keys(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    output_keys::Vector{<:OptimizationContainerKey};
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
    cols::Union{Colon, Vector{String}} = (:),
    table_format = TableFormat.LONG,
)
    meta = RealizedMeta(res; start_time = start_time, len = len)
    timestamps = _process_timestamps(res, meta.start_time, meta.len)
    output_values =
        _read_outputs(
            DataFrame,
            res,
            output_keys,
            timestamps,
            nothing;
            cols = cols,
            table_format = table_format,
        )
    return get_realization(output_values, meta; table_format = table_format)
end

function _are_outputs_cached(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    output_keys::Vector{<:OptimizationContainerKey},
    timestamps::Vector{Dates.DateTime},
    cached_keys,
)
    return isempty(setdiff(timestamps, get_outputs_timestamps(res))) &&
           isempty(setdiff(output_keys, cached_keys))
end

"""
Load the simulation outputs into memory for repeated reads. This is useful when loading
outputs from remote locations over network connections, when reading the same data very many
times, etc. Multiple calls augment the cache according to these rules, where "variable"
means "variable, expression, etc.":
  - Requests for an already cached variable at a lesser `count` than already cached do *not*
    decrease the `count` of the cached variable
  - Requests for an already cached variable at a greater `count` than already cached *do*
    increase the `count` of the cached variable
  - Requests for new variables are fulfilled without evicting existing variables

Note that `count` is global across all variables, so increasing the `count` re-reads already
cached variables. For each variable, each element must be the name encoded as a string, like
`"ActivePowerVariable__ThermalStandard"` or a Tuple with its constituent types, like
`(ActivePowerVariable, ThermalStandard)`. To clear the cache, use [`Base.empty!`](@ref).

# Arguments

  - `count::Int`: Number of windows to load.
  - `initial_time::Dates.DateTime` : Initial time of first window to load. Defaults to
    first.
  - `aux_variables::Vector{Union{String, Tuple}}`: Optional list of aux variables to load.
  - `duals::Vector{Union{String, Tuple}}`: Optional list of duals to load.
  - `expressions::Vector{Union{String, Tuple}}`: Optional list of expressions to load.
  - `parameters::Vector{Union{String, Tuple}}`: Optional list of parameters to load.
  - `variables::Vector{Union{String, Tuple}}`: Optional list of variables to load.
"""
function load_outputs!(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    count::Int;
    initial_time::Union{Dates.DateTime, Nothing} = nothing,
    variables = Vector{Tuple}(),
    duals = Vector{Tuple}(),
    parameters = Vector{Tuple}(),
    aux_variables = Vector{Tuple}(),
    expressions = Vector{Tuple}(),
    store::Union{Nothing, <:SimulationStore} = nothing,
)
    if isnothing(initial_time)
        initial_time = first(get_timestamps(res))
    end
    count = max(count, length(get_outputs_timestamps(res)))
    new_timestamps = _process_timestamps(res, initial_time, count)

    for (key_type, new_items) in [
        (ConstraintKey, duals),
        (ParameterKey, parameters),
        (VariableKey, variables),
        (AuxVarKey, aux_variables),
        (ExpressionKey, expressions),
    ]
        new_keys = key_type[_deserialize_key(key_type, res, x...) for x in new_items]
        existing_outputs = get_cached_outputs(res, key_type)
        total_keys = union(collect(keys(existing_outputs)), new_keys)
        # _read_outputs checks the cache to eliminate unnecessary re-reads
        merge!(existing_outputs, _read_outputs(res, total_keys, new_timestamps, store))
    end
    set_outputs_timestamps!(res, new_timestamps)

    return
end

function _read_optimizer_stats(
    res::SimulationProblemOutputs{DecisionModelSimulationOutputs},
    store::SimulationStore,
)
    return read_optimizer_stats(store, Symbol(res.problem))
end
