abstract type OperationModelSimulationResults end
# Subtypes need to implement the following methods for SimulationProblemResults{T}
# - read_results_with_keys
# - list_aux_variable_keys
# - list_dual_keys
# - list_expression_keys
# - list_parameter_keys
# - list_variable_keys
# - load_results!

"""
Holds the results of a simulation problem for plotting or exporting.
"""
mutable struct SimulationProblemResults{T} <:
               IS.Outputs where {T <: OperationModelSimulationResults}
    problem::String
    base_power::Float64
    execution_path::String
    results_output_folder::String
    timestamps::StepRange{Dates.DateTime, Dates.Millisecond}
    results_timestamps::Vector{Dates.DateTime}
    values::T
    system::Union{Nothing, PSY.System}
    system_uuid::Base.UUID
    resolution::Dates.TimePeriod
    store::Union{Nothing, SimulationStore}
end

function SimulationProblemResults{T}(
    store::SimulationStore,
    model_name::AbstractString,
    problem_params::ModelStoreParams,
    sim_params::SimulationStoreParams,
    path,
    vals::T;
    results_output_path = nothing,
    system = nothing,
) where {T <: OperationModelSimulationResults}
    if isnothing(results_output_path)
        results_output_path = joinpath(path, "results")
    end

    time_steps = range(
        sim_params.initial_time;
        length = problem_params.num_executions * sim_params.num_steps,
        step = problem_params.interval,
    )
    return SimulationProblemResults{T}(
        model_name,
        problem_params.base_power,
        path,
        results_output_path,
        time_steps,
        Vector{Dates.DateTime}(),
        vals,
        system,
        problem_params.system_uuid,
        IOM.get_resolution(problem_params),
        store isa HdfSimulationStore ? nothing : store,
    )
end

get_model_name(res::SimulationProblemResults) = res.problem
IOM.get_system(res::SimulationProblemResults) = res.system
IS.get_source_data(res::SimulationProblemResults) = get_system(res)  # Needed for compatibility with the IS.Outputs interface
IOM.get_resolution(res::SimulationProblemResults) = res.resolution
get_execution_path(res::SimulationProblemResults) = res.execution_path
IOM.get_model_base_power(res::SimulationProblemResults) = res.base_power
get_system_uuid(results::PSI.SimulationProblemResults) = results.system_uuid
IS.get_timestamp(result::SimulationProblemResults) = result.results_timestamps
IOM.get_interval(res::SimulationProblemResults) = res.timestamps.step
IOM.get_base_power(result::SimulationProblemResults) = result.base_power
get_output_dir(res::SimulationProblemResults) = res.results_output_folder

# IOM.export_realized_outputs(::IS.Outputs) calls IOM.read_outputs_with_keys, which IOM only
# defines for its own OptimizationProblemOutputs. Bridge to PSI's equivalent, already-tested
# read_results_with_keys so SimulationProblemResults satisfies the same export path.
function IOM.read_outputs_with_keys(
    res::SimulationProblemResults,
    result_keys::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    return read_results_with_keys(res, result_keys; kwargs...)
end

get_results_timestamps(result::SimulationProblemResults) = result.results_timestamps
function set_results_timestamps!(
    result::SimulationProblemResults,
    results_timestamps::Vector{Dates.DateTime},
)
    result.results_timestamps = results_timestamps
end

list_result_keys(res::SimulationProblemResults, ::AuxVarKey) =
    list_aux_variable_keys(res)
list_result_keys(res::SimulationProblemResults, ::ConstraintKey) =
    list_dual_keys(res)
list_result_keys(res::SimulationProblemResults, ::ExpressionKey) =
    list_expression_keys(res)
list_result_keys(res::SimulationProblemResults, ::ParameterKey) =
    list_parameter_keys(res)
list_result_keys(res::SimulationProblemResults, ::VariableKey) =
    list_variable_keys(res)

get_cached_results(res::SimulationProblemResults, ::Type{<:AuxVarKey}) =
    get_cached_aux_variables(res)
get_cached_results(res::SimulationProblemResults, ::Type{<:ConstraintKey}) =
    get_cached_duals(res)
get_cached_results(res::SimulationProblemResults, ::Type{<:ExpressionKey}) =
    get_cached_expressions(res)
get_cached_results(res::SimulationProblemResults, ::Type{<:ParameterKey}) =
    get_cached_parameters(res)
get_cached_results(res::SimulationProblemResults, ::Type{<:VariableKey}) =
    get_cached_variables(res)
get_cached_results(
    res::SimulationProblemResults,
    ::Type{<:OptimizationContainerKey} = OptimizationContainerKey,
) =
    merge(  # PERF: could be done lazily
        get_cached_aux_variables(res),
        get_cached_duals(res),
        get_cached_expressions(res),
        get_cached_parameters(res),
        get_cached_variables(res),
    )

"""
Return an array of variable names (strings) that are available for reads.
"""
IOM.list_variable_names(res::SimulationProblemResults) =
    encode_keys_as_strings(list_variable_keys(res))

"""
Return an array of dual names (strings) that are available for reads.
"""
IOM.list_dual_names(res::SimulationProblemResults) =
    encode_keys_as_strings(list_dual_keys(res))

"""
Return an array of parmater names (strings) that are available for reads.
"""
IOM.list_parameter_names(res::SimulationProblemResults) =
    encode_keys_as_strings(list_parameter_keys(res))

"""
Return an array of auxillary variable names (strings) that are available for reads.
"""
IOM.list_aux_variable_names(res::SimulationProblemResults) =
    encode_keys_as_strings(list_aux_variable_keys(res))

"""
Return an array of expression names (strings) that are available for reads.
"""
IOM.list_expression_names(res::SimulationProblemResults) =
    encode_keys_as_strings(list_expression_keys(res))

"""
Return a reference to a StepRange of available timestamps.
"""
IOM.get_timestamps(result::SimulationProblemResults) = result.timestamps

"""
Return the system used for the problem. If the system hasn't already been deserialized or
set with [`set_system!`](@ref) then deserialize and store it.

If the simulation was configured to serialize all systems to file then the returned system
will include all data. If that was not configured then the returned system will include
all data except time series data.
"""
function get_system!(
    results::Union{IOM.OptimizationProblemOutputs, SimulationProblemResults};
    kwargs...,
)
    !isnothing(get_system(results)) && return get_system(results)

    file = locate_system_file(results)
    # This flag should remain unpublished because it should never be needed
    # by the general audience.
    if !get(kwargs, :use_system_fallback, false) && isfile(file)
        system = PSY.System(file; time_series_read_only = true)
        @info "De-serialized the system from files."
    else
        system = get_system_fallback(results)
    end

    set_system!(results, system)
    return get_system(results)
end

get_system_fallback(results::SimulationProblemResults) =
    _deserialize_system(results, results.store)
get_system_fallback(results::IOM.OptimizationProblemOutputs) =
    error("Could not locate system")

locate_system_file(results::SimulationProblemResults) = joinpath(
    get_execution_path(results),
    "problems",
    get_model_name(results),
    make_system_filename(results.system_uuid),
)

locate_system_file(results::IOM.OptimizationProblemOutputs) = joinpath(
    get_output_dir(results),
    make_system_filename(get_source_data_uuid(results)),
)

IOM.get_system(results::IOM.OptimizationProblemOutputs) = get_source_data(results)

set_system!(results::IOM.OptimizationProblemOutputs, system) =
    set_source_data!(results, system)

function _deserialize_system(results::SimulationProblemResults, ::Nothing)
    open_store(
        HdfSimulationStore,
        joinpath(get_execution_path(results), "data_store"),
        "r",
    ) do store
        system = deserialize_system(store, results.system_uuid)
        @info "De-serialized the system from the simulation store. The system does " *
              "not include time series data."
        return system
    end
end

function _deserialize_system(::SimulationProblemResults, ::InMemorySimulationStore)
    # This should never be necessary because the system is guaranteed to be in memory.
    error("Deserializing a system from the InMemorySimulationStore is not supported.")
end

"""
Set the system in the results instance.

Throws InvalidValue if the system UUID is incorrect.

# Arguments

  - `results::SimulationProblemResults`: Results object
  - `system::AbstractString`: Path to the system json file

# Examples

```julia
julia > set_system!(res, "my_path/system_data.json")
```
"""
function set_system!(results::SimulationProblemResults, system::AbstractString)
    set_system!(results, System(system))
end

function set_system!(results::SimulationProblemResults, system::PSY.System)
    sys_uuid = PSY.get_system_uuid(system)
    if sys_uuid != results.system_uuid
        throw(
            IS.InvalidValue(
                "System mismatch. $sys_uuid does not match the stored value of $(results.system_uuid)",
            ),
        )
    end

    results.system = system
    return
end

function IOM._deserialize_key(
    ::Type{<:OptimizationContainerKey},
    results::SimulationProblemResults,
    name::AbstractString,
)
    !haskey(results.values.container_key_lookup, name) && error("$name is not stored")
    return results.values.container_key_lookup[name]
end

function IOM._deserialize_key(
    ::Type{T},
    results::SimulationProblemResults,
    args...,
) where {T <: OptimizationContainerKey}
    return make_key(T, args...)
end

get_container_fields(x::SimulationProblemResults) =
    (:aux_variables, :duals, :expressions, :parameters, :variables)

"""
Return the final values for the requested variables for each time step for a problem.

Decision problem results are returned in a Dict{String, Dict{DateTime, DataFrame}}.

Emulation problem results are returned in a Dict{String, DataFrame}.

Limit the data sizes returned by specifying `initial_time` and `count` for decision problems
or `start_time` and `len` for emulation problems.

If the Julia process is started with multiple threads, the code will read the variables in
parallel.

See also [`load_results!`](@ref) to preload data into memory.

# Arguments

  - `variables::Vector{Union{String, Tuple}}`: Variable name as a string or a Tuple with
    variable type and device type. If not provided then return all variables.
  - `initial_time::Dates.DateTime`: Initial time of the requested results. Decision problems
    only.
  - `count::Int`: Number of results. Decision problems only.
  - `start_time::Dates.DateTime`: Start time of the requested results. Emulation problems
    only.
  - `len::Int`: Number of rows in each DataFrame. Emulation problems only.
  - `table_format::TableFormat`: Format of the table to be returned. Default is
    `TableFormat.LONG` where the columns are `DateTime`, `name`, and `value` when the data
    has two dimensions and `DateTime`, `name`, `name2`, and `value` when the data has three
    dimensions.
    Set to it `TableFormat.WIDE` to pivot the names as columns, matching earlier versions
    of PowerSimulations.jl.
    Note: `TableFormat.WIDE` is not supported when the data has three dimensions.

# Examples

```julia
julia> variables_as_strings =
    ["ActivePowerVariable__ThermalStandard", "ActivePowerVariable__RenewableDispatch"]
julia> variables_as_types =
    [(ActivePowerVariable, ThermalStandard), (ActivePowerVariable, RenewableDispatch)]
julia> df_long =read_realized_variables(results, variables_as_strings)
julia> df_long = read_realized_variables(results, variables_as_types)
julia> df_wide = read_realized_variables(results, variables_as_types, table_format = TableFormat.WIDE)
julia> using DataFramesMeta
julia> df_agg_generators = @chain df_long begin
    @groupby(:DateTime)
    @combine(:value = sum(:value))
end
```
"""
function read_realized_variables(res::SimulationProblemResults; kwargs...)
    return read_realized_variables(res, list_variable_keys(res); kwargs...)
end

function read_realized_variables(
    res::SimulationProblemResults,
    variables::Vector{Tuple{DataType, DataType}};
    kwargs...,
)
    return read_realized_variables(
        res,
        [VariableKey(x...) for x in variables];
        kwargs...,
    )
end

function read_realized_variables(
    res::SimulationProblemResults,
    variables::Vector{<:AbstractString};
    kwargs...,
)
    return read_realized_variables(
        res,
        [_deserialize_key(VariableKey, res, x) for x in variables];
        kwargs...,
    )
end

function read_realized_variables(
    res::SimulationProblemResults,
    variables::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    result_values = read_results_with_keys(res, variables; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in result_values)
end

"""
Return the final values for the requested variable for each time step for a problem.

Decision problem results are returned in a Dict{DateTime, DataFrame}.

Emulation problem results are returned in a DataFrame.

Limit the data sizes returned by specifying `initial_time` and `count` for decision problems
or `start_time` and `len` for emulation problems.

See also [`load_results!`](@ref) to preload data into memory.

# Arguments

  - `variable::Union{String, Tuple}`: Variable name as a string or a Tuple with
    variable type and device type.
  - `initial_time::Dates.DateTime`: Initial time of the requested results. Decision problems
    only.
  - `count::Int`: Number of results. Decision problems only.
  - `start_time::Dates.DateTime`: Start time of the requested results. Emulation problems
    only.
  - `len::Int`: Number of rows in each DataFrame. Emulation problems only.
  - `table_format::TableFormat`: Format of the table to be returned. Default is
    `TableFormat.LONG` where the columns are `DateTime`, `name`, and `value` when the data
    has two dimensions and `DateTime`, `name`, `name2`, and `value` when the data has three
    dimensions.
    Set to it `TableFormat.WIDE` to pivot the names as columns.
    Note: `TableFormat.WIDE` is not supported when the data has three dimensions.

# Examples

```julia
julia > read_realized_variable(results, "ActivePowerVariable__ThermalStandard")
julia > read_realized_variable(results, (ActivePowerVariable, ThermalStandard))
julia > read_realized_variable(results, (ActivePowerVariable, ThermalStandard), table_format = TableFormat.WIDE)
```
"""
function read_realized_variable(
    res::SimulationProblemResults,
    variable::AbstractString;
    kwargs...,
)
    return first(
        values(
            read_realized_variables(
                res,
                [_deserialize_key(VariableKey, res, variable)];
                kwargs...,
            ),
        ),
    )
end

function read_realized_variable(res::SimulationProblemResults, variable...; kwargs...)
    return first(
        values(read_realized_variables(res, [VariableKey(variable...)]; kwargs...)),
    )
end

"""
Return the final values for the requested auxiliary variables for each time step for a problem.

Refer to [`read_realized_aux_variables`](@ref) for help and examples.
"""
function read_realized_aux_variables(res::SimulationProblemResults; kwargs...)
    return read_realized_aux_variables(
        res,
        list_aux_variable_keys(res);
        kwargs...,
    )
end

function read_realized_aux_variables(
    res::SimulationProblemResults,
    aux_variables::Vector{Tuple{DataType, DataType}};
    kwargs...,
)
    return read_realized_aux_variables(
        res,
        [AuxVarKey(x...) for x in aux_variables];
        kwargs...,
    )
end

function read_realized_aux_variables(
    res::SimulationProblemResults,
    aux_variables::Vector{<:AbstractString};
    kwargs...,
)
    return read_realized_aux_variables(
        res,
        [_deserialize_key(AuxVarKey, res, x) for x in aux_variables];
        kwargs...,
    )
end

function read_realized_aux_variables(
    res::SimulationProblemResults,
    aux_variables::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    result_values = read_results_with_keys(res, aux_variables; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in result_values)
end

"""
Return the final values for the requested auxiliary variable for each time step for a problem.

Refer to [`read_realized_variable`](@ref) for help and examples.
"""
function read_realized_aux_variable(
    res::SimulationProblemResults,
    aux_variable::AbstractString;
    kwargs...,
)
    return first(
        values(
            read_realized_aux_variables(
                res,
                [_deserialize_key(AuxVarKey, res, aux_variable)];
                kwargs...,
            ),
        ),
    )
end

function read_realized_aux_variable(
    res::SimulationProblemResults,
    aux_variable...;
    kwargs...,
)
    return first(
        values(
            read_realized_aux_variables(res, [AuxVarKey(aux_variable...)]; kwargs...),
        ),
    )
end

"""
Return the final values for the requested parameters for each time step for a problem.

Refer to [`read_realized_parameters`](@ref) for help and examples.
"""
function read_realized_parameters(res::SimulationProblemResults; kwargs...)
    return read_realized_parameters(res, list_parameter_keys(res); kwargs...)
end

function read_realized_parameters(
    res::SimulationProblemResults,
    parameters::Vector{Tuple{DataType, DataType}};
    kwargs...,
)
    return read_realized_parameters(
        res,
        [ParameterKey(x...) for x in parameters];
        kwargs...,
    )
end

function read_realized_parameters(
    res::SimulationProblemResults,
    parameters::Vector{<:AbstractString};
    kwargs...,
)
    return read_realized_parameters(
        res,
        [_deserialize_key(ParameterKey, res, x) for x in parameters];
        kwargs...,
    )
end

function read_realized_parameters(
    res::SimulationProblemResults,
    parameters::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    result_values = read_results_with_keys(res, parameters; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in result_values)
end

"""
Return the final values for the requested parameter for each time step for a problem.

Refer to [`read_realized_variable`](@ref) for help and examples.
"""
function read_realized_parameter(
    res::SimulationProblemResults,
    parameter::AbstractString;
    kwargs...,
)
    return first(
        values(
            read_realized_parameters(
                res,
                [_deserialize_key(ParameterKey, res, parameter)];
                kwargs...,
            ),
        ),
    )
end

function read_realized_parameter(res::SimulationProblemResults, parameter...; kwargs...)
    return first(
        values(read_realized_parameters(res, [ParameterKey(parameter...)]; kwargs...)),
    )
end

"""
Return the final values for the requested duals for each time step for a problem.

Refer to [`read_realized_duals`](@ref) for help and examples.
"""
function read_realized_duals(res::SimulationProblemResults; kwargs...)
    return read_realized_duals(res, list_dual_keys(res); kwargs...)
end

function read_realized_duals(
    res::SimulationProblemResults,
    duals::Vector{Tuple{DataType, DataType}};
    kwargs...,
)
    return read_realized_duals(res, [ConstraintKey(x...) for x in duals]; kwargs...)
end

function read_realized_duals(
    res::SimulationProblemResults,
    duals::Vector{<:AbstractString};
    kwargs...,
)
    return read_realized_duals(
        res,
        [_deserialize_key(ConstraintKey, res, x) for x in duals];
        kwargs...,
    )
end

function read_realized_duals(
    res::SimulationProblemResults,
    duals::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    result_values = read_results_with_keys(res, duals; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in result_values)
end

"""
Return the final values for the requested dual for each time step for a problem.

Refer to [`read_realized_variable`](@ref) for help and examples.
"""
function read_realized_dual(res::SimulationProblemResults, dual::AbstractString; kwargs...)
    return first(
        values(
            read_realized_duals(
                res,
                [_deserialize_key(ConstraintKey, res, dual)];
                kwargs...,
            ),
        ),
    )
end

function read_realized_dual(res::SimulationProblemResults, dual...; kwargs...)
    return first(values(read_realized_duals(res, [ConstraintKey(dual...)]; kwargs...)))
end

"""
Return the final values for the requested expressions for each time step for a problem.

Refer to [`read_realized_expressions`](@ref) for help and examples.
"""
function read_realized_expressions(res::SimulationProblemResults; kwargs...)
    return read_realized_expressions(res, list_expression_keys(res); kwargs...)
end

function read_realized_expressions(
    res::SimulationProblemResults,
    expressions::Vector{Tuple{DataType, DataType}};
    kwargs...,
)
    return read_realized_expressions(
        res,
        [ExpressionKey(x...) for x in expressions];
        kwargs...,
    )
end

function read_realized_expressions(
    res::SimulationProblemResults,
    expressions::Vector{<:AbstractString};
    kwargs...,
)
    return read_realized_expressions(
        res,
        [_deserialize_key(ExpressionKey, res, x) for x in expressions];
        kwargs...,
    )
end

function read_realized_expressions(
    res::SimulationProblemResults,
    expressions::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    result_values = read_results_with_keys(res, expressions; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in result_values)
end

"""
Return the final values for the requested expression for each time step for a problem.

Refer to [`read_realized_variable`](@ref) for help and examples.
"""
function read_realized_expression(
    res::SimulationProblemResults,
    expression::AbstractString;
    kwargs...,
)
    return first(
        values(
            read_realized_expressions(
                res,
                [_deserialize_key(ExpressionKey, res, expression)];
                kwargs...,
            ),
        ),
    )
end

function read_realized_expression(res::SimulationProblemResults, expression...; kwargs...)
    return first(
        values(
            read_realized_expressions(res, [ExpressionKey(expression...)]; kwargs...),
        ),
    )
end

"""
Return the optimizer stats for the problem as a DataFrame.

# Accepted keywords

  - `store::SimulationStore`: a store that has been opened for reading
"""
function IOM.read_optimizer_stats(res::SimulationProblemResults; store = nothing)
    _store = isnothing(store) ? res.store : store
    return _read_optimizer_stats(res, _store)
end

function _read_optimizer_stats(res::SimulationProblemResults, ::Nothing)
    open_store(
        HdfSimulationStore,
        joinpath(get_execution_path(res), "data_store"),
        "r",
    ) do store
        _read_optimizer_stats(res, store)
    end
end

# Chooses the user-passed store or results store for reading values. Either could be
# something or nothing. If both are nothing, we must open the HDF5 store.
try_resolve_store(user::SimulationStore, results::Union{Nothing, SimulationStore}) = user
try_resolve_store(user::Nothing, results::SimulationStore) = results
try_resolve_store(user::Nothing, results::Nothing) = nothing
