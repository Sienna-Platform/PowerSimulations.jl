abstract type OperationModelSimulationOutputs end
# Subtypes need to implement the following methods for SimulationProblemOutputs{T}
# - IOM.read_outputs_with_keys
# - list_aux_variable_keys
# - list_dual_keys
# - list_expression_keys
# - list_parameter_keys
# - list_variable_keys
# - load_outputs!

"""
Holds the outputs of a simulation problem for plotting or exporting.
"""
mutable struct SimulationProblemOutputs{T} <:
               IS.Outputs where {T <: OperationModelSimulationOutputs}
    problem::String
    base_power::Float64
    execution_path::String
    output_dir::String
    timestamps::StepRange{Dates.DateTime, Dates.Millisecond}
    outputs_timestamps::Vector{Dates.DateTime}
    values::T
    system::Union{Nothing, PSY.System}
    system_uuid::Base.UUID
    resolution::Dates.TimePeriod
    store::Union{Nothing, SimulationStore}
    # Shared with every other SimulationProblemOutputs from the same SimulationOutputs (same
    # Dict instance, not a copy). A decision model's bundle-backed System, once loaded via
    # `get_system!`, is registered here under its uuid so a sibling result (e.g. the Emulator,
    # which borrows a decision model's bundle -- R31) can read that bundle's already-open
    # store instead of opening a second, colliding handle to the same sidecar file.
    system_registry::Dict{Base.UUID, POM.ParameterTimeSeriesStore}
end

function SimulationProblemOutputs{T}(
    store::SimulationStore,
    model_name::AbstractString,
    problem_params::ModelStoreParams,
    sim_params::SimulationStoreParams,
    path,
    vals::T;
    output_path = nothing,
    system = nothing,
    system_registry = Dict{Base.UUID, POM.ParameterTimeSeriesStore}(),
) where {T <: OperationModelSimulationOutputs}
    if isnothing(output_path)
        output_path = joinpath(path, OUTPUTS_DIR)
    end

    time_steps = range(
        sim_params.initial_time;
        length = problem_params.num_executions * sim_params.num_steps,
        step = problem_params.interval,
    )
    return SimulationProblemOutputs{T}(
        model_name,
        problem_params.base_power,
        path,
        output_path,
        time_steps,
        Vector{Dates.DateTime}(),
        vals,
        system,
        problem_params.system_uuid,
        IOM.get_resolution(problem_params),
        _retained_store(store),
        system_registry,
    )
end

get_model_name(res::SimulationProblemOutputs) = res.problem
IOM.get_system(res::SimulationProblemOutputs) = res.system
IS.get_source_data(res::SimulationProblemOutputs) = get_system(res)  # Needed for compatibility with the IS.Outputs interface
IOM.get_resolution(res::SimulationProblemOutputs) = res.resolution
get_execution_path(res::SimulationProblemOutputs) = res.execution_path
IOM.get_model_base_power(res::SimulationProblemOutputs) = res.base_power
get_system_uuid(results::PSI.SimulationProblemOutputs) = results.system_uuid
IS.get_timestamp(result::SimulationProblemOutputs) = result.outputs_timestamps
IOM.get_interval(res::SimulationProblemOutputs) = res.timestamps.step
IOM.get_base_power(result::SimulationProblemOutputs) = result.base_power
get_output_dir(res::SimulationProblemOutputs) = res.output_dir

get_outputs_timestamps(result::SimulationProblemOutputs) = result.outputs_timestamps
function set_outputs_timestamps!(
    result::SimulationProblemOutputs,
    outputs_timestamps::Vector{Dates.DateTime},
)
    result.outputs_timestamps = outputs_timestamps
end

list_output_keys(res::SimulationProblemOutputs, ::AuxVarKey) =
    list_aux_variable_keys(res)
list_output_keys(res::SimulationProblemOutputs, ::ConstraintKey) =
    list_dual_keys(res)
list_output_keys(res::SimulationProblemOutputs, ::ExpressionKey) =
    list_expression_keys(res)
list_output_keys(res::SimulationProblemOutputs, ::ParameterKey) =
    list_parameter_keys(res)
list_output_keys(res::SimulationProblemOutputs, ::VariableKey) =
    list_variable_keys(res)

get_cached_outputs(res::SimulationProblemOutputs, ::Type{<:AuxVarKey}) =
    get_cached_aux_variables(res)
get_cached_outputs(res::SimulationProblemOutputs, ::Type{<:ConstraintKey}) =
    get_cached_duals(res)
get_cached_outputs(res::SimulationProblemOutputs, ::Type{<:ExpressionKey}) =
    get_cached_expressions(res)
get_cached_outputs(res::SimulationProblemOutputs, ::Type{<:ParameterKey}) =
    get_cached_parameters(res)
get_cached_outputs(res::SimulationProblemOutputs, ::Type{<:VariableKey}) =
    get_cached_variables(res)
get_cached_outputs(
    res::SimulationProblemOutputs,
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
IOM.list_variable_names(res::SimulationProblemOutputs) =
    encode_keys_as_strings(list_variable_keys(res))

"""
Return an array of dual names (strings) that are available for reads.
"""
IOM.list_dual_names(res::SimulationProblemOutputs) =
    encode_keys_as_strings(list_dual_keys(res))

"""
Return an array of parmater names (strings) that are available for reads.
"""
IOM.list_parameter_names(res::SimulationProblemOutputs) =
    encode_keys_as_strings(list_parameter_keys(res))

"""
Return an array of auxillary variable names (strings) that are available for reads.
"""
IOM.list_aux_variable_names(res::SimulationProblemOutputs) =
    encode_keys_as_strings(list_aux_variable_keys(res))

"""
Return an array of expression names (strings) that are available for reads.
"""
IOM.list_expression_names(res::SimulationProblemOutputs) =
    encode_keys_as_strings(list_expression_keys(res))

"""
Return a reference to a StepRange of available timestamps.
"""
IOM.get_timestamps(result::SimulationProblemOutputs) = result.timestamps

"""
Return the system used for the problem. If the system hasn't already been deserialized or
set with [`set_system!`](@ref) then deserialize and store it.
"""
function get_system!(
    results::Union{IOM.OptimizationProblemOutputs, SimulationProblemOutputs};
    kwargs...,
)
    !isnothing(get_system(results)) && return get_system(results)

    bundle = locate_system_bundle(results)
    # This flag should remain unpublished because it should never be needed
    # by the general audience.
    if !get(kwargs, :use_system_fallback, false) && ispath(bundle)
        system = PSY.from_file(bundle; time_series_read_only = true)
        # The OpenAPI document carries components and attributes, not system identity, so
        # `from_file` hands back a System with a fresh UUID. Restore the one this bundle was
        # named for, or the validating `set_system!` below rejects it as a mismatch.
        PSY.set_system_uuid!(system, _expected_system_uuid(results))
        @info "De-serialized the system from files."
    else
        system = get_system_fallback(results)
    end

    set_system!(results, system)
    return get_system(results)
end

_expected_system_uuid(results::SimulationProblemOutputs) = results.system_uuid
_expected_system_uuid(results::IOM.OptimizationProblemOutputs) =
    get_source_data_uuid(results)

get_system_fallback(results::SimulationProblemOutputs) =
    _deserialize_system(results, results.store)
get_system_fallback(results::IOM.OptimizationProblemOutputs) =
    error("Could not locate system")

# The `system-<uuid>` bundle PowerOperationsModels writes beside a model's outputs -- the only
# form that carries the time series values, which the store's snapshot does not.
locate_system_bundle(results::SimulationProblemOutputs) = joinpath(
    get_execution_path(results),
    "problems",
    get_model_name(results),
    IOM.make_system_dirname(results.system_uuid),
)

locate_system_bundle(results::IOM.OptimizationProblemOutputs) = joinpath(
    get_output_dir(results),
    IOM.make_system_dirname(get_source_data_uuid(results)),
)

set_system!(results::IOM.OptimizationProblemOutputs, system) =
    set_source_data!(results, system)

# Only the in-memory store is kept on the results; an HDF store is reopened on demand.
_retained_store(::HdfSimulationStore) = nothing
_retained_store(store::InMemorySimulationStore) = store

function _deserialize_system(results::SimulationProblemOutputs, ::Nothing)
    error("No System bundle at $(locate_system_bundle(results))")
end

function _deserialize_system(::SimulationProblemOutputs, ::InMemorySimulationStore)
    # This should never be necessary because the system is guaranteed to be in memory.
    error("Deserializing a system from the InMemorySimulationStore is not supported.")
end

"""
Set the system in the results instance.

Throws InvalidValue if the system UUID is incorrect.

# Arguments

  - `results::SimulationProblemOutputs`: Results object
  - `system::AbstractString`: Path to a serialized system -- a bundle directory, a `.json`
    document, or a `.sns` archive

# Examples

```julia
julia > set_system!(res, "my_path/system-\$(uuid)")
```
"""
function set_system!(results::SimulationProblemOutputs, system::AbstractString)
    set_system!(results, PSY.from_file(system))
end

function set_system!(results::SimulationProblemOutputs, system::PSY.System)
    sys_uuid = PSY.get_system_uuid(system)
    if sys_uuid != results.system_uuid
        throw(
            IS.InvalidValue(
                "System mismatch. $sys_uuid does not match the stored value of $(results.system_uuid)",
            ),
        )
    end

    results.system = system
    # Shared registry (R30): any sibling result reading the same bundle by uuid -- e.g. the
    # Emulator borrowing a decision model's bundle -- can reuse this already-open store
    # instead of opening a second, colliding handle. `_has_borrowed_store` verifies the
    # sidecar path before ever trusting an entry, so registering unconditionally here is safe.
    results.system_registry[sys_uuid] = POM.parameter_store_of(system)
    return
end

"""
Merge `res`'s shared `system_registry` into the freshly-opened `store` (R30) before reading.
`system_registry` is shared by every `SimulationProblemOutputs` from the same
`SimulationOutputs` (same `Dict` instance), so a bundle-backed `System` loaded through any of
them -- including a sibling model's, e.g. the Emulator borrowing a decision model's bundle
(R31) -- reuses that already-open handle instead of opening a second, colliding one. A no-op
when nothing has been loaded yet.
"""
_register_borrowed_stores!(::SimulationStore, ::SimulationProblemOutputs) = nothing

function _register_borrowed_stores!(
    store::HdfSimulationStore,
    res::SimulationProblemOutputs,
)
    merge!(store.borrowed_parameter_stores, res.system_registry)
    return nothing
end

function IOM._deserialize_key(
    ::Type{<:OptimizationContainerKey},
    results::SimulationProblemOutputs,
    name::AbstractString,
)
    !haskey(results.values.container_key_lookup, name) && error("$name is not stored")
    return results.values.container_key_lookup[name]
end

function IOM._deserialize_key(
    ::Type{T},
    results::SimulationProblemOutputs,
    args...,
) where {T <: OptimizationContainerKey}
    return make_key(T, args...)
end

get_container_fields(x::SimulationProblemOutputs) =
    (:aux_variables, :duals, :expressions, :parameters, :variables)

"""
Return the final values for the requested variables for each time step for a problem.

Decision problem results are returned in a Dict{String, Dict{DateTime, DataFrame}}.

Emulation problem results are returned in a Dict{String, DataFrame}.

Limit the data sizes returned by specifying `start_time` and `len`.

If the Julia process is started with multiple threads, the code will read the variables in
parallel.

See also [`load_outputs!`](@ref) to preload data into memory.

# Arguments

  - `variables::Vector{Union{String, Tuple}}`: Variable name as a string or a Tuple with
    variable type and device type. If not provided then return all variables.
  - `start_time::Dates.DateTime`: Start time of the requested results.
  - `len::Int`: Number of results (decision problems) or rows in each DataFrame (emulation
    problems).
  - `table_format::TableFormat.Value`: Format of the table to be returned. Default is
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
function read_realized_variables(res::SimulationProblemOutputs; kwargs...)
    return read_realized_variables(res, list_variable_keys(res); kwargs...)
end

function read_realized_variables(
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
    variables::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    output_values = IOM.read_outputs_with_keys(res, variables; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in output_values)
end

"""
Return the final values for the requested variable for each time step for a problem.

Decision problem results are returned in a Dict{DateTime, DataFrame}.

Emulation problem results are returned in a DataFrame.

Limit the data sizes returned by specifying `start_time` and `len`.

See also [`load_outputs!`](@ref) to preload data into memory.

# Arguments

  - `variable::Union{String, Tuple}`: Variable name as a string or a Tuple with
    variable type and device type.
  - `start_time::Dates.DateTime`: Start time of the requested results.
  - `len::Int`: Number of results (decision problems) or rows in each DataFrame (emulation
    problems).
  - `table_format::TableFormat.Value`: Format of the table to be returned. Default is
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
    res::SimulationProblemOutputs,
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

function read_realized_variable(res::SimulationProblemOutputs, variable...; kwargs...)
    return first(
        values(read_realized_variables(res, [VariableKey(variable...)]; kwargs...)),
    )
end

"""
Return the final values for the requested auxiliary variables for each time step for a problem.

Refer to [`read_realized_aux_variables`](@ref) for help and examples.
"""
function read_realized_aux_variables(res::SimulationProblemOutputs; kwargs...)
    return read_realized_aux_variables(
        res,
        list_aux_variable_keys(res);
        kwargs...,
    )
end

function read_realized_aux_variables(
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
    aux_variables::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    output_values = IOM.read_outputs_with_keys(res, aux_variables; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in output_values)
end

"""
Return the final values for the requested auxiliary variable for each time step for a problem.

Refer to [`read_realized_variable`](@ref) for help and examples.
"""
function read_realized_aux_variable(
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
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
function read_realized_parameters(res::SimulationProblemOutputs; kwargs...)
    return read_realized_parameters(res, list_parameter_keys(res); kwargs...)
end

function read_realized_parameters(
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
    parameters::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    output_values = IOM.read_outputs_with_keys(res, parameters; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in output_values)
end

"""
Return the final values for the requested parameter for each time step for a problem.

Refer to [`read_realized_variable`](@ref) for help and examples.
"""
function read_realized_parameter(
    res::SimulationProblemOutputs,
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

function read_realized_parameter(res::SimulationProblemOutputs, parameter...; kwargs...)
    return first(
        values(read_realized_parameters(res, [ParameterKey(parameter...)]; kwargs...)),
    )
end

"""
Return the final values for the requested duals for each time step for a problem.

Refer to [`read_realized_duals`](@ref) for help and examples.
"""
function read_realized_duals(res::SimulationProblemOutputs; kwargs...)
    return read_realized_duals(res, list_dual_keys(res); kwargs...)
end

function read_realized_duals(
    res::SimulationProblemOutputs,
    duals::Vector{Tuple{DataType, DataType}};
    kwargs...,
)
    return read_realized_duals(res, [ConstraintKey(x...) for x in duals]; kwargs...)
end

function read_realized_duals(
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
    duals::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    output_values = IOM.read_outputs_with_keys(res, duals; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in output_values)
end

"""
Return the final values for the requested dual for each time step for a problem.

Refer to [`read_realized_variable`](@ref) for help and examples.
"""
function read_realized_dual(res::SimulationProblemOutputs, dual::AbstractString; kwargs...)
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

function read_realized_dual(res::SimulationProblemOutputs, dual...; kwargs...)
    return first(values(read_realized_duals(res, [ConstraintKey(dual...)]; kwargs...)))
end

"""
Return the final values for the requested expressions for each time step for a problem.

Refer to [`read_realized_expressions`](@ref) for help and examples.
"""
function read_realized_expressions(res::SimulationProblemOutputs; kwargs...)
    return read_realized_expressions(res, list_expression_keys(res); kwargs...)
end

function read_realized_expressions(
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
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
    res::SimulationProblemOutputs,
    expressions::Vector{<:OptimizationContainerKey};
    kwargs...,
)
    output_values = IOM.read_outputs_with_keys(res, expressions; kwargs...)
    return Dict(encode_key_as_string(k) => v for (k, v) in output_values)
end

"""
Return the final values for the requested expression for each time step for a problem.

Refer to [`read_realized_variable`](@ref) for help and examples.
"""
function read_realized_expression(
    res::SimulationProblemOutputs,
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

function read_realized_expression(res::SimulationProblemOutputs, expression...; kwargs...)
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
function IOM.read_optimizer_stats(res::SimulationProblemOutputs; store = nothing)
    return _read_optimizer_stats(res, try_resolve_store(store, res.store))
end

function _read_optimizer_stats(res::SimulationProblemOutputs, ::Nothing)
    _open_outputs_store(get_execution_path(res)) do store
        _read_optimizer_stats(res, store)
    end
end

# Chooses the user-passed store or results store for reading values. Either could be
# something or nothing. If both are nothing, we must open the HDF5 store.
try_resolve_store(user::SimulationStore, results::Union{Nothing, SimulationStore}) = user
try_resolve_store(user::Nothing, results::SimulationStore) = results
try_resolve_store(user::Nothing, results::Nothing) = nothing
