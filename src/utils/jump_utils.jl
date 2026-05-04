const IntegerAxis = Union{Vector{Int}, UnitRange{Int}}

function get_hinted_aff_expr(size::Int)
    expr = JuMP.AffExpr(0.0)
    sizehint!(expr.terms, size)
    return expr
end

#Given the changes in syntax in ParameterJuMP and the new format to create anonymous parameters
function add_jump_parameter(jump_model::JuMP.Model, val::Number)
    param = JuMP.@variable(jump_model, base_name = "param")
    JuMP.fix(param, val; force = true)
    return param
end

function write_data(base_power::Float64, save_path::String)
    JSON3.write(joinpath(save_path, "base_power.json"), JSON3.json(base_power))
    return
end

function jump_value(input::JuMP.VariableRef)::Float64
    if JuMP.is_fixed(input)
        return JuMP.fix_value(input)
    elseif JuMP.has_values(input.model)
        return JuMP.value(input)
    else
        return NaN
    end
end

function jump_value(input::T)::Float64 where {T <: JuMP.AbstractJuMPScalar}
    return JuMP.value(input)
end

function jump_value(input::JuMP.ConstraintRef)::Float64
    return JuMP.dual(input)
end

function jump_value(input::JumpSupportedLiterals)
    return input
end

# Like jump_value but for certain special cases before optimize! is called
jump_fixed_value(input::Number) = input
jump_fixed_value(input::JuMP.VariableRef) = JuMP.fix_value(input)
jump_fixed_value(input::JuMP.AffExpr) =
    sum([coeff * jump_fixed_value(param) for (coeff, param) in JuMP.linear_terms(input)]) +
    JuMP.constant(input)

function fix_parameter_value(input::JuMP.VariableRef, value::Float64)
    JuMP.fix(input, value; force = true)
    return
end

"""
Convert Vectors, DenseAxisArrays, and SparkAxisArrays to a matrix.

- If the input is a 1d array or DenseAxisArray, the returned matrix will have
  a number of rows equal to the length of the input and one column.
- If the input is a 2d DenseAxisArray, the dimensions are transposed, due to the way we
  store outputs in JuMP.
"""
function to_matrix(vec::Vector)
    data = vec[:]
    return reshape(data, length(data), 1)
end

to_matrix(array::Matrix) = array

function to_matrix(array::DenseAxisArray{T, 1}) where {T}
    data = array.data[:]
    return reshape(data, length(data), 1)
end

function to_matrix(array::DenseAxisArray{T, 2}) where {T}
    return permutedims(array.data)
end

function to_matrix(array::DenseAxisArray)
    error("Converting type = $(typeof(array)) to a matrix is not supported.")
end

function _to_matrix(
    array::SparseAxisArray{T, N, K},
    columns,
) where {T, N, K <: NTuple{N, Any}}
    time_steps = Set{Int}(k[N] for k in keys(array.data))
    data = Matrix{Float64}(undef, length(time_steps), length(columns))
    for (ix, col) in enumerate(columns), t in time_steps
        data[t, ix] = array.data[(col..., t)]
    end
    return data
end

function to_matrix(array::SparseAxisArray{T, N, K}) where {T, N, K <: NTuple{N, Any}}
    # Don't use get_column_names_from_axis_array to avoid additional string conversion
    # TODO: I don't understand why we have two mechanisms of creating columns.
    # Why does get_column_names_from_axis_array rely on encode_tuple_to_column?
    columns = sort!(unique!([k[1:(N - 1)] for k in keys(array.data)]))
    return _to_matrix(array, columns)
end

"""
Return column names from the axes of a JuMP DenseAxisArray or SparseAxisArray.
The columns are returned as a tuple of vector of strings.
1d and 2d arrays will return a tuple of length 1.
3d arrays will return a tuple of length 2.

There are two variants of this function:
  - get_column_names_from_axis_array(array)
  - get_column_names_from_axis_array(::OptimizationContainerKey, array)
  
When the variant with the key is called:
  In cases where the array has one dimension, retrieve the column names from the key.
  In cases where the array has two or more dimensions, retrieve the column names from the
  axes.
"""
# TODO: the docstring describes what the code does.
# The behavior of key vs axes seems suspect to me.
function get_column_names_from_axis_array(
    array::DenseAxisArray{T, 1, <:Tuple{Vector{String}}},
) where {T}
    return (axes(array, 1),)
end

function get_column_names_from_axis_array(
    array::DenseAxisArray{T, 1, <:Tuple{IntegerAxis}},
) where {T}
    # This happens because buses are stored by numbers instead of name.
    return (string.(axes(array, 1)),)
end

function get_column_names_from_axis_array(
    array::DenseAxisArray{T, 2, <:Tuple{Vector{String}, IntegerAxis}},
) where {T}
    return (axes(array, 1),)
end

function get_column_names_from_axis_array(
    array::DenseAxisArray{T, 2, <:Tuple{IntegerAxis, IntegerAxis}},
) where {T}
    return (string.(axes(array, 1)),)
end

function get_column_names_from_axis_array(
    array::DenseAxisArray{T, 3, <:Tuple{Vector{String}, Vector{String}, IntegerAxis}},
) where {T}
    return (axes(array, 1), axes(array, 2))
end

function get_column_names_from_axis_array(
    array::DenseAxisArray{T, 3, <:Tuple{Vector{String}, IntegerAxis, IntegerAxis}},
) where {T}
    return (axes(array, 1), string.(axes(array, 2)))
end

function get_column_names_from_axis_array(
    key::OptimizationContainerKey,
    ::DenseAxisArray{T, 1},
) where {T}
    return get_column_names_from_key(key)
end

function get_column_names_from_axis_array(::OptimizationContainerKey, array::DenseAxisArray)
    return get_column_names_from_axis_array(array)
end

function get_column_names_from_axis_array(
    ::OptimizationContainerKey,
    array::SparseAxisArray,
)
    return get_column_names_from_axis_array(array)
end

function get_column_names_from_axis_array(
    array::SparseAxisArray{T, N, K},
) where {T, N, K <: NTuple{N, Any}}
    return (
        sort!(
            collect(Set(encode_tuple_to_column(k[1:(N - 1)]) for k in keys(array.data))),
        ),
    )
end

"""
Return the column names from a key as a tuple of vector of strings.
Only useful for 1d DenseAxisArrays.
"""
function get_column_names_from_key(key::OptimizationContainerKey)
    return ([encode_key_as_string(key)],)
end

function encode_tuple_to_column(val::NTuple{N, <:AbstractString}) where {N}
    return join(val, PSI_NAME_DELIMITER)
end

function encode_tuple_to_column(val::Tuple{String, Int})
    return join(string.(val), PSI_NAME_DELIMITER)
end

"""
Create a DataFrame from a JuMP DenseAxisArray or SparseAxisArray.

# Arguments

  - `array`: JuMP DenseAxisArray or SparseAxisArray to convert
  - `key::OptimizationContainerKey`:
"""
function to_dataframe(
    array::DenseAxisArray{T, 2},
    key::OptimizationContainerKey,
) where {T <: JumpSupportedLiterals}
    return DataFrame(to_matrix(array), get_column_names_from_axis_array(key, array)[1])
end

function to_dataframe(
    array::DenseAxisArray{T, 2, <:Tuple{Vector{String}, UnitRange{Int}}},
) where {T <: JumpSupportedLiterals}
    return DataFrame(to_matrix(array), get_column_names_from_axis_array(array)[1])
end

function to_dataframe(
    array::DenseAxisArray{T, 1},
    key::OptimizationContainerKey,
) where {T <: JumpSupportedLiterals}
    cols = get_column_names_from_axis_array(key, array)[1]
    if length(cols) != 1
        error("Expected a single column, got $(length(cols))")
    end
    return DataFrame(Symbol(cols[1]) => array.data)
end

function to_dataframe(array::SparseAxisArray, key::OptimizationContainerKey)
    return DataFrame(to_matrix(array), get_column_names_from_axis_array(key, array)[1])
end

"""
Convert a DenseAxisArray containing components to a results DataFrame consumable by users.

# Arguments
- `array: DenseAxisArray`: JuMP DenseAxisArray to convert
- `timestamps`: Iterable of timestamps for each component or nothing if time is not known.
  The resulting DataFrame will have the column "DateTime" if timestamps is not nothing.
  Otherwise, it will have the column "time_index", representing the index of the time
  dimension.
- `::Val{TableFormat}`: Format of the table to create.
  If it is TableFormat.LONG, the DataFrame will have the column "name", and, if
  the data has three dimensions, "name2."
  If it is TableFormat.WIDE, the DataFrame will have columns for each component. Wide
  format does not support arrays with more than two dimensions.
"""
function to_results_dataframe(array::DenseAxisArray, timestamps)
    return to_results_dataframe(array, timestamps, Val(TableFormat.LONG))()
end

function to_results_dataframe(
    array::DenseAxisArray{Float64, 1, <:Tuple{Vector{String}}},
    timestamps,
    ::Val{TableFormat.LONG},
)
    return DataFrames.DataFrame(
        :DateTime => ones(Int, length(axes(array, 1))),
        :name => axes(array, 1),
        :value => array.data,
    )
end

function to_results_dataframe(
    array::DenseAxisArray{Float64, 2, <:Tuple{Vector{String}, IntegerAxis}},
    timestamps,
    ::Val{TableFormat.LONG},
)
    num_timestamps = size(array, 2)
    if length(timestamps) != num_timestamps
        error(
            "The number of timestamps must match the number of rows per component. " *
            "timestamps = $(length(timestamps)) " *
            "num_timestamps = $num_timestamps",
        )
    end
    num_rows = length(array.data)
    timestamps_arr = _collect_timestamps(timestamps)
    time_col = Vector{Dates.DateTime}(undef, num_rows)
    name_col = Vector{String}(undef, num_rows)

    row_index = 1
    for name in axes(array, 1)
        for time_index in axes(array, 2)
            time_col[row_index] = timestamps_arr[time_index]
            name_col[row_index] = name
            row_index += 1
        end
    end

    return DataFrame(
        :DateTime => time_col,
        :name => name_col,
        :value => reshape(permutedims(array.data), num_rows),
    )
end

_collect_timestamps(timestamps::Vector{Dates.DateTime}) = timestamps
_collect_timestamps(timestamps) = collect(timestamps)

function to_results_dataframe(
    array::DenseAxisArray{Float64, 2, <:Tuple{Vector{String}, IntegerAxis}},
    ::Nothing,
    ::Val{TableFormat.LONG},
)
    num_rows = length(array.data)
    time_col = Vector{Int}(undef, num_rows)
    name_col = Vector{String}(undef, num_rows)

    row_index = 1
    for name in axes(array, 1)
        for time_index in axes(array, 2)
            time_col[row_index] = time_index
            name_col[row_index] = name
            row_index += 1
        end
    end

    return DataFrame(
        :time_index => time_col,
        :name => name_col,
        :value => reshape(permutedims(array.data), num_rows),
    )
end

function to_results_dataframe(
    array::DenseAxisArray{Float64, 1, <:Tuple{IntegerAxis}},
    ::Nothing,
    ::Val{TableFormat.LONG},
)
    num_rows = length(array.data)
    time_col = Vector{Int}(undef, num_rows)
    name_col = Vector{String}(undef, num_rows)
    row_index = 1
    name = "Result"
    for time_index in axes(array, 1)
        time_col[row_index] = time_index
        name_col[row_index] = name
        row_index += 1
    end

    return DataFrame(
        :time_index => time_col,
        :name => name_col,
        :value => reshape(permutedims(array.data), num_rows),
    )
end

function to_results_dataframe(
    array::DenseAxisArray{Float64, 2, <:Tuple{Vector{String}, IntegerAxis}},
    timestamps,
    ::Val{TableFormat.WIDE},
)
    df = DataFrame(to_matrix(array), axes(array, 1))
    DataFrames.insertcols!(df, 1, :DateTime => timestamps)
    return df
end

function to_results_dataframe(
    array::DenseAxisArray{Float64, 2, <:Tuple{Vector{String}, IntegerAxis}},
    ::Nothing,
    ::Val{TableFormat.WIDE},
)
    df = DataFrame(to_matrix(array), axes(array, 1))
    DataFrames.insertcols!(df, 1, :time_index => axes(array, 2))
    return df
end

function to_results_dataframe(
    array::DenseAxisArray{
        Float64,
        3,
        <:Tuple{Vector{String}, Vector{String}, UnitRange{Int}},
    },
    timestamps,
    ::Val{TableFormat.LONG},
)
    num_timestamps = size(array, 3)
    if length(timestamps) != num_timestamps
        error(
            "The number of timestamps must match the number of rows per component. " *
            "timestamps = $(length(timestamps)) " *
            "num_timestamps = $num_timestamps",
        )
    end
    num_rows = length(array.data)
    timestamps_arr = _collect_timestamps(timestamps)
    time_col = Vector{Dates.DateTime}(undef, num_rows)
    name_col = Vector{String}(undef, num_rows)
    name2_col = Vector{String}(undef, num_rows)
    vals = Vector{Float64}(undef, num_rows)

    row_index = 1
    for name in axes(array, 1)
        for name2 in axes(array, 2)
            for time_index in axes(array, 3)
                time_col[row_index] = timestamps_arr[time_index]
                name_col[row_index] = name
                name2_col[row_index] = name2
                vals[row_index] = array[name, name2, time_index]
                row_index += 1
            end
        end
    end

    return DataFrame(
        :DateTime => time_col,
        :name => name_col,
        :name2 => name2_col,
        :value => vals,
    )
end

function to_results_dataframe(
    array::DenseAxisArray{
        Float64,
        3,
        <:Tuple{Vector{String}, Vector{String}, UnitRange{Int}},
    },
    ::Nothing,
    ::Val{TableFormat.LONG},
)
    num_rows = length(array.data)
    time_col = Vector{Int}(undef, num_rows)
    name_col = Vector{String}(undef, num_rows)
    name2_col = Vector{String}(undef, num_rows)
    vals = Vector{Float64}(undef, num_rows)

    row_index = 1
    for name in axes(array, 1)
        for name2 in axes(array, 2)
            for time_index in axes(array, 3)
                time_col[row_index] = time_index
                name_col[row_index] = name
                name2_col[row_index] = name2
                vals[row_index] = array[name, name2, time_index]
                row_index += 1
            end
        end
    end

    return DataFrame(
        :time_index => time_col,
        :name => name_col,
        :name2 => name2_col,
        :value => vals,
    )
end

function to_dataframe(array::SparseAxisArray{T, N, K}) where {T, N, K <: NTuple{N, Any}}
    columns = get_column_names_from_axis_array(array)
    return DataFrames.DataFrame(_to_matrix(array, columns), columns)
end

"""
Returns the correct container specification for the selected type of JuMP Model
"""
function container_spec(::Type{T}, axs...) where {T <: Any}
    return DenseAxisArray{T}(undef, axs...)
end

"""
Returns the correct container specification for the selected type of JuMP Model
"""
function container_spec(::Type{Float64}, axs...)
    cont = DenseAxisArray{Float64}(undef, axs...)
    cont.data .= fill(NaN, size(cont.data))
    return cont
end

"""
Returns the correct container specification for the selected type of JuMP Model
"""
function sparse_container_spec(::Type{T}, axs...) where {T <: JuMP.AbstractJuMPScalar}
    indexes = Base.Iterators.product(axs...)
    contents = Dict{eltype(indexes), T}(indexes .=> zero(T))
    return SparseAxisArray(contents)
end

function sparse_container_spec(::Type{T}, axs...) where {T <: JuMP.VariableRef}
    indexes = Base.Iterators.product(axs...)
    contents = Dict{eltype(indexes), Union{Nothing, T}}(indexes .=> nothing)
    return SparseAxisArray(contents)
end

function sparse_container_spec(::Type{T}, axs...) where {T <: JuMP.ConstraintRef}
    indexes = Base.Iterators.product(axs...)
    contents = Dict{eltype(indexes), Union{Nothing, T}}(indexes .=> nothing)
    return SparseAxisArray(contents)
end

function sparse_container_spec(::Type{T}, axs...) where {T <: Number}
    indexes = Base.Iterators.product(axs...)
    contents = Dict{eltype(indexes), T}(indexes .=> zero(T))
    return SparseAxisArray(contents)
end

function remove_undef!(expression_array::AbstractArray)
    # iteration is deliberately unsupported for CartesianIndex
    # Makes this code a bit hacky to be able to use isassigned with an array of arbitrary size.
    for i in CartesianIndices(expression_array.data)
        if !isassigned(expression_array.data, i.I...)
            expression_array.data[i] = zero(eltype(expression_array))
        end
    end

    return expression_array
end

remove_undef!(expression_array::SparseAxisArray) = expression_array

function _calc_dimensions(
    array::DenseAxisArray,
    key::OptimizationContainerKey,
    num_rows::Int,
    horizon::Int,
)
    ax = axes(array)
    columns = get_column_names_from_axis_array(key, array)
    # Two use cases for read:
    # 1. Read data for one execution for one device.
    # 2. Read data for one execution for all devices.
    # This will ensure that data on disk is contiguous in both cases.
    if length(ax) == 1
        if length(ax[1]) != horizon
            @debug "$(encode_key_as_string(key)) has length $(length(ax[1])). Different than horizon $horizon."
        end
        dims = (length(ax[1]), 1, num_rows)
    elseif length(ax) == 2
        if length(ax[2]) != horizon
            @debug "$(encode_key_as_string(key)) has length $(length(ax[1])). Different than horizon $horizon."
        end
        dims = (length(ax[2]), length(columns[1]), num_rows)
    elseif length(ax) == 3
        if length(ax[3]) != horizon
            @debug "$(encode_key_as_string(key)) has length $(length(ax[1])). Different than horizon $horizon."
        end
        dims = (length(ax[3]), length(columns[1]), length(columns[2]), num_rows)
    else
        error("unsupported data size $(length(ax))")
    end

    return Dict("columns" => columns, "dims" => dims)
end

function _calc_dimensions(
    array::SparseAxisArray,
    key::OptimizationContainerKey,
    num_rows::Int,
    horizon::Int,
)
    columns = get_column_names_from_axis_array(key, array)
    dims = (horizon, length.(columns)..., num_rows)
    return Dict("columns" => columns, "dims" => dims)
end

"""
Run this function only when getting detailed solver stats
"""
function _summary_to_dict!(optimizer_stats::OptimizerStats, jump_model::JuMP.Model)
    # JuMP.solution_summary uses a lot of try-catch so it has a performance hit and should be opt-in
    jump_summary = JuMP.solution_summary(jump_model; verbose = false)
    # Note we don't grab all the fields from the summary because not all can be encoded as Float for HDF store
    fields = [
        :has_values, # Bool
        :has_duals, # Bool
        # Candidate solution
        :objective_bound, # Union{Missing,Float64}
        :dual_objective_value, # Union{Missing,Float64}
        # Work counters
        :relative_gap, # Union{Missing,Int}
        :barrier_iterations, # Union{Missing,Int}
        :simplex_iterations, # Union{Missing,Int}
        :node_count, # Union{Missing,Int}
    ]

    for field in fields
        field_value = getfield(jump_summary, field)
        if ismissing(field_value)
            setfield!(optimizer_stats, field, missing)
        else
            setfield!(optimizer_stats, field, field_value)
        end
    end
    return
end

function supports_milp(jump_model::JuMP.Model)
    optimizer_backend = JuMP.backend(jump_model)
    return MOI.supports_constraint(optimizer_backend, MOI.VariableIndex, MOI.ZeroOne)
end

function _get_solver_time(jump_model::JuMP.Model)
    solver_solve_time = NaN

    try_s =
        get!(jump_model.ext, :try_supports_solvetime, (trycatch = true, supports = true))
    if try_s.trycatch
        try
            solver_solve_time = MOI.get(jump_model, MOI.SolveTimeSec())
            jump_model.ext[:try_supports_solvetime] = (trycatch = false, supports = true)
        catch
            @debug "SolveTimeSec() property not supported by the Solver"
            jump_model.ext[:try_supports_solvetime] = (trycatch = false, supports = false)
        end
    else
        if try_s.supports
            solver_solve_time = MOI.get(jump_model, MOI.SolveTimeSec())
        end
    end

    return solver_solve_time
end

function write_optimizer_stats!(optimizer_stats::OptimizerStats, jump_model::JuMP.Model)
    if JuMP.primal_status(jump_model) == MOI.FEASIBLE_POINT::MOI.ResultStatusCode
        optimizer_stats.objective_value = JuMP.objective_value(jump_model)
    else
        optimizer_stats.objective_value = Inf
    end

    optimizer_stats.termination_status = Int(JuMP.termination_status(jump_model))
    optimizer_stats.primal_status = Int(JuMP.primal_status(jump_model))
    optimizer_stats.dual_status = Int(JuMP.dual_status(jump_model))
    optimizer_stats.result_count = JuMP.result_count(jump_model)
    optimizer_stats.solve_time = _get_solver_time(jump_model)
    if optimizer_stats.detailed_stats
        _summary_to_dict!(optimizer_stats, jump_model)
    end
    return
end

"""
Exports the JuMP object in MathOptFormat
"""
function serialize_jump_optimization_model(jump_model::JuMP.Model, save_path::String)
    MOF_model = MOPFM(; format = MOI.FileFormats.FORMAT_MOF)
    MOI.copy_to(MOF_model, JuMP.backend(jump_model))
    MOI.write_to_file(MOF_model, save_path)
    return
end

function write_lp_file(jump_model::JuMP.Model, save_path::String)
    MOF_model = MOPFM(; format = MOI.FileFormats.FORMAT_LP)
    MOI.copy_to(MOF_model, JuMP.backend(jump_model))
    MOI.write_to_file(MOF_model, save_path)
    return
end

# check_conflict_status functions can't be tested on CI because free solvers don't support IIS
function check_conflict_status(
    jump_model::JuMP.Model,
    constraint_container::DenseAxisArray{JuMP.ConstraintRef},
)
    conflict_indices = Vector()
    dims = axes(constraint_container)
    for index in Iterators.product(dims...)
        if isassigned(constraint_container, index...) &&
           MOI.get(
            jump_model,
            MOI.ConstraintConflictStatus(),
            constraint_container[index...],
        ) != MOI.NOT_IN_CONFLICT
            push!(conflict_indices, index)
        end
    end
    return conflict_indices
end

function check_conflict_status(
    jump_model::JuMP.Model,
    constraint_container::SparseAxisArray{JuMP.ConstraintRef},
)
    conflict_indices = Vector()
    for (index, constraint) in constraint_container
        if isassigned(constraint_container, index...) &&
           MOI.get(jump_model, MOI.ConstraintConflictStatus(), constraint) !=
           MOI.NOT_IN_CONFLICT
            push!(conflict_indices, index)
        end
    end
    return conflict_indices
end
