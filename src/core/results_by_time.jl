mutable struct ResultsByTime{T, N}
    key::OptimizationContainerKey
    data::SortedDict{Dates.DateTime, T}
    resolution::Dates.Period
    column_names::NTuple{N, Vector{String}}
end

function ResultsByTime(
    key::OptimizationContainerKey,
    data::SortedDict{Dates.DateTime, T},
    resolution::Dates.Period,
    column_names,
) where {T}
    _check_column_consistency(data, column_names)
    ResultsByTime(key, data, resolution, column_names)
end

function _check_column_consistency(
    data::SortedDict{Dates.DateTime, DenseAxisArray{Float64, 2}},
    cols::Tuple{Vector{String}},
)
    for val in values(data)
        if axes(val)[1] != cols[1]
            error("Mismatch in DenseAxisArray column names: $(axes(val)[1]) $cols")
        end
    end
end

function _check_column_consistency(
    data::SortedDict{Dates.DateTime, Matrix{Float64}},
    cols::Tuple{Vector{String}},
)
    for val in values(data)
        if size(val)[2] != length(cols[1])
            error(
                "Mismatch in length of Matrix columns: $(size(val)[2]) $(length(cols[1]))",
            )
        end
    end
end

function _check_column_consistency(
    data::SortedDict{Dates.DateTime, DenseAxisArray{Float64, 2}},
    cols::NTuple{N, Vector{String}},
) where {N}
    # TODO:
end

function _check_column_consistency(
    data::SortedDict{Dates.DateTime, DataFrame},
    cols::NTuple{N, Vector{String}},
) where {N}
    for df in values(data)
        if DataFrames.ncol(df) != length(cols[1])
            error(
                "Mismatch in length of DataFrame columns: $(DataFrames.ncol(df)) $(length(cols[1]))",
            )
        end
    end
end

# TODO: Implement consistency check for other sizes

# This struct behaves like a dict, delegating to its 'data' field.
Base.length(res::ResultsByTime) = length(res.data)
Base.iterate(res::ResultsByTime) = iterate(res.data)
Base.iterate(res::ResultsByTime, state) = iterate(res.data, state)
Base.getindex(res::ResultsByTime, i) = getindex(res.data, i)
Base.setindex!(res::ResultsByTime, v, i) = setindex!(res.data, v, i)
Base.firstindex(res::ResultsByTime) = firstindex(res.data)
Base.lastindex(res::ResultsByTime) = lastindex(res.data)

get_column_names(x::ResultsByTime) = x.column_names
get_num_rows(::ResultsByTime{DenseAxisArray{Float64, 2}}, data) = size(data, 2)
get_num_rows(::ResultsByTime{DenseAxisArray{Float64, 3}}, data) = size(data, 3)
get_num_rows(::ResultsByTime{Matrix{Float64}}, data) = size(data, 1)
get_num_rows(::ResultsByTime{DataFrame}, data) = DataFrames.nrow(data)

function _add_timestamps!(
    df::DataFrames.DataFrame,
    results::ResultsByTime,
    timestamp::Dates.DateTime,
    data,
)
    time_col = _get_timestamps(results, timestamp, get_num_rows(results, data))
    if !isnothing(time_col)
        DataFrames.insertcols!(df, 1, :DateTime => time_col)
    end
    return
end

function _get_timestamps(results::ResultsByTime, timestamp::Dates.DateTime, len::Int)
    if results.resolution == Dates.Period(Dates.Millisecond(0))
        return
    end
    return range(timestamp; length = len, step = results.resolution)
end

function make_dataframe(
    results::ResultsByTime{DenseAxisArray{Float64, 2}},
    timestamp::Dates.DateTime;
    table_format::TableFormat = TableFormat.LONG,
)
    array = results.data[timestamp]
    timestamps = _get_timestamps(results, timestamp, get_num_rows(results, array))
    return to_results_dataframe(array, timestamps, Val(table_format))
end

function make_dataframe(
    results::ResultsByTime{DenseAxisArray{Float64, 3}},
    timestamp::Dates.DateTime;
    table_format::TableFormat = TableFormat.LONG,
)
    array = results.data[timestamp]
    num_timestamps = get_num_rows(results, array)
    timestamps = _get_timestamps(results, timestamp, num_timestamps)
    return to_results_dataframe(array, timestamps, Val(table_format))
end

function make_dataframe(
    results::ResultsByTime{Matrix{Float64}},
    timestamp::Dates.DateTime;
    table_format::TableFormat = TableFormat.LONG,
)
    array = results.data[timestamp]
    df_wide = DataFrames.DataFrame(array, results.column_names)
    _add_timestamps!(df_wide, results, timestamp, array)
    return if table_format == TableFormat.LONG
        measure_vars = [x for x in names(df_wide) if x != "DateTime"]
        DataFrames.stack(
            df_wide,
            measure_vars;
            variable_name = :name,
            value_name = :value,
        )
    elseif table_format == TableFormat.WIDE
        df_wide
    else
        error("Unsupported table format: $table_format")
    end
end

function make_dataframes(results::ResultsByTime; table_format::TableFormat = table_format)
    return SortedDict(
        k => make_dataframe(results, k; table_format = table_format) for
        k in keys(results.data)
    )
end

struct ResultsByKeyAndTime
    "Contains all keys stored in the model."
    result_keys::Vector{OptimizationContainerKey}
    "Contains the results that have been read from the store and cached."
    cached_results::Dict{OptimizationContainerKey, ResultsByTime}
end

ResultsByKeyAndTime(result_keys) = ResultsByKeyAndTime(
    collect(result_keys),
    Dict{OptimizationContainerKey, ResultsByTime}(),
)

Base.empty!(res::ResultsByKeyAndTime) = empty!(res.cached_results)
