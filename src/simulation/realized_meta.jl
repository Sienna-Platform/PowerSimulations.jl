struct RealizedMeta
    start_time::Dates.DateTime
    resolution::Dates.TimePeriod
    len::Int
    start_offset::Int
    end_offset::Int
    interval_len::Int
    realized_timestamps::AbstractVector{Dates.DateTime}
end

function RealizedMeta(
    res::SimulationProblemResults;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
)
    existing_timestamps = get_timestamps(res)
    interval = existing_timestamps.step
    resolution = get_resolution(res)
    interval_len = Int(interval / resolution)
    realized_timestamps = get_realized_timestamps(res; start_time = start_time, len = len)

    result_start_time = existing_timestamps[findlast(
        x -> x .<= first(realized_timestamps),
        existing_timestamps,
    )]
    result_end_time = existing_timestamps[findlast(
        x -> x .<= last(realized_timestamps),
        existing_timestamps,
    )]

    len = length(result_start_time:interval:result_end_time)

    start_offset = length(result_start_time:resolution:first(realized_timestamps))
    end_offset = length(
        (last(realized_timestamps) + resolution):resolution:(result_end_time + interval - resolution),
    )

    return RealizedMeta(
        result_start_time,
        resolution,
        len,
        start_offset,
        end_offset,
        interval_len,
        realized_timestamps,
    )
end

function _make_dataframe(
    results_by_time::OutputsByTime{DataFrame, N},
    num_timestamps::Int,
    meta::RealizedMeta,
    key::OptimizationContainerKey,
    ::Val{TableFormat.LONG},
) where {N}
    @assert !isempty(results_by_time)
    row_index = 1
    dfs = DataFrame[]
    first_cols = names(first(values(results_by_time.data)))
    for (step, (_, df)) in enumerate(results_by_time)
        if step > 1 && names(df) != first_cols
            error("Mismatched columns. First df = $(first_cols), other df = $(names(df))")
        end
        first_id = step > 1 ? 1 : meta.start_offset
        last_id =
            step == meta.len ? meta.interval_len - meta.end_offset : meta.interval_len
        if last_id - first_id > DataFrames.nrow(df)
            error(
                "Variable $(encode_key_as_string(key)) has $(DataFrames.nrow(df)) number of steps, that is different than the default problem horizon. \
            Can't calculate the realized variables. Use `read_variables` instead and write your own concatenation",
            )
        end
        offset = (step - 1) * meta.interval_len
        df2 = @chain df begin
            @subset(first_id .<= :time_index .<= last_id)
            @transform(:actual_time_index = :time_index .+ offset)
            @select(Not(:time_index))
            @rename(:time_index = :actual_time_index)
        end
        push!(dfs, df2)
        row_index += last_id - first_id + 1
    end

    combined_df = vcat(dfs...)
    time_df = DataFrame(;
        DateTime = meta.realized_timestamps,
        time_index = (meta.start_offset):(meta.start_offset + length(
            meta.realized_timestamps,
        ) - 1),
    )
    result_df = @chain begin
        innerjoin(combined_df, time_df; on = :time_index)
        @select(:DateTime, Not(:DateTime, :time_index))
        @orderby(:DateTime)
    end

    actual_num_timestamps = length(unique(result_df.DateTime))
    if actual_num_timestamps != num_timestamps
        error(
            "Mismatched number of timestamps. Expected $(num_timestamps), got $actual_num_timestamps",
        )
    end

    return result_df
end

function _make_dataframe(
    results_by_time::OutputsByTime{DataFrame, N},
    num_timestamps::Int,
    meta::RealizedMeta,
    key::OptimizationContainerKey,
    ::Val{TableFormat.WIDE},
) where {N}
    @assert !isempty(results_by_time)
    row_index = 1
    dfs = DataFrame[]
    first_cols = names(first(values(results_by_time.data)))
    for (step, (_, df)) in enumerate(results_by_time)
        if step > 1 && names(df) != first_cols
            error("Mismatched columns. First df = $(first_cols), other df = $(names(df))")
        end
        first_id = step > 1 ? 1 : meta.start_offset
        last_id =
            step == meta.len ? meta.interval_len - meta.end_offset : meta.interval_len
        if last_id - first_id > DataFrames.nrow(df)
            error(
                "Variable $(encode_key_as_string(key)) has $(DataFrames.nrow(df)) number of steps, that is different than the default problem horizon. \
            Can't calculate the realized variables. Use `read_variables` instead and write your own concatenation",
            )
        end
        df2 = df[first_id:last_id, :]
        push!(dfs, df2)
        row_index += last_id - first_id + 1
    end

    df = vcat(dfs...)
    DataFrames.insertcols!(
        df,
        1,
        :DateTime => meta.realized_timestamps,
    )
    DataFrames.select!(df, DataFrames.Not(:time_index))
    if DataFrames.nrow(df) != num_timestamps
        error(
            "Mismatched number of rows. Expected $(num_timestamps), got $(DataFrames.nrow(df))",
        )
    end

    return df
end

function get_realization(
    results::Dict{OptimizationContainerKey, OutputsByTime{DataFrame}},
    meta::RealizedMeta;
    table_format = TableFormat.LONG,
)
    realized_values = Dict{OptimizationContainerKey, DataFrames.DataFrame}()
    lk = ReentrantLock()
    num_timestamps = length(meta.realized_timestamps)
    start = time()
    Threads.@threads for key in collect(keys(results))
        results_by_time = results[key]
        df = _make_dataframe(
            results_by_time,
            num_timestamps,
            meta,
            key,
            Val(table_format),
        )
        lock(lk) do
            realized_values[key] = df
        end
    end

    duration = time() - start
    if Threads.nthreads() == 1 && duration > 10.0
        @info "Time to read results: $duration seconds. You will likely get faster " *
              "results by starting Julia with multiple threads."
    end
    return realized_values
end
