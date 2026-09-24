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
    res::SimulationProblemOutputs;
    start_time::Union{Nothing, Dates.DateTime} = nothing,
    len::Union{Int, Nothing} = nothing,
)
    existing_timestamps = get_timestamps(res)
    interval = existing_timestamps.step
    resolution = get_resolution(res)
    interval_len = Int(interval / resolution)
    realized_timestamps = get_realized_timestamps(res; start_time = start_time, len = len)

    output_start_time = existing_timestamps[findlast(
        x -> x .<= first(realized_timestamps),
        existing_timestamps,
    )]
    output_end_time = existing_timestamps[findlast(
        x -> x .<= last(realized_timestamps),
        existing_timestamps,
    )]

    len = length(output_start_time:interval:output_end_time)

    start_offset = length(output_start_time:resolution:first(realized_timestamps))
    end_offset = length(
        (last(realized_timestamps) + resolution):resolution:(output_end_time + interval - resolution),
    )

    return RealizedMeta(
        output_start_time,
        resolution,
        len,
        start_offset,
        end_offset,
        interval_len,
        realized_timestamps,
    )
end

# Row window of one step's DataFrame that belongs to the realized output: the first step
# starts at the offset, the last step stops short by the end offset.
function _realized_step_bounds(step::Int, meta::RealizedMeta, df::DataFrame, key)
    if step > 1
        first_id = 1
    else
        first_id = meta.start_offset
    end
    if step == meta.len
        last_id = meta.interval_len - meta.end_offset
    else
        last_id = meta.interval_len
    end
    if last_id - first_id > DataFrames.nrow(df)
        error(
            "Variable $(encode_key_as_string(key)) has $(DataFrames.nrow(df)) number of steps, that is different than the default problem horizon. \
        Can't calculate the realized variables. Use `read_variables` instead and write your own concatenation",
        )
    end
    return first_id, last_id
end

function _make_dataframe(
    outputs_by_time::OutputsByTime{DataFrame, N},
    num_timestamps::Int,
    meta::RealizedMeta,
    key::OptimizationContainerKey,
    ::Val{TableFormat.LONG},
) where {N}
    @assert !isempty(outputs_by_time)
    dfs = DataFrame[]
    first_cols = names(first(values(outputs_by_time.data)))
    for (step, (_, df)) in enumerate(outputs_by_time)
        if step > 1 && names(df) != first_cols
            error("Mismatched columns. First df = $(first_cols), other df = $(names(df))")
        end
        first_id, last_id = _realized_step_bounds(step, meta, df, key)
        offset = (step - 1) * meta.interval_len
        df2 = @chain df begin
            @subset(first_id .<= :time_index .<= last_id)
            @transform(:actual_time_index = :time_index .+ offset)
            @select(Not(:time_index))
            @rename(:time_index = :actual_time_index)
        end
        push!(dfs, df2)
    end

    combined_df = vcat(dfs...)
    time_df = DataFrame(;
        DateTime = meta.realized_timestamps,
        time_index = (meta.start_offset):(meta.start_offset + length(
            meta.realized_timestamps,
        ) - 1),
    )
    output_df = @chain begin
        innerjoin(combined_df, time_df; on = :time_index)
        @select(:DateTime, Not(:DateTime, :time_index))
        @orderby(:DateTime)
    end

    actual_num_timestamps = length(unique(output_df.DateTime))
    if actual_num_timestamps != num_timestamps
        error(
            "Mismatched number of timestamps. Expected $(num_timestamps), got $actual_num_timestamps",
        )
    end

    return output_df
end

function _make_dataframe(
    outputs_by_time::OutputsByTime{DataFrame, N},
    num_timestamps::Int,
    meta::RealizedMeta,
    key::OptimizationContainerKey,
    ::Val{TableFormat.WIDE},
) where {N}
    @assert !isempty(outputs_by_time)
    dfs = DataFrame[]
    first_cols = names(first(values(outputs_by_time.data)))
    for (step, (_, df)) in enumerate(outputs_by_time)
        if step > 1 && names(df) != first_cols
            error("Mismatched columns. First df = $(first_cols), other df = $(names(df))")
        end
        first_id, last_id = _realized_step_bounds(step, meta, df, key)
        df2 = df[first_id:last_id, :]
        push!(dfs, df2)
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
    outputs::Dict{OptimizationContainerKey, OutputsByTime{DataFrame}},
    meta::RealizedMeta;
    table_format = TableFormat.LONG,
)
    realized_values = Dict{OptimizationContainerKey, DataFrames.DataFrame}()
    lk = ReentrantLock()
    num_timestamps = length(meta.realized_timestamps)
    start = time()
    Threads.@threads for key in collect(keys(outputs))
        outputs_by_time = outputs[key]
        df = _make_dataframe(
            outputs_by_time,
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
        @info "Time to read outputs: $duration seconds. You will likely get faster " *
              "reads by starting Julia with multiple threads."
    end
    return realized_values
end
