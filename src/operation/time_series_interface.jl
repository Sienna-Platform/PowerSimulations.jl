function get_time_series_values!(
    time_series_type::Type{T},
    model::DecisionModel,
    component,
    name::String,
    initial_time::Dates.DateTime,
    horizon::Int;
    ignore_scaling_factors = true,
    interval::Union{Nothing, Dates.Period} = nothing,
) where {T <: PSY.Forecast}
    if !use_time_series_cache(get_settings(model))
        return IS.get_time_series_values(
            T,
            component,
            name;
            start_time = initial_time,
            len = horizon,
            ignore_scaling_factors = ignore_scaling_factors,
            interval = interval,
        )
    end

    cache = get_time_series_cache(model)
    key = IS.TimeSeriesCacheKey(IS.get_uuid(component), T, name)
    if haskey(cache, key)
        ts_cache = cache[key]
    else
        ts_cache = IS.make_time_series_cache(
            time_series_type,
            component,
            name,
            initial_time,
            horizon;
            ignore_scaling_factors = ignore_scaling_factors,
            interval = interval,
        )
        cache[key] = ts_cache
    end

    ts = IS.get_time_series_array!(ts_cache, initial_time)
    return TimeSeries.values(ts)
end

function get_time_series_values!(
    ::Type{T},
    model::EmulationModel,
    component::U,
    name::String,
    initial_time::Dates.DateTime,
    len::Int = 1;
    ignore_scaling_factors = true,
) where {T <: PSY.StaticTimeSeries, U <: PSY.Component}
    if !use_time_series_cache(get_settings(model))
        return IS.get_time_series_values(
            T,
            component,
            name;
            start_time = initial_time,
            len = len,
            ignore_scaling_factors = ignore_scaling_factors,
        )
    end

    cache = get_time_series_cache(model)
    key = IS.TimeSeriesCacheKey(IS.get_uuid(component), T, name)
    if haskey(cache, key)
        ts_cache = cache[key]
    else
        ts_cache = IS.make_time_series_cache(
            T,
            component,
            name,
            initial_time,
            len;
            ignore_scaling_factors = ignore_scaling_factors,
        )
        cache[key] = ts_cache
    end

    ts = IS.get_time_series_array!(ts_cache, initial_time)
    return TimeSeries.values(ts)
end
