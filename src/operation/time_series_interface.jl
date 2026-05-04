function get_time_series_values!(
    time_series_type::Type{T},
    model::DecisionModel,
    component,
    name::String,
    initial_time::Dates.DateTime,
    horizon::Int;
    ignore_scaling_factors = true,
    interval::Dates.Millisecond = UNSET_INTERVAL,
) where {T <: PSY.Forecast}
    is_interval = _to_is_interval(interval)
    settings = get_settings(model)
    resolution = get_resolution(settings)
    if !use_time_series_cache(settings)
        return IS.get_time_series_values(
            T,
            component,
            name;
            start_time = initial_time,
            len = horizon,
            ignore_scaling_factors = ignore_scaling_factors,
            interval = is_interval,
        )
    end

    cache = get_time_series_cache(model)
    key = IS.TimeSeriesCacheKey(IS.get_uuid(component), T, name, resolution, is_interval)
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
            interval = is_interval,
            resolution = resolution,
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
    resolution::Dates.Millisecond = UNSET_RESOLUTION,
) where {T <: PSY.StaticTimeSeries, U <: PSY.Component}
    settings = get_settings(model)
    key_resolution =
        resolution == UNSET_RESOLUTION ? get_resolution(settings) : resolution
    is_resolution = _to_is_resolution(key_resolution)
    if !use_time_series_cache(settings)
        return IS.get_time_series_values(
            T,
            component,
            name;
            start_time = initial_time,
            len = len,
            ignore_scaling_factors = ignore_scaling_factors,
            resolution = is_resolution,
        )
    end

    cache = get_time_series_cache(model)
    key = IS.TimeSeriesCacheKey(IS.get_uuid(component), T, name, key_resolution, nothing)
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
            resolution = is_resolution,
        )
        cache[key] = ts_cache
    end

    ts = IS.get_time_series_array!(ts_cache, initial_time)
    return TimeSeries.values(ts)
end
