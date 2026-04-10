function _get_time_series(
    container::OptimizationContainer,
    component::PSY.Component,
    attributes::TimeSeriesAttributes{T};
    interval::Union{Nothing, Dates.Period} = nothing,
) where {T <: PSY.TimeSeriesData}
    return get_time_series_initial_values!(
        container,
        T,
        component,
        get_time_series_name(attributes);
        interval = interval,
    )
end

function get_time_series(
    container::OptimizationContainer,
    component::T,
    parameter::TimeSeriesParameter,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META;
    interval::Union{Nothing, Dates.Period} = nothing,
) where {T <: PSY.Component}
    parameter_container = get_parameter(container, parameter, T, meta)
    return _get_time_series(
        container,
        component,
        parameter_container.attributes;
        interval = interval,
    )
end

# This is just for temporary compatibility with current code. Needs to be eliminated once the time series
# refactor is done.
function get_time_series(
    container::OptimizationContainer,
    component::PSY.Component,
    forecast_name::String;
    interval::Union{Nothing, Dates.Period} = nothing,
)
    ts_type = get_default_time_series_type(container)
    return _get_time_series(
        container,
        component,
        TimeSeriesAttributes(ts_type, forecast_name);
        interval = interval,
    )
end
