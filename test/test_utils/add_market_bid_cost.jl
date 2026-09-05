# WARNING: included in HydroPowerSimulations's tests as well.
# If you make changes, run those tests too!
"""
Add a MarketBidCost object to the selected components, with specified incremental and/or decremental cost curves.
"""
function add_mbc_inner!(
    sys::PSY.System,
    active_components::ComponentSelector;
    incr_curve::Union{Nothing, PiecewiseIncrementalCurve} = nothing,
    decr_curve::Union{Nothing, PiecewiseIncrementalCurve} = nothing,
)
    @assert !isempty(get_components(active_components, sys)) "No components selected"
    if isnothing(incr_curve) && isnothing(decr_curve)
        error("At least one of incr_curve or decr_curve must be provided")
    end
    mbc = MarketBidCost(;
        start_up = (hot = 0.0, warm = 0.0, cold = 0.0),
        shut_down = LinearCurve(0.0),
    )
    if !isnothing(decr_curve)
        set_decremental_offer_curves!(mbc, CostCurve(decr_curve))
    end
    if !isnothing(incr_curve)
        set_incremental_offer_curves!(mbc, CostCurve(incr_curve))
    end
    for comp in get_components(active_components, sys)
        set_operation_cost!(comp, mbc)
    end
end

"""
Add a MarketBidCost object to the selected components, with an incremental cost curve and/or
a decremental cost curve defined by hard-coded values.
"""
function add_mbc!(
    sys::PSY.System,
    active_components::ComponentSelector;
    incremental::Bool = true,
    decremental::Bool = false,
)
    incr_slopes = 100 .* [0.3, 0.5, 0.7]
    decr_slopes = 100 .* [0.7, 0.5, 0.3]
    x_coords = [10.0, 30.0, 50.0, 100.0]
    initial_input = 20.0

    if !incremental && !decremental
        error("At least one of incremental or decremental must be true")
    end
    if incremental
        incr_curve =
            PiecewiseIncrementalCurve(initial_input, x_coords, incr_slopes)
    else
        incr_curve = nothing
    end

    if decremental
        decr_curve =
            PiecewiseIncrementalCurve(initial_input, x_coords, decr_slopes)
    else
        decr_curve = nothing
    end
    add_mbc_inner!(sys, active_components; incr_curve = incr_curve, decr_curve = decr_curve)
end

#################################################################################
# psy6 split `MarketBidCost` (static) from `MarketBidTimeSeriesCost` (all five of
# minimum_energy_offer/start_up/shut_down/incremental_offer_curves/decremental_offer_curves
# are time-series-backed, with no static fallback on any one field). The helpers below
# convert a static `MarketBidCost` into a `MarketBidTimeSeriesCost`, wrapping whichever
# fields aren't given a `new_*` override in a same-valued ("constant") time series so the
# result type-checks. This is the one conversion point every MBC test-fixture builder in
# this file and mbc_system_utils.jl goes through.
#################################################################################

"Wrap a static `LinearCurve`'s current value in a same-valued time series."
function _constant_ts_linear_curve(sys::PSY.System, comp::PSY.Component, name::String,
    linear_curve::LinearCurve)
    fd = get_function_data(linear_curve)
    ts = make_deterministic_ts(sys, name, fd, 0.0, 0.0)
    return TimeSeriesLinearCurve(add_time_series!(sys, comp, ts))
end

"Wrap a static `StartUpStages`'s current value in a same-valued time series, returning the
bare `StartUpStagesKey` (the `start_up` field of `MarketBidTimeSeriesCost` is the key
itself, not a curve wrapper)."
function _constant_ts_start_up_key(sys::PSY.System, comp::PSY.Component,
    stages::PSY.StartUpStages)
    ts = make_deterministic_ts(sys, "start_up", Tuple(stages), 0.0, 0.0)
    return add_time_series!(sys, comp, ts)
end

"Wrap a static offer `CostCurve{PiecewiseIncrementalCurve}`'s current (constant) shape in a
same-valued time series."
function _constant_ts_offer_curve(sys::PSY.System, comp::PSY.Component, name::String,
    curve::CostCurve{PiecewiseIncrementalCurve})
    baseline = get_value_curve(curve)
    pwl_ts = make_deterministic_ts(
        sys,
        name,
        get_function_data(baseline),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )
    pwl_key = add_time_series!(sys, comp, pwl_ts)
    initial_input = get_initial_input(baseline)
    initial_key = if isnothing(initial_input)
        nothing
    else
        ii_ts = make_deterministic_ts(sys, name * "_initial_input", initial_input, 0.0, 0.0)
        add_time_series!(sys, comp, ii_ts)
    end
    return make_market_bid_ts_curve(pwl_key, initial_key, get_power_units(curve))
end

"""
Convert the static `MarketBidCost` `mbc` attached to `comp` into a `MarketBidTimeSeriesCost`.
Pass `new_*` to give a field a genuinely time-varying value (already built: a
`StartUpStagesKey`/`TimeSeriesLinearCurve`/TS-backed `CostCurve`); every field without a
`new_*` override keeps its current value, wrapped in a constant-valued time series.
"""
function to_market_bid_ts_cost(
    sys::PSY.System,
    comp::PSY.Component,
    mbc::MarketBidCost;
    new_minimum_energy_offer = nothing,
    new_start_up = nothing,
    new_shut_down = nothing,
    new_incremental_offer_curves = nothing,
    new_decremental_offer_curves = nothing,
)
    # `something(x, f(...))` evaluates `f(...)` eagerly even when `x` is already given,
    # which would add the "constant-wrap" time series (and, worse, error on a name clash)
    # even for fields the caller is already overriding with a genuinely time-varying value.
    # Branch explicitly instead so the fallback only runs when actually needed.
    meo = if isnothing(new_minimum_energy_offer)
        _constant_ts_linear_curve(sys, comp, "minimum_energy_offer",
            get_minimum_energy_offer(mbc))
    else
        new_minimum_energy_offer
    end
    start_up = if isnothing(new_start_up)
        _constant_ts_start_up_key(sys, comp, get_start_up(mbc))
    else
        new_start_up
    end
    shut_down = if isnothing(new_shut_down)
        _constant_ts_linear_curve(sys, comp, "shut_down", get_shut_down(mbc))
    else
        new_shut_down
    end
    incremental = if isnothing(new_incremental_offer_curves)
        _constant_ts_offer_curve(sys, comp, "incremental_offer_curves",
            get_incremental_offer_curves(mbc))
    else
        new_incremental_offer_curves
    end
    decremental = if isnothing(new_decremental_offer_curves)
        _constant_ts_offer_curve(sys, comp, "decremental_offer_curves",
            get_decremental_offer_curves(mbc))
    else
        new_decremental_offer_curves
    end
    return MarketBidTimeSeriesCost(;
        minimum_energy_offer = meo,
        start_up = start_up,
        shut_down = shut_down,
        incremental_offer_curves = incremental,
        decremental_offer_curves = decremental,
        ancillary_service_offers = get_ancillary_service_offers(mbc),
        incremental_slope = get_incremental_slope(mbc),
        decremental_slope = get_decremental_slope(mbc),
        curve_style = get_curve_style(mbc),
    )
end

"""
Extend the MarketBidCost objects attached to the selected components such that they're determined by a time series.

# Arguments:

  - `initial_varies`: whether the initial input time series should have values that vary
    over time (as opposed to a time series with constant values over time)
  - `breakpoints_vary`: whether the breakpoints in the variable cost time series should vary
    over time
  - `slopes_vary`: whether the slopes of the variable cost time series should vary over time
  - `active_components`: a `ComponentSelector` specifying which components should get time
    series
  - `initial_input_names_vary`: whether the initial input time series names should vary over
    components
  - `variable_cost_names_vary`: whether the variable cost time series names should vary over
    components
"""
function extend_mbc!(
    sys::PSY.System,
    active_components::ComponentSelector;
    modify_baseline_pwl = nothing,
    initial_varies::Bool = false,
    breakpoints_vary::Bool = false,
    slopes_vary::Bool = false,
    initial_input_names_vary::Bool = false,
    variable_cost_names_vary::Bool = false,
    zero_cost_at_min::Bool = false,
    create_extra_tranches::Bool = false,
    do_override_min_x::Bool = false,
)
    @assert !isempty(get_components(active_components, sys)) "No components selected"
    # incremental_initial_input is cost at minimum generation, NOT cost at zero generation
    for comp in get_components(active_components, sys)
        op_cost = get_operation_cost(comp)::MarketBidCost
        if do_override_min_x && :active_power_limits in fieldnames(typeof(comp))
            min_power = get_active_power_limits(comp, PSY.NU).min
        else
            min_power = nothing
        end

        new_curves = Dict{String, Any}()
        for (getter, incr_or_decr) in (
            (get_incremental_offer_curves, "incremental"),
            (get_decremental_offer_curves, "decremental"),
        )
            cost_curve = getter(op_cost)
            # psy6's `MarketBidCost.{in,de}cremental_offer_curves` are non-nullable,
            # defaulting to `PSY.ZERO_OFFER_CURVE` (a degenerate, zero-width curve) rather
            # than pre-psy6's `nothing` for "this side has no offer". Skip perturbing that
            # side here — `to_market_bid_ts_cost`'s own `new_*_offer_curves === nothing`
            # fallback below constant-wraps the (already zero) baseline unperturbed, so the
            # result still faithfully represents "no offer on this side".
            PSY._is_zero_offer_curve(cost_curve) && continue
            baseline = get_value_curve(cost_curve)::PiecewiseIncrementalCurve
            baseline_initial = get_initial_input(baseline)
            if zero_cost_at_min
                baseline_initial = 0.0
            end
            baseline_pwl = get_function_data(baseline)
            if do_override_min_x && isnothing(min_power)
                min_power = first(get_x_coords(baseline_pwl))
            end

            !isnothing(modify_baseline_pwl) &&
                (baseline_pwl = modify_baseline_pwl(baseline_pwl))
            # primes for easier attribution
            incr_initial = initial_varies ? (0.11, 0.05) : (0.0, 0.0)
            incr_x = breakpoints_vary ? (0.02, 0.07, 0.03) : (0.0, 0.0, 0.0)
            incr_y = slopes_vary ? (0.02, 0.07, 0.03) : (0.0, 0.0, 0.0)

            name_modifier = "_$(replace(get_name(comp), " " => "_"))_"

            initial_name =
                "initial_input $(incr_or_decr)" *
                (initial_input_names_vary ? name_modifier : "")
            my_initial_ts = make_deterministic_ts(
                sys,
                initial_name,
                baseline_initial,
                incr_initial...;
            )
            variable_name =
                "variable_cost $(incr_or_decr)" *
                (variable_cost_names_vary ? name_modifier : "")
            my_pwl_ts = make_deterministic_ts(
                sys,
                variable_name,
                baseline_pwl,
                incr_x,
                incr_y;
                create_extra_tranches = create_extra_tranches,
                override_min_x = do_override_min_x ? min_power : nothing,
            )
            initial_key = add_time_series!(sys, comp, my_initial_ts)
            curve_key = add_time_series!(sys, comp, my_pwl_ts)
            new_curves[incr_or_decr] =
                make_market_bid_ts_curve(
                    curve_key,
                    initial_key,
                    get_power_units(cost_curve),
                )
        end
        ts_cost = to_market_bid_ts_cost(
            sys,
            comp,
            op_cost;
            new_incremental_offer_curves = get(new_curves, "incremental", nothing),
            new_decremental_offer_curves = get(new_curves, "decremental", nothing),
        )
        set_operation_cost!(comp, ts_cost)
    end
    return
end

"""`Tuple` broadcasts elementwise against a single increment; `Number` and
`IS.LinearFunctionData` (which defines scalar `+` but is not a collection, so broadcasting it
errors) both add a single per-step increment directly."""
function _stepped_value(ini_val::Tuple, res_incr, interval_incr, j, i)
    return ini_val .+ (res_incr * j + i * interval_incr)
end

function _stepped_value(ini_val, res_incr, interval_incr, j, i)
    return ini_val + res_incr * j + i * interval_incr
end

"""
Make a deterministic time series from a tuple or a float value. See below function for
details about the arguments.
"""
function make_deterministic_ts(
    name::String,
    ini_val::T,
    res_incr::Number,
    interval_incr::Number,
    init_time::DateTime,
    horizon::Period,
    interval::Period,
    window_count::Int,
    resolution::Period,
) where {T <: Union{Number, Tuple, IS.LinearFunctionData}}
    horizon_count = IS.get_horizon_count(horizon, resolution)
    ts_data = OrderedDict{DateTime, Vector{T}}()
    for i in 0:(window_count - 1)
        series = [
            _stepped_value(ini_val, res_incr, interval_incr, j, i) for
            j in 0:(horizon_count - 1)
        ]
        ts_data[init_time + i * interval] = series
    end
    return Deterministic(;
        name = name,
        data = ts_data,
        resolution = resolution,
        interval = interval,
    )
end

"""
Create a deterministic time series with increments to the initial values, breakpoints, and slopes.
Here, the elements of `incrs_x` and `incrs_y` are tuples of three values, corresponding to:

`tranche_incr`: increment between tranche breakpoints.
`res_incr`: increment within the forecast horizon window.
`interval_incr`: increment in baseline, between horizon windows.

`override_min_x`: if provided, overrides the minimum x value in all piecewise curves.
`create_extra_tranches`: if true, split the first tranche of the first timestep into two;
split the last tranche of the last timestep of into three.
"""
function make_deterministic_ts(
    name::String,
    ini_val::PiecewiseStepData,
    incrs_x::NTuple{3, Float64},
    incrs_y::NTuple{3, Float64},
    init_time::DateTime,
    horizon::Period,
    interval::Period,
    count::Int,
    resolution::Period;
    override_min_x = nothing,
    override_max_x = nothing,
    create_extra_tranches = false,
)
    (tranche_incr_x, res_incr_x, interval_incr_x) = incrs_x
    (tranche_incr_y, res_incr_y, interval_incr_y) = incrs_y

    horizon_count = IS.get_horizon_count(horizon, resolution)

    # Perturb the baseline curves by the tranche increments
    xs1, ys1 = deepcopy(get_x_coords(ini_val)), deepcopy(get_y_coords(ini_val))
    xs1 .+= [i * tranche_incr_x for i in 0:(length(xs1) - 1)]
    ys1 .+= [i * tranche_incr_y for i in 0:(length(ys1) - 1)]

    ts_data = OrderedDict{DateTime, Vector{PiecewiseStepData}}()
    for i in 0:(count - 1)
        xs = [deepcopy(xs1) .+ i * interval_incr_x for _ in 1:horizon_count]
        ys = [deepcopy(ys1) .+ i * interval_incr_y for _ in 1:horizon_count]
        for j in 1:horizon_count
            xs[j] .+= (j - 1) * res_incr_x
            ys[j] .+= (j - 1) * res_incr_y
        end
        if !isnothing(override_min_x)
            for j in 1:horizon_count
                xs[j][1] = override_min_x
            end
        end
        if !isnothing(override_max_x)
            for j in 1:horizon_count
                xs[j][end] = override_max_x
            end
        end
        if i == 0 && create_extra_tranches
            xs[1] = [xs[1][1], (xs[1][1] + xs[1][2]) / 2, xs[1][2:end]...]
            ys[1] = [ys[1][1], ys[1][1], ys[1][2:end]...]
        elseif i == count - 1 && create_extra_tranches
            xs[end] = [
                xs[end][1:(end - 1)]...,
                (2 * xs[end][end - 1] + xs[end][end]) / 3,
                (xs[end][end - 1] + 2 * xs[end][end]) / 3,
                xs[end][end],
            ]
            ys[end] = [ys[end][1:(end - 1)]..., ys[end][end], ys[end][end], ys[end][end]]
        end
        ts_data[init_time + i * interval] = PiecewiseStepData.(xs, ys)
    end

    return Deterministic(;
        name = name,
        data = ts_data,
        resolution = resolution,
        interval = interval,
    )
end

"""
Create a deterministic time series as above, with the same horizon, count, and interval as an existing time series.
"""
function make_deterministic_ts(
    sys::PSY.System,
    args...;
    kwargs...,
)
    @assert all(
        PSY.get_time_series_resolutions(sys) .==
        first(PSY.get_time_series_resolutions(sys)),
    )
    return make_deterministic_ts(
        args...,
        first(PSY.get_forecast_initial_times(sys)),
        PSY.get_forecast_horizon(sys),
        PSY.get_forecast_interval(sys),
        PSY.get_forecast_window_count(sys),
        first(PSY.get_time_series_resolutions(sys));
        kwargs...,
    )
end
