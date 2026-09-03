# WARNING: included in HydroPowerSimulations's tests as well.
# If you make changes, run those tests too!
const SEL_INCR = make_selector(ThermalStandard, "Test Unit1")
const SEL_DECR = make_selector(InterruptiblePowerLoad, "Bus1_interruptible")
const SEL_MULTISTART = make_selector(ThermalMultiStart, "115_STEAM_1")

# functions for replacing components in the system
function replace_with_renewable!(
    sys::PSY.System,
    unit1::PSY.Generator;
    use_thermal_max_power = false,
    magnitude = 1.0,
    random_variation = 0.1,
)
    rg1 = PSY.RenewableDispatch(;
        name = "RG1",
        available = true,
        bus = get_bus(unit1),
        active_power = get_active_power(unit1, PSY.DU),
        reactive_power = get_reactive_power(unit1, PSY.DU),
        rating = get_rating(unit1, PSY.DU),
        prime_mover_type = PSY.PrimeMovers.PVe,
        reactive_power_limits = get_reactive_power_limits(unit1, PSY.DU),
        power_factor = 0.9,
        # the start up, shunt down, and no-load cost of renewables should be zero,
        # but we'll use the unit's operation cost as-is for simplicity.
        operation_cost = deepcopy(get_operation_cost(unit1)),
        base_power = get_base_power(unit1),
    )
    add_component!(sys, rg1)
    transfer_mbc!(rg1, unit1, sys)
    remove_component!(sys, unit1)
    zero_out_startup_shutdown_costs!(sys, rg1)

    # add a max_active_power time series to the component
    load = first(PSY.get_components(PSY.PowerLoad, sys))
    load_ts = get_time_series(Deterministic, load, "max_active_power")
    num_windows = length(get_data(load_ts))
    num_forecast_steps =
        floor(Int, IS.get_horizon(load_ts) / IS.get_interval(load_ts))
    total_steps = num_windows + num_forecast_steps - 1
    dates = range(
        get_initial_timestamp(load_ts);
        step = IS.get_interval(load_ts),
        length = total_steps,
    )
    if use_thermal_max_power
        rg_data = fill(get_active_power_limits(unit1, PSY.NU).max, total_steps)
    else
        rg_data = magnitude .* ones(total_steps) .+ random_variation .* rand(total_steps)
    end
    # Build a native `Deterministic` directly (windows keyed exactly like `load_ts`'s own,
    # each holding `num_forecast_steps` values sliced from `rg_data`) instead of adding a
    # `SingleTimeSeries` and calling `transform_single_time_series!`: that call produces a
    # `DeterministicSingleTimeSeries`, a distinct forecast type from the rest of this
    # system's (MarketBidCost-derived) native `Deterministic` time series, and
    # `IOM.get_deterministic_time_series_type` refuses a System mixing the two.
    rg_ts_data = OrderedDict{DateTime, Vector{Float64}}(
        dates[w] => rg_data[w:(w + num_forecast_steps - 1)] for w in 1:num_windows
    )
    rg_ts = Deterministic(;
        name = "max_active_power",
        data = rg_ts_data,
        resolution = IS.get_interval(load_ts),
        interval = IS.get_interval(load_ts),
    )
    add_time_series!(sys, rg1, rg_ts)
    return
end

function replace_load_with_interruptible!(sys::System)
    @assert !isempty(get_components(PSY.PowerLoad, sys))
    load1 = first(get_components(PSY.PowerLoad, sys))
    interruptible_load = PSY.InterruptiblePowerLoad(;
        name = get_name(load1) * "_interruptible",
        bus = get_bus(load1),
        available = get_available(load1),
        active_power = get_active_power(load1, PSY.DU),
        reactive_power = get_reactive_power(load1, PSY.DU),
        max_active_power = get_max_active_power(load1, PSY.DU),
        max_reactive_power = get_max_reactive_power(load1, PSY.DU),
        operation_cost = PSY.LoadCost(nothing),
        base_power = get_base_power(load1),
        conformity = get_conformity(load1),
    )
    add_component!(sys, interruptible_load)
    for md in IS.list_time_series_metadata(load1)
        ts = get_time_series(load1, IS.get_time_series_key(md))
        add_time_series!(
            sys,
            interruptible_load,
            ts,
        )
    end
    remove_component!(sys, load1)
end

# functions for adjusting power/cost curves and manipulating time series
"""
Helper function to tweak load powers, non-MBC generator powers, and non-MBC generator costs
to exercise the generators we want to test.

Multiplies {} for {} by {}:
- max active power, all loads, load_pow_mult
- active power limits, non-MBC ThermalStandard, therm_pow_mult
- operational costs, non-MBC ThermalStandard, therm_price_mult
"""
function tweak_system!(sys::System, load_pow_mult, therm_pow_mult, therm_price_mult)
    for load in get_components(PowerLoad, sys)
        set_max_active_power!(
            load,
            get_max_active_power(load, PSY.SU) * load_pow_mult * PSY.SU,
        )
    end
    # replace with type of component?
    for therm in get_components(ThermalStandard, sys)
        op_cost = get_operation_cost(therm)
        op_cost isa MarketBidCost && continue
        old_limits = get_active_power_limits(therm, PSY.DU)
        new_limits =
            (min = old_limits.min * PSY.DU, max = old_limits.max * therm_pow_mult * PSY.DU)
        set_active_power_limits!(therm, new_limits)
        if get_variable_operation_cost(op_cost) isa CostCurve{LinearCurve} ||
           get_variable_operation_cost(op_cost) isa CostCurve{QuadraticCurve}
            prop =
                get_proportional_term(get_value_curve(get_variable_operation_cost(op_cost)))
            set_variable_operation_cost!(
                op_cost,
                CostCurve(LinearCurve(prop * therm_price_mult)),
            )
        elseif get_variable_operation_cost(op_cost) isa CostCurve{PiecewiseIncrementalCurve}
            pwl = get_value_curve(get_variable_operation_cost(op_cost))
            new_pwl = PiecewiseIncrementalCurve(
                therm_price_mult * get_initial_input(pwl),
                get_x_coords(pwl),
                therm_price_mult * get_slopes(pwl),
            )
            set_variable_operation_cost!(op_cost, CostCurve(new_pwl))
        else
            error(
                "Unhandled operation cost variable type $(typeof(get_variable_operation_cost(op_cost)))",
            )
        end
    end
end

tweak_for_startup_shutdown!(sys::System) = tweak_system!(sys::System, 0.8, 1.0, 1.0)

tweak_for_decremental_initial!(sys::PSY.System) = tweak_system!(sys, 1.0, 1.2, 0.5)

"""Transfer the market bid cost from old_comp to new_comp, copying any time series in the
process. `old_comp`'s cost may be a static `MarketBidCost` (nothing to copy) or a
`MarketBidTimeSeriesCost` (every field is a time series reference that must be re-homed onto
`new_comp`)."""
function transfer_mbc!(
    new_comp::PSY.Device,
    old_comp::PSY.Device,
    new_sys::PSY.System,
)
    mbc = deepcopy(get_operation_cost(old_comp))
    set_operation_cost!(new_comp, _rehomed_mbc(mbc, old_comp, new_sys, new_comp))
    return
end

_rehomed_mbc(mbc::MarketBidCost, ::PSY.Device, ::PSY.System, ::PSY.Device) = mbc

function _rehomed_mbc(
    mbc::MarketBidTimeSeriesCost,
    old_comp::PSY.Device,
    new_sys::PSY.System,
    new_comp::PSY.Device,
)
    rehome_key(key::IS.TimeSeriesKey) =
        add_time_series!(new_sys, new_comp, deepcopy(PSY.get_time_series(old_comp, key)))
    return MarketBidTimeSeriesCost(;
        minimum_energy_offer = TimeSeriesLinearCurve(
            rehome_key(IS.get_time_series_key(get_minimum_energy_offer(mbc))),
        ),
        start_up = rehome_key(get_start_up(mbc)),
        shut_down = TimeSeriesLinearCurve(
            rehome_key(IS.get_time_series_key(get_shut_down(mbc))),
        ),
        incremental_offer_curves =
        _rehomed_offer_curve(get_incremental_offer_curves(mbc), rehome_key),
        decremental_offer_curves =
        _rehomed_offer_curve(get_decremental_offer_curves(mbc), rehome_key),
        ancillary_service_offers = get_ancillary_service_offers(mbc),
        incremental_slope = get_incremental_slope(mbc),
        decremental_slope = get_decremental_slope(mbc),
        curve_style = get_curve_style(mbc),
    )
end

function _rehomed_offer_curve(curve::CostCurve{<:TimeSeriesPiecewiseIncrementalCurve},
    rehome_key,
)
    vc = get_value_curve(curve)
    new_pwl_key = rehome_key(IS.get_time_series_key(vc))
    initial_key = IS.get_initial_input(vc)
    new_initial_key = isnothing(initial_key) ? nothing : rehome_key(initial_key)
    return make_market_bid_ts_curve(new_pwl_key, new_initial_key, get_power_units(curve))
end

zero_out_startup_shutdown_costs!(sys::PSY.System, comp::PSY.Device) =
    _zero_out_startup_shutdown_costs!(sys, comp, get_operation_cost(comp))

function _zero_out_startup_shutdown_costs!(
    ::PSY.System,
    ::PSY.Device,
    op_cost::MarketBidCost,
)
    set_start_up!(op_cost, (hot = 0.0, warm = 0.0, cold = 0.0))
    set_shut_down!(op_cost, LinearCurve(0.0))
    return
end

function _zero_out_startup_shutdown_costs!(
    sys::PSY.System,
    comp::PSY.Device,
    op_cost::MarketBidTimeSeriesCost,
)
    startup_ts = make_deterministic_ts(sys, "zeroed_start_up", (0.0, 0.0, 0.0), 0.0, 0.0)
    shutdown_ts =
        make_deterministic_ts(sys, "zeroed_shut_down", LinearFunctionData(0.0), 0.0, 0.0)
    set_start_up!(op_cost, add_time_series!(sys, comp, startup_ts))
    set_shut_down!(op_cost, TimeSeriesLinearCurve(add_time_series!(sys, comp, shutdown_ts)))
    return
end

"""
Move the `minimum_energy_offer` cost onto the incremental offer curve's `initial_input`,
zeroing `minimum_energy_offer` in exchange (`MEO = no-load cost / P_min`, so the flat dollar
figure baked into `initial_input` is `MEO * P_min`). Not designed for time series.
"""
function no_load_to_initial_input!(sys::PSY.System, comp::Generator)
    cost = get_operation_cost(comp)::MarketBidCost
    meo_rate = IS.get_proportional_term(get_function_data(get_minimum_energy_offer(cost)))
    p_min = get_active_power_limits(comp, PSY.NU).min
    old_fd = get_function_data(
        get_value_curve(get_incremental_offer_curves(cost)),
    )::IS.PiecewiseStepData
    new_vc = PiecewiseIncrementalCurve(old_fd, meo_rate * p_min, nothing)
    set_incremental_offer_curves!(cost, CostCurve(new_vc))
    set_minimum_energy_offer!(cost, LinearCurve(0.0))
    return
end

no_load_to_initial_input!(
    sys::PSY.System,
    sel = make_selector(x -> get_operation_cost(x) isa MarketBidCost, Generator),
) = foreach(comp -> no_load_to_initial_input!(sys, comp), get_components(sel, sys))

"Set all MBC thermal unit min active powers to their min breakpoints"
function adjust_min_power!(sys)
    for comp in get_components(Union{ThermalStandard, ThermalMultiStart}, sys)
        op_cost = get_operation_cost(comp)
        op_cost isa MarketBidCost || continue
        cost_curve = get_incremental_offer_curves(op_cost)::CostCurve
        baseline = get_value_curve(cost_curve)::PiecewiseIncrementalCurve
        x_coords = get_x_coords(get_function_data(baseline))
        set_active_power_limits!(
            comp,
            (min = first(x_coords) * PSY.MW, max = last(x_coords) * PSY.MW),
        )
    end
end

"""
Add startup and shutdown time series to a certain component. `with_increments`: whether the
elements should be increasing over time or constant. Version A: designed for
`c_fixed_market_bid_cost`.
"""
function add_startup_shutdown_ts_a!(sys::System, with_increments::Bool)
    res_incr = with_increments ? 0.05 : 0.0
    interval_incr = with_increments ? 0.01 : 0.0
    unit1 = get_component(ThermalStandard, sys, "Test Unit1")
    op_cost = get_operation_cost(unit1)::MarketBidCost
    startup_ts_1 = make_deterministic_ts(
        sys,
        "start_up",
        (1.0, 1.5, 2.0),
        res_incr,
        interval_incr,
    )
    shutdown_ts_1 =
        make_deterministic_ts(sys, "shut_down", LinearFunctionData(0.5), res_incr,
            interval_incr)
    startup_key = add_time_series!(sys, unit1, startup_ts_1)
    shutdown_key = add_time_series!(sys, unit1, shutdown_ts_1)
    ts_cost = to_market_bid_ts_cost(
        sys,
        unit1,
        op_cost;
        new_start_up = startup_key,
        new_shut_down = TimeSeriesLinearCurve(shutdown_key),
    )
    set_operation_cost!(unit1, ts_cost)
    return startup_ts_1, shutdown_ts_1
end

"""
Add startup and shutdown time series to a certain component. `with_increments`: whether the
elements should be increasing over time or constant. Version B: designed for `c_sys5_pglib`.
"""
function add_startup_shutdown_ts_b!(sys::System, with_increments::Bool)
    res_incr = with_increments ? 0.05 : 0.0
    interval_incr = with_increments ? 0.01 : 0.0
    unit1 = get_component(ThermalMultiStart, sys, "115_STEAM_1")
    op_cost = get_operation_cost(unit1)::MarketBidCost
    base_startup = Tuple(get_start_up(op_cost))
    base_shutdown = get_function_data(get_shut_down(op_cost))
    startup_ts_1 = make_deterministic_ts(
        sys,
        "start_up",
        base_startup,
        res_incr,
        interval_incr,
    )
    shutdown_ts_1 =
        make_deterministic_ts(
            sys,
            "shut_down",
            base_shutdown,
            res_incr,
            interval_incr,
        )
    startup_key = add_time_series!(sys, unit1, startup_ts_1)
    shutdown_key = add_time_series!(sys, unit1, shutdown_ts_1)
    ts_cost = to_market_bid_ts_cost(
        sys,
        unit1,
        op_cost;
        new_start_up = startup_key,
        new_shut_down = TimeSeriesLinearCurve(shutdown_key),
    )
    set_operation_cost!(unit1, ts_cost)
    return startup_ts_1, shutdown_ts_1
end

# functions for building the systems: calls the above

function load_and_fix_system(args...; kwargs...)
    sys = Logging.with_logger(Logging.NullLogger()) do
        build_system(args...; kwargs...)
    end
    no_load_to_initial_input!(sys)
    adjust_min_power!(sys)
    return sys
end

"""Create a system with for testing fixed market bid costs on thermal get_components."""
function load_sys_incr()
    # NOTE we are using the fixed one so we can add time series ourselves
    sys = load_and_fix_system(
        PSITestSystems,
        "c_fixed_market_bid_cost",
    )
    tweak_system!(sys, 1.05, 1.0, 1.0)
    get_y_coords(
        get_function_data(
            get_value_curve(
                get_incremental_offer_curves(
                    get_operation_cost(get_component(ThermalStandard, sys, "Test Unit2")),
                ),
            ),
        ),
    )[1] *= 0.9
    # Give Test Unit1 a device base_power != system base_power, so unit-conversion bugs
    # (e.g. rating-dependent per-unit multipliers) aren't masked by the two coinciding.
    unit1 = get_component(SEL_INCR, sys)
    limits = get_active_power_limits(unit1, PSY.NU)
    rating = get_rating(unit1, PSY.NU)
    active_power = get_active_power(unit1, PSY.NU)
    set_base_power!(unit1, get_base_power(sys) * 1.4)
    set_active_power_limits!(
        unit1,
        (min = limits.min * PSY.MW, max = limits.max * PSY.MW),
    )
    set_rating!(unit1, rating * PSY.MW)
    set_active_power!(unit1, active_power * PSY.MW)
    return sys
end

"""
Create a system with initial input and variable cost time series. Lots of options:

# Arguments:
  - `initial_varies`: whether the initial input time series should have values that vary
    over time (as opposed to a time series with constant values over time)
  - `breakpoints_vary`: whether the breakpoints in the variable cost time series should vary
    over time
  - `slopes_vary`: whether the slopes of the variable cost time series should vary over time
  - `modify_baseline_pwl`: optional, a function to modify the baseline piecewise linear cost
    `FunctionData` from which the variable cost time series is calculated
  - `do_override_min_x`: whether to override the P1 to be equal to the minimum power in all
    time steps
  - `create_extra_tranches`: whether to create extra tranches in some time steps by
    splitting one tranche into two
  - `active_components`: a `ComponentSelector` specifying which components should get time
    series
  - `initial_input_names_vary`: whether the initial input time series names should vary over
    components
  - `variable_cost_names_vary`: whether the variable cost time series names should vary over
    components
"""
function build_sys_incr(
    initial_varies::Bool,
    breakpoints_vary::Bool,
    slopes_vary::Bool;
    modify_baseline_pwl = nothing,
    do_override_min_x = true,
    create_extra_tranches = false,
    active_components = SEL_INCR,
    initial_input_names_vary = false,
    variable_cost_names_vary = false,
)
    sys = load_sys_incr()
    @assert !isempty(get_components(active_components, sys)) "No components selected"
    extend_mbc!(
        sys,
        active_components;
        initial_varies = initial_varies,
        breakpoints_vary = breakpoints_vary,
        slopes_vary = slopes_vary,
        modify_baseline_pwl = modify_baseline_pwl,
        do_override_min_x = do_override_min_x,
        create_extra_tranches = create_extra_tranches,
        initial_input_names_vary = initial_input_names_vary,
        variable_cost_names_vary = variable_cost_names_vary,
    )
    return sys
end

function remove_thermal_mbcs!(sys::PSY.System)
    for comp in get_components(ThermalStandard, sys)
        old_cost = get_operation_cost(comp)
        old_cost isa MarketBidCost || continue
        new_op_cost = ThermalGenerationCost(;
            variable_operation_cost = get_incremental_offer_curves(old_cost),
            start_up = get_start_up(old_cost),
            shut_down = IS.get_proportional_term(
                get_function_data(get_shut_down(old_cost)),
            ),
            fixed = 0.0,
        )
        set_operation_cost!(comp, new_op_cost)
    end
end

function zero_out_thermal_costs!(sys)
    for comp in get_components(ThermalStandard, sys)
        set_operation_cost!(
            comp,
            ThermalGenerationCost(;
                variable_operation_cost = CostCurve(
                    LinearCurve(0.0),
                ),
                start_up = (hot = 0.0, warm = 0.0, cold = 0.0),
                shut_down = 0.0,
                fixed = 0.0,
            ),
        )
    end
end

"""Like `load_sys_incr` but for decremental MarketBidCost on ControllableLoad components."""
function load_sys_decr2()
    sys = load_and_fix_system(
        PSITestSystems,
        "c_fixed_market_bid_cost",
    )
    replace_load_with_interruptible!(sys)
    interruptible_load = first(get_components(PSY.InterruptiblePowerLoad, sys))
    selector = make_selector(PSY.InterruptiblePowerLoad, get_name(interruptible_load))
    add_mbc!(sys, selector; incremental = false, decremental = true)
    # replace the MBCs on the thermals with ThermalCost objects.
    remove_thermal_mbcs!(sys)
    # makes the objective function/constraints simpler, easier to track down issues,
    # but not actually needed.
    zero_out_thermal_costs!(sys)
    return sys
end

"""Like `build_sys_incr` but for decremental MarketBidCost on ControllableLoad components."""
function build_sys_decr2(
    initial_varies::Bool,
    breakpoints_vary::Bool,
    slopes_vary::Bool;
    modify_baseline_pwl = nothing,
    do_override_min_x = true,
    create_extra_tranches = false,
    active_components = SEL_DECR,
    initial_input_names_vary = false,
    variable_cost_names_vary = false,
)
    sys = load_sys_decr2()
    @assert !isempty(get_components(active_components, sys)) "No components selected"
    extend_mbc!(
        sys,
        active_components;
        initial_varies = initial_varies,
        breakpoints_vary = breakpoints_vary,
        slopes_vary = slopes_vary,
        modify_baseline_pwl = modify_baseline_pwl,
        do_override_min_x = do_override_min_x,
        create_extra_tranches = create_extra_tranches,
        initial_input_names_vary = initial_input_names_vary,
        variable_cost_names_vary = variable_cost_names_vary,
    )

    # make the max_active_power time series constant.
    il = first(get_components(PSY.InterruptiblePowerLoad, sys))
    for md in IS.list_time_series_metadata(il)
        if get_name(md) == "max_active_power"
            max_active_power_ts = get_time_series(
                first(get_components(PSY.InterruptiblePowerLoad, sys)),
                IS.get_time_series_key(md),
            )
            max_max_active_power = maximum(maximum(values(max_active_power_ts.data)))
            remove_time_series!(sys, Deterministic, il, "max_active_power")
            new_ts = make_deterministic_ts(
                sys,
                "max_active_power",
                max_max_active_power,
                0.0,
                0.0,
            )
            add_time_series!(sys, il, new_ts)
            break
        end
    end
    return sys
end

function create_multistart_sys(
    with_increments::Bool,
    load_pow_mult,
    therm_pow_mult,
    therm_price_mult;
    add_ts = true,
)
    @assert add_ts || !with_increments
    c_sys5_pglib = load_and_fix_system(PSITestSystems, "c_sys5_pglib")
    tweak_system!(c_sys5_pglib, load_pow_mult, therm_pow_mult, therm_price_mult)
    ms_comp = get_component(SEL_MULTISTART, c_sys5_pglib)
    old_op = get_operation_cost(ms_comp)
    old_ic = IncrementalCurve(get_value_curve(get_variable_operation_cost(old_op)))
    new_ii = get_initial_input(old_ic) + get_fixed(old_op)
    new_ic = IncrementalCurve(get_function_data(old_ic), new_ii, nothing)
    set_operation_cost!(
        ms_comp,
        MarketBidCost(;
            start_up = (hot = 300.0, warm = 450.0, cold = 500.0),
            shut_down = LinearCurve(100.0),
            incremental_offer_curves = CostCurve(new_ic),
        ),
    )

    add_ts && add_startup_shutdown_ts_b!(c_sys5_pglib, with_increments)
    return c_sys5_pglib
end
