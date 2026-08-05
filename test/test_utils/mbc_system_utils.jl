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
        active_power = get_active_power(unit1),
        reactive_power = get_reactive_power(unit1),
        rating = get_rating(unit1),
        prime_mover_type = PSY.PrimeMovers.PVe,
        reactive_power_limits = get_reactive_power_limits(unit1),
        power_factor = 0.9,
        # the start up, shunt down, and no-load cost of renewables should be zero,
        # but we'll use the unit's operation cost as-is for simplicity.
        operation_cost = deepcopy(get_operation_cost(unit1)),
        base_power = get_base_power(unit1),
    )
    add_component!(sys, rg1)
    transfer_mbc!(rg1, unit1, sys)
    remove_component!(sys, unit1)
    zero_out_startup_shutdown_costs!(rg1)

    # add a max_active_power time series to the component
    load = first(PSY.get_components(PSY.PowerLoad, sys))
    load_ts = get_time_series(Deterministic, load, "max_active_power")
    num_windows = length(get_data(load_ts))
    num_forecast_steps =
        floor(Int, get_horizon(load_ts) / get_interval(load_ts))
    total_steps = num_windows + num_forecast_steps - 1
    dates = range(
        get_initial_timestamp(load_ts);
        step = get_interval(load_ts),
        length = total_steps,
    )
    if use_thermal_max_power
        rg_data = fill(get_active_power_limits(unit1).max, total_steps)
    else
        rg_data = magnitude .* ones(total_steps) .+ random_variation .* rand(total_steps)
    end
    rg_ts = SingleTimeSeries("max_active_power", TimeArray(dates, rg_data))
    add_time_series!(sys, rg1, rg_ts)
    transform_single_time_series!(
        sys,
        get_horizon(load_ts),
        get_interval(load_ts),
    )
end

function replace_load_with_interruptible!(sys::System)
    @assert !isempty(get_components(PSY.PowerLoad, sys))
    load1 = first(get_components(PSY.PowerLoad, sys))
    interruptible_load = PSY.InterruptiblePowerLoad(;
        name = get_name(load1) * "_interruptible",
        bus = get_bus(load1),
        available = get_available(load1),
        active_power = get_active_power(load1),
        reactive_power = get_reactive_power(load1),
        max_active_power = get_max_active_power(load1),
        max_reactive_power = get_max_reactive_power(load1),
        operation_cost = PSY.LoadCost(nothing),
        base_power = get_base_power(load1),
        conformity = get_conformity(load1),
    )
    add_component!(sys, interruptible_load)
    for ts_key in get_time_series_keys(load1)
        ts = get_time_series(load1, ts_key)
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
        set_max_active_power!(load, get_max_active_power(load) * load_pow_mult)
    end
    # replace with type of component?
    for therm in get_components(ThermalStandard, sys)
        op_cost = get_operation_cost(therm)
        op_cost isa MarketBidCost && continue
        with_units_base(sys, UnitSystem.DEVICE_BASE) do
            old_limits = get_active_power_limits(therm)
            new_limits = (min = old_limits.min, max = old_limits.max * therm_pow_mult)
            set_active_power_limits!(therm, new_limits)
        end
        if get_variable(op_cost) isa CostCurve{LinearCurve} ||
           get_variable(op_cost) isa CostCurve{QuadraticCurve}
            prop = get_proportional_term(get_value_curve(get_variable(op_cost)))
            set_variable!(op_cost, CostCurve(LinearCurve(prop * therm_price_mult)))
        elseif get_variable(op_cost) isa CostCurve{PiecewiseIncrementalCurve}
            pwl = get_value_curve(get_variable(op_cost))
            new_pwl = PiecewiseIncrementalCurve(
                therm_price_mult * get_initial_input(pwl),
                get_x_coords(pwl),
                therm_price_mult * get_slopes(pwl),
            )
            set_variable!(op_cost, CostCurve(new_pwl))
        else
            error("Unhandled operation cost variable type $(typeof(get_variable(op_cost)))")
        end
    end
end

tweak_for_startup_shutdown!(sys::System) = tweak_system!(sys::System, 0.8, 1.0, 1.0)

tweak_for_decremental_initial!(sys::PSY.System) = tweak_system!(sys, 1.0, 1.2, 0.5)

"""Transfer the market bid cost from old_comp to new_comp, copying any time series in the process."""
function transfer_mbc!(
    new_comp::PSY.Device,
    old_comp::PSY.Device,
    new_sys::PSY.System,
)
    mbc = deepcopy(get_operation_cost(old_comp))
    @assert mbc isa PSY.MarketBidCost
    for field in fieldnames(PSY.MarketBidCost)
        val = getfield(mbc, field)
        if val isa IS.TimeSeriesKey
            ts = PSY.get_time_series(old_comp, val)
            new_ts_key = add_time_series!(new_sys, new_comp, deepcopy(ts))
            setfield!(mbc, field, new_ts_key)
        end
    end
    set_operation_cost!(new_comp, mbc)
    return
end

function zero_out_startup_shutdown_costs!(comp::PSY.Device)
    op_cost = get_operation_cost(comp)::MarketBidCost
    set_start_up!(op_cost, (hot = 0.0, warm = 0.0, cold = 0.0))
    set_shut_down!(op_cost, 0.0)
end

"""Set everything except the incremental_offer_curves to zero on the MarketBidCost attached to the unit."""
function zero_out_non_incremental_curve!(sys::PSY.System, unit::PSY.Component)
    cost = deepcopy(get_operation_cost(unit)::MarketBidCost)
    set_no_load_cost!(cost, 0.0)
    set_start_up!(cost, (hot = 0.0, warm = 0.0, cold = 0.0))
    set_shut_down!(cost, 0.0)
    # set minimum generation cost (but not min gen power) to zero.
    if get_incremental_offer_curves(cost) isa IS.TimeSeriesKey
        zero_ts = make_deterministic_ts(sys, "initial_input", 0.0, 0.0, 0.0)
        zero_ts_key = add_time_series!(sys, unit, zero_ts)
        set_incremental_initial_input!(cost, zero_ts_key)
    else
        base_curve = get_value_curve(get_incremental_offer_curves(cost))
        x_coords = get_x_coords(base_curve)
        slopes = get_slopes(base_curve)
        new_curve = PiecewiseIncrementalCurve(0.0, x_coords, slopes)
        set_incremental_offer_curves!(cost, CostCurve(new_curve))
    end
    set_operation_cost!(unit, cost)
end

"Set the no_load_cost to `nothing` and the initial_input to the old no_load_cost. Not designed for time series"
function no_load_to_initial_input!(comp::Generator)
    cost = get_operation_cost(comp)::MarketBidCost
    no_load = PSY.get_no_load_cost(cost)
    old_fd = get_function_data(
        get_value_curve(get_incremental_offer_curves(get_operation_cost(comp))),
    )::IS.PiecewiseStepData
    new_vc = PiecewiseIncrementalCurve(old_fd, no_load, nothing)
    set_incremental_offer_curves!(get_operation_cost(comp), CostCurve(new_vc))
    set_no_load_cost!(get_operation_cost(comp), nothing)
    return
end

no_load_to_initial_input!(
    sys::PSY.System,
    sel = make_selector(x -> get_operation_cost(x) isa MarketBidCost, Generator),
) = no_load_to_initial_input!.(get_components(sel, sys))

"Set all MBC thermal unit min active powers to their min breakpoints"
function adjust_min_power!(sys)
    for comp in get_components(Union{ThermalStandard, ThermalMultiStart}, sys)
        op_cost = get_operation_cost(comp)
        op_cost isa MarketBidCost || continue
        cost_curve = get_incremental_offer_curves(op_cost)::CostCurve
        baseline = get_value_curve(cost_curve)::PiecewiseIncrementalCurve
        x_coords = get_x_coords(get_function_data(baseline))
        with_units_base(sys, UnitSystem.NATURAL_UNITS) do
            set_active_power_limits!(comp, (min = first(x_coords), max = last(x_coords)))
        end
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
    @assert get_operation_cost(unit1) isa MarketBidCost
    startup_ts_1 = make_deterministic_ts(
        sys,
        "start_up",
        (1.0, 1.5, 2.0),
        res_incr,
        interval_incr,
    )
    set_start_up!(sys, unit1, startup_ts_1)
    shutdown_ts_1 =
        make_deterministic_ts(sys, "shut_down", 0.5, res_incr, interval_incr)
    set_shut_down!(sys, unit1, shutdown_ts_1)
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
    base_startup = Tuple(get_start_up(get_operation_cost(unit1)))
    base_shutdown = get_shut_down(get_operation_cost(unit1))
    @assert get_operation_cost(unit1) isa MarketBidCost
    startup_ts_1 = make_deterministic_ts(
        sys,
        "start_up",
        base_startup,
        res_incr,
        interval_incr,
    )
    set_start_up!(sys, unit1, startup_ts_1)
    shutdown_ts_1 =
        make_deterministic_ts(
            sys,
            "shut_down",
            base_shutdown,
            res_incr,
            interval_incr,
        )
    set_shut_down!(sys, unit1, shutdown_ts_1)
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
    with_units_base(sys, UnitSystem.NATURAL_UNITS) do
        limits = get_active_power_limits(unit1)
        rating = get_rating(unit1)
        active_power = get_active_power(unit1)
        set_base_power!(unit1, get_base_power(sys) * 1.4)
        set_active_power_limits!(unit1, limits)
        set_rating!(unit1, rating)
        set_active_power!(unit1, active_power)
    end
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
            variable = get_incremental_offer_curves(old_cost),
            start_up = get_start_up(old_cost),
            shut_down = get_shut_down(old_cost),
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
                variable = CostCurve(
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
    for ts_key in get_time_series_keys(il)
        if get_name(ts_key) == "max_active_power"
            max_active_power_ts = get_time_series(
                first(get_components(PSY.InterruptiblePowerLoad, sys)),
                ts_key,
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
    old_ic = IncrementalCurve(get_value_curve(get_variable(old_op)))
    new_ii = get_initial_input(old_ic) + get_fixed(old_op)
    new_ic = IncrementalCurve(get_function_data(old_ic), new_ii, nothing)
    set_operation_cost!(
        ms_comp,
        MarketBidCost(;
            no_load_cost = nothing,
            start_up = (hot = 300.0, warm = 450.0, cold = 500.0),
            shut_down = 100.0,
            incremental_offer_curves = CostCurve(new_ic),
        ),
    )

    add_ts && add_startup_shutdown_ts_b!(c_sys5_pglib, with_increments)
    return c_sys5_pglib
end
