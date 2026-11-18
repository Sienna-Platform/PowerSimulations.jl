# WARNING: included in HydroPowerSimulations's tests as well.
# If you make changes, run those tests too!
const SEL_INCR = make_selector(ThermalStandard, "Test Unit1")
const SEL_DECR = make_selector(InterruptiblePowerLoad, "Bus1_interruptible")
const SEL_MULTISTART = make_selector(ThermalMultiStart, "115_STEAM_1")

# functions for replacing components in the system
function replace_load_with_interruptible!(sys::System)
    @assert !isempty(get_components(PSY.PowerLoad, sys))
    load1 = first(get_components(PSY.PowerLoad, sys))
    interruptible_load = PSY.InterruptiblePowerLoad(;
        name = get_name(load1) * "_interruptible",
        bus = get_bus(load1),
        available = get_available(load1),
        active_power = get_active_power(load1, u"CU"),
        reactive_power = get_reactive_power(load1, u"CU"),
        max_active_power = get_max_active_power(load1, u"CU"),
        max_reactive_power = get_max_reactive_power(load1, u"CU"),
        operation_cost = PSY.LoadCost(nothing),
        base_power = get_base_power(load1),
        conformity = get_conformity(load1),
        input_basis = u"CU",
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
            get_max_active_power(load, u"SU") * load_pow_mult * u"SU",
        )
        # The series is per unit of base_power, not max_active_power, so scale it as well.
        PSB.scale_time_series!(sys, load, "max_active_power", load_pow_mult)
    end
    # replace with type of component?
    for therm in get_components(ThermalStandard, sys)
        op_cost = get_operation_cost(therm)
        _is_market_bid_cost(op_cost) && continue
        old_limits = get_active_power_limits(therm, u"CU")
        new_limits =
            (min = old_limits.min * u"CU", max = old_limits.max * therm_pow_mult * u"CU")
        set_active_power_limits!(therm, new_limits)
        set_variable_operation_cost!(
            op_cost,
            _scaled_variable_cost(get_variable_operation_cost(op_cost), therm_price_mult),
        )
    end
end

_is_market_bid_cost(::MarketBidCost) = true
_is_market_bid_cost(::PSY.OperationalCost) = false

function _scaled_variable_cost(curve::CostCurve{LinearCurve}, mult)
    prop = get_proportional_term(get_value_curve(curve))
    return CostCurve(LinearCurve(prop * mult))
end

function _scaled_variable_cost(curve::CostCurve{QuadraticCurve}, mult)
    prop = get_proportional_term(get_value_curve(curve))
    return CostCurve(LinearCurve(prop * mult))
end

function _scaled_variable_cost(curve::CostCurve{PiecewiseIncrementalCurve}, mult)
    pwl = get_value_curve(curve)
    new_pwl = PiecewiseIncrementalCurve(
        mult * get_initial_input(pwl),
        get_x_coords(pwl),
        mult * get_slopes(pwl),
    )
    return CostCurve(new_pwl)
end

function _scaled_variable_cost(curve, mult)
    error("Unhandled operation cost variable type $(typeof(curve))")
end

tweak_for_startup_shutdown!(sys::System) = tweak_system!(sys::System, 0.8, 1.0, 1.0)

tweak_for_decremental_initial!(sys::PSY.System) = tweak_system!(sys, 1.0, 1.2, 0.5)

"""
Move the `minimum_energy_offer` cost onto the incremental offer curve's `initial_input`,
zeroing `minimum_energy_offer` in exchange (`MEO = no-load cost / P_min`, so the flat dollar
figure baked into `initial_input` is `MEO * P_min`). Not designed for time series.
"""
function no_load_to_initial_input!(sys::PSY.System, comp::Generator)
    cost = get_operation_cost(comp)::MarketBidCost
    meo_rate = IS.get_proportional_term(get_function_data(get_minimum_energy_offer(cost)))
    p_min = get_active_power_limits(comp, u"NU").min
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
            (min = first(x_coords) * u"MW", max = last(x_coords) * u"MW"),
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
    rebase_component!(get_component(SEL_INCR, sys), get_base_power(sys) * 1.4)
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
    PSB.set_constant_max_active_power!(
        sys,
        first(get_components(PSY.InterruptiblePowerLoad, sys)),
    )
    return sys
end

"""
Build the `ThermalMultiStart` system of the market bid cost tests. `115_STEAM_1` is off in
the hours `off_hours` of every forecast window and on otherwise (see
`PSB.force_multistart_commitment!`), so the off durations alone decide the start types.
"""
function create_multistart_sys(with_increments::Bool, off_hours; add_ts = true)
    @assert add_ts || !with_increments
    c_sys5_pglib = load_and_fix_system(PSITestSystems, "c_sys5_pglib")
    PSB.force_multistart_commitment!(c_sys5_pglib, off_hours)
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
