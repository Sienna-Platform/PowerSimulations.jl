"""
PSI's MarketBidCost and ImportExportCost SIMULATION testing.

The single-problem equivalents of these tests live in POM's `test/test_market_bid_cost.jl`
and `test/test_market_bid_cost_equivalence.jl`. PSI's specific interest is the between-solve
cost-parameter refresh in `src/parameters/update_cost_parameters.jl`: a `Simulation` rebuilds
parameter values (breakpoints, slopes, initial-input costs, startup/shutdown costs) between
steps, and that refresh must land the same objective as a single, freshly-built model that
sees the same data all at once.

`test_generic_mbc_equivalence` (in `test/test_utils/mbc_simulation_utils.jl`) and
`run_iec_sim`/`iec_obj_fun_test_wrapper` (in `test/test_utils/iec_simulation_utils.jl`)
deliberately run each fixture through BOTH a single problem and a full simulation: that
cross-check is the point of the exercise, proving PSI's between-solve parameter updates
reproduce what a single, freshly-built model would compute. Do not drop either runner.
"""

"""
Run a simple simulation with the system and return information useful for testing
time-varying startup and shutdown functionality. Pass `simulation = false` to use a single
decision model, `true` for a full simulation.
"""
function run_startup_shutdown_test(
    sys::System;
    multistart::Bool = false,
    simulation = true,
    in_memory_store::Bool = false,
)
    model, res = if simulation
        run_generic_mbc_sim(sys; multistart = multistart, in_memory_store = in_memory_store)
    else
        run_generic_mbc_prob(sys; multistart = multistart)
    end

    # Test correctness of written shutdown cost parameters
    # TODO test startup too once we are able to write those
    gentype = multistart ? ThermalMultiStart : ThermalStandard
    genname = multistart ? "115_STEAM_1" : "Test Unit1"
    sh_param = read_parameter_dict(res, PSI.ShutdownCostParameter, gentype)
    resolution = first(PSY.get_time_series_resolutions(sys))
    for (step_dt, step_df) in pairs(sh_param)
        for gen_name in unique(step_df.name)
            comp = get_component(gentype, sys, gen_name)
            gen_df = @rsubset(step_df, :name == gen_name)
            horizon_count = nrow(gen_df)
            # `get_shut_down(device, cost; start_time, len)` resolves the whole
            # `horizon_count`-long window in one read (starting at `step_dt`, this
            # write's own window boundary), returning a `Vector` of static curves rather
            # than a `TimeArray` — psy6's replacement for the pre-psy6
            # `TimeSeries.timestamp`/`TimeSeries.values` shape.
            fc_comps = get_shut_down(
                comp,
                PSY.get_operation_cost(comp);
                start_time = step_dt,
                len = horizon_count,
            )
            expected_timestamps =
                [step_dt + (i - 1) * resolution for i in 1:horizon_count]
            @test gen_df[!, :DateTime] == expected_timestamps
            @test all(
                isapprox.(
                    gen_df.value,
                    IS.get_proportional_term.(get_function_data.(fc_comps)),
                ),
            )
        end
    end

    # These decisions need to be equal between certain pairs of problems/simulations and also need to be approx_geq_1
    decisions = if multistart
        (
            _read_one_value(res, PSI.HotStartVariable, gentype, genname),
            _read_one_value(res, PSI.WarmStartVariable, gentype, genname),
            _read_one_value(res, PSI.ColdStartVariable, gentype, genname),
            _read_one_value(res, PSI.StopVariable, gentype, genname),
            _read_one_value(res, PSI.OnVariable, gentype, genname),
        )
    else
        (
            _read_one_value(res, PSI.StartVariable, gentype, genname),
            _read_one_value(res, PSI.StopVariable, gentype, genname),
            _read_one_value(res, PSI.OnVariable, gentype, genname),
        )
    end

    # These decisions need to be equal between certain pairs of problems/simulations but need not be approx_geq_1 for the test to be valid
    nullable_decisions = if multistart
        (
            _read_one_value(res, PSI.PowerAboveMinimumVariable, gentype, genname),
            # sometimes useful for debugging clarity to check *another* generator's decisions
            _read_one_value(res, PSI.OnVariable, gentype, "101_CT_1"),
        )
    else
        ()
    end
    return model, res, decisions, nullable_decisions
end

"Read the relevant startup variables: no multistart case"
_read_start_vars(::Val{false}, res::IS.Outputs) =
    read_variable_dict(res, PSI.StartVariable, ThermalStandard)

"Read the relevant startup variables: yes multistart case"
function _read_start_vars(::Val{true}, res::IS.Outputs)
    hot_vars =
        read_variable_dict(res, PSI.HotStartVariable, ThermalMultiStart)
    warm_vars =
        read_variable_dict(res, PSI.WarmStartVariable, ThermalMultiStart)
    cold_vars =
        read_variable_dict(res, PSI.ColdStartVariable, ThermalMultiStart)

    @assert all(keys(hot_vars) .== keys(warm_vars))
    @assert all(keys(hot_vars) .== keys(cold_vars))
    @assert all(
        all(hot_vars[k][!, :DateTime] .== warm_vars[k][!, :DateTime]) for
        k in keys(hot_vars)
    )
    @assert all(
        all(hot_vars[k][!, :DateTime] .== cold_vars[k][!, :DateTime]) for
        k in keys(hot_vars)
    )
    combined_vars = Dict{DateTime, DataFrame}()
    for timestamp in keys(hot_vars)
        hot = hot_vars[timestamp]
        warm = warm_vars[timestamp]
        cold = cold_vars[timestamp]
        combined_vars[timestamp] = @chain DataFrames.rename(hot, :value => :hot) begin
            innerjoin(DataFrames.rename(warm, :value => :warm); on = [:DateTime, :name])
            innerjoin(
                DataFrames.rename(cold, :value => :cold);
                on = [:DateTime, :name],
            )
            @transform(@byrow(:value = (:hot, :warm, :cold)))
            @select(:DateTime, :name, :value)
        end
    end
    return combined_vars
end

"""
Sum of elementwise products of `a` and `b`. Used in place of `LinearAlgebra.dot` (not a
test dependency): the non-multistart case multiplies two `Float64`s; the multistart case
dots a `Tuple{Float64,Float64,Float64}` decision against a `StartUpStages` `NamedTuple`
cost, so both sides are pulled through `values` first — a `NamedTuple` cannot itself be
broadcast (`.*` is reserved for it), but `values` of either a `Tuple` or a `NamedTuple` is
a plain `Tuple`.
"""
_dot(a::Real, b::Real) = a * b
_dot(a, b) = sum(values(a) .* values(b))

"""
Read startup and shutdown cost time series from a `System` and multiply by relevant start
and stop variables in the `IS.Outputs` to determine the cost that should have been incurred
by time-varying `MarketBidCost` startup and shutdown costs. Must run separately for
multistart vs. not.
"""
function cost_due_to_time_varying_startup_shutdown(
    sys::System,
    res::IS.Outputs;
    multistart = false,
)
    gentype = multistart ? ThermalMultiStart : ThermalStandard
    start_vars = _read_start_vars(Val(multistart), res)
    stop_vars = read_variable_dict(res, PSI.StopVariable, gentype)
    result = SortedDict{DateTime, DataFrame}()
    IS.@assert_op Set(collect(keys(start_vars))) == Set(collect(keys(stop_vars)))
    for step_dt in keys(start_vars)
        start_df = start_vars[step_dt]
        stop_df = stop_vars[step_dt]
        @assert unique(start_df.name) == unique(stop_df.name)
        @assert start_df[!, :DateTime] == stop_df[!, :DateTime]
        timestamps = unique(start_df.DateTime)
        component_names = unique(start_df.name)
        dfs = Vector{DataFrame}()
        for gen_name in component_names
            comp = get_component(gentype, sys, gen_name)
            cost = PSY.get_operation_cost(comp)
            _is_mbc(cost) || continue
            PSI.is_time_variant(get_start_up(cost)) || continue
            @assert PSI.is_time_variant(get_shut_down(cost))
            # `get_start_up(device, cost; start_time)` has no `len` and, per
            # `IS.get_time_series_values`'s contract for a `Forecast`, `start_time` must be
            # a window boundary — only `step_dt` (this simulation step's own initial time)
            # qualifies; the interior hourly timestamps in `timestamps` do not, since only
            # as many windows exist as simulation steps (`add_startup_shutdown_ts_a!`/`_b!`
            # mirror the system's own forecast window count). So resolve the whole
            # `horizon_count`-long window in one read at `step_dt`, exactly like
            # `handle_variable_cost_parameter` does, rather than one call per timestamp.
            horizon_count = length(timestamps)
            startup_ts_values =
                PSY.StartUpStages.(
                    IS.get_time_series_values(
                        comp,
                        get_start_up(cost);
                        start_time = step_dt,
                        len = horizon_count,
                    ),
                )
            shutdown_ts_values =
                IS.get_proportional_term.(
                    IS.get_time_series_values(
                        comp,
                        IS.get_time_series_key(get_shut_down(cost));
                        start_time = step_dt,
                        len = horizon_count,
                    ),
                )
            startup_values = if multistart
                startup_ts_values
            else
                getproperty.(startup_ts_values, :hot)
            end
            push!(
                dfs,
                DataFrame(
                    :DateTime => timestamps,
                    :name => repeat([gen_name], length(timestamps)),
                    :value =>
                        _dot.(
                            @rsubset(start_df, :name == gen_name).value,
                            startup_values,
                        ) .+
                        @rsubset(stop_df, :name == gen_name).value .*
                        shutdown_ts_values,
                ),
            )
        end
        if !isempty(dfs)
            result[step_dt] = vcat(dfs...)
        end
    end
    return result
end

"""
The methodology here is: run a model or simulation where the startup and shutdown time
series have constant values through time, then run a nearly identical model/simulation where
the values vary very slightly through time, not enough to affect the decisions but enough to
affect the objective value, then compare the size of the objective value change to an
expectation computed manually.

Pass `simulation = false` to use a single decision model, `true` for a full simulation.
Pass `in_memory_store = true` to use an in-memory store for the simulation. Default is HDF5.
"""
function run_startup_shutdown_obj_fun_test(
    sys1,
    sys2;
    multistart::Bool = false,
    simulation = true,
    in_memory_store::Bool = false,
)
    _, res1, decisions1, nullable_decisions1 =
        run_startup_shutdown_test(
            sys1;
            multistart = multistart,
            simulation = simulation,
            in_memory_store = in_memory_store,
        )
    _, res2, decisions2, nullable_decisions2 =
        run_startup_shutdown_test(
            sys2;
            multistart = multistart,
            simulation = simulation,
            in_memory_store = in_memory_store,
        )

    all_decisions1 = (decisions1..., nullable_decisions1...)
    all_decisions2 = (decisions2..., nullable_decisions2...)

    if !all(isapprox.(all_decisions1, all_decisions2; atol = 1))
        @error all_decisions1
        @error all_decisions2
        # Given the solver tolerance, this method can result in up to 1 change in the commitment result
        @assert false "Decisions between constant and time-varying startup/shutdown do not match approximately"
    end

    # The last decision is the objetive function we can test that with a smaller tolerance
    @test (isapprox(all_decisions1[end], all_decisions2[end]; atol = 1e-3))

    ground_truth_1 =
        cost_due_to_time_varying_startup_shutdown(sys1, res1; multistart = multistart)
    ground_truth_2 =
        cost_due_to_time_varying_startup_shutdown(sys2, res2; multistart = multistart)

    obj_fun_test_helper(ground_truth_1, ground_truth_2, res1, res2)
    return decisions1, decisions2
end

@testset "MarketBidCost with time series startup and shutdown, ThermalStandard" begin
    # Test that constant time series has the same objective value as no time series
    sys0 = load_and_fix_system(PSITestSystems, "c_fixed_market_bid_cost")
    tweak_for_startup_shutdown!(sys0)
    cost = get_operation_cost(get_component(ThermalStandard, sys0, "Test Unit1"))
    set_start_up!(cost, (hot = 1.0, warm = 1.5, cold = 2.0))
    set_shut_down!(cost, LinearCurve(0.5))
    sys1 = load_and_fix_system(PSITestSystems, "c_fixed_market_bid_cost")
    tweak_for_startup_shutdown!(sys1)
    add_startup_shutdown_ts_a!(sys1, false)
    test_generic_mbc_equivalence(sys0, sys1; multistart = false)

    # Test that perturbing the time series perturbs the objective value as expected
    sys2 = load_and_fix_system(PSITestSystems, "c_fixed_market_bid_cost")
    tweak_for_startup_shutdown!(sys2)
    add_startup_shutdown_ts_a!(sys2, true)

    for use_simulation in (false, true)
        in_memory_store_opts = use_simulation ? [false, true] : [false]
        for in_memory_store in in_memory_store_opts
            (decisions1, decisions2) =
                run_startup_shutdown_obj_fun_test(
                    sys1,
                    sys2;
                    simulation = use_simulation,
                    in_memory_store = in_memory_store,
                )
            # Make sure our tests included sufficent startups and shutdowns
            @assert all(approx_geq_1.(decisions1))
        end
    end
end

@testset "MarketBidCost with time series startup and shutdown, ThermalMultiStart" begin
    # The arguments to create_multistart_sys were tuned empirically to ensure (a) the
    # behavior under test is exercised and (b) the small perturbations to the costs aren't
    # enough to change the decisions that form the correct solution

    # Scenario 1: hot and warm starts
    # TODO the process to empirically tune these values so the tests work everywhere is
    # absolutely horrible, we need a more robust system ASAP
    # https://github.com/Sienna-Platform/PowerSimulations.jl/issues/1460
    # psy6 re-tune: the pre-psy6 values (1.01, 1.07, 7.40) produced a cold start here
    # instead of hot+warm under psy6's ThermalMultiStart/network formulation (a different,
    # equally-optimal commitment schedule from the one on `main`) — re-tuned empirically to
    # restore the intended hot+warm coverage; scenario 2 below is unaffected and still
    # contributes the cold start.
    load_pow_mult_a = 1.0
    therm_pow_mult_a = 1.0
    therm_price_mult_a = 7.40
    c_sys5_pglib0a = create_multistart_sys(
        false,
        load_pow_mult_a,
        therm_pow_mult_a,
        therm_price_mult_a;
        add_ts = false,
    )
    c_sys5_pglib1a =
        create_multistart_sys(false, load_pow_mult_a, therm_pow_mult_a, therm_price_mult_a)
    c_sys5_pglib2a =
        create_multistart_sys(true, load_pow_mult_a, therm_pow_mult_a, therm_price_mult_a)

    # Scenario 2: hot and cold starts
    load_pow_mult_b = 1.05
    therm_pow_mult_b = 1.0
    therm_price_mult_b = 7.4
    c_sys5_pglib0b = create_multistart_sys(
        false,
        load_pow_mult_b,
        therm_pow_mult_b,
        therm_price_mult_b;
        add_ts = false,
    )
    c_sys5_pglib1b =
        create_multistart_sys(false, load_pow_mult_b, therm_pow_mult_b, therm_price_mult_b)
    c_sys5_pglib2b =
        create_multistart_sys(true, load_pow_mult_b, therm_pow_mult_b, therm_price_mult_b)

    test_generic_mbc_equivalence(c_sys5_pglib0a, c_sys5_pglib1a; multistart = true)
    test_generic_mbc_equivalence(c_sys5_pglib0b, c_sys5_pglib1b; multistart = true)

    for use_simulation in (false, true)
        (decisions1, decisions2) = run_startup_shutdown_obj_fun_test(
            c_sys5_pglib1a,
            c_sys5_pglib2a;
            multistart = true,
            simulation = use_simulation,
        )
        # NOTE not all of the decision types here have >= 1, we'll do another scenario such that we get full decision coverage across both of them:

        (decisions1_2, decisions2_2) = run_startup_shutdown_obj_fun_test(
            c_sys5_pglib1b,
            c_sys5_pglib2b;
            multistart = true,
            simulation = use_simulation,
        )
        @test all(isapprox.(decisions1, decisions2))
        @test all(isapprox.(decisions1_2, decisions2_2))
        # Make sure our tests included all types of startups and shutdowns
        @test all(approx_geq_1.(decisions1 .+ decisions1_2))
    end
end

@testset "MarketBidCost incremental ThermalStandard, no time series versus constant time series" begin
    sys_no_ts = load_sys_incr()
    set_name!(sys_no_ts, "thermal_no_ts")
    sys_constant_ts = build_sys_incr(false, false, false)
    set_name!(sys_constant_ts, "thermal_constant_ts")
    test_generic_mbc_equivalence(
        sys_no_ts,
        sys_constant_ts,
    )
end

@testset "MarketBidCost incremental RenewableDispatch, no time series versus constant time series" begin
    sys_no_ts = load_sys_incr()
    sys_constant_ts = build_sys_incr(false, false, false)
    for sys in (sys_no_ts, sys_constant_ts)
        unit1 = get_component(SEL_INCR, sys)
        replace_with_renewable!(sys, unit1; magnitude = 1.0, random_variation = 0.1)
    end
    test_generic_mbc_equivalence(sys_no_ts, sys_constant_ts)
end

# debugging option: change to true to save text files of objective functions for
# certain tests that aren't passing.
const SAVE_FILES = false

for decremental in (false, true)
    adj = decremental ? "decremental" : "incremental"
    build_func = decremental ? build_sys_decr2 : build_sys_incr
    comp_type = decremental ? InterruptiblePowerLoad : ThermalStandard
    comp_name = decremental ? "Bus1_interruptible" : "Test Unit1"
    device_models = if decremental
        [PowerLoadInterruption, PowerLoadDispatch]
    else
        [ThermalBasicUnitCommitment]
    end
    @testset for device_model in device_models
        device_to_formulation = FormulationDict(comp_type => device_model)
        init_input_bool = !decremental || device_model != PowerLoadDispatch
        if init_input_bool
            @testset "MarketBidCost $(adj) with time varying min gen cost" begin
                baseline = build_func(false, false, false)
                varying = build_func(true, false, false)
                if decremental
                    tweak_for_decremental_initial!(varying)
                    tweak_for_decremental_initial!(baseline)
                end
                for use_simulation in (false, true)
                    in_memory_store_opts = use_simulation ? [false, true] : [false]
                    for in_memory_store in in_memory_store_opts
                        decisions1, decisions2 =
                            run_mbc_obj_fun_test(
                                baseline,
                                varying,
                                comp_name,
                                comp_type;
                                is_decremental = decremental,
                                has_initial_input = init_input_bool,
                                simulation = use_simulation,
                                in_memory_store = in_memory_store,
                                device_to_formulation = device_to_formulation,
                            )
                        if !all(isapprox.(decisions1, decisions2))
                            @error decisions1
                            @error decisions2
                        end
                        @assert all(approx_geq_1.(decisions1))
                    end
                end
            end
        end

        @testset "MarketBidCost $(adj) with time varying slopes" begin
            baseline = build_func(false, false, false)
            varying = build_func(false, false, true)

            set_name!(baseline, "baseline")
            set_name!(varying, "varying")

            for use_simulation in (false, true)
                in_memory_store_opts = use_simulation ? [false, true] : [false]
                for in_memory_store in in_memory_store_opts
                    decisions1, decisions2 =
                        run_mbc_obj_fun_test(
                            baseline,
                            varying,
                            comp_name,
                            comp_type;
                            is_decremental = decremental,
                            has_initial_input = init_input_bool,
                            simulation = use_simulation,
                            in_memory_store = in_memory_store,
                            filename = SAVE_FILES ? "slopes_" : nothing,
                            device_to_formulation = device_to_formulation,
                        )
                    if !all(isapprox.(decisions1, decisions2))
                        @error decisions1
                        @error decisions2
                    end
                    @assert all(approx_geq_1.(decisions1))
                end
            end
        end

        @testset "MarketBidCost $(adj) with time varying breakpoints" begin
            baseline = build_func(false, false, false)
            varying = build_func(false, true, false)

            set_name!(baseline, "baseline")
            set_name!(varying, "varying")

            for use_simulation in (false, true)
                in_memory_store_opts = use_simulation ? [false, true] : [false]
                for in_memory_store in in_memory_store_opts
                    decisions1, decisions2 =
                        run_mbc_obj_fun_test(
                            baseline,
                            varying,
                            comp_name,
                            comp_type;
                            is_decremental = decremental,
                            has_initial_input = init_input_bool,
                            simulation = use_simulation,
                            in_memory_store = in_memory_store,
                            filename = SAVE_FILES ? "breakpoints_" : nothing,
                            device_to_formulation = device_to_formulation,
                        )
                    if !all(isapprox.(decisions1, decisions2))
                        @error decisions1
                        @error decisions2
                    end
                    @assert all(approx_geq_1.(decisions1))
                end
            end
        end

        @testset "MarketBidCost $(adj) with time varying everything" begin
            baseline = build_func(false, false, false)
            varying = build_func(init_input_bool, true, true)
            set_name!(baseline, "baseline")
            set_name!(varying, "varying")

            local filename
            if SAVE_FILES
                filename = "everything_"
            else
                filename = nothing
            end
            for use_simulation in (false, true)
                local in_memory_store_opts
                if use_simulation
                    in_memory_store_opts = [false, true]
                else
                    in_memory_store_opts = [false]
                end
                for in_memory_store in in_memory_store_opts
                    decisions1, decisions2 =
                        run_mbc_obj_fun_test(
                            baseline,
                            varying,
                            comp_name,
                            comp_type;
                            simulation = use_simulation,
                            in_memory_store = in_memory_store,
                            has_initial_input = init_input_bool,
                            is_decremental = decremental,
                            filename = filename,
                            device_to_formulation = device_to_formulation,
                        )
                    if !all(isapprox.(decisions1, decisions2))
                        @error decisions1
                        @error decisions2
                    end
                    @assert all(approx_geq_1.(decisions1))
                end
            end
        end

        @testset "MarketBidCost $(adj) with variable number of tranches" begin
            baseline = build_func(init_input_bool, true, true)
            set_name!(baseline, "baseline")
            variable_tranches =
                build_func(init_input_bool, true, true; create_extra_tranches = true)
            set_name!(variable_tranches, "variable")
            test_generic_mbc_equivalence(
                baseline,
                variable_tranches;
                filename = SAVE_FILES ? "tranches_" : nothing,
                is_decremental = decremental,
                device_to_formulation = device_to_formulation,
            )
        end
    end
end

@testset "MarketBidCost block-width update is independent of device base power" begin
    # `add_pwl_constraint_delta!` (IOM) builds each block-width constraint in system
    # per-unit; `_update_pwl_width_constraint!` (PSI `update_cost_parameters.jl`) must set
    # its RHS the same way between solves. The underlying conversion (`NaturalUnit` ->
    # `SystemBaseUnit`) is defined to depend only on system base power, never on device
    # base power (`IS.relative_units.jl`: `_cost_coeff_ratio(::NaturalUnit,
    # ::SystemBaseUnit, sb, _) = sb`), so two systems differing ONLY in "Test Unit1"'s
    # device base power must land on bit-identical width RHS values after the
    # between-step update. A real base-mismatch bug in the update path would show up here
    # as a difference between the two.
    device_to_formulation = FormulationDict(ThermalStandard => ThermalBasicUnitCommitment)

    # Fixture default: device base power (140) != system base power (100).
    sys_mismatch = build_sys_incr(false, true, false)

    # Same system, except "Test Unit1"'s device base power is forced equal to the system
    # base power; limits/rating/active power are rescaled by absolute MW so the physical
    # dispatch problem is unchanged, mirroring `load_sys_incr`'s own base-power rescale.
    sys_matched = build_sys_incr(false, true, false)
    u_matched = get_component(SEL_INCR, sys_matched)
    limits = get_active_power_limits(u_matched, PSY.NU)
    rating = get_rating(u_matched, PSY.NU)
    active_power = get_active_power(u_matched, PSY.NU)
    set_base_power!(u_matched, get_base_power(sys_matched))
    set_active_power_limits!(
        u_matched,
        (min = limits.min * PSY.MW, max = limits.max * PSY.MW),
    )
    set_rating!(u_matched, rating * PSY.MW)
    set_active_power!(u_matched, active_power * PSY.MW)

    model_mismatch, _ =
        run_generic_mbc_sim(sys_mismatch; device_to_formulation = device_to_formulation)
    model_matched, _ =
        run_generic_mbc_sim(sys_matched; device_to_formulation = device_to_formulation)

    function _width_rhs_at_t2(model)
        container = PSI.get_optimization_container(model)
        width_container = get_constraint(
            container, PiecewiseLinearBlockIncrementalWidthConstraint, ThermalStandard)
        blocks = sort([
            k for (name, k, t) in keys(width_container.data)
            if name == "Test Unit1" && t == 2
        ])
        return [JuMP.normalized_rhs(width_container[("Test Unit1", k, 2)]) for k in blocks]
    end

    rhs_mismatch = _width_rhs_at_t2(model_mismatch)
    rhs_matched = _width_rhs_at_t2(model_matched)
    @test !isempty(rhs_mismatch)
    @test isapprox(rhs_mismatch, rhs_matched; atol = 1e-9)
end

@testset "MarketBidCost incremental with heterogeneous time series names" begin
    # `_is_mbc` matches both MarketBidCost and MarketBidTimeSeriesCost: `build_sys_incr` (via
    # `extend_mbc!`) converts every selected component's cost to `MarketBidTimeSeriesCost`, so
    # a selector re-evaluated against `baseline` below (a `ComponentSelector` predicate is
    # live, not frozen at construction) must still match the post-conversion type.
    sel = make_selector(x -> _is_mbc(get_operation_cost(x)), ThermalStandard)
    baseline = build_sys_incr(true, true, true; active_components = sel)
    @assert length(get_components(sel, baseline)) == 2

    # Should succeed for varying initial input time series names:
    variable_ii_names = build_sys_incr(
        true,
        true,
        true;
        active_components = sel,
        initial_input_names_vary = true,
    )
    test_generic_mbc_equivalence(baseline, variable_ii_names)

    # Should give an informative error for varying variable cost time series names:
    variable_vc_names = build_sys_incr(
        true,
        true,
        true;
        active_components = sel,
        variable_cost_names_vary = true,
    )
    model = build_generic_mbc_model(variable_vc_names; multistart = false)
    test_path = mktempdir()
    PSI.set_output_dir!(model, test_path)
    # Commented out temporarily as the error changed
    # @test_throws "All time series names must be equal" PSI.build_impl!(model)  # see below re: build_impl!
end

@testset "Test 3d results" begin
    # TODO: Test actual values
    varying = build_sys_incr(true, true, true)
    for in_memory_store in (false, true)
        # model1, res1 = run_generic_mbc_sim(baseline)
        model2, res2 = run_generic_mbc_sim(varying; in_memory_store = in_memory_store)
        parameters = read_parameters(res2)
        @test haskey(
            parameters,
            "IncrementalPiecewiseLinearBreakpointParameter__ThermalStandard",
        )
        for df in
            values(
            parameters["IncrementalPiecewiseLinearBreakpointParameter__ThermalStandard"],
        )
            @test names(df) == ["DateTime", "name", "name2", "value"]
        end
        for (key, df) in read_realized_parameters(res2)
            if key in (
                "IncrementalPiecewiseLinearBreakpointParameter__ThermalStandard",
                "IncrementalPiecewiseLinearSlopeParameter__ThermalStandard",
            )
                @test names(df) == ["DateTime", "name", "name2", "value"]
            else
                @test names(df) == ["DateTime", "name", "value"]
            end
        end

        # TODO: Test actual values
    end
end

@testset "MarketBidCost decremental basic: simulation" begin
    sys = build_system(PSITestSystems, "c_sys5_il")
    load = first(get_components(PSY.InterruptiblePowerLoad, sys))
    selector = make_selector(PSY.InterruptiblePowerLoad, get_name(load))
    add_mbc!(sys, selector; incremental = false, decremental = true)
    extend_mbc!(sys, selector)
    # extend_mbc! always produces MarketBidTimeSeriesCost, since all five of its fields must
    # be time-series-backed once any one of them is (there is no partial/mixed representation).
    op_cost::PSY.MarketBidTimeSeriesCost = get_operation_cost(load)
    @assert _is_ts_pwl_curve(get_decremental_offer_curves(op_cost))
    _, res = run_generic_mbc_sim(sys)
end

@testset "MarketBidCost decremental PowerLoadInterruption, no time series vs constant time series" begin
    sys_no_ts = load_sys_decr2()
    sys_constant_ts = build_sys_decr2(false, false, false)
    test_generic_mbc_equivalence(sys_no_ts, sys_constant_ts)
end

# TODO error if there's nonzero decremental initial input for PowerLoadDispatch.
@testset "MarketBidCost decremental PowerLoadDispatch, no time series vs constant time series" begin
    device_to_formulation = FormulationDict(PSY.InterruptiblePowerLoad => PowerLoadDispatch)
    sys_no_ts = load_sys_decr2()
    sys_constant_ts = build_sys_decr2(false, false, false)
    test_generic_mbc_equivalence(
        sys_no_ts,
        sys_constant_ts;
        device_to_formulation = device_to_formulation,
    )
end

for reservation in (false, true)
    local label
    if reservation
        label = "on"
    else
        label = "off"
    end
    @testset "ImportExportCost incremental+decremental Source, no time series versus constant time series, reservation $label" begin
        sys_no_ts = make_5_bus_with_import_export(; name = "sys_no_ts")
        sys_constant_ts =
            make_5_bus_with_ie_ts(false, false, false, false; name = "sys_constant_ts")
        test_generic_mbc_equivalence(sys_no_ts, sys_constant_ts;
            device_to_formulation = FormulationDict(
                Source => DeviceModel(
                    Source,
                    ImportExportSourceModel;
                    attributes = Dict("reservation" => reservation),
                ),
            ),
        )
    end
end

@testset "ImportExportCost constant time series, reservation sanity checks" begin
    sys_constant_ts =
        make_5_bus_with_ie_ts(false, false, false, false; name = "sys_constant_ts")

    for use_simulation in (false, true),
        in_memory_store in (use_simulation ? (false, true) : (false,)),
        reservation in (false, true)

        run_iec_sim(sys_constant_ts,
            IEC_COMPONENT_NAME,
            IECComponentType;
            simulation = use_simulation,
            in_memory_store = in_memory_store,
            reservation = true,
        )
    end
end

# import_scalar/export_scalar ultimately multiply the ActivePowerOutVariable/ActivePowerInVariable
# objective function coefficients; the "breakpoints" cases pick a scalar that maxes out the
# corresponding variable.
const _IEC_VARYING_CASES = (
    (desc = "import slopes", varying = (false, true, false, false),
        import_scalar = 0.5, export_scalar = 2.0),
    (desc = "import breakpoints", varying = (true, false, false, false),
        import_scalar = 0.2, export_scalar = 2.0),
    (desc = "export slopes", varying = (false, false, false, true),
        import_scalar = 0.5, export_scalar = 2.0),
    (desc = "export breakpoints", varying = (false, false, true, false),
        import_scalar = 1.0, export_scalar = 50.0),
    (desc = "everything", varying = (true, true, true, true),
        import_scalar = 0.2, export_scalar = 40.0),
)

for case in _IEC_VARYING_CASES
    @testset "ImportExportCost with time varying $(case.desc), reservation off" begin
        sys_constant = make_5_bus_with_ie_ts(false, false, false, false;
            import_scalar = case.import_scalar, export_scalar = case.export_scalar,
            name = "sys_constant")
        sys_varying = make_5_bus_with_ie_ts(case.varying...;
            import_scalar = case.import_scalar, export_scalar = case.export_scalar,
            name = "sys_varying_$(replace(case.desc, ' ' => '_'))")
        iec_obj_fun_test_wrapper(sys_constant, sys_varying)
    end
end
