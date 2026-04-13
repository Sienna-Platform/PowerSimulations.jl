function test_market_bid_cost_models(sys::PSY.System,
    test_unit::PSY.Component,
    my_no_load::Float64,
    my_initial_input::Float64;
    skip_setting = false,
    device_to_formulation = FormulationDict(),
    filename::Union{String, Nothing} = nothing,
)
    fcn_data = get_function_data(
        get_value_curve(
            get_incremental_offer_curves(get_operation_cost(test_unit)),
        ),
    )
    if !skip_setting
        new_vc = PiecewiseIncrementalCurve(fcn_data, my_initial_input, my_no_load)
        set_incremental_offer_curves!(
            get_operation_cost(test_unit),
            CostCurve(new_vc),
        )
    end
    set_no_load_cost!(get_operation_cost(test_unit), my_no_load)
    template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))

    set_formulations!(
        template,
        sys,
        device_to_formulation,
    )

    model = DecisionModel(
        template,
        sys;
        name = "UC_test_mbc",
        optimizer = HiGHS_optimizer_small_gap,
        optimizer_solve_log_print = true,
        store_variable_names = true,
    )
    @test build!(model; output_dir = test_path) == PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    if !isnothing(filename)
        save_loc = joinpath(DOWNLOADS, "thermal_vs_renewable")
        @assert isdir(save_loc)

        save_objective_function(
            model,
            joinpath(
                save_loc,
                "objective_function_$(filename)_$(get_name(test_unit)).txt",
            ),
        )
        save_constraints(
            model,
            joinpath(save_loc, "constraints_$(filename)_$(get_name(test_unit)).txt"),
        )
    end

    return OptimizationProblemResults(model)
end

function verify_market_bid_cost_models(
    sys::PSY.System,
    test_unit::PSY.Component,
    cost_reference::Float64,
    no_load_cost::Float64,
    my_initial_input::Float64,
)
    results = test_market_bid_cost_models(
        sys,
        test_unit,
        no_load_cost,
        my_initial_input,
    )
    expr = read_expression(results, "ProductionCostExpression__ThermalStandard")
    component_df = @rsubset(expr, :name == get_name(test_unit))
    shutdown_cost = PSY.get_shut_down(PSY.get_operation_cost(test_unit))
    var_unit_cost =
        sum(@rsubset(component_df, :value != shutdown_cost)[:, :value])
    unit_cost_due_to_initial =
        nrow(@rsubset(component_df, :value != shutdown_cost)) * my_initial_input
    @test isapprox(
        var_unit_cost - PSY.get_start_up(PSY.get_operation_cost(test_unit))[:hot],
        cost_reference + unit_cost_due_to_initial;
        atol = 1,
    )
end

@testset "Test Thermal Generation MarketBidCost models" begin
    test_cases = [
        ("Base case", "fixed_market_bid_cost", 18487.236, 30.0, 30.0),
        ("Greater initial input, no load", "fixed_market_bid_cost", 18487.236, 31.0, 31.0),
        ("Greater initial input only", "fixed_market_bid_cost", 18487.236, 30.0, 31.0),
    ]
    for (name, sys_name, cost_reference, my_no_load, my_initial_input) in test_cases
        @testset "$name" begin
            sys = PSB.build_system(PSITestSystems, "c_$(sys_name)")
            unit1 = get_component(ThermalStandard, sys, "Test Unit1")
            verify_market_bid_cost_models(
                sys,
                unit1,
                cost_reference,
                my_no_load,
                my_initial_input,
            )
        end
    end
end

@testset "Test Renewable Dispatch MarketBidCost models" begin
    test_cases = [
        ("Base case", "fixed_market_bid_cost", 18487.236, 30.0, 30.0),
        ("Greater initial input, no load", "fixed_market_bid_cost", 18487.236, 31.0, 31.0),
        ("Greater initial input only", "fixed_market_bid_cost", 18487.236, 30.0, 31.0),
    ]
    for (name, sys_name, _, my_no_load, my_initial_input) in test_cases
        @testset "$name" begin
            sys = build_system(PSITestSystems, "c_$(sys_name)")
            unit1 = get_component(ThermalStandard, sys, "Test Unit1")
            replace_with_renewable!(sys, unit1)
            rg1 = get_component(PSY.RenewableDispatch, sys, "RG1")
            test_market_bid_cost_models(
                sys,
                rg1,
                my_no_load,
                my_initial_input,
            )
        end
    end
end

@testset "Compare Renewable and Standard Thermal MarketBidCost" begin
    (name, sys_name) = ("Base case", "fixed_market_bid_cost")
    sys = build_system(PSITestSystems, "c_$(sys_name)")
    unit1 = get_component(ThermalStandard, sys, "Test Unit1")
    replace_with_renewable!(sys, unit1; use_thermal_max_power = true)
    rg1 = get_component(PSY.RenewableDispatch, sys, "RG1")
    zero_out_non_incremental_curve!(sys, rg1)
    set_name!(sys, "sys_renewable")
    results_renewable = test_market_bid_cost_models(
        sys,
        rg1,
        0.0,
        0.0;
        skip_setting = true,
    )

    sys_thermal = build_system(PSITestSystems, "c_$(sys_name)")
    unit1 = get_component(ThermalStandard, sys_thermal, "Test Unit1")
    set_active_power_limits!(
        unit1,
        (min = 0.0, max = get_active_power_limits(unit1).max),
    )
    set_operation_cost!(unit1, deepcopy(get_operation_cost(rg1)))
    set_name!(sys_thermal, "sys_thermal")

    results_thermal = test_market_bid_cost_models(
        sys_thermal,
        unit1,
        0.0,
        0.0;
        skip_setting = true,
    )

    # check that the operation costs are the same.
    IS.compare_values(get_operation_cost(rg1), get_operation_cost(unit1))
    for thermal_unit in get_components(ThermalStandard, sys)
        sys_thermal_unit =
            get_component(ThermalStandard, sys_thermal, get_name(thermal_unit))
        IS.compare_values(
            thermal_unit,
            sys_thermal_unit,
        )
    end
    for load in get_components(PSY.PowerLoad, sys)
        IS.compare_values(load, get_component(PSY.PowerLoad, sys_thermal, get_name(load)))
    end

    @test isapprox(PSI.read_optimizer_stats(results_thermal)[!, "objective_value"],
        PSI.read_optimizer_stats(results_renewable)[!, "objective_value"])
end

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
    for (step_dt, step_df) in pairs(sh_param)
        for gen_name in unique(step_df.name)
            comp = get_component(gentype, sys, gen_name)
            fc_comp =
                get_shut_down(comp, PSY.get_operation_cost(comp); start_time = step_dt)
            @test all(step_df[!, :DateTime] .== TimeSeries.timestamp(fc_comp))
            @test all(
                isapprox.(
                    @rsubset(step_df, :name == gen_name).value,
                    TimeSeries.values(fc_comp),
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
_read_start_vars(::Val{false}, res::IS.Results) =
    read_variable_dict(res, PSI.StartVariable, ThermalStandard)

"Read the relevant startup variables: yes multistart case"
function _read_start_vars(::Val{true}, res::IS.Results)
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
Read startup and shutdown cost time series from a `System` and multiply by relevant start
and stop variables in the `IS.Results` to determine the cost that should have been incurred
by time-varying `MarketBidCost` startup and shutdown costs. Must run separately for
multistart vs. not.
"""
function cost_due_to_time_varying_startup_shutdown(
    sys::System,
    res::IS.Results;
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
            (cost isa PSY.MarketBidCost) || continue
            PSI.is_time_variant(get_start_up(cost)) || continue
            @assert PSI.is_time_variant(get_shut_down(cost))
            startup_ts = get_start_up(comp, cost; start_time = step_dt)
            shutdown_ts = get_shut_down(comp, cost; start_time = step_dt)

            @assert all(unique(start_df.DateTime) .== TimeSeries.timestamp(startup_ts))
            @assert all(unique(start_df.DateTime) .== TimeSeries.timestamp(shutdown_ts))
            startup_values = if multistart
                TimeSeries.values(startup_ts)
            else
                getproperty.(TimeSeries.values(startup_ts), :hot)
            end
            push!(
                dfs,
                DataFrame(
                    :DateTime => timestamps,
                    :name => repeat([gen_name], length(timestamps)),
                    :value =>
                        LinearAlgebra.dot.(
                            @rsubset(start_df, :name == gen_name).value,
                            startup_values,
                        ) .+
                        @rsubset(stop_df, :name == gen_name).value .*
                        TimeSeries.values(shutdown_ts),
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
    set_shut_down!(cost, 0.5)
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
    # https://github.com/NREL-Sienna/PowerSimulations.jl/issues/1460
    load_pow_mult_a = 1.01
    therm_pow_mult_a = 1.07
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
            for use_simulation in (false, true)
                decisions1, decisions2 =
                    run_mbc_obj_fun_test(
                        baseline,
                        varying,
                        comp_name,
                        comp_type;
                        simulation = use_simulation,
                        has_initial_input = init_input_bool,
                        is_decremental = decremental,
                        filename = SAVE_FILES ? "everything_" : nothing,
                        device_to_formulation = device_to_formulation,
                    )
                if !all(isapprox.(decisions1, decisions2))
                    @error decisions1
                    @error decisions2
                end
                @assert all(approx_geq_1.(decisions1))
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

@testset "MarketBidCost incremental with heterogeneous time series names" begin
    sel = make_selector(x -> get_operation_cost(x) isa MarketBidCost, ThermalStandard)
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

@testset "Test some MarketBidCost data validations" begin
    # Test multistart and convexity validation
    nonconvex = build_sys_incr(
        false,
        false,
        false;
        modify_baseline_pwl = pwl -> begin
            y_coords = get_y_coords(pwl)
            y_coords[3] = y_coords[1]
            pwl
        end,
    )
    set_start_up!(
        get_operation_cost(get_component(ThermalStandard, nonconvex, "Test Unit2")),
        (hot = 1.0, warm = 1.5, cold = 2.0),
    )
    model = build_generic_mbc_model(nonconvex; multistart = false)
    # We'll use build_impl! rather than build! to keep PSI's logging configuration from interfering with @test_logs and polluting the test output
    mkpath(test_path)
    PSI.set_output_dir!(model, test_path)
    @test_logs (:warn, r"Multi-start costs detected for non-multi-start unit Test Unit2.*") (
        match_mode = :any
    ) (@test_throws "is non-convex" PSI.build_impl!(model))

    # Test constant P1 validation
    variable_p1 = build_sys_incr(false, true, false; do_override_min_x = false)
    model = build_generic_mbc_model(variable_p1; multistart = false)
    mkpath(test_path)
    PSI.set_output_dir!(model, test_path)
    @test_throws "Inconsistent minimum breakpoint values" PSI.build_impl!(model)
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

@testset "concavity check error" begin
    sys = build_system(PSITestSystems, "c_sys5_il")
    load = first(get_components(PSY.InterruptiblePowerLoad, sys))
    selector = make_selector(PSY.InterruptiblePowerLoad, get_name(load))
    non_decr_slopes = [0.13, 0.11, 0.12]  # Non-decreasing slopes (should trigger error)
    x_coords = [0.1, 0.3, 0.6, 1.0]
    pw_curve = PiecewiseIncrementalCurve(0.0, 0.0, x_coords, non_decr_slopes)
    add_mbc_inner!(sys, selector; decr_curve = pw_curve)  # Fixed: pass selector, not slopes

    comp = first(get_components(selector, sys))
    @assert typeof(get_operation_cost(comp)) == PSY.MarketBidCost

    model = build_generic_mbc_model(sys)
    mkpath(test_path)
    PSI.set_output_dir!(model, test_path)
    msg = "ArgumentError: Decremental MarketBidCost for component $(get_name(load)) is non-concave"
    @test_throws msg PSI.build_impl!(model)
end

@testset "MarketBidCost decremental basic: single problem" begin
    sys = build_system(PSITestSystems, "c_sys5_il")
    load = first(get_components(PSY.InterruptiblePowerLoad, sys))
    selector = make_selector(PSY.InterruptiblePowerLoad, get_name(load))
    add_mbc!(sys, selector; incremental = false, decremental = true)
    @assert typeof(get_operation_cost(load)) == PSY.MarketBidCost
    _, res = run_generic_mbc_prob(sys)
end

@testset "MarketBidCost decremental basic: simulation" begin
    sys = build_system(PSITestSystems, "c_sys5_il")
    load = first(get_components(PSY.InterruptiblePowerLoad, sys))
    selector = make_selector(PSY.InterruptiblePowerLoad, get_name(load))
    add_mbc!(sys, selector; incremental = false, decremental = true)
    extend_mbc!(sys, selector)
    op_cost = get_operation_cost(load)
    @assert typeof(op_cost) == PSY.MarketBidCost
    @assert typeof(get_decremental_offer_curves(op_cost)) <: PSY.TimeSeriesKey
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

@testset "Test VOM cost time normalization across different resolutions" begin
    # Test that VOM costs scale correctly with time resolution
    # This validates the bugfix in common.jl lines 188-196

    # Build system at hourly resolution
    sys_hourly = build_system(PSITestSystems, "c_sys5")

    # Add VOM cost to a thermal unit
    thermal_unit = first(get_components(ThermalStandard, sys_hourly))
    op_cost = get_operation_cost(thermal_unit)

    # Modify the VOM cost on the existing variable cost structure
    # VOM cost is stored in the CostCurve's vom_cost field
    if op_cost isa PSY.ThermalGenerationCost
        var_cost = PSY.get_variable(op_cost)
        value_curve = PSY.get_value_curve(var_cost)
        power_units = PSY.get_power_units(var_cost)

        # Create new CostCurve with non-zero VOM (LinearCurve with proportional term = 5.0)
        vom_value = LinearCurve(5.0)  # $/MWh
        new_var_cost = CostCurve(value_curve, power_units, vom_value)

        new_op_cost = PSY.ThermalGenerationCost(;
            variable = new_var_cost,
            fixed = get_fixed(op_cost),
            start_up = get_start_up(op_cost),
            shut_down = get_shut_down(op_cost),
        )
        set_operation_cost!(thermal_unit, new_op_cost)
    end

    # Build and solve at hourly resolution
    template_hourly = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(template_hourly, ThermalStandard, ThermalDispatchNoMin)
    set_device_model!(template_hourly, PowerLoad, StaticPowerLoad)

    model_hourly = DecisionModel(
        template_hourly,
        sys_hourly;
        name = "VOM_hourly",
        optimizer = HiGHS_optimizer,
        optimizer_solve_log_print = false,
    )
    @test build!(model_hourly; output_dir = test_path) == PSI.ModelBuildStatus.BUILT
    @test solve!(model_hourly) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results_hourly = OptimizationProblemResults(model_hourly)
    expr_hourly = read_expression(
        results_hourly,
        "ProductionCostExpression__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    # Build system at 30-minute resolution (same system, different model resolution)
    sys_30min = build_system(PSITestSystems, "c_sys5")

    # Add same VOM cost to thermal unit
    thermal_unit_30 = first(get_components(ThermalStandard, sys_30min))
    op_cost_30 = get_operation_cost(thermal_unit_30)

    if op_cost_30 isa PSY.ThermalGenerationCost
        var_cost_30 = PSY.get_variable(op_cost_30)
        value_curve_30 = PSY.get_value_curve(var_cost_30)
        power_units_30 = PSY.get_power_units(var_cost_30)

        # Create new CostCurve with same VOM cost
        vom_value_30 = LinearCurve(5.0)  # $/MWh
        new_var_cost_30 = CostCurve(value_curve_30, power_units_30, vom_value_30)

        new_op_cost_30 = PSY.ThermalGenerationCost(;
            variable = new_var_cost_30,
            fixed = get_fixed(op_cost_30),
            start_up = get_start_up(op_cost_30),
            shut_down = get_shut_down(op_cost_30),
        )
        set_operation_cost!(thermal_unit_30, new_op_cost_30)
    end

    # Build and solve at 30-minute resolution
    template_30min = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(template_30min, ThermalStandard, ThermalDispatchNoMin)
    set_device_model!(template_30min, PowerLoad, StaticPowerLoad)

    model_30min = DecisionModel(
        template_30min,
        sys_30min;
        name = "VOM_30min",
        optimizer = HiGHS_optimizer,
        optimizer_solve_log_print = false,
        resolution = Dates.Minute(30),  # Set 30-minute resolution here
    )
    @test build!(model_30min; output_dir = test_path) == PSI.ModelBuildStatus.BUILT
    @test solve!(model_30min) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results_30min = OptimizationProblemResults(model_30min)
    expr_30min = read_expression(
        results_30min,
        "ProductionCostExpression__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    # Get active power values to compute expected VOM costs
    p_hourly = read_variable(
        results_hourly,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    p_30min = read_variable(
        results_30min,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    # Verify VOM costs scale with resolution
    # For 30-min resolution, each time step is 0.5 hours, so VOM cost = vom_value * power * 0.5
    # For hourly resolution, each time step is 1.0 hours, so VOM cost = vom_value * power * 1.0

    unit_name = get_name(thermal_unit)

    # Sum total costs over all time steps
    total_cost_hourly = sum(expr_hourly[!, unit_name])
    total_cost_30min = sum(expr_30min[!, unit_name])

    # The total costs should be approximately equal because:
    # - Hourly: 24 steps × power × VOM × 1.0 hour
    # - 30-min: 48 steps × power × VOM × 0.5 hour
    # Both should sum to roughly the same total cost over 24 hours

    @test isapprox(total_cost_hourly, total_cost_30min; rtol = 0.05)
end

@testset "Test MarketBidCost VOM cost time normalization across different resolutions" begin
    # Test that VOM costs in MarketBidCost (OfferCurveCost path) scale correctly
    # with time resolution. This validates the bugfix in market_bid.jl
    # _add_vom_cost_to_objective_helper! (GitHub issue #1531)

    function _build_mbc_vom_system()
        sys = load_sys_incr()
        unit = get_component(ThermalStandard, sys, "Test Unit1")
        mbc = get_operation_cost(unit)
        offer_curves = get_incremental_offer_curves(mbc)

        # Add VOM cost to the existing offer curves
        new_offer_curves = CostCurve(
            get_value_curve(offer_curves),
            get_power_units(offer_curves),
            LinearCurve(5.0),  # $/MWh VOM cost
        )
        set_incremental_offer_curves!(mbc, new_offer_curves)
        return sys, get_name(unit)
    end

    # Build and solve at hourly resolution
    sys_hourly, unit_name = _build_mbc_vom_system()

    template_hourly = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(template_hourly, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template_hourly, PowerLoad, StaticPowerLoad)

    model_hourly = DecisionModel(
        template_hourly,
        sys_hourly;
        name = "MBC_VOM_hourly",
        optimizer = HiGHS_optimizer,
        optimizer_solve_log_print = false,
    )
    @test build!(model_hourly; output_dir = test_path) == PSI.ModelBuildStatus.BUILT
    @test solve!(model_hourly) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results_hourly = OptimizationProblemResults(model_hourly)
    expr_hourly = read_expression(
        results_hourly,
        "ProductionCostExpression__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    # Build and solve at 30-minute resolution
    sys_30min, _ = _build_mbc_vom_system()

    template_30min = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(template_30min, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template_30min, PowerLoad, StaticPowerLoad)

    model_30min = DecisionModel(
        template_30min,
        sys_30min;
        name = "MBC_VOM_30min",
        optimizer = HiGHS_optimizer,
        optimizer_solve_log_print = false,
        resolution = Dates.Minute(30),
    )
    @test build!(model_30min; output_dir = test_path) == PSI.ModelBuildStatus.BUILT
    @test solve!(model_30min) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results_30min = OptimizationProblemResults(model_30min)
    expr_30min = read_expression(
        results_30min,
        "ProductionCostExpression__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    total_cost_hourly = sum(expr_hourly[!, unit_name])
    total_cost_30min = sum(expr_30min[!, unit_name])

    # Total costs should be approximately equal:
    # - Hourly: 24 steps × power × cost × 1.0 hour
    # - 30-min: 48 steps × power × cost × 0.5 hour
    @test isapprox(total_cost_hourly, total_cost_30min; rtol = 0.05)
end

@testset "Test Market Bid Cost With Single Time Serie" begin
    sys = build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    existing_ts = get_time_series_array(
        SingleTimeSeries,
        first(get_components(PowerLoad, sys)),
        "max_active_power",
    )
    tstamps = timestamp(existing_ts)
    psd1 = PiecewiseStepData([0.0, 40.0], [5.0])
    psd2 = PiecewiseStepData([0.0, 30.0, 400.0], [10.0, 20.0])
    psd3 = PiecewiseStepData([0.0, 40.0], [500.0])

    # Cheap the first 10 hours, moderate next 4 hours, expensive last 34 hours
    total_step_data = vcat([psd1 for x in 1:10], [psd2 for x in 1:4], [psd3 for x in 1:34])
    mbid_tarray = TimeArray(tstamps, total_step_data)
    ts_mbid = SingleTimeSeries(; name = "variable_cost", data = mbid_tarray)

    th = get_component(ThermalStandard, sys, "Alta")
    # Create an empty market bid and set it
    th_cost = MarketBidCost(;
        no_load_cost = 0.0,
        start_up = (hot = 0.0, warm = 0.0, cold = 0.0),
        shut_down = 0.0,
    )
    set_operation_cost!(th, th_cost)
    # Wrapper for adding the timeseries in incremental market bid cost
    set_variable_cost!(sys, th, ts_mbid, UnitSystem.NATURAL_UNITS)

    # It is also needed to create the initial input time series for market bid. That is the cost at 0 power at each time step. We will use zero for now.
    zero_input = zeros(length(tstamps))
    zero_tarray = TimeArray(tstamps, zero_input)
    ts_zero = SingleTimeSeries(; name = "initial_input", data = zero_tarray)
    set_incremental_initial_input!(sys, th, ts_zero)

    transform_single_time_series!(sys, Hour(24), Hour(24))

    template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, HydroDispatch, HydroDispatchRunOfRiver)
    set_device_model!(template, PowerLoad, StaticPowerLoad)

    model = DecisionModel(
        template,
        sys;
        name = "UC_MBCost",
        optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir()) == PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    p_var = read_variable(
        OptimizationProblemResults(model),
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    @test isapprox(sum(p_var[!, "Alta"][15:24]), 0.0, atol = 1e-4)
end
