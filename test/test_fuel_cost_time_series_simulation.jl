# Regression test for the FuelCostParameter time-series update path
# (src/parameters/update_cost_parameters.jl): a thermal unit whose FuelCurve carries a
# time-varying fuel cost must get a fresh FuelCostParameter read every simulation step, not
# a value frozen at build time.
@testset "FuelCostParameter time series updates across simulation steps" begin
    sys, thermal, fuel_cost_data, windows = fuel_cost_time_series_system()

    template = test_template_unit_commitment(CopperPlateNetworkModel)
    model = DecisionModel(
        template,
        sys;
        name = "UC",
        optimizer = HiGHS_optimizer_small_gap,
    )
    models = SimulationModels(; decision_models = [model])
    sequence = SimulationSequence(;
        models = models,
        feedforwards = Dict(),
        ini_cond_chronology = InterProblemChronology(),
    )
    sim = Simulation(;
        name = "fuel_cost_ts_sim",
        steps = length(windows),
        models = models,
        sequence = sequence,
        initial_time = first(first(windows)),
        simulation_folder = mktempdir(; cleanup = true),
    )

    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED

    sim_res = SimulationResults(sim)
    res = get_decision_problem_results(sim_res, "UC")
    fuel_param = read_parameter(res, PSI.FuelCostParameter, PSY.ThermalStandard)
    @test length(fuel_param) == length(windows)

    thermal_name = PSY.get_name(thermal)
    for (step_dt, expected_values) in fuel_cost_data
        step_df = fuel_param[step_dt]
        actual = @rsubset(step_df, :name == thermal_name)
        @test actual[!, :value] == expected_values
    end
end

# A "fuel_cost" series, shaped like the load forecast, that `unit`'s cost does not read.
function _add_unused_fuel_cost!(sys::PSY.System, unit::PSY.ThermalStandard)
    load = first(PSY.get_components(PSY.PowerLoad, sys))
    load_ts = PSY.get_time_series(PSY.Deterministic, load, "max_active_power")
    data = OrderedDict(dt => fill(7.0, length(v)) for (dt, v) in get_data(load_ts))
    PSY.add_time_series!(
        sys,
        unit,
        PSY.Deterministic(;
            name = "fuel_cost",
            data = data,
            resolution = IS.get_resolution(load_ts),
            interval = IS.get_interval(load_ts),
        ),
    )
    return
end

function _unused_fuel_cost_operation_cost(unit::PSY.ThermalStandard, kind::Symbol)
    kind == :market_bid || return PSY.ThermalGenerationCost(;
        variable_operation_cost = if kind == :static_fuel
            PSY.FuelCurve(PSY.LinearCurve(1.0, 5.0), PSY.MMBtu, 3.0)
        else
            PSY.CostCurve(PSY.LinearCurve(15.0))
        end,
        fixed = 0.0,
        start_up = 0.0,
        shut_down = 0.0,
    )
    mbc = PSY.MarketBidCost(;
        start_up = (hot = 0.0, warm = 0.0, cold = 0.0),
        shut_down = PSY.LinearCurve(0.0),
    )
    p_max = PSY.get_active_power_limits(unit, u"MW").max
    PSY.set_incremental_offer_curves!(
        mbc,
        PSY.CostCurve(
            PSY.PiecewiseIncrementalCurve(0.0, [0.0, 0.5, 1.0] .* p_max, [10.0, 20.0]),
        ),
    )
    return mbc
end

@testset "A fuel_cost series that the unit's cost does not read" begin
    for with_time_series_cost in (true, false),
        kind in (:static_fuel, :cost_curve, :market_bid)

        @testset "$kind, time series cost in model: $with_time_series_cost" begin
            sys = if with_time_series_cost
                first(fuel_cost_time_series_system())
            else
                deepcopy(PSB.build_system(PSITestSystems, "c_sys5_uc"))
            end
            unit = last(collect(PSY.get_components(PSY.ThermalStandard, sys)))
            PSY.set_operation_cost!(unit, _unused_fuel_cost_operation_cost(unit, kind))
            _add_unused_fuel_cost!(sys, unit)

            model = DecisionModel(
                test_template_unit_commitment(CopperPlateNetworkModel),
                sys;
                name = "UC",
                optimizer = HiGHS_optimizer_small_gap,
            )
            models = SimulationModels(; decision_models = [model])
            sequence = SimulationSequence(;
                models = models,
                ini_cond_chronology = InterProblemChronology(),
            )
            sim = Simulation(;
                name = "unused_fuel_cost",
                steps = 2,
                models = models,
                sequence = sequence,
                simulation_folder = mktempdir(; cleanup = true),
            )
            @test build!(sim; console_level = Logging.Error) ==
                  PSI.SimulationBuildStatus.BUILT
            @test execute!(sim; enable_progress_bar = false) ==
                  PSI.RunStatus.SUCCESSFULLY_FINALIZED
        end
    end
end
