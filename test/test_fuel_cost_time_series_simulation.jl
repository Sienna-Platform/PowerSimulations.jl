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
