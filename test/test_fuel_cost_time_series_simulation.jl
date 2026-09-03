# Regression test for the FuelCostParameter time-series update path
# (src/parameters/update_cost_parameters.jl): a thermal unit whose FuelCurve carries a
# time-varying fuel cost must get a fresh FuelCostParameter read every simulation step, not
# a value frozen at build time.
@testset "FuelCostParameter time series updates across simulation steps" begin
    sys = deepcopy(PSB.build_system(PSITestSystems, "c_sys5_uc"))
    thermal = first(PSY.get_components(PSY.ThermalStandard, sys))

    # Mirror an existing Deterministic forecast's windows/resolution/interval exactly, so
    # the new "fuel_cost" series is compatible with the rest of the system's forecasts (a
    # System cannot mix Deterministic and DeterministicSingleTimeSeries) and is read with
    # the same horizon the model already uses for the load.
    load = first(PSY.get_components(PSY.PowerLoad, sys))
    load_ts = PSY.get_time_series(PSY.Deterministic, load, "max_active_power")
    windows = collect(pairs(get_data(load_ts)))
    horizon_count = length(last(first(windows)))

    # Distinct values per window/step, so a frozen (never-updated) parameter would read
    # back step 1's values at step 2 and fail the comparison below.
    fuel_cost_data = OrderedDict{DateTime, Vector{Float64}}(
        dt => Float64.((100 * step_ix) .+ (1:horizon_count))
        for (step_ix, (dt, _)) in enumerate(windows)
    )
    fuel_ts = PSY.Deterministic(;
        name = "fuel_cost",
        data = fuel_cost_data,
        resolution = IS.get_resolution(load_ts),
        interval = IS.get_interval(load_ts),
    )
    fuel_key = PSY.add_time_series!(sys, thermal, fuel_ts)

    old_cost = PSY.get_operation_cost(thermal)
    PSY.set_operation_cost!(
        thermal,
        PSY.ThermalGenerationCost(;
            variable_operation_cost = PSY.FuelCurve(PSY.LinearCurve(0.0, 5.0), fuel_key),
            fixed = PSY.get_fixed(old_cost),
            start_up = PSY.get_start_up(old_cost),
            shut_down = PSY.get_shut_down(old_cost),
        ),
    )

    template = template_unit_commitment(; network = CopperPlateNetworkModel)
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
