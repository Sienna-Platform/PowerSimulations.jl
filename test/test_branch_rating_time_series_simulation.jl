# A branch rating time series is a parameter POM fills at build; between steps PSI must
# re-read it for every branch in the network model's branch catalog.
@testset "Branch rating time series update between simulation steps" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_ed")
    line = first(sort!(collect(get_components(Line, sys)); by = get_name))
    initial_times = collect(PSY.get_forecast_initial_times(sys))
    resolution = first(PSY.get_time_series_resolutions(sys))
    n = Dates.Millisecond(PSY.get_forecast_horizon(sys)) ÷ Dates.Millisecond(resolution)
    # One value per forecast window, so a stale or skipped update is visible. A line has no
    # base power, so its rating series is in MVA.
    rating = PSY.get_rating(line, u"MVA")
    window_value(k) = (0.5 + 0.1 * k) * rating
    data = DataStructures.SortedDict(
        t => fill(window_value(k), n)
        for (k, t) in enumerate(initial_times)
    )
    add_time_series!(sys, line,
        Deterministic("dynamic_line_rating", data, resolution;
            PSB.natural_units("apparent_power", "MVA")...))

    template =
        get_template_nomin_ed_simulation(NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(
        template,
        DeviceModel(Line, StaticBranch;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "dynamic_line_rating",
            )),
    )
    model = DecisionModel(template, sys; name = "ED", optimizer = HiGHS_optimizer)
    models = SimulationModels(; decision_models = [model])
    sim = Simulation(;
        name = "branch_rating_ts",
        steps = 2,
        models,
        sequence = SimulationSequence(; models),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT

    container = PSI.get_optimization_container(model)
    rating_values() =
        PSI.jump_value.(
            IOM.get_parameter_array(container,
                IOM.ParameterKey(BranchRatingTimeSeriesParameter, Line)).data,
        )
    @test all(rating_values() .≈ window_value(1))

    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
    @test all(rating_values() .≈ window_value(2))
end
