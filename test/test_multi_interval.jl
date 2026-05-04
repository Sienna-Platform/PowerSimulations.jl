@testset "Multi-interval validation errors" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5"; add_single_time_series = true)
    transform_single_time_series!(c_sys5, Hour(24), Hour(12); delete_existing = false)
    transform_single_time_series!(c_sys5, Hour(24), Hour(24); delete_existing = false)

    template = get_thermal_dispatch_template_network()

    # Without interval kwarg on a multi-interval system, should error
    @test_throws IS.ConflictingInputsError DecisionModel(
        template,
        c_sys5;
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
    )

    # With a non-existent interval on a system that has no SingleTimeSeries to auto-transform from, should error.
    c_sys5_forecasts = PSB.build_system(PSITestSystems, "c_sys5"; add_forecasts = true)
    @test_throws IS.ConflictingInputsError DecisionModel(
        template,
        c_sys5_forecasts;
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(6),
    )
end

@testset "DecisionModel with explicit interval" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5"; add_single_time_series = true)
    transform_single_time_series!(c_sys5, Hour(24), Hour(12); delete_existing = false)
    transform_single_time_series!(c_sys5, Hour(24), Hour(24); delete_existing = false)

    template = get_thermal_dispatch_template_network()

    # Build with interval = 24h
    model_24h = DecisionModel(
        template,
        c_sys5;
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(24),
    )
    @test build!(model_24h; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model_24h) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    # Build with interval = 12h on the same system
    model_12h = DecisionModel(
        template,
        c_sys5;
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(12),
    )
    @test build!(model_12h; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model_12h) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    # Verify models have the correct interval in store params
    @test PSI.get_interval(model_24h) == Dates.Millisecond(Hour(24))
    @test PSI.get_interval(model_12h) == Dates.Millisecond(Hour(12))
end

@testset "Auto-transform SingleTimeSeries with interval" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5"; add_single_time_series = true)
    template = get_thermal_dispatch_template_network()

    model = DecisionModel(
        template,
        c_sys5;
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(24),
    )
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    # Second model with a different interval reuses the same system and auto-transforms.
    model2 = DecisionModel(
        template,
        c_sys5;
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(12),
    )
    @test build!(model2; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model2) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "RTS system shared across two intervals - build only" begin
    sys_rts = PSB.build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    # Clear any pre-existing transform so we can attach two fresh intervals.
    PSY.transform_single_time_series!(sys_rts, Hour(24), Hour(24); delete_existing = true)
    PSY.transform_single_time_series!(sys_rts, Hour(24), Hour(12); delete_existing = false)

    template = get_template_standard_uc_simulation()
    set_network_model!(template, NetworkModel(CopperPlatePowerModel))

    model_24h = DecisionModel(
        template,
        sys_rts;
        name = "UC_24h",
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(24),
    )
    @test build!(model_24h; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test PSI.get_interval(model_24h) == Dates.Millisecond(Hour(24))

    model_12h = DecisionModel(
        template,
        sys_rts;
        name = "UC_12h",
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(12),
    )
    @test build!(model_12h; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test PSI.get_interval(model_12h) == Dates.Millisecond(Hour(12))

    # Same underlying system, different intervals selected per model.
    @test get_system(model_24h) === get_system(model_12h)
end

@testset "Single interval system works without interval kwarg" begin
    # Backward compatibility: existing single-interval systems work without changes
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    template = get_thermal_dispatch_template_network()
    model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end
