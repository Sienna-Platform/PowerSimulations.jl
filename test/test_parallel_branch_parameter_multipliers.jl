@testset "Parallel-branch multiplier rows are populated per-branch (no NaN)" begin
    # Regression coverage for the multiplier-axis bug where parallel branches
    # sharing a time-series UUID had their multipliers written to the wrong row
    # of the device-name-keyed multiplier array, leaving the other branch's row
    # at the construction-time NaN fill.
    line_device_model = DeviceModel(
        Line,
        StaticBranch;
        time_series_names = Dict(
            DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
        ),
    )

    branches_dlr = ["1", "2", "6"]
    dlr_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    for parallel_line_name in ["1", "2", "3"]
        sys = PSB.build_system(PSITestSystems, "c_sys5")
        line_to_add_parallel = get_component(Line, sys, parallel_line_name)
        add_equivalent_ac_transmission_with_parallel_circuits!(
            sys,
            line_to_add_parallel,
            PSY.Line,
        )
        add_dlr_to_system_branches!(
            sys,
            branches_dlr,
            2,
            dlr_factors;
            initial_date = "2024-01-01",
        )

        template = get_thermal_dispatch_template_network(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
        )
        set_device_model!(template, line_device_model)
        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT

        container = PSI.get_optimization_container(ps_model)
        param_key = PSI.ParameterKey(DynamicBranchRatingTimeSeriesParameter, Line)
        param_container = PSI.get_parameter(container, param_key)
        mult_array = PSI.get_multiplier_array(param_container)
        device_name_axis = axes(mult_array)[1]

        # Every device row in the multiplier array must be fully populated for
        # branches that actually carry the time series — no rows left at the
        # NaN sentinel from the fill! at construction time.
        for name in device_name_axis
            row = mult_array[name, :]
            @test !any(isnan, row)
        end
    end
end
