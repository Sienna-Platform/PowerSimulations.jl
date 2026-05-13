function check_branch_rating_time_series_flows!(
    res::Union{OptimizationProblemResults, PSI.SimulationProblemResults},
    sys::PSY.System,
    branches_with_rating_ts::Vector{<:AbstractString},
    rating_factors::Vector{Float64},
    add_parallel_line_name::Union{Nothing, AbstractString} = nothing,
)
    for branch_name in branches_with_rating_ts
        branch = get_component(PSY.ACTransmission, sys, branch_name)
        is_parallel_group_flow =
            add_parallel_line_name !== nothing &&
            contains(branch_name, add_parallel_line_name)
        col_key = if is_parallel_group_flow
            replace(branch_name, "_copy" => "") * "double_circuit"
        else
            branch_name
        end

        # For a parallel group the flow is bounded by the group's
        # sum-of-max rating. The test setup (`add_equivalent_ac_transmission_with_parallel_circuits!`)
        # adds a single equal-rating parallel circuit, so the group rating is
        # 2× the original single-branch rating.
        static_rating = get_rating(branch) * get_base_power(sys)
        if is_parallel_group_flow
            static_rating *= 2
        end
        branch_type = string(typeof(branch))
        if typeof(res) <: PSI.SimulationProblemResults
            flow = read_realized_expression(
                res,
                "PTDFBranchFlow__$branch_type";
                table_format = TableFormat.WIDE,
            )[
                :,
                col_key,
            ]
        else
            flow = read_expression(
                res,
                "PTDFBranchFlow__$branch_type";
                table_format = TableFormat.WIDE,
            )[
                :,
                col_key,
            ]
        end
        n_rating = length(rating_factors)
        for (i, f) in enumerate(flow)
            rating_idx = mod1(i, n_rating)
            @test f <= static_rating * rating_factors[rating_idx] + 1e-5
            @test f >= -static_rating * rating_factors[rating_idx] - 1e-5
        end
    end
end

@testset "Network DC-PF with VirtualPTDF Model and implementing branch rating time series" begin
    line_device_model = DeviceModel(
        Line,
        StaticBranch;
        time_series_names = Dict(
            BranchRatingTimeSeriesParameter => "branch_rating",
        ))
    TapTransf_device_model = DeviceModel(
        TapTransformer,
        StaticBranch;
        time_series_names = Dict(
            BranchRatingTimeSeriesParameter => "branch_rating",
        ))
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5 => PTDF(c_sys5),
        c_sys14 => PTDF(c_sys14),
        c_sys14_dc => PTDF(c_sys14_dc),
    )
    branches_with_rating_ts = IdDict{System, Vector{String}}(
        c_sys5 => ["1", "2", "6"],
        c_sys14 => ["Line1", "Line2", "Line9", "Line10", "Line12", "Trans2"],
        c_sys14_dc => ["Line1", "Line9", "Line10", "Line12", "Trans2"],
    )
    rating_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 264, 264, 24],
        c_sys14 => [120, 0, 600, 600, 24],
        c_sys14_dc => [168, 0, 648, 552, 24],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 241293.703,
        c_sys14 => 143365.0,
        c_sys14_dc => 142000.0,
    )
    n_steps = 2
    for (ix, sys) in enumerate(systems)
        add_branch_rating_time_series_to_system!(
            sys,
            branches_with_rating_ts[sys],
            n_steps,
            rating_factors;
            initial_date = "2024-01-01",
        )
        template = get_thermal_dispatch_template_network(
            NetworkModel(
                PTDFPowerModel;
                PTDF_matrix = PTDF_ref[sys],
            ),
        )

        set_device_model!(template, line_device_model)
        set_device_model!(template, TapTransf_device_model)
        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)

        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )

        res = OptimizationProblemResults(ps_model)
        check_branch_rating_time_series_flows!(
            res,
            sys,
            branches_with_rating_ts[sys],
            rating_factors,
            nothing,
        )
    end
end

@testset "Network DC-PF with PTDF Model and implementing branch rating time series with BranchesParallel of different types" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    branches_with_rating_ts = ["1", "2", "6"]
    rating_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    # TimeSeriesBound constraints are correctly applied to parallel arcs shared
    # between different branch types. The mixed parallel group's max rating is
    # the sum of its individual members (`get_sum_of_max_rating`), so adding a
    # parallel copy doubles the group capacity and lowers the optimum cost.
    test_obj_values = [259395.96, 241417.66, 241293.703]
    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without TimeSeriesBounds like TimeSeriesBound
    n_steps = 2

    for slack_flag in [false, true]
        if slack_flag
            test_results = [408, 0, 264, 264, 24]
        else
            test_results = [120, 0, 264, 264, 24]
        end
        line_device_model = DeviceModel(
            Line,
            StaticBranch;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "branch_rating",
            ),
            use_slacks = slack_flag,
        )
        for (ix, add_parallel_line_name) in enumerate(parallel_lines_names_to_add)
            sys = PSB.build_system(PSITestSystems, "c_sys5")
            line_to_add_parallel = get_component(Line, sys, add_parallel_line_name)
            add_equivalent_ac_transmission_with_parallel_circuits!(
                sys,
                line_to_add_parallel,
                PSY.Line,
                PSY.MonitoredLine,
            )

            add_branch_rating_time_series_to_system!(
                sys,
                branches_with_rating_ts,
                n_steps,
                rating_factors;
                initial_date = "2024-01-01",
            )

            template = get_thermal_dispatch_template_network(
                NetworkModel(
                    PTDFPowerModel;
                    PTDF_matrix = PTDF(sys),
                ),
            )
            set_device_model!(template, line_device_model)
            set_device_model!(template, PSY.MonitoredLine, StaticBranch)
            ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            psi_constraint_test(ps_model, constraint_keys)

            moi_tests(
                ps_model,
                test_results...,
                false,
            )
            psi_checkobjfun_test(ps_model, objfuncs[1])
            psi_checksolve_test(
                ps_model,
                [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
                test_obj_values[ix],
                10000,
            )

            res = OptimizationProblemResults(ps_model)
            check_branch_rating_time_series_flows!(
                res,
                sys,
                branches_with_rating_ts,
                rating_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Network DC-PF with PTDF Model and implementing branch rating time series with BranchesParallel of different types (MonitoredLine with TimeSeriesBound)" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]

    rating_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    # Mixed parallel groups use `get_sum_of_max_rating` (sum of branch ratings),
    # so the group capacity is double the single-line case.
    test_obj_values = [259395.96, 240206.07, 241293.703]
    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without TimeSeriesBounds like TimeSeriesBound
    n_steps = 2

    for slack_flag in [false, true]
        if slack_flag
            test_results = [408, 0, 264, 264, 24]
        else
            test_results = [120, 0, 264, 264, 24]
        end
        line_device_model = DeviceModel(
            Line,
            StaticBranch;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "branch_rating",
            ),
            use_slacks = slack_flag,
        )
        for (ix, add_parallel_line_name) in enumerate(parallel_lines_names_to_add)
            sys = PSB.build_system(PSITestSystems, "c_sys5")
            line_to_add_parallel = get_component(Line, sys, add_parallel_line_name)
            add_equivalent_ac_transmission_with_parallel_circuits!(
                sys,
                line_to_add_parallel,
                PSY.Line,
                PSY.MonitoredLine,
            )

            add_branch_rating_time_series_to_system!(
                sys,
                [add_parallel_line_name * "_copy"],
                n_steps,
                rating_factors;
                initial_date = "2024-01-01",
            )

            template = get_thermal_dispatch_template_network(
                NetworkModel(
                    PTDFPowerModel;
                    PTDF_matrix = PTDF(sys),
                ),
            )
            set_device_model!(template, line_device_model)
            set_device_model!(template, PSY.MonitoredLine, StaticBranch)
            ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            psi_constraint_test(ps_model, constraint_keys)

            moi_tests(
                ps_model,
                test_results...,
                false,
            )
            psi_checkobjfun_test(ps_model, objfuncs[1])
            psi_checksolve_test(
                ps_model,
                [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
                test_obj_values[ix],
                10000,
            )

            res = OptimizationProblemResults(ps_model)
            check_branch_rating_time_series_flows!(
                res,
                sys,
                [add_parallel_line_name * "_copy"],
                rating_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Network DC-PF with PTDF Model and implementing branch rating time series with BranchesParallel" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    branches_with_rating_ts = ["1", "2", "6"]
    rating_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    test_obj_values = [356577.0, 279735.0, 241293.703]
    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without TimeSeriesBounds like TimeSeriesBound
    n_steps = 2

    for slack_flag in [false, true]
        if slack_flag
            test_results = [408, 0, 264, 264, 24]
        else
            test_results = [120, 0, 264, 264, 24]
        end
        line_device_model = DeviceModel(
            Line,
            StaticBranch;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "branch_rating",
            ),
            use_slacks = slack_flag,
        )
        for (ix, add_parallel_line_name) in enumerate(parallel_lines_names_to_add)
            sys = PSB.build_system(PSITestSystems, "c_sys5")
            line_to_add_parallel = get_component(Line, sys, add_parallel_line_name)
            add_equivalent_ac_transmission_with_parallel_circuits!(
                sys,
                line_to_add_parallel,
                PSY.Line,
            )

            add_branch_rating_time_series_to_system!(
                sys,
                branches_with_rating_ts,
                n_steps,
                rating_factors;
                initial_date = "2024-01-01",
            )

            template = get_thermal_dispatch_template_network(
                NetworkModel(
                    PTDFPowerModel;
                    PTDF_matrix = PTDF(sys),
                ),
            )
            set_device_model!(template, line_device_model)
            ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            psi_constraint_test(ps_model, constraint_keys)

            moi_tests(
                ps_model,
                test_results...,
                false,
            )
            psi_checkobjfun_test(ps_model, objfuncs[1])
            psi_checksolve_test(
                ps_model,
                [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
                test_obj_values[ix],
                10000,
            )
            res = OptimizationProblemResults(ps_model)
            check_branch_rating_time_series_flows!(
                res,
                sys,
                branches_with_rating_ts,
                rating_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Network DC-PF with PTDF Model and implementing branch rating time series with Reductions" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    branches_with_rating_ts = ["1", "2", "6"]
    rating_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    test_obj_values = [356577.0, 279735.0, 241293.703]
    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without TimeSeriesBounds like TimeSeriesBound
    n_steps = 2
    test_results_slacks = Dict(
        1 => [456, 0, 288, 288, 24],
        2 => [456, 0, 288, 288, 24],
        3 => [408, 0, 264, 264, 24],
    )
    test_results_no_slacks = Dict(
        1 => [120, 0, 288, 288, 24],
        2 => [120, 0, 288, 288, 24],
        3 => [120, 0, 264, 264, 24],
    )

    for slack_flag in [false, true]
        line_device_model = DeviceModel(
            Line,
            StaticBranch;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "branch_rating",
            ),
            use_slacks = slack_flag,
        )
        for (ix, add_parallel_line_name) in enumerate(parallel_lines_names_to_add)
            if slack_flag
                test_results = test_results_slacks[ix]
            else
                test_results = test_results_no_slacks[ix]
            end
            sys = PSB.build_system(PSITestSystems, "c_sys5")

            line_to_add_parallel = get_component(Line, sys, add_parallel_line_name)
            add_equivalent_ac_transmission_with_series_parallel_circuits!(
                sys,
                line_to_add_parallel,
                PSY.Line,
            )

            add_branch_rating_time_series_to_system!(
                sys,
                branches_with_rating_ts,
                n_steps,
                rating_factors;
                initial_date = "2024-01-01",
            )
            nr = NetworkReduction[DegreeTwoReduction()]
            ptdf = PTDF(sys; network_reductions = nr)
            template = get_thermal_dispatch_template_network(
                NetworkModel(
                    PTDFPowerModel;
                    #PTDF_matrix = ptdf,
                    reduce_degree_two_branches = PNM.has_degree_two_reduction(
                        ptdf.network_reduction_data,
                    ),
                ),
            )
            set_device_model!(template, line_device_model)
            ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            psi_constraint_test(ps_model, constraint_keys)

            moi_tests(
                ps_model,
                test_results...,
                false,
            )
            psi_checkobjfun_test(ps_model, objfuncs[1])
            psi_checksolve_test(
                ps_model,
                [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
                test_obj_values[ix],
                10000,
            )
            res = OptimizationProblemResults(ps_model)
            check_branch_rating_time_series_flows!(
                res,
                sys,
                branches_with_rating_ts,
                rating_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Network DC-PF Simulation with PTDF Model and implementing branch rating time series with Reductions" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    branches_with_rating_ts = ["1", "2", "6"]
    rating_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without TimeSeriesBounds like TimeSeriesBound
    n_steps = 2
    test_results_slacks = Dict(
        1 => [600, 0, 288, 288, 24],
        2 => [600, 0, 288, 288, 24],
        3 => [552, 0, 264, 264, 24],
    )
    test_results_no_slacks = Dict(
        1 => [264, 0, 288, 288, 24],
        2 => [264, 0, 288, 288, 24],
        3 => [264, 0, 264, 264, 24],
    )

    for slack_flag in [false, true]
        line_device_model = DeviceModel(
            Line,
            StaticBranch;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "branch_rating",
            ),
            use_slacks = slack_flag,
        )
        for (ix, add_parallel_line_name) in enumerate(parallel_lines_names_to_add)
            if slack_flag
                test_results = test_results_slacks[ix]
            else
                test_results = test_results_no_slacks[ix]
            end

            sys = PSB.build_system(PSITestSystems, "c_sys5")

            line_to_add_parallel = get_component(Line, sys, add_parallel_line_name)
            add_equivalent_ac_transmission_with_series_parallel_circuits!(
                sys,
                line_to_add_parallel,
                PSY.Line,
            )

            add_branch_rating_time_series_to_system!(
                sys,
                branches_with_rating_ts,
                n_steps,
                rating_factors;
                initial_date = "2024-01-01",
            )
            nr = NetworkReduction[DegreeTwoReduction()]
            ptdf = PTDF(sys; network_reductions = nr)
            template = get_thermal_dispatch_template_network(
                NetworkModel(
                    PTDFPowerModel;
                    #PTDF_matrix = ptdf,
                    reduce_degree_two_branches = PNM.has_degree_two_reduction(
                        ptdf.network_reduction_data,
                    ),
                ),
            )
            set_device_model!(template, line_device_model)
            ps_model =
                DecisionModel(template, sys; optimizer = HiGHS_optimizer, name = "UC")

            models = SimulationModels(;
                decision_models = [ps_model],
            )

            DA_sequence = SimulationSequence(;
                models = models,
                ini_cond_chronology = InterProblemChronology(),
            )

            current_date = string(today())
            steps_sim = 2
            sim = Simulation(;
                name = "",
                steps = steps_sim,
                models = models,
                initial_time = DateTime("2024-01-01T00:00:00"),
                sequence = DA_sequence,
                simulation_folder = tempdir())

            @test build!(sim) == PSI.SimulationBuildStatus.BUILT

            @test execute!(sim) ==
                  IS.Simulation.RunStatusModule.RunStatus.SUCCESSFULLY_FINALIZED

            psi_constraint_test(ps_model, constraint_keys)

            moi_tests(
                ps_model,
                test_results...,
                false,
            )
            psi_checkobjfun_test(ps_model, objfuncs[1])

            results = SimulationResults(sim)
            res = get_decision_problem_results(results, "UC")
            check_branch_rating_time_series_flows!(
                res,
                sys,
                branches_with_rating_ts,
                rating_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Branch rating time series formulation validation" begin
    branches_with_rating_ts = ["1", "2", "6"]
    rating_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)
    n_steps = 2

    # Case 1: incompatible formulation (StaticBranchBounds) must raise an error.
    sys_bounds = PSB.build_system(PSITestSystems, "c_sys5")
    add_branch_rating_time_series_to_system!(
        sys_bounds,
        branches_with_rating_ts,
        n_steps,
        rating_factors;
        initial_date = "2024-01-01",
    )
    template_bounds = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys_bounds)),
    )
    set_device_model!(
        template_bounds,
        DeviceModel(
            Line,
            StaticBranchBounds;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "branch_rating",
            ),
        ),
    )
    model_bounds =
        DecisionModel(template_bounds, sys_bounds; optimizer = HiGHS_optimizer)
    @test_throws IS.ConflictingInputsError PSI.validate_template(model_bounds)

    # Case 2: StaticBranchUnbounded with rating time series must only warn, not error.
    sys_unbounded = PSB.build_system(PSITestSystems, "c_sys5")
    add_branch_rating_time_series_to_system!(
        sys_unbounded,
        branches_with_rating_ts,
        n_steps,
        rating_factors;
        initial_date = "2024-01-01",
    )
    template_unbounded = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys_unbounded)),
    )
    set_device_model!(
        template_unbounded,
        DeviceModel(
            Line,
            StaticBranchUnbounded;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "branch_rating",
            ),
        ),
    )
    model_unbounded =
        DecisionModel(template_unbounded, sys_unbounded; optimizer = HiGHS_optimizer)
    @test_logs (:warn, r"StaticBranchUnbounded does not enforce flow limits") match_mode =
        :any PSI.validate_template(model_unbounded)
end
