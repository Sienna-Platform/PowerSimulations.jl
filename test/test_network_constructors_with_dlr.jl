function check_dlr_branch_flows!(
    res::Union{OptimizationProblemResults, PSI.SimulationProblemResults},
    sys::PSY.System,
    branches_dlr::Vector{<:AbstractString},
    dlr_factors::Vector{Float64},
    add_parallel_line_name::Union{Nothing, AbstractString} = nothing,
)
    for branch_name in branches_dlr
        branch = get_component(PSY.ACTransmission, sys, branch_name)
        col_key =
            if (
                add_parallel_line_name !== nothing &&
                contains(branch_name, add_parallel_line_name)
            )
                replace(branch_name, "_copy" => "") * "double_circuit"
            else
                branch_name
            end

        static_rating = get_rating(branch) * get_base_power(sys)
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
        n_dlr = length(dlr_factors)
        for (i, f) in enumerate(flow)
            dlr_idx = mod1(i, n_dlr)
            @test f <= static_rating * dlr_factors[dlr_idx] + 1e-5
            @test f >= -static_rating * dlr_factors[dlr_idx] - 1e-5
        end
    end
end

@testset "Network DC-PF with VirtualPTDF Model and implementing Dynamic Branch Ratings" begin
    line_device_model = DeviceModel(
        Line,
        StaticBranch;
        time_series_names = Dict(
            DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
        ))
    TapTransf_device_model = DeviceModel(
        TapTransformer,
        StaticBranch;
        time_series_names = Dict(
            DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
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
    branches_dlr = IdDict{System, Vector{String}}(
        c_sys5 => ["1", "2", "6"],
        c_sys14 => ["Line1", "Line2", "Line9", "Line10", "Line12", "Trans2"],
        c_sys14_dc => ["Line1", "Line9", "Line10", "Line12", "Trans2"],
    )
    dlr_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)
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
        add_dlr_to_system_branches!(
            sys,
            branches_dlr[sys],
            n_steps,
            dlr_factors;
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
        check_dlr_branch_flows!(res, sys, branches_dlr[sys], dlr_factors, nothing)
    end
end

@testset "Network DC-PF with PTDF Model and implementing Dynamic Branch Ratings with BranchesParallel of different types" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    branches_dlr = ["1", "2", "6"]
    dlr_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    # DLR constraints are now correctly applied to parallel arcs shared between different branch types.
    # The first two cases (parallel on lines "1" and "2") have DLR, resulting in a higher optimal cost.
    test_obj_values = [375109.0, 320486.0, 241293.703]
    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without DLRs
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
                DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
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

            add_dlr_to_system_branches!(
                sys,
                branches_dlr,
                n_steps,
                dlr_factors;
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
            check_dlr_branch_flows!(
                res,
                sys,
                branches_dlr,
                dlr_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Network DC-PF with PTDF Model and implementing Dynamic Branch Ratings with BranchesParallel of different types (MonitoredLine with DLR)" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]

    dlr_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    test_obj_values = [375109.0, 320486.0, 241293.703]
    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without DLRs
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
                DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
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

            add_dlr_to_system_branches!(
                sys,
                [add_parallel_line_name * "_copy"],
                n_steps,
                dlr_factors;
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
            check_dlr_branch_flows!(
                res,
                sys,
                [add_parallel_line_name * "_copy"],
                dlr_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Network DC-PF with PTDF Model and implementing Dynamic Branch Ratings with BranchesParallel" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    branches_dlr = ["1", "2", "6"]
    dlr_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    test_obj_values = [356577.0, 279735.0, 241293.703]
    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without DLRs
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
                DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
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

            add_dlr_to_system_branches!(
                sys,
                branches_dlr,
                n_steps,
                dlr_factors;
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
            check_dlr_branch_flows!(
                res,
                sys,
                branches_dlr,
                dlr_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Network DC-PF with PTDF Model and implementing Dynamic Branch Ratings with Reductions" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    branches_dlr = ["1", "2", "6"]
    dlr_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    test_obj_values = [356577.0, 279735.0, 241293.703]
    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without DLRs
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
                DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
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

            add_dlr_to_system_branches!(
                sys,
                branches_dlr,
                n_steps,
                dlr_factors;
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
            check_dlr_branch_flows!(
                res,
                sys,
                branches_dlr,
                dlr_factors,
                add_parallel_line_name,
            )
        end
    end
end

@testset "Network DC-PF Simulation with PTDF Model and implementing Dynamic Branch Ratings with Reductions" begin
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
    ]
    branches_dlr = ["1", "2", "6"]
    dlr_factors = vcat([fill(x, 6) for x in [0.99, 0.98, 1.0, 0.95]]...)

    parallel_lines_names_to_add = ["1", "2", "3"]#Add parallel lines in lines with and without DLRs
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
                DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
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

            add_dlr_to_system_branches!(
                sys,
                branches_dlr,
                n_steps,
                dlr_factors;
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
            check_dlr_branch_flows!(
                res,
                sys,
                branches_dlr,
                dlr_factors,
                add_parallel_line_name,
            )
        end
    end
end
