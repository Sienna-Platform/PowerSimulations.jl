
# Re-attempted on 2026-04-29 after PNM ≥0.21 made VirtualMODF parallel-safe;
# the testset still fails with INFEASIBLE_POINT during optimize!. The original
# `comment out unfeasible test` (commit 5fe9232bc) was a real modeling issue,
# not KLU instability — re-commented and left as a follow-up.

# TEMPORARY CI diagnostic (PR #1619) — trace every KLU solve in this file so the
# last line before a SIGSEGV pinpoints the crashing call. Remove before merge.
PowerNetworkMatrices.KLUWrapper.PNM_KLU_CI_TRACE[] = true

@testset "Security Constrained branch formulation Network DC-PF with VirtualPTDF + auto-MODF" begin
    # Guards against regressions on the threaded Woodbury code path: combining
    # VirtualPTDF with MODF contingency solves has shown KLU-solver instability,
    # so this testset keeps that combination exercised even if other testsets
    # use a concrete PTDF.
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    all_branches = collect(get_components(ACTransmission, c_sys5))
    for line_name in ["1", "2", "3"]
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
            monitored_components = all_branches,
        )
        component = get_component(ACTransmission, c_sys5, line_name)
        add_supplemental_attribute!(c_sys5, component, transition_data)
    end
    template = get_thermal_dispatch_template_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = VirtualPTDF(c_sys5),
            # MODF_matrix intentionally omitted — exercises auto-construction
        ),
    )
    set_device_model!(template, Line, SecurityConstrainedStaticBranch)
    set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
    set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    # MODF should have been auto-populated during build
    nm = PSI.get_network_model(PSI.get_template(ps_model))
    @test !isnothing(PSI.get_MODF_matrix(nm))

    constraint_keys = [
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
    ]
    psi_constraint_test(ps_model, constraint_keys)
end

@testset "Security Constrained branch formulation Network DC-PF with PTDF/MODF Model and parallel lines" begin
    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    parallel_branches_to_add = IdDict{System, Vector{String}}(
        c_sys5 => ["3", "4"],
        c_sys14 => ["Line1", "Line14"],
        c_sys14_dc => ["Line1", "Line14"],
    )
    systems = [c_sys5, c_sys14, c_sys14_dc]
    for sys in systems
        for branch_name in parallel_branches_to_add[sys]
            branch = first(
                get_components(b -> get_name(b) == branch_name, PSY.ACTransmission, sys),
            )
            add_equivalent_ac_transmission_with_parallel_circuits!(
                sys,
                branch,
                typeof(branch),
            )
        end
    end

    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
    ]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5 => PTDF(c_sys5),
        c_sys14 => PTDF(c_sys14),
        c_sys14_dc => PTDF(c_sys14_dc),
    )
    lines_outages = IdDict{System, Vector{String}}(
        c_sys5 => ["1", "2", "3"],
        c_sys14 => ["Line1", "Line2", "Line9", "Line10", "Line12", "Trans2"],
        c_sys14_dc => ["Line9"],
    )

    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 696, 696, 24],
        c_sys14 => [120, 0, 3480, 3480, 24],
        c_sys14_dc => [168, 0, 1080, 984, 24],
    )

    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 306904.39,
        c_sys14 => 159087,
        c_sys14_dc => 154585.1,
    )
    for (ix, sys) in enumerate(systems)
        # outages should be added before to MODF matrix computation
        all_branches = collect(get_components(ACTransmission, sys))
        for line_name in lines_outages[sys]
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = all_branches,
            )
            component = get_component(ACTransmission, sys, line_name)
            add_supplemental_attribute!(sys, component, transition_data)
        end
        template = get_thermal_dispatch_template_network(
            NetworkModel(
                PTDFPowerModel;
                PTDF_matrix = PTDF_ref[sys],
                MODF_matrix = VirtualMODF(sys),
            ),
        )
        set_device_model!(template, Line, SecurityConstrainedStaticBranch)
        set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
        set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)

        moi_tests(ps_model, test_results[sys]..., false)
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        if ix > 2
            continue # skipping test for c_sys14_dc as Highs takes so long to find optimal solution
        end
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "Security Constrained branch formulation Network DC-PF with PTDF/MODF Model and parallel lines removing complete arc" begin
    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    parallel_branches_to_add = IdDict{System, Vector{String}}(
        c_sys5 => ["3", "4"],
        c_sys14 => ["Line1", "Line14"],
        c_sys14_dc => ["Line1", "Line14"],
    )
    systems = [c_sys5, c_sys14, c_sys14_dc]
    for sys in systems
        for branch_name in parallel_branches_to_add[sys]
            branch = first(
                get_components(b -> get_name(b) == branch_name, PSY.ACTransmission, sys),
            )
            add_equivalent_ac_transmission_with_parallel_circuits!(
                sys,
                branch,
                typeof(branch),
            )
        end
    end

    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
    ]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5 => PTDF(c_sys5),
        c_sys14 => PTDF(c_sys14),
        c_sys14_dc => PTDF(c_sys14_dc),
    )
    lines_outages = IdDict{System, Vector{String}}(
        c_sys5 => ["3", "4"],
        c_sys14 => ["Line1", "Line14"],
        c_sys14_dc => ["Line1", "Line14"],
    )

    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 552, 552, 24],
        c_sys14 => [120, 0, 1560, 1560, 24],
        c_sys14_dc => [168, 0, 1512, 1416, 24],
    )

    test_obj_values =
        IdDict{System, Float64}(c_sys5 => 355231, c_sys14 => 159087, c_sys14_dc => 154585.1)
    for (ix, sys) in enumerate(systems)
        # outages should be added before to MODF matrix computation
        all_branches = collect(get_components(ACTransmission, sys))
        for line_name in lines_outages[sys]
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = all_branches,
            )
            component = get_component(ACTransmission, sys, line_name)
            add_supplemental_attribute!(sys, component, transition_data)
            component_parallel = get_component(ACTransmission, sys, line_name * "_copy")
            add_supplemental_attribute!(sys, component_parallel, transition_data)
        end
        template = get_thermal_dispatch_template_network(
            NetworkModel(
                PTDFPowerModel;
                PTDF_matrix = PTDF_ref[sys],
                MODF_matrix = VirtualMODF(sys),
            ),
        )
        set_device_model!(template, Line, SecurityConstrainedStaticBranch)
        set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
        set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)

        moi_tests(ps_model, test_results[sys]..., false)
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        if ix > 2
            continue # skipping test for c_sys14_dc as Highs takes so long to find optimal solution
        end
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "Security Constrained branch formulation Network DC-PF with PTDF/MODF Model and Reductions" begin
    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    parallel_branches_to_add = IdDict{System, Vector{String}}(
        c_sys5 => ["4"],
        c_sys14 => ["Line14"],
        c_sys14_dc => ["Line14"],
    )
    systems = [c_sys5, c_sys14, c_sys14_dc]
    for sys in systems
        for branch_name in parallel_branches_to_add[sys]
            branch = first(
                get_components(b -> get_name(b) == branch_name, PSY.ACTransmission, sys),
            )
            add_equivalent_ac_transmission_with_series_parallel_circuits!(
                sys,
                branch,
                typeof(branch),
            )
        end
    end

    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
    ]

    lines_outages = IdDict{System, Vector{String}}(
        c_sys5 => ["1", "2", "3"],
        c_sys14 => ["Line1", "Line2", "Line9", "Line10", "Line12", "Trans2"],
        c_sys14_dc => ["Line9"],
    )

    # Counts are smaller than the dense-container era because each outage now
    # monitors only its own line (rather than every branch under every outage),
    # which is what allows the degree-two reduction to fire on the rest of the
    # network. Objective values are correspondingly lower (fewer post-contingency
    # constraints → cheaper dispatch).
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 336, 336, 24],
        c_sys14 => [120, 0, 744, 744, 24],
        c_sys14_dc => [168, 0, 672, 576, 24],
    )

    test_obj_values =
        IdDict{System, Float64}(c_sys5 => 241294, c_sys14 => 143365, c_sys14_dc => 154585.1)
    for (ix, sys) in enumerate(systems)
        # In the reduction path each outage monitors only its own outaged line.
        # Monitoring `all_branches` would pin every bus as irreducible and
        # cancel the reduction the test exists to exercise.
        for line_name in lines_outages[sys]
            component = get_component(ACTransmission, sys, line_name)
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = [component],
            )
            add_supplemental_attribute!(sys, component, transition_data)
        end
        nr = NetworkReduction[DegreeTwoReduction()]
        ptdf = PTDF(sys; network_reductions = nr)
        modf = VirtualMODF(sys; network_reductions = nr)
        template = get_thermal_dispatch_template_network(
            NetworkModel(
                PTDFPowerModel;
                PTDF_matrix = ptdf,
                MODF_matrix = modf,
                reduce_degree_two_branches = PNM.has_degree_two_reduction(
                    ptdf.network_reduction_data,
                ),
            ),
        )
        set_device_model!(template, Line, SecurityConstrainedStaticBranch)
        set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
        set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)

        # Sparse-container state on c_sys5: each outage that monitors lines
        # should produce a `PostContingencyFlowRateConstraint`
        # SparseAxisArray whose axis-1 (outage_id) covers exactly that outage
        # set, and whose axis-2 (branch_name) is non-empty per outage. Reductions
        # may redirect individual branch names to a representative reduction name
        # in the container, so we don't assert on per-name keys here — the ground-
        # truth testset later in this file does that structurally.
        if ix == 1
            template_under_test = PSI.get_template(ps_model)
            line_dm = PSI.get_model(template_under_test, PSY.Line)
            line_outages = PSI.get_outages(line_dm)
            @test !isempty(line_outages)
            container = PSI.get_optimization_container(ps_model)
            time_steps = PSI.get_time_steps(container)
            con_ub = PSI.get_constraint(
                container,
                PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
            )
            ub_keys = collect(keys(con_ub.data))
            ub_outages = Set(k[1] for k in ub_keys)
            ub_names = Set(k[2] for k in ub_keys)
            @test !isempty(ub_outages)
            @test !isempty(ub_names)
            # Every outage in the Line DeviceModel's outages dict that has a
            # non-empty Line entry should appear in the constraint container's
            # outage-id axis.
            line_monitoring_outages = Set(
                string(uuid) for (uuid, per_type) in line_outages if
                haskey(per_type, PSY.Line) && !isempty(per_type[PSY.Line])
            )
            @test ub_outages == line_monitoring_outages
            # Each (outage_id, t) combination present in the container must be
            # full-rank in time — i.e. every t in time_steps must have at least
            # one (outage_id, *, t) key.
            for outage_id in ub_outages
                for t in time_steps
                    @test any(k -> k[1] == outage_id && k[3] == t, ub_keys)
                end
            end
        end

        moi_tests(ps_model, test_results[sys]..., false)
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        if ix > 2
            continue # skipping test for c_sys14_dc as Highs takes so long to find optimal solution
        end
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "Security Constrained branch formulation Network DC-PF with PTDF/MODF Model and Reductions with separate monitored lines" begin
    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    parallel_branches_to_add = IdDict{System, Vector{String}}(
        c_sys5 => ["4"],
        c_sys14 => ["Line14"],
        c_sys14_dc => ["Line14"],
    )
    systems = [c_sys5, c_sys14, c_sys14_dc]
    for sys in systems
        for branch_name in parallel_branches_to_add[sys]
            branch = first(
                get_components(b -> get_name(b) == branch_name, PSY.ACTransmission, sys),
            )
            add_equivalent_ac_transmission_with_series_parallel_circuits!(
                sys,
                branch,
                typeof(branch),
            )
        end
    end

    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
    ]

    # Outaged lines - the lines that will be taken out of service
    lines_outages = IdDict{System, Vector{String}}(
        c_sys5 => ["1", "2", "3"],
        c_sys14 => ["Line1", "Line2", "Line9", "Line10", "Line12", "Trans2"],
        c_sys14_dc => ["Line9"],
    )

    # Monitored lines - different lines that will be monitored for post-contingency flows
    # These should be different from the outaged lines to create a non-trivial test
    monitored_lines = IdDict{System, Vector{String}}(
        c_sys5 => ["4", "5", "6"],
        c_sys14 => ["Line3", "Line4", "Line5", "Line6", "Line7", "Line8"],
        c_sys14_dc => ["Line1"],
    )

    # Test results will need to be updated after running the test
    # Placeholder values - these may need adjustment
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 360, 360, 24],
        c_sys14 => [120, 0, 744, 744, 24],
        c_sys14_dc => [168, 0, 672, 576, 24],
    )

    test_obj_values =
        IdDict{System, Float64}(c_sys5 => 355231, c_sys14 => 143365, c_sys14_dc => 154585.1)
    for (ix, sys) in enumerate(systems)
        # Add outages with separate monitored components
        # Each outaged line monitors a different line to create non-trivial constraints
        for (idx, line_name) in enumerate(lines_outages[sys])
            outaged_component = get_component(ACTransmission, sys, line_name)
            monitored_component =
                get_component(ACTransmission, sys, monitored_lines[sys][idx])
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = [monitored_component],
            )
            add_supplemental_attribute!(sys, outaged_component, transition_data)
        end
        nr = NetworkReduction[DegreeTwoReduction()]
        ptdf = PTDF(sys; network_reductions = nr)
        modf = VirtualMODF(sys; network_reductions = nr)
        template = get_thermal_dispatch_template_network(
            NetworkModel(
                PTDFPowerModel;
                PTDF_matrix = ptdf,
                MODF_matrix = modf,
                reduce_degree_two_branches = PNM.has_degree_two_reduction(
                    ptdf.network_reduction_data,
                ),
            ),
        )
        set_device_model!(template, Line, SecurityConstrainedStaticBranch)
        set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
        set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)

        # Verify that monitored lines are different from outaged lines
        if ix == 1
            template_under_test = PSI.get_template(ps_model)
            line_dm = PSI.get_model(template_under_test, PSY.Line)
            line_outages = PSI.get_outages(line_dm)
            @test !isempty(line_outages)
            container = PSI.get_optimization_container(ps_model)
            time_steps = PSI.get_time_steps(container)
            con_ub = PSI.get_constraint(
                container,
                PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
            )
            ub_keys = collect(keys(con_ub.data))
            ub_outages = Set(k[1] for k in ub_keys)
            ub_names = Set(k[2] for k in ub_keys)
            @test !isempty(ub_outages)
            @test !isempty(ub_names)
            line_monitoring_outages = Set(
                string(uuid) for (uuid, per_type) in line_outages if
                haskey(per_type, PSY.Line) && !isempty(per_type[PSY.Line])
            )
            @test ub_outages == line_monitoring_outages
            for outage_id in ub_outages
                for t in time_steps
                    @test any(k -> k[1] == outage_id && k[3] == t, ub_keys)
                end
            end
        end

        moi_tests(ps_model, test_results[sys]..., false)
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        if ix > 2
            continue # skipping test for c_sys14_dc as Highs takes so long to find optimal solution
        end
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "Post-contingency expressions match modf-derived ground truth" begin
    # Validates that every JuMP.AffExpr in the post-contingency expression
    # container equals dot(modf_matrix[arc, ctg], nodal_balance[:, t]).
    # Holds for both the serial and the parallel implementation, so this
    # testset is the regression net for the parallel rewrite in
    # `add_post_contingency_flow_expressions!`.
    c_sys14 = PSB.build_system(PSB.PSITestSystems, "c_sys14")
    outage_line_names = ["Line1", "Line2", "Line9", "Line10"]
    all_branches = collect(get_components(ACTransmission, c_sys14))
    for line_name in outage_line_names
        line = get_component(ACTransmission, c_sys14, line_name)
        transition = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
            monitored_components = all_branches,
        )
        add_supplemental_attribute!(c_sys14, line, transition)
    end

    template = get_thermal_dispatch_template_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = VirtualPTDF(c_sys14),
            MODF_matrix = VirtualMODF(c_sys14),
        ),
    )
    set_device_model!(template, Line, SecurityConstrainedStaticBranch)
    set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
    set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

    ps_model = DecisionModel(template, c_sys14; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    network_model = PSI.get_network_model(PSI.get_template(ps_model))
    @test PSI.get_MODF_matrix(network_model) isa PNM.VirtualMODF

    # Build a SECOND VirtualMODF for the ground-truth column. Reading from a
    # fresh instance (independent KLULinSolvePool, independent Woodbury and
    # row caches) ensures a build-time race that corrupted the production
    # cache cannot pass the test by being read identically here. The fresh
    # MODF auto-registers the same outages from c_sys14's supplemental
    # attributes, so its ContingencySpec UUIDs match the production matrix's.
    ground_truth_modf = PNM.VirtualMODF(c_sys14)
    ground_truth_registered = PNM.get_registered_contingencies(ground_truth_modf)
    @test !isempty(ground_truth_registered)

    nodal_balance = PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus).data
    time_steps = PSI.get_time_steps(container)

    # Walk every (V, (outage_id, name, t)) tuple stored in the sparse
    # PostContingencyBranchFlow container and assert structural equality to
    # the PNM-derived ground truth. V indexes the outaged component type owned
    # by the DeviceModel; the container's `name` axis spans every monitored
    # type, so the arc lookup must scan all per-type reduction maps.
    net_reduction_data = network_model.network_reduction
    modeled_branch_types = network_model.modeled_ac_branch_types
    name_to_arc_maps = PNM.get_name_to_arc_maps(net_reduction_data)
    n_checked = 0
    for V in modeled_branch_types
        PSI.has_container_key(container, PSI.PostContingencyBranchFlow, V) || continue
        pcbf = PSI.get_expression(container, PSI.PostContingencyBranchFlow(), V)
        n_checked += 1
        for (outage_id_str, name, t) in keys(pcbf.data)
            uuid = Base.UUID(outage_id_str)
            ctg = ground_truth_registered[uuid]
            # The monitored component's type is whichever reduction map holds
            # this `name` — names are unique across types within a system.
            arc = nothing
            for n2a in values(name_to_arc_maps)
                if haskey(n2a, name)
                    arc = n2a[name][1]
                    break
                end
            end
            @assert !isnothing(arc) "monitored name $name not found in any \
                                     reduction map"
            modf_col = ground_truth_modf[arc, ctg]
            nz_idx =
                [i for i in eachindex(modf_col) if abs(modf_col[i]) > PSI.PTDF_ZERO_TOL]
            # Mirror `_make_flow_expressions!` exactly (including
            # `get_hinted_aff_expr`) so `JuMP.isequal_canonical` cannot
            # diverge due to internal AffExpr capacity differences.
            expected = PSI.get_hinted_aff_expr(length(nz_idx))
            for i in nz_idx
                JuMP.add_to_expression!(expected, modf_col[i], nodal_balance[i, t])
            end
            actual = pcbf[outage_id_str, name, t]
            @test JuMP.isequal_canonical(actual, expected)
        end
    end
    # Sanity: at least one branch type should have had a container; otherwise
    # the testset would be silently passing with zero structural assertions.
    @test n_checked >= 1
end

@testset "PTDFBranchFlow expressions match ptdf-derived ground truth" begin
    # Validates that every JuMP.AffExpr in the PTDFBranchFlow expression
    # container equals dot(ptdf_matrix[arc, :], nodal_balance[:, t]).
    # Phase A moved the `ptdf[arc, :]` KLU solve INTO the spawned task at
    # `AC_branches.jl:925`; this testset is the regression net for that
    # change, analogous to the post-contingency MODF testset above.
    c_sys14 = PSB.build_system(PSB.PSITestSystems, "c_sys14")

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = VirtualPTDF(c_sys14)),
    )
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, Transformer2W, StaticBranch)
    set_device_model!(template, TapTransformer, StaticBranch)

    ps_model = DecisionModel(template, c_sys14; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    network_model = PSI.get_network_model(PSI.get_template(ps_model))
    @test PSI.get_PTDF_matrix(network_model) isa PNM.VirtualPTDF

    # Fresh VirtualPTDF for the ground-truth column. Independent
    # KLULinSolvePool and row cache from the production matrix, so a
    # build-time race that corrupted the production cache cannot pass the
    # test by being read identically here.
    ground_truth_ptdf = PNM.VirtualPTDF(c_sys14)

    nodal_balance = PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus).data
    time_steps = PSI.get_time_steps(container)

    net_reduction_data = network_model.network_reduction
    modeled_branch_types = network_model.modeled_ac_branch_types

    # Iterate every (V, name, t) tuple in every modeled-branch PTDFBranchFlow
    # container and assert structural equality to the PNM-derived ground truth.
    n_checked = 0
    for V in modeled_branch_types
        PSI.has_container_key(container, PSI.PTDFBranchFlow, V) || continue
        pbf = PSI.get_expression(container, PSI.PTDFBranchFlow(), V)
        name_to_arc_map = collect(PNM.get_name_to_arc_map(net_reduction_data, V))
        isempty(name_to_arc_map) && continue
        n_checked += 1
        for (name, (arc, _)) in name_to_arc_map
            ptdf_col = ground_truth_ptdf[arc, :]
            nz_idx =
                [i for i in eachindex(ptdf_col) if abs(ptdf_col[i]) > PSI.PTDF_ZERO_TOL]
            # Mirror `_make_flow_expressions!` (including
            # `get_hinted_aff_expr`) so `JuMP.isequal_canonical` cannot
            # diverge due to internal AffExpr capacity differences.
            for t in time_steps
                expected = PSI.get_hinted_aff_expr(length(nz_idx))
                for i in nz_idx
                    JuMP.add_to_expression!(expected, ptdf_col[i], nodal_balance[i, t])
                end
                actual = pbf[name, t]
                @test JuMP.isequal_canonical(actual, expected)
            end
        end
    end
    @test n_checked >= 1
end

@testset "PTDFBranchFlow member orientation sign under network reduction" begin
    # Regression: degree-two series-reduction members whose native arc opposes
    # the merged path were reported with the representative's sign.

    # --- Part A: sign helper against real series-reduction data -----------
    sys_red = PSB.build_system(PSB.PSITestSystems, "case10_radial_series_reductions")
    ptdf_red = PTDF(sys_red; network_reductions = NetworkReduction[DegreeTwoReduction()])
    nrd = ptdf_red.network_reduction_data
    PNM.populate_branch_maps_by_type!(nrd)

    # Series segments can be Line OR ThreeWindingTransformerWinding; the sign
    # helper must be queried with the type the name is keyed under.
    tofrom_members = Tuple{DataType, String}[]
    fromto_members = Tuple{DataType, String}[]
    for (T, m) in nrd.name_to_arc_map
        for (name, (arc, red)) in m
            red == "series_branch_map" || continue
            bs = nrd.all_branch_maps_by_type[red][T][arc]
            for (i, seg) in enumerate(bs)
                PNM.get_name(seg) == name || continue
                if bs.segment_orientations[i] == :ToFrom
                    push!(tofrom_members, (T, name))
                else
                    push!(fromto_members, (T, name))
                end
            end
        end
    end
    @test !isempty(tofrom_members)   # guard: fixture must exercise the bug
    for (T, name) in tofrom_members
        @test PSI.get_ptdf_orientation_sign(nrd, T, name) == -1.0
    end
    for (T, name) in fromto_members
        @test PSI.get_ptdf_orientation_sign(nrd, T, name) == 1.0
    end

    # --- Part B: no-op on the unreduced path (c_sys5 has time series) -----
    c_sys5 = PSB.build_system(PSB.PSITestSystems, "c_sys5")
    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = VirtualPTDF(c_sys5)),
    )
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, Transformer2W, StaticBranch)
    set_device_model!(template, TapTransformer, StaticBranch)
    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    network_model = PSI.get_network_model(PSI.get_template(ps_model))
    net_reduction_data = network_model.network_reduction
    ground_truth_ptdf = PNM.VirtualPTDF(c_sys5)
    nodal_balance = PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus).data
    time_steps = PSI.get_time_steps(container)

    n_checked = 0
    for V in network_model.modeled_ac_branch_types
        PSI.has_container_key(container, PSI.PTDFBranchFlow, V) || continue
        pbf = PSI.get_expression(container, PSI.PTDFBranchFlow(), V)
        nta = collect(PNM.get_name_to_arc_map(net_reduction_data, V))
        isempty(nta) && continue
        n_checked += 1
        for (name, (arc, _)) in nta
            @test PSI.get_ptdf_orientation_sign(net_reduction_data, V, name) == 1.0
            ptdf_col = ground_truth_ptdf[arc, :]
            nz_idx =
                [i for i in eachindex(ptdf_col) if abs(ptdf_col[i]) > PSI.PTDF_ZERO_TOL]
            for t in time_steps
                expected = PSI.get_hinted_aff_expr(length(nz_idx))
                for i in nz_idx
                    JuMP.add_to_expression!(expected, ptdf_col[i], nodal_balance[i, t])
                end
                @test JuMP.isequal_canonical(pbf[name, t], expected)
            end
        end
    end
    @test n_checked >= 1
end

@testset "Flow-expression dimension guard converts MODF/PTDF mismatch into a clear error" begin
    # Regression: a MODF column built on a different bus set than the
    # PTDF-reduced nodal-balance expressions used to index out of bounds under
    # `@inbounds` and SIGSEGV. The guard must turn this into a trappable
    # `ErrorException` before the loop.
    nb = JuMP.AffExpr[JuMP.AffExpr(0.0) for _ in 1:3, _ in 1:2]

    # Direct helper: matching dimension is accepted, mismatch errors clearly.
    @test PSI._assert_flow_expression_dimensions("b", 3, nb) === nothing
    err = try
        PSI._assert_flow_expression_dimensions("badbranch", 5, nb)
        nothing
    catch e
        e
    end
    @test err isa ErrorException
    @test occursin("badbranch", err.msg)
    @test occursin("dimension mismatch", err.msg)

    # End-to-end through the dense `_make_flow_expressions!`: a too-long column
    # raises (no `@inbounds` OOB / segfault); a matching column succeeds.
    @test_throws ErrorException PSI._make_flow_expressions!(
        "oob",
        1:2,
        zeros(Float64, 5),
        nb,
    )
    ok_name, ok_expr = PSI._make_flow_expressions!("ok", 1:2, zeros(Float64, 3), nb)
    @test ok_name == "ok"
    @test length(ok_expr) == 2
end

@testset "SecurityConstrainedStaticBranch respects user-supplied outages on DeviceModel" begin
    # Build a system with three line outages, then build two templates against
    # the same system: one with default empty `outages` (auto-discover), one
    # with explicit `outages = [outage_1, outage_2]`. The constraint container
    # axis-1 (outage_id) for the explicit template must equal exactly the two
    # listed UUIDs; the auto-discover template must include all three.
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    all_branches = collect(get_components(ACTransmission, c_sys5))
    outage_components = ["1", "2", "3"]
    outage_uuids = Base.UUID[]
    outage_objs = PSY.Outage[]
    for line_name in outage_components
        component = get_component(ACTransmission, c_sys5, line_name)
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
            monitored_components = all_branches,
        )
        add_supplemental_attribute!(c_sys5, component, transition_data)
        push!(outage_uuids, IS.get_uuid(transition_data))
        push!(outage_objs, transition_data)
    end

    # Auto-discover path: empty `outages` kwarg.
    auto_template = get_thermal_dispatch_template_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(c_sys5),
            MODF_matrix = VirtualMODF(c_sys5),
        ),
    )
    set_device_model!(auto_template, Line, SecurityConstrainedStaticBranch)
    set_device_model!(auto_template, Transformer2W, SecurityConstrainedStaticBranch)
    set_device_model!(auto_template, TapTransformer, SecurityConstrainedStaticBranch)
    auto_model = DecisionModel(auto_template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(auto_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    auto_line_outages =
        PSI.get_outages(PSI.get_model(PSI.get_template(auto_model), PSY.Line))
    @test Set(keys(auto_line_outages)) == Set(outage_uuids)

    # Explicit selection: only outages 1 and 2.
    selected = outage_objs[1:2]
    explicit_template = get_thermal_dispatch_template_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(c_sys5),
            MODF_matrix = VirtualMODF(c_sys5),
        ),
    )
    set_device_model!(
        explicit_template,
        DeviceModel(Line, SecurityConstrainedStaticBranch; outages = selected),
    )
    set_device_model!(explicit_template, Transformer2W, SecurityConstrainedStaticBranch)
    set_device_model!(explicit_template, TapTransformer, SecurityConstrainedStaticBranch)
    explicit_model = DecisionModel(explicit_template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(explicit_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    explicit_line_outages =
        PSI.get_outages(PSI.get_model(PSI.get_template(explicit_model), PSY.Line))
    @test Set(keys(explicit_line_outages)) == Set(outage_uuids[1:2])
    @test !(outage_uuids[3] in keys(explicit_line_outages))

    # Constraint container outage-id axis must reflect the selection.
    container = PSI.get_optimization_container(explicit_model)
    con_ub = PSI.get_constraint(
        container,
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
    )
    ub_outages = Set(k[1] for k in keys(con_ub.data))
    @test ub_outages == Set(string(u) for u in outage_uuids[1:2])
end

@testset "DeviceModel.outages kwarg is dropped with a warning for non-SC formulations" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    line = first(get_components(ACTransmission, c_sys5))
    transition = GeometricDistributionForcedOutage(;
        mean_time_to_recovery = 10,
        outage_transition_probability = 0.9999,
        monitored_components = [line],
    )
    add_supplemental_attribute!(c_sys5, line, transition)
    @test_logs (:warn, r"formulation does not support N-1 contingencies") begin
        dm = DeviceModel(Line, StaticBranch; outages = [transition])
        @test isempty(PSI.get_outages(dm))
    end
end

@testset "Security Constrained branch formulation builds for supported network formulations" begin
    # Every NetworkModel formulation that has a construct_device! dispatch for
    # SecurityConstrainedStaticBranch must build to completion and emit the
    # post-contingency emergency-rate constraint container:
    #   - <:AbstractPTDFModel  → PTDFPowerModel, AreaPTDFPowerModel
    #   - <:PM.AbstractACPModel → ACPPowerModel
    # `two_area_pjm_DA` carries PSY.Area components (required by AreaPTDFPowerModel)
    # and PTDF/ACP build on it without modification.
    sys = PSB.build_system(PSISystems, "two_area_pjm_DA")
    transform_single_time_series!(sys, Hour(24), Hour(1))
    all_branches = collect(get_components(ACTransmission, sys))
    for line in Iterators.take(get_components(PSY.Line, sys), 3)
        add_supplemental_attribute!(
            sys,
            line,
            GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = all_branches,
            ),
        )
    end

    constraint_keys = [
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
    ]
    for (label, NetFormulation, optimizer) in [
        ("PTDFPowerModel", PTDFPowerModel, HiGHS_optimizer),
        ("AreaPTDFPowerModel", AreaPTDFPowerModel, HiGHS_optimizer),
        ("ACPPowerModel", ACPPowerModel, ipopt_optimizer),
    ]
        @testset "$label" begin
            template = get_thermal_dispatch_template_network(
                NetworkModel(NetFormulation; MODF_matrix = VirtualMODF(sys)),
            )
            set_device_model!(template, Line, SecurityConstrainedStaticBranch)
            set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
            set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

            ps_model = DecisionModel(template, sys; optimizer = optimizer)
            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            psi_constraint_test(ps_model, constraint_keys)
        end
    end
end

@testset "SCUC constraint tracking: parallel circuits collapse to representative" begin
    # Targeted regression for the BranchReductionOptimizationTracker under
    # parallel-circuit reduction. After two parallel members collapse to a
    # single reduction representative, the pre-contingency `FlowRateConstraint`
    # tracker must record the arc once (representative-keyed) — never twice
    # under the two individual names. The post-contingency constraint
    # container's name axis must show the representative as well, not the
    # individual branch names. Companion to the existing "PTDF/MODF Model and
    # Reductions" testset, which exercises DegreeTwoReduction with a
    # series-parallel topology and asserts only outage-axis correctness; this
    # one isolates the reduction-tracker mechanics on a pure parallel pair.
    sys = PSB.build_system(PSITestSystems, "c_sys5")
    parallel_line_name = "1"
    parallel_line = first(
        get_components(b -> get_name(b) == parallel_line_name, PSY.ACTransmission, sys),
    )
    add_equivalent_ac_transmission_with_parallel_circuits!(
        sys,
        parallel_line,
        typeof(parallel_line),
    )

    outage = GeometricDistributionForcedOutage(;
        mean_time_to_recovery = 10,
        outage_transition_probability = 0.9999,
        monitored_components = [parallel_line],
    )
    add_supplemental_attribute!(sys, parallel_line, outage)
    outage_uuid = string(IS.get_uuid(outage))

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; MODF_matrix = VirtualMODF(sys)),
    )
    set_device_model!(template, Line, SecurityConstrainedStaticBranch)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    network_model = PSI.get_network_model(PSI.get_template(ps_model))
    tracker = PSI.get_reduced_branch_tracker(network_model)
    net_reduction_data = PSI.get_network_reduction(network_model)

    representative_name = parallel_line_name * "double_circuit"
    c2r = PNM.get_component_to_reduction_name_map(net_reduction_data)
    @test haskey(c2r, PSY.Line)
    @test get(c2r[PSY.Line], parallel_line_name, nothing) == representative_name
    @test get(c2r[PSY.Line], parallel_line_name * "_copy", nothing) == representative_name

    # FlowRateConstraint tracker: representative-name key, no individual names.
    flow_cmap = PSI.get_constraint_map_by_type(tracker)[FlowRateConstraint][PSY.Line]
    @test haskey(flow_cmap, representative_name)
    @test !haskey(flow_cmap, parallel_line_name)
    @test !haskey(flow_cmap, parallel_line_name * "_copy")

    # Post-contingency container: name axis uses the representative; both
    # individual member names are redirected, not stored.
    con_ub = PSI.get_constraint(
        container,
        PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, "ub"),
    )
    name_axis = Set(k[2] for k in keys(con_ub.data))
    @test representative_name in name_axis
    @test !(parallel_line_name in name_axis)
    @test !(parallel_line_name * "_copy" in name_axis)
    @test outage_uuid in Set(k[1] for k in keys(con_ub.data))
end

@testset "Multi-component outage: dual-claim + dedup at build" begin
    # An outage attached to BOTH a Line and a Transformer2W is owned by both
    # `DeviceModel{Line, SC}` and `DeviceModel{Transformer2W, SC}`. The
    # post-contingency build dedups: the second DeviceModel's expression and
    # constraint containers reference the first claimer's `AffExpr` /
    # `ConstraintRef` rather than recomputing the MODF column or issuing a
    # duplicate `@constraint` call.
    sys = PSB.build_system(PSITestSystems, "c_sys14")
    line = first(get_components(PSY.Line, sys))
    transformer = first(get_components(PSY.Transformer2W, sys))
    @test !isnothing(line)
    @test !isnothing(transformer)

    outage = GeometricDistributionForcedOutage(;
        mean_time_to_recovery = 10,
        outage_transition_probability = 0.9999,
        monitored_components = [line, transformer],
    )
    add_supplemental_attribute!(sys, line, outage)
    add_supplemental_attribute!(sys, transformer, outage)
    outage_uuid = IS.get_uuid(outage)
    outage_uuid_str = string(outage_uuid)

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; MODF_matrix = VirtualMODF(sys)),
    )
    set_device_model!(template, Line, SecurityConstrainedStaticBranch)
    set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    template_under_test = PSI.get_template(ps_model)
    line_dm = PSI.get_model(template_under_test, PSY.Line)
    transformer_dm = PSI.get_model(template_under_test, PSY.Transformer2W)

    # Both SC DeviceModels claim the multi-component outage.
    @test haskey(PSI.get_outages(line_dm), outage_uuid)
    @test haskey(PSI.get_outages(transformer_dm), outage_uuid)

    container = PSI.get_optimization_container(ps_model)
    line_pcbf = PSI.get_expression(container, PSI.PostContingencyBranchFlow(), PSY.Line)
    transformer_pcbf =
        PSI.get_expression(container, PSI.PostContingencyBranchFlow(), PSY.Transformer2W)

    line_name = PSY.get_name(line)
    transformer_name = PSY.get_name(transformer)
    time_steps = PSI.get_time_steps(container)

    # Expression-level dedup: the second-claimer's container holds the SAME
    # `AffExpr` object (===) as the first-claimer's, not a re-computed copy.
    for t in time_steps
        @test line_pcbf[outage_uuid_str, line_name, t] ===
              transformer_pcbf[outage_uuid_str, line_name, t]
        @test line_pcbf[outage_uuid_str, transformer_name, t] ===
              transformer_pcbf[outage_uuid_str, transformer_name, t]
    end

    # Constraint-level dedup: same `ConstraintRef` in both per-V containers.
    for meta in ("lb", "ub")
        line_cons = PSI.get_constraint(
            container,
            PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, meta),
        )
        transformer_cons = PSI.get_constraint(
            container,
            PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Transformer2W, meta),
        )
        for t in time_steps
            @test line_cons[outage_uuid_str, line_name, t] ===
                  transformer_cons[outage_uuid_str, line_name, t]
            @test line_cons[outage_uuid_str, transformer_name, t] ===
                  transformer_cons[outage_uuid_str, transformer_name, t]
        end
    end
end

function _attach_all_branch_outages!(sys)
    branches = collect(get_components(ACTransmission, sys))
    for line_name in ("1", "2", "3")
        add_supplemental_attribute!(
            sys,
            get_component(ACTransmission, sys, line_name),
            GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = branches,
            ),
        )
    end
    return sys
end

function _test_post_contingency_line_duals(container)
    duals = PSI.get_duals(container)
    collected = Float64[]
    for meta in ("lb", "ub")
        cons_key = PSI.ConstraintKey(PostContingencyFlowRateConstraint, PSY.Line, meta)
        cons = PSI.get_constraint(container, cons_key)
        @test haskey(duals, cons_key)
        dual = duals[cons_key]
        # Dual container must mirror the sparse constraint's keys exactly.
        @test Set(keys(dual.data)) == Set(keys(cons.data))
        @test !isempty(dual.data)
        @test all(isfinite, values(dual.data))
        append!(collected, values(dual.data))
    end
    # Some post-contingency constraint binds in this congested system, so the
    # duals are not all the zero-initialized default — proves the sparse path
    # actually computed them rather than leaving the container untouched.
    @test any(!iszero, collected)

    # SparseAxisArray duals must also round-trip through `read_duals`.
    dual_frames = PSI.read_duals(container)
    for meta in ("lb", "ub")
        df = dual_frames[PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.Line,
            meta,
        )]
        @test nrow(df) > 0
        @test ncol(df) > 0
        @test all(isfinite, Matrix(df))
    end
end

@testset "Duals of post-contingency flow constraints (sparse dual path)" begin
    # Exercises the sparse `assign_dual_variable!` / `_calculate_dual_variable_value!`
    # path on an LP (thermal dispatch) so HiGHS returns dual values directly.
    c_sys5 = _attach_all_branch_outages!(PSB.build_system(PSITestSystems, "c_sys5"))
    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(c_sys5)),
    )
    set_device_model!(
        template,
        DeviceModel(
            Line,
            SecurityConstrainedStaticBranch;
            duals = [PostContingencyFlowRateConstraint],
        ),
    )
    set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
    set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    _test_post_contingency_line_duals(PSI.get_optimization_container(ps_model))
end

@testset "Duals of post-contingency flow constraints (MILP / unit commitment path)" begin
    # Unit-commitment binaries make this a MILP, so duals go through
    # `process_duals` (relax integers, re-solve LP, copy duals) — the sparse
    # `_copy_dual_values!` path the LP testset above does not reach.
    c_sys5 = _attach_all_branch_outages!(PSB.build_system(PSITestSystems, "c_sys5"))
    template = ProblemTemplate(NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(c_sys5)))
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
    set_device_model!(
        template,
        DeviceModel(
            Line,
            SecurityConstrainedStaticBranch;
            duals = [PostContingencyFlowRateConstraint],
        ),
    )
    set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)
    set_device_model!(template, TapTransformer, SecurityConstrainedStaticBranch)

    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test PSI.is_milp(PSI.get_optimization_container(ps_model))
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    _test_post_contingency_line_duals(PSI.get_optimization_container(ps_model))
end

@testset "Security-constrained formulation rejected for ThreeWindingTransformer" begin
    # SC branch formulations are not implemented yet for ThreeWindingTransformer.
    # Configuring one must raise at template validation, not fail deep in the
    # post-contingency build.
    branch_models = PSI.BranchModelContainer()
    branch_models[nameof(PSY.Transformer3W)] =
        DeviceModel(PSY.Transformer3W, SecurityConstrainedStaticBranch)
    @test_throws IS.ConflictingInputsError PSI._check_security_constrained_three_winding_transformer!(
        branch_models,
    )

    # Allowed combinations must pass: non-SC formulation on a 3WT, and an SC
    # formulation on a supported branch type.
    ok_models = PSI.BranchModelContainer()
    ok_models[nameof(PSY.Transformer3W)] = DeviceModel(PSY.Transformer3W, StaticBranch)
    ok_models[nameof(PSY.Line)] = DeviceModel(PSY.Line, SecurityConstrainedStaticBranch)
    @test isnothing(PSI._check_security_constrained_three_winding_transformer!(ok_models))
end

@testset "SC PTDF/MODF reductions are reconciled to a cohesive bus set" begin
    # Regression: PTDF/MODF supplied with only [Radial, DegreeTwo] (no
    # pre-baked irreducible buses) plus many monitored components can reduce to
    # different bus sets. PSI must reconcile them onto one cohesive reduction
    # so `build!` succeeds without the caller replicating the irreducible-bus
    # computation.
    sys = PSB.build_system(PSB.PSITestSystems, "test_RTS_GMLC_sys")
    all_lines = collect(get_components(Line, sys))
    @test length(all_lines) > 1
    monitored = all_lines                       # force many irreducible buses
    for l in first(all_lines, 5)                # several N-1 contingencies
        add_supplemental_attribute!(
            sys,
            l,
            GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.5,
                monitored_components = monitored,
            ),
        )
    end

    nr = NetworkReduction[RadialReduction(), DegreeTwoReduction()]
    # Caller provides matrices WITHOUT pre-baking irreducible buses.
    ptdf = PTDF(sys; network_reductions = nr)
    modf = VirtualMODF(sys; network_reductions = nr)
    template = get_thermal_dispatch_template_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = ptdf,
            MODF_matrix = modf,
            reduce_radial_branches = true,
            reduce_degree_two_branches = true,
        ),
    )
    set_device_model!(template, Line, SecurityConstrainedStaticBranch)

    model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    nm = PSI.get_network_model(PSI.get_template(model))
    ptdf_retained = PSI._retained_buses(PSI.get_PTDF_matrix(nm).network_reduction_data)
    modf_retained = PSI._retained_buses(PSI.get_MODF_matrix(nm).network_reduction_data)
    @test ptdf_retained == modf_retained
    @test PSI._retained_buses(nm.network_reduction) == modf_retained

    # The container nodal balance must be dimensioned on the same bus set the
    # MODF columns are indexed on (the guard's invariant).
    container = PSI.get_optimization_container(model)
    nodal = PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus)
    @test size(nodal.data, 1) == length(modf_retained)
end

# TEMPORARY CI diagnostic (PR #1619) — disable KLU solve tracing. Remove before merge.
PowerNetworkMatrices.KLUWrapper.PNM_KLU_CI_TRACE[] = false
