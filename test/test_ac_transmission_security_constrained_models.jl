
@testset "Security Constrained branch formulation Network DC-PF with PTDF/MODF Model" begin
    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "ub"),
    ]
    lines_outages = IdDict{System, Vector{String}}(
        c_sys5 => ["3"],
        c_sys14 => ["Line1", "Line2", "Line9", "Line10", "Line12", "Trans2"],
        c_sys14_dc => ["Line1", "Line9", "Line10", "Line12", "Trans2"],
    )
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 408, 408, 24],
        c_sys14 => [120, 0, 3480, 3480, 24],
        c_sys14_dc => [168, 0, 2808, 2712, 24],
    )

    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 297424,
        c_sys14 => 152839.4,
        c_sys14_dc => 154585.1,
    )
    for (ix, sys) in enumerate(systems)
        if ix == 1
            # skipping c_sys5 outage of line 3 as it creates a degenerate MODF matrix that causes KLU to fail with a numerical error
            continue
        end
        # outages should be added before to MODF matrix computation
        for line_name in lines_outages[sys]
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
            )
            component = get_component(ACTransmission, sys, line_name)
            add_supplemental_attribute!(sys, component, transition_data)
        end
        template = get_thermal_dispatch_template_network(
            NetworkModel(
                PTDFPowerModel;
                PTDF_matrix = PTDF(sys),
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

        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
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

@testset "Security Constrained branch formulation Network DC-PF with VirtualPTDF + auto-MODF" begin
    # Guards against regressions on the threaded Woodbury code path: combining
    # VirtualPTDF with MODF contingency solves has shown KLU-solver instability,
    # so this testset keeps that combination exercised even if other testsets
    # use a concrete PTDF.
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    for line_name in ["1", "2", "3"]
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
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
    @test PSI.get_MODF_matrix(nm) !== nothing

    constraint_keys = [
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "ub"),
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
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "ub"),
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
        for line_name in lines_outages[sys]
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
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

        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
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
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "ub"),
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

    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 355231,
        c_sys14 => 159087,
        c_sys14_dc => 154585.1,
    )
    for (ix, sys) in enumerate(systems)
        # outages should be added before to MODF matrix computation
        for line_name in lines_outages[sys]
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
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

        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
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
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PostContingencyEmergencyFlowRateConstraint, PSY.Line, "ub"),
    ]

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
        c_sys5 => 355231,
        c_sys14 => 159087,
        c_sys14_dc => 154585.1,
    )
    for (ix, sys) in enumerate(systems)
        # outages should be added before to MODF matrix computation
        for line_name in lines_outages[sys]
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
            )
            component = get_component(ACTransmission, sys, line_name)
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

        # Tracker-state assertions on c_sys5 only: every arc registered in a
        # per-type submap must appear exactly once in the dedupe set, so that
        # constraint containers cannot silently pick up duplicate arcs under
        # a reduction.
        if ix == 1
            nm = PSI.get_network_model(PSI.get_template(ps_model))
            tracker = PSI.get_reduced_branch_tracker(nm)
            c_dict = PSI.get_constraint_dict(tracker)
            c_map = PSI.get_constraint_map_by_type(tracker)
            pcfr = PostContingencyEmergencyFlowRateConstraint
            @test haskey(c_dict, pcfr)
            @test haskey(c_map, pcfr)
            post_arc_set = c_dict[pcfr]
            post_type_map = c_map[pcfr]
            for (_, submap) in post_type_map
                for (_, (arc, _)) in submap
                    @test arc in post_arc_set
                end
            end
            @test haskey(post_type_map, PSY.Line)
            @test !isempty(post_type_map[PSY.Line])
            total_submap_entries = sum(length(v) for v in values(post_type_map))
            @test total_submap_entries == length(post_arc_set)
        end

        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
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
