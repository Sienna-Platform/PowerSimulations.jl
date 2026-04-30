
# Re-attempted on 2026-04-29 after PNM ≥0.21 made VirtualMODF parallel-safe;
# the testset still fails with INFEASIBLE_POINT during optimize!. The original
# `comment out unfeasible test` (commit 5fe9232bc) was a real modeling issue,
# not KLU instability — re-commented and left as a follow-up.

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

@testset "Post-contingency expressions match modf-derived ground truth" begin
    # Validates that every JuMP.AffExpr in the post-contingency expression
    # container equals dot(modf_matrix[arc, ctg], nodal_balance[:, t]).
    # Holds for both the serial and the parallel implementation, so this
    # testset is the regression net for the parallel rewrite in
    # `add_post_contingency_flow_expressions!`.
    c_sys14 = PSB.build_system(PSB.PSITestSystems, "c_sys14")
    outage_line_names = ["Line1", "Line2", "Line9", "Line10"]
    for line_name in outage_line_names
        line = get_component(ACTransmission, c_sys14, line_name)
        transition = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
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

    nodal_balance =
        PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus).data
    time_steps = PSI.get_time_steps(container)

    # Reproduce branch_type_data the same way add_post_contingency_flow_expressions!
    # does, so we can resolve (name -> arc) for every entry in the container.
    reduced_branch_tracker = PSI.get_reduced_branch_tracker(network_model)
    modeled_branch_types = network_model.modeled_ac_branch_types
    constraint_map = PSI.get_constraint_map_by_type(reduced_branch_tracker)[
        PostContingencyEmergencyFlowRateConstraint,
    ]

    # Iterate every (V, outage, name, t) tuple in every modeled-branch container
    # and assert structural equality to the PNM-derived ground truth. A given V
    # only has a PostContingencyBranchFlow container if outages were registered
    # for components of type V; skip the rest via has_container_key.
    n_checked = 0
    for V in modeled_branch_types
        haskey(constraint_map, V) || continue
        PSI.has_container_key(container, PSI.PostContingencyBranchFlow, V) || continue
        name_to_arc_map = constraint_map[V]
        pcbf = PSI.get_expression(container, PSI.PostContingencyBranchFlow(), V)
        n_checked += 1
        for outage_id_str in pcbf.axes[1]
            uuid = Base.UUID(outage_id_str)
            ctg = ground_truth_registered[uuid]
            for (name, (arc, _)) in name_to_arc_map
                modf_col = ground_truth_modf[arc, ctg]
                nz_idx =
                    [i for i in eachindex(modf_col) if abs(modf_col[i]) > PSI.PTDF_ZERO_TOL]
                # Mirror `_make_flow_expressions!` exactly (including
                # `get_hinted_aff_expr`) so `JuMP.isequal_canonical` cannot
                # diverge due to internal AffExpr capacity differences.
                for t in time_steps
                    expected = PSI.get_hinted_aff_expr(length(nz_idx))
                    for i in nz_idx
                        JuMP.add_to_expression!(
                            expected,
                            modf_col[i],
                            nodal_balance[i, t],
                        )
                    end
                    actual = pcbf[outage_id_str, name, t]
                    @test JuMP.isequal_canonical(actual, expected)
                end
            end
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
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = VirtualPTDF(c_sys14),
        ),
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

    nodal_balance =
        PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus).data
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
                    JuMP.add_to_expression!(
                        expected,
                        ptdf_col[i],
                        nodal_balance[i, t],
                    )
                end
                actual = pbf[name, t]
                @test JuMP.isequal_canonical(actual, expected)
            end
        end
    end
    @test n_checked >= 1
end
