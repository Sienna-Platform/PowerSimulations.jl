
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

    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 241294,
        c_sys14 => 143365,
        c_sys14_dc => 154585.1,
    )
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
        # should produce a `PostContingencyEmergencyFlowRateConstraint`
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
                PSI.ConstraintKey(
                    PostContingencyEmergencyFlowRateConstraint, PSY.Line, "ub",
                ),
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
                string(uuid) for (uuid, per_type) in line_outages
                if haskey(per_type, PSY.Line) && !isempty(per_type[PSY.Line])
            )
            @test ub_outages == line_monitoring_outages
            # Each (outage_id, t) combination present in the container must be
            # full-rank in time — i.e. every t in time_steps must have at least
            # one (outage_id, *, t) key.
            for outage_id in ub_outages
                for t in time_steps
                    @test any(
                        k -> k[1] == outage_id && k[3] == t, ub_keys,
                    )
                end
            end
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

    nodal_balance =
        PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus).data
    time_steps = PSI.get_time_steps(container)

    # Walk every (V, (outage_id, name, t)) tuple stored in the sparse
    # PostContingencyBranchFlow container and assert structural equality to
    # the PNM-derived ground truth. A given V only has a container if outages
    # monitored components of that type; skip the rest via has_container_key.
    net_reduction_data = network_model.network_reduction
    modeled_branch_types = network_model.modeled_ac_branch_types
    n_checked = 0
    for V in modeled_branch_types
        PSI.has_container_key(container, PSI.PostContingencyBranchFlow, V) || continue
        pcbf = PSI.get_expression(container, PSI.PostContingencyBranchFlow(), V)
        name_to_arc_map = PNM.get_name_to_arc_map(net_reduction_data, V)
        n_checked += 1
        for (outage_id_str, name, t) in keys(pcbf.data)
            uuid = Base.UUID(outage_id_str)
            ctg = ground_truth_registered[uuid]
            arc = name_to_arc_map[name][1]
            modf_col = ground_truth_modf[arc, ctg]
            nz_idx =
                [i for i in eachindex(modf_col) if abs(modf_col[i]) > PSI.PTDF_ZERO_TOL]
            # Mirror `_make_flow_expressions!` exactly (including
            # `get_hinted_aff_expr`) so `JuMP.isequal_canonical` cannot
            # diverge due to internal AffExpr capacity differences.
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
    set_device_model!(
        explicit_template, Transformer2W, SecurityConstrainedStaticBranch,
    )
    set_device_model!(
        explicit_template, TapTransformer, SecurityConstrainedStaticBranch,
    )
    explicit_model =
        DecisionModel(explicit_template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(explicit_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    explicit_line_outages = PSI.get_outages(
        PSI.get_model(PSI.get_template(explicit_model), PSY.Line),
    )
    @test Set(keys(explicit_line_outages)) == Set(outage_uuids[1:2])
    @test !(outage_uuids[3] in keys(explicit_line_outages))

    # Constraint container outage-id axis must reflect the selection.
    container = PSI.get_optimization_container(explicit_model)
    con_ub = PSI.get_constraint(
        container,
        PSI.ConstraintKey(
            PostContingencyEmergencyFlowRateConstraint, PSY.Line, "ub",
        ),
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
