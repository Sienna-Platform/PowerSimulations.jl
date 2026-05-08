@testset "Post-contingency outage axes — attribute drives inclusion" begin
    scb_formulation = SecurityConstrainedStaticBranch

    # `PSY.UnplannedOutage` is abstract; use the concrete `GeometricDistributionForcedOutage`.
    # The two outages intentionally monitor different-sized sets so the
    # `include_planned_outages=true` testset can assert that sparse containers
    # carry per-outage axes, not a shared global axis.
    function _build_mixed_outage_system()
        sys = PSB.build_system(PSITestSystems, "c_sys5")
        lines = collect(PSY.get_components(PSY.Line, sys))
        @assert length(lines) >= 3
        all_branches = collect(PSY.get_components(PSY.ACTransmission, sys))
        unplanned = PSY.GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
            monitored_components = all_branches,
        )
        planned = PSY.PlannedOutage(;
            outage_schedule = "planned_outage_ts",
            monitored_components = [lines[3]],
        )
        PSY.add_supplemental_attribute!(sys, lines[1], unplanned)
        PSY.add_supplemental_attribute!(sys, lines[2], planned)
        return sys, unplanned, planned
    end

    function _build_model(sys; attributes = Dict{String, Any}(), outages = PSY.Outage[])
        template = get_thermal_dispatch_template_network(
            NetworkModel(
                PTDFPowerModel;
                use_slacks = false,
                MODF_matrix = PNM.VirtualMODF(sys),
            ),
        )
        set_device_model!(
            template,
            DeviceModel(
                PSY.Line, scb_formulation;
                attributes = attributes,
                outages = outages,
            ),
        )
        model = DecisionModel(template, sys)
        @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        return model
    end

    function _axes(model)
        container = PSI.get_optimization_container(model)
        expr = PSI.get_expression(
            container, PSI.PostContingencyBranchFlow(), PSY.Line,
        )
        cons_ub = PSI.get_constraint(
            container,
            PSI.ConstraintKey(
                PSI.PostContingencyEmergencyFlowRateConstraint,
                PSY.Line, "ub",
            ),
        )
        # SparseAxisArray stores tuple keys; project axis 1 (outage_id).
        expr_outages = Set(k[1] for k in keys(expr.data))
        cons_outages = Set(k[1] for k in keys(cons_ub.data))
        return expr_outages, cons_outages
    end

    @testset "default: only UnplannedOutage appears in axes" begin
        sys, unplanned, planned = _build_mixed_outage_system()
        model = _build_model(sys)   # default: empty outages, no planned

        expr_ax, cons_ax = _axes(model)
        @test expr_ax == cons_ax
        @test expr_ax == Set([string(IS.get_uuid(unplanned))])
        @test !(string(IS.get_uuid(planned)) in expr_ax)
    end

    @testset "include_planned_outages=true: both outages appear in axes" begin
        sys, unplanned, planned = _build_mixed_outage_system()
        model = _build_model(
            sys; attributes = Dict{String, Any}("include_planned_outages" => true),
        )

        expr_ax, cons_ax = _axes(model)
        @test expr_ax == cons_ax
        @test expr_ax == Set([
            string(IS.get_uuid(unplanned)),
            string(IS.get_uuid(planned)),
        ])

        # Different-size monitored sets: unplanned monitors all `ACTransmission`
        # branches in c_sys5 (6 lines, all on distinct arcs), planned monitors
        # one. Sparse containers must carry per-outage branch sets — not a
        # shared global axis — so the (outage, branch_name) projection sizes
        # differ between the two outages.
        container = PSI.get_optimization_container(model)
        expr = PSI.get_expression(
            container, PSI.PostContingencyBranchFlow(), PSY.Line,
        )
        unplanned_id = string(IS.get_uuid(unplanned))
        planned_id = string(IS.get_uuid(planned))
        unplanned_branches =
            Set(k[2] for k in keys(expr.data) if k[1] == unplanned_id)
        planned_branches =
            Set(k[2] for k in keys(expr.data) if k[1] == planned_id)
        @test length(unplanned_branches) == 6
        @test length(planned_branches) == 1
        @test length(unplanned_branches) != length(planned_branches)
    end

    @testset "outages kwarg selects a subset" begin
        # Replaces the legacy `contingency_uuids` attribute filter. Explicit
        # `outages = [unplanned]` restricts the model to that one outage,
        # bypassing the `include_planned_outages` type filter.
        sys, unplanned, planned = _build_mixed_outage_system()
        model = _build_model(sys; outages = [unplanned])

        expr_ax, cons_ax = _axes(model)
        @test expr_ax == cons_ax
        @test expr_ax == Set([string(IS.get_uuid(unplanned))])
        @test !(string(IS.get_uuid(planned)) in expr_ax)
    end
end
