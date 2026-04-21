@testset "Post-contingency outage axes — attribute drives inclusion" begin
    scb_formulation = SecurityConstrainedStaticBranch

    # `PSY.UnplannedOutage` is abstract; use the concrete `GeometricDistributionForcedOutage`.
    function _build_mixed_outage_system()
        sys = PSB.build_system(PSITestSystems, "c_sys5")
        lines = collect(PSY.get_components(PSY.Line, sys))
        @assert length(lines) >= 2
        unplanned = PSY.GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
        )
        planned = PSY.PlannedOutage(; outage_schedule = "planned_outage_ts")
        PSY.add_supplemental_attribute!(sys, lines[1], unplanned)
        PSY.add_supplemental_attribute!(sys, lines[2], planned)
        return sys, unplanned, planned
    end

    function _build_model(sys, attrs)
        template = get_thermal_dispatch_template_network(
            NetworkModel(
                PTDFPowerModel;
                use_slacks = false,
                MODF_matrix = PNM.VirtualMODF(sys),
            ),
        )
        set_device_model!(
            template,
            DeviceModel(PSY.Line, scb_formulation; attributes = attrs),
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
        return Set(axes(expr)[1]), Set(axes(cons_ub)[1])
    end

    @testset "default: only UnplannedOutage appears in axes" begin
        sys, unplanned, planned = _build_mixed_outage_system()
        model = _build_model(sys, Dict{String, Any}())  # default attribute

        expr_ax, cons_ax = _axes(model)
        @test expr_ax == cons_ax
        @test expr_ax == Set([string(IS.get_uuid(unplanned))])
        @test !(string(IS.get_uuid(planned)) in expr_ax)
    end

    @testset "include_planned_outages=true: both outages appear in axes" begin
        sys, unplanned, planned = _build_mixed_outage_system()
        model = _build_model(
            sys, Dict{String, Any}("include_planned_outages" => true),
        )

        expr_ax, cons_ax = _axes(model)
        @test expr_ax == cons_ax
        @test expr_ax == Set([
            string(IS.get_uuid(unplanned)),
            string(IS.get_uuid(planned)),
        ])
    end

    @testset "contingency_uuids filter selects a subset" begin
        sys, unplanned, planned = _build_mixed_outage_system()
        model = _build_model(
            sys,
            Dict{String, Any}(
                "include_planned_outages" => true,
                "contingency_uuids" => [IS.get_uuid(unplanned)],
            ),
        )

        expr_ax, cons_ax = _axes(model)
        @test expr_ax == cons_ax
        @test expr_ax == Set([string(IS.get_uuid(unplanned))])
        @test !(string(IS.get_uuid(planned)) in expr_ax)
    end
end
