@testset "DC Power Flow Models Monitored Line Flow Constraints and Static Unbounded" begin
    system = PSB.build_system(PSITestSystems, "c_sys5_ml")
    limits = PSY.get_flow_limits(PSY.get_component(MonitoredLine, system, "1"))
    for model in [DCPPowerModel, PTDFPowerModel]
        template = get_thermal_dispatch_template_network(
            NetworkModel(model; PTDF_matrix = PTDF(system)),
        )
        model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        @test check_variable_bounded(model_m, FlowActivePowerVariable, MonitoredLine)

        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
        @test check_flow_variable_values(
            model_m,
            FlowActivePowerVariable,
            MonitoredLine,
            "1",
            limits.from_to,
        )
    end
end

@testset "AC Power Flow Monitored Line Flow Constraints" begin
    system = PSB.build_system(PSITestSystems, "c_sys5_ml")
    limits = PSY.get_flow_limits(PSY.get_component(MonitoredLine, system, "1"))
    template = get_thermal_dispatch_template_network(ACPPowerModel)
    model_m = DecisionModel(template, system; optimizer = ipopt_optimizer)
    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    @test check_variable_bounded(model_m, FlowActivePowerFromToVariable, MonitoredLine)
    @test check_variable_unbounded(model_m, FlowReactivePowerFromToVariable, MonitoredLine)

    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    @test check_flow_variable_values(
        model_m,
        FlowActivePowerFromToVariable,
        FlowReactivePowerFromToVariable,
        MonitoredLine,
        "1",
        0.0,
        limits.from_to,
    )
end

@testset "DC Power Flow Models Monitored Line Flow Constraints and Static with inequalities" begin
    system = PSB.build_system(PSITestSystems, "c_sys5_ml")
    set_rating!(PSY.get_component(Line, system, "2"), 1.5)
    for model in [DCPPowerModel, PTDFPowerModel]
        template = get_thermal_dispatch_template_network(
            NetworkModel(model; PTDF_matrix = PTDF(system)),
        )
        set_device_model!(template, DeviceModel(Line, StaticBranch))
        set_device_model!(template, DeviceModel(MonitoredLine, StaticBranchUnbounded))
        model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT

        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
        @test check_flow_variable_values(model_m, FlowActivePowerVariable, Line, "2", 1.5)
    end
end

@testset "DC Power Flow Models Monitored Line Flow Constraints and Static with Bounds" begin
    system = PSB.build_system(PSITestSystems, "c_sys5_ml")
    set_rating!(PSY.get_component(Line, system, "2"), 1.5)
    for model in [DCPPowerModel, PTDFPowerModel]
        template = get_thermal_dispatch_template_network(NetworkModel(model))
        set_device_model!(template, DeviceModel(Line, StaticBranchBounds))
        set_device_model!(template, DeviceModel(MonitoredLine, StaticBranchUnbounded))
        model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT

        @test check_variable_bounded(model_m, FlowActivePowerVariable, Line)

        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
        @test check_flow_variable_values(model_m, FlowActivePowerVariable, Line, "2", 1.5)
    end

    # Test the addition of slacks
    template = get_thermal_dispatch_template_network(NetworkModel(PTDFPowerModel))
    set_device_model!(template, DeviceModel(Line, StaticBranchBounds; use_slacks = true))
    set_device_model!(
        template,
        DeviceModel(MonitoredLine, StaticBranchBounds; use_slacks = true),
    )
    model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    @test check_variable_bounded(model_m, FlowActivePowerVariable, Line)
    @test check_variable_bounded(model_m, FlowActivePowerVariable, MonitoredLine)
    @test !check_variable_bounded(model_m, FlowActivePowerSlackLowerBound, Line)
    @test !check_variable_bounded(model_m, FlowActivePowerSlackUpperBound, Line)
    @test !check_variable_bounded(model_m, FlowActivePowerSlackLowerBound, MonitoredLine)
    @test !check_variable_bounded(model_m, FlowActivePowerSlackUpperBound, MonitoredLine)

    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "DC Power Flow Models for TwoTerminalGenericHVDCLine  with with Line Flow Constraints, TapTransformer & Transformer2W Unbounded" begin
    ratelimit_constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, Transformer2W, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, Transformer2W, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, TapTransformer, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, TapTransformer, "lb"),
    ]

    system = PSB.build_system(PSITestSystems, "c_sys14_dc")
    hvdc_line = PSY.get_component(TwoTerminalGenericHVDCLine, system, "DCLine3")
    limits_from = PSY.get_active_power_limits_from(hvdc_line)
    limits_to = PSY.get_active_power_limits_to(hvdc_line)
    limits_min = min(limits_from.min, limits_to.min)
    limits_max = min(limits_from.max, limits_to.max)

    tap_transformer = PSY.get_component(TapTransformer, system, "Trans3")
    rate_limit = PSY.get_rating(tap_transformer)

    transformer = PSY.get_component(Transformer2W, system, "Trans4")
    rate_limit2w = PSY.get_rating(tap_transformer)

    for model in [DCPPowerModel, PTDFPowerModel]
        template = get_template_dispatch_with_network(
            NetworkModel(model),
        )
        set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)
        set_device_model!(template, DeviceModel(Transformer2W, StaticBranch))
        set_device_model!(template, DeviceModel(TapTransformer, StaticBranch))
        model_m = DecisionModel(template, system; optimizer = ipopt_optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT

        psi_constraint_test(model_m, ratelimit_constraint_keys)

        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

        @test check_flow_variable_values(
            model_m,
            FlowActivePowerVariable,
            TwoTerminalGenericHVDCLine,
            "DCLine3",
            limits_min,
            limits_max,
        )
        @test check_flow_variable_values(
            model_m,
            FlowActivePowerVariable,
            TapTransformer,
            "Trans3",
            -rate_limit,
            rate_limit,
        )
        @test check_flow_variable_values(
            model_m,
            FlowActivePowerVariable,
            Transformer2W,
            "Trans4",
            -rate_limit2w,
            rate_limit2w,
        )
    end
end

@testset "DC Power Flow Models for Unbounded TwoTerminalGenericHVDCLine , and StaticBranchBounds for TapTransformer & Transformer2W" begin
    system = PSB.build_system(PSITestSystems, "c_sys14_dc")
    hvdc_line = PSY.get_component(TwoTerminalGenericHVDCLine, system, "DCLine3")
    limits_from = PSY.get_active_power_limits_from(hvdc_line)
    limits_to = PSY.get_active_power_limits_to(hvdc_line)
    limits_min = min(limits_from.min, limits_to.min)
    limits_max = min(limits_from.max, limits_to.max)

    tap_transformer = PSY.get_component(TapTransformer, system, "Trans3")
    rate_limit = PSY.get_rating(tap_transformer)

    transformer = PSY.get_component(Transformer2W, system, "Trans4")
    rate_limit2w = PSY.get_rating(tap_transformer)

    for model in [DCPPowerModel, PTDFPowerModel]
        template = get_template_dispatch_with_network(
            NetworkModel(model; PTDF_matrix = PTDF(system)),
        )
        set_device_model!(
            template,
            DeviceModel(TwoTerminalGenericHVDCLine, HVDCTwoTerminalUnbounded),
        )
        set_device_model!(template, DeviceModel(TapTransformer, StaticBranchBounds))
        set_device_model!(template, DeviceModel(Transformer2W, StaticBranchBounds))
        model_m = DecisionModel(template, system; optimizer = ipopt_optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT

        @test check_variable_unbounded(
            model_m,
            FlowActivePowerVariable,
            TwoTerminalGenericHVDCLine,
        )
        @test check_variable_bounded(model_m, FlowActivePowerVariable, TapTransformer)
        @test check_variable_bounded(model_m, FlowActivePowerVariable, TapTransformer)

        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

        @test check_flow_variable_values(
            model_m,
            FlowActivePowerVariable,
            TwoTerminalGenericHVDCLine,
            "DCLine3",
            limits_min,
            limits_max,
        )
        @test check_flow_variable_values(
            model_m,
            FlowActivePowerVariable,
            TapTransformer,
            "Trans3",
            -rate_limit,
            rate_limit,
        )
        @test check_flow_variable_values(
            model_m,
            FlowActivePowerVariable,
            Transformer2W,
            "Trans4",
            -rate_limit2w,
            rate_limit2w,
        )
    end
end

@testset "HVDCTwoTerminalLossless values check between network models" begin
    # Test to compare lossless models with lossless formulation
    sys_5 = build_system(PSITestSystems, "c_sys5_uc")

    line = get_component(Line, sys_5, "1")
    remove_component!(sys_5, line)

    hvdc = TwoTerminalGenericHVDCLine(;
        name = get_name(line),
        available = true,
        active_power_flow = 0.0,
        # Force the flow in the opposite direction for testing purposes
        active_power_limits_from = (min = -0.5, max = -0.5),
        active_power_limits_to = (min = -3.0, max = 2.0),
        reactive_power_limits_from = (min = -1.0, max = 1.0),
        reactive_power_limits_to = (min = -1.0, max = 1.0),
        arc = get_arc(line),
        loss = LinearCurve(0.0),
    )

    add_component!(sys_5, hvdc)

    template_uc = ProblemTemplate(
        NetworkModel(PTDFPowerModel),
    )

    set_device_model!(template_uc, ThermalStandard, ThermalStandardUnitCommitment)
    set_device_model!(template_uc, RenewableDispatch, FixedOutput)
    set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
    set_device_model!(template_uc, DeviceModel(Line, StaticBranch))
    set_device_model!(
        template_uc,
        DeviceModel(TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless),
    )

    model = DecisionModel(
        template_uc,
        sys_5;
        name = "UC",
        optimizer = HiGHS_optimizer,
    )
    build!(model; output_dir = mktempdir())

    solve!(model)

    ptdf_vars =
        read_variables(OptimizationProblemResults(model); table_format = TableFormat.WIDE)
    ptdf_values = ptdf_vars["FlowActivePowerVariable__TwoTerminalGenericHVDCLine"]
    ptdf_objective = PSI.get_optimization_container(model).optimizer_stats.objective_value

    set_network_model!(template_uc, NetworkModel(DCPPowerModel))

    model = DecisionModel(
        template_uc,
        sys_5;
        name = "UC",
        optimizer = HiGHS_optimizer,
    )

    solve!(model; output_dir = mktempdir())
    dcp_vars =
        read_variables(OptimizationProblemResults(model); table_format = TableFormat.WIDE)
    dcp_values = dcp_vars["FlowActivePowerVariable__TwoTerminalGenericHVDCLine"]
    dcp_objective =
        PSI.get_optimization_container(model).optimizer_stats.objective_value

    @test isapprox(dcp_objective, ptdf_objective; atol = 0.1)
    # Resulting solution is in the 4e5 order of magnitude
    @test all(isapprox.(ptdf_values[!, "1"], dcp_values[!, "1"]; atol = 10))
end

@testset "HVDCDispatch Model Tests" begin
    # Test to compare lossless models with lossless formulation
    sys_5 = build_system(PSITestSystems, "c_sys5_uc")
    # Revert to previous rating before data change to prevent different optimal solutions for the lossless model and lossless formulation:
    PSY.set_rating!(PSY.get_component(PSY.Line, sys_5, "6"), 2.0)

    line = get_component(Line, sys_5, "1")
    remove_component!(sys_5, line)

    hvdc = TwoTerminalGenericHVDCLine(;
        name = get_name(line),
        available = true,
        active_power_flow = 0.0,
        # Force the flow in the opposite direction for testing purposes
        active_power_limits_from = (min = -2.0, max = 2.0),
        active_power_limits_to = (min = -2.0, max = 2.0),
        reactive_power_limits_from = (min = -1.0, max = 1.0),
        reactive_power_limits_to = (min = -1.0, max = 1.0),
        arc = get_arc(line),
        loss = LinearCurve(0.0),
    )

    add_component!(sys_5, hvdc)
    for net_model in [DCPPowerModel, PTDFPowerModel]
        @testset "$net_model" begin
            PSY.set_loss!(hvdc, PSY.LinearCurve(0.0))
            template_uc = ProblemTemplate(
                NetworkModel(net_model; use_slacks = true),
            )

            set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
            set_device_model!(template_uc, RenewableDispatch, FixedOutput)
            set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
            set_device_model!(template_uc, DeviceModel(Line, StaticBranchBounds))
            set_device_model!(
                template_uc,
                DeviceModel(TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless),
            )

            model_ref = DecisionModel(
                template_uc,
                sys_5;
                name = "UC",
                optimizer = HiGHS_optimizer,
                store_variable_names = true,
            )

            solve!(model_ref; output_dir = mktempdir())
            ref_vars = read_variables(
                OptimizationProblemResults(model_ref);
                table_format = TableFormat.WIDE,
            )
            ref_values = ref_vars["FlowActivePowerVariable__Line"]
            hvdc_ref_values =
                ref_vars["FlowActivePowerVariable__TwoTerminalGenericHVDCLine"]
            ref_objective = model_ref.internal.container.optimizer_stats.objective_value
            ref_total_gen = sum(
                sum.(
                    eachrow(
                        DataFrames.select(
                            ref_vars["ActivePowerVariable__ThermalStandard"],
                            Not(:DateTime),
                        ),
                    )
                ),
            )
            set_device_model!(
                template_uc,
                DeviceModel(TwoTerminalGenericHVDCLine, HVDCTwoTerminalDispatch),
            )

            model = DecisionModel(
                template_uc,
                sys_5;
                name = "UC",
                optimizer = HiGHS_optimizer,
            )

            solve!(model; output_dir = mktempdir())
            no_loss_vars = read_variables(
                OptimizationProblemResults(model);
                table_format = TableFormat.WIDE,
            )
            no_loss_values = no_loss_vars["FlowActivePowerVariable__Line"]
            hvdc_ft_no_loss_values =
                no_loss_vars["FlowActivePowerFromToVariable__TwoTerminalGenericHVDCLine"]
            hvdc_tf_no_loss_values =
                no_loss_vars["FlowActivePowerToFromVariable__TwoTerminalGenericHVDCLine"]
            no_loss_objective =
                PSI.get_optimization_container(model).optimizer_stats.objective_value
            no_loss_total_gen = sum(
                sum.(
                    eachrow(
                        DataFrames.select(
                            no_loss_vars["ActivePowerVariable__ThermalStandard"],
                            Not(:DateTime),
                        ),
                    ),
                ),
            )

            @test isapprox(no_loss_objective, ref_objective; atol = 0.1)

            for col in names(ref_values)
                test_result =
                    all(isapprox.(ref_values[!, col], no_loss_values[!, col]; atol = 0.1))
                @test test_result
                test_result || break
            end

            @test all(
                isapprox.(
                    hvdc_ft_no_loss_values[!, "1"],
                    -hvdc_tf_no_loss_values[!, "1"];
                    atol = 1e-3,
                ),
            )

            @test isapprox(no_loss_total_gen, ref_total_gen; atol = 0.1)

            PSY.set_loss!(hvdc, PSY.LinearCurve(0.005, 0.1))

            model_wl = DecisionModel(
                template_uc,
                sys_5;
                name = "UC",
                optimizer = HiGHS_optimizer,
            )

            solve!(model_wl; output_dir = mktempdir())
            dispatch_vars = read_variables(
                OptimizationProblemResults(model_wl);
                table_format = TableFormat.WIDE,
            )
            dispatch_values_ft =
                dispatch_vars["FlowActivePowerFromToVariable__TwoTerminalGenericHVDCLine"]
            dispatch_values_tf =
                dispatch_vars["FlowActivePowerToFromVariable__TwoTerminalGenericHVDCLine"]
            wl_total_gen = sum(
                sum.(
                    eachrow(
                        DataFrames.select(
                            dispatch_vars["ActivePowerVariable__ThermalStandard"],
                            Not(:DateTime),
                        ),
                    ),
                ),
            )
            dispatch_objective = model_wl.internal.container.optimizer_stats.objective_value

            # Note: for this test data the system does better by allowing more losses so
            # the total cost is lower.
            @test wl_total_gen > no_loss_total_gen

            for col in names(dispatch_values_tf)
                test_result = all(dispatch_values_tf[!, col] .<= dispatch_values_ft[!, col])
                @test test_result
                test_result || break
            end
        end
    end
end

@testset "DC Power Flow Models for TwoTerminalGenericHVDCLine  Dispatch and TapTransformer & Transformer2W Unbounded" begin
    ratelimit_constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraint, Transformer2W, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, Line, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, TapTransformer, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, Transformer2W, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, TapTransformer, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, TwoTerminalGenericHVDCLine, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, TwoTerminalGenericHVDCLine, "lb"),
    ]

    system = PSB.build_system(PSITestSystems, "c_sys14_dc")

    hvdc_line = PSY.get_component(TwoTerminalGenericHVDCLine, system, "DCLine3")
    limits_from = PSY.get_active_power_limits_from(hvdc_line)
    limits_to = PSY.get_active_power_limits_to(hvdc_line)
    limits_min = min(limits_from.min, limits_to.min)
    limits_max = min(limits_from.max, limits_to.max)

    tap_transformer = PSY.get_component(TapTransformer, system, "Trans3")
    rate_limit = PSY.get_rating(tap_transformer)

    transformer = PSY.get_component(Transformer2W, system, "Trans4")
    rate_limit2w = PSY.get_rating(tap_transformer)

    template = get_template_dispatch_with_network(
        NetworkModel(PTDFPowerModel),
    )
    set_device_model!(template, DeviceModel(TapTransformer, StaticBranch))
    set_device_model!(template, DeviceModel(Transformer2W, StaticBranch))
    set_device_model!(
        template,
        DeviceModel(TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless),
    )
    model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    psi_constraint_test(model_m, ratelimit_constraint_keys)

    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    @test check_flow_variable_values(
        model_m,
        FlowActivePowerVariable,
        TwoTerminalGenericHVDCLine,
        "DCLine3",
        limits_max,
    )
    @test check_flow_variable_values(
        model_m,
        FlowActivePowerVariable,
        TapTransformer,
        "Trans3",
        rate_limit,
    )
    @test check_flow_variable_values(
        model_m,
        FlowActivePowerVariable,
        Transformer2W,
        "Trans4",
        rate_limit2w,
    )
end

@testset "DC Power Flow Models for PhaseShiftingTransformer and Line" begin
    system = build_system(PSITestSystems, "c_sys5_uc")

    line = get_component(Line, system, "1")
    remove_component!(system, line)

    ps = PhaseShiftingTransformer(;
        name = get_name(line),
        available = true,
        active_power_flow = 0.0,
        reactive_power_flow = 0.0,
        r = get_r(line),
        x = get_r(line),
        primary_shunt = 0.0,
        tap = 1.0,
        α = 0.0,
        rating = get_rating(line),
        arc = get_arc(line),
        base_power = get_base_power(system),
    )

    add_component!(system, ps)

    template = get_template_dispatch_with_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(system)),
    )
    set_device_model!(template, DeviceModel(PhaseShiftingTransformer, PhaseAngleControl))
    model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    @test check_variable_unbounded(
        model_m,
        FlowActivePowerVariable,
        PhaseShiftingTransformer,
    )

    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    @test check_flow_variable_values(
        model_m,
        FlowActivePowerVariable,
        PhaseShiftingTransformer,
        "1",
        get_rating(ps),
    )

    @test check_flow_variable_values(
        model_m,
        PhaseShifterAngle,
        PhaseShiftingTransformer,
        "1",
        -π / 2,
        π / 2,
    )
end

@testset "AC Power Flow Models for TwoTerminalGenericHVDCLine  Flow Constraints and TapTransformer & Transformer2W Unbounded" begin
    ratelimit_constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraintFromTo, Transformer2W),
        PSI.ConstraintKey(FlowRateConstraintToFrom, Transformer2W),
        PSI.ConstraintKey(FlowRateConstraintFromTo, TapTransformer),
        PSI.ConstraintKey(FlowRateConstraintToFrom, TapTransformer),
        PSI.ConstraintKey(FlowRateConstraint, TwoTerminalGenericHVDCLine, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, TwoTerminalGenericHVDCLine, "lb"),
    ]

    system = PSB.build_system(PSITestSystems, "c_sys14_dc")

    hvdc_line = PSY.get_component(TwoTerminalGenericHVDCLine, system, "DCLine3")
    limits_from = PSY.get_active_power_limits_from(hvdc_line)
    limits_to = PSY.get_active_power_limits_to(hvdc_line)
    limits_min = min(limits_from.min, limits_to.min)
    limits_max = min(limits_from.max, limits_to.max)

    tap_transformer = PSY.get_component(TapTransformer, system, "Trans3")
    rate_limit = PSY.get_rating(tap_transformer)

    transformer = PSY.get_component(Transformer2W, system, "Trans4")
    rate_limit2w = PSY.get_rating(tap_transformer)

    template = get_template_dispatch_with_network(ACPPowerModel)
    set_device_model!(template, TapTransformer, StaticBranchBounds)
    set_device_model!(template, Transformer2W, StaticBranchBounds)
    set_device_model!(
        template,
        DeviceModel(TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless),
    )
    model_m = DecisionModel(template, system; optimizer = ipopt_optimizer)
    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test check_variable_bounded(model_m, FlowActivePowerFromToVariable, TapTransformer)
    @test check_variable_unbounded(model_m, FlowReactivePowerFromToVariable, TapTransformer)
    @test check_variable_bounded(model_m, FlowActivePowerToFromVariable, Transformer2W)
    @test check_variable_unbounded(model_m, FlowReactivePowerToFromVariable, Transformer2W)

    psi_constraint_test(model_m, ratelimit_constraint_keys)

    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    @test check_flow_variable_values(
        model_m,
        FlowActivePowerVariable,
        FlowReactivePowerToFromVariable,
        TwoTerminalGenericHVDCLine,
        "DCLine3",
        limits_max,
    )
    @test check_flow_variable_values(
        model_m,
        FlowActivePowerFromToVariable,
        FlowReactivePowerFromToVariable,
        TapTransformer,
        "Trans3",
        rate_limit,
    )
    @test check_flow_variable_values(
        model_m,
        FlowActivePowerToFromVariable,
        FlowReactivePowerToFromVariable,
        Transformer2W,
        "Trans4",
        rate_limit2w,
    )
end

@testset "Test Line and Monitored Line models with slacks" begin
    system = PSB.build_system(PSITestSystems, "c_sys5_ml")
    # This rating (0.247479) was previously inferred in PSY.check_component after setting the rating to 0.0 in the tests
    set_rating!(PSY.get_component(Line, system, "2"), 0.247479)
    for (model, optimizer) in NETWORKS_FOR_TESTING
        if model ∈ [PM.SDPWRMPowerModel, SOCWRConicPowerModel]
            # Skip because the data is too in the feasibility margins for these models
            continue
        end
        template = get_thermal_dispatch_template_network(
            NetworkModel(model; use_slacks = true),
        )
        set_device_model!(template, DeviceModel(Line, StaticBranch; use_slacks = true))
        set_device_model!(
            template,
            DeviceModel(MonitoredLine, StaticBranch; use_slacks = true),
        )
        model_m = DecisionModel(template, system; optimizer = optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
        res = OptimizationProblemResults(model_m)
        vars = read_variable(
            res,
            "FlowActivePowerSlackUpperBound__Line";
            table_format = TableFormat.WIDE,
        )
        # some relaxations will find a solution with 0.0 slack
        @test sum(vars[!, "2"]) >= -1e-6
    end

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; use_slacks = true),
    )
    set_device_model!(template, DeviceModel(Line, StaticBranchBounds; use_slacks = true))
    set_device_model!(
        template,
        DeviceModel(MonitoredLine, StaticBranchBounds; use_slacks = true),
    )
    model_m = DecisionModel(template, system; optimizer = fast_ipopt_optimizer)
    @test build!(
        model_m;
        console_level = Logging.AboveMaxLevel,
        output_dir = mktempdir(; cleanup = true),
    ) == PSI.ModelBuildStatus.BUILT

    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    res = OptimizationProblemResults(model_m)
    vars = read_variable(
        res,
        "FlowActivePowerSlackUpperBound__Line";
        table_format = TableFormat.WIDE,
    )
    # some relaxations will find a solution with 0.0 slack
    @test sum(vars[!, "2"]) >= -1e-6

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; use_slacks = true),
    )
    set_device_model!(template, DeviceModel(Line, StaticBranch; use_slacks = true))
    set_device_model!(
        template,
        DeviceModel(MonitoredLine, StaticBranch; use_slacks = true),
    )
    model_m = DecisionModel(template, system; optimizer = fast_ipopt_optimizer)
    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    res = OptimizationProblemResults(model_m)
    vars = read_variable(
        res,
        "FlowActivePowerSlackUpperBound__Line";
        table_format = TableFormat.WIDE,
    )
    # some relaxations will find a solution with 0.0 slack
    @test sum(vars[!, "2"]) >= -1e-6
end

@testset "Three Winding Transformer Test - Basic Setup and Model" begin
    # Start with the base system
    system = PSB.build_system(PSITestSystems, "c_sys5_ml")
    busD = PSY.get_component(ACBus, system, "nodeD")
    # Create a new bus for the tertiary winding (connected via transformer to Bus 4)
    new_bus1 = ACBus(;
        number = 101,
        name = "Bus3WT_1",
        available = true,
        bustype = ACBusTypes.PQ,
        angle = 0.0,
        magnitude = 1.0,
        voltage_limits = (min = 0.95, max = 1.05),
        base_voltage = 230.0,
        area = PSY.get_area(busD),
        load_zone = PSY.get_load_zone(busD),
    )
    PSY.add_component!(system, new_bus1)

    new_bus2 = ACBus(;
        number = 102,
        name = "Bus3WT_2",
        available = true,
        bustype = ACBusTypes.PQ,
        angle = 0.0,
        magnitude = 1.0,
        voltage_limits = (min = 0.95, max = 1.05),
        base_voltage = 230.0,
        area = PSY.get_area(busD),
        load_zone = PSY.get_load_zone(busD),
    )
    PSY.add_component!(system, new_bus2)

    # Add a new load at the new bus
    new_load = PowerLoad(;
        name = "Load_Bus3WT",
        available = true,
        bus = new_bus1,
        active_power = 0.5,
        reactive_power = 0.1,
        base_power = 100.0,
        max_active_power = 0.5,
        max_reactive_power = 0.1,
    )
    PSY.add_component!(system, new_load)

    # Add a new generator at the new bus to provide power
    new_gen = ThermalStandard(;
        name = "Gen_Bus100",
        available = true,
        status = true,
        bus = new_bus2,
        active_power = 0.4,
        reactive_power = 0.0,
        rating = 0.5,
        prime_mover_type = PrimeMovers.ST,
        fuel = ThermalFuels.COAL,
        active_power_limits = (min = 0.0, max = 0.5),
        reactive_power_limits = (min = -0.3, max = 0.3),
        ramp_limits = (up = 0.5, down = 0.5),
        operation_cost = ThermalGenerationCost(;
            variable = CostCurve(LinearCurve(0.0)),
            start_up = 0.0,
            shut_down = 0.0,
            fixed = 0.0,
        ),
        base_power = 100.0,
        time_limits = nothing,
    )
    PSY.add_component!(system, new_gen)

    # Create a star bus for the Transformer3W
    star_bus = ACBus(;
        number = 103,
        name = "Star_Bus_T3W",
        available = true,
        bustype = ACBusTypes.PQ,
        angle = 0.0,
        magnitude = 1.0,
        voltage_limits = (min = 0.95, max = 1.05),
        base_voltage = 230.0,
        area = PSY.get_area(busD),
        load_zone = PSY.get_load_zone(busD),
    )
    PSY.add_component!(system, star_bus)

    transformer3w = Transformer3W(;
        name = "Transformer3W_busD",
        available = true,
        primary_star_arc = Arc(; from = busD, to = star_bus),
        secondary_star_arc = Arc(; from = new_bus1, to = star_bus),
        tertiary_star_arc = Arc(; from = new_bus2, to = star_bus),
        star_bus = star_bus,
        active_power_flow_primary = 0.0,
        reactive_power_flow_primary = 0.0,
        active_power_flow_secondary = 0.0,
        reactive_power_flow_secondary = 0.0,
        active_power_flow_tertiary = 0.0,
        reactive_power_flow_tertiary = 0.0,
        # Star-to-winding impedances
        r_primary = 0.01,
        x_primary = 0.1,
        r_secondary = 0.01,
        x_secondary = 0.1,
        r_tertiary = 0.01,
        x_tertiary = 0.1,
        # Winding-to-winding impedances
        r_12 = 0.01,
        x_12 = 0.1,
        r_23 = 0.01,
        x_23 = 0.1,
        r_13 = 0.01,
        x_13 = 0.1,
        # Base powers for each winding pair
        base_power_12 = 100.0,
        base_power_23 = 100.0,
        base_power_13 = 100.0,
        # Ratings for each winding
        rating = nothing,
        rating_primary = 1.0,
        rating_secondary = 1.0,
        rating_tertiary = 0.5,
    )
    PSY.add_component!(system, transformer3w)

    # Add Transformer3W device model when available
    # Test with DC Power Flow Model
    for net_model in [DCPPowerModel, PTDFPowerModel]
        template = get_template_dispatch_with_network(
            NetworkModel(net_model; PTDF_matrix = PTDF(system)),
        )
        # Set device model for Transformer3W
        set_device_model!(template, DeviceModel(Transformer3W, StaticBranch))
        set_device_model!(template, MonitoredLine, StaticBranch)

        model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT

        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

        # Test flow constraints
        transformer = PSY.get_component(Transformer3W, system, "Transformer3W_busD")
        @test check_flow_variable_values(
            model_m,
            FlowActivePowerVariable,
            Transformer3W,
            "Transformer3W_busD_winding_3",
            PSY.get_rating_tertiary(transformer),
        )
    end

    template_ac = get_thermal_dispatch_template_network(ACPPowerModel)
    set_device_model!(template_ac, DeviceModel(Transformer3W, StaticBranch))
    model_ac = DecisionModel(template_ac, system; optimizer = ipopt_optimizer)
    @test build!(model_ac; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model_ac) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end
