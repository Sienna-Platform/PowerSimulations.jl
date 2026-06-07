@testset "Renewable DCPLossless FullDispatch" begin
    device_model = DeviceModel(RenewableDispatch, RenewableFullDispatch)
    c_sys5_re = PSB.build_system(PSITestSystems, "c_sys5_re")
    model = DecisionModel(MockOperationProblem, DCPPowerModel, c_sys5_re)
    mock_construct_device!(model, device_model)
    moi_tests(model, 72, 0, 72, 0, 0, false)
    psi_checkobjfun_test(model, GAEVF)
    model = DecisionModel(MockOperationProblem, DCPPowerModel, c_sys5_re)
    mock_construct_device!(model, device_model; add_event_model = true)
    moi_tests(model, 72, 0, 96, 0, 0, false)
end

@testset "Renewable ACPPower Full Dispatch" begin
    device_model = DeviceModel(RenewableDispatch, RenewableFullDispatch)
    c_sys5_re = PSB.build_system(PSITestSystems, "c_sys5_re")
    model = DecisionModel(MockOperationProblem, ACPPowerModel, c_sys5_re;)
    mock_construct_device!(model, device_model)
    moi_tests(model, 144, 0, 144, 72, 0, false)
    psi_checkobjfun_test(model, GAEVF)
    model = DecisionModel(MockOperationProblem, ACPPowerModel, c_sys5_re;)
    mock_construct_device!(model, device_model; add_event_model = true)
    moi_tests(model, 144, 0, 168, 72, 0, false, 24)
end

@testset "Renewable DCPLossless Constantpower_factor" begin
    device_model = DeviceModel(RenewableDispatch, RenewableConstantPowerFactor)
    c_sys5_re = PSB.build_system(PSITestSystems, "c_sys5_re")
    model = DecisionModel(MockOperationProblem, DCPPowerModel, c_sys5_re)
    mock_construct_device!(model, device_model)
    moi_tests(model, 72, 0, 72, 0, 0, false)
    psi_checkobjfun_test(model, GAEVF)
    model = DecisionModel(MockOperationProblem, DCPPowerModel, c_sys5_re)
    mock_construct_device!(model, device_model; add_event_model = true)
    moi_tests(model, 72, 0, 96, 0, 0, false)
end

@testset "Renewable ACPPower Constantpower_factor" begin
    device_model = DeviceModel(RenewableDispatch, RenewableConstantPowerFactor)
    c_sys5_re = PSB.build_system(PSITestSystems, "c_sys5_re")
    model = DecisionModel(MockOperationProblem, ACPPowerModel, c_sys5_re;)
    mock_construct_device!(model, device_model)
    moi_tests(model, 144, 0, 72, 0, 72, false)
    psi_checkobjfun_test(model, GAEVF)
    model = DecisionModel(MockOperationProblem, ACPPowerModel, c_sys5_re;)
    mock_construct_device!(model, device_model; add_event_model = true)
    moi_tests(model, 144, 0, 96, 0, 72, false, 24)
end

@testset "Renewable DCPLossless FixedOutput" begin
    device_model = DeviceModel(RenewableDispatch, FixedOutput)
    c_sys5_re = PSB.build_system(PSITestSystems, "c_sys5_re")
    model = DecisionModel(MockOperationProblem, DCPPowerModel, c_sys5_re;)
    mock_construct_device!(model, device_model)
    moi_tests(model, 0, 0, 0, 0, 0, false)
    psi_checkobjfun_test(model, GAEVF)
    model = DecisionModel(MockOperationProblem, DCPPowerModel, c_sys5_re;)
    mock_construct_device!(model, device_model; add_event_model = true)
    moi_tests(model, 0, 0, 0, 0, 0, false)
end

@testset "Renewable ACPPowerModel FixedOutput" begin
    device_model = DeviceModel(RenewableDispatch, FixedOutput)
    c_sys5_re = PSB.build_system(PSITestSystems, "c_sys5_re")
    model = DecisionModel(MockOperationProblem, ACPPowerModel, c_sys5_re;)
    mock_construct_device!(model, device_model)
    moi_tests(model, 0, 0, 0, 0, 0, false)
    psi_checkobjfun_test(model, GAEVF)
    model = DecisionModel(MockOperationProblem, ACPPowerModel, c_sys5_re;)
    mock_construct_device!(model, device_model; add_event_model = true)
    moi_tests(model, 0, 0, 0, 0, 0, false)
end

@testset "Test Renewable CurtailmentCostExpression nonnegativity" begin
    c_sys5_re = PSB.build_system(PSITestSystems, "c_sys5_re")

    template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, ThermalStandard, ThermalStandardDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)

    model = DecisionModel(
        template,
        c_sys5_re;
        name = "RE_curtailment_cost",
        optimizer = HiGHS_optimizer,
        optimizer_solve_log_print = true,
    )

    @test build!(model; output_dir = test_path) == PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results = OptimizationProblemResults(model)

    expr_curt = read_expression(
        results,
        "CurtailmentCostExpression__RenewableDispatch";
        table_format = TableFormat.WIDE,
    )

    tol = 1e-8
    for unit in names(expr_curt)[2:end]
        @test all(expr_curt[!, unit] .>= -tol)
    end
end

@testset "Test Renewable curtailment_cost incentive affects dispatch" begin
    # Use a renewable variable cost high enough that the optimizer prefers thermal
    # in the baseline (forcing curtailment). The modified run sets curtailment_cost
    # > variable so the net objective coefficient on dispatch is negative and the
    # optimizer dispatches up to the time-series limit.
    variable_cost = 100.0
    curtailment_cost_modified = 1000.0

    function _build_and_solve_renewable_cost(curtailment_cost_value)
        sys = PSB.build_system(PSITestSystems, "c_sys5_re")
        for r in PSY.get_components(PSY.RenewableDispatch, sys)
            PSY.set_operation_cost!(
                r,
                PSY.RenewableGenerationCost(;
                    variable = CostCurve(LinearCurve(variable_cost)),
                    curtailment_cost = CostCurve(LinearCurve(curtailment_cost_value)),
                ),
            )
        end

        template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
        set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
        set_device_model!(template, ThermalStandard, ThermalStandardDispatch)
        set_device_model!(template, PowerLoad, StaticPowerLoad)

        model = DecisionModel(
            template,
            sys;
            name = "RE_curtailment_cost_$(curtailment_cost_value)",
            optimizer = HiGHS_optimizer,
        )

        @test build!(model; output_dir = test_path) == PSI.ModelBuildStatus.BUILT
        @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
        return model
    end

    baseline_model = _build_and_solve_renewable_cost(0.0)
    modified_model = _build_and_solve_renewable_cost(curtailment_cost_modified)

    baseline_dispatch = read_variable(
        OptimizationProblemResults(baseline_model),
        "ActivePowerVariable__RenewableDispatch";
        table_format = TableFormat.WIDE,
    )
    modified_dispatch = read_variable(
        OptimizationProblemResults(modified_model),
        "ActivePowerVariable__RenewableDispatch";
        table_format = TableFormat.WIDE,
    )

    baseline_total =
        sum(sum(baseline_dispatch[!, u]) for u in names(baseline_dispatch)[2:end])
    modified_total =
        sum(sum(modified_dispatch[!, u]) for u in names(modified_dispatch)[2:end])
    @test modified_total > baseline_total + 1e-6

    # Direct check on the JuMP objective: the coefficient on each renewable's
    # ActivePowerVariable should be negative when curtailment_cost > variable.
    container = PSI.get_optimization_container(modified_model)
    obj = JuMP.objective_function(PSI.get_jump_model(modified_model))
    re_vars = PSI.get_variable(container, PSI.ActivePowerVariable(), PSY.RenewableDispatch)
    @test all(JuMP.coefficient(obj, v) < 0 for v in re_vars)
end
