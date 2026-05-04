test_path = mktempdir()

@testset "StaticPowerLoad" begin
    models = [StaticPowerLoad, PowerLoadDispatch, PowerLoadInterruption]
    c_sys5_il = PSB.build_system(PSITestSystems, "c_sys5_il")
    networks = [DCPPowerModel, ACPPowerModel]
    for m in models, n in networks
        device_model = DeviceModel(PowerLoad, m)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model)
        moi_tests(model, 0, 0, 0, 0, 0, false)
        psi_checkobjfun_test(model, GAEVF)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model; add_event_model = true)
        moi_tests(model, 0, 0, 0, 0, 0, false)
    end
end

@testset "PowerLoadDispatch DC- PF" begin
    models = [PowerLoadDispatch]
    c_sys5_il = PSB.build_system(PSITestSystems, "c_sys5_il")
    networks = [DCPPowerModel]
    for m in models, n in networks
        device_model = DeviceModel(InterruptiblePowerLoad, m)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model)
        moi_tests(model, 24, 0, 24, 0, 0, false)
        psi_checkobjfun_test(model, GAEVF)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model; add_event_model = true)
        moi_tests(model, 24, 0, 48, 0, 0, false)
    end
end

@testset "PowerLoadDispatch AC- PF" begin
    models = [PowerLoadDispatch]
    c_sys5_il = PSB.build_system(PSITestSystems, "c_sys5_il")
    networks = [ACPPowerModel]
    for m in models, n in networks
        device_model = DeviceModel(InterruptiblePowerLoad, m)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model)
        moi_tests(model, 48, 0, 24, 0, 24, false)
        psi_checkobjfun_test(model, GAEVF)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model; add_event_model = true)
        moi_tests(model, 48, 0, 48, 0, 24, false, 24)
    end
end

@testset "PowerLoadDispatch AC- PF with MarketBidCost Invalid" begin
    models = [PowerLoadDispatch]
    c_sys5_il = PSB.build_system(PSITestSystems, "c_sys5_il")
    iloadbus4 = get_component(InterruptiblePowerLoad, c_sys5_il, "IloadBus4")
    set_operation_cost!(
        iloadbus4,
        MarketBidCost(;
            no_load_cost = 0.0,
            start_up = (hot = 0.0, warm = 0.0, cold = 0.0),
            shut_down = 0.0,
            incremental_offer_curves = make_market_bid_curve(
                [0.0, 100.0, 200.0, 300.0, 400.0, 500.0, 600.0],
                [25.0, 25.5, 26.0, 27.0, 28.0, 30.0],
                0.0,
            ),
        ),
    )
    networks = [ACPPowerModel]
    for m in models, n in networks
        device_model = DeviceModel(InterruptiblePowerLoad, m)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        @test_throws ArgumentError mock_construct_device!(model, device_model)
    end
end

@testset "PowerLoadDispatch AC- PF with MarketBidCost" begin
    c_sys5_il = PSB.build_system(PSITestSystems, "c_sys5_il")
    iloadbus4 = get_component(InterruptiblePowerLoad, c_sys5_il, "IloadBus4")
    set_operation_cost!(
        iloadbus4,
        MarketBidCost(;
            no_load_cost = 0.0,
            start_up = (hot = 0.0, warm = 0.0, cold = 0.0),
            shut_down = 0.0,
            decremental_offer_curves = make_market_bid_curve(
                [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0],
                [90.0, 85.0, 75.0, 70.0, 60.0, 50.0, 45.0, 40.0, 30.0, 25.0],
                0.0,
            ),
        ),
    )
    template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, InterruptiblePowerLoad, PowerLoadDispatch)
    model = DecisionModel(template,
        c_sys5_il;
        name = "UC_fixed_market_bid_cost",
        optimizer = HiGHS_optimizer,
        optimizer_solve_log_print = true)
    @test build!(model; output_dir = test_path) == PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    results = OptimizationProblemResults(model)
    expr = read_expression(
        results,
        "ProductionCostExpression__InterruptiblePowerLoad";
        table_format = TableFormat.WIDE,
    )
    p_l = read_variable(
        results,
        "ActivePowerVariable__InterruptiblePowerLoad";
        table_format = TableFormat.WIDE,
    )
    index = findfirst(row -> isapprox(100, row; atol = 1e-6), p_l.IloadBus4)
    calculated_cost = expr[index, "IloadBus4"][1]
    @test isapprox(-5700, calculated_cost; atol = 1)
end

@testset "PowerLoadInterruption DC- PF" begin
    models = [PowerLoadInterruption]
    c_sys5_il = PSB.build_system(PSITestSystems, "c_sys5_il")
    networks = [DCPPowerModel]
    for m in models, n in networks
        device_model = DeviceModel(InterruptiblePowerLoad, m)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model)
        moi_tests(model, 48, 0, 48, 0, 0, true)
        psi_checkobjfun_test(model, GAEVF)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model; add_event_model = true)
        moi_tests(model, 48, 0, 72, 0, 0, true)
    end
end

@testset "PowerLoadInterruption AC- PF" begin
    models = [PowerLoadInterruption]
    c_sys5_il = PSB.build_system(PSITestSystems, "c_sys5_il")
    networks = [ACPPowerModel]
    for m in models, n in networks
        device_model = DeviceModel(InterruptiblePowerLoad, m)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model)
        moi_tests(model, 72, 0, 48, 0, 24, true)
        psi_checkobjfun_test(model, GAEVF)
        model = DecisionModel(MockOperationProblem, n, c_sys5_il)
        mock_construct_device!(model, device_model; add_event_model = true)
        moi_tests(model, 72, 0, 72, 0, 24, true), 24
    end
end

@testset "Loads without TimeSeries" begin
    sys = build_system(PSITestSystems, "c_sys5_uc"; force_build = true)
    load = get_component(PowerLoad, sys, "Bus2")
    remove_time_series!(sys, Deterministic, load, "max_active_power")

    networks = [CopperPlatePowerModel, PTDFPowerModel, DCPPowerModel, ACPPowerModel]
    solvers = [HiGHS_optimizer, HiGHS_optimizer, HiGHS_optimizer, ipopt_optimizer]
    for (ix, net) in enumerate(networks)
        template = ProblemTemplate(
            NetworkModel(
                net;
            ),
        )
        set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
        set_device_model!(template, PowerLoad, StaticPowerLoad)
        set_device_model!(template, Line, StaticBranch)

        model = DecisionModel(
            template,
            sys;
            name = "UC",
            store_variable_names = true,
            optimizer = solvers[ix],
        )

        @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    end
end

@testset "Loads with MotorLoad" begin
    sys = build_system(PSITestSystems, "c_sys5_uc"; force_build = true)
    load = get_component(PowerLoad, sys, "Bus2")

    mload = MotorLoad(;
        name = "MotorLoadBus2",
        available = true,
        bus = load.bus,
        active_power = load.active_power / 10.0,
        reactive_power = load.reactive_power / 10.0,
        base_power = load.base_power,
        rating = load.max_active_power / 10.0,
        max_active_power = load.max_active_power / 10.0,
        reactive_power_limits = nothing,
    )
    add_component!(sys, mload)

    networks = [CopperPlatePowerModel, PTDFPowerModel, DCPPowerModel, ACPPowerModel]
    solvers = [HiGHS_optimizer, HiGHS_optimizer, HiGHS_optimizer, ipopt_optimizer]
    for (ix, net) in enumerate(networks)
        template = ProblemTemplate(
            NetworkModel(
                net;
            ),
        )
        set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
        set_device_model!(template, PowerLoad, StaticPowerLoad)
        set_device_model!(template, MotorLoad, StaticPowerLoad)
        set_device_model!(template, Line, StaticBranch)

        model = DecisionModel(
            template,
            sys;
            name = "UC",
            store_variable_names = true,
            optimizer = solvers[ix],
        )

        @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    end
end

@testset "PowerLoadShift with NonAnticipativityConstraint" begin
    c_sys5_il =
        PSB.build_system(PSITestSystems, "c_sys5_il"; add_single_time_series = true)
    il_load = first(PSY.get_components(InterruptiblePowerLoad, c_sys5_il))

    shiftable_load = ShiftablePowerLoad(;
        name = "shiftable_load",
        available = true,
        bus = PSY.get_bus(il_load),
        active_power = PSY.get_active_power(il_load),
        active_power_limits = (min = 0.0, max = PSY.get_active_power(il_load)),
        reactive_power = PSY.get_reactive_power(il_load),
        max_active_power = PSY.get_max_active_power(il_load),
        max_reactive_power = PSY.get_max_reactive_power(il_load),
        base_power = PSY.get_base_power(il_load),
        load_balance_time_horizon = 1,
        operation_cost = LoadCost(;
            variable = CostCurve(
                LinearCurve(0.0),
                UnitSystem.NATURAL_UNITS,
                LinearCurve(1.0),
            ),
            fixed = 0.0,
        ),
    )
    PSY.add_component!(c_sys5_il, shiftable_load)
    PSY.set_available!(il_load, false)
    PSY.copy_time_series!(shiftable_load, il_load)

    tstamps = TimeSeries.timestamp(
        PSY.get_time_series_array(SingleTimeSeries, shiftable_load, "max_active_power"),
    )
    n = length(tstamps)
    up_vals = ones(n)
    down_vals = ones(n)

    PSY.add_time_series!(
        c_sys5_il,
        shiftable_load,
        SingleTimeSeries(
            "shift_up_max_active_power",
            TimeArray(tstamps, up_vals);
            scaling_factor_multiplier = PSY.get_max_active_power,
        ),
    )
    PSY.add_time_series!(
        c_sys5_il,
        shiftable_load,
        SingleTimeSeries(
            "shift_down_max_active_power",
            TimeArray(tstamps, down_vals);
            scaling_factor_multiplier = PSY.get_max_active_power,
        ),
    )

    PSY.transform_single_time_series!(c_sys5_il, Hour(24), Hour(24))

    template = ProblemTemplate(
        NetworkModel(
            CopperPlatePowerModel;
            duals = [CopperPlateBalanceConstraint],
        ),
    )
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(
        template,
        DeviceModel(
            ShiftablePowerLoad,
            PowerLoadShift;
            attributes = Dict{String, Any}("additional_balance_interval" => Hour(12)),
        ),
    )

    model = DecisionModel(
        template,
        c_sys5_il;
        name = "UC_shiftable",
        store_variable_names = true,
        optimizer = HiGHS_optimizer,
    )

    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results = OptimizationProblemResults(model)
    up = read_variable(
        results,
        "ShiftUpActivePowerVariable__ShiftablePowerLoad";
        table_format = TableFormat.WIDE,
    )
    dn = read_variable(
        results,
        "ShiftDownActivePowerVariable__ShiftablePowerLoad";
        table_format = TableFormat.WIDE,
    )

    # Verify the non-anticipativity constraint holds in the solution:
    # the running sum of (shift_down - shift_up) must be >= 0 at every time step.
    @test all(
        cumsum(dn[!, "shiftable_load"] .- up[!, "shiftable_load"]) .>= -1e-6,
    )
end
