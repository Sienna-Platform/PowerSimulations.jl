# Note to devs. Use HiGHS for models with linear constraints and linear cost functions
# Use OSQP for models with quadratic cost function and linear constraints and ipopt otherwise

@testset "All PowerModels models construction" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    for (network, solver) in NETWORKS_FOR_TESTING
        template = get_thermal_dispatch_template_network(
            NetworkModel(network; PTDF_matrix = PTDF(c_sys5)),
        )
        ps_model = DecisionModel(template, c_sys5; optimizer = solver)
        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        opt_container = PSI.get_optimization_container(ps_model)
        @test opt_container.pm !== nothing
        @test PSI.has_container_key(opt_container, ActivePowerBalance, ACBus)
    end
end

@testset "Network Copper Plate" begin
    template = get_thermal_dispatch_template_network(CopperPlatePowerModel)
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 120, 120, 24],
        c_sys14 => [120, 0, 120, 120, 24],
        c_sys14_dc => [120, 0, 120, 120, 24],
    )
    constraint_keys = [PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System)]
    objfuncs = [GAEVF, GQEVF, GQEVF]
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 240000.0,
        c_sys14 => 142000.0,
        c_sys14_dc => 142000.0,
    )

    for (ix, sys) in enumerate(systems)
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
        psi_checksolve_test(ps_model, [MOI.OPTIMAL], test_obj_values[sys], 10000)
    end
    template = get_thermal_dispatch_template_network(
        NetworkModel(CopperPlatePowerModel; use_slacks = true),
    )
    ps_model_re = DecisionModel(
        template,
        PSB.build_system(PSITestSystems, "c_sys5_re");
        optimizer = HiGHS_optimizer,
    )
    @test build!(ps_model_re; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    psi_checksolve_test(ps_model_re, [MOI.OPTIMAL], 240000.0, 10000)
end

@testset "Network DC-PF with PTDF Model" begin
    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
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
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 264, 264, 24],
        c_sys14 => [120, 0, 600, 600, 24],
        c_sys14_dc => [168, 0, 648, 552, 24],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 240000.0,
        c_sys14 => 142000.0,
        c_sys14_dc => 142000.0,
    )
    test_objfuncs_types = IdDict{System, Type}(
        c_sys5 => GAEVF,
        c_sys14 => GQEVF,
        c_sys14_dc => GQEVF,
    )
    for sys in systems
        template = get_thermal_dispatch_template_network(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]),
        )
        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
        psi_checkobjfun_test(ps_model, test_objfuncs_types[sys])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "Network DC-PF with VirtualPTDF Model" begin
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
    PTDF_ref = IdDict{System, VirtualPTDF}(
        c_sys5 => VirtualPTDF(c_sys5),
        c_sys14 => VirtualPTDF(c_sys14),
        c_sys14_dc => VirtualPTDF(c_sys14_dc),
    )
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [120, 0, 264, 264, 24],
        c_sys14 => [120, 0, 600, 600, 24],
        c_sys14_dc => [168, 0, 648, 552, 24],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 240000.0,
        c_sys14 => 142000.0,
        c_sys14_dc => 142000.0,
    )
    test_objfuncs_types = IdDict{System, Type}(
        c_sys5 => GAEVF,
        c_sys14 => GQEVF,
        c_sys14_dc => GQEVF,
    )
    for (ix, sys) in enumerate(systems)
        template = get_thermal_dispatch_template_network(PTDFPowerModel)
        template = get_thermal_dispatch_template_network(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]),
        )
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
    end
end

@testset "Network DC lossless -PF network with PowerModels DCPlosslessForm" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(PSI.FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(PSI.FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(PSI.NodalBalanceActiveConstraint, PSY.ACBus),
    ]
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [384, 144, 264, 264, 288],
        c_sys14 => [936, 480, 600, 600, 840],
        c_sys14_dc => [984, 432, 648, 552, 840],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 242000.0,
        c_sys14 => 143000.0,
        c_sys14_dc => 143000.0,
    )
    for (ix, sys) in enumerate(systems)
        template = get_thermal_dispatch_template_network(DCPPowerModel)
        ps_model = DecisionModel(template, sys; optimizer = ipopt_optimizer)
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
            [MOI.OPTIMAL, MOI.LOCALLY_SOLVED],
            test_obj_values[sys],
            1000,
        )
    end
end

@testset "Network Solve AC-PF PowerModels StandardACPModel" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
    objfuncs = [GAEVF, GQEVF, GQEVF]
    # Check for voltage and angle constraints
    constraint_keys = [
        PSI.ConstraintKey(FlowRateConstraintFromTo, PSY.Line),
        PSI.ConstraintKey(FlowRateConstraintToFrom, PSY.Line),
        PSI.ConstraintKey(PSI.NodalBalanceActiveConstraint, PSY.ACBus),
        PSI.ConstraintKey(PSI.NodalBalanceReactiveConstraint, PSY.ACBus),
    ]
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [1056, 144, 240, 240, 264],
        c_sys14 => [2832, 480, 240, 240, 696],
        c_sys14_dc => [2832, 432, 336, 240, 744],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 240000.0,
        c_sys14 => 142000.0,
        c_sys14_dc => 142000.0,
    )
    for (ix, sys) in enumerate(systems)
        template = get_thermal_dispatch_template_network(ACPPowerModel)
        ps_model = DecisionModel(template, sys; optimizer = ipopt_optimizer)
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
            [MOI.TIME_LIMIT, MOI.OPTIMAL, MOI.LOCALLY_SOLVED],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "Network Solve AC-PF PowerModels NFAPowerModel" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [PSI.ConstraintKey(PSI.NodalBalanceActiveConstraint, PSY.ACBus)]
    test_results = Dict{System, Vector{Int}}(
        c_sys5 => [264, 0, 264, 264, 120],
        c_sys14 => [600, 0, 600, 600, 336],
        c_sys14_dc => [648, 0, 648, 552, 384],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 240000.0,
        c_sys14 => 142000.0,
        c_sys14_dc => 142000.0,
    )
    for (ix, sys) in enumerate(systems)
        template = get_thermal_dispatch_template_network(NFAPowerModel)
        ps_model = DecisionModel(template, sys; optimizer = ipopt_optimizer)
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
            [MOI.OPTIMAL, MOI.LOCALLY_SOLVED],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "Other Network AC PowerModels models" begin
    networks = [#ACPPowerModel, Already tested
        ACRPowerModel,
        ACTPowerModel,
    ]
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
    # TODO: add model specific constraints to this list. Voltages, etc.
    constraint_keys = [
        PSI.ConstraintKey(PSI.NodalBalanceActiveConstraint, PSY.ACBus),
        PSI.ConstraintKey(PSI.NodalBalanceReactiveConstraint, PSY.ACBus),
    ]
    ACR_test_results = Dict{System, Vector{Int}}(
        c_sys5 => [1056, 0, 240, 240, 264],
        c_sys14 => [2832, 0, 240, 240, 696],
        c_sys14_dc => [2832, 0, 336, 240, 744],
    )
    ACT_test_results = Dict{System, Vector{Int}}(
        c_sys5 => [1344, 144, 240, 240, 840],
        c_sys14 => [3792, 480, 240, 240, 2616],
        c_sys14_dc => [3696, 432, 336, 240, 2472],
    )
    test_results = Dict(zip(networks, [ACR_test_results, ACT_test_results]))
    for network in networks, sys in systems
        template = get_thermal_dispatch_template_network(network)
        ps_model = DecisionModel(template, sys; optimizer = fast_ipopt_optimizer)
        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[network][sys][1],
            test_results[network][sys][2],
            test_results[network][sys][3],
            test_results[network][sys][4],
            test_results[network][sys][5],
            false,
        )
        @test PSI.get_optimization_container(ps_model).pm !== nothing
    end
end

# TODO: Add constraint tests for these models, other is redundant with first test
@testset "Network DC-PF PowerModels quadratic loss approximations models" begin
    networks = [DCPLLPowerModel, LPACCPowerModel]
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")
    c_sys14 = PSB.build_system(PSITestSystems, "c_sys14")
    c_sys14_dc = PSB.build_system(PSITestSystems, "c_sys14_dc")
    systems = [c_sys5, c_sys14, c_sys14_dc]
    # TODO: add model specific constraints to this list. Bi-directional flows etc
    constraint_keys = [PSI.ConstraintKey(PSI.NodalBalanceActiveConstraint, PSY.ACBus)]
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 240000.0,
        c_sys14 => 142000.0,
        c_sys14_dc => 142000.0,
    )
    DCPLL_test_results = Dict{System, Vector{Int}}(
        c_sys5 => [528, 144, 264, 264, 288],
        c_sys14 => [1416, 480, 600, 600, 840],
        c_sys14_dc => [1416, 432, 648, 552, 840],
    )
    LPACC_test_results = Dict{System, Vector{Int}}(
        c_sys5 => [1200, 144, 240, 240, 840],
        c_sys14 => [3312, 480, 240, 240, 2616],
        c_sys14_dc => [3264, 432, 336, 240, 2472],
    )
    test_results = Dict(zip(networks, [DCPLL_test_results, LPACC_test_results]))
    for network in networks, (ix, sys) in enumerate(systems)
        template = get_thermal_dispatch_template_network(network)
        ps_model = DecisionModel(template, sys; optimizer = ipopt_optimizer)
        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[network][sys][1],
            test_results[network][sys][2],
            test_results[network][sys][3],
            test_results[network][sys][4],
            test_results[network][sys][5],
            false,
        )
        @test PSI.get_optimization_container(ps_model).pm !== nothing
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.LOCALLY_SOLVED],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "Network Unsupported Power Model Formulations" begin
    for network in PSI.UNSUPPORTED_POWERMODELS
        template = get_thermal_dispatch_template_network(network)
        ps_model = DecisionModel(
            template,
            PSB.build_system(PSITestSystems, "c_sys5");
            optimizer = ipopt_optimizer,
        )
        @test build!(
            ps_model;
            console_level = Logging.AboveMaxLevel,  # Ignore expected errors.
            output_dir = mktempdir(; cleanup = true),
        ) == PSI.ModelBuildStatus.FAILED
    end
end

@testset "2 Subnetworks HVDC DC-PF with CopperPlatePowerModel" begin
    c_sys5 = PSB.build_system(PSISystems, "2Area 5 Bus System")
    template = get_thermal_dispatch_template_network(NetworkModel(CopperPlatePowerModel))
    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    solve!(ps_model)

    moi_tests(ps_model, 264, 0, 288, 240, 48, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.System)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 480288, 100)

    results = OptimizationProblemResults(ps_model)
    hvdc_flow =
        read_variable(
            results,
            "FlowActivePowerVariable__TwoTerminalGenericHVDCLine";
            table_format = TableFormat.WIDE,
        )
    @test all(hvdc_flow[!, "nodeC-nodeC2"] .<= 200)
    @test all(hvdc_flow[!, "nodeC-nodeC2"] .>= -200)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Load-nodeC", "Load-nodeD", "Load-nodeB"]]))
    zone_1_gen = sum(
        eachcol(thermal_gen[!, ["Solitude", "Park City", "Sundance", "Brighton", "Alta"]]),
    )
    @test all(
        isapprox.(
            sum(zone_1_gen .+ zone_1_load .- hvdc_flow[!, "nodeC-nodeC2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    zone_2_load = sum(eachcol(load[!, ["Load-nodeC2", "Load-nodeD2", "Load-nodeB2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude-2", "Park City-2", "Sundance-2", "Brighton-2", "Alta-2"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_2_gen .+ zone_2_load .+ hvdc_flow[!, "nodeC-nodeC2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    # Test forcing flows to 0.0
    hvdc_link = get_component(TwoTerminalGenericHVDCLine, c_sys5, "nodeC-nodeC2")
    set_active_power_limits_from!(hvdc_link, (min = 0.0, max = 0.0))
    set_active_power_limits_to!(hvdc_link, (min = 0.0, max = 0.0))

    # Test not passing the PTDF to the Template
    template = get_thermal_dispatch_template_network(NetworkModel(PTDFPowerModel))
    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    solve!(ps_model)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.System)

    results = OptimizationProblemResults(ps_model)
    hvdc_flow =
        read_variable(
            results,
            "FlowActivePowerVariable__TwoTerminalGenericHVDCLine";
            table_format = TableFormat.WIDE,
        )
    @test all(hvdc_flow[!, "nodeC-nodeC2"] .== 0.0)
    @test all(hvdc_flow[!, "nodeC-nodeC2"] .== 0.0)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Load-nodeC", "Load-nodeD", "Load-nodeB"]]))
    zone_1_gen = sum(
        eachcol(thermal_gen[!, ["Solitude", "Park City", "Sundance", "Brighton", "Alta"]]),
    )
    @test all(isapprox.(sum(zone_1_gen .+ zone_1_load; dims = 2), 0.0; atol = 1e-3))

    zone_2_load = sum(eachcol(load[!, ["Load-nodeC2", "Load-nodeD2", "Load-nodeB2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude-2", "Park City-2", "Sundance-2", "Brighton-2", "Alta-2"],
            ],
        ),
    )
    @test all(isapprox.(sum(zone_2_gen .+ zone_2_load; dims = 2), 0.0; atol = 1e-3))
end

@testset "2 Subnetworks DC-PF with PTDF Model" begin
    c_sys5 = PSB.build_system(PSISystems, "2Area 5 Bus System")
    # Test passing a VirtualPTDF Model
    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = VirtualPTDF(c_sys5)),
    )
    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    solve!(ps_model)

    moi_tests(ps_model, 264, 0, 576, 528, 48, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.System)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 482587, 100)

    results = OptimizationProblemResults(ps_model)
    hvdc_flow =
        read_variable(
            results,
            "FlowActivePowerVariable__TwoTerminalGenericHVDCLine";
            table_format = TableFormat.WIDE,
        )
    @test all(hvdc_flow[!, "nodeC-nodeC2"] .<= 200 + PSI.ABSOLUTE_TOLERANCE)
    @test all(hvdc_flow[!, "nodeC-nodeC2"] .>= -200 - PSI.ABSOLUTE_TOLERANCE)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Load-nodeC", "Load-nodeD", "Load-nodeB"]]))
    zone_1_gen = sum(
        eachcol(thermal_gen[!, ["Solitude", "Park City", "Sundance", "Brighton", "Alta"]]),
    )
    @test all(
        isapprox.(
            sum(zone_1_gen .+ zone_1_load .- hvdc_flow[!, "nodeC-nodeC2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    zone_2_load = sum(eachcol(load[!, ["Load-nodeC2", "Load-nodeD2", "Load-nodeB2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude-2", "Park City-2", "Sundance-2", "Brighton-2", "Alta-2"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_2_gen .+ zone_2_load .+ hvdc_flow[!, "nodeC-nodeC2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    # Test forcing flows to 0.0
    hvdc_link = get_component(PSY.TwoTerminalGenericHVDCLine, c_sys5, "nodeC-nodeC2")
    set_active_power_limits_from!(hvdc_link, (min = 0.0, max = 0.0))
    set_active_power_limits_to!(hvdc_link, (min = 0.0, max = 0.0))

    # Test not passing the PTDF to the Template
    template = get_thermal_dispatch_template_network(NetworkModel(PTDFPowerModel))
    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    solve!(ps_model)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.System)

    results = OptimizationProblemResults(ps_model)
    hvdc_flow =
        read_variable(
            results,
            "FlowActivePowerVariable__TwoTerminalGenericHVDCLine";
            table_format = TableFormat.WIDE,
        )
    @test all(hvdc_flow[!, "nodeC-nodeC2"] .== 0.0)
    @test all(hvdc_flow[!, "nodeC-nodeC2"] .== 0.0)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Load-nodeC", "Load-nodeD", "Load-nodeB"]]))
    zone_1_gen = sum(
        eachcol(thermal_gen[!, ["Solitude", "Park City", "Sundance", "Brighton", "Alta"]]),
    )
    @test all(isapprox.(sum(zone_1_gen .+ zone_1_load; dims = 2), 0.0; atol = 1e-3))

    zone_2_load = sum(eachcol(load[!, ["Load-nodeC2", "Load-nodeD2", "Load-nodeB2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude-2", "Park City-2", "Sundance-2", "Brighton-2", "Alta-2"],
            ],
        ),
    )
    @test all(isapprox.(sum(zone_2_gen .+ zone_2_load; dims = 2), 0.0; atol = 1e-3))

    # Test passing a Virtual PTDF Model with higher tolerance
    c_sys5 = PSB.build_system(PSISystems, "2Area 5 Bus System")
    # Test passing a VirtualPTDF Model
    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = VirtualPTDF(c_sys5; tol = 1e-2)),
    )
    ps_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

# These models are easier to test due to their lossless nature
@testset "StandardPTDF/DCPPowerModel Radial Branches Test" begin
    new_sys = PSB.build_system(PSITestSystems, "c_sys5_radial")
    for net_model in [DCPPowerModel, PTDFPowerModel]
        template_uc = template_unit_commitment(;
            network = NetworkModel(net_model;
                reduce_radial_branches = true,
                use_slacks = false,
            ),
        )
        thermal_model = ThermalStandardUnitCommitment
        set_device_model!(template_uc, ThermalStandard, thermal_model)

        ##### Solve Reduced Model ####
        uc_model_red = DecisionModel(
            template_uc,
            new_sys;
            optimizer = HiGHS_optimizer,
            name = "UC_RED",
            store_variable_names = true,
        )

        @test build!(uc_model_red; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        solve!(uc_model_red)

        res_red = OptimizationProblemResults(uc_model_red)

        if net_model == DCPPowerModel
            flow_lines = read_variable(
                res_red,
                "FlowActivePowerVariable__Line";
                table_format = TableFormat.WIDE,
            )
        else
            flow_lines = read_expression(
                res_red,
                "PTDFBranchFlow__Line";
                table_format = TableFormat.WIDE,
            )
        end
        line_names = DataFrames.names(flow_lines)[2:end]

        ##### Solve Original Model ####
        template_uc_orig = template_unit_commitment(;
            network = NetworkModel(net_model;
                reduce_radial_branches = false,
                use_slacks = false,
            ),
        )
        set_device_model!(template_uc_orig, ThermalStandard, thermal_model)

        uc_model_orig = DecisionModel(
            template_uc_orig,
            new_sys;
            optimizer = HiGHS_optimizer,
            name = "UC_ORIG",
            store_variable_names = true,
        )

        @test build!(uc_model_orig; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        solve!(uc_model_orig)

        res_orig = OptimizationProblemResults(uc_model_orig)

        if net_model == DCPPowerModel
            flow_lines_orig = read_variable(
                res_orig,
                "FlowActivePowerVariable__Line";
                table_format = TableFormat.WIDE,
            )
        else
            flow_lines_orig = read_expression(
                res_orig,
                "PTDFBranchFlow__Line";
                table_format = TableFormat.WIDE,
            )
        end

        for line in line_names
            @test isapprox(flow_lines[!, line], flow_lines_orig[!, line])
        end
    end
end

@testset "StandardPTDF with Ward reduction Test" begin
    new_sys = PSB.build_system(PSITestSystems, "c_sys5_radial")
    #This ward reduction is equivalent to the radial reduciton, therefore flows should be unchanged
    nr = NetworkReduction[WardReduction([1, 2, 3, 4, 5])]
    ptdf = PTDF(new_sys; network_reductions = nr)
    template_uc = template_unit_commitment(;
        network = NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            use_slacks = false,
        ),
    )
    thermal_model = ThermalStandardUnitCommitment
    set_device_model!(template_uc, ThermalStandard, thermal_model)

    ##### Solve Reduced Model ####
    solver = HiGHS_optimizer
    uc_model_red = DecisionModel(
        template_uc,
        new_sys;
        optimizer = solver,
        name = "UC_RED",
        store_variable_names = true,
    )

    @test build!(uc_model_red; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    solve!(uc_model_red)

    res_red = OptimizationProblemResults(uc_model_red)

    flow_lines = read_expression(
        res_red,
        "PTDFBranchFlow__Line";
        table_format = TableFormat.WIDE,
    )
    line_names = DataFrames.names(flow_lines)[2:end]

    ##### Solve Original Model ####
    template_uc_orig = template_unit_commitment(;
        network = NetworkModel(PTDFPowerModel;
            reduce_radial_branches = false,
            use_slacks = false,
        ),
    )
    set_device_model!(template_uc_orig, ThermalStandard, thermal_model)

    uc_model_orig = DecisionModel(
        template_uc_orig,
        new_sys;
        optimizer = solver,
        name = "UC_ORIG",
        store_variable_names = true,
    )

    @test build!(uc_model_orig; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    solve!(uc_model_orig)

    res_orig = OptimizationProblemResults(uc_model_orig)

    flow_lines_orig = read_expression(
        res_orig,
        "PTDFBranchFlow__Line";
        table_format = TableFormat.WIDE,
    )

    for line in line_names
        @test isapprox(flow_lines[!, line], flow_lines_orig[!, line])
    end
end

@testset "All PowerModels models construction with reduced radial branches" begin
    new_sys = PSB.build_system(PSITestSystems, "c_sys5_radial")
    for (network, solver) in NETWORKS_FOR_TESTING
        template = get_thermal_dispatch_template_network(
            NetworkModel(network;
                PTDF_matrix = PTDF(new_sys),
                reduce_radial_branches = true,
                use_slacks = true),
        )
        ps_model = DecisionModel(template, new_sys; optimizer = solver)
        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        @test PSI.get_optimization_container(ps_model).pm !== nothing
    end
end

@testset "2 Areas AreaBalance PowerModel - with slacks" begin
    c_sys = build_system(PSITestSystems, "c_sys5_uc")
    # Extend the system with two areas
    areas = [Area("Area_1", 0, 0, 0), Area("Area_2", 0, 0, 0)]
    add_components!(c_sys, areas)
    for (i, comp) in enumerate(get_components(ACBus, c_sys))
        (i < 3) ? set_area!(comp, areas[1]) : set_area!(comp, areas[2])
    end
    # Deactivate generators on Area 1: as there is no area interchange defined,
    # slacks will be required for feasibility
    for gen in get_components(x -> (get_area(get_bus(x)) == areas[1]), Generator, c_sys)
        set_available!(gen, false)
    end

    template = get_thermal_dispatch_template_network(
        NetworkModel(AreaBalancePowerModel; use_slacks = true),
    )
    ps_model = DecisionModel(template, c_sys; optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    results = OptimizationProblemResults(ps_model)
    slacks_up = read_variable(
        results,
        "SystemBalanceSlackUp__Area";
        table_format = TableFormat.WIDE,
    )
    @test all(slacks_up[!, "Area_1"] .> 0.0)
    @test all(slacks_up[!, "Area_2"] .≈ 0.0)
end

@testset "2 Areas AreaBalance PowerModel" begin
    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA")
    transform_single_time_series!(c_sys, Hour(24), Hour(1))
    template = get_thermal_dispatch_template_network(NetworkModel(AreaBalancePowerModel))
    set_device_model!(template, AreaInterchange, StaticBranch)
    ps_model =
        DecisionModel(template, c_sys; resolution = Hour(1), optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 264, 0, 264, 264, 48, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 482055, 1)

    results = OptimizationProblemResults(ps_model)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    # The values for these tests come from the data
    @test all(interarea_flow[!, "1_2"] .<= 150)
    @test all(interarea_flow[!, "1_2"] .>= -150)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Bus4_1", "Bus3_1", "Bus2_1"]]))
    zone_1_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_1", "Park City_1", "Sundance_1", "Brighton_1", "Alta_1"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_1_gen .+ zone_1_load .- interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    zone_2_load = sum(eachcol(load[!, ["Bus4_2", "Bus3_2", "Bus2_2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_2", "Park City_2", "Sundance_2", "Brighton_2", "Alta_2"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_2_gen .+ zone_2_load .+ interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )
end

@testset "2 Areas AreaBalance PowerModel with TimeSeries" begin
    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA")
    load = first(get_components(PowerLoad, c_sys))
    ts_array = get_time_series_array(SingleTimeSeries, load, "max_active_power")
    tstamp = timestamp(ts_array)
    area_int = first(get_components(AreaInterchange, c_sys))
    day_data = [
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
    ]
    weekly_data = repeat(day_data, 7)
    ts_from_to = SingleTimeSeries(
        "from_to_flow_limit",
        TimeArray(tstamp, weekly_data);
        scaling_factor_multiplier = get_from_to_flow_limit,
    )
    ts_to_from = SingleTimeSeries(
        "to_from_flow_limit",
        TimeArray(tstamp, weekly_data);
        scaling_factor_multiplier = get_from_to_flow_limit,
    )
    add_time_series!(c_sys, area_int, ts_from_to)
    add_time_series!(c_sys, area_int, ts_to_from)
    ## Transform Time Series ##
    transform_single_time_series!(c_sys, Hour(24), Hour(24))
    template = get_thermal_dispatch_template_network(NetworkModel(AreaBalancePowerModel))
    set_device_model!(template, AreaInterchange, StaticBranch)
    ps_model =
        DecisionModel(template, c_sys; resolution = Hour(1), optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 264, 0, 264, 264, 48, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 482055, 1)

    results = OptimizationProblemResults(ps_model)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    # The values for these tests come from the data
    @test interarea_flow[4, "1_2"] != 0.0
    @test interarea_flow[5, "1_2"] == 0.0
    @test interarea_flow[6, "1_2"] == 0.0
end

@testset "2 Areas AreaPTDFPowerModel" begin
    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA")
    transform_single_time_series!(c_sys, Hour(24), Hour(1))
    set_flow_limits!(
        get_component(AreaInterchange, c_sys, "1_2"),
        (from_to = 1.0, to_from = 1.0),
    )
    template = get_thermal_dispatch_template_network(NetworkModel(AreaPTDFPowerModel))
    set_device_model!(template, AreaInterchange, StaticBranch)
    set_device_model!(template, MonitoredLine, StaticBranchUnbounded)
    ps_model =
        DecisionModel(template, c_sys; resolution = Hour(1), optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 264, 0, 576, 576, 48, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 497551, 1)

    results = OptimizationProblemResults(ps_model)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    # The values for these tests come from the data
    @test all(interarea_flow[!, "1_2"] .<= 100.0 + PSI.ABSOLUTE_TOLERANCE)
    @test all(interarea_flow[!, "1_2"] .>= -100.0 - PSI.ABSOLUTE_TOLERANCE)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Bus4_1", "Bus3_1", "Bus2_1"]]))
    zone_1_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_1", "Park City_1", "Sundance_1", "Brighton_1", "Alta_1"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_1_gen .+ zone_1_load .- interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    zone_2_load = sum(eachcol(load[!, ["Bus4_2", "Bus3_2", "Bus2_2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_2", "Park City_2", "Sundance_2", "Brighton_2", "Alta_2"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_2_gen .+ zone_2_load .+ interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )
end

@testset "2 Areas AreaPTDFPowerModel with Double Circuit" begin
    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA")
    l = get_component(MonitoredLine, c_sys, "inter_area_line")
    l2 = MonitoredLine(;
        name = "inter_area_line_2",
        available = get_available(l),
        active_power_flow = get_active_power_flow(l),
        reactive_power_flow = get_reactive_power_flow(l),
        arc = get_arc(l),
        r = get_r(l),
        x = get_x(l),
        b = get_b(l),
        flow_limits = get_flow_limits(l),
        rating = get_rating(l),
        angle_limits = get_angle_limits(l),
        rating_b = get_rating_b(l),
        rating_c = get_rating_c(l),
        g = get_g(l),
        services = get_services(l),
    )
    add_component!(c_sys, l2)
    transform_single_time_series!(c_sys, Hour(24), Hour(1))
    set_flow_limits!(
        get_component(AreaInterchange, c_sys, "1_2"),
        (from_to = 1.0, to_from = 1.0),
    )
    template = get_thermal_dispatch_template_network(NetworkModel(AreaPTDFPowerModel))
    set_device_model!(template, AreaInterchange, StaticBranch)
    set_device_model!(template, MonitoredLine, StaticBranchUnbounded)
    ps_model =
        DecisionModel(template, c_sys; resolution = Hour(1), optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 264, 0, 576, 576, 48, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 497551, 1)

    results = OptimizationProblemResults(ps_model)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    # The values for these tests come from the data
    @test all(interarea_flow[!, "1_2"] .<= 100.0 + PSI.ABSOLUTE_TOLERANCE)
    @test all(interarea_flow[!, "1_2"] .>= -100.0 - PSI.ABSOLUTE_TOLERANCE)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Bus4_1", "Bus3_1", "Bus2_1"]]))
    zone_1_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_1", "Park City_1", "Sundance_1", "Brighton_1", "Alta_1"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_1_gen .+ zone_1_load .- interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    zone_2_load = sum(eachcol(load[!, ["Bus4_2", "Bus3_2", "Bus2_2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_2", "Park City_2", "Sundance_2", "Brighton_2", "Alta_2"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_2_gen .+ zone_2_load .+ interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )
end

@testset "2 Areas AreaPTDFPowerModel with Time Series" begin
    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA")
    load = first(get_components(PowerLoad, c_sys))
    new_line = Line(;
        name = "C2_D1",
        available = true,
        active_power_flow = 0.0,
        reactive_power_flow = 0.0,
        arc = Arc(;
            from = get_component(ACBus, c_sys, "Bus_nodeC_2"),
            to = get_component(ACBus, c_sys, "Bus_nodeD_1"),
        ),
        r = 0.00297,
        x = 0.0297,
        b = (from = 0.00337, to = 0.00337),
        rating = 40.53,
        angle_limits = (min = -0.7, max = 0.7),
    )
    add_component!(c_sys, new_line)
    ts_array = get_time_series_array(SingleTimeSeries, load, "max_active_power")
    tstamp = timestamp(ts_array)
    area_int = first(get_components(AreaInterchange, c_sys))
    day_data = [
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
    ]
    weekly_data = repeat(day_data, 7)
    ts_from_to = SingleTimeSeries(
        "from_to_flow_limit",
        TimeArray(tstamp, weekly_data);
        scaling_factor_multiplier = get_from_to_flow_limit,
    )
    ts_to_from = SingleTimeSeries(
        "to_from_flow_limit",
        TimeArray(tstamp, weekly_data);
        scaling_factor_multiplier = get_from_to_flow_limit,
    )
    add_time_series!(c_sys, area_int, ts_from_to)
    add_time_series!(c_sys, area_int, ts_to_from)
    ## Transform Time Series ##
    transform_single_time_series!(c_sys, Hour(24), Hour(24))
    set_flow_limits!(
        get_component(AreaInterchange, c_sys, "1_2"),
        (from_to = 1.0, to_from = 1.0),
    )
    template = get_thermal_dispatch_template_network(NetworkModel(AreaPTDFPowerModel))
    set_device_model!(template, AreaInterchange, StaticBranch)
    set_device_model!(template, MonitoredLine, StaticBranchUnbounded)
    ps_model =
        DecisionModel(template, c_sys; resolution = Hour(1), optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 264, 0, 600, 600, 48, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 489842, 1)

    results = OptimizationProblemResults(ps_model)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    # The values for these tests come from the data
    @test interarea_flow[1, "1_2"] != 0.0
    @test interarea_flow[5, "1_2"] == 0.0
    @test interarea_flow[6, "1_2"] == 0.0
end

function add_dummy_time_series_data!(sys)
    # Attach dummy data so the problem builds:
    dummy_data = Dict(
        DateTime("2020-01-01T08:00:00") => [5.0, 6, 7, 7, 7],
        DateTime("2020-01-01T08:30:00") => [9.0, 9, 9, 9, 8],
        DateTime("2020-01-01T09:00:00") => [6.0, 6, 5, 5, 4],
    )
    resolution = Dates.Minute(5)
    dummy_forecast = Deterministic("max_active_power", dummy_data, resolution)
    load = collect(get_components(StandardLoad, sys))[1]
    add_time_series!(sys, load, dummy_forecast)
    return sys
end

@testset "Network reductions - PTDF StaticBranch" begin
    # Base Case : Only reductions for double circuits:
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, Transformer2W, StaticBranch)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(ps_model, 0, 0, 60, 60, 5, false)
    container = PSI.get_optimization_container(ps_model)
    expression = PSI.get_expression(
        container,
        PTDFBranchFlow(),
        Line,
    )
    @test size(expression) == (10, 5)

    # Radial Reduction :
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[RadialReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, Transformer2W, StaticBranch)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(ps_model, 0, 0, 55, 55, 5, false)
    container = PSI.get_optimization_container(ps_model)
    expression = PSI.get_expression(
        container,
        PTDFBranchFlow(),
        Line,
    )
    @test size(expression) == (9, 5)

    # Degree Two Reduction :
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[DegreeTwoReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, Transformer2W, StaticBranch)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(ps_model, 0, 0, 35, 35, 5, false)
    expression = PSI.get_expression(
        container,
        PTDFBranchFlow(),
        Line,
    )
    @test size(expression) == (9, 5)

    container = PSI.get_optimization_container(ps_model)
    line_flow = PSI.get_expression(container, PTDFBranchFlow(), Line)
    tfw_flow = PSI.get_expression(container, PTDFBranchFlow(), Transformer2W)
    # Parallel branch within chain to d2:
    @test line_flow["2-10-i_double_circuit", :] == line_flow["10-3-i_1", :]
    # D2 chain with different component types:
    @test line_flow["1-9-i_1", :] == tfw_flow["9-5-i_1", :]
    # Parallel branch within chain to d2 with mixed types (parallel comes from tracker):
    @test line_flow["3-11-i_1", :] == tfw_flow["11-4-i_double_circuit", :]

    # Radial + Degree Two Reduction :
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[RadialReduction(), DegreeTwoReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, Transformer2W, StaticBranch)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(ps_model, 0, 0, 30, 30, 5, false)
    expression = PSI.get_expression(
        container,
        PTDFBranchFlow(),
        Line,
    )
    @test size(expression) == (10, 5)
end

@testset "Network reductions - PTDF StaticBranchBounds" begin
    # Base Case : Only reductions for double circuits:
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranchBounds)
    set_device_model!(template, Transformer2W, StaticBranchBounds)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(ps_model, 60, 0, 0, 0, 65, false)
    container = PSI.get_optimization_container(ps_model)
    expression = PSI.get_expression(
        container,
        PTDFBranchFlow(),
        Line,
    )
    @test size(expression) == (10, 5)
    flow_var = PSI.get_variable(
        container,
        FlowActivePowerVariable(),
        Line,
    )
    @test size(expression) == (10, 5)

    # Radial Reduction :
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[RadialReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranchBounds)
    set_device_model!(template, Transformer2W, StaticBranchBounds)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(ps_model, 55, 0, 0, 0, 60, false)
    container = PSI.get_optimization_container(ps_model)
    expression = PSI.get_expression(
        container,
        PTDFBranchFlow(),
        Line,
    )
    @test size(expression) == (9, 5)

    # Degree Two Reduction :
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[DegreeTwoReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranchBounds)
    set_device_model!(template, Transformer2W, StaticBranchBounds)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(ps_model, 35, 0, 0, 0, 40, false)
    expression = PSI.get_expression(
        container,
        PTDFBranchFlow(),
        Line,
    )
    @test size(expression) == (9, 5)

    flow_var = PSI.get_variable(
        container,
        FlowActivePowerVariable(),
        Line,
    )
    @test size(expression) == (9, 5)

    container = PSI.get_optimization_container(ps_model)
    line_flow_exp = PSI.get_expression(container, PTDFBranchFlow(), Line)
    tfw_flow_exp = PSI.get_expression(container, PTDFBranchFlow(), Transformer2W)
    # Parallel branch within chain to d2:
    @test line_flow_exp["2-10-i_double_circuit", :] == line_flow_exp["10-3-i_1", :]
    # D2 chain with different component types:
    @test line_flow_exp["1-9-i_1", :] == tfw_flow_exp["9-5-i_1", :]
    # Parallel branch within chain to d2 with mixed types (parallel comes from tracker):
    @test line_flow_exp["3-11-i_1", :] == tfw_flow_exp["11-4-i_double_circuit", :]

    container = PSI.get_optimization_container(ps_model)
    line_flow_var = PSI.get_variable(container, FlowActivePowerVariable(), Line)
    tfw_flow_var = PSI.get_variable(container, FlowActivePowerVariable(), Transformer2W)
    # Parallel branch within chain to d2:
    @test line_flow_var["2-10-i_double_circuit", :] == line_flow_var["10-3-i_1", :]
    # D2 chain with different component types:
    @test line_flow_var["1-9-i_1", :] == tfw_flow_var["9-5-i_1", :]
    # Parallel branch within chain to d2 with mixed types (parallel comes from tracker):
    @test line_flow_var["3-11-i_1", :] == tfw_flow_var["11-4-i_double_circuit", :]

    # Radial + Degree Two Reduction :
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[RadialReduction(), DegreeTwoReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranchBounds)
    set_device_model!(template, Transformer2W, StaticBranchBounds)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(ps_model, 30, 0, 0, 0, 35, false)
    expression = PSI.get_expression(
        container,
        PTDFBranchFlow(),
        Line,
    )
    @test size(expression) == (10, 5)

    flow_var = PSI.get_variable(
        container,
        FlowActivePowerVariable(),
        Line,
    )
    @test size(expression) == (10, 5)
end

@testset "Network reductions for system with subnetworks" begin
    sys = build_system(PSISystems, "HVDC_TWO_RTO_RTS_1Hr_sys")
    nr = NetworkReduction[RadialReduction(), DegreeTwoReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, Transformer2W, StaticBranch)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
end

# Tests a case with series reductions containing different branch types, some with and some without filters applied
@testset "Network reductions + branch filter edge cases" begin
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[RadialReduction(), DegreeTwoReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, DeviceModel(Line, StaticBranch))
    modeled_transformer_names = ["9-5-i_1"]
    set_device_model!(
        template,
        DeviceModel(
            Transformer2W,
            StaticBranch;
            attributes = Dict(
                "filter_function" => x -> PSY.get_name(x) in modeled_transformer_names,
            ),
        ),
    )
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
end

@testset "Branch bounds of parallel and series reductions" begin
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    nr = NetworkReduction[DegreeTwoReduction()]
    ptdf = PTDF(sys; network_reductions = nr)
    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_radial_branches = PNM.has_radial_reduction(ptdf.network_reduction_data),
            reduce_degree_two_branches = PNM.has_degree_two_reduction(
                ptdf.network_reduction_data,
            ),
            use_slacks = false),
    )
    set_device_model!(template, Line, StaticBranchBounds)
    set_device_model!(template, Transformer2W, StaticBranchBounds)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    l1_parallel = PSY.get_rating(PSY.get_component(ACTransmission, sys, "1-4-i_1"))
    l2_parallel = PSY.get_rating(PSY.get_component(ACTransmission, sys, "1-4-i_2"))
    container = PSI.get_optimization_container(ps_model)
    line_flow_var = PSI.get_variable(
        container,
        FlowActivePowerVariable(),
        Line,
    )
    @test JuMP.upper_bound(line_flow_var["1-4-i_double_circuit", 1]) ==
          minimum([l1_parallel, l2_parallel])
    l1_series = PSY.get_rating(PSY.get_component(ACTransmission, sys, "9-5-i_1"))
    l2_series = PSY.get_rating(PSY.get_component(ACTransmission, sys, "1-9-i_1"))
    tx_flow_var = PSI.get_variable(
        container,
        FlowActivePowerVariable(),
        Transformer2W,
    )
    @test JuMP.upper_bound(tx_flow_var["9-5-i_1", 1]) == minimum([l1_series, l2_series])
    @test JuMP.upper_bound(line_flow_var["1-9-i_1", 1]) == minimum([l1_series, l2_series])
end

@testset "Network reductions - PowerModels" begin
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    for (network_model, optimizer) in NETWORKS_FOR_TESTING
        @testset "Network Model: $(network_model)" begin
            # Only default reductions:
            template = ProblemTemplate(
                NetworkModel(network_model;
                    reduce_radial_branches = false,
                    reduce_degree_two_branches = false,
                    use_slacks = false),
            )
            set_device_model!(template, Line, StaticBranch)
            set_device_model!(template, Transformer2W, StaticBranch)
            ps_model = DecisionModel(template, sys; optimizer = optimizer)
            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            JuMPmodel = PSI.get_jump_model(ps_model)
            n_vars = JuMP.num_variables(JuMPmodel)

            # Radial reductions:
            template = ProblemTemplate(
                NetworkModel(network_model;
                    reduce_radial_branches = true,
                    reduce_degree_two_branches = false,
                    use_slacks = false),
            )
            set_device_model!(template, Line, StaticBranch)
            set_device_model!(template, Transformer2W, StaticBranch)
            ps_model = DecisionModel(template, sys; optimizer = optimizer)
            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            JuMPmodel = PSI.get_jump_model(ps_model)
            n_vars_radial = JuMP.num_variables(JuMPmodel)

            # Radial + degree two reductions:
            template = ProblemTemplate(
                NetworkModel(network_model;
                    reduce_radial_branches = true,
                    reduce_degree_two_branches = true,
                    use_slacks = false),
            )
            set_device_model!(template, Line, StaticBranch)
            set_device_model!(template, Transformer2W, StaticBranch)
            ps_model = DecisionModel(template, sys; optimizer = optimizer)
            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            JuMPmodel = PSI.get_jump_model(ps_model)
            n_vars_radial_d2 = JuMP.num_variables(JuMPmodel)

            @test n_vars_radial_d2 < n_vars_radial < n_vars
        end
    end
end

@testset "Network reductions - PowerModels with slacks" begin
    sys = build_system(PSITestSystems, "case11_network_reductions")
    add_dummy_time_series_data!(sys)
    for (network_model, optimizer) in NETWORKS_FOR_TESTING
        @testset "Network Model: $(network_model)" begin
            template = ProblemTemplate(
                NetworkModel(network_model;
                    reduce_radial_branches = true,
                    reduce_degree_two_branches = true,
                    use_slacks = true),
            )
            set_device_model!(
                template,
                DeviceModel(Line, StaticBranch; use_slacks = true),
            )
            set_device_model!(
                template,
                DeviceModel(Transformer2W, StaticBranch; use_slacks = true),
            )
            ps_model = DecisionModel(template, sys; optimizer = optimizer)
            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
        end
    end
end
