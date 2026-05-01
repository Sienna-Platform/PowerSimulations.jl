@testset "Test Reserves from Thermal Dispatch" begin
    template = get_thermal_dispatch_template_network(CopperPlatePowerModel)
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve1"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve11"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveDown}, RangeReserve, "Reserve2"),
    )
    set_service_model!(
        template,
        ServiceModel(ReserveDemandCurve{ReserveUp}, StepwiseCostReserve, "ORDC1"),
    )

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 624, 0, 216, 216, 48, false)
    reserve_variables = [
        :ActivePowerReserveVariable__VariableReserve__ReserveUp__Reserve1
        :ActivePowerReserveVariable__ReserveDemandCurve__ReserveUp__ORDC1
        :ActivePowerReserveVariable__VariableReserve__ReserveDown__Reserve2
        :ActivePowerReserveVariable__VariableReserve__ReserveUp__Reserve11
    ]
    found_vars = 0
    for (k, var_array) in PSI.get_optimization_container(model).variables
        if ISOPT.encode_key(k) in reserve_variables
            for var in var_array
                @test JuMP.has_lower_bound(var)
                @test JuMP.lower_bound(var) == 0.0
            end
            found_vars += 1
        end
    end
    @test found_vars == 4
end

@testset "Test Ramp Reserves from Thermal Dispatch" begin
    template = get_thermal_dispatch_template_network(CopperPlatePowerModel)
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RampReserve, "Reserve1"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RampReserve, "Reserve11"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveDown}, RampReserve, "Reserve2"),
    )

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 384, 0, 336, 192, 24, false)
    reserve_variables = [
        :ActivePowerReserveVariable__VariableReserve_ReserveDown_Reserve2,
        :ActivePowerReserveVariable__VariableReserve_ReserveUp_Reserve1,
        :ActivePowerReserveVariable__VariableReserve_ReserveUp_Reserve11,
    ]
    for (k, var_array) in PSI.get_optimization_container(model).variables
        if ISOPT.encode_key(k) in reserve_variables
            for var in var_array
                @test JuMP.has_lower_bound(var)
                @test JuMP.lower_bound(var) == 0.0
            end
        end
    end
end

@testset "Test Reserves from Thermal Standard UC" begin
    template = get_thermal_standard_uc_template()
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve1"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve11"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveDown}, RangeReserve, "Reserve2"),
    )
    set_service_model!(
        template,
        ServiceModel(ReserveDemandCurve{ReserveUp}, StepwiseCostReserve, "ORDC1"),
    )
    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    model = DecisionModel(template, c_sys5_uc; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 984, 0, 576, 216, 168, true)
end

@testset "Test Reserves from Thermal Standard UC with NonSpinningReserve" begin
    template = get_thermal_standard_uc_template()
    set_device_model!(
        template,
        DeviceModel(ThermalMultiStart, ThermalStandardUnitCommitment),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserveNonSpinning, NonSpinningReserve, "NonSpinningReserve"),
    )

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc_non_spin"; add_reserves = true)
    model = DecisionModel(template, c_sys5_uc; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 1032, 0, 888, 192, 288, true)
end

@testset "Test Upwards Reserves from Renewable Dispatch" begin
    template = ProblemTemplate(CopperPlatePowerModel)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve3"),
    )
    set_service_model!(
        template,
        ServiceModel(ReserveDemandCurve{ReserveUp}, StepwiseCostReserve, "ORDC1"),
    )

    c_sys5_re = PSB.build_system(PSITestSystems, "c_sys5_re"; add_reserves = true)
    model = DecisionModel(template, c_sys5_re)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 336, 0, 168, 120, 48, false)
end

@testset "Test Reserves from Hydro" begin
    template = ProblemTemplate(CopperPlatePowerModel)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, HydroTurbine, HydroTurbineEnergyDispatch)
    set_device_model!(template, HydroReservoir, HydroEnergyModelReservoir)
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve5"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveDown}, RangeReserve, "Reserve6"),
    )
    set_service_model!(
        template,
        ServiceModel(ReserveDemandCurve{ReserveUp}, StepwiseCostReserve, "ORDC1"),
    )

    c_sys5_hyd = PSB.build_system(PSITestSystems, "c_sys5_hyd"; add_reserves = true)
    model = DecisionModel(template, c_sys5_hyd)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 312, 0, 120, 96, 72, false)
end

@testset "Test Reserves from with slack variables" begin
    template = get_thermal_dispatch_template_network(
        NetworkModel(CopperPlatePowerModel; use_slacks = true),
    )
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RangeReserve,
            "Reserve1";
            use_slacks = true,
        ),
    )
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RangeReserve,
            "Reserve11";
            use_slacks = true,
        ),
    )
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveDown},
            RangeReserve,
            "Reserve2";
            use_slacks = true,
        ),
    )

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    model = DecisionModel(template, c_sys5_uc;)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 504, 0, 120, 192, 24, false)
end

#=
@testset "Test AGC" begin
    c_sys5_reg = PSB.build_system(PSITestSystems, "c_sys5_reg")
    @test_throws ArgumentError template_agc_reserve_deployment(; dummy_arg = 0.0)

    template_agc = template_agc_reserve_deployment()
    set_service_model!(template_agc, ServiceModel(PSY.AGC, PIDSmoothACE, "AGC_Area1"))
    agc_problem = DecisionModel(AGCReserveDeployment, template_agc, c_sys5_reg)
    @test build!(agc_problem; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    # These values might change as the AGC model is refined
    moi_tests(agc_problem, 696, 0, 480, 0, 384, false)
end

@testset "Test GroupReserve from Thermal Dispatch" begin
    template = get_thermal_dispatch_template_network()
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve1"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve11"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveDown}, RangeReserve, "Reserve2"),
    )
    set_service_model!(
        template,
        ServiceModel(ReserveDemandCurve{ReserveUp}, StepwiseCostReserve, "ORDC1"),
    )
    set_service_model!(
        template,
        ServiceModel(ConstantReserveGroup{ReserveDown}, GroupReserve, "init"),
    )

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    services = get_components(Service, c_sys5_uc)
    contributing_services = Vector{Service}()
    for service in services
        if !(typeof(service) <: PSY.ReserveDemandCurve)
            push!(contributing_services, service)
        end
    end
    groupservice = ConstantReserveGroup{ReserveDown}(;
        name = "init",
        available = true,
        requirement = 0.0,
        ext = Dict{String, Any}(),
    )
    add_service!(c_sys5_uc, groupservice, contributing_services)

    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 624, 0, 216, 240, 48, false)
end

@testset "Test GroupReserve Errors" begin
    template = get_thermal_dispatch_template_network()
    set_service_model!(template, ServiceModel(VariableReserve{ReserveUp}, RangeReserve))
    set_service_model!(template, ServiceModel(VariableReserve{ReserveDown}, RangeReserve))
    set_service_model!(
        template,
        ServiceModel(ReserveDemandCurve{ReserveUp}, StepwiseCostReserve),
    )
    set_service_model!(
        template,
        ServiceModel(ConstantReserveGroup{ReserveDown}, GroupReserve),
    )

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    services = get_components(Service, c_sys5_uc)
    contributing_services = Vector{Service}()
    for service in services
        if !(typeof(service) <: PSY.ReserveDemandCurve)
            push!(contributing_services, service)
        end
    end
    groupservice = ConstantReserveGroup{ReserveDown}(;
        name = "init",
        available = true,
        requirement = 0.0,
        ext = Dict{String, Any}(),
    )
    add_service!(c_sys5_uc, groupservice, contributing_services)

    off_service = VariableReserve{ReserveUp}("Reserveoff", true, 0.6, 10)
    push!(groupservice.contributing_services, off_service)

    model = DecisionModel(template, c_sys5_uc)
    @test build!(
        model;
        output_dir = mktempdir(; cleanup = true),
        console_level = Logging.AboveMaxLevel,
    ) == PSI.ModelBuildStatus.FAILED
end

@testset "Test ConstantReserve" begin
    template = get_thermal_dispatch_template_network()
    set_service_model!(
        template,
        ServiceModel(ConstantReserve{ReserveUp}, RangeReserve, "Reserve3"),
    )

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    static_reserve = ConstantReserve{ReserveUp}("Reserve3", true, 30, 100)
    add_service!(c_sys5_uc, static_reserve, get_components(ThermalGen, c_sys5_uc))
    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test typeof(model) <: DecisionModel{<:PSI.DecisionProblem}
end
=#

@testset "Test Reserves with Feedforwards" begin
    template = get_thermal_dispatch_template_network()
    service_model = ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve1")
    ff_lb = LowerBoundFeedforward(;
        component_type = VariableReserve{ReserveUp},
        source = ActivePowerReserveVariable,
        affected_values = [ActivePowerReserveVariable],
        meta = "Reserve1",
    )
    PSI.attach_feedforward!(service_model, ff_lb)

    set_service_model!(template, service_model)

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    model = DecisionModel(template, c_sys5_uc; optimizer = HiGHS_optimizer)
    # set manually to test cases for simulation
    PSI.get_optimization_container(model).built_for_recurrent_solves = true
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 456, 0, 120, 264, 24, false)
end

@testset "Test Reserves with Participation factor limits" begin
    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    for service in get_components(Reserve, c_sys5_uc)
        set_max_participation_factor!(service, 0.8)
    end

    template = get_thermal_dispatch_template_network(CopperPlatePowerModel)
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve1"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve11"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveDown}, RangeReserve, "Reserve2"),
    )
    set_service_model!(
        template,
        ServiceModel(ReserveDemandCurve{ReserveUp}, StepwiseCostReserve, "ORDC1"),
    )

    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 624, 0, 480, 216, 48, false)
    reserve_variables = [
        :ActivePowerReserveVariable__VariableReserve__ReserveUp__Reserve1
        :ActivePowerReserveVariable__ReserveDemandCurve__ReserveUp__ORDC1
        :ActivePowerReserveVariable__VariableReserve__ReserveDown__Reserve2
        :ActivePowerReserveVariable__VariableReserve__ReserveUp__Reserve11
    ]
    found_vars = 0
    for (k, var_array) in PSI.get_optimization_container(model).variables
        if ISOPT.encode_key(k) in reserve_variables
            for var in var_array
                @test JuMP.has_lower_bound(var)
                @test JuMP.lower_bound(var) == 0.0
            end
            found_vars += 1
        end
    end
    @test found_vars == 4

    participation_constraints = [
        :ParticipationFractionConstraint__VariableReserve__ReserveUp__Reserve11,
        :ParticipationFractionConstraint__VariableReserve__ReserveDown__Reserve2,
    ]

    found_constraints = 0

    for (k, _) in PSI.get_optimization_container(model).constraints
        if ISOPT.encode_key(k) in participation_constraints
            found_constraints += 1
        end
    end

    @test found_constraints == 2
end

@testset "Test Transmission Interface" begin
    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    interface = TransmissionInterface(;
        name = "west_east",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 400.0),
    )
    interface_lines = [
        get_component(Line, c_sys5_uc, "1"),
        get_component(Line, c_sys5_uc, "2"),
        get_component(Line, c_sys5_uc, "6"),
    ]
    add_service!(c_sys5_uc, interface, interface_lines)

    template = get_thermal_dispatch_template_network(DCPPowerModel)
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, ConstantMaxInterfaceFlow; use_slacks = true),
    )

    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 432, 144, 288, 288, 288, false)

    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, ConstantMaxInterfaceFlow; use_slacks = true),
    )
    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 168, 0, 288, 288, 24, false)

    #= TODO: Implement Interfaces in AC
    template = get_thermal_dispatch_template_network(ACPPowerModel)
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, ConstantMaxInterfaceFlow; use_slacks = true),
    )
    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) == PSI.BuildStatus.BUILT
    moi_tests(model, 312, 0, 288, 288, 168, false)
    =#
end

@testset "Test Transmission Interface with TimeSeries" begin
    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    interface = TransmissionInterface(;
        name = "west_east",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 400.0),
    )
    interface_lines = [
        get_component(Line, c_sys5_uc, "1"),
        get_component(Line, c_sys5_uc, "2"),
        get_component(Line, c_sys5_uc, "6"),
    ]
    add_service!(c_sys5_uc, interface, interface_lines)
    # Add TimeSeries Data
    data_minflow = Dict(
        DateTime("2024-01-01T00:00:00") => zeros(24),
        DateTime("2024-01-02T00:00:00") => zeros(24),
    )

    forecast_minflow = Deterministic(
        "min_active_power_flow_limit",
        data_minflow,
        Hour(1);
        scaling_factor_multiplier = get_min_active_power_flow_limit,
    )

    data_maxflow = Dict(
        DateTime("2024-01-01T00:00:00") => [
            0.9, 0.85, 0.95, 0.2, 0.15, 0.2,
            0.9, 0.85, 0.95, 0.2, 0.15, 0.2,
            0.9, 0.85, 0.95, 0.2, 0.5, 0.5,
            0.9, 0.85, 0.95, 0.2, 0.6, 0.6,
        ],
        DateTime("2024-01-02T00:00:00") => [
            0.9, 0.85, 0.95, 0.2, 0.15, 0.2,
            0.9, 0.85, 0.95, 0.2, 0.15, 0.2,
            0.9, 0.85, 0.95, 0.2, 0.5, 0.5,
            0.9, 0.85, 0.95, 0.2, 0.6, 0.6,
        ],
    )

    forecast_maxflow = Deterministic(
        "max_active_power_flow_limit",
        data_maxflow,
        Hour(1);
        scaling_factor_multiplier = get_max_active_power_flow_limit,
    )

    add_time_series!(c_sys5_uc, interface, forecast_minflow)
    add_time_series!(c_sys5_uc, interface, forecast_maxflow)

    template = get_thermal_dispatch_template_network(DCPPowerModel)
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, VariableMaxInterfaceFlow; use_slacks = true),
    )

    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 432, 144, 288, 288, 288, false)

    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, VariableMaxInterfaceFlow; use_slacks = true),
    )
    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    moi_tests(model, 168, 0, 288, 288, 24, false)
end

@testset "Test Transmission Interface with Feedforwards" begin
    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    interface = TransmissionInterface(;
        name = "west_east",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 400.0),
    )
    interface_lines = [
        get_component(Line, c_sys5_uc, "1"),
        get_component(Line, c_sys5_uc, "2"),
        get_component(Line, c_sys5_uc, "6"),
    ]
    add_service!(c_sys5_uc, interface, interface_lines)
    c_sys5_uc2 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    interface2 = TransmissionInterface(;
        name = "west_east",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 400.0),
    )
    interface_lines2 = [
        get_component(Line, c_sys5_uc2, "1"),
        get_component(Line, c_sys5_uc2, "2"),
        get_component(Line, c_sys5_uc2, "6"),
    ]
    add_service!(c_sys5_uc2, interface2, interface_lines2)

    template = get_thermal_dispatch_template_network(DCPPowerModel)
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, ConstantMaxInterfaceFlow; use_slacks = true),
    )
    models = SimulationModels(;
        decision_models = [
            DecisionModel(template, c_sys5_uc; optimizer = HiGHS_optimizer, name = "Sys1"),
            DecisionModel(template, c_sys5_uc2; optimizer = HiGHS_optimizer, name = "Sys2"),
        ],
    )

    feedforward = Dict(
        "Sys2" => [
            FixValueFeedforward(;
                component_type = TransmissionInterface,
                source = PSI.FlowActivePowerVariable,
                affected_values = [PSI.FlowActivePowerVariable],
            ),
        ],
    )

    sequence = SimulationSequence(;
        models = models,
        ini_cond_chronology = InterProblemChronology(),
        feedforwards = feedforward,
    )

    sim = Simulation(;
        name = "interface-fail",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test_throws ArgumentError build!(sim; console_level = Logging.AboveMaxLevel)
end

@testset "2 Areas AreaBalance With Transmission Interface" begin
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

@testset "Test Interfaces on Interchanges with AreaBalance" begin
    sys_rts_da = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    transform_single_time_series!(sys_rts_da, Hour(24), Hour(1))
    interchange1 = AreaInterchange(;
        name = "interchange1_2",
        available = true,
        active_power_flow = 100.0,
        flow_limits = (from_to = 1.0, to_from = 1.0),
        from_area = get_component(Area, sys_rts_da, "1"),
        to_area = get_component(Area, sys_rts_da, "2"),
    )
    interchange2 = AreaInterchange(;
        name = "interchange1_3",
        available = true,
        active_power_flow = 100.0,
        flow_limits = (from_to = 1.0, to_from = 1.0),
        from_area = get_component(Area, sys_rts_da, "1"),
        to_area = get_component(Area, sys_rts_da, "3"),
    )
    interchange3 = AreaInterchange(;
        name = "interchange3_2",
        available = true,
        active_power_flow = 100.0,
        flow_limits = (from_to = 1.0, to_from = 1.0),
        from_area = get_component(Area, sys_rts_da, "3"),
        to_area = get_component(Area, sys_rts_da, "2"),
    )
    add_components!(
        sys_rts_da,
        [interchange1, interchange2, interchange3],
    )
    # This interface is limiting all the flows into 1
    interface = TransmissionInterface(;
        name = "interface1_2_3",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 1.0),
        violation_penalty = 1000.0,
        direction_mapping = Dict("interchange1_2" => 1,
            "interchange1_3" => -1,
        ),
    )
    add_service!(
        sys_rts_da,
        interface,
        [interchange1, interchange2],
    )
    template = ProblemTemplate(NetworkModel(AreaBalancePowerModel))
    set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, RenewableNonDispatch, FixedOutput)
    set_device_model!(template, HydroDispatch, HydroDispatchRunOfRiver)
    set_device_model!(template, AreaInterchange, StaticBranch)
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, ConstantMaxInterfaceFlow),
    )
    ps_model =
        DecisionModel(
            template,
            sys_rts_da;
            resolution = Hour(1),
            optimizer = HiGHS_optimizer,
            store_variable_names = true,
        )

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 8568, 0, 2136, 1416, 2664, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (3, 24)

    interchange_constraints_ub =
        PSI.get_constraint(opt_container, InterfaceFlowLimit(), TransmissionInterface, "ub")
    interchange_constraints_lb =
        PSI.get_constraint(opt_container, InterfaceFlowLimit(), TransmissionInterface, "lb")
    @test size(interchange_constraints_ub) == (1, 24)
    @test size(interchange_constraints_lb) == (1, 24)

    interchange_constraints_ub["interface1_2_3", 1]
    # Obj Function test is disabled because with this constraints
    # the model goes into the slacks and the cost is larger than 1e6
    # psi_checksolve_test(ps_model, [MOI.OPTIMAL], 482055, 1)

    results = OptimizationProblemResults(ps_model)
    interface_results =
        read_expression(
            results,
            "InterfaceTotalFlow__TransmissionInterface";
            table_format = TableFormat.WIDE,
        )
    for i in 1:24
        @test interface_results[!, "interface1_2_3"][i] <= 100.0 + PSI.ABSOLUTE_TOLERANCE
    end
end

@testset "Test Interfaces on Interchanges and Double Circuits with AreaPTDFPowerModel" begin
    sys_rts_da = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    transform_single_time_series!(sys_rts_da, Hour(24), Hour(1))
    interchange1 = AreaInterchange(;
        name = "interchange1_2",
        available = true,
        active_power_flow = 100.0,
        flow_limits = (from_to = 1.0, to_from = 1.0),
        from_area = get_component(Area, sys_rts_da, "1"),
        to_area = get_component(Area, sys_rts_da, "2"),
    )
    interchange2 = AreaInterchange(;
        name = "interchange1_3",
        available = true,
        active_power_flow = 100.0,
        flow_limits = (from_to = 1.0, to_from = 1.0),
        from_area = get_component(Area, sys_rts_da, "1"),
        to_area = get_component(Area, sys_rts_da, "3"),
    )
    interchange3 = AreaInterchange(;
        name = "interchange3_2",
        available = true,
        active_power_flow = 100.0,
        flow_limits = (from_to = 1.0, to_from = 1.0),
        from_area = get_component(Area, sys_rts_da, "3"),
        to_area = get_component(Area, sys_rts_da, "2"),
    )
    add_components!(
        sys_rts_da,
        [interchange1, interchange2, interchange3],
    )
    # This interface is limiting all the flows into 1
    interface1 = TransmissionInterface(;
        name = "interface1_2_3",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 1.0),
        violation_penalty = 1000.0,
        direction_mapping = Dict("interchange1_2" => 1,
            "interchange1_3" => -1,
        ),
    )
    add_service!(
        sys_rts_da,
        interface1,
        [interchange1, interchange2],
    )

    #Add an interface on a double circuit:
    double_circuit_1 = get_component(Line, sys_rts_da, "A33-1")
    double_circuit_2 = get_component(Line, sys_rts_da, "A33-2")
    interface2 = TransmissionInterface(;
        name = "interface_double_circuit",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 1.0),
        violation_penalty = 1000.0,
        direction_mapping = Dict("A33-1" => 1, "A33-2" => 1),
    )
    add_service!(
        sys_rts_da,
        interface2,
        [double_circuit_1, double_circuit_2],
    )

    template = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks = true))
    set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, RenewableNonDispatch, FixedOutput)
    set_device_model!(template, HydroDispatch, HydroDispatchRunOfRiver)
    set_device_model!(template, Line, StaticBranchUnbounded)
    set_device_model!(
        template,
        DeviceModel(AreaInterchange, StaticBranchUnbounded; use_slacks = false),
    )
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, ConstantMaxInterfaceFlow),
    )
    ps_model =
        DecisionModel(
            template,
            sys_rts_da;
            resolution = Hour(1),
            optimizer = HiGHS_optimizer,
            store_variable_names = true,
            optimizer_solve_log_print = true,
        )

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 8712, 0, 2160, 1440, 2664, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (3, 24)

    interchange_constraints_ub =
        PSI.get_constraint(opt_container, InterfaceFlowLimit(), TransmissionInterface, "ub")
    interchange_constraints_lb =
        PSI.get_constraint(opt_container, InterfaceFlowLimit(), TransmissionInterface, "lb")
    @test size(interchange_constraints_ub) == (2, 24)
    @test size(interchange_constraints_lb) == (2, 24)

    interchange_constraints_ub["interface1_2_3", 1]
    # Obj Function test is disabled because with this constraints
    # the model goes into the slacks and the cost is larger than 1e6
    # psi_checksolve_test(ps_model, [MOI.OPTIMAL], 482055, 1)

    results = OptimizationProblemResults(ps_model)
    interface_results =
        read_expression(
            results,
            "InterfaceTotalFlow__TransmissionInterface";
            table_format = TableFormat.WIDE,
        )
    for i in 1:24
        @test interface_results[!, "interface1_2_3"][i] <= 100.0 + PSI.ABSOLUTE_TOLERANCE
    end
end

@testset "Test bad data for interfaces with reductions" begin
    sys_rts_da = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    transform_single_time_series!(sys_rts_da, Hour(24), Hour(1))

    # Add an interface on a double circuit:
    double_circuit_1 = get_component(Line, sys_rts_da, "A33-1")
    double_circuit_2 = get_component(Line, sys_rts_da, "A33-2")
    interface_double_circuit = TransmissionInterface(;
        name = "interface_double_circuit",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 1.0),
        violation_penalty = 1000.0,
        direction_mapping = Dict("A33-1" => 1, "A33-2" => 1),
    )
    add_service!(
        sys_rts_da,
        interface_double_circuit,
        [double_circuit_1, double_circuit_2],
    )

    # Add interface on a series chain:
    series_chain_1 = get_component(Line, sys_rts_da, "CA-1")
    series_chain_2 = get_component(Line, sys_rts_da, "C35")
    interface_series_chain = TransmissionInterface(;
        name = "interface_series_chain",
        available = true,
        active_power_flow_limits = (min = 0.0, max = 1.0),
        violation_penalty = 1000.0,
        direction_mapping = Dict("CA-1" => -1, "C35" => -1),
    )
    # Order matters here; must compute the ptdf before adding the service for the lines that 
    # are included in the interface to be reduced (in order to test the bad data checking)
    ptdf = PTDF(sys_rts_da; network_reductions = NetworkReduction[DegreeTwoReduction()])
    add_service!(
        sys_rts_da,
        interface_series_chain,
        [series_chain_1, series_chain_2],
    )
    template = ProblemTemplate(
        NetworkModel(
            AreaPTDFPowerModel;
            PTDF_matrix = ptdf,
            reduce_degree_two_branches = true,
            use_slacks = true,
        ),
    )
    set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, RenewableNonDispatch, FixedOutput)
    set_device_model!(template, HydroDispatch, HydroDispatchRunOfRiver)
    set_device_model!(template, Line, StaticBranchUnbounded)
    set_device_model!(
        template,
        DeviceModel(AreaInterchange, StaticBranchUnbounded; use_slacks = false),
    )
    set_service_model!(
        template,
        ServiceModel(TransmissionInterface, ConstantMaxInterfaceFlow),
    )
    ps_model =
        DecisionModel(
            template,
            sys_rts_da;
            resolution = Hour(1),
            optimizer = HiGHS_optimizer,
            store_variable_names = true,
            optimizer_solve_log_print = true,
        )

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    # Test bad direction data for interface on series chain :
    set_direction_mapping!(interface_series_chain, Dict("CA-1" => 1, "C35" => -1))
    ps_model =
        DecisionModel(
            template,
            sys_rts_da;
            resolution = Hour(1),
            optimizer = HiGHS_optimizer,
            store_variable_names = true,
        )
    @test build!(
        ps_model;
        console_level = Logging.AboveMaxLevel,  # Ignore expected errors.
        output_dir = mktempdir(; cleanup = true),
    ) == PSI.ModelBuildStatus.FAILED

    # Test bad direction data for interface on double circuit:
    set_direction_mapping!(interface_series_chain, Dict("CA-1" => 1, "C35" => 1))
    set_direction_mapping!(interface_double_circuit, Dict("A33-1" => 1, "A33-2" => -1))
    ps_model =
        DecisionModel(
            template,
            sys_rts_da;
            resolution = Hour(1),
            optimizer = HiGHS_optimizer,
            store_variable_names = true,
            optimizer_solve_log_print = true,
        )
    @test build!(
        ps_model;
        console_level = Logging.AboveMaxLevel,  # Ignore expected errors.
        output_dir = mktempdir(; cleanup = true),
    ) == PSI.ModelBuildStatus.FAILED
    set_direction_mapping!(interface_double_circuit, Dict("A33-1" => 1, "A33-2" => 1))

    # Test only including part of a double circuit in an interface:
    pop!(get_services(double_circuit_1))
    ps_model =
        DecisionModel(
            template,
            sys_rts_da;
            resolution = Hour(1),
            optimizer = HiGHS_optimizer,
            store_variable_names = true,
        )
    @test build!(
        ps_model;
        console_level = Logging.AboveMaxLevel,  # Ignore expected errors.
        output_dir = mktempdir(; cleanup = true),
    ) == PSI.ModelBuildStatus.FAILED

    # Test only including part of a series chain in an interface:
    push!(get_services(double_circuit_1), interface_double_circuit)
    pop!(get_services(series_chain_1))
    ps_model =
        DecisionModel(
            template,
            sys_rts_da;
            resolution = Hour(1),
            optimizer = HiGHS_optimizer,
            store_variable_names = true,
        )
    @test build!(
        ps_model;
        console_level = Logging.AboveMaxLevel,  # Ignore expected errors.
        output_dir = mktempdir(; cleanup = true),
    ) == PSI.ModelBuildStatus.FAILED
end

@testset "Test Multiple GroupReserve Constraints" begin
    template = get_thermal_dispatch_template_network()
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve1"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "Reserve11"),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveDown}, RangeReserve, "Reserve2"),
    )
    set_service_model!(
        template,
        ServiceModel(ReserveDemandCurve{ReserveUp}, StepwiseCostReserve, "ORDC1"),
    )

    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    services = get_components(Service, c_sys5_uc)
    requirement_group1 = 5.0
    contributing_services = Vector{Service}()
    for service in services
        if (typeof(service) <: PSY.VariableReserve{ReserveUp})
            push!(contributing_services, service)
        end
    end
    groupservice = ConstantReserveGroup{ReserveSymmetric}(;
        name = "group_reserve_1",
        available = true,
        requirement = requirement_group1,
        ext = Dict{String, Any}(),
    )
    add_service!(c_sys5_uc, groupservice, contributing_services)

    contributing_services = Vector{Service}()
    requirement_group2 = 10.0
    for service in services
        if (typeof(service) <: PSY.VariableReserve{ReserveUp}) || (typeof(service) <: PSY.VariableReserve{ReserveDown})
            push!(contributing_services, service)
        end
    end
    groupservice = ConstantReserveGroup{ReserveSymmetric}(;
        name = "group_reserve_2",
        available = true,
        requirement = requirement_group2,
        ext = Dict{String, Any}(),
    )
    add_service!(c_sys5_uc, groupservice, contributing_services)

    set_service_model!(
        template,
        ServiceModel(ConstantReserveGroup{ReserveSymmetric}, GroupReserve),
    )

    model = DecisionModel(template, c_sys5_uc)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = model.internal.container

    group_reserve_key1 = PSI.ConstraintKey(
        RequirementConstraint,
        ConstantReserveGroup{ReserveSymmetric},
        "group_reserve_1",
    )
    cons1 = PSI.get_constraint(container, group_reserve_key1)

    group_reserve_key2 = PSI.ConstraintKey(
        RequirementConstraint,
        ConstantReserveGroup{ReserveSymmetric},
        "group_reserve_2",
    )
    cons2 = PSI.get_constraint(container, group_reserve_key2)

    @test all((
        all(JuMP.normalized_rhs.(cons1).data .== requirement_group1),
        all(JuMP.normalized_rhs.(cons2).data .== requirement_group2),
    ))

end
