@testset "InitialReservoirVolume update between simulation steps" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_hy_turbine_head")

    template = PowerOperationsProblemTemplate(NetworkModel(CopperPlateNetworkModel))
    set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, HydroTurbine, HydroTurbineWaterLinearDispatch)
    reservoir_model = DeviceModel(
        HydroReservoir,
        HydroWaterModelReservoir;
        attributes = Dict("hydro_target" => false, "hydro_budget" => false),
    )
    set_device_model!(template, reservoir_model)

    decision_model = DecisionModel(
        template,
        sys;
        name = "UC",
        optimizer = HiGHS_optimizer,
        store_variable_names = true,
        optimizer_solve_log_print = false,
    )
    models = SimulationModels([decision_model])
    sequence = SimulationSequence(;
        models = models,
        ini_cond_chronology = InterProblemChronology(),
    )
    sim = Simulation(;
        name = "hydro_reservoir_sim",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )

    build_out = build!(sim; console_level = Logging.Error)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    execute_out = execute!(sim)
    @test execute_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = IOM.get_optimization_container(decision_model)
    ic = only(
        IOM.get_initial_condition(
            container,
            POM.InitialReservoirVolume(),
            PSY.HydroReservoir,
        ),
    )
    step2_ic_value = IOM.get_condition(ic)

    results = SimulationResults(sim)
    uc_results = get_decision_problem_results(results, "UC")
    set_system!(uc_results, sys)
    volume = read_realized_variable(
        uc_results,
        "HydroReservoirVolumeVariable__HydroReservoir",
    )
    step1_last_volume = volume[24, "value"]

    @test isapprox(step2_ic_value, step1_last_volume; atol = 1e-6)
end
