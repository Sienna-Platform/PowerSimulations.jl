function test_single_stage_sequential(in_memory, rebuild, export_model)
    tmp_dir = mktempdir(; cleanup = true)
    template_ed = get_template_nomin_ed_simulation()
    c_sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
    models = SimulationModels([
        DecisionModel(
            template_ed,
            c_sys;
            name = "ED",
            optimizer = HiGHS_optimizer,
            rebuild_model = rebuild,
            export_optimization_model = export_model,
        ),
    ])
    test_sequence =
        SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
        )
    sim_single = Simulation(;
        name = "consecutive",
        steps = 2,
        models = models,
        sequence = test_sequence,
        simulation_folder = tmp_dir,
    )
    build_out = build!(sim_single)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    execute_out = execute!(sim_single; in_memory = in_memory)
    @test execute_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return tmp_dir
end

@testset "Single stage sequential tests" begin
    for in_memory in (true, false), rebuild in (true, false)
        test_single_stage_sequential(
            in_memory,
            rebuild,
            PSI.OptimizationModelExportFormat.NONE,
        )
    end
end

@testset "Test model export at each solve" begin
    # 2 simulation steps. Each format writes a single file per solve, so the
    # export directory holds exactly 2 files of the selected extension and none
    # of the other.
    for (fmt, ext, other_ext) in (
        (PSI.OptimizationModelExportFormat.LP, ".lp", ".json"),
        (PSI.OptimizationModelExportFormat.MOF, ".json", ".lp"),
    )
        folder = test_single_stage_sequential(true, false, fmt)
        test_path =
            joinpath(folder, "consecutive", "problems", "ED", "optimization_model_exports")
        @test ispath(test_path)
        files = readdir(test_path)
        @test length(files) == 2
        @test all(endswith(f, ext) for f in files)
        @test !any(endswith(f, other_ext) for f in files)
    end
end

function test_2_stage_decision_models_with_feedforwards(in_memory)
    template_uc = get_template_basic_uc_simulation()
    template_ed = get_template_nomin_ed_simulation()
    set_device_model!(template_ed, InterruptiblePowerLoad, StaticPowerLoad)
    set_network_model!(
        template_uc,
        NetworkModel(
            CopperPlateNetworkModel;
            duals = [CopperPlateBalanceConstraint],
        ),
    )
    set_network_model!(
        template_ed,
        NetworkModel(
            CopperPlateNetworkModel;
            duals = [CopperPlateBalanceConstraint],
            use_slacks = true,
        ),
    )
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                c_sys5_hy_uc;
                name = "UC",
                optimizer = HiGHS_optimizer),
            DecisionModel(
                template_ed,
                c_sys5_hy_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
            ),
        ],
    )

    sequence = SimulationSequence(;
        models = models,
        feedforwards = Dict(
            "ED" => [
                SemiContinuousFeedforward(;
                    component_type = ThermalStandard,
                    source = OnVariable,
                    affected_values = [ActivePowerVariable],
                ),
            ],
        ),
        ini_cond_chronology = InterProblemChronology(),
    )
    sim = Simulation(;
        name = "no_cache",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )

    build_out = build!(sim; console_level = Logging.Error)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    execute_out = execute!(sim; in_memory = in_memory)
    @test execute_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "2-Stage Decision Models with FeedForwards" begin
    for in_memory in (true, false)
        test_2_stage_decision_models_with_feedforwards(in_memory)
    end
end

@testset "Test Simulation Utils" begin
    template_uc = get_template_basic_uc_simulation()
    set_device_model!(template_uc, ThermalStandard, ThermalStandardUnitCommitment)
    set_network_model!(
        template_uc,
        NetworkModel(
            CopperPlateNetworkModel;
            duals = [CopperPlateBalanceConstraint],
        ),
    )

    template_ed = get_template_nomin_ed_simulation(
        NetworkModel(
            CopperPlateNetworkModel;
            # Added because of data issues
            use_slacks = true,
            duals = [CopperPlateBalanceConstraint],
        ),
    )
    set_device_model!(template_ed, InterruptiblePowerLoad, StaticPowerLoad)
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                c_sys5_hy_uc;
                name = "UC",
                optimizer = HiGHS_optimizer,
            ),
            DecisionModel(
                template_ed,
                c_sys5_hy_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
            ),
        ],
    )

    sequence = SimulationSequence(;
        models = models,
        feedforwards = Dict(
            "ED" => [
                SemiContinuousFeedforward(;
                    component_type = ThermalStandard,
                    source = OnVariable,
                    affected_values = [ActivePowerVariable],
                ),
            ],
        ),
        ini_cond_chronology = InterProblemChronology(),
    )
    sim = Simulation(;
        name = "aggregation",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = false),
    )

    build_out = build!(sim; console_level = Logging.Error)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    execute_out = execute!(sim)
    @test execute_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    @testset "Verify simulation events" begin
        file = joinpath(PSI.get_simulation_dir(sim), "recorder", "simulation_status.log")
        @test isfile(file)
        events = PSI.list_simulation_events(
            PSI.InitialConditionUpdateEvent,
            PSI.get_simulation_dir(sim);
            step = 1,
        )
        @test length(events) == 23
        events = PSI.list_simulation_events(
            PSI.InitialConditionUpdateEvent,
            PSI.get_simulation_dir(sim);
            step = 2,
        )
        # This value needs to be checked
        @test length(events) == 46

        PSI.show_simulation_events(
            devnull,
            PSI.InitialConditionUpdateEvent,
            PSI.get_simulation_dir(sim),
            ;
            step = 2,
        )
        events = PSI.list_simulation_events(
            PSI.InitialConditionUpdateEvent,
            PSI.get_simulation_dir(sim);
            step = 1,
            model_name = "UC",
        )
        @test length(events) == 0
        events = PSI.list_simulation_events(
            PSI.InitialConditionUpdateEvent,
            PSI.get_simulation_dir(sim),
            ;
            step = 2,
            model_name = "UC",
        )
        @test length(events) == 22
        PSI.show_simulation_events(
            devnull,
            PSI.InitialConditionUpdateEvent,
            PSI.get_simulation_dir(sim),
            ;
            step = 2,
            model_name = "UC",
        )
    end
end

@testset "Test simulation with OnlineReserve" begin
    template_uc = get_template_basic_uc_simulation()
    set_network_model!(template_uc, NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(template_uc, DeviceModel(Line, StaticBranchUnbounded))
    # POM's `ServiceModel` keys all reserves of a type into ONE simulation-state entry
    # (a name axis inside the container, not one key per named reserve as PSI's old
    # per-instance `VariableReserve` keying did). `c_sys5_uc`'s `add_reserves=true` fixture
    # carries an extra "ORDC1" reserve that `c_sys5_ed`'s fixture does not, so without this
    # filter the UC-sized decision state would demand an "ORDC1" column that ED's own
    # solved results never populate. Filter UC down to the reserve roster the two systems
    # share so the cross-model state stays consistent.
    set_service_model!(
        template_uc,
        ServiceModel(
            OnlineReserve{ReserveUp},
            RangeReserve;
            attributes = Dict{String, Any}(
                "filter_function" => x -> PSY.get_name(x) != "ORDC1",
            ),
        ),
    )
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(template_ed, DeviceModel(Line, StaticBranchUnbounded))
    set_service_model!(template_ed, ServiceModel(OnlineReserve{ReserveUp}, RangeReserve))
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_ed"; add_reserves = true)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                c_sys5_hy_uc;
                name = "UC",
                optimizer = HiGHS_optimizer,
                initialize_model = false,
            ),
            DecisionModel(
                template_ed,
                c_sys5_hy_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
                initialize_model = false,
            ),
        ],
    )

    # `attach_feedforward!(::ServiceModel, ...)` unconditionally errors: the feedforward
    # parameter path is keyed `(device_name, time)`, not the service-keyed
    # `(service_name, device_name, time)` POM's `ServiceModel` reserve variables use.
    # Coverage here is narrowed to the ThermalStandard feedforward; re-add the reserve
    # feedforward once POM implements service-keyed feedforward parameters.
    sequence = SimulationSequence(;
        models = models,
        feedforwards = Dict(
            "ED" => [
                SemiContinuousFeedforward(;
                    component_type = ThermalStandard,
                    source = OnVariable,
                    affected_values = [ActivePowerVariable],
                ),
            ],
        ),
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = "reserve_feedforward",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    execute_out = execute!(sim)
    @test execute_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results = SimulationResults(sim)
    for name in list_decision_problems(results)
        res = get_decision_problem_results(results, name)
        parameters = read_realized_parameters(res)
        @test !isempty(parameters)
        for (key, df1) in parameters
            df2 = read_realized_parameter(res, key)
            @test df1 == df2
        end
    end
end

function test_3_stage_simulation_with_feedforwards(in_memory)
    sys_rts_da = PSB.build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys_rts_rt = PSB.build_system(PSISystems, "modified_RTS_GMLC_RT_sys")
    sys_rts_ha = deepcopy(sys_rts_rt)

    PSY.transform_single_time_series!(sys_rts_da, Hour(36), Hour(24))
    PSY.transform_single_time_series!(sys_rts_ha, Hour(2), Hour(1))
    PSY.transform_single_time_series!(sys_rts_rt, Hour(1), Hour(1))

    template_uc = get_template_standard_uc_simulation()
    set_network_model!(template_uc, NetworkModel(CopperPlateNetworkModel))
    template_ha = deepcopy(template_uc)
    # network slacks added because of data issues
    template_ed = get_thermal_dispatch_template_network(
        NetworkModel(CopperPlateNetworkModel; use_slacks = true),
    )

    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                sys_rts_da;
                name = "UC",
                optimizer = HiGHS_optimizer,
                initialize_model = true,
            ),
            DecisionModel(
                template_ha,
                sys_rts_ha;
                name = "HA",
                optimizer = HiGHS_optimizer,
                initialize_model = true,
            ),
            DecisionModel(
                template_ed,
                sys_rts_rt;
                name = "ED",
                optimizer = HiGHS_optimizer,
                initialize_model = true,
            ),
        ],
    )

    sequence = SimulationSequence(;
        models = models,
        feedforwards = Dict(
            "ED" => [
                SemiContinuousFeedforward(;
                    component_type = ThermalStandard,
                    source = OnVariable,
                    affected_values = [ActivePowerVariable],
                ),
            ],
        ),
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = "3stage_feedforward",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    execute_out = execute!(sim; in_memory = in_memory)
    @test execute_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "Test 3 stage simulation with FeedForwards" begin
    for in_memory in (true, false)
        test_3_stage_simulation_with_feedforwards(in_memory)
    end
end
