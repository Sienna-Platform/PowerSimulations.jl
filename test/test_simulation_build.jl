@testset "Simulation Build Tests" begin
    models = create_simulation_build_test_problems(get_template_basic_uc_simulation())
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
        name = "test",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )

    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT

    for field in fieldnames(SimulationSequence)
        if fieldtype(SimulationSequence, field) == Union{Dates.DateTime, Nothing}
            @test getfield(sim.sequence, field) !== nothing
        end
    end

    @test length(findall(x -> x == 2, sequence.execution_order)) == 24
    @test length(findall(x -> x == 1, sequence.execution_order)) == 1

    state = PSI.get_simulation_state(sim)

    uc_vars = [OnVariable, StartVariable, StopVariable]
    ed_vars = [ActivePowerVariable]
    for (key, data) in state.decision_states.variables
        if PSI.get_entry_type(key) ∈ uc_vars
            _, count = size(data.values)
            @test count == 24
        elseif PSI.get_entry_type(key) ∈ ed_vars
            _, count = size(data.values)
            @test count == 288
        end
    end
end

@testset "Simulation with provided initial time" begin
    models = create_simulation_build_test_problems(get_template_basic_uc_simulation())
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
    second_day = DateTime("1/1/2024  23:00:00", "d/m/y  H:M:S") + Hour(1)
    sim = Simulation(;
        name = "test",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
        initial_time = second_day,
    )
    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT

    for model in PSI.get_decision_models(PSI.get_models(sim))
        @test PSI.get_initial_time(model) == second_day
    end

    for field in fieldnames(SimulationSequence)
        if fieldtype(SimulationSequence, field) == Union{Dates.DateTime, Nothing}
            @test getfield(sim.sequence, field) !== nothing
        end
    end

    @test length(findall(x -> x == 2, sequence.execution_order)) == 24
    @test length(findall(x -> x == 1, sequence.execution_order)) == 1
end

@testset "Negative Tests (Bad Parametrization)" begin
    models = create_simulation_build_test_problems(get_template_basic_uc_simulation())
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

    @test_throws UndefKeywordError sim = Simulation(; name = "test", steps = 1)

    sim = Simulation(;
        name = "test",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
        initial_time = Dates.now(),
    )

    @test_throws IS.ConflictingInputsError build!(
        sim,
        console_level = Logging.AboveMaxLevel,
    )

    sim = Simulation(;
        name = "fake_path",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = "fake_path",
    )

    @test_throws IS.ConflictingInputsError PSI._check_folder(sim)
end

@testset "Test SemiContinuous Feedforward with Active and Reactive Power variables" begin
    template_uc = get_template_basic_uc_simulation()
    set_device_model!(template_uc, Line, StaticBranchUnbounded)
    set_network_model!(template_uc, NetworkModel(DCPPowerModel; use_slacks = true))
    # network slacks added because of data issues
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(ACPPowerModel; use_slacks = true))
    set_device_model!(template_ed, Line, StaticBranchUnbounded)
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
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
                optimizer = ipopt_optimizer,
                initialize_model = false,
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
                    affected_values = [ActivePowerVariable, ReactivePowerVariable],
                ),
            ],
        ),
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = "reactive_feedforward",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    ac_power_model = PSI.get_simulation_model(PSI.get_models(sim), :ED)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ac_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ActivePowerVariable_ub",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ac_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ActivePowerVariable_lb",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ac_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ReactivePowerVariable_ub",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ac_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ReactivePowerVariable_lb",
    )
    @test !isempty(c)
end

@testset "Test Upper/Lower Bound Feedforwards" begin
    template_uc = get_template_basic_uc_simulation()
    set_network_model!(template_uc, NetworkModel(PTDFPowerModel; use_slacks = true))
    set_device_model!(template_uc, DeviceModel(Line, StaticBranchBounds))
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(PTDFPowerModel; use_slacks = true))
    set_device_model!(template_ed, DeviceModel(Line, StaticBranchBounds))
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
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
                optimizer = ipopt_optimizer,
                initialize_model = false,
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
                LowerBoundFeedforward(;
                    component_type = Line,
                    source = FlowActivePowerVariable,
                    affected_values = [FlowActivePowerVariable],
                    add_slacks = true,
                ),
                UpperBoundFeedforward(;
                    component_type = Line,
                    source = FlowActivePowerVariable,
                    affected_values = [FlowActivePowerVariable],
                    add_slacks = true,
                ),
            ],
        ),
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = "reactive_feedforward",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    ed_power_model = PSI.get_simulation_model(PSI.get_models(sim), :ED)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ed_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ActivePowerVariable_ub",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ed_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ActivePowerVariable_lb",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ed_power_model),
        FeedforwardLowerBoundConstraint(),
        Line,
        "FlowActivePowerVariablelb",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ed_power_model),
        FeedforwardUpperBoundConstraint(),
        Line,
        "FlowActivePowerVariableub",
    )
    @test !isempty(c)
    c = PSI.get_variable(
        PSI.get_optimization_container(ed_power_model),
        UpperBoundFeedForwardSlack(),
        Line,
        "FlowActivePowerVariable",
    )
    @test !isempty(c)
    c = PSI.get_variable(
        PSI.get_optimization_container(ed_power_model),
        LowerBoundFeedForwardSlack(),
        Line,
        "FlowActivePowerVariable",
    )
    @test !isempty(c)
end

@testset "Test FixValue Feedforwards" begin
    template_uc = get_template_basic_uc_simulation()
    set_network_model!(template_uc, NetworkModel(PTDFPowerModel; use_slacks = true))
    set_device_model!(template_uc, DeviceModel(Line, StaticBranchUnbounded))
    set_service_model!(template_uc, ServiceModel(VariableReserve{ReserveUp}, RangeReserve))
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(PTDFPowerModel; use_slacks = true))
    set_device_model!(template_ed, DeviceModel(Line, StaticBranchUnbounded))
    set_service_model!(template_ed, ServiceModel(VariableReserve{ReserveUp}, RangeReserve))
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
                optimizer = ipopt_optimizer,
                initialize_model = false,
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
                FixValueFeedforward(;
                    component_type = VariableReserve{ReserveUp},
                    source = ActivePowerReserveVariable,
                    affected_values = [ActivePowerReserveVariable],
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
    ed_power_model = PSI.get_simulation_model(PSI.get_models(sim), :ED)
    c = PSI.get_parameter(
        PSI.get_optimization_container(ed_power_model),
        FixValueParameter(),
        VariableReserve{ReserveUp},
        "Reserve1",
    )
    @test !isempty(c.multiplier_array)
    @test !isempty(c.parameter_array)
    c = PSI.get_parameter(
        PSI.get_optimization_container(ed_power_model),
        FixValueParameter(),
        VariableReserve{ReserveUp},
        "Reserve11",
    )
    @test !isempty(c.multiplier_array)
    @test !isempty(c.parameter_array)
end

@testset "Build with store_systems_in_results option" begin
    models = create_simulation_build_test_problems(get_template_basic_uc_simulation())
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

    # Test store_systems_in_results = true (default)
    sim_with = Simulation(;
        name = "test_with_systems",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim_with; store_systems_in_results = true)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    PSI.open_store(PSI.HdfSimulationStore, PSI.get_store_dir(sim_with), "r") do store
        root = store.file["simulation"]
        @test haskey(root, "systems")
        @test length(keys(root["systems"])) > 0
    end

    # Test store_systems_in_results = false
    models2 = create_simulation_build_test_problems(get_template_basic_uc_simulation())
    sequence2 = SimulationSequence(;
        models = models2,
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
    sim_without = Simulation(;
        name = "test_without_systems",
        steps = 1,
        models = models2,
        sequence = sequence2,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim_without; store_systems_in_results = false)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    PSI.open_store(PSI.HdfSimulationStore, PSI.get_store_dir(sim_without), "r") do store
        root = store.file["simulation"]
        @test !haskey(root, "systems")
    end
end
