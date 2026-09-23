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
            @test !isnothing(getfield(sim.sequence, field))
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
            @test !isnothing(getfield(sim.sequence, field))
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
    set_network_model!(template_uc, NetworkModel(DCPNetworkModel; use_slacks = true))
    # network slacks added because of data issues
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(ACPNetworkModel; use_slacks = true))
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
                optimizer = HiGHS_optimizer,
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
    set_network_model!(template_uc, NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(template_uc, DeviceModel(Line, StaticBranchBounds))
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(PTDFNetworkModel; use_slacks = true))
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
                optimizer = HiGHS_optimizer,
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
        UpperBoundFeedForwardSlack,
        Line,
        "FlowActivePowerVariable",
    )
    @test !isempty(c)
    c = PSI.get_variable(
        PSI.get_optimization_container(ed_power_model),
        LowerBoundFeedForwardSlack,
        Line,
        "FlowActivePowerVariable",
    )
    @test !isempty(c)
end

# No FixValueFeedforward-on-ServiceModel test: POM's ServiceModel keys reserve variables by
# (service_name, device_name, time), while the feedforward parameter path is keyed
# (device_name, time), so `attach_feedforward!(::ServiceModel, ff)` errors loudly. Re-add
# once POM re-keys that path.

@testset "Build writes system bundles" begin
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

    sim_with = Simulation(;
        name = "test_with_systems",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim_with)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    for model in PSI.get_all_models(models)
        bundle =
            joinpath(IOM.get_output_dir(model), IOM.make_system_dirname(get_system(model)))
        @test isdir(bundle)
        @test !isempty(readdir(bundle))
    end
    PSI.open_store(PSI.HdfSimulationStore, PSI.get_store_dir(sim_with), "r") do store
        root = store.file["simulation"]
        @test !haskey(root, "systems")
    end
end

@testset "Parameters are written to the bundle's InfraStore as forecast windows" begin
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
    sim = run_simulation(
        c_sys5_hy_uc,
        c_sys5_hy_ed,
        mktempdir(; cleanup = true),
        mktempdir(; cleanup = true);
        in_memory = false,
    )
    folder = PSI.get_simulation_dir(sim)
    uc_dir = joinpath(folder, "problems", "UC")
    bundle = joinpath(uc_dir, only(filter(startswith("system-"), readdir(uc_dir))))
    sidecar = joinpath(bundle, PSY.TIME_SERIES_FILE)
    @test isfile(sidecar)

    store_dir = joinpath(folder, "data_store")
    num_executions, num_steps, horizon_count = PSI.open_store(
        PSI.HdfSimulationStore,
        store_dir,
        "r",
    ) do store
        params = PSI.get_decision_model_params(store, :UC)
        @test !haskey(
            store.file["simulation/decision_models/UC"],
            "parameters",
        )
        (
            IOM.get_num_executions(params),
            store.params.num_steps,
            IOM.get_horizon_count(params),
        )
    end
    expected_executions = num_executions * num_steps

    key = IOM.ParameterKey(POM.ActivePowerTimeSeriesParameter, PSY.PowerLoad)
    pstore = POM.open_parameter_store(sidecar)
    windows = POM.read_parameter_windows(
        pstore,
        key;
        extra_features = Dict{String, Any}("model" => "UC"),
    )
    POM.close_parameter_store!(pstore)
    @test !isempty(windows)
    label, per_time = first(windows)
    @test length(per_time) == expected_executions
    @test all(length(v) == horizon_count for v in values(per_time))
end
