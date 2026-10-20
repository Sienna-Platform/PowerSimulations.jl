# A standalone `EmulationModel` run never updates parameters or initial conditions between
# steps, so it produces no recorder events regardless of which recorders are registered.
# This test exercises the recorder-event machinery through a `Simulation` instead, since PSI
# genuinely records both event types on that path.
@testset "Show recorder events in a Simulation" begin
    template_uc = test_template_unit_commitment(CopperPlateNetworkModel)
    template_ed = test_template_economic_dispatch(CopperPlateNetworkModel)
    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    c_sys5_ed = PSB.build_system(PSITestSystems, "c_sys5_ed")

    models = SimulationModels(;
        decision_models = [
            DecisionModel(template_uc, c_sys5_uc; name = "UC", optimizer = HiGHS_optimizer),
            DecisionModel(template_ed, c_sys5_ed; name = "ED", optimizer = HiGHS_optimizer),
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
        name = "recorder_events",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )

    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; enable_progress_bar = false) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    recorder_log = joinpath(PSI.get_recorder_folder(sim), "execution.log")
    events = list_recorder_events(PSI.ParameterUpdateEvent, recorder_log)
    @test !isempty(events)
    ic_events = list_recorder_events(PSI.InitialConditionUpdateEvent, recorder_log)
    @test !isempty(ic_events)
    for event in ic_events
        @test !isempty(event.device_names)
        @test length(event.new_values) == length(event.device_names)
        @test length(event.previous_values) == length(event.device_names)
    end
    # One event per initial-condition key per model execution.
    ic_keys = [
        (x.model_name, x.simulation_time, x.initial_condition_type, x.component_type)
        for
        x in ic_events
    ]
    @test allunique(ic_keys)

    sim_dir = PSI.get_simulation_dir(sim)
    status_events = PSI.list_simulation_events(PSI.ProblemExecutionEvent, sim_dir)
    done_events = filter(x -> x.status == "done", status_events)
    @test length(done_events) == count(x -> x.status == "start", status_events)
    for event in done_events
        @test event.termination_status == :OPTIMAL
        @test isfinite(event.objective_value)
        @test event.solve_time_s >= 0.0
        @test event.exec_time_s >= event.solve_time_s
    end

    # The step filter splits each model's events across the steps with no overlap.
    for T in (PSI.ParameterUpdateEvent, PSI.InitialConditionUpdateEvent),
        model_name in ("UC", "ED")

        model_events = PSI.list_simulation_events(T, sim_dir; model_name = model_name)
        @test all(x -> x.model_name == Symbol(model_name), model_events)
        per_step = [
            PSI.list_simulation_events(T, sim_dir; step = step, model_name = model_name)
            for step in 1:2
        ]
        @test sum(length, per_step) == length(model_events)
    end
    @test isempty(
        PSI.list_simulation_events(
            PSI.InitialConditionUpdateEvent,
            sim_dir;
            step = 1,
            model_name = "UC",
        ),
    )
    @test !isempty(
        PSI.list_simulation_events(
            PSI.InitialConditionUpdateEvent,
            sim_dir;
            step = 2,
            model_name = "UC",
        ),
    )

    buf = IOBuffer()
    io = IOContext(buf, :displaysize => (100, 1000))
    show_recorder_events(
        io,
        PSI.InitialConditionUpdateEvent,
        recorder_log;
        wall_time = true,
    )
    text = String(take!(buf))
    @test occursin("timestamp", text)
    @test occursin("device_name", text)
    show_recorder_events(io, PSI.InitialConditionUpdateEvent, recorder_log)
    @test !occursin("timestamp", String(take!(buf)))
    show_recorder_events(io, PSI.ParameterUpdateEvent, recorder_log)
    @test !occursin("timestamp", String(take!(buf)))
end

@testset "A failed model execution writes a failed status line" begin
    template_uc = test_template_unit_commitment(CopperPlateNetworkModel)
    c_sys5_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    optimizer = optimizer_with_attributes(
        HiGHS.Optimizer,
        "time_limit" => 0.0,
        "output_flag" => false,
    )
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                c_sys5_uc;
                name = "UC",
                optimizer = optimizer,
                initialize_model = false,
            ),
        ],
    )
    sim = Simulation(;
        name = "failed_status",
        steps = 1,
        models = models,
        sequence = SimulationSequence(; models = models),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; enable_progress_bar = false) == PSI.RunStatus.FAILED

    events = PSI.list_simulation_events(
        PSI.ProblemExecutionEvent,
        PSI.get_simulation_dir(sim),
    )
    @test [x.status for x in events] == ["start", "failed"]
    @test events[1].termination_status == :OPTIMIZE_NOT_CALLED
    @test events[2].termination_status == :TIME_LIMIT
    @test isnan(events[2].objective_value)
    @test events[2].exec_time_s >= 0.0
end
