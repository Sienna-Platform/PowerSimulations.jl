# A standalone `EmulationModel` run never updates parameters or initial conditions between
# steps, so it produces no recorder events regardless of which recorders are registered.
# This test exercises the recorder-event machinery through a `Simulation` instead, since PSI
# genuinely records both event types on that path.
@testset "Show recorder events in a Simulation" begin
    template_uc = template_unit_commitment(; network = CopperPlateNetworkModel)
    template_ed = template_economic_dispatch(; network = CopperPlateNetworkModel)
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
    events = list_recorder_events(PSI.InitialConditionUpdateEvent, recorder_log)
    @test !isempty(events)
    for wall_time in (true, false)
        show_recorder_events(
            devnull,
            PSI.InitialConditionUpdateEvent,
            recorder_log;
            wall_time = wall_time,
        )
    end
end
