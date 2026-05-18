@testset "Test Simulation Models" begin
    models = SimulationModels(
        [
            DecisionModel(
                MockOperationProblem;
                horizon = Hour(48),
                interval = Hour(24),
                steps = 2,
                name = "DAUC",
            ),
            DecisionModel(
                MockOperationProblem;
                horizon = Hour(24),
                interval = Hour(1),
                steps = 2 * 24,
                name = "HAUC",
            ),
            DecisionModel(
                MockOperationProblem;
                horizon = Hour(12),
                interval = Minute(5),
                steps = 2 * 24 * 12,
                name = "ED",
            ),
        ],
        EmulationModel(MockEmulationProblem; resolution = Minute(1), name = "AGC"),
    )

    @test length(PSI.get_decision_models(models)) == 3
    @test !isnothing(PSI.get_emulation_model(models))

    @test_throws ErrorException SimulationModels(
        [
            DecisionModel(
                MockOperationProblem;
                horizon = Hour(48),
                interval = Hour(24),
                steps = 2,
                name = "DAUC",
            ),
            DecisionModel(
                MockOperationProblem;
                horizon = Hour(24),
                interval = Hour(1),
                steps = 2 * 24,
                name = "DAUC",
            ),
            DecisionModel(
                MockOperationProblem;
                horizon = Hour(12),
                interval = Minute(5),
                steps = 2 * 24 * 12,
                name = "ED",
            ),
        ],
        EmulationModel(MockEmulationProblem; resolution = Minute(1), name = "AGC"),
    )
end
