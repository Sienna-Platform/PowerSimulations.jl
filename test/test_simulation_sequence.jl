@testset "Simulation Sequence Correct Execution Order" begin
    models_array = [
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(48),
            interval = Hour(24),
            resolution = Hour(1),
            steps = 2,
            name = "DAUC",
        ),
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(24),
            resolution = Minute(5),
            interval = Hour(1),
            steps = 2 * 24,
            name = "HAUC",
        ),
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(12),
            resolution = Minute(5),
            interval = Minute(5),
            steps = 2 * 24 * 12,
            name = "ED",
        ),
    ]
    set_device_model!(
        PSI.get_template(models_array[3]),
        ThermalStandard,
        ThermalBasicDispatch,
    )
    models = SimulationModels(
        models_array,
        EmulationModel(
            MockEmulationProblem;
            interval = Minute(1),
            resolution = Minute(1),
            name = "AGC",
        ),
    )

    test_sequence = SimulationSequence(;
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

    @test !isempty(
        PSI.get_model(PSI.get_template(models_array[3]), ThermalStandard).feedforwards,
    )

    @test length(findall(x -> x == 4, test_sequence.execution_order)) == 24 * 60
    @test length(findall(x -> x == 3, test_sequence.execution_order)) == 24 * 12
    @test length(findall(x -> x == 2, test_sequence.execution_order)) == 24
    @test length(findall(x -> x == 1, test_sequence.execution_order)) == 1

    for model in PSI.get_decision_models(models)
        @test PSI.get_sequence_uuid(model) == test_sequence.uuid
    end

    # Test single stage sequence
    test_sequence = SimulationSequence(;
        models = SimulationModels(
        # TODO: support passing one model without making a vector
            [DecisionModel(MockOperationProblem; horizon = Hour(48), name = "DAUC")]),
        ini_cond_chronology = InterProblemChronology(),
    )

    # Disabled temporarily
    # @test isa(test_sequence.ini_cond_chronology, IntraProblemChronology)
    @test test_sequence.execution_order == [1]
end

@testset "Simulation Sequence invalid sequences" begin
    models = SimulationModels([
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(48),
            interval = Hour(24),
            resolution = Hour(1),
            steps = 2,
            name = "DAUC",
        ),
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(24),
            interval = Hour(5),
            resolution = Minute(5),
            steps = 2 * 24,
            name = "HAUC",
        ),
    ])

    @test_throws IS.ConflictingInputsError SimulationSequence(models = models)

    models = SimulationModels([
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(2),
            interval = Hour(1),
            resolution = Hour(1),
            steps = 2,
            name = "DAUC",
        ),
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(24),
            interval = Hour(1),
            resolution = Minute(5),
            steps = 2 * 24,
            name = "HAUC",
        ),
    ])

    @test_throws IS.ConflictingInputsError SimulationSequence(models = models)

    models = SimulationModels([
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(24),
            interval = Hour(1),
            resolution = Hour(1),
            steps = 2,
            name = "DAUC",
        ),
        DecisionModel(
            MockOperationProblem;
            horizon = Hour(24),
            interval = Minute(22),
            resolution = Hour(1),
            steps = 2 * 24,
            name = "HAUC",
        ),
    ])

    @test_throws IS.ConflictingInputsError SimulationSequence(models = models)
end

@testset "SimulationSequence records whether models may differ in components" begin
    make_models() = SimulationModels([
        DecisionModel(MockOperationProblem; horizon = Hour(48), name = "DAUC"),
    ])
    default_sequence = SimulationSequence(; models = make_models())
    @test !PSI.get_allow_different_components(default_sequence)

    sequence = SimulationSequence(;
        models = make_models(),
        allow_different_components = true,
    )
    @test PSI.get_allow_different_components(sequence)

    text = sprint(show, MIME"text/plain"(), sequence)
    @test occursin("Allow Different Components", text)
    @test occursin("true", text)
end
