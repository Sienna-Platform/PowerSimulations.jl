@testset "DeviceModel Tests" begin
    @test_throws ArgumentError DeviceModel(ThermalGen, ThermalStandardUnitCommitment)
    @test_throws ArgumentError DeviceModel(ThermalStandard, PSI.AbstractThermalFormulation)
    @test_throws ArgumentError NetworkModel(PM.AbstractPowerModel)
end

@testset "NetworkModel Tests" begin
    @test_throws ArgumentError NetworkModel(PM.AbstractPowerModel)
    @test NetworkModel(
        PTDFPowerModel;
        use_slacks = true,
        power_flow_evaluation = [DCPowerFlow(), PSSEExportPowerFlow(:v33, "exports")],
    ) isa NetworkModel
    @test NetworkModel(
        PTDFPowerModel;
        use_slacks = true,
        power_flow_evaluation = ACPowerFlow(;
            exporter =
            PSSEExportPowerFlow(
                :v33,
                "exports";
                name = "my_export_name",
                write_comments = true,
                overwrite = true,
            ),
        ),
    ) isa NetworkModel
end

@testset "validate_template dispatch Tests" begin
    struct CustomDecisionProblem <: PSI.DecisionProblem end
    struct CustomEmulationProblem <: PSI.EmulationProblem end

    sys = PSB.build_system(PSITestSystems, "c_sys5")
    template = ProblemTemplate(CopperPlatePowerModel)

    # DecisionModel has no inner constructor, so use the default field constructor
    decision_model = DecisionModel{CustomDecisionProblem}(
        :test,
        template,
        sys,
        nothing,
        PSI.SimulationInfo(),
        PSI.DecisionModelStore(),
        Dict{String, Any}(),
    )
    @test_throws ErrorException PSI.validate_template(decision_model)

    # EmulationModel has an inner constructor; build with settings then test
    settings = PSI.Settings(sys)
    emulation_model = EmulationModel{CustomEmulationProblem}(
        deepcopy(template),
        sys,
        settings,
        nothing,
    )
    @test_throws ErrorException PSI.validate_template(emulation_model)
end

@testset "Feedforward Struct Tests" begin
    ffs = [
        UpperBoundFeedforward(;
            component_type = RenewableDispatch,
            source = ActivePowerVariable,
            affected_values = [ActivePowerVariable],
            add_slacks = true,
        ),
        LowerBoundFeedforward(;
            component_type = RenewableDispatch,
            source = ActivePowerVariable,
            affected_values = [ActivePowerVariable],
            add_slacks = true,
        ),
        SemiContinuousFeedforward(;
            component_type = ThermalMultiStart,
            source = OnVariable,
            affected_values = [ActivePowerVariable, ReactivePowerVariable],
        ),
    ]

    for ff in ffs
        for av in PSI.get_affected_values(ff)
            @test isa(av, PSI.VariableKey)
        end
    end

    ff = FixValueFeedforward(;
        component_type = HydroDispatch,
        source = OnVariable,
        affected_values = [OnStatusParameter],
    )

    for av in PSI.get_affected_values(ff)
        @test isa(av, PSI.ParameterKey)
    end

    @test_throws ErrorException UpperBoundFeedforward(
        component_type = RenewableDispatch,
        source = ActivePowerVariable,
        affected_values = [OnStatusParameter],
        add_slacks = true,
    )

    @test_throws ErrorException LowerBoundFeedforward(
        component_type = RenewableDispatch,
        source = ActivePowerVariable,
        affected_values = [OnStatusParameter],
        add_slacks = true,
    )

    @test_throws ErrorException SemiContinuousFeedforward(
        component_type = ThermalMultiStart,
        source = OnVariable,
        affected_values = [ActivePowerVariable, OnStatusParameter],
    )
end
