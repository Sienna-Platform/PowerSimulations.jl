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
        power_flow_evaluation = ACPolarPowerFlow(;
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

@testset "Branch validation scoped to modeled networks" begin
    # A >1% endpoint voltage mismatch makes the line fail PSY.check_component.
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
    arc = get_arc(first(get_components(Line, sys)))
    set_base_voltage!(get_to(arc), 10 * get_base_voltage(get_from(arc)))

    # CopperPlate does not model branches, so the invalid line is not validated
    # and template validation succeeds.
    cp_model =
        DecisionModel(get_thermal_dispatch_template_network(CopperPlatePowerModel), sys)
    @test PSI.validate_template(cp_model) === nothing

    # PTDF models branches, so the same invalid line must raise during validation.
    # The deliberate failure logs an @error before throwing; silence it with a
    # NullLogger so it does not trip the suite-wide "no @error logged" assertion
    # in runtests.jl (build! failures avoid this by logging under their own logger).
    ptdf_model =
        DecisionModel(get_thermal_dispatch_template_network(PTDFPowerModel), sys)
    with_logger(NullLogger()) do
        @test_throws IS.InvalidValue PSI.validate_template(ptdf_model)
    end
end

@testset "Settings export_optimization_model format Tests" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5")
    # Default is no export.
    @test PSI.get_export_optimization_model(PSI.Settings(sys)) ==
          PSI.OptimizationModelExportFormat.NONE
    # The enum is accepted directly.
    @test PSI.get_export_optimization_model(
        PSI.Settings(sys; export_optimization_model = PSI.OptimizationModelExportFormat.LP),
    ) == PSI.OptimizationModelExportFormat.LP
    # A case-insensitive, trimmed string naming a value is also accepted.
    @test PSI.get_export_optimization_model(
        PSI.Settings(sys; export_optimization_model = "mof"),
    ) == PSI.OptimizationModelExportFormat.MOF
    @test PSI.get_export_optimization_model(
        PSI.Settings(sys; export_optimization_model = " lp "),
    ) == PSI.OptimizationModelExportFormat.LP
    @test PSI.get_export_optimization_model(
        PSI.Settings(sys; export_optimization_model = ""),
    ) == PSI.OptimizationModelExportFormat.NONE
    # Invalid string and the legacy Bool both raise a clear error.
    @test_throws IS.ConflictingInputsError PSI.Settings(
        sys;
        export_optimization_model = "json",
    )
    @test_throws IS.ConflictingInputsError PSI.Settings(
        sys;
        export_optimization_model = true,
    )
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
