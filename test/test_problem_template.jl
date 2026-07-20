# This file is WIP while the interface for templates is finalized
@testset "Manual Operations Template" begin
    template = ProblemTemplate(CopperPlatePowerModel)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
    set_device_model!(template, Line, StaticBranchUnbounded)
    @test !isempty(template.devices)
    @test !isempty(template.branches)
    @test isempty(template.services)
end

@testset "Operations Template Overwrite" begin
    template = ProblemTemplate(CopperPlatePowerModel)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
    @test_logs (:warn, "Overwriting ThermalStandard existing model") set_device_model!(
        template,
        DeviceModel(ThermalStandard, ThermalBasicUnitCommitment),
    )
    @test PSI.get_formulation(template.devices[:ThermalStandard]) ==
          ThermalBasicUnitCommitment
end

@testset "Provided Templates Tests" begin
    uc_template = template_unit_commitment()
    @test !isempty(uc_template.devices)
    @test PSI.get_formulation(uc_template.devices[:ThermalStandard]) ==
          ThermalBasicUnitCommitment
    uc_template = template_unit_commitment(; network = DCPPowerModel)
    @test get_network_formulation(uc_template) == DCPPowerModel
    @test !isempty(uc_template.branches)
    @test !isempty(uc_template.services)

    ed_template = template_economic_dispatch()
    @test !isempty(ed_template.devices)
    @test PSI.get_formulation(ed_template.devices[:ThermalStandard]) == ThermalBasicDispatch
    ed_template = template_economic_dispatch(; network = ACPPowerModel)
    @test get_network_formulation(ed_template) == ACPPowerModel
    @test !isempty(ed_template.branches)
    @test !isempty(ed_template.services)
end

@testset "Template model keys are namespace-stable (issue #1615)" begin
    template = ProblemTemplate()
    set_device_model!(template, MonitoredLine, StaticBranchBounds)
    set_device_model!(template, Line, StaticBranch)

    # Branch keys must be the bare, module-qualifier-free symbols that the build-time
    # network lookup (`nameof(branch_type)` in `instantiate_network_model!`) uses.
    # `Symbol(D)` qualifies the module path (e.g. `Symbol("PowerSystems.MonitoredLine")`)
    # in some namespace contexts, which produced `KeyError(:MonitoredLine)` during build!.
    branch_keys = keys(PSI.get_branch_models(template))
    @test Set(branch_keys) == Set([nameof(MonitoredLine), nameof(Line)])
    @test all(k -> !occursin(".", string(k)), branch_keys)
    @test PSI.get_model(template, MonitoredLine) isa PSI.DeviceModel
    @test PSI.get_model(template, Line) isa PSI.DeviceModel

    # Core invariant that prevents the KeyError regardless of namespace visibility:
    # the key the setter stores equals the one the network model looks up.
    for D in (MonitoredLine, Line)
        @test Symbol(IS.strip_module_name(D)) == nameof(D)
    end
end

@testset "_copy_template_for_build preserves the caller's template across build!" begin
    # `_copy_template_for_build` shares PTDF/MODF matrices by reference (their
    # libklu/libSparse factorizations are unsafe to deepcopy) and deepcopies the
    # device/branch/service dicts so `validate_template_impl!` and
    # `_build_device_model_outages!` mutate the build copy without leaking back.
    # The invariants below catch a future regression in either half: a missing
    # `finally` restore would null the caller's matrices, and dropping the
    # device-dict deepcopy would leak auto-discovered outages onto the caller.
    sys = PSB.build_system(PSITestSystems, "c_sys5")
    lines = collect(PSY.get_components(PSY.Line, sys))
    @assert length(lines) >= 2
    outage = PSY.GeometricDistributionForcedOutage(;
        mean_time_to_recovery = 10,
        outage_transition_probability = 0.9999,
        monitored_components = [lines[2]],
    )
    PSY.add_supplemental_attribute!(sys, lines[1], outage)

    ptdf = PNM.PTDF(sys)
    modf = PNM.VirtualMODF(sys)
    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = ptdf, MODF_matrix = modf),
    )
    set_device_model!(template, DeviceModel(PSY.Line, SecurityConstrainedStaticBranch))

    caller_network = PSI.get_network_model(template)
    caller_line_model = PSI.get_model(template, PSY.Line)
    caller_device_keys = Set(keys(PSI.get_device_models(template)))
    caller_branch_keys = Set(keys(PSI.get_branch_models(template)))
    @assert isempty(PSI.get_outages(caller_line_model))

    model = DecisionModel(template, sys)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    # Caller's matrices survived the try/finally restore (identity, not equality).
    @test PSI.get_PTDF_matrix(caller_network) === ptdf
    @test PSI.get_MODF_matrix(caller_network) === modf
    # Caller's device/branch dicts were not mutated by validate_template_impl!.
    @test Set(keys(PSI.get_device_models(template))) == caller_device_keys
    @test Set(keys(PSI.get_branch_models(template))) == caller_branch_keys
    # Auto-discovered outages stayed on the build copy and did not leak onto the
    # caller's DeviceModel.
    @test isempty(PSI.get_outages(caller_line_model))
    built_line_model = PSI.get_model(PSI.get_template(model), PSY.Line)
    @test !isempty(PSI.get_outages(built_line_model))

    # Build copy aliases the matrices rather than deepcopying their solver caches.
    built_network = PSI.get_network_model(PSI.get_template(model))
    @test PSI.get_PTDF_matrix(built_network) === ptdf
    @test PSI.get_MODF_matrix(built_network) === modf

    # Re-building from the same template must succeed — catches a broken
    # `finally` that nulls the caller's matrices on the first pass.
    model2 = DecisionModel(template, sys)
    @test build!(model2; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
end

@testset "modeled_ac_branch_types is rebuilt per validation pass (issue #1621)" begin
    # `validate_template_impl!` populates `network_model.modeled_ac_branch_types`
    # from the branch types whose device cache is non-empty. Re-building the same
    # model after a branch type is pruned (here HVDC, made unavailable) must NOT
    # leave a stale entry in that vector: the PTDF `instantiate_network_model!`
    # path indexes `branch_models[nameof(branch_type)]` for every modeled type,
    # so a stale `:TwoTerminalGenericHVDCLine` raised `KeyError` on the rebuild.
    sys = PSB.build_system(PSITestSystems, "c_sys14_dc"; add_forecasts = true)
    ptdf = PNM.PTDF(sys)

    template = ProblemTemplate(
        NetworkModel(PTDFPowerModel; use_slacks = false, PTDF_matrix = ptdf),
    )
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, Line, StaticBranch)
    set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)

    out = mktempdir(; cleanup = true)
    model = DecisionModel(
        template,
        sys;
        name = "issue_1621",
        horizon = Hour(1),
        initial_time = DateTime(2024, 1, 1, 0),
    )

    # First build models the HVDC line, so its type is recorded.
    @test build!(model; output_dir = out) == PSI.ModelBuildStatus.BUILT
    built_network = PSI.get_network_model(PSI.get_template(model))
    @test TwoTerminalGenericHVDCLine in built_network.modeled_ac_branch_types

    # Prune the HVDC: its device cache becomes empty on the next validation pass,
    # so it is deleted from the template's branches.
    for hvdc in PSY.get_components(TwoTerminalGenericHVDCLine, sys)
        PSY.set_available!(hvdc, false)
    end

    # Rebuilding the same model must succeed (previously KeyError), and the stale
    # HVDC entry must be gone from the rebuilt vector.
    @test build!(model; output_dir = out) == PSI.ModelBuildStatus.BUILT
    rebuilt_network = PSI.get_network_model(PSI.get_template(model))
    @test TwoTerminalGenericHVDCLine ∉ rebuilt_network.modeled_ac_branch_types
end

@testset "Device model with empty device cache is skipped on build" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc_re")
    template = get_thermal_dispatch_template_network()
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    for d in PSY.get_components(RenewableDispatch, sys)
        PSY.set_available!(d, false)
    end
    model = DecisionModel(template, sys)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
end