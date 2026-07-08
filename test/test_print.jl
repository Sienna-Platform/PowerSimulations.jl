function _test_plain_print_methods(list::Array)
    for object in list
        normal = repr(object)
        io = IOBuffer()
        show(io, "text/plain", object)
        grabbed = String(take!(io))
        @test !isnothing(grabbed)
    end
end

function _test_html_print_methods(list::Array)
    for object in list
        normal = repr(object)
        io = IOBuffer()
        show(io, "text/html", object)
        grabbed = String(take!(io))
        @test !isnothing(grabbed)
    end
end

@testset "Test Model Print Methods" begin
    template = get_thermal_dispatch_template_network()
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5")

    dm_model = DecisionModel(template, c_sys5; optimizer = HiGHS_optimizer)
    @test build!(dm_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(dm_model; optimizer = HiGHS_optimizer) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
    results = OptimizationProblemResults(dm_model)
    variables = read_variables(results)

    list = [
        template,
        dm_model,
        PSI.get_model(template, ThermalStandard),
        PSI.get_network_model(template),
        results,
    ]

    _test_plain_print_methods(list)
    _test_html_print_methods(list)

    # Regression: showing results with a single time period used to throw
    # `MethodError: no method matching Minute(::Nothing)` because the
    # resolution is `nothing` when there is only one timestamp.
    dm_model_single = DecisionModel(
        template,
        c_sys5;
        optimizer = HiGHS_optimizer,
        horizon = Dates.Hour(1),
    )
    @test build!(dm_model_single; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(dm_model_single; optimizer = HiGHS_optimizer) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
    results_single = OptimizationProblemResults(dm_model_single)
    @test length(get_timestamps(results_single)) == 1
    output = sprint(show, "text/plain", results_single)
    @test occursin("Resolution: N/A (single period)", output)
end

@testset "Test Simulation Print Methods" begin
    template_uc = get_template_basic_uc_simulation()
    template_ed = get_template_nomin_ed_simulation()
    set_device_model!(template_ed, InterruptiblePowerLoad, StaticPowerLoad)
    set_network_model!(template_uc, NetworkModel(
        CopperPlatePowerModel,
        # MILP "duals" not supported with free solvers
        # duals = [CopperPlateBalanceConstraint],
    ))
    set_network_model!(
        template_ed,
        NetworkModel(
            CopperPlatePowerModel;
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

    sim_not_built = Simulation(;
        name = "printing_sim",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )

    sim = Simulation(;
        name = "printing_sim",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )

    build!(sim)
    execute!(sim)
    results = SimulationResults(sim)
    results_uc = get_decision_problem_results(results, "UC")
    list = [models, sequence, template_uc, template_ed, sim, sim_not_built, results_uc]
    _test_plain_print_methods(list)
    _test_html_print_methods(list)
end
