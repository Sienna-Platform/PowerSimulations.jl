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

@testset "Test Simulation Print Methods" begin
    template_uc = get_template_basic_uc_simulation()
    template_ed = get_template_nomin_ed_simulation()
    set_device_model!(template_ed, InterruptiblePowerLoad, StaticPowerLoad)
    set_network_model!(template_uc, NetworkModel(
        CopperPlateNetworkModel,
        # MILP "duals" not supported with free solvers
        # duals = [CopperPlateBalanceConstraint],
    ))
    set_network_model!(
        template_ed,
        NetworkModel(
            CopperPlateNetworkModel;
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
