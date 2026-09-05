function run_simulation(
    c_sys5_hy_uc,
    c_sys5_hy_ed,
    file_path::String,
    export_path;
    in_memory = false,
    uc_network_model = nothing,
    ed_network_model = nothing,
    store_systems_in_results = true,
)
    template_uc = get_template_basic_uc_simulation()
    template_ed = get_template_nomin_ed_simulation()
    isnothing(uc_network_model) && (
        uc_network_model =
            NetworkModel(CopperPlateNetworkModel; duals = [CopperPlateBalanceConstraint])
    )
    isnothing(ed_network_model) && (
        ed_network_model =
            NetworkModel(
                CopperPlateNetworkModel;
                duals = [CopperPlateBalanceConstraint],
                use_slacks = true,
            )
    )
    set_device_model!(template_ed, InterruptiblePowerLoad, StaticPowerLoad)
    set_network_model!(
        template_uc,
        uc_network_model,
    )
    set_network_model!(
        template_ed,
        ed_network_model,
    )
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
    sim = Simulation(;
        name = "no_cache",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = file_path,
    )

    build_out = build!(
        sim;
        console_level = Logging.Error,
        store_systems_in_results = store_systems_in_results,
    )
    @test build_out == PSI.SimulationBuildStatus.BUILT

    exports = Dict(
        "models" => [
            Dict(
                "name" => "UC",
                "store_all_variables" => true,
                "store_all_parameters" => true,
                "store_all_duals" => true,
                "store_all_aux_variables" => true,
            ),
            Dict(
                "name" => "ED",
                "store_all_variables" => true,
                "store_all_parameters" => true,
                "store_all_duals" => true,
                "store_all_aux_variables" => true,
            ),
        ],
        "path" => export_path,
        "optimizer_stats" => true,
    )
    execute_out = execute!(sim; exports = exports, in_memory = in_memory)
    @test execute_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    return sim
end
