# Note: this function is used in HydroPowerSimulations.jl and StorageSystemsSimulations.jl as well for testing of events
function run_fixed_forced_outage_sim_with_timeseries(;
    sys,
    networks,
    optimizers,
    outage_status_timeseries,
    device_type,
    device_names,
    renewable_formulation,
)
    sys_em = deepcopy(sys)
    sys_d1 = deepcopy(sys)
    sys_d2 = deepcopy(sys)
    transform_single_time_series!(sys_d1, Day(2), Day(1))
    transform_single_time_series!(sys_d2, Hour(4), Hour(1))
    event_model = EventModel(
        FixedForcedOutage,
        POM.ContinuousCondition();
        timeseries_mapping = Dict(
            :outage_status => "outage_profile_1",
        ),
    )
    template_d1 = get_template_basic_uc_simulation()
    set_network_model!(template_d1, NetworkModel(networks[1]))
    template_d2 = get_template_basic_uc_simulation()
    set_network_model!(template_d2, NetworkModel(networks[2]))
    template_em = get_template_nomin_ed_simulation(networks[3])

    set_device_model!(template_d1, RenewableDispatch, renewable_formulation)
    set_device_model!(template_d2, RenewableDispatch, renewable_formulation)
    set_device_model!(template_em, RenewableDispatch, renewable_formulation)
    set_device_model!(template_em, ThermalStandard, ThermalBasicDispatch)
    set_service_model!(template_d1, ServiceModel(OnlineReserve{ReserveUp}, RangeReserve))
    set_service_model!(template_d2, ServiceModel(OnlineReserve{ReserveUp}, RangeReserve))
    set_device_model!(template_em, InterruptiblePowerLoad, PowerLoadDispatch)
    set_device_model!(template_d1, InterruptiblePowerLoad, PowerLoadDispatch)
    set_device_model!(template_d2, InterruptiblePowerLoad, PowerLoadDispatch)
    set_device_model!(template_d1, Line, StaticBranch)
    set_device_model!(template_d2, Line, StaticBranch)
    set_device_model!(template_em, Line, StaticBranch)

    for sys in [sys_d1, sys_d2, sys_em]
        for name in device_names
            g = get_component(device_type, sys, name)
            transition_data = PSY.FixedForcedOutage(;
                outage_status = 0.0,
            )
            add_supplemental_attribute!(sys, g, transition_data)
            PSY.add_time_series!(
                sys,
                transition_data,
                PSY.SingleTimeSeries("outage_profile_1", outage_status_timeseries),
            )
        end
    end

    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_d1,
                sys_d1;
                name = "D1",
                initialize_model = false,
                optimizer = optimizers[1],
            ),
            DecisionModel(
                template_d2,
                sys_d2;
                name = "D2",
                initialize_model = false,
                optimizer = optimizers[2],
                store_variable_names = true,
            ),
        ],
        emulation_model = EmulationModel(
            template_em,
            sys_em;
            name = "EM",
            optimizer = optimizers[3],
            calculate_conflict = true,
            store_variable_names = true,
        ),
    )
    sequence = SimulationSequence(;
        models = models,
        ini_cond_chronology = InterProblemChronology(),
        feedforwards = Dict(
            "EM" => [# This FeedForward will force the commitment to be kept in the emulator
                SemiContinuousFeedforward(;
                    component_type = ThermalStandard,
                    source = OnVariable,
                    affected_values = [ActivePowerVariable],
                ),
            ],
        ),
        events = [event_model],
    )

    sim = Simulation(;
        name = "no_cache",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim; console_level = Logging.Error)
    @test build_out == SimulationBuildStatus.BUILT
    execute_out = execute!(sim; in_memory = true)
    @test execute_out == IOM.RunStatus.SUCCESSFULLY_FINALIZED
    results = SimulationResults(sim; ignore_status = true)
    return results
end

function run_events_simulation(;
    sys_emulator,
    networks,
    optimizers,
    outage_time,   #DateTime
    outage_length, #hrs
    uc_formulation,   #
    ed_formulation,
    feedforward,
    in_memory,
)
    sys_em = deepcopy(sys_emulator)
    sys_d1 = build_system(PSITestSystems, "c_sys5_events")
    transform_single_time_series!(sys_d1, Day(2), Day(1))
    sys_d2 = build_system(PSITestSystems, "c_sys5_events")
    transform_single_time_series!(sys_d2, Hour(4), Hour(1))

    event_model = EventModel(
        GeometricDistributionForcedOutage,
        POM.PresetTimeCondition([outage_time]),
    )
    if uc_formulation == "basic"
        template_d1 = get_template_basic_uc_simulation()
        set_network_model!(template_d1, NetworkModel(networks[1]))
        template_d2 = get_template_basic_uc_simulation()
        set_network_model!(template_d2, NetworkModel(networks[2]))
    elseif uc_formulation == "standard"
        template_d1 = get_template_standard_uc_simulation()
        set_network_model!(template_d1, NetworkModel(networks[1]))
        template_d2 = get_template_standard_uc_simulation()
        set_network_model!(template_d2, NetworkModel(networks[2]))
    else
        @error "invalid uc formulation: $(uc_formulation). Must be basic or standard"
    end
    template_em = get_template_nomin_ed_simulation(networks[3])
    if ed_formulation == "basic"
        set_device_model!(template_em, ThermalStandard, ThermalBasicDispatch)
    elseif ed_formulation == "nomin"
    else
        @error "invalid ed formulation: $(ed). Must be basic or nomin"
    end
    set_device_model!(template_d1, Line, StaticBranch)
    set_device_model!(template_d2, Line, StaticBranch)
    set_device_model!(template_em, Line, StaticBranch)

    set_service_model!(template_d1, ServiceModel(OnlineReserve{ReserveUp}, RangeReserve))
    set_service_model!(template_d2, ServiceModel(OnlineReserve{ReserveUp}, RangeReserve))

    for sys in [sys_d1, sys_d2, sys_em]
        outage_gens = ["Alta"]
        for name in outage_gens
            g = get_component(ThermalStandard, sys, name)
            transition_data = PSY.GeometricDistributionForcedOutage(;
                mean_time_to_recovery = outage_length,
                outage_transition_probability = 1.0,
            )
            add_supplemental_attribute!(sys, g, transition_data)
        end
    end

    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_d1,
                sys_d1;
                name = "D1",
                initialize_model = false,
                optimizer = optimizers[1],
            ),
            DecisionModel(
                template_d2,
                sys_d2;
                name = "D2",
                initialize_model = false,
                optimizer = optimizers[2],
                store_variable_names = true,
            ),
        ],
        emulation_model = EmulationModel(
            template_em,
            sys_em;
            name = "EM",
            optimizer = optimizers[3],
            calculate_conflict = true,
            store_variable_names = true,
        ),
    )
    if feedforward
        sequence = SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
            feedforwards = Dict(
                "EM" => [# This FeedForward will force the commitment to be kept in the emulator
                    SemiContinuousFeedforward(;
                        component_type = ThermalStandard,
                        source = OnVariable,
                        affected_values = [ActivePowerVariable],
                        #   add_slacks = false,
                    ),
                ],
            ),
            events = [event_model],
        )
    else
        sequence = SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
            events = [event_model],
        )
    end
    sim = Simulation(;
        name = "no_cache",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim; console_level = Logging.Error)
    @test build_out == SimulationBuildStatus.BUILT
    execute_out = execute!(sim; in_memory = in_memory)
    @test execute_out == IOM.RunStatus.SUCCESSFULLY_FINALIZED
    results = SimulationResults(sim; ignore_status = true)
    return results
end

function test_event_results(;
    res,
    outage_time,
    outage_length,
    expected_power_recovery,
    expected_on_variable_recovery,
    test_reactive_power = false,
)
    em = get_emulation_problem_results(res)
    p = read_realized_variable(
        em,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    count = read_realized_variable(
        em,
        "AvailableStatusChangeCountdownParameter__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    on = read_realized_variable(
        em,
        "OnVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    status = read_realized_variable(
        em,
        "AvailableStatusParameter__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    outage_ix = indexin([outage_time], p[!, :DateTime])[1]
    outage_length_ix = Int64((Hour(1) / em.resolution) * outage_length)
    on_recover_ix = indexin([expected_on_variable_recovery], p[!, :DateTime])[1]
    p_recover_ix = indexin([expected_power_recovery], p[!, :DateTime])[1]
    #Test condition at time of outage
    @test count[outage_ix, "Alta"] == 0.0
    @test status[outage_ix, "Alta"] == 1.0
    @test on[outage_ix, "Alta"] == 1.0
    @test p[outage_ix, "Alta"] != 0.0
    #Test condition during outage
    @test count[(outage_ix + 1):(outage_ix + outage_length_ix), "Alta"] ==
          outage_length_ix:-1.0:1.0
    @test status[(outage_ix + 1):(outage_ix + outage_length_ix), "Alta"] ==
          zeros(outage_length_ix)
    @test on[(on_recover_ix - 1), "Alta"] == 0.0  #on variable not necessarily zero for full time; possibly updated later
    length_p_zero = p_recover_ix - outage_ix - 1
    @test isapprox(
        p[(outage_ix + 1):(p_recover_ix - 1), "Alta"],
        zeros(length_p_zero);
        atol = 1e-5,
    )
    #Test condition after outage
    @test count[(outage_ix + outage_length_ix + 1), "Alta"] == 0.0
    @test status[(outage_ix + outage_length_ix + 1), "Alta"] == 1.0
    @test isapprox(on[on_recover_ix, "Alta"], 1.0; atol = 1e-5)
    @test !isapprox(p[p_recover_ix, "Alta"], 0.0; atol = 1e-5)
    if test_reactive_power == true
        q = read_realized_variable(
            em,
            "ReactivePowerVariable__ThermalStandard";
            table_format = TableFormat.WIDE,
        )
        @test isapprox(
            q[(outage_ix + 1):(p_recover_ix - 1), "Alta"],
            zeros(length_p_zero);
            atol = 5e-2,
        )
        @test q[p_recover_ix, "Alta"] != 0.0
        @test !isapprox(q[p_recover_ix, "Alta"], 0.0; atol = 5e-2)
    end
    return
end
