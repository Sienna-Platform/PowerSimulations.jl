### HOURLY DATA ###
#Note: if using basic for ed, emulator fails at timestep  after outage due to OutageConstraint_ub
@testset "Hourly; uc basic; ed nomin; no ff" begin
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        uc_formulation = "basic",
        ed_formulation = "nomin",
        feedforward = false,
        in_memory = true,
    )
    test_event_results(;
        res = res,
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
    #Test no ramping constraint in D2 model results
    d2 = get_decision_problem_results(res, "D2")
    p_d2 =
        read_realized_variables(d2; table_format = TableFormat.WIDE)["ActivePowerVariable__ThermalStandard"]
    p_recover_ix = indexin([DateTime("2024-01-01T22:00:00")], p_d2[!, :DateTime])[1]
    @test p_d2[p_recover_ix, "Alta"] == 40.0
end

#This passes with nomin or basic dispatch
@testset "Hourly; uc basic; ed basic; ff" begin
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        uc_formulation = "basic",
        ed_formulation = "basic",  #should also pass with nomin
        feedforward = true,
        in_memory = true,
    )
    test_event_results(;
        res = res,
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
    #Test no ramping constraint in D2 model results
    d2 = get_decision_problem_results(res, "D2")
    p_d2 =
        read_realized_variables(d2; table_format = TableFormat.WIDE)["ActivePowerVariable__ThermalStandard"]
    p_recover_ix = indexin([DateTime("2024-01-01T22:00:00")], p_d2[!, :DateTime])[1]
    @test p_d2[p_recover_ix, "Alta"] == 40.0
end

# Note: Running a standard UC formulation without a feedforward to the ED is not a feasible modeling setup
#Active power can change in Em without regard for OnVariable which messes up initializing the standard UC models.

# This tests for both min up and down times being handled properly with events.
# Generator not turned back on until 4 hours after the event (event only lasts 3 hours)
# Generator is only on for one hour when the event happens; the constraint is bypassed by resetting the TimeDurationOn variable to a large value.
@testset "Hourly; uc standard; ed basic; ff" begin
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = DateTime("2024-01-01T17:00:00"),
        outage_length = 3.0,
        uc_formulation = "standard",
        ed_formulation = "basic",  #should also pass with nomin
        feedforward = true,
        in_memory = true,
    )
    test_event_results(;
        res = res,
        outage_time = DateTime("2024-01-01T17:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
    #Test ramping constraint in D2 model results
    d2 = get_decision_problem_results(res, "D2")
    p_d2 =
        read_realized_variables(d2; table_format = TableFormat.WIDE)["ActivePowerVariable__ThermalStandard"]
    p_recover_ix = indexin([DateTime("2024-01-01T22:00:00")], p_d2[!, :DateTime])[1]
    @test p_d2[p_recover_ix, "Alta"] < 40.0
end

### 5 MINUTE DATA (RESOLUTION MISMATCH) ###

#Note: if using basic for ed, emulator fails at timestep  after outage due to OutageConstraint_ub
@testset "5 min; uc basic; ed nomin; no ff" begin
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events_rt"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        uc_formulation = "basic",
        ed_formulation = "nomin",
        feedforward = false,
        in_memory = true,
    )
    test_event_results(;
        res = res,
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T21:05:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
end

@testset "5 min; uc basic; ed basic; ff" begin
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events_rt"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        uc_formulation = "basic",
        ed_formulation = "basic",
        feedforward = true,
        in_memory = false,
    )
    test_event_results(;
        res = res,
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
end

# Note: Running a standard UC formulation without a feedforward to the ED is not a feasible modeling setup
#Active power can change in Em without regard for OnVariable which messes up initializing the standard UC models.

@testset "5 min; uc standard; ed basic; ff" begin
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events_rt"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = DateTime("2024-01-01T17:00:00"),
        outage_length = 3.0,
        uc_formulation = "standard",
        ed_formulation = "basic",  #should also pass with nomin
        feedforward = true,
        in_memory = true,
    )
    test_event_results(;
        res = res,
        outage_time = DateTime("2024-01-01T17:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
    #Test ramping constraint in D2 model results
    d2 = get_decision_problem_results(res, "D2")
    p_d2 =
        read_realized_variables(d2; table_format = TableFormat.WIDE)["ActivePowerVariable__ThermalStandard"]
    p_recover_ix = indexin([DateTime("2024-01-01T22:00:00")], p_d2[!, :DateTime])[1]
    @test p_d2[p_recover_ix, "Alta"] < 40.0
end

# A UC-only key updates the system state at hourly boundaries only, so the 5-minute rows of
# the final hour are never flushed by the step loop. They must still be written, holding the
# last hourly value, or the store reads them back as NaN (in memory) or 0.0 (HDF).
function _test_trailing_rows_held(res)
    em = get_emulation_problem_results(res)
    on_em = read_realized_variable(
        em,
        "OnVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    last_hour = on_em[(end - 11):end, :]
    @test last_hour[1, :DateTime] == DateTime("2024-01-01T23:00:00")
    @test !any(isnan, Matrix(last_hour[:, Not(:DateTime)]))
    # Both D1 and D2 write OnVariable; the hourly D2 runs after the daily D1, so its 23:00
    # solve is the last writer of the decision state the emulator copies verbatim.
    d2 = get_decision_problem_results(res, "D2")
    on_d2 = read_realized_variable(
        d2,
        "OnVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    held = on_d2[on_d2.DateTime .== DateTime("2024-01-01T23:00:00"), :]
    @test nrow(held) == 1
    for name in names(last_hour, Not(:DateTime))
        @test all(last_hour[!, name] .== held[1, name])
    end
    return
end

@testset "5 min emulator; no events attached" begin
    # Regression for the alignment bug: a sub-hourly emulator reading hourly decision
    # state must succeed with no event models attached at all.
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events_rt"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        uc_formulation = "basic",
        ed_formulation = "nomin",
        feedforward = false,
        in_memory = true,
        attach_events = false,
    )
    em = get_emulation_problem_results(res)
    p = read_realized_variable(
        em,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    @test nrow(p) > 0
    @test p[2, :DateTime] - p[1, :DateTime] == Minute(5)
    #Test the hourly D2 decision model is also readable
    d2 = get_decision_problem_results(res, "D2")
    p_d2 =
        read_realized_variables(d2; table_format = TableFormat.WIDE)["ActivePowerVariable__ThermalStandard"]
    @test nrow(p_d2) > 0
    _test_trailing_rows_held(res)
end

@testset "5 min emulator; no events attached; HDF store" begin
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events_rt"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        uc_formulation = "basic",
        ed_formulation = "nomin",
        feedforward = false,
        in_memory = false,
        attach_events = false,
    )
    _test_trailing_rows_held(res)
end

@testset "FixedForcedOutage with timeseries" begin
    dates_ts = collect(
        DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"),
    )
    outage_data = fill!(Vector{Int64}(undef, 48), 0)
    outage_data[3] = 1
    outage_data[10:11] .= 1
    outage_data[22:23] .= 1
    outage_timeseries = TimeArray(dates_ts, outage_data)
    res = run_fixed_forced_outage_sim_with_timeseries(;
        sys = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_status_timeseries = outage_timeseries,
        device_type = ThermalStandard,
        device_names = ["Alta"],
        renewable_formulation = RenewableFullDispatch,
    )
    em = get_emulation_problem_results(res)
    status = read_realized_variable(
        em,
        "AvailableStatusParameter__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    apv = read_realized_variable(
        em,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    for (ix, x) in enumerate(outage_data[1:24])
        @test x != Int64(status[!, "Alta"][ix])
        if Int64(status[!, "Alta"][ix]) == 0.0
            @test apv[!, "Alta"][ix] == 0.0
        end
    end
end

@testset "Renewable outage" begin
    dates_ts = collect(
        DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"),
    )
    outage_data = fill!(Vector{Int64}(undef, 48), 0)
    outage_data[3] = 1
    outage_data[10:11] .= 1
    outage_data[22:23] .= 1
    outage_timeseries = TimeArray(dates_ts, outage_data)
    res = run_fixed_forced_outage_sim_with_timeseries(;
        sys = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_status_timeseries = outage_timeseries,
        device_type = RenewableDispatch,
        device_names = ["WindBus1"],
        renewable_formulation = RenewableFullDispatch,
    )
    em = get_emulation_problem_results(res)
    status = read_realized_variable(
        em,
        "AvailableStatusParameter__RenewableDispatch";
        table_format = TableFormat.WIDE,
    )
    apv = read_realized_variable(
        em,
        "ActivePowerVariable__RenewableDispatch";
        table_format = TableFormat.WIDE,
    )
    for (ix, x) in enumerate(outage_data[1:24])
        @test x != Int64(status[!, "WindBus1"][ix])
        if Int64(status[!, "WindBus1"][ix]) == 0.0
            @test apv[!, "WindBus1"][ix] == 0.0
        end
    end
end

@testset "Renewable outage projected into decision model" begin
    # Regression for removal of the `_supports_event_extension` gate that used
    # to skip RenewableDispatch in `extend_event_parameters!`. The existing
    # "Renewable outage" testset only checks the emulator; this asserts the
    # outage is also projected into the D2 *decision* model's status/power.
    dates_ts = collect(
        DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"),
    )
    outage_data = fill!(Vector{Int64}(undef, 48), 0)
    outage_data[3] = 1
    outage_data[10:11] .= 1
    outage_data[22:23] .= 1
    outage_timeseries = TimeArray(dates_ts, outage_data)
    res = run_fixed_forced_outage_sim_with_timeseries(;
        sys = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_status_timeseries = outage_timeseries,
        device_type = RenewableDispatch,
        device_names = ["WindBus1"],
        renewable_formulation = RenewableFullDispatch,
    )
    d2 = PSI.get_decision_problem_results(res, "D2")
    status = read_realized_variable(
        d2,
        "AvailableStatusParameter__RenewableDispatch";
        table_format = TableFormat.WIDE,
    )
    apv = read_realized_variable(
        d2,
        "ActivePowerVariable__RenewableDispatch";
        table_format = TableFormat.WIDE,
    )
    for (ix, x) in enumerate(outage_data[1:24])
        @test x != Int64(status[!, "WindBus1"][ix])
        if Int64(status[!, "WindBus1"][ix]) == 0.0
            @test apv[!, "WindBus1"][ix] == 0.0
        end
    end
end

@testset "Load outage" begin
    dates_ts = collect(
        DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"),
    )
    outage_data = fill!(Vector{Int64}(undef, 48), 0)
    outage_data[3] = 1
    outage_data[10:11] .= 1
    outage_data[22:23] .= 1
    outage_timeseries = TimeArray(dates_ts, outage_data)
    res = run_fixed_forced_outage_sim_with_timeseries(;
        sys = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_status_timeseries = outage_timeseries,
        device_type = InterruptiblePowerLoad,
        device_names = ["IloadBus4"],
        renewable_formulation = RenewableFullDispatch,
    )
    em = get_emulation_problem_results(res)
    status = read_realized_variable(
        em,
        "AvailableStatusParameter__InterruptiblePowerLoad";
        table_format = TableFormat.WIDE,
    )
    apv = read_realized_variable(
        em,
        "ActivePowerVariable__InterruptiblePowerLoad";
        table_format = TableFormat.WIDE,
    )

    for (ix, x) in enumerate(outage_data[1:24])
        @test x != Int64(status[!, "IloadBus4"][ix])
        if Int64(status[!, "IloadBus4"][ix]) == 0.0
            @test apv[!, "IloadBus4"][ix] == 0.0
        end
    end
end

@testset "StaticPowerLoad outage" begin
    dates_ts = collect(
        DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"),
    )
    outage_data = fill!(Vector{Int64}(undef, 48), 0)
    outage_timeseries = TimeArray(dates_ts, outage_data)
    res = run_fixed_forced_outage_sim_with_timeseries(;
        sys = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_status_timeseries = outage_timeseries,
        device_type = PowerLoad,
        device_names = ["Bus2"],
        renewable_formulation = RenewableFullDispatch,
    )
    em = get_emulation_problem_results(res)
    active_power_thermal_no_outage =
        read_realized_variable(
            em,
            "ActivePowerVariable__ThermalStandard";
            table_format = TableFormat.WIDE,
        )
    outage_data[3] = 1
    outage_data[10:11] .= 1
    outage_data[22:23] .= 1
    outage_timeseries = TimeArray(dates_ts, outage_data)
    res = run_fixed_forced_outage_sim_with_timeseries(;
        sys = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_status_timeseries = outage_timeseries,
        device_type = PowerLoad,
        device_names = ["Bus2"],
        renewable_formulation = RenewableFullDispatch,
    )
    em = get_emulation_problem_results(res)
    status = read_realized_variable(
        em,
        "AvailableStatusParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    active_power_load =
        read_realized_variable(
            em,
            "ActivePowerTimeSeriesParameter__PowerLoad";
            table_format = TableFormat.WIDE,
        )
    active_power_thermal_outage =
        read_realized_variable(
            em,
            "ActivePowerVariable__ThermalStandard";
            table_format = TableFormat.WIDE,
        )
    for (ix, x) in enumerate(outage_data[1:24])
        @test x != Int64(status[!, "Bus2"][ix])
        if outage_data[ix] == 1.0
            change_in_thermal_generation = sum(
                Vector(active_power_thermal_outage[ix, 2:end]) .-
                Vector(active_power_thermal_no_outage[ix, 2:end]),
            )
            active_power_outaged_load = active_power_load[ix, "Bus2"]
            @test isapprox(change_in_thermal_generation, active_power_outaged_load)
        end
    end
end

@testset "FixedOutput outage" begin
    dates_ts = collect(
        DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"),
    )
    outage_data = fill!(Vector{Int64}(undef, 48), 0)
    outage_timeseries = TimeArray(dates_ts, outage_data)
    res = run_fixed_forced_outage_sim_with_timeseries(;
        sys = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_status_timeseries = outage_timeseries,
        device_type = PowerLoad,
        device_names = ["Bus2"],
        renewable_formulation = RenewableFullDispatch,
    )
    em = get_emulation_problem_results(res)
    active_power_thermal_no_outage =
        read_realized_variable(
            em,
            "ActivePowerVariable__ThermalStandard";
            table_format = TableFormat.WIDE,
        )
    outage_data[3] = 1
    outage_data[10:11] .= 1
    outage_data[22:23] .= 1
    outage_timeseries = TimeArray(dates_ts, outage_data)
    res = run_fixed_forced_outage_sim_with_timeseries(;
        sys = build_system(PSITestSystems, "c_sys5_events"),
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_status_timeseries = outage_timeseries,
        device_type = RenewableDispatch,
        device_names = ["WindBus1"],
        renewable_formulation = FixedOutput,
    )
    em = get_emulation_problem_results(res)
    renewable_status =
        read_realized_variable(
            em,
            "AvailableStatusParameter__RenewableDispatch";
            table_format = TableFormat.WIDE,
        )
    active_power_thermal_outage =
        read_realized_variable(
            em,
            "ActivePowerVariable__ThermalStandard";
            table_format = TableFormat.WIDE,
        )
    active_power_renewable =
        read_realized_variable(
            em,
            "ActivePowerTimeSeriesParameter__RenewableDispatch";
            table_format = TableFormat.WIDE,
        )
    for (ix, x) in enumerate(outage_data[1:24])
        @test x != Int64(renewable_status[!, "WindBus1"][ix])
        if outage_data[ix] == 1.0
            change_in_thermal_generation = sum(
                Vector(active_power_thermal_outage[ix, 2:end]) .-
                Vector(active_power_thermal_no_outage[ix, 2:end]),
            )
            active_power_outaged_renewable = active_power_renewable[ix, "WindBus1"]
            @test isapprox(change_in_thermal_generation, active_power_outaged_renewable)
        end
    end
end

@testset "Reactive power formulation w/ outage" begin
    res = run_events_simulation(;
        sys_emulator = build_system(PSITestSystems, "c_sys5_events"),
        networks = [PTDFNetworkModel, PTDFNetworkModel, ACPNetworkModel],
        optimizers = [
            HiGHS_optimizer_small_gap,
            HiGHS_optimizer_small_gap,
            ipopt_optimizer,
        ],
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        uc_formulation = "basic",
        ed_formulation = "basic",
        feedforward = true,
        in_memory = true,
    )
    test_event_results(;
        res = res,
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
        test_reactive_power = true,
    )
end

# c_sys5_events without the thermal units in `removed_gens`, and with a
# GeometricDistributionForcedOutage, drawn with certainty, on each of `outage_gens`.
function events_system(;
    outage_gens = String[],
    removed_gens = String[],
    horizon_interval = nothing,
)
    sys = build_system(PSITestSystems, "c_sys5_events")
    for name in removed_gens
        remove_component!(sys, get_component(ThermalStandard, sys, name))
    end
    isnothing(horizon_interval) || transform_single_time_series!(sys, horizon_interval...)
    for name in outage_gens
        add_supplemental_attribute!(
            sys,
            get_component(ThermalStandard, sys, name),
            PSY.GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 3.0,
                outage_transition_probability = 1.0,
            ),
        )
    end
    return sys
end

function events_simulation(
    sys_d1,
    sys_d2,
    sys_em,
    event_model;
    rebuild_model = false,
)
    templates = map(1:2) do _
        template = get_template_basic_uc_simulation()
        set_network_model!(template, NetworkModel(CopperPlateNetworkModel))
        set_device_model!(template, Line, StaticBranch)
        set_service_model!(template, ServiceModel(OnlineReserve{ReserveUp}, RangeReserve))
        template
    end
    decision_models = [
        DecisionModel(
            templates[1],
            sys_d1;
            name = "D1",
            initialize_model = false,
            optimizer = HiGHS_optimizer_small_gap,
            rebuild_model = rebuild_model,
        ),
        DecisionModel(
            templates[2],
            sys_d2;
            name = "D2",
            initialize_model = false,
            optimizer = HiGHS_optimizer_small_gap,
            rebuild_model = rebuild_model,
        ),
    ]
    if isnothing(sys_em)
        models = SimulationModels(; decision_models = decision_models)
    else
        template_em = get_template_nomin_ed_simulation(CopperPlateNetworkModel)
        set_device_model!(template_em, Line, StaticBranch)
        models = SimulationModels(;
            decision_models = decision_models,
            emulation_model = EmulationModel(
                template_em,
                sys_em;
                name = "EM",
                optimizer = HiGHS_optimizer_small_gap,
                rebuild_model = rebuild_model,
            ),
        )
    end
    sequence = SimulationSequence(;
        models = models,
        ini_cond_chronology = InterProblemChronology(),
        events = [event_model],
    )
    return Simulation(;
        name = "events",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
end

preset_outage_event(outage_time) =
    EventModel(GeometricDistributionForcedOutage, POM.PresetTimeCondition([outage_time]))

function event_device_names(model)
    status = IOM.get_parameter_array(
        IOM.get_optimization_container(model),
        AvailableStatusParameter(),
        ThermalStandard,
    )
    return Set(axes(status)[1])
end

@testset "Outages are applied from the emulator's system when attribute ids diverge" begin
    outage_time = DateTime("2024-01-01T18:00:00")
    sys_em = build_system(PSITestSystems, "c_sys5_events")
    bus = first(get_components(ACBus, sys_em))
    add_supplemental_attribute!(
        sys_em,
        bus,
        PSY.GeographicInfo(;
            geo_json = Dict("type" => "Point", "coordinates" => [0.0, 0.0]),
        ),
    )
    res = run_events_simulation(;
        sys_emulator = sys_em,
        networks = repeat([CopperPlateNetworkModel], 3),
        optimizers = repeat([HiGHS_optimizer_small_gap], 3),
        outage_time = outage_time,
        outage_length = 3.0,
        uc_formulation = "basic",
        ed_formulation = "nomin",
        feedforward = false,
        in_memory = true,
    )
    test_event_results(;
        res = res,
        outage_time = outage_time,
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
end

@testset "Every decision model sees the emulator's outages, and only those" begin
    # The emulator's system alone decides which outages exist; both of its outages are
    # drawn at `outage_time`. D1's own outage on Brighton does not affect events.
    outage_time = DateTime("2024-01-01T18:00:00")
    outaged = ["Alta", "Park City"]
    sys_d1 = events_system(;
        outage_gens = ["Brighton"],
        horizon_interval = (Day(2), Day(1)),
    )
    sys_d2 = events_system(; horizon_interval = (Hour(4), Hour(1)))
    sys_em = events_system(; outage_gens = outaged)
    event_model = preset_outage_event(outage_time)
    sim = events_simulation(sys_d1, sys_d2, sys_em, event_model)
    @test build!(sim; console_level = Logging.Error) == SimulationBuildStatus.BUILT

    @test POM.get_attribute_device_map(event_model) == Dict(
        IS.get_id(outage) =>
            Dict(
                ThermalStandard =>
                    Set(get_name.(get_associated_components(sys_em, outage))),
            )
        for outage in get_supplemental_attributes(GeometricDistributionForcedOutage, sys_em)
    )
    for model in PSI.get_decision_models(PSI.get_models(sim))
        @test event_device_names(model) == Set(outaged)
    end

    @test execute!(sim; in_memory = true) == IOM.RunStatus.SUCCESSFULLY_FINALIZED
    d2 = get_decision_problem_results(SimulationResults(sim; ignore_status = true), "D2")
    read_d2(name) = read_realized_variable(d2, name; table_format = TableFormat.WIDE)
    status = read_d2("AvailableStatusParameter__ThermalStandard")
    p = read_d2("ActivePowerVariable__ThermalStandard")
    @test Set(names(status, Not(:DateTime))) == Set(outaged)
    out = outage_time + Hour(1) .<= status.DateTime .<= outage_time + Hour(3)
    for name in outaged
        @test all(status[status.DateTime .<= outage_time, name] .== 1.0)
        @test all(status[out, name] .== 0.0)
        @test isapprox(p[out, name], zeros(count(out)); atol = 1e-5)
    end
end

# Run a simulation whose emulator outages `outaged` while the decision systems lack
# `emulator_only`, and return the realized results of the emulator and of D2.
function run_emulator_only_outage(outaged, emulator_only, outage_time)
    sys_d1 = events_system(;
        removed_gens = emulator_only,
        horizon_interval = (Day(2), Day(1)),
    )
    sys_d2 = events_system(;
        removed_gens = emulator_only,
        horizon_interval = (Hour(4), Hour(1)),
    )
    sys_em = events_system(; outage_gens = outaged)
    sim = events_simulation(sys_d1, sys_d2, sys_em, preset_outage_event(outage_time))
    @test build!(sim; console_level = Logging.Error) == SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == IOM.RunStatus.SUCCESSFULLY_FINALIZED
    res = SimulationResults(sim; ignore_status = true)
    return get_emulation_problem_results(res), get_decision_problem_results(res, "D2")
end

read_wide(res, name) = read_realized_variable(res, name; table_format = TableFormat.WIDE)

# The device is out for the three hours after `outage_time` and available before.
function test_outaged(res, name, outage_time)
    status = read_wide(res, "AvailableStatusParameter__ThermalStandard")
    p = read_wide(res, "ActivePowerVariable__ThermalStandard")
    out = outage_time + Hour(1) .<= status.DateTime .<= outage_time + Hour(3)
    @test all(status[status.DateTime .<= outage_time, name] .== 1.0)
    @test all(status[out, name] .== 0.0)
    @test isapprox(p[out, name], zeros(count(out)); atol = 1e-5)
    return
end

@testset "An outage on a device only the emulator has" begin
    outage_time = DateTime("2024-01-01T18:00:00")
    em, d2 = run_emulator_only_outage(["Alta", "Park City"], ["Park City"], outage_time)
    test_outaged(em, "Park City", outage_time)
    test_outaged(em, "Alta", outage_time)
    test_outaged(d2, "Alta", outage_time)
    status = read_wide(d2, "AvailableStatusParameter__ThermalStandard")
    @test names(status, Not(:DateTime)) == ["Alta"]
    @test !("Park City" in names(read_wide(d2, "ActivePowerVariable__ThermalStandard")))
end

@testset "An outage of a device type no decision model has event parameters for" begin
    outage_time = DateTime("2024-01-01T18:00:00")
    em, d2 = run_emulator_only_outage(["Park City"], ["Park City"], outage_time)
    test_outaged(em, "Park City", outage_time)
    @test !(
        "AvailableStatusParameter__ThermalStandard" in
        list_parameter_names(d2)
    )
end

@testset "Events without an emulation model fail the build" begin
    sys_d1 = events_system(; outage_gens = ["Alta"], horizon_interval = (Day(2), Day(1)))
    sys_d2 = events_system(; outage_gens = ["Alta"], horizon_interval = (Hour(4), Hour(1)))
    event_model = preset_outage_event(DateTime("2024-01-01T18:00:00"))
    sim = events_simulation(sys_d1, sys_d2, nothing, event_model)
    @test_throws IS.ConflictingInputsError build!(sim; console_level = Logging.Error)
    @test PSI.get_simulation_build_status(sim) == SimulationBuildStatus.FAILED
    @test isempty(POM.get_attribute_device_map(event_model))
end

@testset "Rebuilt models with events (rebuild_model = true)" begin
    outage_time = DateTime("2024-01-01T18:00:00")
    sys_d1 = events_system(; horizon_interval = (Day(2), Day(1)))
    sys_d2 = events_system(; horizon_interval = (Hour(4), Hour(1)))
    sys_em = events_system(; outage_gens = ["Alta"])
    event_model = preset_outage_event(outage_time)
    sim = events_simulation(
        sys_d1,
        sys_d2,
        sys_em,
        event_model;
        rebuild_model = true,
    )
    @test all(
        IOM.get_rebuild_model(IOM.get_settings(model)) for
        model in PSI.get_decision_models(PSI.get_models(sim))
    )
    @test build!(sim; console_level = Logging.Error) == SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == IOM.RunStatus.SUCCESSFULLY_FINALIZED
    test_event_results(;
        res = SimulationResults(sim; ignore_status = true),
        outage_time = outage_time,
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
end
