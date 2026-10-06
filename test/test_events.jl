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
    test_event_outputs(;
        res = res,
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
    #Test no ramping constraint in D2 model outputs
    d2 = get_decision_problem_outputs(res, "D2")
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
    test_event_outputs(;
        res = res,
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
    #Test no ramping constraint in D2 model outputs
    d2 = get_decision_problem_outputs(res, "D2")
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
    test_event_outputs(;
        res = res,
        outage_time = DateTime("2024-01-01T17:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
    #Test ramping constraint in D2 model outputs
    d2 = get_decision_problem_outputs(res, "D2")
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
    test_event_outputs(;
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
    test_event_outputs(;
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
    test_event_outputs(;
        res = res,
        outage_time = DateTime("2024-01-01T17:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
    )
    #Test ramping constraint in D2 model outputs
    d2 = get_decision_problem_outputs(res, "D2")
    p_d2 =
        read_realized_variables(d2; table_format = TableFormat.WIDE)["ActivePowerVariable__ThermalStandard"]
    p_recover_ix = indexin([DateTime("2024-01-01T22:00:00")], p_d2[!, :DateTime])[1]
    @test p_d2[p_recover_ix, "Alta"] < 40.0
end

# A UC-only key updates the system state at hourly boundaries only, so the 5-minute rows of
# the final hour are never flushed by the step loop. They must still be written, holding the
# last hourly value, or the store reads them back as NaN (in memory) or 0.0 (HDF).
function _test_trailing_rows_held(res)
    em = get_emulation_problem_outputs(res)
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
    d2 = get_decision_problem_outputs(res, "D2")
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
    em = get_emulation_problem_outputs(res)
    p = read_realized_variable(
        em,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    @test nrow(p) > 0
    @test p[2, :DateTime] - p[1, :DateTime] == Minute(5)
    #Test the hourly D2 decision model is also readable
    d2 = get_decision_problem_outputs(res, "D2")
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
    em = get_emulation_problem_outputs(res)
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
    em = get_emulation_problem_outputs(res)
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
    d2 = PSI.get_decision_problem_outputs(res, "D2")
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
    em = get_emulation_problem_outputs(res)
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
    em = get_emulation_problem_outputs(res)
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
    em = get_emulation_problem_outputs(res)
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
    em = get_emulation_problem_outputs(res)
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
    em = get_emulation_problem_outputs(res)
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
    test_event_outputs(;
        res = res,
        outage_time = DateTime("2024-01-01T18:00:00"),
        outage_length = 3.0,
        expected_power_recovery = DateTime("2024-01-01T22:00:00"),
        expected_on_variable_recovery = DateTime("2024-01-01T22:00:00"),
        test_reactive_power = true,
    )
end
