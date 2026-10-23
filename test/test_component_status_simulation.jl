# Component statuses are parameters POM fills at build from outage and switching schedules;
# between steps PSI must recompute them for the new window, since the rebuild keeps the
# parameter containers.
@testset "Component statuses refresh between simulation steps" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_ed")
    line = first(sort!(collect(get_components(Line, sys)); by = get_name))
    initial_times = collect(PSY.get_forecast_initial_times(sys))
    resolution = first(PSY.get_time_series_resolutions(sys))
    n = Dates.Millisecond(PSY.get_forecast_horizon(sys)) ÷ Dates.Millisecond(resolution)
    # In service throughout window 1; out for the first three periods of window 2.
    outage_values(k) = [(k == 2 && i <= 3) ? 1.0 : 0.0 for i in 1:n]
    data = DataStructures.SortedDict(
        t => outage_values(k) for (k, t) in enumerate(initial_times)
    )
    outage = PSY.PlannedOutage(; outage_schedule = "maintenance")
    add_supplemental_attribute!(sys, line, outage)
    add_time_series!(sys, outage, Deterministic("maintenance", data, resolution))

    template =
        get_template_nomin_ed_simulation(NetworkModel(PTDFNetworkModel; use_slacks = true))
    model = DecisionModel(template, sys; name = "ED", optimizer = HiGHS_optimizer)
    models = SimulationModels(; decision_models = [model])
    sim = Simulation(;
        name = "component_status",
        steps = 2,
        models,
        sequence = SimulationSequence(; models),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test IOM.get_rebuild_model(IOM.get_settings(model))

    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
    results_ed = get_decision_problem_results(SimulationResults(sim), "ED")
    statuses = read_parameter(results_ed, POM.BranchStatusParameter, PSY.Line)
    @test sort!(collect(keys(statuses))) == initial_times[1:2]
    for (k, t) in enumerate(initial_times[1:2])
        step = @rsubset(statuses[t], :name == get_name(line))
        @test step[!, :value] == 1.0 .- outage_values(k)
    end
end

# A must-run compact unit's ramp rows read whether it was in service before each window from
# its outage schedule. The component's stored active power is the user's initial data and is
# not updated between windows, so in window 2, which starts mid-outage, it would claim the unit
# was producing and charge a drop from its minimum that its ramp-down limit cannot follow.
@testset "A must-run compact unit with an outage spanning the window boundary" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
    for load in get_components(PowerLoad, sys)
        set_max_active_power!(load, 0.4 * get_max_active_power(load, u"SU") * u"SU")
    end
    for g in get_components(ThermalStandard, sys)
        ramp = get_active_power_limits(g, u"SU").max
        set_ramp_limits!(g, (up = ramp * u"SU/minute", down = ramp * u"SU/minute"))
        set_time_limits!(g, nothing)
    end
    unit = get_component(ThermalStandard, sys, "Brighton")
    set_commitment_mode!(unit, PSY.CommitmentModes.MUST_RUN)
    pmax = get_max_active_power(unit, u"SU")
    # Its ramp-down is 0.25 Pmax per hour. Window 1 runs at a minimum of 0.1 Pmax, which the
    # unit can drop from; window 2 at 0.3 Pmax, which it cannot drop from in one step.
    set_ramp_limits!(
        unit,
        (up = pmax * u"SU/minute", down = 0.25 * pmax / 60 * u"SU/minute"),
    )
    set_active_power!(unit, 0.5 * pmax * u"SU")
    initial_times = collect(PSY.get_forecast_initial_times(sys))
    resolution = first(PSY.get_time_series_resolutions(sys))
    n = Dates.Millisecond(PSY.get_forecast_horizon(sys)) ÷ Dates.Millisecond(resolution)
    minimum_series = DataStructures.SortedDict(
        t => fill(k == 1 ? 0.1 : 0.3, n) for (k, t) in enumerate(initial_times)
    )
    add_time_series!(
        sys,
        unit,
        Deterministic("min_active_power", minimum_series, resolution),
    )
    # Out from step 13 of window 1 through step 4 of window 2.
    is_out(k, i) = (k == 1 && i >= 13) || (k == 2 && i <= 4)
    outage_series = DataStructures.SortedDict(
        t => [is_out(k, i) ? 1.0 : 0.0 for i in 1:n] for (k, t) in enumerate(initial_times)
    )
    outage = PSY.PlannedOutage(; outage_schedule = "maintenance")
    add_supplemental_attribute!(sys, unit, outage)
    add_time_series!(sys, outage, Deterministic("maintenance", outage_series, resolution))

    template = PowerOperationsProblemTemplate(CopperPlateNetworkModel)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(
        template,
        DeviceModel(
            ThermalStandard,
            ThermalCompactUnitCommitment;
            time_series_names = Dict(
                POM.MinActivePowerTimeSeriesParameter => "min_active_power",
            ),
        ),
    )
    model = DecisionModel(template, sys; name = "UC", optimizer = HiGHS_optimizer)
    models = SimulationModels(; decision_models = [model])
    sim = Simulation(;
        name = "must_run_outage_boundary",
        steps = 2,
        models,
        sequence = SimulationSequence(; models),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
    results = get_decision_problem_results(SimulationResults(sim), "UC")
    output = read_aux_variable(results, POM.PowerOutput, ThermalStandard)
    for (k, t) in enumerate(initial_times[1:2])
        values = @rsubset(output[t], :name == "Brighton")[!, :value]
        @test all(isapprox.(values[[is_out(k, i) for i in 1:n]], 0.0; atol = 1e-6))
    end
end

const INJECTOR_STATUS_KEY =
    IOM.ParameterKey{POM.InjectorStatusParameter, ThermalStandard}("")

# Hourly UC (24-hour windows) and ED (1-hour windows) systems from `c_sys5_uc`, with loads
# scaled down so the thermal fleet covers them with any unit out.
function _status_stage_systems()
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    PSY.transform_single_time_series!(sys_ed, Hour(1), Hour(1))
    for sys in (sys_uc, sys_ed), load in get_components(PowerLoad, sys)
        set_max_active_power!(load, 0.4 * get_max_active_power(load, u"SU") * u"SU")
    end
    return sys_uc, sys_ed
end

# A planned outage on `unit_name` whose schedule is 1 (out) at the hours `is_out` selects.
function _add_uc_outage!(sys, unit_name, is_out)
    unit = get_component(ThermalStandard, sys, unit_name)
    initial_times = collect(PSY.get_forecast_initial_times(sys))
    resolution = first(PSY.get_time_series_resolutions(sys))
    n = Dates.Millisecond(PSY.get_forecast_horizon(sys)) ÷ Dates.Millisecond(resolution)
    data = DataStructures.SortedDict(
        t => [is_out(i) ? 1.0 : 0.0 for i in 1:n] for t in initial_times
    )
    outage = PSY.PlannedOutage(; outage_schedule = "maintenance")
    add_supplemental_attribute!(sys, unit, outage)
    add_time_series!(sys, outage, Deterministic("maintenance", data, resolution))
    return
end

# The ED system reads schedules as `SingleTimeSeries` under its transformed forecasts.
function _add_ed_outage!(sys, unit_name, is_out)
    unit = get_component(ThermalStandard, sys, unit_name)
    timestamps =
        collect(DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"))
    values = [is_out(Dates.hour(t) + 1) ? 1.0 : 0.0 for t in timestamps]
    outage = PSY.PlannedOutage(; outage_schedule = "maintenance")
    add_supplemental_attribute!(sys, unit, outage)
    add_time_series!(
        sys,
        outage,
        SingleTimeSeries("maintenance", TimeArray(timestamps, values)),
    )
    return
end

function _run_status_stages(sys_uc, sys_ed)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                get_template_basic_uc_simulation(),
                sys_uc;
                name = "UC",
                optimizer = HiGHS_optimizer,
            ),
            DecisionModel(
                get_template_nomin_ed_simulation(),
                sys_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
            ),
        ],
    )
    sim = Simulation(;
        name = "status_stages",
        steps = 1,
        models = models,
        sequence = SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
        ),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return sim
end

# The decision-state value of `unit_name` at the step's `hour` (0-based), after every model ran.
function _state_status(sim, unit_name, hour)
    dataset = PSI.get_dataset(
        PSI.get_decision_states(PSI.get_simulation_state(sim)),
        INJECTOR_STATUS_KEY,
    )
    row = findfirst(==(PSI.get_initial_time(sim) + Hour(hour)), dataset.timestamps)
    return dataset.values[unit_name, row]
end

function _written_status_columns(results, name)
    steps = read_parameter(
        get_decision_problem_results(results, name),
        POM.InjectorStatusParameter,
        ThermalStandard,
    )
    return sort!(unique(reduce(vcat, (step[!, :name] for step in values(steps)))))
end

@testset "Stages scheduling different units share one status state" begin
    sys_uc, sys_ed = _status_stage_systems()
    # UC takes Alta out for its last five hours; ED takes Park City out for its last two.
    _add_uc_outage!(sys_uc, "Alta", i -> i >= 20)
    _add_ed_outage!(sys_ed, "Park City", i -> i >= 23)
    sim = _run_status_stages(sys_uc, sys_ed)

    state = PSI.get_simulation_state(sim)
    for datasets in (PSI.get_decision_states(state), PSI.get_system_states(state))
        columns = PSI.get_column_names(
            INJECTOR_STATUS_KEY,
            PSI.get_dataset(datasets, INJECTOR_STATUS_KEY),
        )[1]
        @test sort(columns) == ["Alta", "Park City"]
    end

    # Each model writes the columns its own system schedules.
    results = SimulationResults(sim)
    @test _written_status_columns(results, "UC") == ["Alta"]
    @test _written_status_columns(results, "ED") == ["Park City"]

    # ED ran last at every hour. Park City follows ED's schedule; Alta is unscheduled and
    # available in ED's system, so ED's fill replaces the outage UC wrote.
    @test _state_status(sim, "Park City", 21) == 1.0
    @test _state_status(sim, "Park City", 23) == 0.0
    @test _state_status(sim, "Alta", 10) == 1.0
    @test _state_status(sim, "Alta", 22) == 1.0
end

@testset "A model without the status key fills it from its own system" begin
    sys_uc, sys_ed = _status_stage_systems()
    out_late = i -> i >= 20
    _add_uc_outage!(sys_uc, "Alta", out_late)
    _add_uc_outage!(sys_uc, "Park City", out_late)
    _add_uc_outage!(sys_uc, "Sundance", _ -> false)
    # ED schedules nothing, so it has no status parameter. Alta is available in its system,
    # Sundance is not, and Park City is not in it.
    set_available!(get_component(ThermalStandard, sys_ed, "Sundance"), false)
    remove_component!(sys_ed, get_component(ThermalStandard, sys_ed, "Park City"))
    sim = _run_status_stages(sys_uc, sys_ed)

    @test !haskey(
        IOM.get_parameters(
            IOM.get_optimization_container(PSI.get_simulation_model(sim, :ED)),
        ),
        INJECTOR_STATUS_KEY,
    )
    @test _state_status(sim, "Alta", 22) == 1.0
    @test _state_status(sim, "Sundance", 5) == 0.0
    # Absent from ED's system: keeps what UC wrote.
    @test _state_status(sim, "Park City", 5) == 1.0
    @test _state_status(sim, "Park City", 22) == 0.0
end
