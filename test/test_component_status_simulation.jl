# Component statuses are parameters POM fills at build from availability and status schedules;
# between steps PSI must recompute them for the new window, since the rebuild keeps the
# parameter containers.
@testset "Component statuses refresh between simulation steps" begin
    sys = deepcopy(PSB.build_system(PSITestSystems, "c_sys5_ed"))
    line = first(sort!(collect(get_components(Line, sys)); by = get_name))
    initial_times = collect(PSY.get_forecast_initial_times(sys))
    resolution = first(PSY.get_time_series_resolutions(sys))
    n = Dates.Millisecond(PSY.get_forecast_horizon(sys)) ÷ Dates.Millisecond(resolution)
    # In service throughout window 1; out for the first three periods of window 2.
    outage_values(k) = [(k == 2 && i <= 3) ? 1.0 : 0.0 for i in 1:n]
    data = DataStructures.SortedDict(
        t => 1.0 .- outage_values(k) for (k, t) in enumerate(initial_times)
    )
    add_time_series!(sys, line, Deterministic(POM.AVAILABLE_SCHEDULE, data, resolution))

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

# Scale the "max_active_power" series of every `PowerLoad` by `factor`, keeping each
# series' units labels.
function _scale_load_series!(sys, factor)
    name = "max_active_power"
    for load in get_components(PowerLoad, sys)
        if PSY.has_time_series(load, SingleTimeSeries, name)
            ts = get_time_series(SingleTimeSeries, load, name)
            scaled = SingleTimeSeries(ts, factor .* get_data(ts))
            remove_time_series!(sys, SingleTimeSeries, load, name)
            add_time_series!(sys, load, scaled)
        end
        if PSY.has_time_series(load, Deterministic, name)
            ts = get_time_series(Deterministic, load, name)
            data = DataStructures.SortedDict(k => factor .* v for (k, v) in get_data(ts))
            remove_time_series!(sys, Deterministic, load, name)
            add_time_series!(
                sys,
                load,
                Deterministic(;
                    name = name,
                    data = data,
                    resolution = PSY.get_resolution(ts),
                    units = IS.get_units(ts),
                    quantity_kind = IS.get_quantity_kind(ts),
                    per_unit_of = IS.get_per_unit_of(ts),
                ),
            )
        end
    end
    return
end

# A must-run compact unit's ramp rows read whether it was in service before each window from
# its outage schedule. The component's stored active power is the user's initial data and is
# not updated between windows, so in window 2, which starts mid-outage, it would claim the unit
# was producing and charge a drop from its minimum that its ramp-down limit cannot follow.
@testset "A must-run compact unit with an outage spanning the window boundary" begin
    sys = deepcopy(PSB.build_system(PSITestSystems, "c_sys5_uc"))
    _scale_load_series!(sys, 0.4)
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
    max_mw = get_max_active_power(unit, u"MW")
    minimum_series = DataStructures.SortedDict(
        t => fill((k == 1 ? 0.1 : 0.3) * max_mw, n) for (k, t) in enumerate(initial_times)
    )
    add_time_series!(
        sys,
        unit,
        Deterministic(
            "min_active_power",
            minimum_series,
            resolution;
            PSB.natural_units("active_power", "MW")...,
        ),
    )
    # Out from step 13 of window 1 through step 4 of window 2.
    is_out(k, i) = (k == 1 && i >= 13) || (k == 2 && i <= 4)
    outage_series = DataStructures.SortedDict(
        t => [is_out(k, i) ? 0.0 : 1.0 for i in 1:n] for (k, t) in enumerate(initial_times)
    )
    add_time_series!(
        sys,
        unit,
        Deterministic(POM.AVAILABLE_SCHEDULE, outage_series, resolution),
    )

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
# `ed_schedules!` adds the ED system's schedules before its series are transformed, so the
# transform derives their forecast views.
function _status_stage_systems(; ed_schedules! = sys -> nothing)
    sys_ed = deepcopy(
        PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true),
    )
    sys_uc = deepcopy(PSB.build_system(PSITestSystems, "c_sys5_uc"))
    _scale_load_series!(sys_uc, 0.4)
    _scale_load_series!(sys_ed, 0.4)
    ed_schedules!(sys_ed)
    PSY.transform_single_time_series!(sys_ed, Hour(1), Hour(1))
    return sys_uc, sys_ed
end

# An availability schedule on `unit_name` that is 0 (out) at the hours `is_out` selects.
function _add_uc_outage!(sys, unit_name, is_out)
    unit = get_component(ThermalStandard, sys, unit_name)
    initial_times = collect(PSY.get_forecast_initial_times(sys))
    resolution = first(PSY.get_time_series_resolutions(sys))
    n = Dates.Millisecond(PSY.get_forecast_horizon(sys)) ÷ Dates.Millisecond(resolution)
    data = DataStructures.SortedDict(
        t => [is_out(i) ? 0.0 : 1.0 for i in 1:n] for t in initial_times
    )
    add_time_series!(sys, unit, Deterministic(POM.AVAILABLE_SCHEDULE, data, resolution))
    return
end

# An hourly availability schedule over two days, for the ED system before its transform.
function _add_ed_outage!(sys, unit_name, is_out)
    unit = get_component(ThermalStandard, sys, unit_name)
    timestamps =
        collect(DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"))
    values = [is_out(Dates.hour(t) + 1) ? 0.0 : 1.0 for t in timestamps]
    add_time_series!(
        sys,
        unit,
        SingleTimeSeries(POM.AVAILABLE_SCHEDULE, TimeArray(timestamps, values)),
    )
    return
end

function _run_status_stages(sys_uc, sys_ed; allow_different_components = false)
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
            allow_different_components = allow_different_components,
        ),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return sim
end

# The decision-state value of `unit_name` at the step's `hour` (0-based), after every model ran.
function _state_status(sim, unit_name, hour)
    states = PSI.get_decision_states(PSI.get_simulation_state(sim))
    timestamps = PSI.get_dataset(states, INJECTOR_STATUS_KEY).timestamps
    row = findfirst(==(PSI.get_initial_time(sim) + Hour(hour)), timestamps)
    return PSI.get_dataset_values(states, INJECTOR_STATUS_KEY)[unit_name, row]
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
    sys_uc, sys_ed = _status_stage_systems(;
        ed_schedules! = sys -> _add_ed_outage!(sys, "Park City", i -> i >= 23),
    )
    # UC takes Alta out for its last five hours; ED takes Park City out for its last two.
    _add_uc_outage!(sys_uc, "Alta", i -> i >= 20)
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

    # Each column holds the schedule of the model that writes it.
    @test _state_status(sim, "Park City", 21) == 1.0
    @test _state_status(sim, "Park City", 23) == 0.0
    @test _state_status(sim, "Alta", 10) == 1.0
    @test _state_status(sim, "Alta", 22) == 0.0
end

@testset "A model without the status key leaves the status state to the models that write it" begin
    sys_uc, sys_ed = _status_stage_systems()
    out_late = i -> i >= 20
    _add_uc_outage!(sys_uc, "Alta", out_late)
    _add_uc_outage!(sys_uc, "Park City", out_late)
    _add_uc_outage!(sys_uc, "Sundance", _ -> false)
    # ED schedules nothing, so it has no status parameter. Sundance is unavailable in its
    # system and Park City is not in it.
    set_available!(get_component(ThermalStandard, sys_ed, "Sundance"), false)
    remove_component!(sys_ed, get_component(ThermalStandard, sys_ed, "Park City"))
    sim = _run_status_stages(sys_uc, sys_ed; allow_different_components = true)

    @test !haskey(
        IOM.get_parameters(
            IOM.get_optimization_container(PSI.get_simulation_model(sim, :ED)),
        ),
        INJECTOR_STATUS_KEY,
    )
    # Every column keeps what UC wrote, whatever ED's system holds.
    @test _state_status(sim, "Alta", 22) == 0.0
    @test _state_status(sim, "Sundance", 5) == 1.0
    @test _state_status(sim, "Park City", 5) == 1.0
    @test _state_status(sim, "Park City", 22) == 0.0
end
