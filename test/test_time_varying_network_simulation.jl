# Scheduled outages in simulations: each model reads its own status series, every step is
# rebuilt with that step's topology, and the solution equals a static model on a system with
# the out components unavailable.

const _TVN_T0 = DateTime("2024-01-01T00:00:00")

# A `PlannedOutage` on `component` whose series reads 1.0 (out) at the timestamps in `out`.
function _add_planned_outage!(sys, component, name, timestamps, out)
    outage = PSY.PlannedOutage(; outage_schedule = name)
    add_supplemental_attribute!(sys, component, outage)
    values = [t in out ? 1.0 : 0.0 for t in timestamps]
    add_time_series!(sys, outage, SingleTimeSeries(name, TimeArray(timestamps, values)))
    return outage
end

# A `PlannedOutage` on `component` whose forecast window starting at the `k`-th initial time
# reads `pattern(k, h)` at its `h`-th step.
function _add_forecast_outage!(sys, component, name, pattern)
    initial_times = collect(PSY.get_forecast_initial_times(sys))
    resolution = first(PSY.get_time_series_resolutions(sys))
    n = Dates.Millisecond(PSY.get_forecast_horizon(sys)) ÷ Dates.Millisecond(resolution)
    data = SortedDict(
        t => [Float64(pattern(k, h)) for h in 1:n] for
        (k, t) in enumerate(initial_times)
    )
    outage = PSY.PlannedOutage(; outage_schedule = name)
    add_supplemental_attribute!(sys, component, outage)
    add_time_series!(sys, outage, Deterministic(name, data, resolution))
    return outage
end

function _status_values(statuses, window_start, name)
    return @rsubset(statuses[window_start], :name == name)[!, :value]
end

#################################################################################
# Outage with an isolated injector, mid-simulation
#################################################################################

# No single c_sys5 line isolates a bus; bus 3 is isolated when both of its lines are out,
# and it carries a generator and a load, which go out with it.
const _ISLAND_LINES = ("4", "5")
const _ISLAND_INJECTORS = ((ThermalStandard, "Solitude"), (PowerLoad, "Bus3"))

# Windows of 12 hours every 4 hours: step 2 starts at hour 4 and step 3 at hour 8. The outage
# runs from hour 6, mid step 2, to hour 9, inside step 3.
const _TVN_HORIZON = Hour(12)
const _TVN_INTERVAL = Hour(4)
const _TVN_STEPS = 3
const _TVN_OUT = collect((_TVN_T0 + Hour(6)):Hour(1):(_TVN_T0 + Hour(9)))

# c_sys5_uc windowed for the simulation. Without the island's load, the minimum generation of
# the other units exceeds the demand at the low-load hours, so those minimums are zeroed; the
# island's unit keeps its minimum, which only its outage lifts.
function _windowed_dispatch_system()
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    for g in get_components(ThermalStandard, sys)
        get_name(g) == "Solitude" && continue
        limits = PSY.get_active_power_limits(g, u"SU")
        PSY.set_active_power_limits!(g, (min = 0.0 * u"SU", max = limits.max * u"SU"))
    end
    return sys
end

function _island_outage_system()
    sys = _windowed_dispatch_system()
    timestamps = collect(_TVN_T0:Hour(1):(_TVN_T0 + Hour(47)))
    for name in _ISLAND_LINES
        line = get_component(Line, sys, name)
        _add_planned_outage!(sys, line, "out_$name", timestamps, _TVN_OUT)
    end
    for (T, name) in _ISLAND_INJECTORS
        _add_planned_outage!(sys, get_component(T, sys, name), "out_$name", timestamps,
            _TVN_OUT)
    end
    PSY.transform_single_time_series!(sys, _TVN_HORIZON, _TVN_INTERVAL)
    return sys
end

# A static reference for one window: the unscheduled system with `out` unavailable.
function _static_reference_system(out::Bool)
    sys = _windowed_dispatch_system()
    if out
        for name in _ISLAND_LINES
            set_available!(get_component(Line, sys, name), false)
        end
        for (T, name) in _ISLAND_INJECTORS
            set_available!(get_component(T, sys, name), false)
        end
    end
    PSY.transform_single_time_series!(sys, _TVN_HORIZON, _TVN_INTERVAL)
    return sys
end

_island_template() = get_thermal_dispatch_template_network(NetworkModel(PTDFNetworkModel))

function _solve_static_window(window_start, out::Bool)
    model = DecisionModel(
        _island_template(),
        _static_reference_system(out);
        name = "static",
        optimizer = HiGHS_optimizer,
        initial_time = window_start,
        horizon = _TVN_HORIZON,
        interval = _TVN_INTERVAL,
    )
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          IOM.ModelBuildStatus.BUILT
    @test solve!(model) == IOM.RunStatus.SUCCESSFULLY_FINALIZED
    outputs = IOM.OptimizationProblemOutputs(model)
    return (
        dispatch = read_variable(
            outputs,
            ActivePowerVariable,
            ThermalStandard;
            table_format = TableFormat.WIDE,
        ),
        flows = read_expression(
            outputs,
            POM.PTDFBranchFlow,
            Line;
            table_format = TableFormat.WIDE,
        ),
    )
end

_wide(df) = unstack(df, :DateTime, :name, :value)

# `actual` and `expected` cover `window`, every column of `expected` is in `actual`, and every
# column of `actual` at the rows where `rows` holds equals the same column of `expected`; a
# column missing from `expected` (an unavailable component) is zero.
function _columns_match(actual, expected, rows, window)
    actual[!, "DateTime"] == window || return false
    expected[!, "DateTime"] == window || return false
    issubset(names(expected), names(actual)) || return false
    for name in names(actual)
        name == "DateTime" && continue
        reference = if name in names(expected)
            expected[rows, name]
        else
            zeros(count(rows))
        end
        isapprox(actual[rows, name], reference; atol = 1e-5) || return false
    end
    return true
end

@testset "An isolating outage mid-simulation matches static rebuilds per window" begin
    sys = _island_outage_system()
    model = DecisionModel(_island_template(), sys; name = "ED", optimizer = HiGHS_optimizer)
    models = SimulationModels(; decision_models = [model])
    sim = Simulation(;
        name = "isolating_outage",
        steps = _TVN_STEPS,
        models,
        sequence = SimulationSequence(; models),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results = get_decision_problem_results(SimulationResults(sim), "ED")
    dispatch = read_variable(results, ActivePowerVariable, ThermalStandard)
    flows = read_expression(results, POM.PTDFBranchFlow, Line)
    statuses = read_parameter(results, POM.BranchStatusParameter, Line)
    for step in 1:_TVN_STEPS
        window_start = _TVN_T0 + (step - 1) * _TVN_INTERVAL
        window = collect(window_start:Hour(1):(window_start + _TVN_HORIZON - Hour(1)))
        is_out = [t in _TVN_OUT for t in window]
        @test _status_values(statuses, window_start, "4") == Float64.(.!is_out)
        @test any(is_out)

        step_dispatch = _wide(dispatch[window_start])
        step_flows = _wide(flows[window_start])
        for out in (false, true)
            rows = is_out .== out
            reference = _solve_static_window(window_start, out)
            @test _columns_match(step_dispatch, reference.dispatch, rows, window)
            @test _columns_match(step_flows, reference.flows, rows, window)
        end
    end
end

#################################################################################
# Network caches across windows
#################################################################################

# Line "3" is out at hours 1-2, inside the first window only; line "6" at hours 14-15, inside
# the second and third windows only.
function _moving_outages_system()
    sys = _windowed_dispatch_system()
    timestamps = collect(_TVN_T0:Hour(1):(_TVN_T0 + Hour(47)))
    for (name, hours) in (("3", 1:2), ("6", 14:15))
        _add_planned_outage!(sys, get_component(Line, sys, name), "out_$name", timestamps,
            Set(_TVN_T0 + Hour(h) for h in hours))
    end
    PSY.transform_single_time_series!(sys, _TVN_HORIZON, _TVN_INTERVAL)
    return sys
end

@testset "The topology caches hold only the last window's networks" begin
    model = DecisionModel(
        _island_template(),
        _moving_outages_system();
        name = "ED",
        optimizer = HiGHS_optimizer,
    )
    models = SimulationModels(; decision_models = [model])
    sim = Simulation(;
        name = "moving_outages",
        steps = _TVN_STEPS,
        models,
        sequence = SimulationSequence(; models),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED

    network_model = IOM.get_network_model(IOM.get_template(model))
    matrix = POM.get_topology_matrix(network_model)
    ts = POM.get_topology_schedule(network_model)
    time_steps = IOM.get_time_steps(IOM.get_optimization_container(model))
    in_use = Set(
        PNM.get_network_modification(ts, t) for
        t in time_steps if PNM.get_topology_index(ts, t) != 0
    )
    @test length(in_use) == 1
    base = PNM.NetworkModification("", PNM.ArcModification[])
    @test Set(keys(PNM.get_row_caches(matrix))) == union(in_use, [base])
    @test issubset(keys(PNM.get_woodbury_cache(matrix)), in_use)
end

#################################################################################
# Two stages, two resolutions
#################################################################################

# The UC takes its line out at hours 6-8; the ED takes its line out at its own 5-minute steps,
# which the UC series does not: steps 2-4 of its first window and 1-2 of its second.
_uc_pattern(k, h) = (k == 1 && h in 7:9) ? 1.0 : 0.0
_ed_pattern(k, h) = ((k == 1 && h in 2:4) || (k == 2 && h in 1:2)) ? 1.0 : 0.0

# An hourly UC and a 5-minute ED, each scheduling its own line with its own series; returns
# the executed simulation and its execution status.
function _run_two_resolutions(uc_line, ed_line)
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_ed")
    _add_forecast_outage!(sys_uc, get_component(Line, sys_uc, uc_line), "uc_maint",
        _uc_pattern)
    _add_forecast_outage!(sys_ed, get_component(Line, sys_ed, ed_line), "ed_maint",
        _ed_pattern)

    template_uc = get_template_basic_uc_simulation()
    set_network_model!(template_uc, NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(template_uc, Line, StaticBranch)
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(template_ed, Line, StaticBranch)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(template_uc, sys_uc; name = "UC", optimizer = HiGHS_optimizer,
                initialize_model = false),
            DecisionModel(template_ed, sys_ed; name = "ED", optimizer = HiGHS_optimizer,
                initialize_model = false),
        ],
    )
    sequence = SimulationSequence(;
        models,
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
        name = "two_resolutions",
        steps = 1,
        models,
        sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    return sim, execute!(sim; enable_progress_bar = false)
end

function _read_line_statuses(sim, model_name)
    results = get_decision_problem_results(SimulationResults(sim), model_name)
    return read_parameter(results, POM.BranchStatusParameter, Line)
end

@testset "A day-ahead UC and a 5-minute ED each follow their own schedule series" begin
    sim, status = _run_two_resolutions("1", "1")
    @test status == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    uc_statuses = _read_line_statuses(sim, "UC")
    @test _status_values(uc_statuses, _TVN_T0, "1") ==
          [1.0 - _uc_pattern(1, h) for h in 1:24]
    ed_statuses = _read_line_statuses(sim, "ED")
    @test length(ed_statuses) == 24
    for k in 1:3
        window_start = _TVN_T0 + (k - 1) * Hour(1)
        @test _status_values(ed_statuses, window_start, "1") ==
              [1.0 - _ed_pattern(k, h) for h in 1:12]
    end
end

@testset "Stages that schedule different components run and report their own statuses" begin
    sim, status = _run_two_resolutions("1", "2")
    @test status == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    uc_statuses = _read_line_statuses(sim, "UC")
    @test unique(uc_statuses[_TVN_T0][!, :name]) == ["1"]
    @test _status_values(uc_statuses, _TVN_T0, "1") ==
          [1.0 - _uc_pattern(1, h) for h in 1:24]
    ed_statuses = _read_line_statuses(sim, "ED")
    for k in 1:3
        window_start = _TVN_T0 + (k - 1) * Hour(1)
        @test unique(ed_statuses[window_start][!, :name]) == ["2"]
        @test _status_values(ed_statuses, window_start, "2") ==
              [1.0 - _ed_pattern(k, h) for h in 1:12]
    end
end

#################################################################################
# Planned outage plus forced-outage event
#################################################################################

const _PLANNED_STEPS = 4:6
const _EVENT_STATUS = [h == 2 ? 1.0 : 0.0 for h in 1:48]

function _add_forced_outage!(sys, timestamps, component)
    outage = PSY.FixedForcedOutage(; outage_status = 0.0)
    add_supplemental_attribute!(sys, component, outage)
    add_time_series!(
        sys,
        outage,
        SingleTimeSeries("outage_profile", TimeArray(timestamps, _EVENT_STATUS)),
    )
    return outage
end

# With `forced`, "Alta" (or the unit `forced_units` names for a system) carries a
# `FixedForcedOutage` series at hour 2 in every system and the simulation runs an event model
# for it. `planned` (a component type and name) carries a
# `PlannedOutage` at `planned_steps` (hours) in the systems named in `planned_on`, added
# before the forced outage in the systems named in `planned_first`. A 2-day UC "D1", a
# rolling 4-hour UC "D2" and an hourly emulator "EM" run one day. The event runtime resets
# the outaged unit's decision state, so an hourly decision model re-solves between the
# day-ahead commitment and the emulator, as in the event tests. Returns the simulation and
# its folder.
function _planned_and_forced_simulation(
    planned_on;
    planned = (ThermalStandard, "Alta"),
    planned_steps = _PLANNED_STEPS,
    forced = true,
    forced_units = Dict{String, String}(),
    planned_first = (),
)
    timestamps = collect(_TVN_T0:Hour(1):(_TVN_T0 + Hour(47)))
    systems = Dict(
        name => PSB.build_system(PSITestSystems, "c_sys5_events") for
        name in ("D1", "D2", "EM")
    )
    for (name, sys) in systems
        add_planned! =
            if name in planned_on
                component = get_component(first(planned), sys, last(planned))
                () -> _add_planned_outage!(sys, component, "maint", timestamps,
                    timestamps[planned_steps])
            else
                () -> nothing
            end
        forced_unit = get(forced_units, name, "Alta")
        add_forced! =
            if forced
                unit = get_component(ThermalStandard, sys, forced_unit)
                () -> _add_forced_outage!(sys, timestamps, unit)
            else
                () -> nothing
            end
        if name in planned_first
            add_planned!()
            add_forced!()
        else
            add_forced!()
            add_planned!()
        end
    end
    PSY.transform_single_time_series!(systems["D1"], Day(2), Day(1))
    PSY.transform_single_time_series!(systems["D2"], Hour(4), Hour(1))

    template_em = get_template_nomin_ed_simulation()
    set_device_model!(template_em, ThermalStandard, ThermalBasicDispatch)
    event_model = EventModel(
        FixedForcedOutage,
        POM.ContinuousCondition();
        timeseries_mapping = Dict(:outage_status => "outage_profile"),
    )
    models = SimulationModels(;
        decision_models = [
            DecisionModel(get_template_basic_uc_simulation(), systems[name];
                name = name,
                optimizer = HiGHS_optimizer_small_gap, initialize_model = false)
            for name in ("D1", "D2")
        ],
        emulation_model = EmulationModel(template_em, systems["EM"]; name = "EM",
            optimizer = HiGHS_optimizer_small_gap),
    )
    sequence = SimulationSequence(;
        models,
        ini_cond_chronology = InterProblemChronology(),
        feedforwards = Dict(
            "EM" => [
                SemiContinuousFeedforward(;
                    component_type = ThermalStandard,
                    source = OnVariable,
                    affected_values = [ActivePowerVariable],
                ),
            ],
        ),
        events = forced ? [event_model] : POM.EventModel[],
    )
    folder = mktempdir(; cleanup = true)
    sim = Simulation(;
        name = "planned_and_forced",
        steps = 1,
        models,
        sequence,
        simulation_folder = folder,
    )
    return sim, folder
end

# Builds and runs `_planned_and_forced_simulation(planned_on; kwargs...)` on the in-memory
# store, or on the HDF store when `in_memory = false`. Returns the executed simulation, its
# execution status and its folder.
function _run_planned_and_forced(planned_on; in_memory = true, kwargs...)
    sim, folder = _planned_and_forced_simulation(planned_on; kwargs...)
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    status = execute!(sim; in_memory = in_memory, enable_progress_bar = false)
    return sim, status, folder
end

_realized(em, key, name) =
    read_realized_variable(em, key; table_format = TableFormat.WIDE)[!, name]
_realized_alta(em, key) = _realized(em, key, "Alta")

@testset "A planned outage and a forced-outage event on one unit both take it out" begin
    sim, status, _ = _run_planned_and_forced(("D1", "D2", "EM"))
    @test status == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    for name in (:D1, :D2, :EM)
        container = PSI.get_optimization_container(PSI.get_simulation_model(sim, name))
        outage_key =
            IOM.ConstraintKey(POM.ActivePowerOutageConstraint, ThermalStandard, "ub")
        @test "Alta" in axes(IOM.get_constraint(container, outage_key), 1)
    end
    day_ahead = PSI.get_optimization_container(PSI.get_simulation_model(sim, :D1))
    power = IOM.get_variable(day_ahead, ActivePowerVariable, ThermalStandard)
    on = IOM.get_variable(day_ahead, OnVariable, ThermalStandard)
    fixed_at_zero(v) = JuMP.is_fixed(v) && JuMP.fix_value(v) == 0.0
    for t in axes(power, 2)
        @test fixed_at_zero(power["Alta", t]) == (t in _PLANNED_STEPS)
        @test fixed_at_zero(on["Alta", t]) == (t in _PLANNED_STEPS)
    end

    em = get_emulation_problem_results(SimulationResults(sim))
    @test _realized_alta(em, "AvailableStatusParameter__ThermalStandard") ==
          1.0 .- _EVENT_STATUS[1:24]
    @test _realized_alta(em, "InjectorStatusParameter__ThermalStandard") ==
          [h in _PLANNED_STEPS ? 0.0 : 1.0 for h in 1:24]
    @test all(
        iszero,
        _realized_alta(em, "ActivePowerVariable__ThermalStandard")[[2, 4, 5, 6]],
    )
end

# Supplemental-attribute ids are assigned per system, so with the attributes added in a
# different order the emulator's forced outage has another id than the decision models'.
@testset "The event finds its outage in the emulator whatever the attribute order" begin
    sim, status, _ = _run_planned_and_forced(("D1", "D2", "EM"); planned_first = ("EM",))
    @test status == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    em_sys = PSI.get_system(PSI.get_simulation_model(sim, :EM))
    (attribute, devices) = only(last(only(PSI.get_event_attributes(sim))))
    @test attribute === only(get_supplemental_attributes(FixedForcedOutage, em_sys))
    @test devices == Dict(ThermalStandard => Set(["Alta"]))
    em = get_emulation_problem_results(SimulationResults(sim))
    @test _realized_alta(em, "AvailableStatusParameter__ThermalStandard") ==
          1.0 .- _EVENT_STATUS[1:24]
    @test all(
        iszero,
        _realized_alta(em, "ActivePowerVariable__ThermalStandard")[[2, 4, 5, 6]],
    )
end

@testset "An outage missing from the emulator's system fails the build" begin
    sim, _ = _planned_and_forced_simulation(
        ("D1", "D2", "EM");
        forced_units = Dict("EM" => "Park City"),
    )
    err = try
        build!(sim; console_level = Logging.Error)
    catch e
        e
    end
    @test err isa IS.ConflictingInputsError
    @test occursin(
        "emulation model EM has no FixedForcedOutage on ThermalStandard \"Alta\"",
        err.msg)
end

# A `FixedForcedOutage` on a branch is contingency data, which the event model ignores, so
# the decision systems may carry it, twice on one line, without the emulator's system.
@testset "Branch contingency data stays out of the event checks" begin
    sim, _ = _planned_and_forced_simulation(("D1", "D2", "EM"))
    timestamps = collect(_TVN_T0:Hour(1):(_TVN_T0 + Hour(47)))
    for name in (:D1, :D2)
        sys = PSI.get_system(PSI.get_simulation_model(sim, name))
        line = get_component(Line, sys, "1")
        _add_forced_outage!(sys, timestamps, line)
        _add_forced_outage!(sys, timestamps, line)
    end
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
end

# A type scheduled only in the emulator's system has no status container in the decision
# models and no status key in the decision states; the emulator writes it to the system state.
@testset "Schedules only in the emulator's system reach the emulator's outputs" begin
    sim, status, _ =
        _run_planned_and_forced(("EM",); planned = (RenewableDispatch, "WindBus1"))
    @test status == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    for name in (:D1, :D2)
        container = PSI.get_optimization_container(PSI.get_simulation_model(sim, name))
        @test !IOM.has_container_key(
            container,
            POM.InjectorStatusParameter,
            RenewableDispatch,
        )
    end
    status_key = IOM.ParameterKey{POM.InjectorStatusParameter, RenewableDispatch}("")
    state = PSI.get_simulation_state(sim)
    @test !PSI.has_dataset(PSI.get_decision_states(state), status_key)
    @test PSI.has_dataset(PSI.get_system_states(state), status_key)
    em = get_emulation_problem_results(SimulationResults(sim))
    @test _realized(em, "InjectorStatusParameter__RenewableDispatch", "WindBus1") ==
          [h in _PLANNED_STEPS ? 0.0 : 1.0 for h in 1:24]
    @test all(
        iszero,
        _realized(em, "ActivePowerVariable__RenewableDispatch", "WindBus1")[_PLANNED_STEPS],
    )
end

# The day-ahead models commit "Alta" at hours 7-8, with "Park City" online to pick up its
# output; only the emulator's system schedules the outage there. The semi-continuous
# feedforward's lower bound would hold Alta at its minimum unless it follows the emulator's
# status. The simulation runs on the HDF store, which holds the emulator's statuses apart
# from the simulation state.
@testset "An emulator-only outage on a committed thermal unit takes it out" begin
    out_steps = 7:8
    sim, status, _ = _run_planned_and_forced(
        ("EM",);
        planned_steps = out_steps,
        forced = false,
        in_memory = false,
    )
    @test status == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    results = SimulationResults(sim)
    rolling = get_decision_problem_results(results, "D2")
    on = read_realized_variable(rolling, OnVariable, ThermalStandard;
        table_format = TableFormat.WIDE)
    @test all(isone, on[out_steps, "Alta"])
    em = get_emulation_problem_results(results)
    @test _realized_alta(em, "InjectorStatusParameter__ThermalStandard") ==
          [h in out_steps ? 0.0 : 1.0 for h in 1:24]
    @test all(iszero, _realized_alta(em, "ActivePowerVariable__ThermalStandard")[out_steps])
end
