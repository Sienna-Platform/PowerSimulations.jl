@testset "Simulation Build Tests" begin
    models = create_simulation_build_test_problems(get_template_basic_uc_simulation())
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
        name = "test",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )

    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT

    for field in fieldnames(SimulationSequence)
        if fieldtype(SimulationSequence, field) == Union{Dates.DateTime, Nothing}
            @test !isnothing(getfield(sim.sequence, field))
        end
    end

    @test length(findall(x -> x == 2, sequence.execution_order)) == 24
    @test length(findall(x -> x == 1, sequence.execution_order)) == 1

    state = PSI.get_simulation_state(sim)

    uc_vars = [OnVariable, StartVariable, StopVariable]
    ed_vars = [ActivePowerVariable]
    for (key, data) in state.decision_states.variables
        if PSI.get_entry_type(key) ∈ uc_vars
            _, count = size(data.values)
            @test count == 24
        elseif PSI.get_entry_type(key) ∈ ed_vars
            _, count = size(data.values)
            @test count == 288
        end
    end
end

@testset "Simulation with provided initial time" begin
    models = create_simulation_build_test_problems(get_template_basic_uc_simulation())
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
    second_day = DateTime("1/1/2024  23:00:00", "d/m/y  H:M:S") + Hour(1)
    sim = Simulation(;
        name = "test",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
        initial_time = second_day,
    )
    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT

    for model in PSI.get_decision_models(PSI.get_models(sim))
        @test PSI.get_initial_time(model) == second_day
    end

    for field in fieldnames(SimulationSequence)
        if fieldtype(SimulationSequence, field) == Union{Dates.DateTime, Nothing}
            @test !isnothing(getfield(sim.sequence, field))
        end
    end

    @test length(findall(x -> x == 2, sequence.execution_order)) == 24
    @test length(findall(x -> x == 1, sequence.execution_order)) == 1
end

@testset "Negative Tests (Bad Parametrization)" begin
    models = create_simulation_build_test_problems(get_template_basic_uc_simulation())
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

    @test_throws UndefKeywordError sim = Simulation(; name = "test", steps = 1)

    sim = Simulation(;
        name = "test",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
        initial_time = Dates.now(),
    )

    @test_throws IS.ConflictingInputsError build!(
        sim,
        console_level = Logging.AboveMaxLevel,
    )

    sim = Simulation(;
        name = "fake_path",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = "fake_path",
    )

    @test_throws IS.ConflictingInputsError PSI._check_folder(sim)
end

@testset "Test SemiContinuous Feedforward with Active and Reactive Power variables" begin
    template_uc = get_template_basic_uc_simulation()
    set_device_model!(template_uc, Line, StaticBranchUnbounded)
    set_network_model!(template_uc, NetworkModel(DCPNetworkModel; use_slacks = true))
    # network slacks added because of data issues
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(ACPNetworkModel; use_slacks = true))
    set_device_model!(template_ed, Line, StaticBranchUnbounded)
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                c_sys5_hy_uc;
                name = "UC",
                optimizer = HiGHS_optimizer,
                initialize_model = false,
            ),
            DecisionModel(
                template_ed,
                c_sys5_hy_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
                initialize_model = false,
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
                    affected_values = [ActivePowerVariable, ReactivePowerVariable],
                ),
            ],
        ),
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = "reactive_feedforward",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    ac_power_model = PSI.get_simulation_model(PSI.get_models(sim), :ED)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ac_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ActivePowerVariable_ub",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ac_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ActivePowerVariable_lb",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ac_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ReactivePowerVariable_ub",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ac_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ReactivePowerVariable_lb",
    )
    @test !isempty(c)
end

@testset "Test Upper/Lower Bound Feedforwards" begin
    template_uc = get_template_basic_uc_simulation()
    set_network_model!(template_uc, NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(template_uc, DeviceModel(Line, StaticBranchBounds))
    template_ed =
        get_template_nomin_ed_simulation(NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(template_ed, DeviceModel(Line, StaticBranchBounds))
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                c_sys5_hy_uc;
                name = "UC",
                optimizer = HiGHS_optimizer,
                initialize_model = false,
            ),
            DecisionModel(
                template_ed,
                c_sys5_hy_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
                initialize_model = false,
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
                LowerBoundFeedforward(;
                    component_type = Line,
                    source = FlowActivePowerVariable,
                    affected_values = [FlowActivePowerVariable],
                    add_slacks = true,
                ),
                UpperBoundFeedforward(;
                    component_type = Line,
                    source = FlowActivePowerVariable,
                    affected_values = [FlowActivePowerVariable],
                    add_slacks = true,
                ),
            ],
        ),
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = "reactive_feedforward",
        steps = 2,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    ed_power_model = PSI.get_simulation_model(PSI.get_models(sim), :ED)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ed_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ActivePowerVariable_ub",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ed_power_model),
        FeedforwardSemiContinuousConstraint(),
        ThermalStandard,
        "ActivePowerVariable_lb",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ed_power_model),
        FeedforwardLowerBoundConstraint(),
        Line,
        "FlowActivePowerVariablelb",
    )
    @test !isempty(c)
    c = PSI.get_constraint(
        PSI.get_optimization_container(ed_power_model),
        FeedforwardUpperBoundConstraint(),
        Line,
        "FlowActivePowerVariableub",
    )
    @test !isempty(c)
    c = PSI.get_variable(
        PSI.get_optimization_container(ed_power_model),
        UpperBoundFeedForwardSlack,
        Line,
        "FlowActivePowerVariable",
    )
    @test !isempty(c)
    c = PSI.get_variable(
        PSI.get_optimization_container(ed_power_model),
        LowerBoundFeedForwardSlack,
        Line,
        "FlowActivePowerVariable",
    )
    @test !isempty(c)
end

# No FixValueFeedforward-on-ServiceModel test: POM's ServiceModel keys reserve variables by
# (service_name, device_name, time), while the feedforward parameter path is keyed
# (device_name, time), so `attach_feedforward!(::ServiceModel, ff)` errors loudly. Re-add
# once POM re-keys that path.

@testset "Build with store_systems_in_results option" begin
    models = create_simulation_build_test_problems(get_template_basic_uc_simulation())
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

    # Test store_systems_in_results = true
    sim_with = Simulation(;
        name = "test_with_systems",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    # SYSTEM-IN-RESULTS: storing the systems in the results is disabled until it is redesigned.
    @test_throws ErrorException build!(sim_with; store_systems_in_results = true)
    #=
    build_out = build!(sim_with; store_systems_in_results = true)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    PSI.open_store(PSI.HdfSimulationStore, PSI.get_store_dir(sim_with), "r") do store
        root = store.file["simulation"]
        @test haskey(root, "systems")
        @test length(keys(root["systems"])) > 0
    end
    =#

    # Test store_systems_in_results = false (default)
    models2 = create_simulation_build_test_problems(get_template_basic_uc_simulation())
    sequence2 = SimulationSequence(;
        models = models2,
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
    sim_without = Simulation(;
        name = "test_without_systems",
        steps = 1,
        models = models2,
        sequence = sequence2,
        simulation_folder = mktempdir(; cleanup = true),
    )
    build_out = build!(sim_without; store_systems_in_results = false)
    @test build_out == PSI.SimulationBuildStatus.BUILT
    PSI.open_store(PSI.HdfSimulationStore, PSI.get_store_dir(sim_without), "r") do store
        root = store.file["simulation"]
        @test !haskey(root, "systems")
    end
end

"""One bus, one thermal unit and one load with 24 h of hourly load data, transformed to a
single 24 h forecast window. InfrastructureSystems records the interval of a one-window
forecast as an empty period."""
function _single_window_system()
    sys = PSY.System(100.0)
    bus = PSY.ACBus(; number = 1, name = "bus1", available = true,
        bustype = PSY.ACBusTypes.REF, angle = 0.0, magnitude = 1.0,
        voltage_limits = (min = 0.9, max = 1.1), base_voltage = 230.0,
        input_basis = u"CU")
    PSY.add_component!(sys, bus)
    PSY.add_component!(
        sys,
        PSY.ThermalStandard(; name = "gen1", available = true,
            status = PSY.OperationalStates.ONLINE, bus, active_power = 1.0,
            reactive_power = 0.0,
            rating = 2.0, active_power_limits = (min = 0.0, max = 2.0),
            reactive_power_limits = nothing, ramp_limits = nothing,
            operation_cost = PSY.ThermalGenerationCost(;
                variable_operation_cost = PSY.CostCurve(PSY.LinearCurve(20.0)), fixed = 0.0,
                start_up = 0.0, shut_down = 0.0),
            base_power = 100.0, prime_mover_type = PSY.PrimeMovers.CT,
            fuel = PSY.ThermalFuels.NATURAL_GAS, input_basis = u"CU"),
    )
    load = PSY.PowerLoad(; name = "load1", available = true, bus, active_power = 1.0,
        reactive_power = 0.0, base_power = 100.0, max_active_power = 1.0,
        max_reactive_power = 0.0, input_basis = u"CU")
    PSY.add_component!(sys, load)
    timestamps =
        collect(range(DateTime("2024-01-01T00:00:00"); step = Hour(1), length = 24))
    profile = [0.6 + 0.4 * sin(pi * h / 24) for h in 0:23]
    PSY.add_time_series!(sys, load,
        IS.SingleTimeSeries("max_active_power", TimeSeries.TimeArray(timestamps, profile);
            PSB.component_base("active_power")...))
    PSY.transform_single_time_series!(sys, Hour(24), Hour(24))
    return sys
end

function _single_window_simulation(; steps = 1, kwargs...)
    template = POM.PowerOperationsProblemTemplate(NetworkModel(CopperPlateNetworkModel))
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    model = DecisionModel(template, _single_window_system(); name = "UC",
        optimizer = HiGHS_optimizer, kwargs...)
    models = SimulationModels(; decision_models = [model])
    sequence = SimulationSequence(; models)
    return Simulation(; name = "single_window", steps, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
end

@testset "Simulation from a single forecast window" begin
    # A one-step simulation of a one-day model needs only the one window the system has.
    sim = _single_window_simulation()
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    # A second step would need a second window; say so rather than fail on the interval.
    sim = _single_window_simulation(; steps = 2)
    @test_throws IS.ConflictingInputsError build!(sim)
end

@testset "Explicit model interval on a single forecast window" begin
    # The system has no forecast at a 24 h interval, so the request cannot be honored.
    @test_throws IS.ConflictingInputsError _single_window_simulation(; interval = Hour(24))
end

function _emulator_resolution_simulation(; decision_system, emulator_system)
    sys_dm = PSB.build_system(PSITestSystems, decision_system)
    sys_em =
        PSB.build_system(PSITestSystems, emulator_system; add_single_time_series = true)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(get_template_nomin_ed_simulation(), sys_dm; name = "DM",
                optimizer = HiGHS_optimizer),
        ],
        emulation_model = EmulationModel(get_template_nomin_ed_simulation(), sys_em;
            name = "EM", optimizer = HiGHS_optimizer),
    )
    sequence = SimulationSequence(; models = models,
        ini_cond_chronology = InterProblemChronology())
    return Simulation(; name = "em_resolution", steps = 2, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
end

@testset "Emulator coarser than a decision model is rejected at build" begin
    # A 5-minute decision model with an hourly emulator.
    sim = _emulator_resolution_simulation(;
        decision_system = "c_sys5_ed", emulator_system = "c_sys5_uc")
    err = try
        build!(sim; console_level = Logging.AboveMaxLevel)
        nothing
    catch e
        e
    end
    @test err isa IS.ConflictingInputsError
    msg = sprint(showerror, err)
    @test occursin("EM", msg)
    @test occursin("1 hour", msg)
    @test occursin("DM (5 minutes)", msg)
end

@testset "Emulator at equal or finer resolution than decision models builds" begin
    # Equal: 5-minute decision model and 5-minute emulator.
    sim = _emulator_resolution_simulation(;
        decision_system = "c_sys5_ed", emulator_system = "c_sys5_ed")
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    # Finer: hourly decision model and 5-minute emulator.
    sim = _emulator_resolution_simulation(;
        decision_system = "c_sys5_uc", emulator_system = "c_sys5_ed")
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
end

_status_feedforward() = SemiContinuousFeedforward(;
    component_type = ThermalStandard,
    source = OnVariable,
    affected_values = [ActivePowerVariable],
)

_power_bound_feedforward(; source = ActivePowerVariable) = UpperBoundFeedforward(;
    component_type = ThermalStandard,
    source,
    affected_values = [ActivePowerVariable],
    add_slacks = true,
)

# A UC and a 5-minute ED over the 5-bus systems; `uc_without` names a unit the UC does not
# model, `ed_without` a unit removed from the ED's system, `ed_thermal` is the ED's thermal
# formulation and `feedforward` the ED's thermal feedforward, if any.
function _state_source_simulation(;
    uc_without = nothing,
    ed_without = nothing,
    ed_thermal = ThermalDispatchNoMin,
    feedforward = _status_feedforward(),
    uc_must_run = nothing,
    ed_must_run = nothing,
    allow_different_components = false,
    steps = 1,
)
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_ed")
    isnothing(uc_without) ||
        set_available!(get_component(ThermalStandard, sys_uc, uc_without), false)
    isnothing(ed_without) ||
        remove_component!(sys_ed, get_component(ThermalStandard, sys_ed, ed_without))
    for (sys, must_run) in ((sys_uc, uc_must_run), (sys_ed, ed_must_run))
        isnothing(must_run) ||
            set_commitment_mode!(get_component(ThermalStandard, sys, must_run),
                CommitmentModes.MUST_RUN)
    end
    template_ed = get_template_nomin_ed_simulation()
    set_device_model!(template_ed, ThermalStandard, ed_thermal)
    models = create_simulation_build_test_problems(
        get_template_basic_uc_simulation(), template_ed, sys_uc, sys_ed)
    if isnothing(feedforward)
        feedforwards = Dict()
    else
        feedforwards = Dict("ED" => [feedforward])
    end
    sequence = SimulationSequence(;
        models,
        feedforwards,
        ini_cond_chronology = InterProblemChronology(),
        allow_different_components,
    )
    return Simulation(; name = "state_sources", steps, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
end

@testset "Models with different units fail at build by default" begin
    @test build!(_state_source_simulation(); console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    err = _build_error(_state_source_simulation(; uc_without = "Alta"))
    @test err isa IS.ConflictingInputsError
    msg = sprint(showerror, err)
    @test occursin("UC", msg) && occursin("ED", msg) && occursin("Alta", msg)
    @test occursin("ActivePowerVariable__ThermalStandard", msg)
    @test occursin("allow_different_components = true", msg)
end

@testset "An emulator with different units fails at build by default" begin
    sys_dm = PSB.build_system(PSITestSystems, "c_sys5_ed")
    sys_em = PSB.build_system(PSITestSystems, "c_sys5_ed"; add_single_time_series = true)
    set_available!(get_component(ThermalStandard, sys_em, "Alta"), false)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(get_template_nomin_ed_simulation(), sys_dm; name = "DM",
                optimizer = HiGHS_optimizer),
        ],
        emulation_model = EmulationModel(get_template_nomin_ed_simulation(), sys_em;
            name = "EM", optimizer = HiGHS_optimizer),
    )
    sequence = SimulationSequence(; models, ini_cond_chronology = InterProblemChronology())
    sim = Simulation(; name = "em_components", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
    err = _build_error(sim)
    @test err isa IS.ConflictingInputsError
    msg = sprint(showerror, err)
    @test occursin("EM", msg) && occursin("DM", msg) && occursin("Alta", msg)
    @test occursin("allow_different_components = true", msg)
end

@testset "Must-run units do not trip the default check" begin
    # Both models commit units, so each leaves the must-run Alta out of its OnVariable.
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_ed")
    for sys in (sys_uc, sys_ed)
        set_commitment_mode!(get_component(ThermalStandard, sys, "Alta"),
            CommitmentModes.MUST_RUN)
    end
    template_ed = get_template_nomin_ed_simulation()
    set_device_model!(template_ed, ThermalStandard, ThermalBasicUnitCommitment)
    models = create_simulation_build_test_problems(
        get_template_basic_uc_simulation(), template_ed, sys_uc, sys_ed)
    sequence = SimulationSequence(; models, ini_cond_chronology = InterProblemChronology())
    sim = Simulation(; name = "must_run", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    on_key = PSI.VariableKey(OnVariable, ThermalStandard)
    for model in PSI.get_decision_models(PSI.get_models(sim))
        @test !("Alta" in first(PSI._model_columns(model, on_key)))
    end
end

@testset "Models with different units build when allowed" begin
    sim = _state_source_simulation(;
        uc_without = "Alta",
        feedforward = nothing,
        allow_different_components = true,
    )
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
end

# A "dynamic_line_rating" series on `line` at its static rating, over the windows of `sys`'s
# load forecast.
function _add_line_rating_series!(sys::PSY.System, line::Line)
    load = first(get_components(PowerLoad, sys))
    load_ts = get_time_series(Deterministic, load, "max_active_power")
    rating = PSY.get_rating(line, u"MVA")
    data = DataStructures.SortedDict(
        dt => fill(rating, length(values)) for (dt, values) in get_data(load_ts)
    )
    add_time_series!(sys, line,
        Deterministic("dynamic_line_rating", data, IS.get_resolution(load_ts);
            PSB.natural_units("apparent_power", "MVA")...))
    return
end

function _line_rating_template()
    template =
        get_template_nomin_ed_simulation(NetworkModel(PTDFNetworkModel; use_slacks = true))
    set_device_model!(
        template,
        DeviceModel(Line, StaticBranch;
            time_series_names = Dict(
                BranchRatingTimeSeriesParameter => "dynamic_line_rating",
            )),
    )
    return template
end

function _parameter_columns(sim, model_name::Symbol, key)
    model = PSI.get_simulation_model(sim, model_name)
    return first(PSI._model_columns(model, key))
end

@testset "Models with rating series on different lines build by default" begin
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_ed")
    line_uc, line_ed = first(sort!(get_name.(get_components(Line, sys_uc))), 2)
    _add_line_rating_series!(sys_uc, get_component(Line, sys_uc, line_uc))
    _add_line_rating_series!(sys_ed, get_component(Line, sys_ed, line_ed))
    models = create_simulation_build_test_problems(
        _line_rating_template(), _line_rating_template(), sys_uc, sys_ed)
    sequence = SimulationSequence(; models, ini_cond_chronology = InterProblemChronology())
    sim = Simulation(; name = "rating_series", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    key = PSI.ParameterKey(BranchRatingTimeSeriesParameter, Line)
    @test _parameter_columns(sim, :UC, key) == [line_uc]
    @test _parameter_columns(sim, :ED, key) == [line_ed]
    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "Models with fuel cost series on different units build by default" begin
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_ed")
    # Both units burn fuel in both models; each model reads a fuel cost series for one.
    for sys in (sys_uc, sys_ed), name in ("Alta", "Brighton")
        unit = get_component(ThermalStandard, sys, name)
        cost = get_operation_cost(unit)
        set_operation_cost!(unit,
            ThermalGenerationCost(;
                variable_operation_cost = FuelCurve(LinearCurve(0.0, 5.0), PSY.MMBtu, 3.0),
                fixed = get_fixed(cost),
                start_up = get_start_up(cost),
                shut_down = get_shut_down(cost),
            ))
    end
    add_fuel_cost_time_series!(sys_uc, get_component(ThermalStandard, sys_uc, "Alta"))
    add_fuel_cost_time_series!(sys_ed, get_component(ThermalStandard, sys_ed, "Brighton"))
    models = create_simulation_build_test_problems(
        get_template_basic_uc_simulation(), get_template_nomin_ed_simulation(),
        sys_uc, sys_ed)
    sequence = SimulationSequence(; models, ini_cond_chronology = InterProblemChronology())
    sim = Simulation(; name = "fuel_cost_series", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    key = PSI.ParameterKey(FuelCostParameter, ThermalStandard)
    @test _parameter_columns(sim, :UC, key) == ["Alta"]
    @test _parameter_columns(sim, :ED, key) == ["Brighton"]
    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

# A "dynamic_line_rating" SingleTimeSeries on `line` at its static rating, spanning `sys`'s
# load series.
function _add_single_line_rating_series!(sys::PSY.System, line::Line)
    load = first(get_components(PowerLoad, sys))
    stamps = TimeSeries.timestamp(
        get_data(get_time_series(SingleTimeSeries, load, "max_active_power")))
    ratings = fill(PSY.get_rating(line, u"MVA"), length(stamps))
    add_time_series!(sys, line,
        SingleTimeSeries("dynamic_line_rating", TimeSeries.TimeArray(stamps, ratings);
            PSB.natural_units("apparent_power", "MVA")...))
    return
end

@testset "An emulator with rating series on different lines builds when allowed" begin
    sys_em = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    line_ed, line_em = first(sort!(get_name.(get_components(Line, sys_ed))), 2)
    _add_single_line_rating_series!(sys_em, get_component(Line, sys_em, line_em))
    _add_single_line_rating_series!(sys_ed, get_component(Line, sys_ed, line_ed))
    PSY.transform_single_time_series!(sys_ed, Hour(1), Hour(1))
    models = SimulationModels(;
        decision_models = [
            DecisionModel(_line_rating_template(), sys_ed; name = "ED",
                optimizer = HiGHS_optimizer),
        ],
        emulation_model = EmulationModel(_line_rating_template(), sys_em; name = "EM",
            optimizer = HiGHS_optimizer),
    )
    sequence = SimulationSequence(;
        models,
        ini_cond_chronology = InterProblemChronology(),
        allow_different_components = true,
    )
    sim = Simulation(; name = "em_rating_series", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    key = PSI.ParameterKey(BranchRatingTimeSeriesParameter, Line)
    @test _parameter_columns(sim, :ED, key) == [line_ed]
    @test _parameter_columns(sim, :EM, key) == [line_em]
    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "Models with different units compare initial conditions on the units they share" begin
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_ed")
    # The ED does not model Brighton, and starts Park City 50 MW above where the UC does.
    set_available!(get_component(ThermalStandard, sys_ed, "Brighton"), false)
    park_city = get_component(ThermalStandard, sys_ed, "Park City")
    set_active_power!(park_city, (get_active_power(park_city, u"SU") + 0.5) * u"SU")
    # Ramp limits give the ED a DevicePower initial condition to compare with the UC's.
    # A third model without Park City must not hide the mismatch between the other two.
    sys_ed2 = PSB.build_system(PSITestSystems, "c_sys5_ed")
    set_available!(get_component(ThermalStandard, sys_ed2, "Park City"), false)
    template_ed = get_template_nomin_ed_simulation()
    set_device_model!(template_ed, ThermalStandard, ThermalStandardDispatch)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(get_template_standard_uc_simulation(), sys_uc;
                name = "UC", optimizer = HiGHS_optimizer),
            DecisionModel(template_ed, sys_ed; name = "ED", optimizer = HiGHS_optimizer),
            DecisionModel(deepcopy(template_ed), sys_ed2;
                name = "ED2", optimizer = HiGHS_optimizer),
        ],
    )
    sequence = SimulationSequence(;
        models,
        ini_cond_chronology = InterProblemChronology(),
        allow_different_components = true,
    )
    sim = Simulation(; name = "ic_shared_units", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT

    reconcile() =
        PSI._initial_conditions_reconciliation!(
            PSI.get_decision_models(PSI.get_models(sim)),
        )
    ics =
        @test_logs (:info, r"mismatch on component Park City") match_mode = :any reconcile()
    power = ics[IOM.InitialConditionKey(POM.DevicePower, ThermalStandard)]
    @test "Brighton" in keys(power[1]) && !("Brighton" in keys(power[2]))
    @test !("Park City" in keys(power[3]))
end

@testset "An initial power without a status source fails at build unless the unit is must-run" begin
    # The ED gives every unit an initial power, and only the UC commits units.
    dispatch = (;
        ed_thermal = ThermalStandardDispatch,
        feedforward = nothing,
        allow_different_components = true,
    )
    @test build!(_state_source_simulation(; dispatch...);
        console_level = Logging.AboveMaxLevel) == PSI.SimulationBuildStatus.BUILT
    err = _build_error(_state_source_simulation(; uc_without = "Alta", dispatch...))
    @test err isa IS.ConflictingInputsError
    msg = sprint(showerror, err)
    @test occursin("ED", msg) && occursin("Alta", msg) && occursin("OnVariable", msg)
    @test occursin("must-run", msg)
    # A must-run unit's status is taken as on, so it needs no model to commit it.
    sim = _state_source_simulation(; uc_without = "Alta", ed_must_run = "Alta", dispatch...)
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
end

@testset "A semicontinuous feedforward needs a status source for every unit" begin
    # Both models hold Alta, but only the ED commits it: the UC runs it as must-run, so the
    # UC writes no status for it. A semicontinuous feedforward applies to every unit, so
    # allowing different components cannot help and the error does not suggest it.
    for allow_different_components in (false, true)
        err = _build_error(
            _state_source_simulation(; uc_must_run = "Alta", steps = 2,
                allow_different_components),
        )
        @test err isa IS.ConflictingInputsError
        msg = sprint(showerror, err)
        @test occursin("ED", msg) && occursin("Alta", msg)
        @test occursin("OnStatusParameter__ThermalStandard", msg)
        @test !occursin("allow_different_components", msg)
    end
end

@testset "A semicontinuous feedforward fails at build for a unit its source model lacks" begin
    for (ed_thermal, scheduled_power) in (
        (ThermalDispatchNoMin, ActivePowerVariable),
        (ThermalCompactDispatch, PowerAboveMinimumVariable),
    )
        feedforward = SemiContinuousFeedforward(;
            component_type = ThermalStandard,
            source = OnVariable,
            affected_values = [scheduled_power],
        )
        err = _build_error(
            _state_source_simulation(;
                uc_without = "Alta",
                ed_thermal,
                feedforward,
                allow_different_components = true,
            ),
        )
        @test err isa IS.ConflictingInputsError
        msg = sprint(showerror, err)
        @test occursin("ED", msg) && occursin("Alta", msg)
        @test occursin("OnStatusParameter__ThermalStandard", msg)
        @test !occursin("allow_different_components", msg)
    end
end

@testset "A bound feedforward with no source for a unit fails at build unless allowed" begin
    # The UC runs Alta as must-run, so it writes no status for it.
    feedforward = _power_bound_feedforward(; source = OnVariable)
    err = _build_error(
        _state_source_simulation(; uc_must_run = "Alta", feedforward, steps = 2),
    )
    @test err isa IS.ConflictingInputsError
    msg = sprint(showerror, err)
    @test occursin("ED", msg) && occursin("Alta", msg)
    @test occursin("UpperBoundValueParameter__ThermalStandard", msg)
    @test occursin("allow_different_components = true", msg)
    sim = _state_source_simulation(;
        uc_must_run = "Alta",
        feedforward,
        steps = 2,
        allow_different_components = true,
    )
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
end

_feedforward_constraint_rows(model, constraint = POM.FeedforwardSemiContinuousConstraint) =
    [
        axes(v)[1] for
        (k, v) in IOM.get_constraints(IOM.get_optimization_container(model)) if
        IOM.get_entry_type(k) === constraint
    ]

@testset "A feedforward leaves out the components its source model lacks" begin
    sim = _state_source_simulation(;
        uc_without = "Alta",
        feedforward = _power_bound_feedforward(),
        allow_different_components = true,
    )
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    # The build logs to the simulation's own logger, so the check is replayed here.
    @test_logs (:warn, r"leaves 1 components? out of the feedforward") min_level =
        Logging.Warn PSI._check_state_sources(sim)
    rows = _feedforward_constraint_rows(
        PSI.get_simulation_model(PSI.get_models(sim), :ED),
        POM.FeedforwardUpperBoundConstraint,
    )
    @test !isempty(rows)
    @test all(names -> !("Alta" in names) && "Park City" in names, rows)
end

# A new sequence and simulation over the models of `sim`, with the same feedforward.
function _resequenced_simulation(sim; allow_different_components)
    models = PSI.get_models(sim)
    sequence = SimulationSequence(;
        models,
        feedforwards = Dict("ED" => [_power_bound_feedforward()]),
        ini_cond_chronology = InterProblemChronology(),
        allow_different_components,
    )
    return Simulation(; name = "resequenced", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
end

_ed_feedforwards(sim) =
    IOM.get_feedforwards(
        POM.get_device_models(
            PSI.get_template(PSI.get_simulation_model(PSI.get_models(sim), :ED)),
        )[:ThermalStandard],
    )

@testset "A feedforward restriction is decided at every build" begin
    sim = _state_source_simulation(;
        uc_without = "Alta",
        feedforward = _power_bound_feedforward(),
        allow_different_components = true,
    )
    warning = (:warn, r"leaves 1 components? out of the feedforward")
    for _ in 1:2
        @test build!(sim; console_level = Logging.AboveMaxLevel) ==
              PSI.SimulationBuildStatus.BUILT
        @test_logs warning min_level = Logging.Warn PSI._check_state_sources(sim)
    end

    allowed = _resequenced_simulation(sim; allow_different_components = true)
    @test build!(allowed; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    @test_logs warning min_level = Logging.Warn PSI._check_state_sources(allowed)

    default = _resequenced_simulation(sim; allow_different_components = false)
    err = _build_error(default)
    @test err isa IS.ConflictingInputsError
    @test occursin("allow_different_components = true", sprint(showerror, err))
    @test only(_ed_feedforwards(default)) isa UpperBoundFeedforward
    @test POM.get_feedforward_components(only(_ed_feedforwards(default))) ==
          make_selector(ThermalStandard)
    ed = PSI.get_simulation_model(PSI.get_models(default), :ED)
    @test !haskey(IOM.get_internal(ed).ext, PSI.FEEDFORWARD_SOURCES_KEY)
end

# Day-ahead models in the listed order, so the dispatch can be listed before a UC. Each entry
# is a model name and the unit that model does not model, if any. The ED reads its
# feedforward from the UC models.
function _late_source_simulation(entries::Pair{String}...)
    decision_models = DecisionModel[]
    for (name, without) in entries
        sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
        isnothing(without) ||
            set_available!(get_component(ThermalStandard, sys, without), false)
        if name == "ED"
            template = get_template_nomin_ed_simulation()
        else
            template = get_template_basic_uc_simulation()
        end
        push!(decision_models,
            DecisionModel(template, sys; name, optimizer = HiGHS_optimizer))
    end
    models = SimulationModels(; decision_models)
    sequence = SimulationSequence(;
        models,
        feedforwards = Dict("ED" => [_power_bound_feedforward()]),
        ini_cond_chronology = InterProblemChronology(),
        allow_different_components = true,
    )
    return Simulation(; name = "late_source", steps = 2, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
end

@testset "A whole feedforward reads a source built after its target" begin
    # No model built before the ED writes the source, so its feedforward stays whole and the
    # UC listed after it writes every unit.
    sim = _late_source_simulation("ED" => nothing, "UC" => nothing)
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    @test POM.get_feedforward_components(only(_ed_feedforwards(sim))) ==
          make_selector(ThermalStandard)
end

@testset "A restricted feedforward source built after its target fails at build" begin
    # UC1 lacks Alta and is built before the ED, so the ED's feedforward leaves Alta out, but
    # UC2, listed after the ED, writes Alta.
    sim = _late_source_simulation("UC1" => "Alta", "ED" => nothing, "UC2" => nothing)
    err = _build_error(sim)
    @test err isa IS.ConflictingInputsError
    msg = sprint(showerror, err)
    @test occursin("ED", msg) && occursin("ActivePowerVariable__ThermalStandard", msg)
    @test occursin(
        "List the models that write a feedforward's source before the models that read it",
        msg,
    )
end

@testset "A restricted simulation runs" begin
    sim = _state_source_simulation(;
        uc_without = "Alta",
        feedforward = _power_bound_feedforward(),
        allow_different_components = true,
        steps = 2,
    )
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; enable_progress_bar = false) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "Bound and fix-value feedforwards leave out the units their source model lacks" begin
    for (feedforward_type, constraint, keywords) in (
        (UpperBoundFeedforward, POM.FeedforwardUpperBoundConstraint, (; add_slacks = true)),
        (LowerBoundFeedforward, POM.FeedforwardLowerBoundConstraint, (; add_slacks = true)),
        (FixValueFeedforward, POM.FeedforwardFixValueConstraint, (;)),
    )
        # The ED writes its own power for Alta; that is no source built after it.
        feedforward = feedforward_type(;
            component_type = ThermalStandard,
            source = ActivePowerVariable,
            affected_values = [ActivePowerVariable],
            keywords...,
        )
        sim = _state_source_simulation(;
            uc_without = "Alta",
            feedforward,
            allow_different_components = true,
            steps = 2,
        )
        @test build!(sim; console_level = Logging.AboveMaxLevel) ==
              PSI.SimulationBuildStatus.BUILT
        ed = PSI.get_simulation_model(PSI.get_models(sim), :ED)
        rows = _feedforward_constraint_rows(ed, constraint)
        @test !isempty(rows)
        @test all(names -> !("Alta" in names) && "Park City" in names, rows)
    end
end

@testset "A feedforward that leaves nothing out stays whole" begin
    sim = _state_source_simulation(;
        feedforward = _power_bound_feedforward(),
        allow_different_components = true,
    )
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    @test POM.get_feedforward_components(only(_ed_feedforwards(sim))) ==
          make_selector(ThermalStandard)
end

_selected_names(ff, sys) =
    Set(get_name.(get_components(POM.get_feedforward_components(ff), sys)))

@testset "A feedforward source may write units that the target's system lacks" begin
    sim = _state_source_simulation(;
        uc_without = "Park City",
        ed_without = "Alta",
        feedforward = _power_bound_feedforward(),
        allow_different_components = true,
    )
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    ed = PSI.get_simulation_model(PSI.get_models(sim), :ED)
    ed_sys = PSI.get_system(ed)
    @test isnothing(get_component(ThermalStandard, ed_sys, "Alta"))
    @test _selected_names(only(_ed_feedforwards(sim)), ed_sys) ==
          Set(["Brighton", "Solitude", "Sundance"])
    rows = _feedforward_constraint_rows(ed, POM.FeedforwardUpperBoundConstraint)
    @test !isempty(rows)
    @test all(names -> issetequal(names, ["Brighton", "Solitude", "Sundance"]), rows)
end

# A UC that does not model Alta and an hourly emulator that does, with a power bound
# feedforward on the emulator.
function _emulator_feedforward_simulation(models; allow_different_components)
    sequence = SimulationSequence(;
        models,
        feedforwards = Dict("EM" => [_power_bound_feedforward()]),
        ini_cond_chronology = InterProblemChronology(),
        allow_different_components,
    )
    return Simulation(; name = "em_feedforward", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
end

_em_feedforwards(models) =
    IOM.get_feedforwards(
        POM.get_device_models(PSI.get_template(PSI.get_emulation_model(models)))[:ThermalStandard],
    )

@testset "An emulator feedforward leaves out the units no decision model writes" begin
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    set_available!(get_component(ThermalStandard, sys_uc, "Alta"), false)
    sys_em = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(get_template_basic_uc_simulation(), sys_uc; name = "UC",
                optimizer = HiGHS_optimizer),
        ],
        emulation_model = EmulationModel(get_template_nomin_ed_simulation(), sys_em;
            name = "EM", optimizer = HiGHS_optimizer),
    )
    sim = _emulator_feedforward_simulation(models; allow_different_components = true)
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
    rows = _feedforward_constraint_rows(
        PSI.get_emulation_model(models),
        POM.FeedforwardUpperBoundConstraint,
    )
    @test !isempty(rows)
    @test all(names -> !("Alta" in names) && "Park City" in names, rows)
    @test _selected_names(only(_em_feedforwards(models)), sys_em) ==
          Set(["Brighton", "Park City", "Solitude", "Sundance"])

    # A new sequence starts from the feedforward as attached.
    _emulator_feedforward_simulation(models; allow_different_components = false)
    @test POM.get_feedforward_components(only(_em_feedforwards(models))) ==
          make_selector(ThermalStandard)
end

# The UC dispatches without commitment, so no model writes the status the ED's
# feedforward reads.
function _no_source_simulation(allow_different_components)
    template_uc = get_template_basic_uc_simulation()
    set_device_model!(template_uc, ThermalStandard, ThermalDispatchNoMin)
    models = create_simulation_build_test_problems(template_uc,
        get_template_nomin_ed_simulation(),
        PSB.build_system(PSITestSystems, "c_sys5_uc"),
        PSB.build_system(PSITestSystems, "c_sys5_ed"))
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
        allow_different_components,
    )
    return Simulation(; name = "no_source", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
end

@testset "A feedforward whose source no model writes fails at build" begin
    for allow_different_components in (false, true)
        err = _build_error(_no_source_simulation(allow_different_components))
        @test err isa IS.ConflictingInputsError
        msg = sprint(showerror, err)
        @test occursin("ED", msg) && occursin("OnStatusParameter__ThermalStandard", msg)
        @test occursin("no model writes", msg)
        @test !occursin("allow_different_components", msg)
    end
end

@testset "A must-run unit needs no feedforward source" begin
    sim = _state_source_simulation(;
        uc_without = "Alta",
        ed_must_run = "Alta",
        allow_different_components = true,
    )
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
end

@testset "A model that never reads the state skips the writer checks" begin
    # No model commits units, so the ED's initial power has no status source, but a single
    # execution keeps the ED's own initial conditions.
    template_ed = get_template_nomin_ed_simulation()
    set_device_model!(template_ed, ThermalStandard, ThermalStandardDispatch)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(template_ed, PSB.build_system(PSITestSystems, "c_sys5_ed");
                name = "ED", optimizer = HiGHS_optimizer),
        ],
    )
    sequence = SimulationSequence(; models, ini_cond_chronology = InterProblemChronology())
    sim = Simulation(; name = "no_state_reads", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
    @test build!(sim; console_level = Logging.AboveMaxLevel) ==
          PSI.SimulationBuildStatus.BUILT
end

@testset "A feedforward source written only by the emulator fails at build" begin
    # The emulator commits Alta, but feedforwards read the decision state, which only the
    # decision models write, and the UC does not model Alta.
    sys_em = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = deepcopy(sys_em)
    PSY.transform_single_time_series!(sys_ed, Hour(1), Hour(1))
    set_available!(get_component(ThermalStandard, sys_uc, "Alta"), false)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(get_template_basic_uc_simulation(), sys_uc; name = "UC",
                optimizer = HiGHS_optimizer),
            DecisionModel(get_template_nomin_ed_simulation(), sys_ed; name = "ED",
                optimizer = HiGHS_optimizer),
        ],
        emulation_model = EmulationModel(get_template_basic_uc_simulation(), sys_em;
            name = "EM", optimizer = HiGHS_optimizer),
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
        allow_different_components = true,
    )
    sim = Simulation(; name = "em_only_source", steps = 1, models, sequence,
        simulation_folder = mktempdir(; cleanup = true))
    err = _build_error(sim)
    @test err isa IS.ConflictingInputsError
    msg = sprint(showerror, err)
    @test occursin("ED", msg) && occursin("Alta", msg)
    @test occursin("OnStatusParameter__ThermalStandard", msg)
    on_key = PSI.VariableKey(OnVariable, ThermalStandard)
    @test "Alta" in first(PSI._model_columns(PSI.get_emulation_model(models), on_key))
end
