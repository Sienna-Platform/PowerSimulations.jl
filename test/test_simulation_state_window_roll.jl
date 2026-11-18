# A probe parameter whose value identifies the model that wrote it and the timestamp it was
# written for, so every decision-state cell can be checked against the row label it carries
# after the state window starts a new step.
struct WindowRollParameter <: IOM.ParameterType end
IOM.should_write_resulting_value(::Type{WindowRollParameter}) = true

const WINDOW_ROLL_KEY = IOM.ParameterKey{WindowRollParameter, PowerLoad}("")
const WINDOW_ROLL_T0 = DateTime("2024-01-01T00:00:00")
const WINDOW_ROLL_WRITER_SCALE = 1_000_000

struct WindowRollProbe <: POM.AbstractLoadFormulation end
POM.get_default_time_series_names(::Type{PowerLoad}, ::Type{WindowRollProbe}) =
    Dict{Type{<:IOM.TimeSeriesParameter}, String}()
POM.get_default_attributes(::Type{PowerLoad}, ::Type{WindowRollProbe}) =
    Dict{String, Any}("columns" => String[], "writer" => 0)

_probe_value(writer::Int, timestamp::DateTime) =
    Float64(
        writer * WINDOW_ROLL_WRITER_SCALE + Dates.value(Minute(timestamp - WINDOW_ROLL_T0)),
    )

_probe_timestamp(value::Float64) =
    WINDOW_ROLL_T0 + Minute(mod(round(Int, value), WINDOW_ROLL_WRITER_SCALE))

function _fill_window_roll_probe!(
    container::IOM.OptimizationContainer,
    model::IOM.DeviceModel{PowerLoad, WindowRollProbe},
    window_start::DateTime,
)
    parameter = IOM.get_parameter(container, WINDOW_ROLL_KEY)
    values = IOM.get_parameter_array(parameter)
    resolution = IOM.get_resolution(container)
    writer = IOM.get_attribute(model, "writer")
    for t in IOM.get_time_steps(container), name in IOM.get_attribute(model, "columns")
        values[name, t] = _probe_value(writer, window_start + (t - 1) * resolution)
    end
    fill!(IOM.get_multiplier_array(parameter), 1.0)
    return
end

function POM.construct_device!(
    container::IOM.OptimizationContainer,
    ::PSY.System,
    ::IOM.ArgumentConstructStage,
    model::IOM.DeviceModel{PowerLoad, WindowRollProbe},
    ::IOM.NetworkModel,
)
    IOM.add_param_container_shared_axes!(
        container,
        WINDOW_ROLL_KEY,
        IOM.NoAttributes(),
        Float64,
        IOM.get_attribute(model, "columns"),
        IOM.get_time_steps(container),
    )
    _fill_window_roll_probe!(container, model, IOM.get_initial_time(container))
    return
end

POM.construct_device!(
    ::IOM.OptimizationContainer,
    ::PSY.System,
    ::IOM.ModelConstructStage,
    ::IOM.DeviceModel{PowerLoad, WindowRollProbe},
    ::IOM.NetworkModel,
) = nothing

# What the decision state and the system state hold each time a model reads its parameters
# and initial conditions.
const WINDOW_ROLL_SIM = Ref{Any}(nothing)
const WINDOW_ROLL_SNAPSHOTS = NamedTuple[]

function _window_roll_snapshot(read_time::DateTime, horizon::Dates.Millisecond)
    state = PSI.get_simulation_state(WINDOW_ROLL_SIM[])
    decision = PSI.get_dataset(PSI.get_decision_states(state), WINDOW_ROLL_KEY)
    system = PSI.get_dataset(PSI.get_system_states(state), WINDOW_ROLL_KEY)
    return (
        read_time = read_time,
        horizon = horizon,
        decision_timestamps = copy(decision.timestamps),
        decision_values = copy(decision.values.data),
        system_timestamp = system.timestamps[1],
        system_values = copy(system.values.data[:, 1]),
    )
end

function IOM.update_container_parameter_values!(
    container::IOM.OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    ::IOM.ParameterKey{WindowRollParameter, PowerLoad},
    ::IOM.DatasetContainer{IOM.InMemoryDataset},
)
    window_start = IOM.get_current_timestamp(model)
    push!(
        WINDOW_ROLL_SNAPSHOTS,
        _window_roll_snapshot(window_start, IOM.get_horizon(model)),
    )
    device_model = IOM.get_device_models(IOM.get_template(model))[:PowerLoad]
    _fill_window_roll_probe!(container, device_model, window_start)
    return
end

function _window_roll_template(template, columns, writer)
    set_device_model!(
        template,
        DeviceModel(
            PowerLoad,
            WindowRollProbe;
            attributes = Dict{String, Any}("columns" => columns, "writer" => writer),
        ),
    )
    return template
end

function _run_window_roll_simulation(;
    uc_horizon,
    uc_interval,
    ed_horizon,
    uc_columns,
    ed_columns,
    allow_different_components,
)
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    PSY.transform_single_time_series!(sys_uc, uc_horizon, uc_interval)
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    PSY.transform_single_time_series!(sys_ed, ed_horizon, Hour(1))
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                _window_roll_template(get_template_basic_uc_simulation(), uc_columns, 1),
                sys_uc;
                name = "UC",
                optimizer = HiGHS_optimizer,
            ),
            DecisionModel(
                _window_roll_template(get_template_nomin_ed_simulation(), ed_columns, 2),
                sys_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
            ),
        ],
    )
    sim = Simulation(;
        name = "window_roll",
        steps = 3,
        models = models,
        sequence = SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
            allow_different_components = allow_different_components,
        ),
        simulation_folder = mktempdir(; cleanup = true),
    )
    empty!(WINDOW_ROLL_SNAPSHOTS)
    WINDOW_ROLL_SIM[] = sim
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    WINDOW_ROLL_SIM[] = nothing
    return copy(WINDOW_ROLL_SNAPSHOTS)
end

_is_mislabeled(value::Float64, label::DateTime) =
    !isnan(value) && _probe_timestamp(value) != label

# Decision-state values that hold a value written for another time than their row label.
_mislabeled_state_values(snapshot) = [
    (snapshot.read_time, label) for
    (row, label) in enumerate(snapshot.decision_timestamps)
    for value in snapshot.decision_values[:, row] if _is_mislabeled(value, label)
]

# System-state values, the source of `InterProblemChronology` initial conditions, that hold
# a value written for another time than the system state reports.
_mislabeled_system_values(snapshot) = [
    (snapshot.read_time, snapshot.system_timestamp) for
    value in snapshot.system_values if
    _is_mislabeled(value, snapshot.system_timestamp)
]

@testset "Simulation state window starts" begin
    @testset "Default mode keeps the initial condition source on its timestamps" begin
        # UC has no lookahead and ED looks ahead, so the decision state spans past what UC
        # writes and UC starts each new window. Rows UC does not reach keep earlier values
        # until ED writes them; reads that reach those rows first find them unset in the
        # first step.
        columns = ["Bus2", "Bus3", "Bus4"]
        snapshots = _run_window_roll_simulation(;
            uc_horizon = Hour(12),
            uc_interval = Hour(12),
            ed_horizon = Hour(3),
            uc_columns = columns,
            ed_columns = columns,
            allow_different_components = false,
        )
        @test !isempty(snapshots)
        @test isempty(reduce(vcat, _mislabeled_system_values.(snapshots)))
    end

    @testset "With different components, every value keeps its timestamp" begin
        # UC starts each new window and lacks Bus4, which ED writes.
        snapshots = _run_window_roll_simulation(;
            uc_horizon = Hour(24),
            uc_interval = Hour(12),
            ed_horizon = Hour(3),
            uc_columns = ["Bus2", "Bus3"],
            ed_columns = ["Bus2", "Bus3", "Bus4"],
            allow_different_components = true,
        )
        @test !isempty(snapshots)
        @test isempty(reduce(vcat, _mislabeled_state_values.(snapshots)))
        @test isempty(reduce(vcat, _mislabeled_system_values.(snapshots)))
    end

    @testset "Shifting rows moves values with their timestamps" begin
        data = reshape(collect(1.0:12.0), 2, 2, 3)
        PSI._shift_state_rows!(data, 1, NaN)
        @test data[:, :, 1:2] == reshape(collect(5.0:12.0), 2, 2, 2)
        @test all(isnan, data[:, :, 3])
        data = [1.0 2.0; 3.0 4.0]
        PSI._shift_state_rows!(data, 5, 0.0)
        @test all(iszero, data)
    end
end

@testset "An outage across a step boundary holds for a unit the first model lacks" begin
    # Alta and Brighton go out at 09:00 for 6 hours, so they are unavailable from 10:00
    # through 15:00, across the 12:00 step boundary. UC starts each new window and holds
    # Brighton but not Alta.
    outage_time = DateTime("2024-01-01T09:00:00")
    outage_hours = 6
    sys_em = build_system(PSITestSystems, "c_sys5_events")
    sys_uc = build_system(PSITestSystems, "c_sys5_events")
    transform_single_time_series!(sys_uc, Hour(24), Hour(12))
    sys_ed = build_system(PSITestSystems, "c_sys5_events")
    transform_single_time_series!(sys_ed, Hour(3), Hour(1))
    for sys in (sys_uc, sys_ed, sys_em), name in ("Alta", "Brighton")
        add_supplemental_attribute!(
            sys,
            get_component(ThermalStandard, sys, name),
            PSY.GeometricDistributionForcedOutage(;
                mean_time_to_recovery = Float64(outage_hours),
                outage_transition_probability = 1.0,
            ),
        )
    end
    template_uc = get_template_basic_uc_simulation()
    set_device_model!(
        template_uc,
        DeviceModel(
            ThermalStandard,
            ThermalBasicUnitCommitment;
            attributes = Dict{String, Any}(
                "device_filter" =>
                    make_selector(x -> PSY.get_name(x) != "Alta", ThermalStandard),
            ),
        ),
    )
    template_ed = get_template_basic_uc_simulation()
    template_em = get_template_nomin_ed_simulation(CopperPlateNetworkModel)
    for template in (template_uc, template_ed, template_em)
        set_device_model!(template, Line, StaticBranch)
    end
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                sys_uc;
                name = "UC",
                initialize_model = false,
                optimizer = HiGHS_optimizer_small_gap,
            ),
            DecisionModel(
                template_ed,
                sys_ed;
                name = "ED",
                initialize_model = false,
                optimizer = HiGHS_optimizer_small_gap,
            ),
        ],
        emulation_model = EmulationModel(
            template_em,
            sys_em;
            name = "EM",
            optimizer = HiGHS_optimizer_small_gap,
        ),
    )
    sim = Simulation(;
        name = "boundary_outage",
        steps = 3,
        models = models,
        sequence = SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
            events = [
                EventModel(
                    GeometricDistributionForcedOutage,
                    POM.PresetTimeCondition([outage_time]),
                ),
            ],
            allow_different_components = true,
        ),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    em = get_emulation_problem_results(SimulationResults(sim))
    status = read_realized_parameter(
        em,
        "AvailableStatusParameter__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    power = read_realized_variable(
        em,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )
    out = outage_time .< status.DateTime .<= outage_time + Hour(outage_hours)
    @test count(out) == outage_hours
    for name in ("Alta", "Brighton")
        @test all(iszero, status[out, name])
        @test all(isone, status[.!out, name])
        @test all(x -> abs(x) < 1e-6, power[out, name])
    end
end
