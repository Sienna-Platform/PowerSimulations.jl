# Parameters written to the simulation state whose columns differ between models. Each model
# fills them for the loads named in its device model, with a value that identifies the model.
struct UnionColumnsParameter <: IOM.ParameterType end
IOM.should_write_resulting_value(::Type{UnionColumnsParameter}) = true

const UNION_KEY = IOM.ParameterKey{UnionColumnsParameter, PowerLoad}("")

struct UnionColumnsProbe <: POM.AbstractLoadFormulation end
POM.get_default_time_series_names(::Type{PowerLoad}, ::Type{UnionColumnsProbe}) =
    Dict{Type{<:IOM.TimeSeriesParameter}, String}()
POM.get_default_attributes(::Type{PowerLoad}, ::Type{UnionColumnsProbe}) =
    Dict{String, Any}("parameters" => DataType[], "columns" => String[], "value" => 0.0)

function _fill_probe_parameters!(
    container::IOM.OptimizationContainer,
    model::IOM.DeviceModel{PowerLoad, UnionColumnsProbe},
)
    for T in IOM.get_attribute(model, "parameters")
        parameter = IOM.get_parameter(container, IOM.ParameterKey{T, PowerLoad}(""))
        fill!(IOM.get_parameter_array(parameter), IOM.get_attribute(model, "value"))
        fill!(IOM.get_multiplier_array(parameter), 1.0)
    end
    return
end

function POM.construct_device!(
    container::IOM.OptimizationContainer,
    ::PSY.System,
    ::IOM.ArgumentConstructStage,
    model::IOM.DeviceModel{PowerLoad, UnionColumnsProbe},
    ::IOM.NetworkModel,
)
    for T in IOM.get_attribute(model, "parameters")
        IOM.add_param_container_shared_axes!(
            container,
            IOM.ParameterKey{T, PowerLoad}(""),
            IOM.NoAttributes(),
            Float64,
            IOM.get_attribute(model, "columns"),
            IOM.get_time_steps(container),
        )
    end
    _fill_probe_parameters!(container, model)
    return
end

POM.construct_device!(
    ::IOM.OptimizationContainer,
    ::PSY.System,
    ::IOM.ModelConstructStage,
    ::IOM.DeviceModel{PowerLoad, UnionColumnsProbe},
    ::IOM.NetworkModel,
) = nothing

function IOM.update_container_parameter_values!(
    container::IOM.OptimizationContainer,
    model::IOM.AbstractOptimizationModel,
    ::IOM.ParameterKey{UnionColumnsParameter, PowerLoad},
    ::IOM.DatasetContainer{IOM.InMemoryDataset},
)
    template = IOM.get_template(model)
    _fill_probe_parameters!(container, IOM.get_device_models(template)[:PowerLoad])
    return
end

function _probe_template(template, parameters, columns, value)
    set_device_model!(
        template,
        DeviceModel(
            PowerLoad,
            UnionColumnsProbe;
            attributes = Dict{String, Any}(
                "parameters" => DataType[parameters...],
                "columns" => columns,
                "value" => value,
            ),
        ),
    )
    return template
end

function _union_columns_models(; uc, ed, em = nothing)
    # Every model is hourly, so the emulator steps on the state's grid.
    sys_em = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = deepcopy(sys_em)
    PSY.transform_single_time_series!(sys_ed, Hour(1), Hour(1))
    decision_models = [
        DecisionModel(
            _probe_template(get_template_basic_uc_simulation(), uc...),
            sys_uc;
            name = "UC",
            optimizer = HiGHS_optimizer,
        ),
        DecisionModel(
            _probe_template(get_template_nomin_ed_simulation(), ed...),
            sys_ed;
            name = "ED",
            optimizer = HiGHS_optimizer,
        ),
    ]
    isnothing(em) && return SimulationModels(; decision_models = decision_models)
    return SimulationModels(;
        decision_models = decision_models,
        emulation_model = EmulationModel(
            _probe_template(get_template_nomin_ed_simulation(), em...),
            sys_em;
            name = "EM",
            optimizer = HiGHS_optimizer,
        ),
    )
end

function _run_union_columns_simulation(models; steps = 1)
    sequence = SimulationSequence(;
        models = models,
        ini_cond_chronology = InterProblemChronology(),
    )
    sim = Simulation(;
        name = "union_columns",
        steps = steps,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return sim
end

_state_columns(datasets, key) = PSI.get_column_names(key, PSI.get_dataset(datasets, key))[1]

# The decision-state row at the start of the last ED window: ED is the last writer there.
function _last_ed_row(sim, key)
    dataset = PSI.get_dataset(PSI.get_decision_states(PSI.get_simulation_state(sim)), key)
    ed = PSI.get_simulation_model(sim, :ED)
    last_ed_start =
        PSI.get_initial_time(sim) + PSI.get_step_resolution(sim.sequence) -
        IOM.get_interval(ed)
    return dataset, findfirst(==(last_ed_start), dataset.timestamps)
end

function _written_columns(results, key)
    df = read_realized_parameter(
        results,
        IOM.encode_key_as_string(key);
        table_format = TableFormat.WIDE,
    )
    return df, sort!(setdiff(names(df), ["DateTime"]))
end

@testset "A parameter update with no attributes is recorded" begin
    event = PSI.ParameterUpdateEvent(
        UnionColumnsParameter,
        PowerLoad,
        IOM.NoAttributes(),
        DateTime("2024-01-01T00:00:00"),
        :UC,
    )
    @test event.parameter_type == "UnionColumnsParameter"
    @test event.component_type == "PowerLoad"
    @test event.tag == ""
end

@testset "State columns are the union across models" begin
    key = UNION_KEY
    @test PSI._union_state_columns(key, (["a", "b"],), [:UC], (["c", "b"],), :ED) ==
          (["a", "b", "c"],)
    agreeing = (["a", "b"],)
    @test PSI._union_state_columns(key, agreeing, [:UC], (["b", "a"],), :ED) === agreeing
    union_3d =
        PSI._union_state_columns(key, (["a"], ["x", "y"]), [:UC], (["b"], ["x", "y"]), :ED)
    @test union_3d == (["a", "b"], ["x", "y"])
    err = try
        PSI._union_state_columns(key, (["a"], ["x", "y"]), [:UC], (["b"], ["x", "z"]), :ED)
        nothing
    catch e
        e
    end
    @test err isa IS.InvalidValue
    message = sprint(showerror, err)
    @test occursin(IOM.encode_key_as_string(key), message)
    @test occursin("UC", message) && occursin("ED", message)
end

@testset "A model updates only the state columns it has" begin
    ts = DateTime("2024-01-01T00:00:00")
    resolution = Millisecond(Hour(1))
    params = PSI.ModelStoreParams(1, 2, resolution, resolution, 100.0, UUIDs.uuid4())

    key_2d = PSI.VariableKey(ActivePowerVariable, ThermalStandard)
    state = PSI.SimulationState()
    PSI.set_dataset!(
        PSI.get_decision_states(state),
        key_2d,
        PSI.InMemoryDataset(-1.0, ts, resolution, 1, 2, ["gen", "other"]),
    )
    store_data = DenseAxisArray([1.0 2.0], ["gen"], 1:2)
    PSI.update_decision_state!(state, key_2d, store_data, ts, params, :M)
    values = PSI.get_decision_state_value(state, key_2d)
    @test values["gen", :].data == [1.0, 2.0]
    @test values["other", :].data == [-1.0, -1.0]

    key_3d = PSI.VariableKey(ActivePowerVariable, ThermalStandard)
    state = PSI.SimulationState()
    PSI.set_dataset!(
        PSI.get_decision_states(state),
        key_3d,
        PSI.InMemoryDataset(-1.0, ts, resolution, 1, 2, (["gen", "other"], ["1", "2"])),
    )
    store_data = DenseAxisArray(reshape([1.0, 2.0, 3.0, 4.0], 1, 2, 2), ["gen"], 1:2, 1:2)
    PSI.update_decision_state!(state, key_3d, store_data, ts, params, :M)
    values = PSI.get_decision_state_value(state, key_3d)
    @test values["gen", "1", :].data == [1.0, 3.0]
    @test values["other", "2", :].data == [-1.0, -1.0]
end

@testset "Stages and an emulator with different columns share one state" begin
    load_names = ["Bus2", "Bus3", "Bus4"]
    uc_columns, ed_columns, em_columns =
        ["Bus2", "Bus3"], ["Bus3", "Bus4"], ["Bus2", "Bus4"]
    models = _union_columns_models(;
        uc = ([UnionColumnsParameter], uc_columns, 1.0),
        ed = ([UnionColumnsParameter], ed_columns, 2.0),
        em = ([UnionColumnsParameter], em_columns, 3.0),
    )
    sim = _run_union_columns_simulation(models)
    state = PSI.get_simulation_state(sim)
    @test sort(_state_columns(PSI.get_decision_states(state), UNION_KEY)) == load_names
    @test sort(_state_columns(PSI.get_system_states(state), UNION_KEY)) == load_names

    # ED wrote the columns it has; the UC value for the column ED lacks stays.
    dataset, row = _last_ed_row(sim, UNION_KEY)
    @test dataset.values["Bus3", row] == 2.0
    @test dataset.values["Bus4", row] == 2.0
    @test dataset.values["Bus2", row] == 1.0

    results = SimulationResults(sim)
    for (name, columns, value) in (("UC", uc_columns, 1.0), ("ED", ed_columns, 2.0))
        df, written =
            _written_columns(get_decision_problem_results(results, name), UNION_KEY)
        @test written == columns
        @test all(==(value), Matrix(df[!, columns]))
    end
    df, written = _written_columns(get_emulation_problem_results(results), UNION_KEY)
    @test written == em_columns
    @test all(==(3.0), Matrix(df[!, em_columns]))
end

@testset "Reading a state value no model wrote names the component and key" begin
    # The emulator writes the commitment state without Alta, so Alta's system-state column is
    # never written and UC's initial conditions read it at the second step.
    sys_em = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc")
    sys_ed = deepcopy(sys_em)
    PSY.transform_single_time_series!(sys_ed, Hour(1), Hour(1))
    em_template = get_template_basic_uc_simulation()
    set_device_model!(
        em_template,
        DeviceModel(
            ThermalStandard,
            ThermalBasicUnitCommitment;
            attributes = Dict{String, Any}(
                "device_filter" =>
                    make_selector(x -> PSY.get_name(x) != "Alta", ThermalStandard),
            ),
        ),
    )
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
        emulation_model = EmulationModel(
            em_template,
            sys_em;
            name = "EM",
            optimizer = HiGHS_optimizer,
        ),
    )
    sim = Simulation(;
        name = "unwritten_state",
        steps = 2,
        models = models,
        sequence = SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
        ),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.FAILED
    log = read(joinpath(sim.internal.logs_dir, PSI.SIMULATION_LOG_FILENAME), String)
    @test occursin("Alta", log)
    @test occursin("OnVariable__ThermalStandard", log)
    @test occursin("device filters", log)
end

@testset "A model output with a non-finite value is rejected" begin
    ts = DateTime("2024-01-01T00:00:00")
    resolution = Millisecond(Hour(1))
    params = PSI.ModelStoreParams(1, 2, resolution, resolution, 100.0, UUIDs.uuid4())
    keys = (
        PSI.VariableKey(ActivePowerVariable, ThermalStandard),
        PSI.AuxVarKey(PSI.TimeDurationOn, ThermalStandard),
    )
    for key in keys
        state = PSI.SimulationState()
        PSI.set_dataset!(
            PSI.get_decision_states(state),
            key,
            PSI.InMemoryDataset(0.0, ts, resolution, 1, 2, ["gen"]),
        )
        store_data = DenseAxisArray([1.0 NaN], ["gen"], 1:2)
        err = try
            PSI.update_decision_state!(state, key, store_data, ts, params, :UC)
            nothing
        catch e
            e
        end
        @test err isa IS.InvalidValue
        message = sprint(showerror, err)
        @test occursin(IOM.encode_key_as_string(key), message)
        @test occursin("gen", message)
        @test occursin("UC", message)
    end
end
