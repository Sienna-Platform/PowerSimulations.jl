struct ValueAxesParameter <: IOM.TimeSeriesParameter end
struct ValueAxesLHSParameter <: IOM.LeftHandSideTimeSeriesParameter end

const _VALUE_AXES_T0 = DateTime("2024-01-01T00:00:00")

# Adds a two-window "factors" series whose steps have the shape of `value_axes`, and returns
# the second window flattened as the parameter holds it: (position, time).
function _add_value_axes_series!(sys, component, value_axes)
    dims = Tuple(length(axis.labels) for axis in value_axes)
    windows = Dict(
        _VALUE_AXES_T0 + Day(d) =>
            reshape(collect(1.0:(24 * prod(dims))) .+ 1000.0 * d, 24, dims...) for
        d in 0:1
    )
    PSY.add_time_series!(
        sys,
        component,
        IS.Deterministic("factors", windows, Hour(1), Day(1); value_axes = value_axes),
    )
    second = windows[_VALUE_AXES_T0 + Day(1)]
    return reduce(hcat, [vec(selectdim(second, 1, t)) for t in 1:24])
end

# A load zone has no availability flag, so the update cannot list it by availability.
function _value_axes_cases(sys)
    zone = PSY.LoadZone("zone", 0.0, 0.0)
    PSY.add_component!(sys, zone)
    return [
        (
            ValueAxesParameter,
            ThermalStandard,
            PSY.get_component(ThermalStandard, sys, "Alta"),
            [IS.TimeSeriesAxis("block", [1, 2]), IS.TimeSeriesAxis("product", ["a", "b"])],
        ),
        (ValueAxesLHSParameter, PSY.LoadZone, zone, [IS.TimeSeriesAxis("bus", [1, 2, 3])]),
    ]
end

# Adds the containers the build would make for `cases`, advances one step, updates them,
# and checks each one against `expected`.
function _test_value_axes_update!(model, ts_type, cases, expected)
    container = IOM.get_optimization_container(model)
    time_steps = IOM.get_time_steps(container)
    parameters = map(cases, expected) do (P, V, component, _), values
        parameter = IOM.add_time_series_parameter_container!(
            container,
            P,
            V,
            ts_type,
            "factors",
            ["factors_hash"],
            [PSY.get_name(component)],
            (1:size(values, 1),),
            time_steps,
        )
        IOM.add_component_name!(
            IOM.get_attributes(parameter),
            PSY.get_name(component),
            "factors_hash",
        )
        for t in time_steps
            IOM._set_parameter_at!(
                IOM.get_parameter_array(parameter).data,
                IOM.get_jump_model(container),
                zeros(size(values, 1)),
                1,
                t,
            )
        end
        return parameter
    end
    IOM.advance_execution_count!(model)
    for ((P, V, _, _), parameter, values) in zip(cases, parameters, expected)
        IOM.update_container_parameter_values!(
            container,
            model,
            IOM.ParameterKey(P, V),
            PSI.DatasetContainer{PSI.InMemoryDataset}(),
        )
        array = IOM.get_parameter_array(parameter)
        @test jump_value.(array["factors_hash", :, :].data) == values
    end
    return
end

@testset "A parameter with an extra axis refreshes from the next window" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
    cases = _value_axes_cases(sys)
    expected = [_add_value_axes_series!(sys, c, axes) for (_, _, c, axes) in cases]
    template = PowerOperationsProblemTemplate(CopperPlateNetworkModel)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    model = DecisionModel(
        template,
        sys;
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(24),
    )
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          IOM.ModelBuildStatus.BUILT
    _test_value_axes_update!(model, IS.Deterministic, cases, expected)
end

# Adds a 48-hour "factors" series whose steps have the shape of `value_axes`, and returns the
# second hour flattened as the emulator's one-step parameter holds it: (position, 1).
function _add_value_axes_single_series!(sys, component, value_axes)
    dims = Tuple(length(axis.labels) for axis in value_axes)
    data = reshape(collect(1.0:(48 * prod(dims))), 48, dims...)
    PSY.add_time_series!(
        sys,
        component,
        IS.SingleTimeSeries(
            "factors",
            _VALUE_AXES_T0,
            Hour(1),
            data;
            value_axes = value_axes,
        ),
    )
    return reshape(vec(selectdim(data, 1, 2)), :, 1)
end

@testset "An emulator parameter with an extra axis refreshes from the next hour" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
    cases = _value_axes_cases(sys)
    expected = [_add_value_axes_single_series!(sys, c, axes) for (_, _, c, axes) in cases]
    template = PowerOperationsProblemTemplate(CopperPlateNetworkModel)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    model = EmulationModel(template, sys; optimizer = HiGHS_optimizer, resolution = Hour(1))
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          IOM.ModelBuildStatus.BUILT
    _test_value_axes_update!(model, IS.SingleTimeSeries, cases, expected)
end

# Gives the Bus4 load the same forecast array as the Bus2 load, so the two share one
# parameter row (rows dedupe on the stored array's content hash) while Bus3 keeps its own.
function _share_load_forecast!(sys, ts_type)
    source = PSY.get_component(PowerLoad, sys, "Bus2")
    target = PSY.get_component(PowerLoad, sys, "Bus4")
    if ts_type <: PSY.DeterministicSingleTimeSeries
        series = PSY.get_time_series(PSY.SingleTimeSeries, source, "max_active_power")
        PSY.remove_time_series!(sys, PSY.SingleTimeSeries, target, "max_active_power")
    else
        series = PSY.get_time_series(ts_type, source, "max_active_power")
        PSY.remove_time_series!(sys, ts_type, target, "max_active_power")
    end
    PSY.add_time_series!(sys, target, series)
    return
end

# Steps a built model through `steps` updates and checks, at each one, that every load's
# row holds that load's own series for the current window, read one series at a time.
function _test_shared_rows_update!(model, ts_type, steps)
    container = IOM.get_optimization_container(model)
    key = IOM.ParameterKey(ActivePowerTimeSeriesParameter, PowerLoad)
    parameter = IOM.get_parameter(container, key)
    attributes = IOM.get_attributes(parameter)
    array = IOM.get_parameter_array(parameter)
    hashes = Dict(n => IOM._get_ts_uuid(attributes, n) for n in ("Bus2", "Bus3", "Bus4"))
    @test hashes["Bus2"] == hashes["Bus4"] != hashes["Bus3"]
    @test length(axes(array, 1)) == 2
    horizon = length(IOM.get_time_steps(container))
    sys = IOM.get_system(model)
    for _ in 1:steps
        IOM.advance_execution_count!(model)
        IOM.update_container_parameter_values!(
            container,
            model,
            key,
            PSI.DatasetContainer{PSI.InMemoryDataset}(),
        )
        start_time = IOM.get_current_time(model)
        for (name, hash) in hashes
            expected = PSY.get_time_series_values(
                ts_type,
                PSY.get_component(PowerLoad, sys, name),
                "max_active_power";
                start_time = start_time,
                len = horizon,
            )
            @test jump_value.(array[hash, :].data) == expected
        end
    end
    return
end

function _shared_rows_model(sys; horizon, interval)
    template = PowerOperationsProblemTemplate(CopperPlateNetworkModel)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    model = DecisionModel(
        template,
        sys;
        optimizer = HiGHS_optimizer,
        horizon = horizon,
        interval = interval,
    )
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          IOM.ModelBuildStatus.BUILT
    return model
end

@testset "Loads sharing a forecast array share a row and refresh from the next window" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
    _share_load_forecast!(sys, PSY.Deterministic)
    model = _shared_rows_model(sys; horizon = Hour(24), interval = Hour(24))
    _test_shared_rows_update!(model, PSY.Deterministic, 1)
end

@testset "Loads sharing a single time series share a row across several steps" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_single_time_series = true)
    PSY.remove_time_series!(sys, PSY.Deterministic)
    _share_load_forecast!(sys, PSY.DeterministicSingleTimeSeries)
    PSY.transform_single_time_series!(sys, Hour(24), Hour(1))
    model = _shared_rows_model(sys; horizon = Hour(24), interval = Hour(1))
    _test_shared_rows_update!(model, PSY.DeterministicSingleTimeSeries, 3)
end

@testset "Parameters sharing a series name refresh together between solves" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_re")
    template = PowerOperationsProblemTemplate(CopperPlateNetworkModel)
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    model = DecisionModel(
        template,
        sys;
        optimizer = HiGHS_optimizer,
        horizon = Hour(24),
        interval = Hour(24),
    )
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          IOM.ModelBuildStatus.BUILT
    container = IOM.get_optimization_container(model)
    horizon = length(IOM.get_time_steps(container))
    param_keys = [
        IOM.ParameterKey(ActivePowerTimeSeriesParameter, PowerLoad),
        IOM.ParameterKey(ActivePowerTimeSeriesParameter, RenewableDispatch),
    ]

    IOM.advance_execution_count!(model)
    PSI.update_parameters!(model, PSI.SimulationState())
    start_time = IOM.get_current_time(model)
    for key in param_keys
        parameter = IOM.get_parameter(container, key)
        attributes = IOM.get_attributes(parameter)
        array = IOM.get_parameter_array(parameter)
        V = IOM.get_component_type(key)
        for name in IOM.get_component_names(attributes)
            expected = PSY.get_time_series_values(
                PSY.Deterministic,
                PSY.get_component(V, sys, name),
                "max_active_power";
                start_time = start_time,
                len = horizon,
            )
            @test jump_value.(array[IOM._get_ts_uuid(attributes, name), :].data) ==
                  expected
        end
    end
    # Both parameters refresh in the batch, served by one reader.
    @test PSI._refresh_reader_parameters!(model) == Set(param_keys)
    @test collect(keys(IOM.get_time_series_readers(IOM.get_internal(model)))) ==
          [(PSY.Deterministic, "max_active_power", Dates.Millisecond(Hour(1)))]
end
