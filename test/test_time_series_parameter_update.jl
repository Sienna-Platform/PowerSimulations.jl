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
    # A left-hand-side parameter needs a model that rebuilds every step, which template
    # validation sets for a model whose components carry one.
    if any(IOM.requires_rebuild(P) for (P, _, _, _) in cases)
        IOM.set_rebuild_model!(IOM.get_settings(container), true)
    end
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
