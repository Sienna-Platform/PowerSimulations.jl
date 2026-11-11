"""
A copy of `c_sys5_uc` whose first thermal unit has a `FuelCurve` with a time-varying fuel
cost. The fuel cost series mirrors the load forecast's windows, resolution and interval, and
each window holds distinct values, so a parameter frozen at build time reads back the wrong
step. Returns the system, the unit, the fuel cost data by window and the forecast windows.
"""
function fuel_cost_time_series_system()
    sys = deepcopy(PSB.build_system(PSITestSystems, "c_sys5_uc"))
    thermal = first(PSY.get_components(PSY.ThermalStandard, sys))

    # A System cannot mix Deterministic and DeterministicSingleTimeSeries, and the model
    # reads every series with the load's horizon.
    load = first(PSY.get_components(PSY.PowerLoad, sys))
    load_ts = PSY.get_time_series(PSY.Deterministic, load, "max_active_power")
    windows = collect(pairs(get_data(load_ts)))
    horizon_count = length(last(first(windows)))

    fuel_cost_data = OrderedDict{DateTime, Vector{Float64}}(
        dt => Float64.((100 * step_ix) .+ (1:horizon_count))
        for (step_ix, (dt, _)) in enumerate(windows)
    )
    fuel_ts = PSY.Deterministic(;
        name = "fuel_cost",
        data = fuel_cost_data,
        resolution = IS.get_resolution(load_ts),
        interval = IS.get_interval(load_ts),
    )
    fuel_key = PSY.add_time_series!(sys, thermal, fuel_ts)

    old_cost = PSY.get_operation_cost(thermal)
    PSY.set_operation_cost!(
        thermal,
        PSY.ThermalGenerationCost(;
            variable_operation_cost = PSY.FuelCurve(
                PSY.LinearCurve(0.0, 5.0),
                PSY.MMBtu,
                fuel_key,
            ),
            fixed = PSY.get_fixed(old_cost),
            start_up = PSY.get_start_up(old_cost),
            shut_down = PSY.get_shut_down(old_cost),
        ),
    )
    return sys, thermal, fuel_cost_data, windows
end
