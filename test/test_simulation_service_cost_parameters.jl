# Time-varying reserve demand curve (ORDC) cost parameters across simulation steps. POM keys the
# slope and breakpoint parameters of every reserve of a service type into ONE container with an
# empty `meta`, the reserves along its first axis; the updater must walk that axis rather than
# read a service name from the key.

# A time-varying ORDC of reserve type `R` built from the system's static ORDC baseline, backed
# by a deterministic cost-curve forecast with the system's horizon, interval and window count.
function _add_ts_ordc!(
    sys,
    name::String,
    static_ordc;
    reserve_type = OnlineReserve{ReserveUp},
    incrs_x = (0.0, 0.0, 0.0),
    incrs_y = (0.0, 0.0, 0.0),
    create_extra_tranches = false,
)
    baseline_curve = PSY.get_variable(static_ordc)
    power_units = PSY.get_power_units(baseline_curve)
    fd = PSY.get_function_data(PSY.get_value_curve(baseline_curve))
    ordc_ts = reserve_type(;
        name = name,
        available = true,
        time_frame = PSY.get_time_frame(static_ordc),
    )
    add_service!(sys, ordc_ts, get_components(ThermalStandard, sys))
    pwl_ts = make_deterministic_ts(
        sys,
        "variable_cost",
        fd,
        incrs_x,
        incrs_y;
        override_min_x = 0.0,
        override_max_x = last(get_x_coords(fd)),
        create_extra_tranches = create_extra_tranches,
    )
    pwl_key = add_time_series!(sys, ordc_ts, pwl_ts)
    PSY.set_variable!(ordc_ts, PSY.make_market_bid_ts_curve(pwl_key, nothing, power_units))
    return ordc_ts
end

@testset "Time-varying ORDC cost parameters update between steps" begin
    # On-line and off-line reserves carry their demand curve the same way, and POM builds the
    # cost parameters for both, so both have to update.
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    static_ordc = first(get_components(PSY.has_demand_curve, PSY.OnlineReserve, sys))
    # Two ORDCs with different tranche counts share one padded container. The interval
    # increments make every window's curve differ from the previous one.
    _add_ts_ordc!(sys, "ORDC_TS1", static_ordc;
        incrs_x = (0.03, 0.13, 0.07), incrs_y = (0.03, 0.13, 0.07),
        create_extra_tranches = true)
    _add_ts_ordc!(sys, "ORDC_TS2", static_ordc;
        incrs_x = (0.03, 0.13, 0.07), incrs_y = (0.02, 0.14, 0.08),
        create_extra_tranches = true)
    _add_ts_ordc!(sys, "NSPIN_TS", static_ordc; reserve_type = OfflineReserve,
        incrs_x = (0.03, 0.13, 0.07), incrs_y = (0.02, 0.14, 0.08))

    template = POM.PowerOperationsProblemTemplate(
        NetworkModel(CopperPlateNetworkModel; use_slacks = true))
    # Off-line reserve is supplied by units that are off, so the thermals need a commitment.
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_service_model!(template, ServiceModel(OnlineReserve{ReserveDown}, RangeReserve))
    set_service_model!(template, ServiceModel(OnlineReserve{ReserveUp}, StepwiseCostReserve))
    set_service_model!(template, ServiceModel(OfflineReserve, StepwiseCostReserve))
    model = DecisionModel(template, sys; name = "UC", optimizer = HiGHS_optimizer)
    models = SimulationModels(; decision_models = [model])
    sim = Simulation(;
        name = "ordc_ts",
        steps = 2,
        models,
        sequence = SimulationSequence(; models),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT

    container = PSI.get_optimization_container(model)
    function breakpoints_of(::Type{R}, name) where {R}
        param = IOM._breakpoint_param(
            POM._reserve_offer_direction(PSY.get_component(R, sys, name)))
        return IOM.get_parameter_array(container, IOM.ParameterKey(param, R))
    end
    reserves = [(OnlineReserve{ReserveUp}, "ORDC_TS1"), (OnlineReserve{ReserveUp}, "ORDC_TS2"),
        (OfflineReserve, "NSPIN_TS")]
    values_of(R, name) = Array(breakpoints_of(R, name)[name, :, :].data)
    at_build = Dict(name => values_of(R, name) for (R, name) in reserves)

    # Step 1 solves on the build-time values; step 2 updates every reserve in the container.
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    for (R, name) in reserves
        @test values_of(R, name) != at_build[name]
    end
end
