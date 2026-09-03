const IECComponentType = Source
const IEC_COMPONENT_NAME = "source"
const SEL_IEC = make_selector(IECComponentType, IEC_COMPONENT_NAME)

function make_5_bus_with_import_export(;
    add_single_time_series::Bool = false,
    name = nothing,
)
    sys = build_system(
        PSITestSystems,
        "c_sys5_uc";
        add_single_time_series = add_single_time_series,
    )

    source = IECComponentType(;
        name = IEC_COMPONENT_NAME,
        available = true,
        bus = get_component(ACBus, sys, "nodeC"),
        active_power = 0.0,
        reactive_power = 0.0,
        active_power_limits = (min = -2.0, max = 2.0),
        reactive_power_limits = (min = -2.0, max = 2.0),
        R_th = 0.01,
        X_th = 0.02,
        internal_voltage = 1.0,
        internal_angle = 0.0,
        base_power = 100.0,
    )

    import_curve = make_import_curve(
        [0.0, 100.0, 105.0, 120.0, 200.0],
        [5.0, 10.0, 20.0, 40.0],
    )

    export_curve = make_export_curve(
        [0.0, 100.0, 105.0, 120.0, 200.0],
        [12.0, 8.0, 4.0, 1.0],  # elsewhere the final slope is 0.0 but that's problematic here
    )

    ie_cost = ImportExportCost(;
        import_offer_curves = import_curve,
        export_offer_curves = export_curve,
        ancillary_service_offers = Vector{Service}(),
        energy_import_weekly_limit = 1e6,
        energy_export_weekly_limit = 1e6,
    )

    set_operation_cost!(source, ie_cost)
    add_component!(sys, source)
    @assert get_component(SEL_IEC, sys) == source

    isnothing(name) || set_name!(sys, name)
    return sys
end

function make_5_bus_with_ie_ts(
    import_breakpoints_vary::Bool,
    import_slopes_vary::Bool,
    export_breakpoints_vary::Bool,
    export_slopes_vary::Bool;
    zero_min_power::Bool = true,
    unperturb_max_power::Bool = false,
    add_single_time_series::Bool = false,
    import_scalar = 1.0,
    export_scalar = 1.0,
    name = nothing)
    im_incr_x = import_breakpoints_vary ? (0.02, 0.11, 0.05) : (0.0, 0.0, 0.0)
    im_incr_y = import_slopes_vary ? (0.02, 0.11, 0.05) : (0.0, 0.0, 0.0)

    ex_incr_x = export_breakpoints_vary ? (0.03, 0.13, 0.07) : (0.0, 0.0, 0.0)
    ex_incr_y = export_slopes_vary ? (0.03, 0.13, 0.07) : (0.0, 0.0, 0.0)

    sys = make_5_bus_with_import_export(;
        add_single_time_series = add_single_time_series,
        name = name,
    )

    source = get_component(SEL_IEC, sys)
    oc = get_operation_cost(source)::ImportExportCost
    im_oc = get_import_offer_curves(oc)
    ex_oc = get_export_offer_curves(oc)
    im_fd = get_function_data(im_oc) * import_scalar
    ex_fd = get_function_data(ex_oc) * export_scalar

    im_ts = make_deterministic_ts(
        sys,
        "variable_cost_import",
        im_fd,
        im_incr_x,
        im_incr_y;
        override_min_x = zero_min_power ? 0.0 : nothing,
        override_max_x = unperturb_max_power ? last(get_x_coords(im_fd)) : nothing,
    )
    ex_ts = make_deterministic_ts(
        sys,
        "variable_cost_export",
        ex_fd,
        ex_incr_x,
        ex_incr_y;
        override_min_x = zero_min_power ? 0.0 : nothing,
        override_max_x = unperturb_max_power ? last(get_x_coords(ex_fd)) : nothing,
    )

    im_key = add_time_series!(sys, source, im_ts)
    ex_key = add_time_series!(sys, source, ex_ts)

    power_units = PSY.get_power_units(im_oc)
    ts_cost = ImportExportTimeSeriesCost(;
        import_offer_curves = make_import_export_ts_curve(im_key, power_units),
        export_offer_curves = make_import_export_ts_curve(ex_key, power_units),
        energy_import_weekly_limit = get_energy_import_weekly_limit(oc),
        energy_export_weekly_limit = get_energy_export_weekly_limit(oc),
        ancillary_service_offers = get_ancillary_service_offers(oc),
    )
    set_operation_cost!(source, ts_cost)

    return sys
end

# `_resolve_variable_cost_window` (dispatching on the static vs. time-series-backed cost
# type) is defined once in mbc_simulation_utils.jl and reused here for
# `ImportExportCost`/`ImportExportTimeSeriesCost` via its `_STATIC_OFFER_COST`/
# `_TS_OFFER_COST` unions.

# Analogous to run_mbc_obj_fun_test in test_utils/mbc_simulation_utils.jl
function run_iec_obj_fun_test(sys1, sys2, comp_name::String, ::Type{T};
    simulation = true, in_memory_store = false, reservation = false,
) where {T <: PSY.Component}
    _, res1, decisions1, nullable_decisions1 = run_iec_sim(sys1, comp_name, T;
        simulation = simulation,
        in_memory_store = in_memory_store,
        reservation = reservation,
    )
    _, res2, decisions2, nullable_decisions2 = run_iec_sim(sys2, comp_name, T;
        simulation = simulation,
        in_memory_store = in_memory_store,
        reservation = reservation,
    )

    all_decisions1 = (decisions1..., nullable_decisions1...)
    all_decisions2 = (decisions2..., nullable_decisions2...)
    if !all(isapprox.(all_decisions1, all_decisions2))
        @error all_decisions1
        @error all_decisions2
    end
    @assert all(isapprox.(all_decisions1, all_decisions2))

    ground_truth_1 = cost_due_to_time_varying_iec(sys1, res1, T)
    ground_truth_2 = cost_due_to_time_varying_iec(sys2, res2, T)

    success = obj_fun_test_helper(ground_truth_1, ground_truth_2, res1, res2)
    return decisions1, decisions2
end

function run_iec_sim(sys::System, comp_name::String, ::Type{T};
    simulation = true, in_memory_store = false, reservation = false,
) where {T <: PSY.Component}
    device_to_formulation = FormulationDict(
        Source => DeviceModel(
            Source,
            ImportExportSourceModel;
            attributes = Dict("reservation" => reservation),
        ),
    )
    model, res = if simulation
        run_generic_mbc_sim(
            sys;
            in_memory_store = in_memory_store,
            device_to_formulation = device_to_formulation,
        )
    else
        run_generic_mbc_prob(sys; device_to_formulation = device_to_formulation)
    end

    # Test that breakpoint and slope parameters read from results match the
    # ground truth from the system's offer curve time series.
    # We can compare raw PiecewiseStepData values directly because time-variant offer curve
    # time series are always in natural units and the parameter multiplier is 1.0
    # (see the analogous comment in mbc_simulation_utils.jl).
    for (is_decremental, oc_getter) in (
        (false, PSY.get_import_variable_cost),
        (true, PSY.get_export_variable_cost),
    )
        bp_param_type = if is_decremental
            PSI.DecrementalPiecewiseLinearBreakpointParameter
        else
            PSI.IncrementalPiecewiseLinearBreakpointParameter
        end
        sl_param_type = if is_decremental
            PSI.DecrementalPiecewiseLinearSlopeParameter
        else
            PSI.IncrementalPiecewiseLinearSlopeParameter
        end
        bp_param = _maybe_upgrade_to_dict(read_parameter(res, bp_param_type, T))
        sl_param = _maybe_upgrade_to_dict(read_parameter(res, sl_param_type, T))
        for (step_dt, bp_step_df) in pairs(bp_param)
            sl_step_df = sl_param[step_dt]
            for gen_name in unique(bp_step_df.name)
                comp = get_component(T, sys, gen_name)
                cost = PSY.get_operation_cost(comp)
                gen_bp = @rsubset(bp_step_df, :name == gen_name)
                gen_sl = @rsubset(sl_step_df, :name == gen_name)
                timestamps = unique(gen_bp.DateTime)
                psds =
                    get_function_data.(
                        get_value_curve.(
                            _resolve_variable_cost_window(
                                oc_getter,
                                comp,
                                cost,
                                step_dt,
                                length(timestamps),
                            ),
                        ),
                    )
                for (ts, psd) in zip(timestamps, psds)
                    expected_bp = get_x_coords(psd)
                    expected_sl = get_y_coords(psd)
                    actual_bp = sort(@rsubset(gen_bp, :DateTime == ts), :name2).value
                    actual_sl = sort(@rsubset(gen_sl, :DateTime == ts), :name2).value
                    # actual may be longer than expected due to padding (see _unwrap_for_param)
                    @test length(actual_bp) >= length(expected_bp)
                    @test length(actual_sl) >= length(expected_sl)
                    @test all(isapprox.(actual_bp[1:length(expected_bp)], expected_bp))
                    @test all(isapprox.(actual_sl[1:length(expected_sl)], expected_sl))
                end
            end
        end
    end

    decisions = (
        _read_one_value(res, PSI.ActivePowerOutVariable, T, comp_name),
        _read_one_value(res, PSI.ActivePowerInVariable, T, comp_name),
    )

    output_var = read_variable_dict(res, PSI.ActivePowerOutVariable, T)
    input_var = read_variable_dict(res, PSI.ActivePowerInVariable, T)

    for key in keys(output_var)
        output_on = output_var[key][!, "value"] .> PSI.COST_EPSILON
        input_on = input_var[key][!, "value"] .> PSI.COST_EPSILON
        if reservation
            @test all(.~(output_on .& input_on))  # no simultaneous import/export
        else
            @test any(output_on .& input_on)  # some simultaneous import/export
        end
    end

    return model, res, decisions, ()  # return format follows the MBC run_startup_shutdown_test convention
end

# Analogous to cost_due_to_time_varying_mbc in test_utils/mbc_simulation_utils.jl
# TODO deduplicate after initial time-sensitive merge
function cost_due_to_time_varying_iec(
    sys::System,
    res::IS.Outputs,
    ::Type{T},
) where {T <: PSY.Component}
    power_in_vars = read_variable_dict(res, PSI.ActivePowerInVariable, T)
    power_out_vars = read_variable_dict(res, PSI.ActivePowerOutVariable, T)
    result = SortedDict{DateTime, DataFrame}()

    for step_dt in keys(power_in_vars)
        power_in_df = power_in_vars[step_dt]
        step_df = DataFrame(:DateTime => unique(power_in_df.DateTime))
        gen_names = unique(power_in_df.name)
        @assert !isempty(gen_names)

        power_out_df = power_out_vars[step_dt]
        @assert names(power_in_df) == names(power_out_df)
        @assert all(power_in_df.DateTime .== power_out_df.DateTime)

        @assert any([
            get_operation_cost(comp) isa _ANY_IEC for
            comp in get_components(T, sys)
        ])
        for gen_name in gen_names
            comp = get_component(T, sys, gen_name)
            cost = PSY.get_operation_cost(comp)
            (cost isa _ANY_IEC) || continue
            step_df[!, gen_name] .= 0.0
            # imports = addition of power = power flowing out of the device
            # exports = reduction of power = power flowing into the device
            for (multiplier, power_df, raw_getter, variable_getter) in (
                (
                    1.0,
                    power_out_df,
                    PSY.get_import_offer_curves,
                    PSY.get_import_variable_cost,
                ),
                (
                    -1.0,
                    power_in_df,
                    PSY.get_export_offer_curves,
                    PSY.get_export_variable_cost,
                ),
            )
                offer_curves = raw_getter(cost)
                if PSI.is_time_variant(offer_curves)
                    timestamps = unique(power_df.DateTime)
                    psds =
                        get_function_data.(
                            get_value_curve.(
                                _resolve_variable_cost_window(
                                    variable_getter,
                                    comp,
                                    cost,
                                    step_dt,
                                    length(timestamps),
                                ),
                            ),
                        )
                    step_df[!, gen_name] .+=
                        multiplier *
                        _calc_pwi_cost.(
                            @rsubset(power_df, :name == gen_name).value,
                            psds,
                        )
                end
            end
        end

        measure_vars = [x for x in names(step_df) if x != "DateTime"]
        # rows represent: [time, component, time-varying MBC cost for {component} at {time}]
        result[step_dt] =
            DataFrames.stack(
                step_df,
                measure_vars;
                variable_name = :name,
                value_name = :value,
            )
    end
    return result
end

function iec_obj_fun_test_wrapper(
    sys_constant,
    sys_varying;
    reservation = false,
    use_simulation_opts = (false, true),
)
    for use_simulation in use_simulation_opts
        for in_memory_store in (use_simulation ? (false, true) : (false,))
            decisions1, decisions2 = run_iec_obj_fun_test(
                sys_constant,
                sys_varying,
                IEC_COMPONENT_NAME,
                IECComponentType;
                simulation = use_simulation,
                in_memory_store = in_memory_store,
                reservation = reservation,
            )

            if !all(isapprox.(decisions1, decisions2))
                @error decisions1
                @error decisions2
            end
            @assert all(approx_geq_1.(decisions1))
        end
    end
end
