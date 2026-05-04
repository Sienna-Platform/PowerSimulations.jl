# See also test_import_export_cost.jl

const BASIC_SOURCE_CONSTRAINT_KEYS = [
    PSI.ConstraintKey(ImportExportBudgetConstraint, PSY.Source, "import"),
    PSI.ConstraintKey(ImportExportBudgetConstraint, PSY.Source, "export"),
    PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.Source, "ub"),
    PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.Source, "lb"),
    PSI.ConstraintKey(InputActivePowerVariableLimitsConstraint, PSY.Source, "ub"),
    PSI.ConstraintKey(InputActivePowerVariableLimitsConstraint, PSY.Source, "lb"),
    PSI.ConstraintKey(PiecewiseLinearBlockIncrementalOfferConstraint, PSY.Source),
    PSI.ConstraintKey(PiecewiseLinearBlockDecrementalOfferConstraint, PSY.Source),
]

const TS_SOURCE_CONSTRAINT_KEYS = [
    BASIC_SOURCE_CONSTRAINT_KEYS...,
    PSI.ConstraintKey(ActivePowerOutVariableTimeSeriesLimitsConstraint, Source, "ub"),
    PSI.ConstraintKey(ActivePowerInVariableTimeSeriesLimitsConstraint, Source, "ub"),
]

@testset "ImportExportSource Source With CopperPlate" begin
    sys = make_5_bus_with_import_export(; add_single_time_series = false)

    model = DecisionModel(MockOperationProblem, CopperPlatePowerModel, sys)
    device_model = DeviceModel(
        Source,
        ImportExportSourceModel;
        attributes = Dict("reservation" => false),
    )
    mock_construct_device!(model, device_model)
    moi_tests(model, 240, 0, 242, 48, 48, false)
    psi_constraint_test(model, BASIC_SOURCE_CONSTRAINT_KEYS)
    psi_checkobjfun_test(model, GAEVF)

    model = DecisionModel(MockOperationProblem, CopperPlatePowerModel, sys)
    device_model = DeviceModel(
        Source,
        ImportExportSourceModel;
        attributes = Dict("reservation" => true),
    )
    mock_construct_device!(model, device_model)
    moi_tests(model, 264, 0, 242, 48, 48, true)
    psi_constraint_test(model, BASIC_SOURCE_CONSTRAINT_KEYS)
    psi_checkobjfun_test(model, GAEVF)
end

@testset "ImportExportSource Source With ACPPowerModel" begin
    sys = make_5_bus_with_import_export(; add_single_time_series = false)

    model = DecisionModel(MockOperationProblem, ACPPowerModel, sys)
    device_model = DeviceModel(
        Source,
        ImportExportSourceModel;
        attributes = Dict("reservation" => false),
    )
    mock_construct_device!(model, device_model)
    moi_tests(model, 264, 0, 266, 72, 48, false)
    psi_constraint_test(model, BASIC_SOURCE_CONSTRAINT_KEYS)
    psi_checkobjfun_test(model, GAEVF)

    model = DecisionModel(MockOperationProblem, ACPPowerModel, sys)
    device_model = DeviceModel(
        Source,
        ImportExportSourceModel;
        attributes = Dict("reservation" => true),
    )
    mock_construct_device!(model, device_model)
    moi_tests(model, 288, 0, 266, 72, 48, true)
    psi_constraint_test(model, BASIC_SOURCE_CONSTRAINT_KEYS)
    psi_checkobjfun_test(model, GAEVF)
end

@testset "Source With CopperPlate and TimeSeries" begin
    for source_formulation in [ImportExportSourceModel, FixedOutput]
        sys = make_5_bus_with_import_export(; add_single_time_series = true)
        source = get_component(Source, sys, "source")

        load = first(get_components(PowerLoad, sys))
        tstamp =
            TimeSeries.timestamp(
                get_time_series_array(SingleTimeSeries, load, "max_active_power"),
            )

        day_data = [
            0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
            0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
            0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
            0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        ]

        ts_data = repeat(day_data, 2)
        ts_out = SingleTimeSeries(
            "max_active_power_out",
            TimeArray(tstamp, ts_data);
            scaling_factor_multiplier = get_max_active_power,
        )
        ts_in = SingleTimeSeries(
            "max_active_power_in",
            TimeArray(tstamp, ts_data);
            scaling_factor_multiplier = get_max_active_power,
        )
        add_time_series!(sys, source, ts_out)
        add_time_series!(sys, source, ts_in)
        transform_single_time_series!(sys, Hour(24), Hour(24))

        template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
        set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
        set_device_model!(template, PowerLoad, StaticPowerLoad)
        source_model = DeviceModel(
            Source,
            source_formulation;
            attributes = Dict("reservation" => false),
        )
        set_device_model!(template, source_model)

        model = DecisionModel(
            template,
            sys;
            name = "UC",
            optimizer = HiGHS_optimizer,
            store_variable_names = true,
            optimizer_solve_log_print = false,
        )

        @test build!(model; output_dir = mktempdir()) == PSI.ModelBuildStatus.BUILT
        @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

        res = OptimizationProblemResults(model)
        if source_formulation == ImportExportSourceModel
            p_out = read_variable(
                res,
                "ActivePowerOutVariable__Source";
                table_format = TableFormat.WIDE,
            )[
                !,
                2,
            ]
            p_in =
                read_variable(
                    res,
                    "ActivePowerInVariable__Source";
                    table_format = TableFormat.WIDE,
                )[
                    !,
                    2,
                ]
        elseif source_formulation == FixedOutput
            p_out = read_parameter(
                res,
                "ActivePowerOutTimeSeriesParameter__Source";
                table_format = TableFormat.WIDE,
            )[
                !,
                2,
            ]
            p_in =
                read_parameter(
                    res,
                    "ActivePowerInTimeSeriesParameter__Source";
                    table_format = TableFormat.WIDE,
                )[
                    !,
                    2,
                ]
        end
        # Test that is zero when the time series is zero
        @test p_out[5] == 0.0
        @test p_in[5] == 0.0
        @test p_out[6] == 0.0
        @test p_in[6] == 0.0
    end
end

@testset "ImportExportSource Source With ACPPowerModel and TimeSeries" begin
    sys = make_5_bus_with_import_export(; add_single_time_series = true)
    source = get_component(Source, sys, "source")

    load = first(get_components(PowerLoad, sys))
    tstamp =
        TimeSeries.timestamp(
            get_time_series_array(SingleTimeSeries, load, "max_active_power"),
        )

    day_data = [
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
    ]

    ts_data = repeat(day_data, 2)
    ts_out = SingleTimeSeries(
        "max_active_power_out",
        TimeArray(tstamp, ts_data);
        scaling_factor_multiplier = get_max_active_power,
    )
    ts_in = SingleTimeSeries(
        "max_active_power_in",
        TimeArray(tstamp, ts_data);
        scaling_factor_multiplier = get_max_active_power,
    )
    add_time_series!(sys, source, ts_out)
    add_time_series!(sys, source, ts_in)
    transform_single_time_series!(sys, Hour(24), Hour(24))

    model = DecisionModel(MockOperationProblem, ACPPowerModel, sys)
    device_model = DeviceModel(
        Source,
        ImportExportSourceModel;
        attributes = Dict("reservation" => false),
    )
    mock_construct_device!(model, device_model)
    moi_tests(model, 264, 0, 314, 72, 48, false)
    psi_constraint_test(model, TS_SOURCE_CONSTRAINT_KEYS)
    psi_checkobjfun_test(model, GAEVF)

    model = DecisionModel(MockOperationProblem, ACPPowerModel, sys)
    device_model = DeviceModel(
        Source,
        ImportExportSourceModel;
        attributes = Dict("reservation" => true),
    )
    mock_construct_device!(model, device_model)
    moi_tests(model, 288, 0, 314, 72, 48, true)
    psi_constraint_test(model, TS_SOURCE_CONSTRAINT_KEYS)
    psi_checkobjfun_test(model, GAEVF)
end

@testset "FixedOutput Source With PTDFPowerModel and TimeSeries" begin
    sys = make_5_bus_with_import_export(; add_single_time_series = true)
    source = get_component(Source, sys, "source")

    load = first(get_components(PowerLoad, sys))
    tstamp =
        TimeSeries.timestamp(
            get_time_series_array(SingleTimeSeries, load, "max_active_power"),
        )

    day_data = [
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
    ]

    ts_data = repeat(day_data, 2)
    ts_out = SingleTimeSeries(
        "max_active_power_out",
        TimeArray(tstamp, ts_data);
        scaling_factor_multiplier = get_max_active_power,
    )
    ts_in = SingleTimeSeries(
        "max_active_power_in",
        TimeArray(tstamp, ts_data);
        scaling_factor_multiplier = get_max_active_power,
    )
    add_time_series!(sys, source, ts_out)
    add_time_series!(sys, source, ts_in)
    transform_single_time_series!(sys, Hour(24), Hour(24))

    model = DecisionModel(MockOperationProblem, PTDFPowerModel, sys)
    device_model = DeviceModel(
        Source,
        FixedOutput;
        attributes = Dict("reservation" => false),
    )
    mock_construct_device!(model, device_model)
    # Fixed output does not add variables nor constraints. 
    moi_tests(model, 0, 0, 0, 0, 0, false)
    psi_checkobjfun_test(model, GAEVF)

    model = DecisionModel(MockOperationProblem, PTDFPowerModel, sys)
    device_model = DeviceModel(
        Source,
        FixedOutput;
        attributes = Dict("reservation" => true),
    )
    mock_construct_device!(model, device_model)
    # Fixed output does not add variables nor constraints. 
    moi_tests(model, 0, 0, 0, 0, 0, false)
    psi_checkobjfun_test(model, GAEVF)
end
