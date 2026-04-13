using Base: COMPILETIME_PREFERENCES
# Read the actual data of a result to see what the timestamps are
actual_timestamps(result) = result |> values |> first |> x -> x.data |> keys |> collect

# Test that a particular call to _read_results reads from outside the cache; pass through the results
macro test_no_cache(expr)
    :(@test_logs(
        match_mode = :any,
        (:debug, r"reading results from data store"),
        min_level = Logging.Debug,
        $(esc(expr))))
end
@test_no_cache((@debug "reading results from data store"; @debug "msg 2"))

# Test that a particular call to _read_results reads from the cache; pass through the results
macro test_yes_cache(expr)
    :(@test_logs(
        match_mode = :any,
        (:debug, r"reading results from SimulationsResults cache"),
        min_level = Logging.Debug,
        $(esc(expr))))
end
@test_yes_cache((@debug "reading results from SimulationsResults cache"; @debug "msg 2"))

ED_EXPECTED_VARS = [
    "ActivePowerVariable__HydroTurbine",
    "EnergyVariable__HydroReservoir",
    "WaterSpillageVariable__HydroReservoir",
    "HydroEnergySurplusVariable__HydroReservoir",
    "HydroEnergyShortageVariable__HydroReservoir",
    "ActivePowerVariable__RenewableDispatch",
    "ActivePowerVariable__ThermalStandard",
    "SystemBalanceSlackDown__System",
    "SystemBalanceSlackUp__System",
]

UC_EXPECTED_VARS = [
    "ActivePowerVariable__HydroTurbine",
    "EnergyVariable__HydroReservoir",
    "WaterSpillageVariable__HydroReservoir",
    "HydroEnergySurplusVariable__HydroReservoir",
    "HydroEnergyShortageVariable__HydroReservoir",
    "ActivePowerVariable__RenewableDispatch",
    "ActivePowerVariable__ThermalStandard",
    "OnVariable__ThermalStandard",
    "StartVariable__ThermalStandard",
    "StopVariable__ThermalStandard",
]

function verify_export_results(results, export_path)
    exports = SimulationResultsExport(
        make_export_all(keys(results.decision_problem_results)),
        results.params,
    )
    export_results(results, exports)

    for problem_results in values(results.decision_problem_results)
        rpath = problem_results.results_output_folder
        problem = problem_results.problem
        for timestamp in get_timestamps(problem_results)
            for name in list_dual_names(problem_results)
                @test compare_results(rpath, export_path, problem, "duals", name, timestamp)
            end
            for name in list_parameter_names(problem_results)
                @test compare_results(
                    rpath,
                    export_path,
                    problem,
                    "parameters",
                    name,
                    timestamp,
                )
            end
            for name in list_variable_names(problem_results)
                @test compare_results(
                    rpath,
                    export_path,
                    problem,
                    "variables",
                    name,
                    timestamp,
                )
            end

            for name in list_aux_variable_names(problem_results)
                @test compare_results(
                    rpath,
                    export_path,
                    problem,
                    "aux_variables",
                    name,
                    timestamp,
                )
            end
        end

        # This file is not currently exported during the simulation.
        @test isfile(
            joinpath(
                problem_results.results_output_folder,
                problem_results.problem,
                "optimizer_stats.csv",
            ),
        )
    end
end

NATURAL_UNITS_VALUES = [
    "ActivePowerVariable__HydroTurbine",
    "ActivePowerVariable__RenewableDispatch",
    "ActivePowerVariable__ThermalStandard",
    "EnergyVariable__HydroReservoir",
    "ActivePowerTimeSeriesParameter__PowerLoad",
    "ActivePowerTimeSeriesParameter__HydroTurbine",
    "ActivePowerTimeSeriesParameter__RenewableDispatch",
    "ActivePowerTimeSeriesParameter__InterruptiblePowerLoad",
    "SystemBalanceSlackDown__System",
    "SystemBalanceSlackUp__System",
]

function compare_results(rpath, epath, model, field, name, timestamp)
    filename = string(name) * "_" * IS.convert_for_path(timestamp) * ".csv"
    rp = joinpath(rpath, model, field, filename)
    ep = joinpath(epath, model, field, filename)
    df1 = PSI.read_dataframe(rp)
    df2 = PSI.read_dataframe(ep)
    # TODO: Store the tables in the same format.
    measure_vars = [x for x in names(df2) if x != "DateTime"]
    df2_long = DataFrames.stack(
        df2,
        measure_vars;
        variable_name = :name,
        value_name = :value,
    )

    if name ∈ NATURAL_UNITS_VALUES
        df2_long[!, :value] .*= 100.0
    end

    names1 = names(df1)
    names2 = names(df2_long)
    names1 != names2 && return false
    size(df1) != size(df2_long) && return false

    if !isapprox(df1.value, df2_long.value)
        @error "File mismatch" rp ep row1 row2
        return false
    end

    return true
end

function make_export_all(problems)
    return [
        OptimizationProblemResultsExport(
            x;
            store_all_duals = true,
            store_all_variables = true,
            store_all_aux_variables = true,
            store_all_parameters = true,
        ) for x in problems
    ]
end

function test_simulation_results(
    file_path::String,
    export_path;
    in_memory = false,
)
    @testset "Test simulation results in_memory = $in_memory" begin
        c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
        c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
        sim = run_simulation(
            c_sys5_hy_uc,
            c_sys5_hy_ed,
            file_path,
            export_path;
            in_memory = in_memory,
        )
        results = SimulationResults(sim)
        test_decision_problem_results(results, c_sys5_hy_ed, c_sys5_hy_uc, in_memory)
        if !in_memory
            test_decision_problem_results_kwargs_handling(
                dirname(results.path),
                c_sys5_hy_ed,
                c_sys5_hy_uc,
            )
        end
        test_emulation_problem_results(results, in_memory)

        results_ed = get_decision_problem_results(results, "ED")
        @test !isempty(results_ed)
        @test !isempty(results)
        empty!(results)
        @test isempty(results_ed)
        @test isempty(results)

        verify_export_results(results, export_path)
        @test length(readdir(export_realized_results(results_ed))) === 30

        # Test that you can't read a failed simulation.
        PSI.set_simulation_status!(sim, PSI.RunStatus.FAILED)
        PSI.serialize_status(sim)
        @test PSI.deserialize_status(sim) == PSI.RunStatus.FAILED
        @test_throws ErrorException SimulationResults(sim)
        @test_logs(
            match_mode = :any,
            (:warn, r"Results may not be valid"),
            SimulationResults(sim, ignore_status = true),
        )

        if in_memory
            @test !isempty(
                sim.internal.store.dm_data[:ED].variables[PSI.VariableKey(
                    ActivePowerVariable,
                    ThermalStandard,
                )],
            )
            @test !isempty(sim.internal.store.dm_data[:ED].optimizer_stats)
            empty!(sim.internal.store)
            @test isempty(sim.internal.store.dm_data[:ED].variables)
            @test isempty(sim.internal.store.dm_data[:ED].optimizer_stats)
        end
    end
end

function test_decision_problem_results_values(
    results_ed,
    results_uc,
    c_sys5_hy_ed,
    c_sys5_hy_uc,
)
    @test IS.get_uuid(get_system(results_uc)) === IS.get_uuid(c_sys5_hy_uc)
    @test IS.get_uuid(get_system(results_ed)) === IS.get_uuid(c_sys5_hy_ed)

    # Temporarily mark some stuff unavailable
    unav_uc = first(PSY.get_available_components(ThermalStandard, get_system(results_uc)))
    PSY.set_available!(unav_uc, false)
    unav_ed = first(PSY.get_available_components(ThermalStandard, get_system(results_ed)))
    PSY.set_available!(unav_ed, false)
    sel = PSY.make_selector(ThermalStandard; groupby = :each)
    @test collect(get_components(ThermalStandard, results_uc)) ==
          collect(get_available_components(ThermalStandard, get_system(results_uc)))
    @test collect(get_components(ThermalStandard, results_ed)) ==
          collect(get_available_components(ThermalStandard, get_system(results_ed)))
    @test collect(get_groups(sel, results_uc)) ==
          collect(get_available_groups(sel, get_system(results_uc)))
    @test collect(get_groups(sel, results_ed)) ==
          collect(get_available_groups(sel, get_system(results_ed)))
    PSY.set_available!(unav_uc, true)
    PSY.set_available!(unav_ed, true)

    @test isempty(setdiff(UC_EXPECTED_VARS, list_variable_names(results_uc)))
    @test isempty(setdiff(ED_EXPECTED_VARS, list_variable_names(results_ed)))

    p_thermal_standard_ed = read_variable(results_ed, ActivePowerVariable, ThermalStandard)
    @test length(keys(p_thermal_standard_ed)) == 48
    for v in values(p_thermal_standard_ed)
        @test length(unique(v.DateTime)) == 12
        @test length(unique(v.name)) == 5
    end
    p_thermal_standard_ed_wide = read_variable(
        results_ed,
        ActivePowerVariable,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    @test length(keys(p_thermal_standard_ed)) == 48
    for v in values(p_thermal_standard_ed_wide)
        @test size(v) == (12, 6)
    end

    ren_dispatch_params =
        read_parameter(results_ed, ActivePowerTimeSeriesParameter, RenewableDispatch)
    @test length(keys(ren_dispatch_params)) == 48
    for v in values(ren_dispatch_params)
        @test length(unique(v.DateTime)) == 12
        @test length(unique(v.name)) == 3
    end

    network_duals = read_dual(results_ed, CopperPlateBalanceConstraint, PSY.System)
    @test length(keys(network_duals)) == 48
    for v in values(network_duals)
        @test length(unique(v.DateTime)) == 12
        @test length(unique(v.name)) == 1
    end

    expression = read_expression(results_ed, PSI.ProductionCostExpression, ThermalStandard)
    @test length(keys(expression)) == 48
    for v in values(expression)
        @test length(unique(v.DateTime)) == 12
        @test length(unique(v.name)) == 5
    end

    for var_key in
        ((ActivePowerVariable, RenewableDispatch), (ActivePowerVariable, ThermalStandard))
        variable_by_initial_time = read_variable(results_uc, var_key...)
        for df in values(variable_by_initial_time)
            @test length(unique(df.DateTime)) == 24
        end
    end

    realized_variable_uc = read_realized_variables(results_uc)
    @test length(keys(realized_variable_uc)) == length(UC_EXPECTED_VARS)
    for var in values(realized_variable_uc)
        @test length(unique(var.DateTime)) == 48
    end
    compare_long_and_wide_realized_results_2d(read_realized_variables, results_uc)

    realized_variable_uc =
        read_realized_variables(results_uc, [(ActivePowerVariable, ThermalStandard)])
    @test realized_variable_uc ==
          read_realized_variables(results_uc, ["ActivePowerVariable__ThermalStandard"])
    @test length(keys(realized_variable_uc)) == 1
    for var in values(realized_variable_uc)
        @test length(unique(var.DateTime)) == 48
    end

    # Test custom indexing.
    realized_variable_uc2 =
        read_realized_variables(
            results_uc,
            [(ActivePowerVariable, ThermalStandard)];
            start_time = Dates.DateTime("2024-01-01T01:00:00"),
            len = 47,
        )
    @test unique(realized_variable_uc["ActivePowerVariable__ThermalStandard"].DateTime)[2:end] ==
          unique(realized_variable_uc2["ActivePowerVariable__ThermalStandard"].DateTime)
    @test realized_variable_uc2["ActivePowerVariable__ThermalStandard"] ==
          @rsubset(
        realized_variable_uc["ActivePowerVariable__ThermalStandard"],
        :DateTime > Dates.DateTime("2024-01-01T00:00:00")
    )
    compare_long_and_wide_realized_results_2d(
        read_realized_variables,
        results_uc,
        [(ActivePowerVariable, ThermalStandard)];
        start_time = Dates.DateTime("2024-01-01T01:00:00"),
        len = 47,
    )
    compare_long_and_wide_realized_results_2d(
        read_realized_variables,
        results_uc,
        [(ActivePowerVariable, ThermalStandard)];
        start_time = Dates.DateTime("2024-01-02T12:00:00"),
        len = 10,
    )

    realized_param_uc = read_realized_parameters(results_uc)
    @test length(keys(realized_param_uc)) == 3
    for param in values(realized_param_uc)
        @test length(unique(param.DateTime)) == 48
    end
    compare_long_and_wide_realized_results_2d(read_realized_parameters, results_uc)

    realized_param_uc = read_realized_parameters(
        results_uc,
        [(ActivePowerTimeSeriesParameter, RenewableDispatch)],
    )
    compare_long_and_wide_realized_results_2d(
        read_realized_parameters,
        results_uc,
        [(ActivePowerTimeSeriesParameter, RenewableDispatch)],
    )
    @test realized_param_uc == read_realized_parameters(
        results_uc,
        ["ActivePowerTimeSeriesParameter__RenewableDispatch"],
    )
    @test length(keys(realized_param_uc)) == 1
    for param in values(realized_param_uc)
        @test length(unique(param.DateTime)) == 48
    end

    realized_duals_ed = read_realized_duals(results_ed)
    @test length(keys(realized_duals_ed)) == 1
    for param in values(realized_duals_ed)
        @test size(param)[1] == 576
    end

    realized_duals_ed =
        read_realized_duals(results_ed, [(CopperPlateBalanceConstraint, System)])
    @test realized_duals_ed ==
          read_realized_duals(results_ed, ["CopperPlateBalanceConstraint__System"])
    @test length(keys(realized_duals_ed)) == 1
    for param in values(realized_duals_ed)
        @test size(param)[1] == 576
    end

    realized_duals_uc =
        read_realized_duals(results_uc, [(CopperPlateBalanceConstraint, System)])
    @test length(keys(realized_duals_uc)) == 1
    for param in values(realized_duals_uc)
        @test size(param)[1] == 48
    end

    realized_expressions = read_realized_expressions(
        results_uc,
        [(PSI.ProductionCostExpression, RenewableDispatch)],
    )
    @test realized_expressions == read_realized_expressions(
        results_uc,
        ["ProductionCostExpression__RenewableDispatch"],
    )
    @test length(keys(realized_expressions)) == 1
    for exp in values(realized_expressions)
        @test length(unique(exp.DateTime)) == 48
        @test length(unique(exp.name)) == 3
    end

    #request non sync data
    @test_logs(
        (:error, r"Requested time does not match available results"),
        match_mode = :any,
        @test_throws IS.InvalidValue read_realized_variables(
            results_ed,
            [(ActivePowerVariable, ThermalStandard)];
            start_time = DateTime("2024-01-01T02:12:00"),
            len = 3,
        )
    )

    # request good window
    @test length(
        unique(
            read_realized_variables(
                results_ed,
                [(ActivePowerVariable, ThermalStandard)];
                start_time = DateTime("2024-01-02T23:10:00"),
                len = 10,
            )["ActivePowerVariable__ThermalStandard"].DateTime),
    ) == 10

    # request bad window
    @test_logs(
        (:error, r"Requested time does not match available results"),
        (@test_throws IS.InvalidValue read_realized_variables(
            results_ed,
            [(ActivePowerVariable, ThermalStandard)];
            start_time = DateTime("2024-01-02T23:10:00"),
            len = 11,
        ))
    )

    # request bad window
    @test_logs(
        (:error, r"Requested time does not match available results"),
        (@test_throws IS.InvalidValue read_realized_variables(
            results_ed,
            [(ActivePowerVariable, ThermalStandard)];
            start_time = DateTime("2024-01-02T23:10:00"),
            len = 12,
        ))
    )

    load_results!(
        results_ed,
        3;
        initial_time = DateTime("2024-01-01T00:00:00"),
        variables = [(ActivePowerVariable, ThermalStandard)],
    )

    @test !isempty(
        PSI.get_cached_variables(results_ed)[PSI.VariableKey(
            ActivePowerVariable,
            ThermalStandard,
        )].data,
    )
    @test length(
        PSI.get_cached_variables(results_ed)[PSI.VariableKey(
            ActivePowerVariable,
            ThermalStandard,
        )].data,
    ) == 3
    @test length(results_ed) == 3

    @test_throws(ErrorException, read_parameter(results_ed, "invalid"))
    @test_throws(ErrorException, read_variable(results_ed, "invalid"))
    @test_logs(
        (:error, r"not stored"),
        @test_throws(
            IS.InvalidValue,
            read_variable(
                results_uc,
                ActivePowerVariable,
                ThermalStandard;
                initial_time = now(),
            )
        )
    )
    @test_logs(
        (:error, r"not stored"),
        @test_throws(
            IS.InvalidValue,
            read_variable(results_uc, ActivePowerVariable, ThermalStandard; count = 25)
        )
    )

    empty!(results_ed)
    @test !haskey(
        PSI.get_cached_variables(results_ed),
        PSI.VariableKey(ActivePowerVariable, ThermalStandard),
    )

    initial_time = DateTime("2024-01-01T00:00:00")
    load_results!(
        results_ed,
        3;
        initial_time = initial_time,
        variables = [(ActivePowerVariable, ThermalStandard)],
        duals = [(CopperPlateBalanceConstraint, System)],
        parameters = [(ActivePowerTimeSeriesParameter, RenewableDispatch)],
    )

    @test !isempty(
        PSI.get_cached_variables(results_ed)[PSI.VariableKey(
            ActivePowerVariable,
            ThermalStandard,
        )].data,
    )
    @test !isempty(
        PSI.get_cached_duals(results_ed)[PSI.ConstraintKey(
            CopperPlateBalanceConstraint,
            System,
        )].data,
    )
    @test !isempty(
        PSI.get_cached_parameters(results_ed)[PSI.ParameterKey{
            ActivePowerTimeSeriesParameter,
            RenewableDispatch,
        }(
            "",
        )].data,
    )

    # Inspired by https://github.com/NREL-Sienna/PowerSimulations.jl/issues/1072
    @testset "Test cache behavior" begin
        myres = deepcopy(results_ed)
        initial_time = DateTime("2024-01-01T00:00:00")
        timestamps = PSI._process_timestamps(myres, initial_time, 3)
        variable_tuple = (ActivePowerVariable, ThermalStandard)
        variable_key = PSI.VariableKey(variable_tuple...)

        empty!(myres)
        @test isempty(PSI.get_cached_variables(myres))

        # With nothing cached, all reads should be from outside the cache
        read = @test_no_cache PSI._read_results(myres, [variable_key], timestamps, nothing)
        @test actual_timestamps(read) == timestamps

        # With 2 result windows cached, reading 2 windows should come from cache and reading 3 should come from outside
        load_results!(myres, 2; initial_time = initial_time, variables = [variable_tuple])
        @test haskey(PSI.get_cached_variables(myres), variable_key)
        read = @test_yes_cache PSI._read_results(
            myres,
            [variable_key],
            timestamps[1:2],
            nothing,
        )
        @test actual_timestamps(read) == timestamps[1:2]
        read = @test_no_cache PSI._read_results(myres, [variable_key], timestamps, nothing)
        @test actual_timestamps(read) == timestamps

        # With 3 result windows cached, reading 2 and 3 windows should both come from cache
        load_results!(myres, 3; initial_time = initial_time, variables = [variable_tuple])
        read = @test_yes_cache PSI._read_results(
            myres,
            [variable_key],
            timestamps[1:2],
            nothing,
        )
        @test actual_timestamps(read) == timestamps[1:2]
        read = @test_yes_cache PSI._read_results(myres, [variable_key], timestamps, nothing)
        @test actual_timestamps(read) == timestamps

        # Caching an additional variable should incur an additional read but not evict the old variable
        @test_no_cache load_results!(
            myres,
            3;
            initial_time = initial_time,
            variables = [(ActivePowerVariable, RenewableDispatch)],
        )
        @test haskey(PSI.get_cached_variables(myres), variable_key)
        @test haskey(
            PSI.get_cached_variables(myres),
            PSI.VariableKey(ActivePowerVariable, RenewableDispatch),
        )

        # Reset back down to 2 windows
        empty!(myres)
        @test_no_cache load_results!(
            myres,
            2;
            initial_time = initial_time,
            variables = [variable_tuple],
        )

        # Loading a subset of what has already been loaded should not incur additional reads from outside the cache
        @test_yes_cache load_results!(
            myres,
            2;
            initial_time = initial_time,
            variables = [variable_tuple],
        )
        @test_yes_cache load_results!(
            myres,
            1;
            initial_time = initial_time,
            variables = [variable_tuple],
        )
        # But loading a superset should
        @test_no_cache load_results!(
            myres,
            3;
            initial_time = initial_time,
            variables = [variable_tuple],
        )
        empty!(myres)

        # With windows 2-3 cached, reading 2-3 and 3-3 should be from cache, reading 1-2 should be from outside cache
        @test_no_cache load_results!(
            myres,
            2;
            initial_time = timestamps[2],
            variables = [variable_tuple],
        )
        read = @test_yes_cache PSI._read_results(
            myres,
            [variable_key],
            timestamps[2:3],
            nothing,
        )
        @test actual_timestamps(read) == timestamps[2:3]
        read = @test_yes_cache PSI._read_results(
            myres,
            [variable_key],
            timestamps[3:3],
            nothing,
        )
        @test actual_timestamps(read) == timestamps[3:3]
        read = @test_no_cache PSI._read_results(
            myres,
            [variable_key],
            timestamps[1:2],
            nothing,
        )
        @test actual_timestamps(read) == timestamps[1:2]

        empty!(myres)
        @test isempty(PSI.get_cached_variables(myres))
    end

    @testset "Test read_results_with_keys" begin
        myres = deepcopy(results_ed)
        initial_time = DateTime("2024-01-01T00:00:00")
        timestamps = PSI._process_timestamps(myres, initial_time, 3)
        result_keys = [PSI.VariableKey(ActivePowerVariable, ThermalStandard)]

        res1 =
            PSI.read_results_with_keys(myres, result_keys; table_format = TableFormat.WIDE)
        @test Set(keys(res1)) == Set(result_keys)
        res1_df = res1[first(result_keys)]
        @test size(res1_df) == (576, 6)
        @test names(res1_df) ==
              ["DateTime", "Solitude", "Park City", "Alta", "Brighton", "Sundance"]
        @test first(eltype.(eachcol(res1_df))) === DateTime

        res2 =
            PSI.read_results_with_keys(
                myres,
                result_keys;
                cols = ["Park City", "Brighton"],
                table_format = TableFormat.WIDE,
            )
        @test Set(keys(res2)) == Set(result_keys)
        res2_df = res2[first(result_keys)]
        @test size(res2_df) == (576, 3)
        @test names(res2_df) ==
              ["DateTime", "Park City", "Brighton"]
        @test first(eltype.(eachcol(res2_df))) === DateTime
        compare_long_and_wide_realized_results_2d(
            PSI.read_results_with_keys,
            myres,
            result_keys;
            cols = ["Park City", "Brighton"],
        )

        res3_df =
            PSI.read_results_with_keys(
                myres,
                result_keys;
                start_time = timestamps[2],
                table_format = TableFormat.WIDE,
            )[first(
                result_keys,
            )]
        @test res3_df[1, "DateTime"] == timestamps[2]

        res4_df =
            PSI.read_results_with_keys(
                myres,
                result_keys;
                len = 2,
                table_format = TableFormat.WIDE,
            )[first(result_keys)]
        @test size(res4_df) == (2, 6)
    end
end

function test_decision_problem_results(
    results::SimulationResults,
    c_sys5_hy_ed,
    c_sys5_hy_uc,
    in_memory,
)
    @test list_decision_problems(results) == ["ED", "UC"]
    results_uc = get_decision_problem_results(results, "UC")
    results_ed = get_decision_problem_results(results, "ED")

    test_decision_problem_results_values(results_ed, results_uc, c_sys5_hy_ed, c_sys5_hy_uc)
    if !in_memory
        test_simulation_results_from_file(dirname(results.path), c_sys5_hy_ed, c_sys5_hy_uc)
    end
end

function test_emulation_problem_results(results::SimulationResults, in_memory)
    results_em = get_emulation_problem_results(results)

    read_realized_aux_variables(results_em)

    duals_keys = collect(keys(read_realized_duals(results_em)))
    @test length(duals_keys) == 1
    @test duals_keys[1] == "CopperPlateBalanceConstraint__System"
    duals_inputs =
        (["CopperPlateBalanceConstraint__System"], [(CopperPlateBalanceConstraint, System)])
    for input in duals_inputs
        duals_value = first(values(read_realized_duals(results_em, input)))
        @test duals_value isa DataFrames.DataFrame
        @test length(unique(duals_value.DateTime)) == 576
    end

    expressions_keys = collect(keys(read_realized_expressions(results_em)))
    @test length(expressions_keys) == 12
    expressions_inputs = (
        [
            # "ProductionCostExpression__HydroEnergyReservoir",
            "ProductionCostExpression__ThermalStandard",
        ],
        [
            # (ProductionCostExpression, HydroEnergyReservoir),
            (ProductionCostExpression, ThermalStandard),
        ],
    )
    for input in expressions_inputs
        expressions_value = first(values(read_realized_expressions(results_em, input)))
        @test expressions_value isa DataFrames.DataFrame
        @test length(unique(expressions_value.DateTime)) == 576
    end

    @test length(
        unique(
            read_realized_expression(
                results_em,
                "ProductionCostExpression__ThermalStandard",
            ).DateTime,
        ),
    ) == 576
    @test length(
        unique(
            read_realized_expression(
                results_em,
                ProductionCostExpression,
                ThermalStandard;
                len = 10,
            ).DateTime),
    ) == 10

    parameters_keys = collect(keys(read_realized_parameters(results_em)))
    @test length(parameters_keys) == 5
    parameters_inputs = (
        [
            "ActivePowerTimeSeriesParameter__PowerLoad",
            "ActivePowerTimeSeriesParameter__RenewableDispatch",
        ],
        [
            (ActivePowerTimeSeriesParameter, PowerLoad),
            (ActivePowerTimeSeriesParameter, RenewableDispatch),
        ],
    )
    for input in parameters_inputs
        parameters_value = first(values(read_realized_parameters(results_em, input)))
        @test parameters_value isa DataFrames.DataFrame
        @test length(unique(parameters_value.DateTime)) == 576
    end

    @test length(
        unique(
            read_realized_parameter(
                results_em,
                "ActivePowerTimeSeriesParameter__RenewableDispatch";
                len = 10,
            ).DateTime),
    ) == 10

    expected_vars = union(Set(ED_EXPECTED_VARS), UC_EXPECTED_VARS)
    @test isempty(setdiff(list_variable_names(results_em), expected_vars))
    all_vars = Set(keys(read_realized_variables(results_em)))
    @test isempty(setdiff(all_vars, expected_vars))

    variables_inputs = (
        ["ActivePowerVariable__ThermalStandard", "ActivePowerVariable__RenewableDispatch"],
        [(ActivePowerVariable, ThermalStandard), (ActivePowerVariable, RenewableDispatch)],
    )
    for input in variables_inputs
        vars = read_realized_variables(results_em, input; table_format = TableFormat.WIDE)
        var_keys = collect(keys(vars))
        @test length(var_keys) == 2
        @test first(var_keys) == "ActivePowerVariable__ThermalStandard"
        @test last(var_keys) == "ActivePowerVariable__RenewableDispatch"
        for val in values(vars)
            @test val isa DataFrames.DataFrame
            @test DataFrames.nrow(val) == 576
        end
    end

    start_time = first(results_em.timestamps) + Dates.Hour(1)
    len = 12
    @test DataFrames.nrow(
        read_realized_variable(
            results_em,
            ActivePowerVariable,
            ThermalStandard;
            start_time = start_time,
            len = len,
            table_format = TableFormat.WIDE,
        ),
    ) == len

    vars = read_realized_variables(
        results_em,
        variables_inputs[1];
        start_time = start_time,
        len = len,
        table_format = TableFormat.WIDE,
    )
    df = first(values(vars))
    @test length(unique(df.DateTime)) == len
    @test df[!, "DateTime"][1] == start_time

    @test_throws IS.InvalidValue read_realized_variables(
        results_em,
        variables_inputs[1],
        start_time = start_time,
        len = 100000,
        table_format = TableFormat.WIDE,
    )
    @test_throws IS.InvalidValue read_realized_variables(
        results_em,
        variables_inputs[1],
        start_time = start_time + Dates.Second(1),
        table_format = TableFormat.WIDE,
    )
    @test_throws IS.InvalidValue read_realized_variables(
        results_em,
        variables_inputs[1],
        start_time = start_time - Dates.Hour(1000),
        table_format = TableFormat.WIDE,
    )
    @test_throws IS.InvalidValue read_realized_variables(
        results_em,
        variables_inputs[1],
        len = 100000,
        table_format = TableFormat.WIDE,
    )

    @test isempty(results_em)
    load_results!(
        results_em;
        duals = duals_inputs[2],
        expressions = expressions_inputs[2],
        parameters = parameters_inputs[2],
        variables = variables_inputs[2],
    )
    @test !isempty(results_em)
    @test length(results_em) ==
          length(duals_inputs[2]) +
          length(expressions_inputs[2]) +
          length(parameters_inputs[2]) +
          length(variables_inputs[2])

    # Test that table_format is applied when reading from cache (regression test for GH issue)
    vars_wide_from_cache = read_realized_variables(
        results_em,
        variables_inputs[2];
        table_format = TableFormat.WIDE,
    )
    for val in values(vars_wide_from_cache)
        @test val isa DataFrames.DataFrame
        @test DataFrames.nrow(val) == 576
        @test :DateTime in propertynames(val)
        @test :name ∉ propertynames(val)
        @test :value ∉ propertynames(val)
        @test DataFrames.ncol(val) > 1  # DateTime + at least one component column
    end

    empty!(results_em)
    @test isempty(results_em)

    export_path = mktempdir(; cleanup = true)
    export_realized_results(results_em, export_path)

    var_name = "ActivePowerVariable__ThermalStandard"
    df = read_realized_variable(results_em, var_name)
    export_active_power_file = joinpath(export_path, "$(var_name).csv")
    export_df = PSI.read_dataframe(export_active_power_file)
    # TODO: results A bug in the code produces NaN after index 48.
    @test isapprox(df[48, :], export_df[48, :])
end

function test_simulation_results_from_file(path::AbstractString, c_sys5_hy_ed, c_sys5_hy_uc)
    results = SimulationResults(path, "no_cache")
    @test list_decision_problems(results) == ["ED", "UC"]
    results_uc = get_decision_problem_results(results, "UC")
    results_ed = get_decision_problem_results(results, "ED")

    # Verify this works without system.
    @test get_system(results_uc) === nothing
    @test length(read_realized_variables(results_uc)) == length(UC_EXPECTED_VARS)

    @test_throws IS.InvalidValue set_system!(results_uc, c_sys5_hy_ed)
    set_system!(results_ed, c_sys5_hy_ed)
    set_system!(results_uc, c_sys5_hy_uc)

    test_decision_problem_results_values(results_ed, results_uc, c_sys5_hy_ed, c_sys5_hy_uc)
end

function test_decision_problem_results_kwargs_handling(
    path::AbstractString,
    c_sys5_hy_ed,
    c_sys5_hy_uc,
)
    results = SimulationResults(path, "no_cache")
    @test list_decision_problems(results) == ["ED", "UC"]
    results_uc = get_decision_problem_results(results, "UC")
    results_ed = get_decision_problem_results(results, "ED")

    # Verify this works without system.
    @test get_system(results_uc) === nothing

    results_ed = get_decision_problem_results(results, "ED")
    @test isnothing(get_system(results_ed))

    results_ed = get_decision_problem_results(results, "ED"; populate_system = true)
    @test !isnothing(get_system(results_ed))
    @test PSY.get_units_base(get_system(results_ed)) == "NATURAL_UNITS"

    @test_throws IS.InvalidValue set_system!(results_uc, c_sys5_hy_ed)

    set_system!(results_ed, c_sys5_hy_ed)
    set_system!(results_uc, c_sys5_hy_uc)

    results_ed = get_decision_problem_results(
        results,
        "ED";
        populate_system = true,
        populate_units = IS.UnitSystem.DEVICE_BASE,
    )
    @test !isnothing(PSI.get_system(results_ed))
    @test PSY.get_units_base(get_system(results_ed)) == "DEVICE_BASE"

    @test_throws ArgumentError get_decision_problem_results(
        results,
        "ED";
        populate_system = false,
        populate_units = IS.UnitSystem.DEVICE_BASE,
    )

    test_decision_problem_results_values(results_ed, results_uc, c_sys5_hy_ed, c_sys5_hy_uc)
end

function compare_long_and_wide_realized_results_2d(func, args...; kwargs...)
    long_results = func(args...; table_format = TableFormat.LONG, kwargs...)
    wide_results = func(args...; table_format = TableFormat.WIDE, kwargs...)
    @test sort!(collect(keys(long_results))) == sort!(collect(keys(wide_results)))
    for (key, long_value) in long_results
        wide_value = wide_results[key]
        measure_vars = [x for x in names(wide_value) if x != "DateTime"]
        wide_converted = DataFrames.stack(
            wide_value,
            measure_vars;
            variable_name = :name,
            value_name = :value,
        )
        @test @orderby(wide_converted, :DateTime, :name) ==
              @orderby(long_value, :DateTime, :name)
    end
end

@testset "Test simulation results" begin
    for in_memory in (false, true)
        file_path = mktempdir(; cleanup = true)
        export_path = mktempdir(; cleanup = true)
        test_simulation_results(file_path, export_path; in_memory = in_memory)
    end
end

@testset "Test simulation results with system from store" begin
    file_path = mktempdir(; cleanup = true)
    export_path = mktempdir(; cleanup = true)
    c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
    c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
    in_memory = false
    sim = run_simulation(
        c_sys5_hy_uc,
        c_sys5_hy_ed,
        file_path,
        export_path;
        in_memory = in_memory,
    )
    results = SimulationResults(PSI.get_simulation_folder(sim))
    uc = get_decision_problem_results(results, "UC")
    ed = get_decision_problem_results(results, "ED")
    sys_uc = get_system!(uc)
    sys_ed = get_system!(ed)
    test_decision_problem_results(results, sys_ed, sys_uc, in_memory)
    test_emulation_problem_results(results, in_memory)
end

function read_result_names(results, key::PSI.OptimizationContainerKey)
    result_data = PSI.read_results_with_keys(
        results,
        [key];
        table_format = TableFormat.LONG,
    )
    first_result = only(values(result_data))
    columns_without_datetime = first_result[!, Not(:DateTime)]
    return Set(names(columns_without_datetime))
end
