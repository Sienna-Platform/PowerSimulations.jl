@testset "Test partitions and step ranges" begin
    partitions = SimulationPartitions(2, 1, 0)
    @test PSI.get_absolute_step_range(partitions, 1) == 1:1
    @test PSI.get_valid_step_offset(partitions, 1) == 1
    @test PSI.get_valid_step_length(partitions, 1) == 1
    @test PSI.get_absolute_step_range(partitions, 2) == 2:2
    @test PSI.get_valid_step_offset(partitions, 2) == 1
    @test PSI.get_valid_step_length(partitions, 2) == 1

    partitions = SimulationPartitions(365, 7, 1)
    @test get_num_partitions(partitions) == 53
    @test PSI.get_absolute_step_range(partitions, 1) == 1:7
    @test PSI.get_valid_step_offset(partitions, 1) == 1
    @test PSI.get_valid_step_length(partitions, 1) == 7
    @test PSI.get_absolute_step_range(partitions, 2) == 7:14
    @test PSI.get_valid_step_offset(partitions, 2) == 2
    @test PSI.get_valid_step_length(partitions, 2) == 7
    @test PSI.get_absolute_step_range(partitions, 52) == 357:364
    @test PSI.get_valid_step_offset(partitions, 52) == 2
    @test PSI.get_valid_step_length(partitions, 52) == 7
    @test PSI.get_absolute_step_range(partitions, 53) == 364:365
    @test PSI.get_valid_step_offset(partitions, 53) == 2
    @test PSI.get_valid_step_length(partitions, 53) == 1

    @test_throws ErrorException PSI.get_absolute_step_range(partitions, -1)
    @test_throws ErrorException PSI.get_absolute_step_range(partitions, 54)
end

@testset "Test interrupt detection" begin
    remote = PSI.Distributed.RemoteException(
        1,
        CapturedException(InterruptException(), Any[]),
    )
    @test PSI._is_interrupt(InterruptException())
    @test PSI._is_interrupt(remote)
    interrupt_task = @async throw(InterruptException())
    try
        wait(interrupt_task)
    catch
    end
    failed_task = @async error("failed")
    try
        wait(failed_task)
    catch
    end
    @test PSI._is_interrupt(TaskFailedException(interrupt_task))
    @test !PSI._is_interrupt(TaskFailedException(failed_task))
    @test PSI._is_interrupt(CompositeException([ErrorException("failed"), remote]))
    @test !PSI._is_interrupt(ErrorException("failed"))
    @test !PSI._is_interrupt(
        CompositeException([
            PSI.Distributed.RemoteException(
                1,
                CapturedException(ErrorException("failed"), Any[]),
            ),
        ]),
    )
end

@testset "Test simulation partitions" begin
    sim_dir = mktempdir()
    script = joinpath(BASE_DIR, "test", "run_partitioned_simulation.jl")
    include(script)

    partition_name = "partitioned"
    run_parallel_simulation(
        build_simulation,
        execute_simulation;
        script = script,
        output_dir = sim_dir,
        name = partition_name,
        num_steps = 3,
        period = 1,
        num_overlap_steps = 1,
        # Running multiple processes in CI can kill the VM.
        num_parallel_processes = haskey(ENV, "CI") ? 1 : 3,
        exeflags = "--project=test",
        force = true,
    )

    regular_name = "regular"
    regular_sim = build_simulation(
        sim_dir,
        regular_name;
        initial_time = DateTime("2024-01-02T00:00:00"),
        num_steps = 1,
        HiGHS_optimizer = HiGHS_optimizer,
    )
    @test execute_simulation(regular_sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    regular_results = SimulationResults(sim_dir, regular_name)
    partitioned_results = SimulationResults(sim_dir, partition_name)

    functions = (
        read_realized_aux_variables,
        read_realized_expressions,
        read_realized_parameters,
        read_realized_variables,
    )
    key_strings_to_skip = ("Flow", "On", "Off", "Shut", "Start", "Stop")
    for name in ("ED", "UC")
        regular_model_results = get_decision_problem_results(regular_results, name)
        partitioned_model_results = get_decision_problem_results(partitioned_results, name)

        for func in functions
            regular = func(regular_model_results; table_format = TableFormat.WIDE)
            partitioned = func(partitioned_model_results; table_format = TableFormat.WIDE)
            @test sort(collect(keys(regular))) == sort(collect(keys(partitioned)))
            for key in keys(regular)
                t_start = regular[key][1, 1]
                t_end = regular[key][end, 1]
                rdf = regular[key]
                pdf = partitioned[key]
                pdf = pdf[(pdf.DateTime .>= t_start) .& (pdf.DateTime .<= t_end), :]
                @test nrow(rdf) == nrow(pdf)
                @test ncol(rdf) == ncol(pdf)
                skip = false
                for key_string_to_skip in key_strings_to_skip
                    if occursin(key_string_to_skip, key)
                        skip = true
                        break
                    end
                end
                skip && continue
                r_sum = 0
                p_sum = 0
                atol =
                    if (
                        occursin("ProductionCostExpression", key) ||
                        occursin("FuelCostExpression__ThermalStandard", key)
                    )
                        11000
                    else
                        1e-6
                    end
                for i in 2:ncol(rdf)
                    r_sum += sum(rdf[!, i])
                    p_sum += sum(pdf[!, i])
                end
                if !isapprox(r_sum, p_sum; atol = atol)
                    @error "Mismatch" r_sum p_sum key
                end
                @test isapprox(r_sum, p_sum, atol = atol)
            end
        end
    end

    # The merged bundle's System carries the input windows of every partition.
    uc_joined = get_decision_problem_results(partitioned_results, "UC")
    restored = get_system!(uc_joined)
    ren = first(get_components(PSY.RenewableDispatch, restored))
    fc = PSY.get_time_series(PSY.Deterministic, ren, "max_active_power")
    @test sort(collect(keys(IS.get_data(fc)))) == collect(uc_joined.timestamps)
    # InfraStore allows only one open handle per file per process; the sabotage tests below
    # reopen this same bundle for writing, so this read-only handle must not outlive this check.
    IS.close!(IS.get_data_store(restored.data))

    base_dir = joinpath(sim_dir, partition_name)
    partition_results = PSI.SimulationPartitionResults(base_dir)
    num_partitions = get_num_partitions(partition_results.partitions)

    # Parameters have no HDF5-backed dataset to compare here (Task 8+9): they live in each
    # model's bundle InfraStore sidecar, already compared above via read_realized_parameters.
    dataset_output_types =
        string.(filter(!=(PSI.STORE_CONTAINER_PARAMETERS), PSI.STORE_CONTAINERS))

    function compare_store_dataset(index, src_dataset, dst_dataset, step_dim)
        step_range = PSI.get_absolute_step_range(partition_results.partitions, index)
        per_step = size(src_dataset, step_dim) ÷ length(step_range)
        valid_range = PSI._valid_step_range(partition_results, index)
        len = length(valid_range) * per_step
        src_start = 1 + per_step * (first(valid_range) - first(step_range))
        dst_start = 1 + per_step * (first(valid_range) - 1)
        src_range = src_start:(src_start + len - 1)
        dst_range = dst_start:(dst_start + len - 1)
        src_indexes = ntuple(d -> d == step_dim ? src_range : Colon(), ndims(src_dataset))
        dst_indexes = ntuple(d -> d == step_dim ? dst_range : Colon(), ndims(dst_dataset))
        @test isequal(dst_dataset[dst_indexes...], src_dataset[src_indexes...])
    end

    function compare_store_group(index, partition_store, merged_store, group_path, step_dim)
        @test sort(collect(keys(partition_store[group_path]))) ==
              sort(collect(keys(merged_store[group_path])))
        for dst_dataset in merged_store[group_path]
            name = PSI.HDF5.name(dst_dataset)
            endswith(name, "__columns") && continue
            compare_store_dataset(
                index,
                partition_store[name],
                dst_dataset,
                step_dim(dst_dataset),
            )
        end
    end

    PSI.HDF5.h5open(PSI._store_path(partition_results), "r") do merged_store
        for index in 1:num_partitions
            PSI.HDF5.h5open(
                joinpath(
                    PSI._partition_path(partition_results, index),
                    PSI._store_subpath(),
                ),
                "r",
            ) do partition_store
                @test sort(collect(keys(partition_store["simulation/decision_models"]))) ==
                      sort(collect(keys(merged_store["simulation/decision_models"])))
                for merged_group in merged_store["simulation/decision_models"]
                    group_path = PSI.HDF5.name(merged_group)
                    for output_type in dataset_output_types
                        compare_store_group(
                            index,
                            partition_store,
                            merged_store,
                            "$group_path/$output_type",
                            ndims,
                        )
                    end
                    compare_store_dataset(
                        index,
                        partition_store["$group_path/optimizer_stats"],
                        merged_store["$group_path/optimizer_stats"],
                        ndims(merged_store["$group_path/optimizer_stats"]),
                    )
                end
                for output_type in dataset_output_types
                    compare_store_group(
                        index,
                        partition_store,
                        merged_store,
                        "simulation/emulation_model/$output_type",
                        _ -> 1,
                    )
                end
            end
        end
    end

    # TODO: Can emulation model results be validated through the public results APIs?

    # The checks below sabotage the partition outputs and so must be last.
    joined_status_path = joinpath(base_dir, PSI.RESULTS_DIR)
    partition_status_path(index) =
        joinpath(PSI._partition_path(partition_results, index), PSI.RESULTS_DIR)
    read_realized(results) = Dict(
        name => read_realized_variables(
            get_decision_problem_results(results, name);
            table_format = TableFormat.WIDE,
        ) for name in ("UC", "ED")
    )

    pre_join_variables = read_realized(partitioned_results)
    failed_index = 2

    # Joining a simulation with a failed partition must fail and record the failure.
    PSI.serialize_status(PSI.RunStatus.FAILED, partition_status_path(failed_index))
    @test_throws ErrorException PSI.join_simulation(base_dir)
    @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED

    # Fill the merged store with a sentinel value in order to detect the steps that the
    # next join writes. A skipped partition must not shift the data of the partitions
    # that follow it.
    sentinel = -9999.0
    PSI.HDF5.h5open(joinpath(base_dir, "data_store", "simulation_store.h5"), "r+") do store
        for group in store["simulation/decision_models"]
            for output_type in dataset_output_types
                for dataset in group[output_type]
                    endswith(PSI.HDF5.name(dataset), "__columns") && continue
                    if ndims(dataset) == 2
                        dataset[:, :] = fill(sentinel, size(dataset))
                    elseif ndims(dataset) == 3
                        dataset[:, :, :] = fill(sentinel, size(dataset))
                    else
                        error("Unsupported dataset ndims: $(ndims(dataset))")
                    end
                end
            end
        end
    end
    # The results are read through the same conversions as any other results, so record
    # what the sentinel looks like after them instead of assuming that it is unchanged.
    store_dir = joinpath(base_dir, "data_store")
    PSI.IS.compute_file_hash(store_dir, "simulation_store.h5")
    sentinel_variables =
        read_realized(SimulationResults(sim_dir, partition_name; ignore_status = true))

    @test PSI.join_simulation(base_dir; skip_failures = true) == PSI.RunStatus.FAILED
    @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED
    # The results of the successful partitions must still be readable.
    post_join_results = SimulationResults(sim_dir, partition_name; ignore_status = true)
    post_join_variables = read_realized(post_join_results)
    for (model_name, pre_variables) in pre_join_variables
        post_variables = post_join_variables[model_name]
        @test sort(collect(keys(pre_variables))) == sort(collect(keys(post_variables)))
        for (key, pre_df) in pre_variables
            post_df = post_variables[key]
            @test nrow(post_df) == nrow(pre_df)
            num_steps = partition_results.partitions.num_steps
            @test nrow(pre_df) % num_steps == 0
            rows_per_step = nrow(pre_df) ÷ num_steps
            skipped_steps = PSI._valid_step_range(partition_results, failed_index)
            skipped_rows =
                (rows_per_step * (first(skipped_steps) - 1) + 1):(rows_per_step * last(
                    skipped_steps,
                ))
            merged_rows = setdiff(1:nrow(pre_df), skipped_rows)
            # The successful partitions must be merged at their original steps.
            @test post_df[merged_rows, :] == pre_df[merged_rows, :]
            # The steps of the skipped partition must not be written. A join that writes
            # nothing at all cannot pass this because the merged rows would hold the
            # sentinel too.
            @test post_df[skipped_rows, :] ==
                  sentinel_variables[model_name][key][skipped_rows, :]
        end
    end

    # The join command must behave the same way, with --skip-failures passed as a flag.
    cli_args = ("join", "--simulation-name=$partition_name", "--output-dir=$sim_dir")
    @test_throws ErrorException PSI.process_simulation_partition_cli_args(
        build_simulation,
        execute_simulation,
        cli_args...,
    )
    @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED
    # The command must fail even with --skip-failures so that scripts detect the failure,
    # but only after merging the results of the successful jobs.
    @test_throws ErrorException PSI.process_simulation_partition_cli_args(
        build_simulation,
        execute_simulation,
        cli_args...,
        "--skip-failures",
    )
    @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED

    # Options must be rejected if they have an invalid value or are missing a required
    # value.
    for invalid_option in ("--skip-failures=maybe", "--simulation-name")
        @test_throws ErrorException PSI.process_simulation_partition_cli_args(
            build_simulation,
            execute_simulation,
            cli_args...,
            invalid_option,
        )
    end

    # Option values may contain '=': only the first '=' separates the name from the
    # value. The parse succeeds and the failure comes from the nonexistent simulation
    # named "a=b", not from option parsing.
    parse_err = try
        PSI.process_simulation_partition_cli_args(
            build_simulation,
            execute_simulation,
            "join",
            "--simulation-name=a=b",
            "--output-dir=$sim_dir",
        )
        nothing
    catch e
        e
    end
    @test parse_err !== nothing
    @test !occursin("Invalid option", sprint(showerror, parse_err))

    PSI.serialize_status(
        PSI.RunStatus.SUCCESSFULLY_FINALIZED,
        partition_status_path(failed_index),
    )

    # A store file that cannot be read must be handled like a failed job, even if the job
    # reported success.
    corrupted_index = num_partitions
    store_file = joinpath(
        PSI._partition_path(partition_results, corrupted_index),
        "data_store",
        "simulation_store.h5",
    )
    store_backup = store_file * ".backup"
    cp(store_file, store_backup)
    try
        write(store_file, "not an HDF5 file")
        @test_throws Exception PSI.join_simulation(base_dir)
        @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED
        @test PSI.join_simulation(base_dir; skip_failures = true) == PSI.RunStatus.FAILED
        @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED
    finally
        mv(store_backup, store_file; force = true)
    end

    # A job whose status file is missing must be treated as a failure.
    status_file = joinpath(partition_status_path(corrupted_index), "status.json")
    status_backup = status_file * ".backup"
    mv(status_file, status_backup)
    try
        @test_throws ErrorException PSI.join_simulation(base_dir)
        @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED
        @test PSI.join_simulation(base_dir; skip_failures = true) == PSI.RunStatus.FAILED
        # A status file that exists but cannot be read indicates a problem with the
        # environment rather than a failed partition job, and must propagate even with
        # skip_failures = true.
        write(status_file, "not JSON")
        @test_throws Exception PSI.join_simulation(base_dir)
        @test_throws Exception PSI.join_simulation(base_dir; skip_failures = true)
    finally
        rm(status_file; force = true)
        mv(status_backup, status_file; force = true)
    end

    # With every job successful again, the join must succeed.
    @test PSI.join_simulation(base_dir) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    @test PSI.deserialize_status(joined_status_path) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED

    # The Emulator aggregator borrows one decision model's bundle (it has none of its own); that
    # bundle must carry both a Deterministic input row (the decision model's own) and a
    # SingleTimeSeries input row (the Emulator's) for devices both templates read -- proof that
    # the type-scoped write-once fix (POM `_input_row_exists`) lets the two coexist instead of
    # one silently blocking the other. Find which decision model that is rather than assuming.
    emulator_model_name = PSI.open_store(
        PSI.HdfSimulationStore,
        joinpath(base_dir, PSI.STORE_DIR),
        "r",
    ) do store
        em_bundle = PSI._emulation_bundle_dir(store)
        for name in keys(store.params.decision_models_params)
            params = PSI.get_decision_model_params(store, name)
            PSI._bundle_dir(store, name, PSI.get_system_uuid(params)) == em_bundle &&
                return name
        end
        return nothing
    end
    @test emulator_model_name !== nothing
    final_results = SimulationResults(sim_dir, partition_name)
    restored = get_system!(
        get_decision_problem_results(final_results, string(emulator_model_name)),
    )
    input_rows = POM.list_input_series(POM.parameter_store_of(restored))
    @test any(md -> IS.get_time_series_type(md) <: PSY.Deterministic, input_rows)
    @test any(md -> IS.get_time_series_type(md) <: PSY.SingleTimeSeries, input_rows)
    IS.close!(IS.get_data_store(restored.data))
end

@testset "_write_decision_model_inputs! warns instead of silently dropping new windows" begin
    store = POM.ParameterTimeSeriesStore()
    t0 = Dates.DateTime(2024, 1, 1)
    resolution = Dates.Hour(1)
    interval = Dates.Hour(24)
    # Seed the destination with one window, as an earlier partial join would have written.
    POM.write_input_forecast_row!(
        store, 7, "RenewableDispatch", "max_active_power",
        Dict(t0 => collect(1.0:24.0)), resolution, interval,
    )
    # This merge's accumulated data holds a second window the existing row lacks.
    accumulated = Dict{Tuple{Int64, String, String}, Dict{Dates.DateTime, Vector{Float64}}}(
        (7, "RenewableDispatch", "max_active_power") =>
            Dict{Dates.DateTime, Vector{Float64}}(
                t0 => collect(2.0:25.0),
                t0 + interval => collect(3.0:26.0),
            ),
    )
    @test_logs(
        (:warn, r"input series whose merged-bundle row already exists"),
        match_mode = :any,
        PSI._write_decision_model_inputs!(store, accumulated, :UC, resolution, interval),
    )
    # Write-once: the existing row is left exactly as it was, neither extended nor overwritten.
    row = only(POM.list_input_series(store))
    ts = POM.read_input_time_series(store, row)
    @test collect(keys(IS.get_data(ts))) == [t0]
    @test IS.get_data(ts)[t0] == collect(1.0:24.0)
    POM.close_parameter_store!(store)
end
