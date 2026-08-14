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

    # TODO: Can emulation model results be validated?

    # The checks below sabotage the partition outputs and so must be last.
    base_dir = joinpath(sim_dir, partition_name)
    partition_results = PSI.SimulationPartitionResults(base_dir)
    joined_status_path = joinpath(base_dir, PSI.RESULTS_DIR)
    num_partitions = get_num_partitions(partition_results.partitions)
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
            for output_type in string.(PSI.STORE_CONTAINERS)
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
            @test nrow(pre_df) % num_partitions == 0
            rows_per_step = nrow(pre_df) ÷ num_partitions
            skipped_rows =
                (rows_per_step * (failed_index - 1) + 1):(rows_per_step * failed_index)
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
    PSI.process_simulation_partition_cli_args(
        build_simulation,
        execute_simulation,
        cli_args...,
        "--skip-failures",
    )
    @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED

    # Options must be rejected if they are malformed or have an invalid value.
    for invalid_option in ("--skip-failures=maybe", "--simulation-name=a=b")
        @test_throws ErrorException PSI.process_simulation_partition_cli_args(
            build_simulation,
            execute_simulation,
            cli_args...,
            invalid_option,
        )
    end

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

    # A job whose status cannot be read must be treated as a failure.
    status_file = joinpath(partition_status_path(corrupted_index), "status.json")
    status_backup = status_file * ".backup"
    mv(status_file, status_backup)
    try
        @test_throws ErrorException PSI.join_simulation(base_dir)
        @test PSI.deserialize_status(joined_status_path) == PSI.RunStatus.FAILED
        @test PSI.join_simulation(base_dir; skip_failures = true) == PSI.RunStatus.FAILED
    finally
        mv(status_backup, status_file; force = true)
    end

    # With every job successful again, the join must succeed.
    @test PSI.join_simulation(base_dir) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    @test PSI.deserialize_status(joined_status_path) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
end
