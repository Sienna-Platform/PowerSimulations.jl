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
end
