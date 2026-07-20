import PowerSimulations:
    open_store,
    HdfSimulationStore,
    HDF_FILENAME,
    SimulationStoreParams,
    ModelStoreParams,
    SimulationModelStoreRequirements,
    CacheFlushRules,
    KiB,
    MiB,
    GiB,
    STORE_CONTAINER_VARIABLES,
    initialize_problem_storage!,
    add_rule!,
    write_result!,
    read_result,
    has_dirty,
    get_cache_hit_percentage

function _initialize!(store, sim, variables, model_defs, cache_rules)
    models = OrderedDict{Symbol, ModelStoreParams}()
    model_reqs = Dict{Symbol, SimulationModelStoreRequirements}()
    num_param_containers = 0
    for model in keys(model_defs)
        execution_count = model_defs[model]["execution_count"]
        horizon = model_defs[model]["horizon"]
        num_rows = execution_count * sim["num_steps"]
        resolution = model_defs[model]["resolution"]
        interval = model_defs[model]["interval"]

        model_params = ModelStoreParams(
            execution_count,
            IS.time_period_conversion(horizon),
            IS.time_period_conversion(interval),
            IS.time_period_conversion(resolution),
            model_defs[model]["base_power"],
            model_defs[model]["system_uuid"],
        )
        reqs = SimulationModelStoreRequirements()
        horizon_count = horizon ÷ resolution
        for (key, array) in model_defs[model]["variables"]
            reqs.variables[key] = Dict(
                "columns" => model_defs[model]["names"],
                "dims" =>
                    (horizon_count, length(model_defs[model]["names"][1]), num_rows),
            )
            keep_in_cache = variables[key]["keep_in_cache"]
            add_rule!(cache_rules, model, key, keep_in_cache)
        end

        models[model] = model_params
        model_reqs[model] = reqs
        num_param_containers += length(reqs.variables)
    end

    params = SimulationStoreParams(
        sim["initial_time"],
        sim["step_resolution"],
        sim["num_steps"],
        models,
        # Emulation Model Store requirements. No tests yet
        OrderedDict(
            :Emulator => ModelStoreParams(
                100, # Num Executions
                IS.time_period_conversion(Hour(1)),
                IS.time_period_conversion(Minute(5)), # Interval
                IS.time_period_conversion(Minute(5)), # Resolution
                100.0,
                Base.UUID("4076af6c-e467-56ae-b986-b466b2749572"),
            ),
        ),
    )
    em_reqs = SimulationModelStoreRequirements()
    initialize_problem_storage!(store, params, model_reqs, em_reqs, cache_rules)
    return
end

function _run_sim_test(path, sim, variables, model_defs, cache_rules, seed)
    rng = MersenneTwister(seed)
    open_store(HdfSimulationStore, path, "w") do store
        sim_time = sim["initial_time"]
        _initialize!(store, sim, variables, model_defs, cache_rules)
        for _ in 1:sim["num_steps"]
            for model in keys(model_defs)
                model_time = sim_time
                for i in 1:model_defs[model]["execution_count"]
                    for key in keys(variables)
                        data = rand(rng, size(model_defs[model]["variables"][key])...)
                        columns = model_defs[model]["names"]
                        write_result!(
                            store,
                            model,
                            key,
                            model_time,
                            model_time,
                            Containers.DenseAxisArray(
                                permutedims(data),
                                columns,
                                1:size(data)[1],
                            ),
                        )
                        _verify_data(data, store, model, key, model_time, columns)
                    end

                    model_time += model_defs[model]["resolution"]
                end
            end

            sim_time += sim["step_resolution"]
        end

        for output_cache in values(store.cache.data)
            if PSI.should_keep_in_cache(output_cache)
                @test get_cache_hit_percentage(output_cache) == 100.0
            else
                @test get_cache_hit_percentage(output_cache) < 100.0
            end
        end

        flush(store)
        @test !has_dirty(store.cache)
    end
end

function _verify_read_results(path, sim, variables, model_defs, seed)
    rng = MersenneTwister(seed)
    open_store(HdfSimulationStore, path, "r") do store
        sim_time = sim["initial_time"]
        for _ in 1:sim["num_steps"]
            for model in keys(model_defs)
                model_time = sim_time
                for i in 1:model_defs[model]["execution_count"]
                    for key in keys(variables)
                        data = rand(rng, size(model_defs[model]["variables"][key])...)
                        columns = model_defs[model]["names"]
                        _verify_data(data, store, model, key, model_time, columns)
                    end

                    model_time += model_defs[model]["resolution"]
                end
            end

            sim_time += sim["step_resolution"]
        end

        for output_cache in values(store.cache.data)
            @test get_cache_hit_percentage(output_cache) == 0.0
        end
    end
end

function _verify_data(expected, store, model, name, time, columns::Tuple{Vector{Symbol}})
    expected_df = DataFrames.DataFrame(expected, columns[1])
    df = read_result(DataFrames.DataFrame, store, model, name, time)
    @test expected_df == df
end

@testset "Test SimulationStore 2-d arrays" begin
    sim = Dict(
        "initial_time" => Dates.DateTime("2020-01-01T00:00:00"),
        "step_resolution" => Dates.Hour(24),
        "num_steps" => 50,
    )
    variables = Dict(
        PSI.VariableKey(ActivePowerVariable, ThermalStandard) =>
            Dict("keep_in_cache" => true),
        PSI.VariableKey(ActivePowerVariable, RenewableDispatch) =>
            Dict("keep_in_cache" => true),
        PSI.VariableKey(ActivePowerVariable, InterruptiblePowerLoad) =>
            Dict("keep_in_cache" => false),
        PSI.VariableKey(ActivePowerVariable, RenewableNonDispatch) =>
            Dict("keep_in_cache" => false),
    )
    model_defs = OrderedDict(
        :ED => Dict(
            "execution_count" => 24,
            "horizon" => Hour(12),
            "names" => ([:dev1, :dev2, :dev3, :dev4, :dev5],),
            "variables" => Dict(x => ones(12, 5) for x in keys(variables)),
            "interval" => Dates.Hour(1),
            "resolution" => Dates.Hour(1),
            "base_power" => 100.0,
            "system_uuid" => Base.UUID("4076af6c-e467-56ae-b986-b466b2749572"),
        ),
        :UC => Dict(
            "execution_count" => 1,
            "horizon" => Hour(24),
            "names" => ([:dev1, :dev2, :dev3],),
            "variables" => Dict(x => ones(24, 3) for x in keys(variables)),
            "interval" => Dates.Hour(1),
            "resolution" => Dates.Hour(1),
            "base_power" => 100.0,
            "system_uuid" => Base.UUID("4076af6c-e467-56ae-b986-b466b2749572"),
        ),
    )
    cache_rules = CacheFlushRules(; max_size = 1 * MiB, min_flush_size = 4 * KiB)

    path = mktempdir()
    # Use this seed to produce the same randomly generated arrays for write and verify.
    seed = 1234
    _run_sim_test(path, sim, variables, model_defs, cache_rules, seed)
    _verify_read_results(path, sim, variables, model_defs, seed)
end

@testset "Test OptimizationOutputCache" begin
    key = PSI.OptimizationResultCacheKey(
        :ED,
        PSI.VariableKey(ActivePowerVariable, InterruptiblePowerLoad),
    )
    cache = PSI.OptimizationOutputCache(key, PSI.CacheFlushRule(true))
    @test !PSI.has_clean(cache)
    @test !PSI.is_dirty(cache, Dates.now())

    timestamp1 = Dates.DateTime("2020-01-01T00:00:00")
    timestamp2 = Dates.DateTime("2020-01-01T01:00:00")
    timestamp3 = Dates.DateTime("2020-01-01T02:00:00")
    timestamp4 = Dates.DateTime("2020-01-01T03:00:00")
    PSI.add_result!(cache, timestamp1, ones(2), false)
    @test PSI.is_dirty(cache, timestamp1)
    PSI.add_result!(cache, timestamp2, ones(2), false)
    @test PSI.is_dirty(cache, timestamp2)

    @test_throws IS.InvalidValue PSI.add_result!(cache, timestamp2, ones(2), false)

    @test length(cache.data) == 2
    @test length(cache.dirty_timestamps) == 2

    popfirst!(cache.dirty_timestamps)
    @test !PSI.is_dirty(cache, timestamp1)
    @test PSI.has_clean(cache)
    @test length(cache.data) == 2
    @test length(cache.dirty_timestamps) == 1

    PSI.add_result!(cache, timestamp3, ones(2), false)
    @test length(cache.data) == 3
    @test length(cache.dirty_timestamps) == 2

    PSI.add_result!(cache, timestamp4, ones(2), true)
    @test length(cache.data) == 3
    @test length(cache.dirty_timestamps) == 3

    popfirst!(cache.dirty_timestamps)
    @test PSI.has_clean(cache)

    empty!(cache.dirty_timestamps)
    empty!(cache)
    @test isempty(cache.data)
    @test isempty(cache.dirty_timestamps)

    PSI.add_result!(cache, timestamp1, ones(2), false)
    PSI.add_result!(cache, timestamp2, ones(2), false)
    PSI.discard_results!(cache, [timestamp1, timestamp2])
    @test isempty(cache.data)
end

@testset "Emulation store SparseAxisArray write/read" begin
    key = PSI.VariableKey(ActivePowerVariable, ThermalStandard)
    cols = [("o1", "br1"), ("o1", "br2"), ("o2", "br1")]
    n_exec = 5
    em_reqs = SimulationModelStoreRequirements()
    em_reqs.variables[key] = Dict(
        "columns" => (PSI.encode_tuple_to_column.(cols),),
        "dims" => (n_exec, length(cols)),
    )
    params = SimulationStoreParams(
        Dates.DateTime("2020-01-01T00:00:00"),
        Dates.Hour(1),
        1,
        OrderedDict{Symbol, ModelStoreParams}(),
        OrderedDict(
            :Emulator => ModelStoreParams(
                n_exec,
                IS.time_period_conversion(Hour(1)),
                IS.time_period_conversion(Minute(5)),
                IS.time_period_conversion(Minute(5)),
                100.0,
                Base.UUID("4076af6c-e467-56ae-b986-b466b2749572"),
            ),
        ),
    )
    path = joinpath(mktempdir(), HDF_FILENAME)
    written = Dict{Int, Vector{Float64}}()
    open_store(HdfSimulationStore, path, "w") do store
        initialize_problem_storage!(
            store,
            params,
            Dict{Symbol, SimulationModelStoreRequirements}(),
            em_reqs,
            CacheFlushRules(),
        )
        t = params.initial_time
        for ix in 1:n_exec
            vals = Float64.(ix .* (1:length(cols)))
            arr = Containers.SparseAxisArray(
                Dict((cols[i]..., 1) => vals[i] for i in eachindex(cols)),
            )
            write_result!(store, :Emulator, key, ix, t, arr)
            written[ix] = vals
            t += Minute(5)
        end
    end
    open_store(HdfSimulationStore, path, "r") do store
        result = PSI.read_results(store, key)
        for ix in 1:n_exec, i in eachindex(cols)
            @test result[PSI.encode_tuple_to_column(cols[i]), ix] ≈ written[ix][i]
        end
    end
end

# TODO: test optimizer stats
# TODO: unit tests of individual functions, size checks
# TODO: profiling of memory performance and GC
