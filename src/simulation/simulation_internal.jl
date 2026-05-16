mutable struct SimulationInternal
    sim_files_dir::String
    partitions::Union{Nothing, SimulationPartitions}
    store_dir::String
    logs_dir::String
    models_dir::String
    recorder_dir::String
    results_dir::String
    partitions_dir::String
    run_count::OrderedDict{Int, OrderedDict{Int, Int}}
    date_ref::OrderedDict{Int, Dates.DateTime}
    status::RunStatus
    build_status::SimulationBuildStatus
    simulation_state::SimulationState
    store::Union{Nothing, SimulationStore}
    recorders::Vector{Symbol}
    console_level::Base.CoreLogging.LogLevel
    file_level::Base.CoreLogging.LogLevel
    cache_size_mib::Int
    min_cache_flush_size_mib::Int
    rng::AbstractRNG
end

function SimulationInternal(
    steps::Int,
    models::SimulationModels,
    base_dir::String,
    name::String,
    recorders,
    console_level::Logging.LogLevel,
    file_level::Logging.LogLevel;
    partitions::Union{Nothing, SimulationPartitions} = nothing,
    cache_size_mib = 1024,
    min_cache_flush_size_mib = MIN_CACHE_FLUSH_SIZE_MiB,
)
    count_dict = OrderedDict{Int, OrderedDict{Int, Int}}()

    for s in 1:steps
        count_dict[s] = OrderedDict{Int, Int}()
        model_count = length(get_decision_models(models))
        for st in 1:model_count
            count_dict[s][st] = 0
        end
        if !isnothing(get_emulation_model(models))
            count_dict[s][model_count + 1] = 0
        end
    end

    simulation_dir = joinpath(base_dir, name)
    if isdir(simulation_dir)
        simulation_dir = _get_output_dir_name(base_dir, name)
    end

    sim_files_dir = joinpath(simulation_dir, "simulation_files")
    store_dir = joinpath(simulation_dir, "data_store")
    logs_dir = joinpath(simulation_dir, "logs")
    models_dir = joinpath(simulation_dir, "problems")
    recorder_dir = joinpath(simulation_dir, "recorder")
    results_dir = joinpath(simulation_dir, RESULTS_DIR)
    partitions_dir = joinpath(simulation_dir, "simulation_partitions")

    unique_recorders = Set(REQUIRED_RECORDERS)
    foreach(x -> push!(unique_recorders, x), recorders)

    return SimulationInternal(
        sim_files_dir,
        partitions,
        store_dir,
        logs_dir,
        models_dir,
        recorder_dir,
        results_dir,
        partitions_dir,
        count_dict,
        OrderedDict{Int, Dates.DateTime}(),
        RunStatus.NOT_READY,
        SimulationBuildStatus.EMPTY,
        SimulationState(),
        nothing,
        collect(unique_recorders),
        console_level,
        file_level,
        cache_size_mib,
        min_cache_flush_size_mib,
        Random.Xoshiro(IS.get_random_seed()),
    )
end

function make_dirs(internal::SimulationInternal)
    mkdir(dirname(internal.sim_files_dir))
    for field in
        (:sim_files_dir, :store_dir, :logs_dir, :models_dir, :recorder_dir, :results_dir)
        mkdir(getproperty(internal, field))
    end
end

function _get_output_dir_name(path, sim_name)
    index = _get_most_recent_execution(path, sim_name) + 1
    return joinpath(path, "$sim_name-$index")
end

function _get_most_recent_execution(path, sim_name)
    sim_dirs = readdir(path)
    if isempty(sim_dirs)
        fail = true
    elseif length(sim_dirs) == 1
        fail = sim_dirs[1] != sim_name
    else
        fail = false
    end

    fail && error("No simulation directories with name=$sim_name are in $path")
    executions = [1]
    for path_name in sim_dirs
        regex = Regex("\\Q$sim_name\\E-(\\d+)\$")
        m = match(regex, path_name)
        if !isnothing(m)
            push!(executions, parse(Int, m.captures[1]))
        end
    end

    return maximum(executions)
end

function configure_logging(internal::SimulationInternal, file_mode)
    return IS.configure_logging(;
        console = true,
        console_stream = stderr,
        console_level = internal.console_level,
        file = true,
        filename = joinpath(internal.logs_dir, SIMULATION_LOG_FILENAME),
        file_level = internal.file_level,
        file_mode = file_mode,
        tracker = nothing,
        set_global = false,
    )
end

function register_recorders!(internal::SimulationInternal, file_mode)
    for name in internal.recorders
        IS.register_recorder!(name; mode = file_mode, directory = internal.recorder_dir)
    end
end

function unregister_recorders!(internal::SimulationInternal)
    for name in internal.recorders
        IS.unregister_recorder!(name)
    end
end
