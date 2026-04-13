"""
    Simulation(
        sequence::SimulationSequence,
        name::String,
        steps::Int
        models::SimulationModels,
        simulation_folder::String,
        initial_time::Union{Nothing, Dates.DateTime}
    )

Construct the `Simulation` structure to run the sequence of decision and emulation models specified.

# Arguments

  - `sequence::SimulationSequence`: Simulation sequence that specify how the decision and emulation models will be executed.
  - `name::String`: Name of the Simulation
  - `steps::Int`: Number of steps on which the sequence of models will be executed
  - `models::SimulationModels`: List of Decision and Emulation Models
  - `simulation_folder::String`: Folder on which results will be stored
  - `initial_time::Union{Nothing, Dates.DateTime} = nothing`: Initial time of which the
    simulation starts. If nothing it will default to the first timestamp of time series of the system.

# Example

```julia
template_uc = template_unit_commitment()
template_ed = template_economic_dispatch()
my_decision_model_uc = DecisionModel(template_1, sys_uc, optimizer, name = "UC")
my_decision_model_ed = DecisionModel(template_ed, sys_ed, optimizer, name = "ED")
models = SimulationModels(
    decision_models = [
        my_decision_model_uc,
        my_decision_model_ed
    ]
)
# The following sequence set the commitment variables (`OnVariable`) for `ThermalStandard` units from UC to ED.
sequence = SimulationSequence(;
    models = models,
    feedforwards = Dict(
        "ED" => [
            SemiContinuousFeedforward(;
                component_type = ThermalStandard,
                source = OnVariable,
                affected_values = [ActivePowerVariable],
            ),
        ],
    ),
)

sim = Simulation(
    sequence = sequence,
    name = "Sim",
    steps = 5,
    models = models,
    simulation_folder = mktempdir(cleanup=true),
)
```
"""
mutable struct Simulation
    steps::Int
    models::SimulationModels
    initial_time::Union{Nothing, Dates.DateTime}
    sequence::SimulationSequence
    simulation_folder::String
    name::String
    internal::Union{Nothing, SimulationInternal}

    function Simulation(;
        sequence::SimulationSequence,
        name::String,
        steps::Int,
        models::SimulationModels,
        simulation_folder::AbstractString,
        initial_time = nothing,
    )
        for model in get_decision_models(models)
            if get_sequence_uuid(model) != sequence.uuid
                model_name = get_name(model)
                throw(
                    IS.ConflictingInputsError(
                        "The decision model definition for $model_name doesn't correspond to the simulation sequence",
                    ),
                )
            end
        end
        em = get_emulation_model(models)
        if em !== nothing
            if get_sequence_uuid(em) != sequence.uuid
                model_name = get_name(em)
                throw(
                    IS.ConflictingInputsError(
                        "The emulation model definition for $model_name doesn't correspond to the simulation sequence",
                    ),
                )
            end
        end
        new(steps, models, initial_time, sequence, simulation_folder, name, nothing)
    end
end

###################### Simulation Accessor Functions ####################
function get_base_powers(sim::Simulation)
    base_powers = Dict()
    for model in get_models(sim)
        base_powers[get_name(model)] = PSY.get_base_power(get_system(model))
    end
    return base_powers
end

get_initial_time(sim::Simulation) = sim.initial_time
get_sequence(sim::Simulation) = sim.sequence
get_steps(sim::Simulation) = sim.steps
get_current_time(sim::Simulation) = get_current_time(get_simulation_state(sim))
get_simulation_model(s::Simulation, name) = get_simulation_model(get_models(s), name)
get_models(sim::Simulation) = sim.models
get_simulation_dir(sim::Simulation) = dirname(sim.internal.logs_dir)
get_simulation_files_dir(sim::Simulation) = sim.internal.sim_files_dir
get_simulation_partitions_dir(sim::Simulation) =
    joinpath(get_simulation_dir(sim), "simulation_partitions")
get_store_dir(sim::Simulation) = sim.internal.store_dir
get_simulation_status(sim::Simulation) = sim.internal.status
get_simulation_build_status(sim::Simulation) = sim.internal.build_status
get_simulation_state(sim::Simulation) = sim.internal.simulation_state
set_simulation_store!(sim::Simulation, store) = sim.internal.store = store
get_simulation_store(sim::Simulation) = sim.internal.store
get_results_dir(sim::Simulation) = sim.internal.results_dir
get_models_dir(sim::Simulation) = sim.internal.models_dir

get_interval(sim::Simulation, name::Symbol) = get_interval(sim.sequence, name)

function get_simulation_time(sim::Simulation, problem_number::Int)
    return sim.internal.date_ref[problem_number]
end

get_ini_cond_chronology(sim::Simulation) = get_sequence(sim).ini_cond_chronology
get_name(sim::Simulation) = sim.name
get_simulation_folder(sim::Simulation) = sim.simulation_folder
get_execution_order(sim::Simulation) = get_sequence(sim).execution_order
get_current_execution_index(sim::Simulation) = get_sequence(sim).current_execution_index
get_logs_folder(sim::Simulation) = sim.internal.logs_dir
get_recorder_folder(sim::Simulation) = sim.internal.recorder_dir
get_console_level(sim::Simulation) = sim.internal.console_level
get_file_level(sim::Simulation) = sim.internal.file_level
get_rng(sim::Simulation) = sim.internal.rng

set_simulation_status!(sim::Simulation, status) = sim.internal.status = status
set_simulation_build_status!(sim::Simulation, status::SimulationBuildStatus) =
    sim.internal.build_status = status

function set_current_time!(sim::Simulation, val::Dates.DateTime)
    set_current_time!(get_simulation_state(sim), val)
    return
end

function _get_simulation_initial_times!(sim::Simulation)
    model_initial_times = OrderedDict{Int, Vector{Dates.DateTime}}()
    sim_ini_time = get_initial_time(sim)
    for (model_number, model) in enumerate(get_models(sim).decision_models)
        system = get_system(model)
        model_horizon = get_horizon(model)
        system_horizon = PSY.get_forecast_horizon(system)
        system_interval = PSY.get_forecast_interval(system)
        if model_horizon > system_horizon
            throw(
                IS.ConflictingInputsError(
                    "$(get_name(model)) model horizon: $(Dates.canonicalize(model_horizon)) and forecast horizon: $(Dates.canonicalize(system_horizon)) are not compatible",
                ),
            )
        end
        model_initial_times[model_number] = PSY.get_forecast_initial_times(system)
        for (ix, element) in enumerate(model_initial_times[model_number][1:(end - 1)])
            if !(element + system_interval == model_initial_times[model_number][ix + 1])
                throw(
                    IS.ConflictingInputsError(
                        "The sequence of forecasts in the model's systems are invalid",
                    ),
                )
            end
        end
        if sim_ini_time !== nothing &&
           !mapreduce(x -> x == sim_ini_time, |, model_initial_times[model_number])
            throw(
                IS.ConflictingInputsError(
                    "The specified simulation initial_time $sim_ini_time isn't contained in model $(get_name(model)).
Manually provided initial times have to be compatible with the specified interval and horizon in the models.",
                ),
            )
        end
    end
    if get_initial_time(sim) === nothing
        sim.initial_time = model_initial_times[1][1]
        @debug("Initial Simulation timestamp will be infered from the data. \\
               Initial Simulation timestamp set to $(sim.initial_time)")
    end
    if get_models(sim).emulation_model !== nothing
        em = get_models(sim).emulation_model
        system = get_system(get_models(sim).emulation_model)
        ini_time, ts_length =
            PSY.check_time_series_consistency(system, PSY.SingleTimeSeries)
        resolution = get_resolution(em)
        em_available_times = range(ini_time; step = resolution, length = ts_length)
        if get_initial_time(sim) ∉ em_available_times
            throw(
                IS.ConflictingInputsError(
                    "The simulation initial_time $sim_ini_time isn't contained in the
                    emulation model $(get_name(em)).",
                ),
            )
        else
            model_initial_times[length(model_initial_times) + 1] = [sim.initial_time]
        end
    end
    set_current_time!(sim, sim.initial_time)
    return model_initial_times
end

function _check_steps(
    sim::Simulation,
    model_initial_times::OrderedDict{Int, Vector{Dates.DateTime}},
)
    sequence = get_sequence(sim)
    execution_order = get_execution_order(sequence)
    for (model_number, model) in enumerate(get_models(sim).decision_models)
        execution_counts = get_executions(model)
        # Checks the consistency between two methods of calculating the number of executions
        total_model_executions = length(findall(x -> x == model_number, execution_order))
        @assert_op total_model_executions == execution_counts

        forecast_count = length(model_initial_times[model_number])
        if get_steps(sim) * execution_counts > forecast_count
            throw(
                IS.ConflictingInputsError(
                    "The number of available time series ($(forecast_count)) is not enough to perform the
desired amount of simulation steps ($(sim.steps*execution_counts)).",
                ),
            )
        end
    end
    return
end

function _check_folder(sim::Simulation)
    folder = get_simulation_folder(sim)
    !isdir(folder) &&
        throw(IS.ConflictingInputsError("Specified folder = $folder is not valid"))
    try
        mkdir(joinpath(folder, "fake"))
        rm(joinpath(folder, "fake"))
    catch e
        throw(IS.ConflictingInputsError("Specified folder does not have write access [$e]"))
    end
end

# Compare initial conditions for all `InitialConditionType`s with the
# `requires_reconciliation` trait across `models`, log @info messages for mismatches
function _initial_conditions_reconciliation!(
    models::Vector{<:OperationModel})
    model_names = get_name.(models)
    has_mismatches = false
    @info "Reconciling initial conditions across models $(join(model_names, ", "))"
    # all_ic_keys: all the `ICKey`s that appear in any of the models
    all_ic_keys = union(keys.(get_initial_conditions.(models))...)
    # all_ic_values: Dict{ICKey, Dict{model_index, Dict{component_name, ic_value}}}
    all_ic_values = Dict()
    for ic_key in all_ic_keys
        if !requires_reconciliation(get_entry_type(ic_key))
            @debug "Skipping initial conditions reconciliation for $(get_entry_type(ic_key)) due to false requires_reconciliation"
            continue
        end
        # ic_vals_per_model: Dict{model_index, Dict{component_name, ic_value}}
        ic_vals_per_model = Dict()
        for (i, model) in enumerate(models)
            ics = PSI.get_initial_conditions(model)
            haskey(ics, ic_key) || continue
            # ic_vals_per_component: Dict{component_name, ic_value}
            ic_vals_per_component =
                Dict(get_name(get_component(ic)) => get_condition(ic) for ic in ics[ic_key])
            ic_vals_per_model[i] = ic_vals_per_component
        end

        # Assert that all models have the same components for current ic_key
        @assert allequal(Set.(keys.(values(ic_vals_per_model)))) "For IC key $ic_key, not all models have the same components"

        # For each component in current ic_key, compare values across models
        component_names = keys(first(values(ic_vals_per_model)))
        for component_name in component_names
            all_values = [result[component_name] for result in values(ic_vals_per_model)]
            ref_value = first(all_values)
            if !allequal(isapprox.(all_values, ref_value; atol = ABSOLUTE_TOLERANCE))
                has_mismatches = true
                mismatch_msg = "For IC key $ic_key, mismatch on component $component_name:"
                for (model_i, result) in sort(pairs(ic_vals_per_model); by = first)
                    mismatch_msg *= "\n\t$(model_names[model_i]): $(result[component_name])"
                end
                @info mismatch_msg
            end
        end
        all_ic_values[ic_key] = ic_vals_per_model
    end

    # TODO now that we have found the initial conditions mismatches, we must fix them
    if has_mismatches
        @warn "Models have initial condition mismatches; reconciliation is not yet implemented"
    end

    return all_ic_values
end

function _build_single_model_for_simulation(
    model::DecisionModel,
    sim::Simulation,
    model_number::Int,
)
    initial_time = get_initial_time(sim)
    set_initial_time!(model, initial_time)
    output_dir = joinpath(get_models_dir(sim), string(get_name(model)))
    mkpath(output_dir)
    set_output_dir!(model, output_dir)
    try
        # TODO-PJ: Temporary while are able to switch from PJ to POI
        container = get_optimization_container(model)
        container.built_for_recurrent_solves = true
        build_impl!(model)
        sim.internal.date_ref[model_number] = initial_time
        set_status!(model, ModelBuildStatus.BUILT)
        _pre_solve_model_checks(model)
    catch
        set_status!(model, ModelBuildStatus.FAILED)
        @error "Failed to build $(get_name(model))"
        rethrow()
    end
    return
end

function _build_decision_models!(sim::Simulation)
    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Build Decision Problems" begin
        decision_models = get_decision_models(get_models(sim))
        #TODO: Re-enable Threads.@threads with proper implementation of the timer.
        for model_n in 1:length(decision_models)
            TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Problem $(get_name(decision_models[model_n]))" begin
                _build_single_model_for_simulation(decision_models[model_n], sim, model_n)
            end
        end
    end
    _initial_conditions_reconciliation!(get_decision_models(get_models(sim)))
    return
end

function _build_emulation_model!(sim::Simulation)
    model = get_emulation_model(get_models(sim))

    if model === nothing
        return
    end

    try
        initial_time = get_initial_time(sim)
        set_initial_time!(model, initial_time)
        output_dir = joinpath(get_models_dir(sim), string(get_name(model)))
        mkpath(output_dir)
        set_output_dir!(model, output_dir)
        TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Problem Emulation $(get_name(model))" begin
            build_impl!(model)
        end
        sim.internal.date_ref[length(sim.internal.date_ref) + 1] = initial_time
        set_status!(model, ModelBuildStatus.BUILT)
    catch
        set_status!(model, ModelBuildStatus.FAILED)
        rethrow()
    end
    return
end

function _initialize_simulation_state!(sim::Simulation)
    step_resolution = get_step_resolution(get_sequence(sim))
    simulation_models = get_models(sim)
    initialize_simulation_state!(
        get_simulation_state(sim),
        simulation_models,
        step_resolution,
        get_initial_time(sim),
    )
    return
end

function _get_model_store_requirements!(
    rules::CacheFlushRules,
    model::OperationModel,
    num_rows::Int,
)
    model_name = get_name(model)
    horizon = get_horizon(model)
    resolution = get_resolution(model)
    horizon_count = horizon ÷ resolution
    reqs = SimulationModelStoreRequirements()
    container = get_optimization_container(model)

    for (key, array) in get_duals(container)
        !should_write_resulting_value(key) && continue
        reqs.duals[key] = _calc_dimensions(array, key, num_rows, horizon_count)
        add_rule!(rules, model_name, key, true)
    end

    for (key, param_container) in get_parameters(container)
        !should_write_resulting_value(key) && continue
        array = get_multiplier_array(param_container)
        reqs.parameters[key] = _calc_dimensions(array, key, num_rows, horizon_count)
        add_rule!(rules, model_name, key, false)
    end

    for (key, array) in get_variables(container)
        !should_write_resulting_value(key) && continue
        reqs.variables[key] = _calc_dimensions(array, key, num_rows, horizon_count)
        add_rule!(rules, model_name, key, true)
    end

    for (key, array) in get_aux_variables(container)
        !should_write_resulting_value(key) && continue
        reqs.aux_variables[key] = _calc_dimensions(array, key, num_rows, horizon_count)
        add_rule!(rules, model_name, key, true)
    end

    for (key, array) in get_expressions(container)
        !should_write_resulting_value(key) && continue
        reqs.expressions[key] = _calc_dimensions(array, key, num_rows, horizon_count)
        add_rule!(rules, model_name, key, false)
    end

    return reqs
end

function _get_emulation_store_requirements(sim::Simulation)
    sim_state = get_simulation_state(sim)
    system_state = get_system_states(sim_state)
    sim_time = get_steps(sim) * get_step_resolution(get_sequence(sim))
    reqs = SimulationModelStoreRequirements()

    for (key, state_values) in get_duals_values(system_state)
        !should_write_resulting_value(key) && continue
        num_time_rows = sim_time ÷ get_data_resolution(state_values)
        cols = get_column_names(key, state_values)
        # Use actual data dimensions (excluding last axis which is time) for proper HDF storage
        data_dims = size(state_values.values)[1:(end - 1)]
        reqs.duals[key] =
            Dict("columns" => cols, "dims" => (num_time_rows, data_dims...))
    end

    for (key, state_values) in get_parameters_values(system_state)
        !should_write_resulting_value(key) && continue
        num_time_rows = sim_time ÷ get_data_resolution(state_values)
        cols = get_column_names(key, state_values)
        # Use actual data dimensions (excluding last axis which is time) for proper HDF storage
        data_dims = size(state_values.values)[1:(end - 1)]
        reqs.parameters[key] =
            Dict("columns" => cols, "dims" => (num_time_rows, data_dims...))
    end

    for (key, state_values) in get_variables_values(system_state)
        !should_write_resulting_value(key) && continue
        num_time_rows = sim_time ÷ get_data_resolution(state_values)
        cols = get_column_names(key, state_values)
        # Use actual data dimensions (excluding last axis which is time) for proper HDF storage
        data_dims = size(state_values.values)[1:(end - 1)]
        reqs.variables[key] =
            Dict("columns" => cols, "dims" => (num_time_rows, data_dims...))
    end

    for (key, state_values) in get_aux_variables_values(system_state)
        !should_write_resulting_value(key) && continue
        num_time_rows = sim_time ÷ get_data_resolution(state_values)
        cols = get_column_names(key, state_values)
        # Use actual data dimensions (excluding last axis which is time) for proper HDF storage
        data_dims = size(state_values.values)[1:(end - 1)]
        reqs.aux_variables[key] =
            Dict("columns" => cols, "dims" => (num_time_rows, data_dims...))
    end

    for (key, state_values) in get_expression_values(system_state)
        !should_write_resulting_value(key) && continue
        num_time_rows = sim_time ÷ get_data_resolution(state_values)
        cols = get_column_names(key, state_values)
        # Use actual data dimensions (excluding last axis which is time) for proper HDF storage
        data_dims = size(state_values.values)[1:(end - 1)]
        reqs.expressions[key] =
            Dict("columns" => cols, "dims" => (num_time_rows, data_dims...))
    end
    return reqs
end

function _initialize_problem_storage!(
    sim::Simulation,
    cache_size_mib::Int = DEFAULT_SIMULATION_STORE_CACHE_SIZE_MiB,
    min_cache_flush_size_mib::Int = MIN_CACHE_FLUSH_SIZE_MiB,
)
    sequence = get_sequence(sim)
    executions_by_model = sequence.executions_by_model
    models = get_models(sim)
    decision_model_store_params = OrderedDict{Symbol, ModelStoreParams}()
    dm_model_req = Dict{Symbol, SimulationModelStoreRequirements}()
    rules = CacheFlushRules(;
        max_size = cache_size_mib * MiB,
        min_flush_size = trunc(min_cache_flush_size_mib * MiB),
    )
    for model in get_decision_models(models)
        model_name = get_name(model)
        decision_model_store_params[model_name] = get_store_params(model)
        num_executions = executions_by_model[model_name]
        num_rows = num_executions * get_steps(sim)
        dm_model_req[model_name] = _get_model_store_requirements!(rules, model, num_rows)
    end

    em = get_emulation_model(models)
    if em === nothing
        base_params = last(collect(values(decision_model_store_params)))
        resolution = minimum([v.resolution for v in values(decision_model_store_params)])
        emulation_model_store_params = OrderedDict(
            :Emulator => ModelStoreParams(
                get_step_resolution(sequence) ÷ resolution, # Num Executions
                resolution, # Horizon
                resolution, # Interval
                resolution, # Resolution
                get_base_power(base_params),
                get_system_uuid(base_params),
            ),
        )
    else
        emulation_model_store_params =
            OrderedDict(Symbol(get_name(em)) => get_store_params(em))
    end

    em_model_req = _get_emulation_store_requirements(sim)

    simulation_store_params = SimulationStoreParams(
        get_initial_time(sim),
        get_step_resolution(sequence),
        get_steps(sim),
        decision_model_store_params,
        emulation_model_store_params,
    )
    @debug "initialized problem requirements" simulation_store_params
    store = get_simulation_store(sim)

    initialize_problem_storage!(
        store,
        simulation_store_params,
        dm_model_req,
        em_model_req,
        rules,
    )
    return simulation_store_params
end

function _build!(
    sim::Simulation;
    store_systems_in_results = true,
    setup_simulation_partitions = false,
    partitions = nothing,
    index = nothing,
)
    set_simulation_build_status!(sim, SimulationBuildStatus.IN_PROGRESS)
    problem_initial_times = _get_simulation_initial_times!(sim)
    sequence = get_sequence(sim)
    step_resolution = get_step_resolution(sequence)
    simulation_models = get_models(sim)

    if !isnothing(partitions) && !isnothing(index)
        step_range = get_absolute_step_range(partitions, index)
        sim.initial_time += step_resolution * (step_range.start - 1)
        set_current_time!(sim.internal.simulation_state, sim.initial_time)
        sim.steps = length(step_range)
        @info "Set parameters for simulation partition" index sim.initial_time sim.steps
    end

    for (ix, model) in enumerate(get_decision_models(simulation_models))
        problem_interval = get_interval(sequence, model)
        # Note to devs: Here we are setting the number of operations problem executions we
        # will see for every step of the simulation. The step of the simulation is determined
        # by the first decision problem interval
        if ix == 1
            set_executions!(model, 1)
        else
            if step_resolution % problem_interval != Dates.Millisecond(0)
                error(
                    "The $(get_name(model)) problem interval is not an integer fraction of the simulation step",
                )
            end
            set_executions!(model, Int(step_resolution / problem_interval))
        end
    end

    em = get_emulation_model(simulation_models)
    if em !== nothing
        em_resolution = get_resolution(em)
        set_executions!(em, get_steps(sim) * Int(step_resolution / em_resolution))
    end

    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Check Steps" begin
        _check_steps(sim, problem_initial_times)
    end

    if store_systems_in_results
        # Spawn system serialization (JSON conversion) in parallel with model builds.
        # Systems are read-only during building, so this is safe.
        serialization_task = Threads.@spawn _serialize_systems_to_json(sim)
    end

    _build_decision_models!(sim)
    _build_emulation_model!(sim)

    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Initialize Simulation State" begin
        _initialize_simulation_state!(sim)
    end

    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Initialize Problem Storage" begin
        open_store(HdfSimulationStore, get_store_dir(sim), "w") do store
            set_simulation_store!(sim, store)
            try
                _initialize_problem_storage!(sim)
                if store_systems_in_results
                    # Fetch pre-computed JSON from the parallel task and write to HDF5 store.
                    serialized = fetch(serialization_task)
                    for (uuid, json_text) in serialized
                        write_system_json!(store, uuid, json_text)
                    end
                end
            finally
                set_simulation_store!(sim, nothing)
            end
        end
    end

    if setup_simulation_partitions
        TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Setup Simulation Partition" begin
            _setup_simulation_partitions(sim)
        end
    end

    return
end

function _set_simulation_internal!(
    sim::Simulation,
    partitions::Union{Nothing, SimulationPartitions},
    recorders,
    console_level,
    file_level,
)
    sim.internal = SimulationInternal(
        sim.steps,
        get_models(sim),
        get_simulation_folder(sim),
        get_name(sim),
        recorders,
        console_level,
        file_level;
        partitions = partitions,
    )
    return
end

function _setup_simulation_partitions(sim::Simulation)
    mkdir(sim.internal.partitions_dir)
    filename = joinpath(sim.internal.partitions_dir, "config.json")
    IS.to_json(sim.internal.partitions, filename; pretty = true)
    for i in 1:get_num_partitions(sim.internal.partitions)
        mkdir(joinpath(sim.internal.partitions_dir, string(i)))
    end
end

"""
Build the Simulation, problems and the related folder structure.

# Arguments

  - `sim::Simulation`: simulation object
  - `recorders::Vector{Symbol} = []`: recorder names to register
  - `store_systems_in_results::Bool = true`: stores the systems as JSON in the results HDF5 file
  - `console_level = Logging.Error`:
  - `file_level = Logging.Info`:
"""
function build!(
    sim::Simulation;
    recorders = [],
    console_level = Logging.Error,
    file_level = Logging.Info,
    store_systems_in_results = true,
    partitions::Union{Nothing, SimulationPartitions} = nothing,
    index = nothing,
)
    TimerOutputs.reset_timer!(BUILD_PROBLEMS_TIMER)
    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Build Simulation" begin
        _check_folder(sim)
        _set_simulation_internal!(sim, partitions, recorders, console_level, file_level)
        make_dirs(sim.internal)
        if !isnothing(partitions) && isnothing(index)
            setup_simulation_partitions = true
        else
            setup_simulation_partitions = false
        end
        file_mode = "w"
        logger = configure_logging(sim.internal, file_mode)
        register_recorders!(sim.internal, file_mode)
        try
            Logging.with_logger(logger) do
                try
                    _build!(
                        sim;
                        store_systems_in_results = store_systems_in_results,
                        setup_simulation_partitions = setup_simulation_partitions,
                        partitions = partitions,
                        index = index,
                    )
                    set_simulation_build_status!(sim, SimulationBuildStatus.BUILT)
                    set_simulation_status!(sim, RunStatus.INITIALIZED)
                catch e
                    @error "Simulation build failed" exception = (e, catch_backtrace())
                    set_simulation_build_status!(sim, SimulationBuildStatus.FAILED)
                    set_simulation_status!(sim, RunStatus.NOT_READY)
                    rethrow(e)
                end
            end
        finally
            unregister_recorders!(sim.internal)
            close(logger)
        end
    end
    @info "\n$(BUILD_PROBLEMS_TIMER)\n"
    return get_simulation_build_status(sim)
end

function _apply_warm_start!(model::OperationModel)
    container = get_optimization_container(model)
    # If the model was used to retrieve duals from an MILP the logic has to be different and
    # the results need to be read from the primal cache
    if isempty(container.primal_values_cache)
        jump_model = get_jump_model(container)
        all_vars = JuMP.all_variables(jump_model)
        all_vars_value = jump_value.(all_vars)
        JuMP.set_start_value.(all_vars, all_vars_value)
    else
        for (var_key, variable_value) in container.primal_values_cache.variables_cache
            variable = get_variable(container, var_key)
            JuMP.set_start_value.(variable, variable_value)
        end
    end
    return
end

function _get_next_problem_initial_time(sim::Simulation, model_name::Symbol)
    current_time = get_current_time(sim)
    sequence = get_sequence(sim)
    current_exec_index = sequence.current_execution_index
    exec_order = get_execution_order(sequence)

    if length(exec_order) > 1 && (current_exec_index + 1 > length(exec_order)) # Moving to the next step
        next_initial_time = get_simulation_time(sim, exec_order[1])
    elseif length(exec_order) == 1 ||
           exec_order[current_exec_index + 1] == exec_order[current_exec_index] # Solving the same problem again
        current_model_interval = get_interval(sim.sequence, model_name)
        next_initial_time = current_time + current_model_interval
    else # Solving another problem next
        next_initial_time = get_simulation_time(sim, exec_order[current_exec_index + 1])
    end
    return next_initial_time
end

function _update_system_state!(sim::Simulation, model_name::Symbol)
    sim_state = get_simulation_state(sim)
    system_state = get_system_states(sim_state)
    decision_state = get_decision_states(sim_state)
    simulation_time = get_current_time(sim_state)
    next_stage_initial_time = _get_next_problem_initial_time(sim, model_name)

    for key in get_dataset_keys(decision_state)
        state_data = get_dataset(decision_state, key)
        last_update = get_update_timestamp(decision_state, key)

        if last_update > simulation_time
            error("Something went really wrong. Please report this error. \\
            last_update: $(last_update) \\
            simulation_time: $(simulation_time) \\
            key: $(encode_key_as_string(key))")
        end

        resolution = get_data_resolution(state_data)
        update_timestamp = max(next_stage_initial_time - resolution, simulation_time)
        if update_timestamp < get_update_timestamp(system_state, key)
            error("The update overwrites more recent data with past data")
        elseif update_timestamp > get_update_timestamp(system_state, key)
            update_system_state!(system_state, key, decision_state, update_timestamp)
        else
            @assert_op update_timestamp == get_update_timestamp(system_state, key)
        end
    end

    IS.@record :execution StateUpdateEvent(simulation_time, model_name, "SystemState")
    return
end

function _update_system_state!(sim::Simulation, model::DecisionModel)
    _update_system_state!(sim, get_name(model))
    return
end

function _update_system_state!(sim::Simulation, model::EmulationModel)
    sim_state = get_simulation_state(sim)
    simulation_time = get_current_time(sim)
    system_state = get_system_states(sim_state)
    store = get_simulation_store(sim)
    em_model_name = get_name(model)
    for key in get_container_keys(get_optimization_container(model))
        !should_write_resulting_value(key) && continue
        update_system_state!(system_state, key, store, em_model_name, simulation_time)
    end
    IS.@record :execution StateUpdateEvent(simulation_time, em_model_name, "SystemState")
    return
end

function _update_simulation_state!(sim::Simulation, model::EmulationModel)
    # Order of these operations matters. Do not reverse.
    # This will update the state with the results of the store first and then fill
    # the remaning values with the decision state.
    _update_system_state!(sim, model)
    _update_system_state!(sim, get_name(model))
    return
end

function _update_simulation_state!(sim::Simulation, model::DecisionModel)
    #Order matters; update parameters first to ensure event parameters are updated first
    _update_simulation_state_parameters!(sim, model)
    _update_simulation_state_others!(sim, model)
    model_name = get_name(model)
    simulation_time = get_current_time(sim)
    IS.@record :execution StateUpdateEvent(simulation_time, model_name, "DecisionState")
    return
end

function _update_simulation_state_parameters!(sim::Simulation, model::DecisionModel)
    model_name = get_name(model)
    store = get_simulation_store(sim)
    simulation_time = get_current_time(sim)
    state = get_simulation_state(sim)
    model_params = get_decision_model_params(store, model_name)
    all_parameter_keys = list_decision_model_keys(store, model_name, :parameters)
    countdown_parameter_keys = filter(_is_event_countdown_parameter_key, all_parameter_keys)
    other_parameter_keys = filter(!_is_event_countdown_parameter_key, all_parameter_keys)
    # Order matters; AvailableStatusChangeCountdownParameter must be updated first if it exists
    for key in countdown_parameter_keys
        !has_dataset(get_decision_states(state), key) && continue
        res = read_result(DenseAxisArray, store, model_name, key, simulation_time)
        update_decision_state!(state, key, res, simulation_time, model_params)
    end
    for key in other_parameter_keys
        !has_dataset(get_decision_states(state), key) && continue
        res = read_result(DenseAxisArray, store, model_name, key, simulation_time)
        update_decision_state!(state, key, res, simulation_time, model_params)
    end
end

function _is_event_countdown_parameter_key(
    ::ParameterKey{T, U},
) where {T <: ParameterType, U <: PSY.Component}
    return false
end

function _is_event_countdown_parameter_key(
    ::ParameterKey{AvailableStatusChangeCountdownParameter, U},
) where {U <: PSY.Component}
    return true
end

function _update_simulation_state_others!(sim::Simulation, model::DecisionModel)
    model_name = get_name(model)
    store = get_simulation_store(sim)
    simulation_time = get_current_time(sim)
    state = get_simulation_state(sim)
    model_params = get_decision_model_params(store, model_name)
    for field in [:duals, :aux_variables, :variables, :expressions]
        for key in list_decision_model_keys(store, model_name, field)
            !has_dataset(get_decision_states(state), key) && continue
            res = read_result(DenseAxisArray, store, model_name, key, simulation_time)
            update_decision_state!(state, key, res, simulation_time, model_params)
        end
    end
    return
end

function _write_state_to_store!(store::SimulationStore, sim::Simulation)
    sim_state = get_simulation_state(sim)
    system_state = get_system_states(sim_state)
    model_name = get_last_decision_model(sim_state)
    em_store = get_em_data(store)
    simulation_time = get_current_time(sim)
    sim_ini_time = get_initial_time(sim)
    state_resolution = get_system_states_resolution(sim_state)
    for key in get_dataset_keys(system_state)
        store_update_time = get_last_updated_timestamp(em_store, key)
        state_update_time = get_update_timestamp(system_state, key)
        # If the store is outdated w.r.t to the state
        @assert store_update_time <= simulation_time
        if store_update_time < state_update_time
            _update_timestamp = max(store_update_time + state_resolution, sim_ini_time)
            while _update_timestamp <= state_update_time
                state_values =
                    get_decision_state_value(sim_state, key, _update_timestamp)
                ix = get_last_recorded_row(em_store, key) + 1
                write_result!(
                    store,
                    model_name,
                    key,
                    ix,
                    _update_timestamp,
                    state_values,
                )
                _update_timestamp += state_resolution
            end
        end
    end
    return
end

"""
Default problem update function for most problems with no customization
"""
function update_model!(model::OperationModel, sim::Simulation)
    update_model!(model, get_simulation_state(sim), get_ini_cond_chronology(sim))
    if get_rebuild_model(model)
        container = get_optimization_container(model)
        reset_optimization_model!(container)
        build_impl!(container, get_template(model), get_system(model))
    end

    return
end

function _execute!(
    sim::Simulation;
    cache_size_mib::Union{Nothing, Int} = nothing,
    min_cache_flush_size_mib::Union{Nothing, Int} = nothing,
    exports = nothing,
    enable_progress_bar = progress_meter_enabled(),
    disable_timer_outputs = false,
    results_channel = nothing,
)
    @assert sim.internal !== nothing

    set_simulation_status!(sim, RunStatus.RUNNING)
    execution_order = get_execution_order(sim)
    steps = get_steps(sim)
    num_executions = steps * length(execution_order)
    store = get_simulation_store(sim)
    # For InMemorySimulationStore, initialize storage here since it doesn't persist from build.
    if store isa InMemorySimulationStore
        _initialize_problem_storage!(
            sim,
            something(cache_size_mib, DEFAULT_SIMULATION_STORE_CACHE_SIZE_MiB),
            something(min_cache_flush_size_mib, MIN_CACHE_FLUSH_SIZE_MiB),
        )
    else
        # Override cache flush rules from build phase if user passes kwargs at execution time.
        rules = CacheFlushRules(;
            max_size = something(cache_size_mib, DEFAULT_SIMULATION_STORE_CACHE_SIZE_MiB) * MiB,
            min_flush_size = trunc(
                something(min_cache_flush_size_mib, MIN_CACHE_FLUSH_SIZE_MiB) * MiB,
            ),
        )
        set_cache_flush_rules!(store, rules)
    end
    store_params = get_params(store)
    if exports !== nothing
        if !(exports isa SimulationResultsExport)
            exports = SimulationResultsExport(exports, store_params)
        end

        if exports.path === nothing
            exports.path = get_results_dir(sim)
        end
    end
    sequence = get_sequence(sim)
    models = get_models(sim)

    prog_bar = ProgressMeter.Progress(num_executions; enabled = enable_progress_bar)
    disable_timer_outputs && TimerOutputs.disable_timer!(RUN_SIMULATION_TIMER)
    store = get_simulation_store(sim)
    for step in 1:steps
        IS.@record :simulation_status SimulationStepEvent(
            get_current_time(sim),
            step,
            "start",
        )
        # This progress print is required to show the progress bar upfront
        ProgressMeter.update!(
            prog_bar,
            1;
            showvalues = [
                (:Step, 1),
                (:Problem, get_name(get_simulation_model(models, 1))),
                (:("Simulation Timestamp"), get_current_time(sim)),
            ],
        )
        for (ix, model_number) in enumerate(execution_order)
            model = get_simulation_model(models, model_number)
            model_name = get_name(model)
            set_current_time!(sim, sim.internal.date_ref[model_number])
            sequence.current_execution_index = ix
            current_time = get_current_time(sim)
            IS.@record :simulation_status ProblemExecutionEvent(
                current_time,
                step,
                model_name,
                "start",
            )

            progress_event = SimulationProgressEvent(;
                model_name = string(model_name),
                step = step,
                index = (step - 1) * length(execution_order) + ix,
                timestamp = get_current_time(sim),
                wall_time = Dates.now(),
                exec_time_s = 0.0,
            )
            ProgressMeter.update!(
                prog_bar,
                progress_event.index;
                showvalues = [
                    (:Step, progress_event.step),
                    (:Problem, model_name),
                    (:("Simulation Timestamp"), progress_event.timestamp),
                ],
            )

            start_time = time()
            TimerOutputs.@timeit RUN_SIMULATION_TIMER "Execute $(model_name)" begin
                if !is_built(model)
                    error("$(model_name) status is not ModelBuildStatus.BUILT")
                end

                # Is first run of first problem? Yes -> don't update problem

                TimerOutputs.@timeit RUN_SIMULATION_TIMER "Update $(model_name)" begin
                    !(step == 1 && ix == 1) && update_model!(model, sim)
                end

                TimerOutputs.@timeit RUN_SIMULATION_TIMER "Solve $(model_name)" begin
                    status = solve!(step, model, current_time, store; exports = exports)
                end # Run problem Timer

                TimerOutputs.@timeit RUN_SIMULATION_TIMER "Update State" begin
                    if status == RunStatus.SUCCESSFULLY_FINALIZED
                        # TODO: _update_simulation_state! can use performance improvements
                        _update_simulation_state!(sim, model)
                        if model_number == execution_order[end]
                            _update_system_state!(sim, model)
                            _write_state_to_store!(store, sim)
                            # This function needs to be called last so make sure that the update to the
                            # state get written AFTER the models run.
                            apply_simulation_events!(sim)
                        end
                    end
                end

                sim.internal.run_count[step][model_number] += 1
                sim.internal.date_ref[model_number] += get_interval(sequence, model_name)

                # _apply_warm_start! can only be called once all the operations that read solutions
                # from the optimization container have been called.
                # See https://github.com/NREL-Sienna/PowerSimulations.jl/pull/793#discussion_r761545526
                # for reference
                if warm_start_enabled(model)
                    _apply_warm_start!(model)
                end

                IS.@record :simulation_status ProblemExecutionEvent(
                    get_current_time(sim),
                    step,
                    model_name,
                    "done",
                )
            end #execution problem timer
            progress_event.exec_time_s = time() - start_time
            if !isnothing(results_channel)
                put!(results_channel, SimulationIntermediateResult(progress_event))
            end
        end # execution order for loop

        IS.@record :simulation_status SimulationStepEvent(
            get_current_time(sim),
            step,
            "done",
        )
    end # Steps for loop
    return
end

"""
Solves the simulation model for sequential Simulations.

# Arguments

  - `sim::Simulation=sim`: simulation object created by Simulation()

The optional keyword argument `exports` controls exporting of results to CSV files as
the simulation runs.

# Example
```julia
sim = Simulation("Test", 7, problems, "/Users/folder")
execute!(sim::Simulation; kwargs...)
```
"""
function execute!(sim::Simulation; kwargs...)
    file_mode = "a"
    logger = configure_logging(sim.internal, file_mode)
    register_recorders!(sim.internal, file_mode)

    # Undocumented option for test & dev only.
    in_memory = get(kwargs, :in_memory, false)
    store_type = in_memory ? InMemorySimulationStore : HdfSimulationStore

    sim_build_status = get_simulation_build_status(sim)
    sim_run_status = get_simulation_status(sim)
    if (sim_build_status != SimulationBuildStatus.BUILT) ||
       (sim_run_status != RunStatus.INITIALIZED)
        error(
            "Simulation build status $sim_build_status, or Simulation run status $sim_run_status, are invalid, you need to rebuild the simulation",
        )
    end
    file_mode = in_memory ? "w" : "rw"
    try
        Logging.with_logger(logger) do
            open_store(store_type, get_store_dir(sim), file_mode) do store
                set_simulation_store!(sim, store)
                try
                    TimerOutputs.reset_timer!(RUN_SIMULATION_TIMER)
                    TimerOutputs.@timeit RUN_SIMULATION_TIMER "Execute Simulation" begin
                        _execute!(
                            sim;
                            [k => v for (k, v) in kwargs if k != :in_memory]...,
                        )
                    end
                    @info ("\n$(RUN_SIMULATION_TIMER)\n")
                    set_simulation_status!(sim, RunStatus.SUCCESSFULLY_FINALIZED)
                    log_cache_hit_percentages(store)
                catch e
                    set_simulation_status!(sim, RunStatus.FAILED)
                    @error "simulation failed" exception = (e, catch_backtrace())
                end
            end
        end
    finally
        _empty_problem_caches!(sim)
        unregister_recorders!(sim.internal)
        close(logger)
    end

    if !in_memory && get_simulation_status(sim) == RunStatus.SUCCESSFULLY_FINALIZED
        IS.compute_file_hash(get_store_dir(sim), HDF_FILENAME)
    end

    serialize_status(sim)
    return get_simulation_status(sim)
end

function _empty_problem_caches!(sim::Simulation)
    models = get_models(sim)
    for model in get_decision_models(models)
        empty_time_series_cache!(model)
    end
    return
end

function _serialize_systems_to_json(sim::Simulation)
    simulation_models = get_models(sim)
    results = Dict{String, String}()
    @debug Threads.threadid() "Serializing systems to JSON in parallel with model building"
    for dm in get_decision_models(simulation_models)
        sys = get_system(dm)
        uuid = string(IS.get_uuid(sys))
        if !haskey(results, uuid)
            results[uuid] = PSY.to_json(sys)
        end
    end
    em = get_emulation_model(simulation_models)
    if !isnothing(em)
        sys = get_system(em)
        uuid = string(IS.get_uuid(sys))
        if !haskey(results, uuid)
            results[uuid] = PSY.to_json(sys)
        end
    end
    return results
end

function serialize_status(sim::Simulation)
    serialize_status(get_simulation_status(sim), get_results_dir(sim))
end

function serialize_status(status::RunStatus, results_dir::AbstractString)
    data = Dict("run_status" => string(status))
    filename = joinpath(results_dir, "status.json")
    open(filename, "w") do io
        JSON3.write(io, data)
    end

    return
end

function deserialize_status(sim::Simulation)
    return deserialize_status(get_results_dir(sim))
end

function deserialize_status(results_path::AbstractString)
    filename = joinpath(results_path, "status.json")
    if !isfile(filename)
        error("run status file $filename does not exist")
    end

    data = open(filename, "r") do io
        JSON3.read(io, Dict)
    end

    return get_enum_value(RunStatus, data["run_status"])
end

# The next two structs allow a parent process to monitor the simulation progress.
# They may eventually be extended to pass result data back to the parent.

Base.@kwdef mutable struct SimulationProgressEvent
    model_name::String
    step::Int
    index::Int
    timestamp::Dates.DateTime
    wall_time::Dates.DateTime
    exec_time_s::Float64
end

struct SimulationIntermediateResult
    progress_event::SimulationProgressEvent
end
