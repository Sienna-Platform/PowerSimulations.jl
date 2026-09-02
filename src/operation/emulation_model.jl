function update_parameters!(model::EmulationModel, store::EmulationModelStore)
    update_parameters!(model, store.data_container)
    return
end

function update_parameters!(model::EmulationModel, data::DatasetContainer{InMemoryDataset})
    cost_function_unsynch(get_optimization_container(model))
    for key in keys(get_parameters(model))
        update_parameter_values!(model, key, data)
    end
    if !is_synchronized(model)
        update_objective_function!(get_optimization_container(model))
        obj_func = get_objective_expression(get_optimization_container(model))
        set_synchronized_status!(obj_func, true)
    end
    return
end

function update_initial_conditions!(
    model::EmulationModel,
    source::EmulationModelStore,
    ::InterProblemChronology,
)
    for key in keys(get_initial_conditions(model))
        update_initial_conditions!(model, key, source)
    end
    return
end

function update_model!(
    model::EmulationModel,
    source::EmulationModelStore,
    ini_cond_chronology,
)
    TimerOutputs.@timeit RUN_SIMULATION_TIMER "Parameter Updates" begin
        update_parameters!(model, source)
    end
    TimerOutputs.@timeit RUN_SIMULATION_TIMER "Ini Cond Updates" begin
        update_initial_conditions!(model, source, ini_cond_chronology)
    end
    return
end

"""
Update parameter function an OperationModel
"""
function update_parameter_values!(
    model::EmulationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ParameterType, U <: PSY.Component}
    # Enable again for detailed debugging
    # TimerOutputs.@timeit RUN_SIMULATION_TIMER "$T $U Parameter Update" begin
    optimization_container = get_optimization_container(model)
    update_container_parameter_values!(optimization_container, model, key, input)
    parameter_attributes = get_parameter_attributes(optimization_container, key)
    IS.@record :execution ParameterUpdateEvent(
        T,
        U,
        "event", # parameter_attributes,
        get_current_timestamp(model),
        get_name(model),
    )
    #end
    return
end

function update_model!(model::EmulationModel)
    update_model!(model, get_store(model), InterProblemChronology())
    return
end

"""
Default solve method for an EmulationModel used inside of a Simulation. Solves problems that conform to the requirements of DecisionModel{<: DecisionProblem}

# Arguments

  - `step::Int`: Simulation Step
  - `model::OperationModel`: operation model
  - `start_time::Dates.DateTime`: Initial Time of the simulation step in Simulation time.
  - `store::SimulationStore`: Simulation output store
  - `exports = nothing`: realtime export of output. Use wisely, it can have negative impacts in the simulation times
"""
function POM.solve!(
    step::Int,
    model::EmulationModel{<:POM.AbstractPowerEmulationProblem},
    start_time::Dates.DateTime,
    store::SimulationStore;
    exports = nothing,
)
    # Note, we don't call solve!(decision_model) here because the solve call includes a lot of
    # other logic used when solving the models separate from a simulation
    solve_model!(model)
    @assert get_current_time(model) == start_time
    if get_run_status(model) == RunStatus.SUCCESSFULLY_FINALIZED
        advance_execution_count!(model)
        write_results!(
            store,
            model,
            get_execution_count(model),
            start_time;
            exports = exports,
        )
        write_optimizer_stats!(store, model, get_execution_count(model))
    end
    wait_for_serialization!(model)
    return get_run_status(model)
end
