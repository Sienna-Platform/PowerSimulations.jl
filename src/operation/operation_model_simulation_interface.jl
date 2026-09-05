function update_model!(
    model::IOM.AbstractOptimizationModel,
    source::SimulationState,
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

function update_parameters!(model::EmulationModel, state::SimulationState)
    data = get_decision_states(state)
    update_parameters!(model, data)
    return
end

function update_parameters!(
    model::DecisionModel,
    simulation_state::SimulationState,
)
    cost_function_unsynch(get_optimization_container(model))
    for key in keys(get_parameters(model))
        update_parameter_values!(model, key, simulation_state)
    end
    if !is_synchronized(model)
        update_objective_function!(get_optimization_container(model))
        obj_func = get_objective_expression(get_optimization_container(model))
        set_synchronized_status!(obj_func, true)
    end
    return
end
