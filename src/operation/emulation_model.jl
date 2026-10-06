"""
Default solve method for an EmulationModel used inside of a Simulation. Solves problems that conform to the requirements of DecisionModel{<: DecisionProblem}

# Arguments

  - `step::Int`: Simulation Step
  - `model::IOM.AbstractOptimizationModel`: operation model
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
        write_outputs!(
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
