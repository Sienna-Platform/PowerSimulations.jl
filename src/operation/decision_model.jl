"""
Default solve method for a DecisionModel used inside of a Simulation. Solves problems that conform to the requirements of DecisionModel{<: DecisionProblem}

# Arguments

  - `step::Int`: Simulation Step
  - `model::OperationModel`: operation model
  - `start_time::Dates.DateTime`: Initial Time of the simulation step in Simulation time.
  - `store::SimulationStore`: Simulation output store

# Accepted Key Words

  - `exports`: realtime export of output. Use wisely, it can have negative impacts in the simulation times
"""
function POM.solve!(
    step::Int,
    model::DecisionModel{<:POM.AbstractPowerDecisionProblem},
    start_time::Dates.DateTime,
    store::SimulationStore;
    exports = nothing,
)
    # Note, we don't call solve!(decision_model) here because the solve call includes a lot of
    # other logic used when solving the models separate from a simulation
    solve_model!(model)
    IS.@assert_op get_current_time(model) == start_time
    if get_run_status(model) == RunStatus.SUCCESSFULLY_FINALIZED
        write_results!(store, model, start_time, start_time; exports = exports)
        write_optimizer_stats!(store, model, start_time)
        advance_execution_count!(model)
    end
    wait_for_serialization!(model)
    return get_run_status(model)
end
