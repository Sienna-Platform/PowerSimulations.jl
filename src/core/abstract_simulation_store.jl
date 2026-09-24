"""
Provides storage of simulation data
"""
abstract type SimulationStore end

# Required methods:
# - open_store
# - Base.isopen(store::SimulationStore)
# - Base.close(store::SimulationStore)
# - Base.flush(store::SimulationStore)
# - get_params(store::SimulationStore)
# - initialize_problem_storage!
# - list_decision_model_keys(store::SimulationStore, problem::Symbol, container_type::Symbol)
# - list_emulation_model_keys(store::SimulationStore, container_type::Symbol)
# - list_decision_models(store::SimulationStore)
# - log_cache_hit_percentages(store::SimulationStore)
# - write_output!
# - read_output
# - read_outputs
# - write_optimizer_stats!
# - read_optimizer_stats
# - get_dm_data
# - get_em_data
# - get_container_key_lookup

get_dm_data(store::SimulationStore) = store.dm_data
get_em_data(store::SimulationStore) = store.em_data

get_decision_model_params(store::SimulationStore, model_name::Symbol) =
    get_decision_model_params(get_params(store), model_name)
get_emulation_model_params(store::SimulationStore) =
    get_emulation_model_params(get_params(store))
