const _SHOWN_NAMES_LIMIT = 5

const _ALLOW_DIFFERENT_COMPONENTS_HINT = "pass `allow_different_components = true` to \
    `SimulationSequence` if this is intended"

# The first five names in sorted order, then a count of the rest.
function _shown_names(names)
    sorted = sort!(collect(String, names))
    shown = join(first(sorted, _SHOWN_NAMES_LIMIT), ", ")
    rest = length(sorted) - _SHOWN_NAMES_LIMIT
    return rest > 0 ? "$shown and $rest more" : shown
end

const _ComponentHolders = Vector{Pair{Symbol, Set{String}}}

function _throw_different_components(
    label::String,
    (ref_name, ref)::Pair{Symbol, Set{String}},
    (name, components)::Pair{Symbol, Set{String}},
)
    details = String[]
    for (holder, held, other, other_held) in
        ((ref_name, ref, name, components), (name, components, ref_name, ref))
        absent = setdiff(held, other_held)
        isempty(absent) && continue
        push!(details, "$other lacks $(_shown_names(absent)), which $holder has")
    end
    throw(
        IS.ConflictingInputsError(
            "Models $ref_name and $name hold different components for $label: \
             $(join(details, "; ")). Give every model the same components, or \
             $_ALLOW_DIFFERENT_COMPONENTS_HINT",
        ),
    )
end

function _check_same_components(label::String, holders::_ComponentHolders)
    length(holders) < 2 && return
    reference = first(holders)
    for holder in Iterators.drop(holders, 1)
        last(holder) == last(reference) ||
            _throw_different_components(label, reference, holder)
    end
    return
end

function _state_key_components(models::SimulationModels)
    holders = Dict{OptimizationContainerKey, _ComponentHolders}()
    for model in _state_models(models)
        container = get_optimization_container(model)
        for field in fieldnames(DatasetContainer), key in keys(getfield(container, field))
            _writes_key(model, key) || continue
            components = Set{String}(string.(first(_model_columns(model, key))))
            push!(get!(holders, key, _ComponentHolders()), get_name(model) => components)
        end
    end
    return holders
end

function _initial_condition_components(models::SimulationModels)
    holders = Dict{OptimizationContainerKey, _ComponentHolders}()
    for model in get_decision_models(models)
        for (ic_key, ics) in get_initial_conditions(model)
            requires_reconciliation(get_entry_type(ic_key)) || continue
            components = Set{String}(get_name(IOM.get_component(ic)) for ic in ics)
            push!(get!(holders, ic_key, _ComponentHolders()), get_name(model) => components)
        end
    end
    return holders
end

function _check_each_key(holders::Dict{OptimizationContainerKey, _ComponentHolders}, prefix)
    for key in sort!(collect(keys(holders)); by = encode_key_as_string)
        _check_same_components(prefix * encode_key_as_string(key), holders[key])
    end
    return
end

# Every model holding a simulation state key, and every decision model holding an initial
# condition that needs reconciliation, must have the same components for it unless the
# sequence allows different components.
function _check_component_sets(sim::Simulation)
    get_allow_different_components(sim) && return
    models = get_models(sim)
    _check_each_key(_state_key_components(models), "")
    _check_each_key(_initial_condition_components(models), "initial condition ")
    return
end

# The components that `models` write for `key`. The decision state is written by the
# decision models; the system state also by the emulator.
function _state_writers(models, key::OptimizationContainerKey)
    names = Set{String}()
    for model in models
        _writes_key(model, key) || continue
        union!(names, string.(first(_model_columns(model, key))))
    end
    return names
end

function _total_executions(sim::Simulation, n::Int)
    return count(==(n), get_execution_order(sim)) * get_steps(sim)
end

# Every execution but the simulation's first updates the parameters of the model it runs.
function _reads_parameters_from_state(sim::Simulation, n::Int)
    first_run = first(get_execution_order(sim)) == n ? 1 : 0
    return _total_executions(sim, n) > first_run
end

# A model keeps its own initial conditions at its first execution.
_reads_initial_conditions_from_state(sim::Simulation, n::Int) =
    get_ini_cond_chronology(sim) isa InterProblemChronology &&
    _total_executions(sim, n) > 1

function _throw_missing_writer(model, source_key, what, missing_names, remedy)
    throw(
        IS.ConflictingInputsError(
            "$(get_name(model)) reads $(encode_key_as_string(source_key)) from the \
             simulation state for $what, but no model writes it for \
             $(_shown_names(missing_names)). $remedy",
        ),
    )
end

# Each feedforward parameter is read from the decision state of its source key for every
# component of the parameter.
function _check_parameter_sources(sim::Simulation, model)
    for (key, parameter) in get_parameters(get_optimization_container(model))
        attributes = get_attributes(parameter)
        attributes isa VariableValueAttributes || continue
        source_key = get_attribute_key(attributes)
        writers = _state_writers(get_decision_models(get_models(sim)), source_key)
        names = string.(axes(get_parameter_array(parameter))[1])
        missing_names = setdiff(names, writers)
        isempty(missing_names) && continue
        _throw_missing_writer(model, source_key, encode_key_as_string(key),
            missing_names,
            "Give a model that writes the source every unit the feedforward sets.")
    end
    return
end

# The initial power of a unit that is not must-run is taken with its status, read from the
# `OnVariable` of the system state.
function _check_initial_power_sources(sim::Simulation, model)
    for (key, ics) in get_initial_conditions(model)
        get_entry_type(key) === DevicePower || continue
        status_key = VariableKey(OnVariable, get_component_type(key))
        writers = _state_writers(_state_models(get_models(sim)), status_key)
        missing_names = Set{String}(
            get_component_name(ic) for ic in ics if
            !IOM.get_must_run(IOM.get_component(ic)) && !(get_component_name(ic) in writers)
        )
        isempty(missing_names) && continue
        _throw_missing_writer(model, status_key,
            "the status behind the initial power of a unit", missing_names,
            "Commit those units in some model, or make them must-run.")
    end
    return
end

# Every value a model reads from the simulation state by component name must have a model
# that writes it.
function _check_state_sources(sim::Simulation)
    models = get_models(sim)
    for n in unique(get_execution_order(sim))
        model = get_simulation_model(models, n)
        _reads_parameters_from_state(sim, n) && _check_parameter_sources(sim, model)
        _reads_initial_conditions_from_state(sim, n) &&
            _check_initial_power_sources(sim, model)
    end
    return
end
