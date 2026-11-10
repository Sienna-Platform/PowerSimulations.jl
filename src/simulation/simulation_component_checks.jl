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
