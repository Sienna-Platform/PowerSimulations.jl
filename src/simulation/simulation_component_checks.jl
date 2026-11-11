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

const FEEDFORWARD_SOURCES_KEY = "feedforward_sources"

# A feedforward of a model restricted to the components that the decision models built
# before it write for its source key. `original` is the feedforward as attached, at
# position `index` of `device_model`. `left_out` holds the components outside those writers.
# With no such writer, or none left out, the feedforward is kept whole (`restricted = false`).
struct _FeedforwardRestriction
    device_model::IOM.DeviceModel
    index::Int
    original::AbstractAffectFeedforward
    source_key::OptimizationContainerKey
    affected_keys::Vector{OptimizationContainerKey}
    left_out::Set{String}
    restricted::Bool
end

_template_device_models(model) =
    Iterators.flatten((
        values(POM.get_device_models(get_template(model))),
        values(POM.get_branch_models(get_template(model))),
    ))

_feedforward_device_names(ff, devices) =
    Set{String}(PSY.get_name(d) for d in POM.feedforward_devices(ff, devices))

# The decision models built before `model` and those built after it. The emulator is built
# after every decision model.
function _decision_models_around(sim::Simulation, model::DecisionModel)
    decision_models = get_decision_models(get_models(sim))
    ix = findfirst(m -> m === model, decision_models)
    return decision_models[1:(ix - 1)], decision_models[(ix + 1):end]
end

function _decision_models_around(sim::Simulation, ::EmulationModel)
    decision_models = get_decision_models(get_models(sim))
    return decision_models, empty(decision_models)
end

# Puts back the feedforwards of `model` as they were attached, so a later build or sequence
# starts from them.
function _restore_feedforwards!(model::Union{DecisionModel, EmulationModel})
    restrictions = pop!(IOM.get_internal(model).ext, FEEDFORWARD_SOURCES_KEY, nothing)
    isnothing(restrictions) && return
    for restriction in restrictions::Vector{_FeedforwardRestriction}
        IOM.get_feedforwards(restriction.device_model)[restriction.index] =
            restriction.original
    end
    return
end

# Applies each feedforward of `model` that can leave components out only to the components
# that the decision models built before `model` write for its source key. A component
# outside them is built as if the feedforward were not attached. A source key that no such
# model writes leaves the feedforward whole for `_check_state_sources` to report.
function _restrict_feedforwards!(
    sim::Simulation,
    model::Union{DecisionModel, EmulationModel},
)
    _restore_feedforwards!(model)
    built, _ = _decision_models_around(sim, model)
    restrictions = _FeedforwardRestriction[]
    for device_model in _template_device_models(model)
        feedforwards = IOM.get_feedforwards(device_model)
        isempty(feedforwards) && continue
        devices = collect(IOM.get_available_components(device_model, get_system(model)))
        for (ix, ff) in enumerate(feedforwards)
            POM.supports_component_subset(ff) || continue
            source_key = POM.get_optimization_container_key(ff)
            writers = _state_writers(built, source_key)
            restricted_ff = POM.restrict_feedforward(ff, writers)
            left_out = setdiff(
                _feedforward_device_names(ff, devices),
                _feedforward_device_names(restricted_ff, devices),
            )
            restricted = !isempty(writers) && !isempty(left_out)
            restricted && (feedforwards[ix] = restricted_ff)
            push!(
                restrictions,
                _FeedforwardRestriction(device_model, ix, ff, source_key,
                    collect(OptimizationContainerKey, POM.get_affected_values(ff)),
                    left_out, restricted),
            )
        end
    end
    IOM.get_internal(model).ext[FEEDFORWARD_SOURCES_KEY] = restrictions
    return
end

function _prepare_feedforwards!(
    sim::Simulation,
    model::Union{DecisionModel, EmulationModel},
)
    if get_allow_different_components(sim)
        _restrict_feedforwards!(sim, model)
    else
        _restore_feedforwards!(model)
    end
    return
end

# A component left out of a feedforward because its only writers are built after the model
# would never be set from the state. Each restriction that leaves components out is logged.
function _check_feedforward_restrictions(sim::Simulation, model, reads_parameters::Bool)
    restrictions = get(IOM.get_internal(model).ext, FEEDFORWARD_SOURCES_KEY, nothing)
    isnothing(restrictions) && return
    _, later = _decision_models_around(sim, model)
    for restriction in restrictions::Vector{_FeedforwardRestriction}
        (; source_key, affected_keys, left_out, restricted) = restriction
        isempty(left_out) && continue
        source = encode_key_as_string(source_key)
        affected = join(encode_key_as_string.(affected_keys), ", ")
        late = intersect(left_out, _state_writers(later, source_key))
        if reads_parameters && !isempty(late)
            throw(
                IS.ConflictingInputsError(
                    "$(get_name(model)) is built before the model that writes $source \
                     for $(_shown_names(late)), so its feedforward on $affected leaves \
                     them out. List the models that write a feedforward's source before \
                     the models that read it.",
                ),
            )
        end
        restricted || continue
        n_left_out = length(left_out)
        @warn "$(get_name(model)) leaves $n_left_out $(n_left_out == 1 ? "component" : "components") \
               out of the feedforward on $affected, since no model built before it writes \
               $source for them: $(_shown_names(left_out))"
    end
    return
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
            missing_names, _parameter_source_remedy(sim, model, source_key, writers))
    end
    return
end

# Allowing different components restricts each feedforward that supports a component subset
# to the components its source's decision-model `writers` hold, so the hint applies only to
# those feedforwards and only when such a writer exists.
function _parameter_source_remedy(sim::Simulation, model, source_key, writers)
    remedy = "Give a model that writes the source every unit the feedforward sets."
    (get_allow_different_components(sim) || isempty(writers)) && return remedy
    restrictable = any(
        POM.supports_component_subset(ff) &&
        POM.get_optimization_container_key(ff) == source_key for
        device_model in _template_device_models(model) for
        ff in IOM.get_feedforwards(device_model)
    )
    restrictable || return remedy
    return "$remedy If the models are meant to hold different components, \
            $_ALLOW_DIFFERENT_COMPONENTS_HINT."
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
        reads_parameters = _reads_parameters_from_state(sim, n)
        _check_feedforward_restrictions(sim, model, reads_parameters)
        reads_parameters && _check_parameter_sources(sim, model)
        _reads_initial_conditions_from_state(sim, n) &&
            _check_initial_power_sources(sim, model)
    end
    return
end
