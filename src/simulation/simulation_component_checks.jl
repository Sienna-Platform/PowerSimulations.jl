const _SHOWN_NAMES_LIMIT = 5

const _ALLOW_DIFFERENT_COMPONENTS_HINT = "pass `allow_different_components = true` to \
    `SimulationSequence` if this is intended"

# The first five names in sorted order, then a count of the rest.
function _shown_names(names)
    sorted = sort!(collect(String, names))
    shown = join(first(sorted, _SHOWN_NAMES_LIMIT), ", ")
    rest = length(sorted) - _SHOWN_NAMES_LIMIT
    if rest > 0
        return "$shown and $rest more"
    end
    return shown
end

const _ComponentHolders = Vector{Pair{Symbol, Set{String}}}

function _throw_different_components(
    label::String,
    (ref_name, ref)::Pair{Symbol, Set{String}},
    (name, components)::Pair{Symbol, Set{String}},
)
    details = String[]
    absent = setdiff(ref, components)
    isempty(absent) ||
        push!(details, "$name lacks $(_shown_names(absent)), which $ref_name has")
    absent = setdiff(components, ref)
    isempty(absent) ||
        push!(details, "$ref_name lacks $(_shown_names(absent)), which $name has")
    throw(
        IS.ConflictingInputsError(
            "Models $ref_name and $name hold different components for $label: \
             $(join(details, "; ")). Give every model the same components, or \
             $_ALLOW_DIFFERENT_COMPONENTS_HINT",
        ),
    )
end

function _check_same_components(label::String, holders::_ComponentHolders)
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
        for key in get_container_keys(get_optimization_container(model))
            should_write_resulting_value(key) || continue
            components = Set{String}(first(_model_columns(model, key)))
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
            components = Set{String}(get_component_name(ic) for ic in ics)
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
# sequence allows different components. When it does, the emulator must still hold them all.
function _check_component_sets(sim::Simulation)
    models = get_models(sim)
    if get_allow_different_components(sim)
        _check_emulator_components(models, get_emulation_model(models))
    else
        _check_each_key(_state_key_components(models), "")
        _check_each_key(_initial_condition_components(models), "initial condition ")
    end
    return
end

_check_emulator_components(::SimulationModels, ::Nothing) = nothing

# The emulator is the full view of the system. The system state of a key the emulator writes
# takes the emulator's components only, so the emulator must hold every component that a
# decision model writes for the key.
function _check_emulator_components(models::SimulationModels, em::EmulationModel)
    em_name = get_name(em)
    holders = _state_key_components(models)
    for key in sort!(collect(keys(holders)); by = encode_key_as_string)
        _writes_key(em, key) || continue
        em_components = Set{String}(first(_model_columns(em, key)))
        for (name, components) in holders[key]
            absent = setdiff(components, em_components)
            isempty(absent) && continue
            throw(
                IS.ConflictingInputsError(
                    "Emulation model $em_name lacks the \
                     $(nameof(get_component_type(key))) components \
                     $(_shown_names(absent)) for $(encode_key_as_string(key)), which \
                     decision model $name has. The emulation model must hold every \
                     component of the decision models.",
                ),
            )
        end
    end
    return
end

# The components that `models` write for `key`. The decision state is written by the
# decision models; the system state also by the emulator.
function _state_writers(models, key::OptimizationContainerKey)
    names = Set{String}()
    for model in models
        _writes_key(model, key) || continue
        union!(names, first(_model_columns(model, key)))
    end
    return names
end

const FEEDFORWARD_SOURCES_KEY = "feedforward_sources"

# A feedforward of a model restricted to the components that the decision models built
# before it write for its source key. `original` is the feedforward as attached, at
# position `index` of `device_model`. `left_out` holds the components outside those
# writers.
struct _FeedforwardRestriction
    device_model::IOM.DeviceModel
    index::Int
    original::AbstractAffectFeedforward
    left_out::Set{String}
end

_template_device_models(model) =
    Iterators.flatten((
        values(POM.get_device_models(get_template(model))),
        values(POM.get_branch_models(get_template(model))),
    ))

_feedforward_device_names(ff, devices, sys) =
    Set{String}(PSY.get_name(d) for d in POM.feedforward_devices(ff, devices, sys))

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
    ext = IOM.get_internal(model).ext
    haskey(ext, FEEDFORWARD_SOURCES_KEY) || return
    for restriction in pop!(ext, FEEDFORWARD_SOURCES_KEY)::Vector{_FeedforwardRestriction}
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
    built, _ = _decision_models_around(sim, model)
    restrictions = _FeedforwardRestriction[]
    sys = get_system(model)
    for device_model in _template_device_models(model)
        feedforwards = IOM.get_feedforwards(device_model)
        isempty(feedforwards) && continue
        devices = collect(IOM.get_available_components(device_model, sys))
        for (ix, ff) in enumerate(feedforwards)
            POM.supports_component_subset(typeof(ff)) || continue
            writers = _state_writers(built, POM.get_optimization_container_key(ff))
            isempty(writers) && continue
            left_out = setdiff(_feedforward_device_names(ff, devices, sys), writers)
            isempty(left_out) && continue
            feedforwards[ix] = POM.restrict_feedforward(ff, writers, sys)
            push!(restrictions, _FeedforwardRestriction(device_model, ix, ff, left_out))
        end
    end
    IOM.get_internal(model).ext[FEEDFORWARD_SOURCES_KEY] = restrictions
    return
end

function _prepare_feedforwards!(
    sim::Simulation,
    model::Union{DecisionModel, EmulationModel},
)
    _restore_feedforwards!(model)
    get_allow_different_components(sim) && _restrict_feedforwards!(sim, model)
    return
end

# A component left out of a restricted feedforward because its only writers are built after
# the model would never be set from the state. Each restriction is logged. A feedforward kept
# whole is checked against every writer by `_check_parameter_sources`.
function _check_feedforward_restrictions(sim::Simulation, model, reads_parameters::Bool)
    ext = IOM.get_internal(model).ext
    haskey(ext, FEEDFORWARD_SOURCES_KEY) || return
    _, later = _decision_models_around(sim, model)
    for restriction in ext[FEEDFORWARD_SOURCES_KEY]::Vector{_FeedforwardRestriction}
        (; original, left_out) = restriction
        source_key = POM.get_optimization_container_key(original)
        source = encode_key_as_string(source_key)
        affected = join(encode_key_as_string.(POM.get_affected_values(original)), ", ")
        if reads_parameters
            late = intersect(left_out, _state_writers(later, source_key))
            isempty(late) || throw(
                IS.ConflictingInputsError(
                    "$(get_name(model)) is built before the model that writes $source \
                     for $(_shown_names(late)), so its feedforward on $affected leaves \
                     them out. List the models that write a feedforward's source before \
                     the models that read it.",
                ),
            )
        end
        n_left_out = length(left_out)
        if n_left_out == 1
            noun = "component"
        else
            noun = "components"
        end
        @warn "$(get_name(model)) leaves $n_left_out $noun out of the feedforward on \
               $affected, since no model built before it writes $source for them: \
               $(_shown_names(left_out))"
    end
    return
end

function _total_executions(sim::Simulation, n::Int)
    return count(==(n), get_execution_order(sim)) * get_steps(sim)
end

# Every execution but the simulation's first updates the parameters of the model it runs.
function _reads_parameters_from_state(sim::Simulation, n::Int)
    if first(get_execution_order(sim)) == n
        return _total_executions(sim, n) > 1
    end
    return _total_executions(sim, n) > 0
end

_reads_system_state(::InterProblemChronology) = true
_reads_system_state(::InitialConditionChronology) = false

# A model keeps its own initial conditions at its first execution.
function _reads_initial_conditions_from_state(sim::Simulation, n::Int)
    return _reads_system_state(get_ini_cond_chronology(sim)) &&
           _total_executions(sim, n) > 1
end

function _throw_missing_writer(model, source_key, what, missing_names, remedy)
    throw(
        IS.ConflictingInputsError(
            "$(get_name(model)) reads $(encode_key_as_string(source_key)) from the \
             simulation state for $what, but no model writes it for \
             $(_shown_names(missing_names)). $remedy",
        ),
    )
end

_reads_state_value(::VariableValueAttributes) = true
_reads_state_value(::IOM.ParameterAttributes) = false

# Each feedforward parameter is read from the decision state of its source key for every
# component of the parameter.
function _check_parameter_sources(sim::Simulation, model)
    decision_models = get_decision_models(get_models(sim))
    writers_by_key = Dict{OptimizationContainerKey, Set{String}}()
    for (key, parameter) in get_parameters(get_optimization_container(model))
        attributes = get_attributes(parameter)
        _reads_state_value(attributes) || continue
        source_key = get_attribute_key(attributes)
        writers = get!(writers_by_key, source_key) do
            _state_writers(decision_models, source_key)
        end
        names = string.(axes(get_parameter_array(parameter))[1])
        missing_names = setdiff(names, writers)
        isempty(missing_names) && continue
        _throw_missing_writer(model, source_key, encode_key_as_string(key),
            missing_names, _parameter_source_remedy(sim, model, key, source_key, writers))
    end
    return
end

# Allowing different components restricts each feedforward that supports a component subset
# to the components its source's decision-model `writers` hold, so the hint applies only to
# a parameter set by such a feedforward and only when such a writer exists.
function _parameter_source_remedy(
    sim::Simulation,
    model,
    parameter_key,
    source_key,
    writers,
)
    remedy = "Give a model that writes the source every unit the feedforward sets."
    (get_allow_different_components(sim) || isempty(writers)) && return remedy
    parameter_type = get_entry_type(parameter_key)
    component_type = get_component_type(parameter_key)
    restrictable = any(
        POM.supports_component_subset(typeof(ff)) &&
        POM.get_optimization_container_key(ff) == source_key &&
        POM.get_default_parameter_type(ff, component_type) === parameter_type for
        device_model in _template_device_models(model) for
        ff in IOM.get_feedforwards(device_model)
    )
    restrictable || return remedy
    return "$remedy If the models are meant to hold different components, \
            $_ALLOW_DIFFERENT_COMPONENTS_HINT."
end

# Every other initial condition reads only keys that its own model writes for the same
# components, so only the status behind an initial power can lack a writer.
function _check_initial_power_sources(sim::Simulation, model)
    for (key, ics) in get_initial_conditions(model)
        _check_initial_power_source(sim, model, key, ics)
    end
    return
end

_check_initial_power_source(::Simulation, _, ::InitialConditionKey, _) = nothing

# The initial power of a unit that is not must-run is taken with its status, read from the
# `OnVariable` of the system state. That state holds the components every model writes.
function _check_initial_power_source(
    sim::Simulation,
    model,
    key::InitialConditionKey{DevicePower, T},
    ics,
) where {T <: PSY.Component}
    status_key = VariableKey(OnVariable, T)
    system_states = get_system_states(get_simulation_state(sim))
    if has_dataset(system_states, status_key)
        status = get_dataset(system_states, status_key)
        in_state = name -> _has_component(status, name)
    else
        in_state = _ -> false
    end
    missing_names = Set{String}(
        get_component_name(ic) for ic in ics if
        !IOM.get_must_run(IOM.get_component(ic)) && !in_state(get_component_name(ic))
    )
    isempty(missing_names) && return
    _throw_missing_writer(model, status_key,
        "the status behind the initial power of a unit", missing_names,
        "Commit those units in some model, or make them must-run.")
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
