function check_simulation_chronology(
    horizons::OrderedDict{Symbol, Dates.Millisecond},
    intervals::OrderedDict{Symbol, Dates.Millisecond},
    resolutions::OrderedDict{Symbol, Dates.Millisecond},
)
    models = collect(keys(resolutions))

    for (model, horizon_time) in horizons
        if horizon_time < intervals[model]
            throw(IS.ConflictingInputsError("horizon ($horizon_time) is
                                shorter than interval ($(intervals[model])) for $(model)"))
        end
    end

    for i in 2:length(models)
        upper_level_model = models[i - 1]
        lower_level_model = models[i]
        if horizons[lower_level_model] > horizons[upper_level_model]
            throw(
                IS.ConflictingInputsError(
                    "The lookahead length $(horizons[upper_level_model]) in model $(upper_level_model) is insufficient to syncronize with $(lower_level_model)",
                ),
            )
        end
        if intervals[lower_level_model] == Dates.Millisecond(0)
            throw(
                IS.ConflictingInputsError(
                    "The interval in model $(lower_level_model) is invalid.",
                ),
            )
        end
        if (intervals[upper_level_model] % intervals[lower_level_model]) !=
           Dates.Millisecond(0)
            throw(
                IS.ConflictingInputsError(
                    "The intervals are not compatible for simulation. The interval in model $(upper_level_model) needs to be a mutiple of the interval $(lower_level_model) for a consistent time coordination.",
                ),
            )
        end
    end
    return
end

"""
_calculate_interval_inner_counts(intervals::OrderedDict{String,<:Dates.TimePeriod})

Calculates how many times a problem is executed for every interval of the previous problem
"""
function _calculate_interval_inner_counts(intervals::OrderedDict{Symbol, Dates.Millisecond})
    order = collect(keys(intervals))
    reverse_order = length(intervals):-1:1
    interval_run_counts = Vector{Int}(undef, length(intervals))
    interval_run_counts[1] = 1
    for k in reverse_order[1:(end - 1)]
        model_name = order[k]
        previous_model_name = order[k - 1]
        problem_interval = intervals[model_name]
        previous_problem_interval = intervals[previous_model_name]
        if Dates.Millisecond(previous_problem_interval % problem_interval) !=
           Dates.Millisecond(0)
            throw(
                IS.ConflictingInputsError(
                    "The interval configuration provided results in a fractional number of executions of problem $model_name",
                ),
            )
        end
        interval_run_counts[k] = previous_problem_interval / problem_interval
        @debug "problem $k is executed $(interval_run_counts[k]) time within each interval of problem $(k-1)"
    end
    return interval_run_counts
end

"""
Function calculates the total number of problem executions in the simulation and allocates the appropiate vector
"""
function _allocate_execution_order(interval_run_counts::Vector{Int})
    total_size_of_vector = 0
    for k in eachindex(interval_run_counts)
        mult = 1
        for i in 1:k
            mult *= interval_run_counts[i]
        end
        total_size_of_vector += mult
    end
    return -1 * ones(Int, total_size_of_vector)
end

function _fill_execution_order(
    execution_order::Vector{Int},
    interval_run_counts::Vector{Int},
)
    function _fill_problem(index::Int, problem::Int)
        last_problem = problems[end]
        if problem < last_problem
            next_problem = problem + 1
            for i in 1:interval_run_counts[next_problem]
                index = _fill_problem(index, next_problem)
            end
        end
        execution_order[index] = problem
        index -= 1
    end

    index = length(execution_order)
    problems = sort!(collect(keys(interval_run_counts)))
    _fill_problem(index, problems[1])
    return
end

function _get_execution_order_vector(intervals::OrderedDict{Symbol, Dates.Millisecond})
    length(intervals) == 1 && return [1]
    interval_run_counts = _calculate_interval_inner_counts(intervals)
    execution_order_vector = _allocate_execution_order(interval_run_counts)
    _fill_execution_order(execution_order_vector, interval_run_counts)
    @assert isempty(findall(x -> x == -1, execution_order_vector))
    return execution_order_vector
end

function _get_num_executions_by_model(
    models::SimulationModels,
    execution_order::Vector{Int},
)
    model_names = get_model_names(models)
    executions_by_model = OrderedDict(x => 0 for x in model_names)
    for number in execution_order
        executions_by_model[model_names[number]] += 1
    end
    return executions_by_model
end

function _add_feedforward_to_model(
    sim_model::IOM.AbstractOptimizationModel,
    ff::T,
    ::Type{U},
) where {T <: AbstractAffectFeedforward, U <: PSY.Device}
    device_model = get_model(get_template(sim_model), get_component_type(ff))
    if isnothing(device_model)
        model_name = get_name(sim_model)
        throw(
            IS.ConflictingInputsError(
                "Device model $(get_component_type(ff)) not found in model $model_name",
            ),
        )
    end
    @debug "attaching $T to $(get_component_type(ff))"
    attach_feedforward!(device_model, ff)
    return
end

function _add_feedforward_to_model(
    sim_model::IOM.AbstractOptimizationModel,
    ff::T,
    ::Type{U},
) where {T <: AbstractAffectFeedforward, U <: PSY.Service}
    if get_feedforward_meta(ff) != CONTAINER_KEY_EMPTY_META
        service_model = get_model(
            get_template(sim_model),
            get_component_type(ff),
            get_feedforward_meta(ff),
        )
        if isnothing(service_model)
            throw(
                IS.ConflictingInputsError(
                    "Service model $(get_component_type(ff)) not found in model $(get_name(sim_model))",
                ),
            )
        end
        @debug "attaching $T to $(PSI.get_component_type(ff)) $(PSI.get_feedforward_meta(ff))"
        attach_feedforward!(service_model, ff)
    else
        service_found = false
        for (key, model) in get_service_models(get_template(sim_model))
            if key == Symbol(get_component_type(ff))
                service_found = true
                @debug "attaching $T to $(get_component_type(ff))"
                attach_feedforward!(model, ff)
            end
        end
    end
    return
end

function _attach_feedforwards(models::SimulationModels, feedforwards)
    names = Set(string.(get_model_names(models)))
    ff_dict = Dict{Symbol, Vector}()
    for (model_name, model_feedforwards) in feedforwards
        if model_name ∈ names
            model_name_symbol = Symbol(model_name)
            ff_dict[model_name_symbol] = model_feedforwards
            for ff in model_feedforwards
                sim_model = get_simulation_model(models, model_name_symbol)
                _add_feedforward_to_model(sim_model, ff, get_component_type(ff))
            end
        else
            error("Model $model_name not present in the SimulationModels")
        end
    end
    return ff_dict
end

"""
Attach every event model to the template of every model in the simulation.

Discovery and validation happen in PowerOperationsModels at build time: attaching an
event model to a template is what makes `build!` find the system's outage attributes,
populate the event model's `attribute_device_map`, and distribute the event to the
matching `DeviceModel`s. This function used to do that work itself, per model, against
a map keyed by model name.
"""
function _attach_events!(
    models::SimulationModels,
    event_models::Vector{<:POM.EventModel},
)
    isempty(event_models) && return
    sim_models = IOM.AbstractOptimizationModel[get_decision_models(models)...]
    em_model = get_emulation_model(models)
    isnothing(em_model) || push!(sim_models, em_model)
    for model in sim_models, event_model in event_models
        set_event_model!(get_template(model), event_model)
    end
    return
end

"""
    SimulationSequence(
        models::SimulationModels,
        feedforward::Dict{String, Vector{<:AbstractAffectFeedforward}}
        ini_cond_chronology::InitialConditionChronology
    )

Construct the simulation sequence between decision and emulation models.

# Arguments

  - `models::SimulationModels`: Vector of decisions and emulation models.
  - `feedforward = Dict{String, Vector{<:AbstractAffectFeedforward}}()`: Optional dictionary to specify how information
    and variables are exchanged between decision and emulation models.
  - `ini_cond_chronology::InitialConditionChronology =  InterProblemChronology()`: Define
    information sharing model between stages with [`InterProblemChronology`](@ref)

# Example

```julia
template_uc = PowerOperationsProblemTemplate(NetworkModel(CopperPlateNetworkModel))
set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
template_ed = PowerOperationsProblemTemplate(NetworkModel(CopperPlateNetworkModel))
set_device_model!(template_ed, ThermalStandard, ThermalBasicDispatch)
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
```
"""
mutable struct SimulationSequence
    horizons::OrderedDict{Symbol, Dates.Millisecond}
    intervals::OrderedDict{Symbol, Dates.Millisecond}
    feedforwards::Dict{Symbol, Vector{<:AbstractAffectFeedforward}}
    ini_cond_chronology::InitialConditionChronology
    execution_order::Vector{Int}
    executions_by_model::OrderedDict{Symbol, Int}
    current_execution_index::Int64
    uuid::Base.UUID
    events::Vector{<:POM.EventModel}

    function SimulationSequence(;
        models::SimulationModels,
        feedforwards = Dict{String, Vector{<:AbstractAffectFeedforward}}(),
        ini_cond_chronology = InterProblemChronology(),
        events = POM.EventModel[],
    )
        # Allow strings or symbols as keys; convert to symbols.
        intervals = determine_intervals(models)
        horizons = determine_horizons!(models)
        resolutions = determine_resolutions(models)

        if length(models.decision_models) > 1
            check_simulation_chronology(horizons, intervals, resolutions)
        end

        if length(models.decision_models) == 1
            # TODO: Not implemented yet
            # ini_cond_chronology = IntraProblemChronology()
        end

        execution_order = _get_execution_order_vector(intervals)
        executions_by_model = _get_num_executions_by_model(models, execution_order)
        sequence_uuid = IS.make_uuid()
        initialize_simulation_internals!(models, sequence_uuid)
        _attach_events!(models, events)
        new(
            horizons,
            intervals,
            _attach_feedforwards(models, feedforwards),
            ini_cond_chronology,
            execution_order,
            executions_by_model,
            0,
            sequence_uuid,
            events,
        )
    end
end

get_step_resolution(sequence::SimulationSequence) = first(values(sequence.intervals))

function IOM.get_interval(sequence::SimulationSequence, problem::Symbol)
    return sequence.intervals[problem]
end

function IOM.get_interval(sequence::SimulationSequence, model::DecisionModel)
    return sequence.intervals[get_name(model)]
end

get_events(sequence::SimulationSequence) = sequence.events
get_execution_order(sequence::SimulationSequence) = sequence.execution_order
