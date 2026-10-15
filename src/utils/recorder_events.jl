"""
All events subtyped from this need to be recorded under :simulation_status.
"""
abstract type AbstractSimulationStatusEvent <: IS.AbstractRecorderEvent end

struct SimulationStepEvent <: AbstractSimulationStatusEvent
    common::IS.RecorderEventCommon
    simulation_time::Dates.DateTime
    step::Int
    status::String
end

function SimulationStepEvent(
    simulation_time::Dates.DateTime,
    step::Int,
    status::AbstractString,
)
    return SimulationStepEvent(
        IS.RecorderEventCommon("SimulationStepEvent"),
        simulation_time,
        step,
        status,
    )
end

struct ProblemExecutionEvent <: AbstractSimulationStatusEvent
    common::IS.RecorderEventCommon
    simulation_time::Dates.DateTime
    step::Int
    model_name::Symbol
    status::String
    termination_status::Symbol
    objective_value::Float64
    solve_time_s::Float64
    exec_time_s::Float64
end

function ProblemExecutionEvent(
    simulation_time::Dates.DateTime,
    step::Int,
    model_name::Symbol,
    status::AbstractString,
)
    return ProblemExecutionEvent(
        IS.RecorderEventCommon("ProblemExecutionEvent"),
        simulation_time,
        step,
        model_name,
        status,
        :OPTIMIZE_NOT_CALLED,
        NaN,
        NaN,
        NaN,
    )
end

function ProblemExecutionEvent(
    simulation_time::Dates.DateTime,
    step::Int,
    model::IOM.AbstractOptimizationModel,
    status::AbstractString,
    exec_time_s::Float64,
)
    container = get_optimization_container(model)
    stats = get_optimizer_stats(container)
    if status == "done" && get_run_status(model) == RunStatus.SUCCESSFULLY_FINALIZED
        termination_status =
            Symbol(JuMP.MOI.TerminationStatusCode(stats.termination_status))
        objective_value = stats.objective_value
    else
        # IOM writes the optimizer stats only for a successful solve.
        termination_status = Symbol(JuMP.termination_status(get_jump_model(container)))
        objective_value = NaN
    end
    return ProblemExecutionEvent(
        IS.RecorderEventCommon("ProblemExecutionEvent"),
        simulation_time,
        step,
        get_name(model),
        status,
        termination_status,
        objective_value,
        stats.timed_solve_time,
        exec_time_s,
    )
end

"""
The initial conditions of one key after one model execution, with one entry per device.
"""
struct InitialConditionUpdateEvent <: IS.AbstractRecorderEvent
    common::IS.RecorderEventCommon
    simulation_time::Dates.DateTime
    initial_condition_type::String
    component_type::String
    model_name::Symbol
    device_names::Vector{String}
    new_values::Vector{Float64}
    previous_values::Vector{Float64}
end

# A missing initial-condition value is recorded as NaN, which the recorder writes as null.
_recorded_ic_value(::Nothing) = NaN
_recorded_ic_value(x) = x

function InitialConditionUpdateEvent(
    simulation_time::Dates.DateTime,
    ::InitialConditionKey{T, U},
    initial_conditions::Vector{<:InitialCondition},
    previous_values::Vector,
    model_name::Symbol,
) where {T <: InitialConditionType, U <: PSY.Component}
    return InitialConditionUpdateEvent(
        IS.RecorderEventCommon("InitialConditionUpdateEvent"),
        simulation_time,
        IS.strip_module_name(T),
        IS.strip_module_name(U),
        model_name,
        get_component_name.(initial_conditions),
        Float64[_recorded_ic_value(get_condition(ic)) for ic in initial_conditions],
        Float64[_recorded_ic_value(x) for x in previous_values],
    )
end

struct ParameterUpdateEvent <: IS.AbstractRecorderEvent
    common::IS.RecorderEventCommon
    simulation_time::Dates.DateTime
    parameter_type::String
    component_type::String
    tag::String
    model_name::Symbol
end

function ParameterUpdateEvent(
    ::Type{T},
    ::Type{U},
    tag::String,
    simulation_time::Dates.DateTime,
    model_name::Symbol,
) where {T <: ParameterType, U}
    return ParameterUpdateEvent(
        IS.RecorderEventCommon("ParameterUpdateEvent"),
        simulation_time,
        IS.strip_module_name(T),
        IS.strip_module_name(U),
        tag,
        model_name,
    )
end

function ParameterUpdateEvent(
    ::Type{T},
    ::Type{U},
    attributes::TimeSeriesAttributes,
    simulation_time::Dates.DateTime,
    model_name::Symbol,
) where {T <: ParameterType, U}
    return ParameterUpdateEvent(
        T,
        U,
        attributes.name,
        simulation_time,
        model_name,
    )
end

function ParameterUpdateEvent(
    ::Type{T},
    ::Type{U},
    ::NoAttributes,
    simulation_time::Dates.DateTime,
    model_name::Symbol,
) where {T <: ParameterType, U}
    return ParameterUpdateEvent(T, U, "", simulation_time, model_name)
end

function ParameterUpdateEvent(
    ::Type{T},
    ::Type{U},
    ::EventParametersAttributes{S},
    simulation_time::Dates.DateTime,
    model_name::Symbol,
) where {T <: ParameterType, U, S <: IS.SupplementalAttribute}
    return ParameterUpdateEvent(
        T,
        U,
        IS.strip_module_name(S),
        simulation_time,
        model_name,
    )
end

function ParameterUpdateEvent(
    ::Type{T},
    ::Type{U},
    attributes::VariableValueAttributes,
    simulation_time::Dates.DateTime,
    model_name::Symbol,
) where {T <: ParameterType, U}
    return ParameterUpdateEvent(
        T,
        U,
        get_attribute_key_string(attributes),
        simulation_time,
        model_name,
    )
end

function ParameterUpdateEvent(
    ::Type{T},
    ::Type{U},
    attributes::CostFunctionAttributes,
    simulation_time::Dates.DateTime,
    model_name::Symbol,
) where {T <: ParameterType, U}
    return ParameterUpdateEvent(
        T,
        U,
        get_variable_types_string(attributes),
        simulation_time,
        model_name,
    )
end

struct StateUpdateEvent <: IS.AbstractRecorderEvent
    common::IS.RecorderEventCommon
    simulation_time::Dates.DateTime
    model_name::Symbol
    state_type::String
end

function StateUpdateEvent(simulation_time::Dates.DateTime, model_name, state_type::String)
    return StateUpdateEvent(
        IS.RecorderEventCommon("StateUpdateEvent"),
        simulation_time,
        Symbol(model_name),
        state_type,
    )
end

function get_simulation_step_range(filename::AbstractString, step::Int)
    events = IS.list_recorder_events(SimulationStepEvent, filename, x -> x.step == step)
    if length(events) != 2
        throw(
            ArgumentError(
                "$filename does not have two SimulationStepEvents for step = $step",
            ),
        )
    end

    if events[1].status != "start" || events[2].status != "done"
        throw(
            ArgumentError(
                "$filename does not contain start and done events for step = $step",
            ),
        )
    end

    return (start = events[1].simulation_time, done = events[2].simulation_time)
end

function get_simulation_model_range(filename::AbstractString, step::Int, model::String)
    events = IS.list_recorder_events(
        ProblemExecutionEvent,
        filename,
        x -> x.step == step && x.model_name == Symbol(model),
    )
    if length(events) != 2
        throw(
            ArgumentError(
                "$filename does not have two ProblemExecutionEvent for step = $step model = $model",
            ),
        )
    end

    if events[1].status != "start" || events[2].status != "done"
        throw(
            ArgumentError(
                "$filename does not contain start and done events for step = $step model = $model",
            ),
        )
    end

    return (start = events[1].simulation_time, done = events[2].simulation_time)
end

function _filter_by_type_range!(events::Vector{<:IS.AbstractRecorderEvent}, time_range)
    return filter!(
        x -> x.simulation_time >= time_range.start && x.simulation_time <= time_range.done,
        events,
    )
end

function _get_recorder_filename(output_dir, recorder_name)
    return joinpath(output_dir, "recorder", recorder_name * ".log")
end

function _get_simulation_status_recorder_filename(output_dir)
    return _get_recorder_filename(output_dir, "simulation_status")
end

function _get_simulation_recorder_filename(output_dir)
    return _get_recorder_filename(output_dir, "execution")
end

"""
    list_simulation_events(
        ::Type{T},
        output_dir::AbstractString,
        filter_func::Union{Nothing, Function} = nothing;
        step = nothing,
        model = nothing,
    ) where {T <: IS.AbstractRecorderEvent}

List simulation events of type T in a simulation output directory.

# Arguments

  - `output_dir::AbstractString`: Simulation output directory
  - `filter_func::Union{Nothing, Function} = nothing`: Refer to [`show_simulation_events`](@ref).
  - `step::Int = nothing`: Filter events by step.
  - `model_name::String = nothing`: Filter events by model.
"""
function list_simulation_events(
    ::Type{T},
    output_dir::AbstractString,
    filter_func::Union{Nothing, Function} = nothing;
    step = nothing,
    model_name::Union{String, Nothing} = nothing,
) where {T <: IS.AbstractRecorderEvent}
    recorder_file = _get_simulation_recorder_filename(output_dir)
    events = IS.list_recorder_events(T, recorder_file, filter_func)

    if !isnothing(model_name)
        name = Symbol(model_name)
        filter!(x -> x.model_name == name, events)
    end

    if !isnothing(step)
        # A model executes once per simulation time, so (model_name, simulation_time)
        # identifies the execution that wrote an event.
        starts = IS.list_recorder_events(
            ProblemExecutionEvent,
            _get_simulation_status_recorder_filename(output_dir),
            x -> x.step == step && x.status == "start",
        )
        executions = Set((x.model_name, x.simulation_time) for x in starts)
        filter!(x -> (x.model_name, x.simulation_time) in executions, events)
    end

    return events
end

function list_simulation_events(
    ::Type{T},
    output_dir::AbstractString,
    filter_func::Union{Nothing, Function} = nothing;
    kwargs...,
) where {T <: AbstractSimulationStatusEvent}
    recorder_file = _get_simulation_status_recorder_filename(output_dir)
    return IS.list_recorder_events(T, recorder_file, filter_func)
end

"""
    show_simulation_events(
        ::Type{T},
        output_dir::AbstractString,
        filter_func::Union{Nothing,Function} = nothing;
        step = nothing,
        model_name = nothing,
        wall_time = false,
        kwargs...,
    ) where { T <: IS.AbstractRecorderEvent}

Show all simulation events of type T in a simulation output directory.

# Arguments

  - `::Type{T}`: Recorder event type
  - `output_dir::AbstractString`: Simulation output directory
  - `filter_func::Union{Nothing, Function} = nothing`: Refer to [`show_recorder_events`](@ref).
  - `step::Int = nothing`: Filter events by step.
  - `model_name::String = nothing`: Filter events by model.
  - `wall_time = false`: If true, show the wall_time timestamp.
"""
function show_simulation_events(
    ::Type{T},
    output_dir::AbstractString,
    filter_func::Union{Nothing, Function} = nothing;
    step = nothing,
    model_name = nothing,
    wall_time = false,
    kwargs...,
) where {T <: IS.AbstractRecorderEvent}
    show_simulation_events(
        stdout,
        T,
        output_dir,
        filter_func;
        step = step,
        model_name = model_name,
        wall_time = wall_time,
        kwargs...,
    )
end

function show_simulation_events(
    ::Type{T},
    output_dir::AbstractString,
    filter_func::Union{Nothing, Function} = nothing;
    wall_time = false,
    kwargs...,
) where {T <: AbstractSimulationStatusEvent}
    show_simulation_events(
        stdout,
        T,
        output_dir,
        filter_func;
        wall_time = wall_time,
        kwargs...,
    )
end

function show_simulation_events(
    io::IO,
    ::Type{T},
    output_dir::AbstractString,
    filter_func::Union{Nothing, Function} = nothing;
    wall_time = false,
    kwargs...,
) where {T <: AbstractSimulationStatusEvent}
    events = list_simulation_events(T, output_dir, filter_func)
    show_recorder_events(io, events, filter_func; wall_time = wall_time, kwargs...)
end

function show_simulation_events(
    io::IO,
    ::Type{T},
    output_dir::AbstractString,
    filter_func::Union{Nothing, Function} = nothing;
    step = nothing,
    model_name::Union{String, Nothing} = nothing,
    wall_time = false,
    kwargs...,
) where {T <: IS.AbstractRecorderEvent}
    events =
        list_simulation_events(
            T,
            output_dir,
            filter_func;
            step = step,
            model_name = model_name,
        )
    show_recorder_events(io, events, filter_func; wall_time = wall_time, kwargs...)
end

"""
    show_recorder_events(
        ::Type{T},
        filename::AbstractString,
        filter_func::Union{Nothing, Function} = nothing;
        wall_time = false,
        kwargs...,
    ) where {T <: IS.AbstractRecorderEvent}

Show the events of type T in a recorder file.

# Arguments

  - `::Type{T}`: Recorder event type
  - `filename::AbstractString`: recorder filename
  - `filter_func::Union{Nothing, Function} = nothing`: Optional function that accepts an event
    of type T and returns a Bool. Apply this function to each event and only return events
    where the result is true.
  - `wall_time = false`: If true, show the wall_time timestamp.
"""
function show_recorder_events(
    ::Type{T},
    filename::AbstractString,
    filter_func::Union{Nothing, Function} = nothing;
    wall_time = false,
    kwargs...,
) where {T <: IS.AbstractRecorderEvent}
    show_recorder_events(stdout, T, filename, filter_func; wall_time = wall_time, kwargs...)
end

function show_recorder_events(
    io::IO,
    ::Type{T},
    filename::AbstractString,
    filter_func::Union{Nothing, Function} = nothing;
    wall_time = false,
    kwargs...,
) where {T <: IS.AbstractRecorderEvent}
    events = IS.list_recorder_events(T, filename, filter_func)
    show_recorder_events(io, events, filter_func; wall_time = wall_time, kwargs...)
end

function show_recorder_events(
    io::IO,
    events::Vector{T},
    filter_func::Union{Nothing, Function} = nothing;
    wall_time = false,
    exclude_columns = Set{String}(),
    kwargs...,
) where {T <: IS.AbstractRecorderEvent}
    IS.show_recorder_events(
        io,
        events;
        exclude_columns = _excluded_columns(wall_time, exclude_columns),
        kwargs...,
    )
end

"""
Show one row per device: an [`InitialConditionUpdateEvent`](@ref) holds every device of one
initial-condition key.
"""
function show_recorder_events(
    io::IO,
    events::Vector{InitialConditionUpdateEvent},
    filter_func::Union{Nothing, Function} = nothing;
    wall_time = false,
    exclude_columns = Set{String}(),
    kwargs...,
)
    if isempty(events)
        @warn "Found no events of type InitialConditionUpdateEvent"
        return
    end
    table = DataFrame([
        (
            timestamp = IS.get_timestamp(event),
            simulation_time = event.simulation_time,
            model_name = event.model_name,
            initial_condition_type = event.initial_condition_type,
            component_type = event.component_type,
            device_name = device_name,
            new_value = event.new_values[i],
            previous_value = event.previous_values[i],
        ) for event in events for (i, device_name) in enumerate(event.device_names)
    ])
    excluded = _excluded_columns(wall_time, exclude_columns)
    PrettyTables.pretty_table(
        io,
        DataFrames.select(table, Not(DataFrames.Cols(in(excluded))));
        kwargs...,
    )
    return
end

function _excluded_columns(wall_time::Bool, exclude_columns)
    excluded = Set{String}(exclude_columns)
    wall_time || push!(excluded, "timestamp")
    return excluded
end
