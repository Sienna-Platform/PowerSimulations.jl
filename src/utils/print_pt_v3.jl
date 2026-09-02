function Base.show(io::IO, ::MIME"text/plain", input::SimulationModels)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::SimulationModels)
    # The tf_html_simple format was eliminated from PrettyTables and it was added to PowerSystems
    _show_method(io, input, :html; stand_alone = false, table_format = PSY.tf_html_simple)
end

_get_model_type(::DecisionModel{T}) where {T <: DecisionProblem} = T
_get_model_type(::EmulationModel{T}) where {T <: EmulationProblem} = T

function _show_method(io::IO, sim_models::SimulationModels, backend::Symbol; kwargs...)
    println(io)
    header = ["Model Name", "Model Type", "Status", "Output Directory"]

    table = Matrix{Any}(undef, length(sim_models.decision_models), length(header))
    for (ix, model) in enumerate(sim_models.decision_models)
        table[ix, 1] = string(get_name(model))
        table[ix, 2] = IS.strip_module_name(string(_get_model_type(model)))
        table[ix, 3] = string(get_status(model))
        table[ix, 4] = get_output_dir(model)
    end

    PrettyTables.pretty_table(
        io,
        table;
        column_labels = header,
        backend = backend,
        title = "Decision Models",
        alignment = :l,
        kwargs...,
    )

    if !isnothing(sim_models.emulation_model)
        println(io)
        table = Matrix{Any}(undef, 1, length(header))
        table[1, 1] = string(get_name(sim_models.emulation_model))
        table[1, 2] =
            IS.strip_module_name(string(_get_model_type(sim_models.emulation_model)))
        table[1, 3] = string(get_status(sim_models.emulation_model))
        table[1, 4] = get_output_dir(sim_models.emulation_model)

        PrettyTables.pretty_table(
            io,
            table;
            column_labels = header,
            backend = backend,
            title = "Emulator Models",
            alignment = :l,
            kwargs...,
        )
    else
        println(io)
        println(io, "No Emulator Model Specified")
    end
end

function Base.show(io::IO, ::MIME"text/plain", input::SimulationSequence)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::SimulationSequence)
    # The tf_html_simple format was eliminated from PrettyTables and it was added to PowerSystems
    _show_method(io, input, :html; stand_alone = false, table_format = PSY.tf_html_simple)
end

function _show_method(io::IO, sequence::SimulationSequence, backend::Symbol; kwargs...)
    println(io)
    table = [
        "Simulation Step Interval" Dates.Hour(get_step_resolution(sequence))
        "Number of Problems" length(sequence.executions_by_model)
    ]

    PrettyTables.pretty_table(
        io,
        table;
        backend = backend,
        show_column_labels = false,
        title = "Simulation Sequence",
        alignment = :l,
        kwargs...,
    )

    println(io)
    header = ["Model Name", "Horizon", "Interval", "Executions Per Step"]

    table = Matrix{Any}(undef, length(sequence.executions_by_model), length(header))
    for (ix, (model, executions)) in enumerate(sequence.executions_by_model)
        table[ix, 1] = string(model)
        table[ix, 2] = Dates.canonicalize(sequence.horizons[model])
        table[ix, 3] = Dates.canonicalize(sequence.intervals[model])
        table[ix, 4] = executions
    end

    PrettyTables.pretty_table(
        io,
        table;
        column_labels = header,
        backend = backend,
        title = "Simulation Problems",
        alignment = :l,
    )

    if !isempty(sequence.feedforwards)
        println(io)
        header = ["Model Name", "Feed Forward Type"]
        table = Matrix{Any}(undef, length(sequence.feedforwards), length(header))
        for (ix, (k, ff)) in enumerate(sequence.feedforwards)
            table[ix, 1] = k
            table[ix, 2] = join(string.(typeof.(ff)), " ")
        end
        PrettyTables.pretty_table(
            io,
            table;
            column_labels = header,
            backend = backend,
            title = "Feedforwards",
            alignment = :l,
            kwargs...,
        )
    end
end

function Base.show(io::IO, ::MIME"text/plain", input::Simulation)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::Simulation)
    # The tf_html_simple format was eliminated from PrettyTables and it was added to PowerSystems
    _show_method(io, input, :html; stand_alone = false, table_format = PSY.tf_html_simple)
end

function _get_initial_time_for_show(sim::Simulation)
    ini_time = get_initial_time(sim)
    if isnothing(ini_time)
        return "Unset Initial Time"
    else
        return string(ini_time)
    end
end

function _get_build_status_for_show(sim::Simulation)
    internal = sim.internal
    if isnothing(internal)
        return "EMPTY"
    else
        return string(internal.build_status)
    end
end

function _get_run_status_for_show(sim::Simulation)
    internal = sim.internal
    if isnothing(internal)
        return "NOT_READY"
    else
        return string(internal.status)
    end
end

function _show_method(io::IO, sim::Simulation, backend::Symbol; kwargs...)
    table = [
        "Simulation Name" get_name(sim)
        "Build Status" _get_build_status_for_show(sim)
        "Run Status" _get_run_status_for_show(sim)
        "Initial Time" _get_initial_time_for_show(sim)
        "Steps" get_steps(sim)
    ]

    PrettyTables.pretty_table(
        io,
        table;
        backend = backend,
        show_column_labels = false,
        title = "Simulation",
        alignment = :l,
        kwargs...,
    )

    _show_method(io, sim.models, backend; kwargs...)
    _show_method(io, sim.sequence, backend; kwargs...)
end

function Base.show(io::IO, ::MIME"text/plain", input::SimulationResults)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::SimulationResults)
    # The tf_html_simple format was eliminated from PrettyTables and it was added to PowerSystems
    _show_method(io, input, :html; stand_alone = false, table_format = PSY.tf_html_simple)
end

function _show_method(io::IO, results::SimulationResults, backend::Symbol; kwargs...)
    header = ["Problem Name", "Initial Time", "Resolution", "Last Solution Timestamp"]

    table = Matrix{Any}(undef, length(results.decision_problem_results), length(header))
    for (ix, (key, result)) in enumerate(results.decision_problem_results)
        table[ix, 1] = key
        table[ix, 2] = first(result.timestamps)
        table[ix, 3] = Dates.canonicalize(result.resolution)
        table[ix, 4] = last(result.timestamps)
    end
    println(io)
    PrettyTables.pretty_table(
        io,
        table;
        column_labels = header,
        backend = backend,
        title = "Decision Problem Results",
        alignment = :l,
    )

    println(io)
    table = [
        "Name" results.emulation_problem_results.problem
        "Resolution" Dates.Minute(results.emulation_problem_results.resolution)
        "Number of steps" length(results.emulation_problem_results.timestamps)
    ]
    PrettyTables.pretty_table(
        io,
        table;
        show_column_labels = false,
        backend = backend,
        title = "Emulator Results",
        alignment = :l,
        kwargs...,
    )
end

ProblemResultsTypes = Union{SimulationProblemResults}
function Base.show(io::IO, ::MIME"text/plain", input::ProblemResultsTypes)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::ProblemResultsTypes)
    # The tf_html_simple format was eliminated from PrettyTables and it was added to PowerSystems
    _show_method(io, input, :html; stand_alone = false, table_format = PSY.tf_html_simple)
end

function _show_method(
    io::IO,
    results::T,
    backend::Symbol;
    kwargs...,
) where {T <: ProblemResultsTypes}
    timestamps = get_timestamps(results)

    # `get_resolution` returns `nothing` when there is a single timestamp (no
    # interval to diff), so guard against `Dates.Minute(nothing)`.
    resolution = ISOPT.get_resolution(results)
    resolution_str =
        isnothing(resolution) ? "N/A (single period)" :
        string(Dates.Minute(resolution))

    if backend == :html
        println(io, "<p> Start: $(first(timestamps))</p>")
        println(io, "<p> End: $(last(timestamps))</p>")
        println(io, "<p> Resolution: $(resolution_str)</p>")
    else
        println(io, "Start: $(first(timestamps))")
        println(io, "End: $(last(timestamps))")
        println(io, "Resolution: $(resolution_str)")
    end

    values = Dict{String, Vector{String}}(
        "Variables" => list_variable_names(results),
        "Auxiliary variables" => list_aux_variable_names(results),
        "Duals" => list_dual_names(results),
        "Expressions" => list_expression_names(results),
        "Parameters" => list_parameter_names(results),
    )

    if hasfield(T, :problem)
        name = results.problem
    else
        name = "PowerSimulations"
    end

    for (k, val) in values
        if !isempty(val)
            println(io)
            PrettyTables.pretty_table(
                io,
                val;
                show_column_labels = false,
                backend = backend,
                title = "$name Problem $k Results",
                alignment = :l,
                kwargs...,
            )
        end
    end
end
