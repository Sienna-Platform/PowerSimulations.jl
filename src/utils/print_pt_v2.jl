function Base.show(io::IO, container::OptimizationContainer)
    show(io, get_jump_model(container))
end

function Base.show(io::IO, ::MIME"text/plain", input::Union{ServiceModel, DeviceModel})
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::Union{ServiceModel, DeviceModel})
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
end

function _show_method(
    io::IO,
    model::Union{ServiceModel, DeviceModel},
    backend::Symbol;
    kwargs...,
)
    println(io)
    header = ["Device Type", "Formulation", "Slacks"]

    table = Matrix{String}(undef, 1, length(header))
    table[1, 1] = string(get_component_type(model))
    table[1, 2] = string(get_formulation(model))
    table[1, 3] = string(model.use_slacks)

    PrettyTables.pretty_table(
        io,
        table;
        header = header,
        backend = Val(backend),
        title = "Device Model",
        alignment = :l,
        kwargs...,
    )

    if !isempty(model.attributes)
        println(io)
        header = ["Name", "Value"]

        table = Matrix{String}(undef, length(model.attributes), length(header))
        for (ix, (k, v)) in enumerate(model.attributes)
            table[ix, 1] = string(k)
            table[ix, 2] = string(v)
        end

        PrettyTables.pretty_table(
            io,
            table;
            header = header,
            backend = Val(backend),
            title = "Attributes",
            alignment = :l,
            kwargs...,
        )
    end

    if !isempty(model.time_series_names)
        println(io)
        header = ["Parameter Name", "Time Series Name"]

        table = Matrix{String}(undef, length(model.time_series_names), length(header))
        for (ix, (k, v)) in enumerate(model.time_series_names)
            table[ix, 1] = string(k)
            table[ix, 2] = string(v)
        end

        PrettyTables.pretty_table(
            io,
            table;
            header = header,
            backend = Val(backend),
            title = "Time Series Names",
            alignment = :l,
            kwargs...,
        )
    end

    if !isempty(model.duals)
        println(io)

        table = string.(model.duals)

        PrettyTables.pretty_table(
            io,
            table;
            show_header = false,
            backend = Val(backend),
            title = "Duals",
            alignment = :l,
            kwargs...,
        )
    end

    if !isempty(model.feedforwards)
        println(io)
        header = ["Type", "Source", "Affected Values"]
        table = Matrix{String}(undef, length(model.feedforwards), length(header))
        for (ix, v) in enumerate(model.feedforwards)
            table[ix, 1] = string(typeof(v))
            table[ix, 2] = encode_key_as_string(get_optimization_container_key(v))
            table[ix, 3] = first(encode_key_as_string.(get_affected_values(v)))
        end
        PrettyTables.pretty_table(
            io,
            table;
            header = header,
            backend = Val(backend),
            title = "Feedforwards",
            alignment = :l,
            kwargs...,
        )
    else
        println(io)
        print(io, "No FeedForwards Assigned")
    end
end

function Base.show(io::IO, ::MIME"text/plain", input::NetworkModel)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::NetworkModel)
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
end

function _show_method(io::IO, network_model::NetworkModel, backend::Symbol; kwargs...)
    table = [
        "Network Model" string(get_network_formulation(network_model))
        "Slacks" get_use_slacks(network_model)
        "PTDF" !isnothing(get_PTDF_matrix(network_model))
        "Duals" join(string.(get_duals(network_model)), " ")
    ]

    PrettyTables.pretty_table(
        io,
        table;
        backend = Val(backend),
        header = ["Field", "Value"],
        title = "Network Model",
        alignment = :l,
        kwargs...,
    )
    return
end

function Base.show(io::IO, ::MIME"text/plain", input::OperationModel)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::OperationModel)
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
end

function _show_method(io::IO, model::OperationModel, backend::Symbol; kwargs...)
    _show_method(io, model.template, backend; kwargs...)
end

function Base.show(io::IO, ::MIME"text/plain", input::ProblemTemplate)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::ProblemTemplate)
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
end

function _show_method(io::IO, template::ProblemTemplate, backend::Symbol; kwargs...)
    table = [
        "Network Model" string(get_network_formulation(template.network_model))
        "Slacks" get_use_slacks(template.network_model)
        "PTDF" !isnothing(get_PTDF_matrix(template.network_model))
        "Duals" isempty(get_duals(template.network_model)) ? "None" : string.(get_duals(template.network_model))
        "HVDC Network Model" isnothing(get_hvdc_network_model(template.network_model)) ? "None" : replace(string(get_hvdc_network_model(template.network_model)), r"[()]" => "")
    ]

    PrettyTables.pretty_table(
        io,
        table;
        backend = Val(backend),
        show_header = false,
        title = "Network Model",
        alignment = :l,
        kwargs...,
    )

    println(io)
    header = ["Device Type", "Formulation", "Slacks"]

    table = Matrix{String}(undef, length(template.devices), length(header))
    for (ix, model) in enumerate(values(template.devices))
        table[ix, 1] = string(get_component_type(model))
        table[ix, 2] = string(get_formulation(model))
        table[ix, 3] = string(model.use_slacks)
    end

    PrettyTables.pretty_table(
        io,
        table;
        backend = Val(backend),
        header = header,
        title = "Device Models",
        alignment = :l,
    )

    if !isempty(template.branches)
        println(io)
        header = ["Branch Type", "Formulation", "Slacks"]

        table = Matrix{String}(undef, length(template.branches), length(header))
        for (ix, model) in enumerate(values(template.branches))
            table[ix, 1] = string(get_component_type(model))
            table[ix, 2] = string(get_formulation(model))
            table[ix, 3] = string(model.use_slacks)
        end

        PrettyTables.pretty_table(
            io,
            table;
            header = header,
            backend = Val(backend),
            title = "Branch Models",
            alignment = :l,
            kwargs...,
        )
    end

    if !isempty(template.services)
        println(io)
        if isempty(first(keys(template.services))[1])
            header = ["Service Type", "Formulation", "Slacks", "Aggregated Model"]
        else
            header = ["Name", "Service Type", "Formulation", "Slacks", "Aggregated Model"]
        end

        table = Matrix{String}(undef, length(template.services), length(header))
        for (ix, (key, model)) in enumerate(template.services)
            if isempty(key[1])
                table[ix, 1] = string(get_component_type(model))
                table[ix, 2] = string(get_formulation(model))
                table[ix, 3] = string(model.use_slacks)
                table[ix, 4] =
                    string(get(model.attributes, "aggregated_service_model", "false"))
            else
                table[ix, 1] = key[1]
                table[ix, 2] = string(get_component_type(model))
                table[ix, 3] = string(get_formulation(model))
                table[ix, 4] = string(model.use_slacks)
                table[ix, 5] =
                    string(get(model.attributes, "aggregated_service_model", "false"))
            end
        end

        PrettyTables.pretty_table(
            io,
            table;
            header = header,
            backend = Val(backend),
            title = "Service Models",
            alignment = :l,
            kwargs...,
        )
    end
    return
end

function Base.show(io::IO, ::MIME"text/plain", input::SimulationModels)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::SimulationModels)
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
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
        header = header,
        backend = Val(backend),
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
            header = header,
            backend = Val(backend),
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
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
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
        backend = Val(backend),
        show_header = false,
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
        header = header,
        backend = Val(backend),
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
            header = header,
            backend = Val(backend),
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
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
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
        backend = Val(backend),
        show_header = false,
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
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
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
        header = header,
        backend = Val(backend),
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
        show_header = false,
        backend = Val(backend),
        title = "Emulator Results",
        alignment = :l,
        kwargs...,
    )
end

ProblemResultsTypes = Union{OptimizationProblemResults, SimulationProblemResults}
function Base.show(io::IO, ::MIME"text/plain", input::ProblemResultsTypes)
    _show_method(io, input, :auto)
end

function Base.show(io::IO, ::MIME"text/html", input::ProblemResultsTypes)
    _show_method(io, input, :html; standalone = false, tf = PrettyTables.tf_html_simple)
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
                show_header = false,
                backend = Val(backend),
                title = "$name Problem $k Results",
                alignment = :l,
                kwargs...,
            )
        end
    end
end

function Base.show(io::IO, ::MIME"text/plain", bounds::ConstraintBounds)
    println(io, "ConstraintBounds:")
    println(io, "Constraint Coefficient")
    show(io, MIME"text/plain"(), bounds.coefficient)
    println(io, "Constraint RHS")
    show(io, MIME"text/plain"(), bounds.rhs)
end

function Base.show(io::IO, ::MIME"text/plain", bounds::VariableBounds)
    println(io, "VariableBounds:")
    show(io, MIME"text/plain"(), bounds.bounds)
end

function Base.show(io::IO, ::MIME"text/plain", bounds::NumericalBounds)
    println(io, rpad("  Minimum", 20), "Maximum")
    println(io, rpad("  $(bounds.min)", 20), "$(bounds.max)")
end
