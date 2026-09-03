
const _SUPPORTED_FORMATS = ("csv",)

mutable struct SimulationResultsExport
    models::Dict{Symbol, IOM.OptimizationProblemOutputsExport}
    start_time::Dates.DateTime
    end_time::Dates.DateTime
    path::Union{Nothing, String}
    format::String
end

function SimulationResultsExport(
    models::Vector{IOM.OptimizationProblemOutputsExport},
    params::SimulationStoreParams;
    start_time = nothing,
    end_time = nothing,
    path = nothing,
    format = "csv",
)
    # This end time is outside the bounds of the simulation.
    sim_end_time = params.initial_time + params.step_resolution * params.num_steps

    if isnothing(start_time)
        start_time = params.initial_time
    elseif start_time < params.initial_time || start_time >= sim_end_time
        throw(IS.InvalidValue("invalid start_time: $start_time"))
    end

    if isnothing(end_time)
        # Reduce the end_time to be within the simulation.
        end_time = sim_end_time - Dates.Second(1)
    elseif end_time < params.initial_time || end_time >= sim_end_time
        throw(IS.InvalidValue("invalid end_time: $end_time"))
    end

    if !(format in list_supported_formats(SimulationResultsExport))
        throw(IS.InvalidValue("format = $format is not supported"))
    end

    return SimulationResultsExport(
        Dict(x.name => x for x in models),
        start_time,
        end_time,
        path,
        format,
    )
end

function SimulationResultsExport(filename::AbstractString, params::SimulationStoreParams)
    if splitext(filename)[2] != ".json"
        throw(IS.InvalidValue("only JSON files are supported: $filename"))
    end

    return SimulationResultsExport(read_json(filename), params)
end

function SimulationResultsExport(data::AbstractDict, params::SimulationStoreParams)
    models = Vector{IOM.OptimizationProblemOutputsExport}()
    for model in get(data, "models", [])
        if !haskey(model, "name")
            throw(IS.InvalidValue("model data does not define 'name'"))
        end

        problem_params = params.decision_models_params[Symbol(model["name"])]
        duals = Set(
            deserialize_key(problem_params, x) for
            x in get(model, "duals", Set{ConstraintKey}())
        )
        parameters = Set(
            deserialize_key(problem_params, x) for
            x in get(model, "parameters", Set{ParameterKey}())
        )
        variables = Set(
            deserialize_key(problem_params, x) for
            x in get(model, "variables", Set{VariableKey}())
        )
        aux_variables = Set(
            deserialize_key(problem_params, x) for
            x in get(model, "variables", Set{AuxVarKey}())
        )
        problem_export = IOM.OptimizationProblemOutputsExport(
            model["name"];
            duals = duals,
            parameters = parameters,
            variables = variables,
            optimizer_stats = get(model, "optimizer_stats", false),
            store_all_duals = get(model, "store_all_duals", false),
            store_all_parameters = get(model, "store_all_parameters", false),
            store_all_variables = get(model, "store_all_variables", false),
            store_all_aux_variables = get(model, "store_all_aux_variables", false),
        )
        push!(models, problem_export)
    end

    start_time = get(data, "start_time", nothing)
    if start_time isa AbstractString
        start_time = Dates.DateTime(start_time)
    end

    end_time = get(data, "end_time", nothing)
    if end_time isa AbstractString
        end_time = Dates.DateTime(end_time)
    end

    return SimulationResultsExport(
        models,
        params;
        start_time = start_time,
        end_time = end_time,
        path = get(data, "path", nothing),
        format = get(data, "format", "csv"),
    )
end

function get_problem_exports(x::SimulationResultsExport, model_name)
    name = Symbol(model_name)
    if !haskey(x.models, name)
        throw(IS.InvalidValue("model $name is not stored. keys = $(keys(x.models))"))
    end

    return x.models[name]
end

function get_export_file_type(exports::SimulationResultsExport)
    if exports.format == "csv"
        return CSV.File
    end

    throw(IS.InvalidValue("format not supported: $(exports.format)"))
end

list_supported_formats(::Type{SimulationResultsExport}) = ("csv",)

function should_export(exports::SimulationResultsExport, tstamp::Dates.DateTime)
    return tstamp >= exports.start_time && tstamp <= exports.end_time
end

function IOM.should_export_dual(exports::SimulationResultsExport, tstamp, model, name)
    return _should_export(exports, tstamp, model, STORE_CONTAINER_DUALS, name)
end

function IOM.should_export_parameter(exports::SimulationResultsExport, tstamp, model, name)
    return _should_export(exports, tstamp, model, STORE_CONTAINER_PARAMETERS, name)
end

function IOM.should_export_variable(exports::SimulationResultsExport, tstamp, model, name)
    return _should_export(exports, tstamp, model, STORE_CONTAINER_VARIABLES, name)
end

function IOM.should_export_expression(exports::SimulationResultsExport, tstamp, model, name)
    return _should_export(exports, tstamp, model, STORE_CONTAINER_EXPRESSIONS, name)
end

function IOM.should_export_aux_variable(
    exports::SimulationResultsExport,
    tstamp,
    model,
    name,
)
    return _should_export(exports, tstamp, model, STORE_CONTAINER_AUX_VARIABLES, name)
end

function IOM._should_export(
    exports::SimulationResultsExport,
    tstamp,
    model,
    field_name,
    name,
)
    if tstamp < exports.start_time || tstamp >= exports.end_time
        return false
    end

    problem_exports = get_problem_exports(exports, model)
    return _should_export(problem_exports, field_name, name)
end
