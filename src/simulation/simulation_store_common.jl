function IOM.write_outputs!(
    store::SimulationStore,
    model::IOM.AbstractOptimizationModel,
    index::Union{DecisionModelIndexType, EmulationModelIndexType},
    update_timestamp::Dates.DateTime;
    exports = nothing,
)
    if !isnothing(exports)
        export_params = Dict{Symbol, Any}(
            :exports => exports,
            :exports_path => joinpath(exports.path, string(get_name(model))),
            :file_type => get_export_file_type(exports),
            :resolution => get_resolution(model),
            :horizon_count => get_horizon(get_settings(model)) ÷ get_resolution(model),
        )
    else
        export_params = nothing
    end

    for field in (:duals, :parameters, :variables, :aux_variables, :expressions)
        _write_model_field_outputs!(
            store,
            model,
            index,
            update_timestamp,
            export_params,
            Val(field),
        )
    end
    buffer_parameter_inputs!(store, model, index, update_timestamp)
    return
end

_output_source(container::OptimizationContainer, ::Val{:duals}) = get_duals(container)
_output_source(container::OptimizationContainer, ::Val{:parameters}) =
    get_parameters(container)
_output_source(container::OptimizationContainer, ::Val{:aux_variables}) =
    get_aux_variables(container)
function _output_source(container::OptimizationContainer, ::Val{:variables})
    if !isempty(container.primal_values_cache)
        return container.primal_values_cache.variables_cache
    end
    return get_variables(container)
end
function _output_source(container::OptimizationContainer, ::Val{:expressions})
    if !isempty(container.primal_values_cache)
        return container.primal_values_cache.expressions_cache
    end
    return get_expressions(container)
end

_output_values(x, ::Val) = jump_value.(x)
_output_values(x, ::Val{:parameters}) = calculate_parameter_values(x)

_should_export_field(exports, ts, model_name, key, ::Val{:duals}) =
    should_export_dual(exports, ts, model_name, key)
_should_export_field(exports, ts, model_name, key, ::Val{:parameters}) =
    should_export_parameter(exports, ts, model_name, key)
_should_export_field(exports, ts, model_name, key, ::Val{:variables}) =
    should_export_variable(exports, ts, model_name, key)
_should_export_field(exports, ts, model_name, key, ::Val{:aux_variables}) =
    should_export_aux_variable(exports, ts, model_name, key)
_should_export_field(exports, ts, model_name, key, ::Val{:expressions}) =
    should_export_expression(exports, ts, model_name, key)

function _write_model_field_outputs!(
    store,
    model::IOM.AbstractOptimizationModel,
    index::Union{DecisionModelIndexType, EmulationModelIndexType},
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol, Any}, Nothing},
    field::Val{F},
) where {F}
    container = get_optimization_container(model)
    model_name = get_name(model)
    if !isnothing(export_params)
        exports_path = joinpath(export_params[:exports_path], string(F))
        mkpath(exports_path)
    end

    for (key, value) in _output_source(container, field)
        !should_write_resulting_value(key) && continue
        data = _output_values(value, field)
        write_output!(store, model_name, key, index, update_timestamp, data)

        if !isnothing(export_params) &&
           _should_export_field(
            export_params[:exports],
            update_timestamp,
            model_name,
            key,
            field,
        )
            df = to_dataframe(data, key)
            time_col = range(
                index;
                length = export_params[:horizon_count],
                step = export_params[:resolution],
            )
            DataFrames.insertcols!(df, 1, :DateTime => time_col)
            export_output(export_params[:file_type], exports_path, key, index, df)
        end
    end
    return
end

function _open_outputs_store(f, execution_path::AbstractString)
    return open_store(f, HdfSimulationStore, joinpath(execution_path, STORE_DIR), "r")
end
