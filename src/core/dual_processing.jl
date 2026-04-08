struct VarRestoreInfo
    lb::Union{Nothing, AbstractArray}
    ub::Union{Nothing, AbstractArray}
    fixed_int_value::Union{Nothing, AbstractArray}
    is_integer::Bool
end

function process_duals(container::OptimizationContainer, lp_optimizer)
    var_container = get_variables(container)
    for (k, v) in var_container
        if isa(v, JuMP.Containers.SparseAxisArray)
            container.primal_values_cache.variables_cache[k] = jump_value.(v)
        else
            container.primal_values_cache.variables_cache[k] = jump_value.(v)
        end
    end

    for (k, v) in get_expressions(container)
        container.primal_values_cache.expressions_cache[k] = jump_value.(v)
    end
    var_cache = container.primal_values_cache.variables_cache
    cache = sizehint!(Dict{VariableKey, VarRestoreInfo}(), length(var_container))
    for (key, variable) in get_variables(container)
        if isa(variable, JuMP.Containers.SparseAxisArray)
            continue
        end
        is_integer_flag = false
        is_binary_flag = false
        if JuMP.is_binary(first(variable))
            JuMP.unset_binary.(variable)
            is_binary_flag = true
        elseif JuMP.is_integer(first(variable))
            JuMP.unset_integer.(variable)
            is_integer_flag = true
        else
            continue
        end
        lb = if JuMP.has_lower_bound(first(variable))
            JuMP.lower_bound.(variable)
        else
            nothing
        end
        ub = if JuMP.has_upper_bound(first(variable))
            JuMP.upper_bound.(variable)
        else
            nothing
        end
        fixed_int_value = if JuMP.is_fixed(first(variable)) && is_integer_flag
            jump_value.(variable)
        else
            nothing
        end
        cache[key] = VarRestoreInfo(lb, ub, fixed_int_value, is_integer_flag)
        # Round cached values in-place to nearest integer to avoid infeasibilities
        # from MIP solver numerical tolerances (e.g. 0.9999997 instead of 1.0)
        var_cache[key] .= round.(var_cache[key])
        JuMP.fix.(variable, var_cache[key]; force = true)
    end
    @assert !isempty(cache)
    jump_model = get_jump_model(container)

    if JuMP.mode(jump_model) != JuMP.DIRECT
        JuMP.set_optimizer(jump_model, lp_optimizer)
    else
        @debug("JuMP model set in direct mode during dual calculation")
    end

    JuMP.optimize!(jump_model)

    model_status = JuMP.primal_status(jump_model)
    if model_status ∉ (
        MOI.FEASIBLE_POINT::MOI.ResultStatusCode,
        MOI.NEARLY_FEASIBLE_POINT::MOI.ResultStatusCode,
    )
        @error "Optimizer returned $model_status during dual calculation"
        return RunStatus.FAILED
    end

    if JuMP.has_duals(jump_model)
        for (key, dual) in get_duals(container)
            constraint = get_constraint(container, key)
            dual.data .= jump_value.(constraint.data)
        end
    end

    for (key, variable) in get_variables(container)
        if !haskey(cache, key)
            continue
        end
        if isa(variable, JuMP.Containers.SparseAxisArray)
            continue
        end
        info = cache[key]
        JuMP.unfix.(variable)
        if info.lb !== nothing
            JuMP.set_lower_bound.(variable, info.lb)
        end
        if info.ub !== nothing
            JuMP.set_upper_bound.(variable, info.ub)
        end
        if info.is_integer
            JuMP.set_integer.(variable)
        else
            JuMP.set_binary.(variable)
        end
        if info.fixed_int_value !== nothing
            JuMP.fix.(variable, info.fixed_int_value)
        end
    end
    return RunStatus.SUCCESSFULLY_FINALIZED
end
