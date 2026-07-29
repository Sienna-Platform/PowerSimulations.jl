# ----------------------------------------------------------------------------
# Security-constrained reserve service formulations (G-1 with reserve
# deployment + monitored-branch post-contingency flow constraints).
#
# Sparse + monitored counterpart to a legacy dense-per-branch implementation.
# Mirrors the device-side ac_transmission_security_constrained_models.jl: post-contingency
# flow expressions/constraints (and optional slacks) live in
# `SparseAxisArray`s keyed by `(outage_id::String, monitored_name::String,
# t::Int)`, scoped to the monitored components carried by each outage in
# `service_model.outages[uuid]::Dict{DataType, Set{String}}` (populated by
# `_build_service_model_outages!`).
#
# Per-outage reserve-deployment variables and the per-outage power-balance /
# nodal-deployment / area-deployment expressions remain dense over the
# contributing devices and the modeled bus/area axes — those are independent
# of which branches are monitored.
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# Helpers: monitored-arc resolution + sparse container scaffolding
# ----------------------------------------------------------------------------

"""
Resolve every monitored component in `service_model.outages` to a container
name and arc tuple in the active network reduction. Mirrors the device-side
`_resolve_monitored_arcs` but operates on a `ServiceModel` via the shared
`_resolve_arc_resolution_core`; called only under a
`NetworkModel{<:AbstractPTDFModel}`. Monitored types for which
`_requires_arc_resolution` is `false` (currently only `PSY.AreaInterchange`)
are dropped — under a PTDF-based network model there is no other mechanism
that resolves an AreaInterchange monitor to a post-contingency flow
expression (unlike the AreaBalance path), so a warning naming the dropped
components is emitted once. Any other monitored type absent from the
reduction's arc maps raises `error` instead of being silently dropped.
Outages left with no resolved monitored component (e.g. all dropped) are
excluded from the result.

Returns
`Vector{Pair{UUID, Vector{Tuple{DataType, String, Tuple{Int,Int}, String}}}}`
where each inner tuple is `(monitored_type, container_name, arc,
reduction_kind)`. Outages are sorted by UUID for deterministic axes.
"""
function _resolve_service_monitored_arcs(
    service_model::ServiceModel,
    net_reduction_data::PNM.NetworkReductionData,
)
    dropped_area_interchanges = Set{String}()
    resolved = _resolve_arc_resolution_core(
        get_outages(service_model),
        net_reduction_data,
        names -> union!(dropped_area_interchanges, names),
    )
    filter!(entry -> !isempty(entry.second), resolved)
    if !isempty(dropped_area_interchanges)
        @warn "AreaInterchange monitor(s) $(sort!(collect(dropped_area_interchanges))) \
               dropped: no post-contingency flow expression is built for them under a \
               PTDF-based network model. Use AreaBalancePowerModel to monitor \
               AreaInterchange ties." _group = LOG_GROUP_MODELS_VALIDATION
    end
    return resolved
end

const _AttributeDeviceMap = Vector{
    NamedTuple{(:component, :supplemental_attribute), Tuple{PSY.Generator, PSY.Outage}},
}

"""
Sorted `Vector{String}` of the UUIDs (as strings) of the outages claimed by
`service_model.outages`. This is the service-side counterpart to iterating
`get_outages(device_model)` on the AC-branch side.

No consumer in this file needs the resolved `PSY.Outage` object itself: every
use reduces to `string(IS.get_uuid(outage))` or a membership test against
`get_outages(service_model)`. Callers use ids directly instead of
round-tripping each UUID through a system lookup.
"""
function _service_outage_ids(service_model::ServiceModel)
    return string.(sort!(collect(keys(get_outages(service_model)))))
end

"""
Map each outage id in `outage_ids` to the set of `PSY.Generator`s it outages,
derived from `attribute_device_map` — a single
`get_component_supplemental_attribute_pairs` scan of the system, computed
once by the caller (the arguments-stage entry or
`_construct_service_model_prologue!`) and reused across every consumer that
needs the outage/generator mapping.

Keyed on the device objects (not names): the caller's contributing devices
are heterogeneous (`V <: PSY.Generator`), and PSY only guarantees name
uniqueness per concrete type, so a name-based set could conflate two
same-named generators of different concrete types.
"""
function _outaged_generators_by_outage_id(
    outage_ids::Vector{String},
    attribute_device_map::_AttributeDeviceMap,
)
    outaged_gens = Dict{String, Set{PSY.Generator}}(
        outage_id => Set{PSY.Generator}() for outage_id in outage_ids
    )
    for (component, outage) in attribute_device_map
        outage_id = string(IS.get_uuid(outage))
        haskey(outaged_gens, outage_id) || continue
        push!(outaged_gens[outage_id], component)
    end
    return outaged_gens
end

"""
Register an empty `SparseAxisArray` keyed by
`(outage_id::String, monitored_name::String, t::Int)` for the given
post-contingency expression type / service. The caller
(`add_post_contingency_flow_expressions!`) always overwrites every key it
resolved before returning, so this starts empty rather than zero-filled.
"""
function _add_service_post_contingency_sparse_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{R},
    service_name::String,
) where {T <: PostContingencyExpressions, R <: PSY.AbstractReserve}
    expr_container = SparseAxisArray(Dict{Tuple{String, String, Int}, JuMP.AffExpr}())
    _assign_container!(
        container.expressions,
        ExpressionKey(T, R, service_name),
        expr_container,
    )
    return expr_container
end

"""
Sparse slack variable container keyed by
`(outage_id::String, monitored_name::String, t::Int)`. Each entry is a
non-negative `JuMP.VariableRef` whose objective contribution is
`CONSTRAINT_VIOLATION_SLACK_COST`. Built directly via
`@variable`/`_assign_container!` so the axes can be sparse.
"""
function add_post_contingency_slack_variables!(
    container::OptimizationContainer,
    ::Type{T},
    service::R,
    service_name::String,
    resolved::Vector{Pair{Base.UUID, Vector{String}}},
    ::AbstractSecurityConstrainedReservesFormulation,
) where {T <: AbstractContingencySlackVariableType, R <: PSY.AbstractReserve}
    time_steps = get_time_steps(container)
    jump_model = get_jump_model(container)
    base_prefix = "$(T)_$(R)_$(service_name)_"
    contents = Dict{Tuple{String, String, Int}, JuMP.VariableRef}()
    for (uuid, names) in resolved
        outage_id = string(uuid)
        for name in names
            for t in time_steps
                v = JuMP.@variable(
                    jump_model,
                    base_name = "$(base_prefix){$(outage_id), $(name), $(t)}",
                    lower_bound = 0.0,
                    start = 0.0,
                )
                contents[(outage_id, name, t)] = v
                add_to_objective_invariant_expression!(
                    container,
                    v * CONSTRAINT_VIOLATION_SLACK_COST,
                )
            end
        end
    end
    slack_container = SparseAxisArray(contents)
    _assign_container!(
        container.variables,
        VariableKey(T, R, service_name),
        slack_container,
    )
    return slack_container
end

"""
Build the optional post-contingency flow-rate slack variable pair
`(slack_ub, slack_lb)` when `use_slacks` is set, or `(nothing, nothing)`
otherwise. Shared by the PTDF (`PostContingencyBranchFlow`) and AreaBalance
(`PostContingencyAreaInterchangeFlow`) `PostContingencyFlowRateConstraint`
builders, which differ only in how they extract the monitored name list
(`slack_resolved`) from `resolved` before calling this.
"""
function _add_flow_rate_slack_pair!(
    container::OptimizationContainer,
    service::R,
    service_name::String,
    slack_resolved::Vector{Pair{Base.UUID, Vector{String}}},
    formulation::AbstractSecurityConstrainedReservesFormulation,
    use_slacks::Bool,
) where {R <: PSY.AbstractReserve}
    use_slacks || return nothing, nothing
    slack_ub = add_post_contingency_slack_variables!(
        container,
        PostContingencyFlowActivePowerSlackUpperBound,
        service,
        service_name,
        slack_resolved,
        formulation,
    )
    slack_lb = add_post_contingency_slack_variables!(
        container,
        PostContingencyFlowActivePowerSlackLowerBound,
        service,
        service_name,
        slack_resolved,
        formulation,
    )
    return slack_ub, slack_lb
end

# ----------------------------------------------------------------------------
# Reserve deployment variable per (outage, contributing device, t)
# ----------------------------------------------------------------------------

function add_variables!(
    container::OptimizationContainer,
    variable_type::Type{T},
    service::R,
    service_model::ServiceModel{R, <:AbstractSecurityConstrainedReservesFormulation},
    contributing_devices::Vector{V},
    formulation::AbstractSecurityConstrainedReservesFormulation,
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {
    T <: AbstractContingencyVariableType,
    R <: PSY.AbstractReserve,
    V <: PSY.StaticInjection,
}
    @assert !isempty(contributing_devices)
    time_steps = get_time_steps(container)
    binary = get_variable_binary(variable_type(), R, formulation)
    service_name = PSY.get_name(service)
    device_names = PSY.get_name.(contributing_devices)

    variable = lazy_container_addition!(
        container,
        variable_type(),
        R,
        outage_ids,
        device_names,
        time_steps;
        meta = service_name,
    )

    base_prefix = "$(T)_$(R)_$(service_name)_"
    for outage_id in outage_ids
        outage_pos = variable.lookup[1][outage_id]
        for (i, device) in enumerate(contributing_devices)
            name = device_names[i]
            device_pos = variable.lookup[2][name]
            device_outaged = device in outaged_gens[outage_id]
            for t in time_steps
                v = JuMP.@variable(
                    get_jump_model(container),
                    base_name = "$(base_prefix){$(outage_id), $(name), $(t)}",
                    binary = binary,
                )
                # Positional `t` indexing assumes `time_steps` is `1:horizon`
                # (container.time_steps is never reassigned via
                # set_time_steps!); it doubles as the array's time axis index.
                variable.data[outage_pos, device_pos, t] = v
                if device_outaged
                    # The outaged generator cannot deploy reserves for its own
                    # contingency; force the variable to zero.
                    JuMP.set_upper_bound(v, 0.0)
                    JuMP.set_lower_bound(v, 0.0)
                    JuMP.set_start_value(v, 0.0)
                    continue
                end
                ub = get_variable_upper_bound(variable_type(), service, device, formulation)
                isnothing(ub) || JuMP.set_upper_bound(v, ub)
                lb = get_variable_lower_bound(variable_type(), service, device, formulation)
                (isnothing(lb) || binary) || JuMP.set_lower_bound(v, lb)
                init = get_variable_warm_start_value(variable_type(), device, formulation)
                isnothing(init) || JuMP.set_start_value(v, init)
            end
        end
    end
    return
end

# ----------------------------------------------------------------------------
# Post-contingency power-balance, nodal-deployment, area-deployment
# expressions. Reserve-deployment contributions and generator-outage
# contributions are added in separate dispatches to avoid `isa` checks.
# ----------------------------------------------------------------------------

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractPowerModel},
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {
    T <: PostContingencyActivePowerBalance,
    U <: AbstractContingencyVariableType,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    expression = lazy_container_addition!(
        container,
        T(),
        R,
        outage_ids,
        time_steps;
        meta = service_name,
    )
    reserve_deployment_variable = get_variable(container, U(), R, service_name)
    mult_default = get_variable_multiplier(U(), R, F())
    device_names = PSY.get_name.(contributing_devices)
    for outage_id in outage_ids
        expr_outage_pos = expression.lookup[1][outage_id]
        var_outage_pos = reserve_deployment_variable.lookup[1][outage_id]
        for (i, device) in enumerate(contributing_devices)
            device in outaged_gens[outage_id] && continue
            var_device_pos = reserve_deployment_variable.lookup[2][device_names[i]]
            for t in time_steps
                _add_to_jump_expression!(
                    expression.data[expr_outage_pos, t],
                    reserve_deployment_variable.data[var_outage_pos, var_device_pos, t],
                    mult_default,
                )
            end
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    attribute_device_map::_AttributeDeviceMap,
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PostContingencyActivePowerBalance,
    U <: VariableType,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    outages = get_outages(service_model)
    expression = get_expression(container, T(), R, service_name)
    for (d, outage) in attribute_device_map
        haskey(outages, IS.get_uuid(outage)) || continue
        outage_id = string(IS.get_uuid(outage))
        name = PSY.get_name(d)
        variable = get_variable(container, U(), typeof(d))
        mult = get_variable_multiplier(U(), typeof(d), F())
        for t in time_steps
            _add_to_jump_expression!(
                expression[outage_id, t],
                variable[name, t],
                mult,
            )
        end
    end
    return
end

"""
Sorted, deduplicated bus numbers hosting either a contributing device or a
generator outaged by one of the service's associated outages, mapped through
`network_reduction`. `PostContingencyNodalActivePowerDeployment` is scoped to
this axis rather than the full PTDF bus axis: every other bus's deployment
expression is always the zero `AffExpr` (no contributing device and no
outaged generator sits there), so omitting them from the container and from
the post-contingency flow sum is algebraically neutral.
"""
function _injection_relevant_buses(
    network_reduction::PNM.NetworkReductionData,
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {V <: PSY.Generator}
    buses = Set{Int}()
    for device in contributing_devices
        push!(buses, PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(device)))
    end
    for gens in values(outaged_gens)
        for gen in gens
            push!(buses, PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(gen)))
        end
    end
    return sort!(collect(buses))
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    relevant_buses::Vector{Int},
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{N},
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {
    T <: PostContingencyNodalActivePowerDeployment,
    U <: AbstractContingencyVariableType,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
    N <: AbstractPTDFModel,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    expression = lazy_container_addition!(
        container,
        T(),
        R,
        outage_ids,
        relevant_buses,
        time_steps;
        meta = service_name,
    )
    reserve_deployment_variable = get_variable(container, U(), R, service_name)
    mult_default = get_variable_multiplier(U(), R, F())
    network_reduction = get_network_reduction(network_model)
    device_names = PSY.get_name.(contributing_devices)
    device_buses = [
        PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(d))
        for d in contributing_devices
    ]
    for outage_id in outage_ids
        expr_outage_pos = expression.lookup[1][outage_id]
        var_outage_pos = reserve_deployment_variable.lookup[1][outage_id]
        for (i, device) in enumerate(contributing_devices)
            device in outaged_gens[outage_id] && continue
            expr_bus_pos = expression.lookup[2][device_buses[i]]
            var_device_pos = reserve_deployment_variable.lookup[2][device_names[i]]
            for t in time_steps
                _add_to_jump_expression!(
                    expression.data[expr_outage_pos, expr_bus_pos, t],
                    reserve_deployment_variable.data[var_outage_pos, var_device_pos, t],
                    mult_default,
                )
            end
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    attribute_device_map::_AttributeDeviceMap,
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{N},
) where {
    T <: PostContingencyNodalActivePowerDeployment,
    U <: VariableType,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
    N <: AbstractPTDFModel,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    outages = get_outages(service_model)
    expression = get_expression(container, T(), R, service_name)
    network_reduction = get_network_reduction(network_model)
    for (device, outage) in attribute_device_map
        haskey(outages, IS.get_uuid(outage)) || continue
        outage_id = string(IS.get_uuid(outage))
        name = PSY.get_name(device)
        variable = get_variable(container, U(), typeof(device))
        mult = get_variable_multiplier(U(), typeof(device), F())
        bus_number = PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(device))
        for t in time_steps
            _add_to_jump_expression!(
                expression[outage_id, bus_number, t],
                variable[name, t],
                mult,
            )
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{<:AreaBalancePowerModel},
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {
    T <: PostContingencyAreaActivePowerDeployment,
    U <: AbstractContingencyVariableType,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    area_names = PSY.get_name.(get_available_components(network_model, PSY.Area, sys))
    expression = lazy_container_addition!(
        container,
        T(),
        R,
        outage_ids,
        area_names,
        time_steps;
        meta = service_name,
    )
    reserve_deployment_variable = get_variable(container, U(), R, service_name)
    mult_default = get_variable_multiplier(U(), R, F())
    device_names = PSY.get_name.(contributing_devices)
    device_areas = [
        PSY.get_name(PSY.get_area(PSY.get_bus(d))) for d in contributing_devices
    ]
    for outage_id in outage_ids
        expr_outage_pos = expression.lookup[1][outage_id]
        var_outage_pos = reserve_deployment_variable.lookup[1][outage_id]
        for (i, device) in enumerate(contributing_devices)
            device in outaged_gens[outage_id] && continue
            expr_area_pos = expression.lookup[2][device_areas[i]]
            var_device_pos = reserve_deployment_variable.lookup[2][device_names[i]]
            for t in time_steps
                _add_to_jump_expression!(
                    expression.data[expr_outage_pos, expr_area_pos, t],
                    reserve_deployment_variable.data[var_outage_pos, var_device_pos, t],
                    mult_default,
                )
            end
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    attribute_device_map::_AttributeDeviceMap,
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:AreaBalancePowerModel},
) where {
    T <: PostContingencyAreaActivePowerDeployment,
    U <: VariableType,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    outages = get_outages(service_model)
    expression = get_expression(container, T(), R, service_name)
    for (device, outage) in attribute_device_map
        haskey(outages, IS.get_uuid(outage)) || continue
        outage_id = string(IS.get_uuid(outage))
        name = PSY.get_name(device)
        variable = get_variable(container, U(), typeof(device))
        mult = get_variable_multiplier(U(), typeof(device), F())
        area_name = PSY.get_name(PSY.get_area(PSY.get_bus(device)))
        for t in time_steps
            _add_to_jump_expression!(
                expression[outage_id, area_name, t],
                variable[name, t],
                mult,
            )
        end
    end
    return
end

# Per-(outage, generator, t) post-contingency active power expression.
# Used when no reserve requirement time series is configured (the older
# `has_requirement_ts` branch). The expression is the pre-contingency
# generator dispatch plus the reserve-deployment variable, with the
# outaged generator contributing zero. `PostContingencyActivePowerGeneration`
# is dense over the contributing devices so per-generator min/max bounds
# can be applied directly.
function add_to_expression!(
    container::OptimizationContainer,
    ::Type{T},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractActivePowerModel},
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {
    T <: PostContingencyActivePowerGeneration,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    expression = add_expression_container!(
        container,
        T(),
        R,
        outage_ids,
        PSY.get_name.(contributing_devices),
        time_steps;
        meta = service_name,
    )
    reserve_deployment_variable = get_variable(
        container,
        PostContingencyActivePowerReserveDeploymentVariable(),
        R,
        service_name,
    )
    for device in contributing_devices
        gen_var = get_variable(container, ActivePowerVariable(), typeof(device))
        gen_name = PSY.get_name(device)
        expr_gen_pos = expression.lookup[2][gen_name]
        var_gen_pos = reserve_deployment_variable.lookup[2][gen_name]
        gen_var_pos = gen_var.lookup[1][gen_name]
        for outage_id in outage_ids
            gen_outaged = device in outaged_gens[outage_id]
            expr_outage_pos = expression.lookup[1][outage_id]
            var_outage_pos = reserve_deployment_variable.lookup[1][outage_id]
            for t in time_steps
                _add_to_jump_expression!(
                    expression.data[expr_outage_pos, expr_gen_pos, t],
                    reserve_deployment_variable.data[var_outage_pos, var_gen_pos, t],
                    1.0,
                )
                gen_outaged && continue
                _add_to_jump_expression!(
                    expression.data[expr_outage_pos, expr_gen_pos, t],
                    gen_var.data[gen_var_pos, t],
                    1.0,
                )
            end
        end
    end
    return
end

# ----------------------------------------------------------------------------
# Sparse-monitored post-contingency flow expression (PTDF only):
#     flow[c, ℓ, t] = pre_flow[ℓ, t] + Σ_b PTDF[ℓ, b] * deployment[c, b, t]
# Only built for monitored components carried by the service-claimed outages
# in `service_model.outages`. The branch type is taken from the monitored
# tuple so the correct `PTDFBranchFlow` container is consulted per component.
# ----------------------------------------------------------------------------

function add_post_contingency_flow_expressions!(
    container::OptimizationContainer,
    ::Type{T},
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{N},
    resolved::Vector{
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}},
    },
) where {
    T <: PostContingencyBranchFlow,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
    N <: AbstractPTDFModel,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    net_reduction_data = network_model.network_reduction
    expression_container = _add_service_post_contingency_sparse_expression!(
        container, T, R, service_name,
    )
    isempty(resolved) && return expression_container

    nodal_deployment = get_expression(
        container,
        PostContingencyNodalActivePowerDeployment(),
        R,
        service_name,
    )
    ptdf = get_PTDF_matrix(network_model)
    full_bus_axis = PNM.get_bus_axis(ptdf)
    full_bus_position = Dict{Int, Int}(b => i for (i, b) in enumerate(full_bus_axis))
    # `nodal_deployment`'s bus axis (`relevant_buses`) is the injection-relevant
    # subset built by `_injection_relevant_buses`; `full_bus_position` maps each
    # of those bus numbers to its position in the full PTDF bus axis.
    relevant_buses = collect(nodal_deployment.axes[2])

    # Cache the pre-contingency-flow container and the (local bus position,
    # coefficient) pairs per (monitored_type, arc): `ptdf[arc, :]` is a KLU
    # solve, and multiple outages commonly monitor the same arc, so both the
    # solve and the nonzero-coefficient scan happen once per arc rather than
    # once per (arc, outage) pair. Mirrors `AC_branches.jl`'s `nz_idx` caching.
    pre_flow_cache = Dict{DataType, DenseAxisArray{GAE}}()
    column_cache = Dict{Tuple{DataType, String}, Vector{Tuple{Int, Float64}}}()
    for (_, entries) in resolved
        for (entry_type, name, arc, _) in entries
            haskey(column_cache, (entry_type, name)) && continue
            ptdf_col = ptdf[arc, :]
            # `ptdf[arc, :]` uses the series representative's orientation;
            # flip series members back to native from→to, mirroring
            # `AC_branches.jl`'s treatment of the pre-contingency flow.
            orientation_sign =
                get_ptdf_orientation_sign(net_reduction_data, entry_type, name)
            coefs = Tuple{Int, Float64}[]
            for (local_pos, bus_number) in enumerate(relevant_buses)
                coef = orientation_sign * ptdf_col[full_bus_position[bus_number]]
                abs(coef) < PTDF_ZERO_TOL && continue
                push!(coefs, (local_pos, coef))
            end
            column_cache[(entry_type, name)] = coefs
        end
    end

    for (uuid, entries) in resolved
        outage_id = string(uuid)
        outage_pos = nodal_deployment.lookup[1][outage_id]
        post_cont_expr = view(nodal_deployment.data, outage_pos, :, :)
        for (entry_type, name, _, _) in entries
            pre_flow = get!(pre_flow_cache, entry_type) do
                get_expression(container, PTDFBranchFlow(), entry_type)
            end
            coefs = column_cache[(entry_type, name)]
            pre_flow_pos = pre_flow.lookup[1][name]
            for t in time_steps
                pre_flow_entry = pre_flow.data[pre_flow_pos, t]
                acc = get_hinted_aff_expr(
                    length(JuMP.linear_terms(pre_flow_entry)) + length(coefs),
                )
                JuMP.add_to_expression!(acc, pre_flow_entry)
                @inbounds for (local_pos, coef) in coefs
                    JuMP.add_to_expression!(acc, coef, post_cont_expr[local_pos, t])
                end
                expression_container[outage_id, name, t] = acc
            end
        end
    end
    return expression_container
end

# ----------------------------------------------------------------------------
# Post-contingency constraints
# ----------------------------------------------------------------------------

"""
Per-outage system-wide generation-balance constraint: the
`PostContingencyActivePowerBalance` expression (sum of reserve deployments
minus the outaged generation) must close to zero for every outage and time.
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    ::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractPowerModel},
    outage_ids::Vector{String},
) where {
    T <: PostContingencyGenerationBalanceConstraint,
    U <: PostContingencyActivePowerBalance,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    expressions = get_expression(container, U(), R, service_name)
    constraint = add_constraints_container!(
        container,
        T(),
        R,
        outage_ids,
        time_steps;
        meta = service_name,
    )
    jump_model = get_jump_model(container)
    for outage_id in outage_ids
        for t in time_steps
            constraint[outage_id, t] =
                JuMP.@constraint(jump_model, expressions[outage_id, t] == 0)
        end
    end
    return
end

"""
Sparse-monitored post-contingency branch flow inequalities. The container is
keyed by `(outage_id::String, monitored_name::String, t::Int)` and only
entries resolved from `service_model.outages` are populated. Limits use the
monitored branch's emergency rating. Optional non-negative slacks relax the
inequalities at `CONSTRAINT_VIOLATION_SLACK_COST`.
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{<:AbstractPTDFModel},
    resolved::Vector{
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}},
    },
) where {
    T <: PostContingencyFlowRateConstraint,
    U <: PostContingencyBranchFlow,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    net_reduction_data = network_model.network_reduction
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    con_lb = _add_post_contingency_sparse_constraints!(
        container, T, R; meta = "$(service_name)_lb",
    )
    con_ub = _add_post_contingency_sparse_constraints!(
        container, T, R; meta = "$(service_name)_ub",
    )
    isempty(resolved) && return

    post_cont_flow = get_expression(container, U(), R, service_name)
    jump_model = get_jump_model(container)

    use_slacks = get_use_slacks(service_model)
    slack_resolved = [
        uuid => [name for (_, name, _, _) in entries] for (uuid, entries) in resolved
    ]
    slack_ub, slack_lb =
        _add_flow_rate_slack_pair!(
            container,
            service,
            service_name,
            slack_resolved,
            F(),
            use_slacks,
        )

    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (entry_type, name, arc, reduction_kind) in entries
            reduction_entry =
                all_branch_maps_by_type[reduction_kind][entry_type][arc]
            limits = get_emergency_min_max_limits(
                reduction_entry, T, StaticBranch,
            )
            for t in time_steps
                if use_slacks
                    con_ub[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        post_cont_flow[outage_id, name, t] -
                        slack_ub[outage_id, name, t] <= limits.max,
                    )
                    con_lb[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        post_cont_flow[outage_id, name, t] +
                        slack_lb[outage_id, name, t] >= limits.min,
                    )
                else
                    con_ub[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        post_cont_flow[outage_id, name, t] <= limits.max,
                    )
                    con_lb[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        post_cont_flow[outage_id, name, t] >= limits.min,
                    )
                end
            end
        end
    end
    return
end

# ----------------------------------------------------------------------------
# AreaBalance network model: post-contingency AreaInterchange flow-deviation
# variable Δf[c,ℓ,t] (`PostContingencyAreaInterchangeFlowDeviationVariable`,
# free, built for every AreaInterchange ℓ in the template's AreaInterchange
# `DeviceModel` set) plus the monitored post-contingency flow expression and
# rate-limit constraints. Ties outside that set carry no Δf term and drop out
# of the balance.
#
# The per-area balance (`PostContingencyCopperPlateBalanceConstraint`, below)
# routes reserve deployment across areas through Δf:
#     Σ_{g∈a,g∉G_c} Δrsv[c,g,t] − p[out,t]·1[out∈a]
#       − Σ_{ℓ: from(ℓ)=a} Δf[c,ℓ,t] + Σ_{ℓ: to(ℓ)=a} Δf[c,ℓ,t] = 0
# Contributing devices may sit in a different area than the outaged
# generator; Δf carries the response over the tie(s) between them. This
# constraint never references the pre-contingency `ActivePowerBalance`
# expression.
#
# The monitored post-contingency flow is then simply
#     post_flow[c,ℓ,t] = f[ℓ,t] + Δf[c,ℓ,t]
# for the AreaInterchanges named in `service_model.outages[uuid]`; the
# rate-limit constraint (unchanged in form) bounds this against the tie's
# emergency limits.
# ----------------------------------------------------------------------------

"""
Resolve every monitored `PSY.AreaInterchange` carried by
`service_model.outages` to its system component. Mirrors
`_resolve_service_monitored_arcs` but for the AreaBalance path where the
monitored object is an AreaInterchange (not a branch arc) and the only
information needed downstream is the component itself. Outages with no
monitored AreaInterchanges are skipped; the returned vector is sorted by
UUID for deterministic axes.

Template validation (`_check_monitored_area_interchanges`) already rejects an
unavailable or out-of-scope monitored `AreaInterchange` before build; the
availability check below is a defensive backstop, not the first reporter.
"""
function _resolve_service_monitored_area_interchanges(
    sys::PSY.System,
    service_model::ServiceModel,
)
    resolved = Pair{Base.UUID, Vector{Tuple{String, PSY.AreaInterchange}}}[]
    for (uuid, per_type) in get_outages(service_model)
        kept = Tuple{String, PSY.AreaInterchange}[]
        names = get(per_type, PSY.AreaInterchange, nothing)
        if !isnothing(names)
            for name in sort!(collect(names))
                comp = PSY.get_component(PSY.AreaInterchange, sys, name)
                isnothing(comp) && error(
                    "Monitored AreaInterchange \"$name\" (outage $uuid) does not " *
                    "exist in the system. Verify the outage's monitored_components " *
                    "reference a PSY.AreaInterchange present in the system.",
                )
                !PSY.get_available(comp) && error(
                    "Monitored AreaInterchange \"$name\" (outage $uuid) is not " *
                    "available. A monitored AreaInterchange must be available.",
                )
                push!(kept, (name, comp))
            end
        end
        isempty(kept) && continue
        push!(resolved, uuid => kept)
    end
    sort!(resolved; by = first)
    return resolved
end

# The Δf axis, the per-area balance's tie maps and the monitored
# post-contingency flow expression must all span exactly the ties the
# pre-contingency `FlowActivePowerVariable` was built over — the
# `AreaInterchange` `DeviceModel`'s set, which honors that DeviceModel's
# `filter_function`. Reading that variable's axis is what guarantees the
# lockstep: a tie with no pre-contingency flow variable has no flow for Δf to
# deviate from, so a free Δf on it would be an unanchored transfer path.
# The AreaInterchange device constructor runs in `ArgumentConstructStage`, so
# the container exists by the time the service's `ModelConstructStage` runs.
function _modeled_area_interchange_names(container::OptimizationContainer)
    has_container_key(container, FlowActivePowerVariable, PSY.AreaInterchange) ||
        return String[]
    flow_var = get_variable(container, FlowActivePowerVariable(), PSY.AreaInterchange)
    return collect(axes(flow_var, 1))
end

# A template with no modeled `AreaInterchange` ties models no inter-area
# transfer at all, so the per-area post-contingency balance reduces to in-area
# coverage. That is the right physics for such a template but a silent trap
# for a user who expected cross-area deployment, so say so once per service.
# The empty `FlowActivePowerVariable__AreaInterchange` axis this dispatches on
# collapses two distinct causes: the system has no `PSY.AreaInterchange`
# components at all, or it has some but the template's `AreaInterchange`
# `DeviceModel` was never registered (or its `filter_function`/`subsystem`
# excluded every tie) — `validate_template_impl!` prunes a `DeviceModel` with
# an empty device cache before this ever runs, so "registered but empty" is
# indistinguishable from "not registered" by the time this warning fires.
# Telling a user to register a `DeviceModel` they already registered (because
# their system simply has no ties to model) is worse than saying nothing
# useful, so branch on `sys` to pick the accurate guidance.
function _warn_missing_area_interchange_model(
    container::OptimizationContainer,
    sys::PSY.System,
    service_name::String,
)
    has_container_key(container, FlowActivePowerVariable, PSY.AreaInterchange) && return
    if !PSY.has_components(sys, PSY.AreaInterchange)
        @warn "Security-constrained reserve service $(service_name) under \
               AreaBalancePowerModel: the system has no PSY.AreaInterchange \
               components, so no cross-area post-contingency transfer can be \
               modeled: reserve deployment cannot cross area boundaries and \
               every area must cover its own outages." _group =
            LOG_GROUP_SERVICE_CONSTUCTORS
    else
        @warn "Security-constrained reserve service $(service_name) under \
               AreaBalancePowerModel has no PSY.AreaInterchange device model in the \
               template, so no post-contingency flow-deviation variable is built: \
               reserve deployment cannot cross area boundaries and every area must \
               cover its own outages. Register a DeviceModel via \
               `set_device_model!(template, PSY.AreaInterchange, StaticBranch)`, or, \
               if one is already registered, check that its `filter_function` and \
               `subsystem` are not excluding every AreaInterchange in the system." _group =
            LOG_GROUP_SERVICE_CONSTUCTORS
    end
    return
end

"""
Build the free post-contingency `AreaInterchange` flow-deviation variable
`Δf[c,ℓ,t]` (`PostContingencyAreaInterchangeFlowDeviationVariable`) for the
`AreaBalancePowerModel` network representation. Built over every
`PSY.AreaInterchange` in the template's `AreaInterchange` `DeviceModel` set —
not only the ones a given outage monitors — because the per-area balance
constraint needs a deviation term on every modeled tie that could carry a
rebalancing flow between areas. Ties outside that set (unavailable, out of
the DeviceModel's subsystem, or excluded by its `filter_function`) have no
pre-contingency flow variable, get no Δf term, and so drop out of the balance
entirely. Unbounded: no lower bound, since deployment may flow in either
direction relative to the pre-contingency flow.
"""
function add_post_contingency_area_interchange_flow_deviation_variables!(
    container::OptimizationContainer,
    sys::PSY.System,
    service::R,
    ::ServiceModel{R, F},
    ::NetworkModel{<:AreaBalancePowerModel},
    outage_ids::Vector{String},
) where {R <: PSY.AbstractReserve, F <: AbstractSecurityConstrainedReservesFormulation}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    jump_model = get_jump_model(container)
    _warn_missing_area_interchange_model(container, sys, service_name)
    interchange_names = _modeled_area_interchange_names(container)
    # No modeled tie means no deviation variable at all: an empty-axis
    # container would also break `read_variables` on the results.
    isempty(interchange_names) && return

    variable = lazy_container_addition!(
        container,
        PostContingencyAreaInterchangeFlowDeviationVariable(),
        R,
        outage_ids,
        interchange_names,
        time_steps;
        meta = service_name,
    )
    base_prefix = "PostContingencyAreaInterchangeFlowDeviationVariable_$(R)_$(service_name)_"
    for outage_id in outage_ids, name in interchange_names, t in time_steps
        variable[outage_id, name, t] = JuMP.@variable(
            jump_model,
            base_name = "$(base_prefix){$(outage_id), $(name), $(t)}",
            start = 0.0,
        )
    end
    return
end

"""
Build the post-contingency AreaInterchange flow expression for the
AreaBalance network model. See module-level comment above for the formula.
The container is a `SparseAxisArray` keyed by
`(outage_id, area_interchange_name, t)` registered under
`ExpressionKey(PostContingencyAreaInterchangeFlow, R; meta = service_name)`.
"""
function add_post_contingency_flow_expressions!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:AreaBalancePowerModel},
    resolved::Vector{Pair{Base.UUID, Vector{Tuple{String, PSY.AreaInterchange}}}},
) where {
    T <: PostContingencyAreaInterchangeFlow,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)

    expression_container =
        _add_service_post_contingency_sparse_expression!(container, T, R, service_name)
    isempty(resolved) && return expression_container

    # Backstop for callers that skip template validation
    # (`_check_monitored_area_interchanges` reports this first).
    modeled_names = Set{String}(_modeled_area_interchange_names(container))
    for (uuid, entries) in resolved
        for (name, _) in entries
            name in modeled_names && continue
            throw(
                IS.ConflictingInputsError(
                    "Monitored AreaInterchange \"$name\" (outage $uuid, service " *
                    "$service_name) is not in the template's PSY.AreaInterchange " *
                    "device model set, so it has no pre-contingency flow variable " *
                    "and no post-contingency flow deviation.",
                ),
            )
        end
    end

    # Baseline flow variable for the AreaInterchange (from→to convention).
    flow_var = get_variable(container, FlowActivePowerVariable(), PSY.AreaInterchange)

    # Free flow-deviation variable Δf[c,ℓ,t]; the per-area balance constraint
    # is what pins its value, so the monitored flow is just f_pre + Δf.
    flow_deviation_variable = get_variable(
        container,
        PostContingencyAreaInterchangeFlowDeviationVariable(),
        R,
        service_name,
    )

    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (name, _) in entries, t in time_steps
            acc = JuMP.AffExpr(0.0)
            JuMP.add_to_expression!(acc, flow_var[name, t])
            JuMP.add_to_expression!(acc, flow_deviation_variable[outage_id, name, t])
            expression_container[outage_id, name, t] = acc
        end
    end
    return expression_container
end

"""
Per-(outage, area_interchange, t) flow-rate inequalities under the
AreaBalance network model. Limits come from
`PSY.get_flow_limits(area_interchange)` as `[-from_to, +to_from]`. Optional
non-negative slacks relax the inequalities at
`CONSTRAINT_VIOLATION_SLACK_COST`.
"""
function add_constraints!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:AreaBalancePowerModel},
    resolved::Vector{Pair{Base.UUID, Vector{Tuple{String, PSY.AreaInterchange}}}},
) where {
    T <: PostContingencyFlowRateConstraint,
    U <: PostContingencyAreaInterchangeFlow,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)

    con_lb = _add_post_contingency_sparse_constraints!(
        container, T, R; meta = "$(service_name)_lb",
    )
    con_ub = _add_post_contingency_sparse_constraints!(
        container, T, R; meta = "$(service_name)_ub",
    )
    isempty(resolved) && return

    post_cont_flow = get_expression(container, U(), R, service_name)
    jump_model = get_jump_model(container)

    use_slacks = get_use_slacks(service_model)
    slack_resolved = [
        uuid => [name for (name, _) in entries] for (uuid, entries) in resolved
    ]
    slack_ub, slack_lb =
        _add_flow_rate_slack_pair!(
            container,
            service,
            service_name,
            slack_resolved,
            F(),
            use_slacks,
        )

    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (name, area_interchange) in entries
            flow_limits = PSY.get_flow_limits(area_interchange)
            ub = flow_limits.to_from
            lb = -1.0 * flow_limits.from_to
            for t in time_steps
                if use_slacks
                    con_ub[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        post_cont_flow[outage_id, name, t] -
                        slack_ub[outage_id, name, t] <= ub,
                    )
                    con_lb[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        post_cont_flow[outage_id, name, t] +
                        slack_lb[outage_id, name, t] >= lb,
                    )
                else
                    con_ub[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        post_cont_flow[outage_id, name, t] <= ub,
                    )
                    con_lb[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        post_cont_flow[outage_id, name, t] >= lb,
                    )
                end
            end
        end
    end
    return
end

"""
Per-outage upper bound on the reserve-deployment variable by the
pre-contingency reserve variable. Outaged generators are skipped: their
deployment is already pinned to zero by the variable bounds set in
`add_variables!`, so no row is written for them here.
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{X},
    ::Type{U},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractPowerModel},
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {
    T <: PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
    X <: VariableType,
    U <: AbstractContingencyVariableType,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    device_names = PSY.get_name.(contributing_devices)
    constraint = _add_post_contingency_sparse_constraints!(
        container, T, R; meta = service_name,
    )
    variable = get_variable(container, X(), R, service_name)
    variable_outage = get_variable(container, U(), R, service_name)
    jump_model = get_jump_model(container)
    for outage_id in outage_ids
        outage_pos = variable_outage.lookup[1][outage_id]
        for (i, device) in enumerate(contributing_devices)
            device in outaged_gens[outage_id] && continue
            name = device_names[i]
            device_pos = variable_outage.lookup[2][name]
            var_pos = variable.lookup[1][name]
            for t in time_steps
                constraint[outage_id, name, t] = JuMP.@constraint(
                    jump_model,
                    variable_outage.data[outage_pos, device_pos, t] <=
                    variable.data[var_pos, t],
                )
            end
        end
    end
    return
end

"""
Per-(outage, generator, t) upper bound on the
`PostContingencyActivePowerGeneration` expression. Used when the service has
no reserve requirement time series.

No lower bound is imposed: for an up-reserve formulation, post-contingency
generation is `p[g,t] + Δ[c,g,t]` with `p[g,t] >= limits.min * u[g,t]`
(pre-contingency bound) and `Δ[c,g,t] >= 0` (deployment is strictly
non-negative), so `expressions[...] >= limits.min` already holds whenever
`u[g,t] == 1`. With unit-commitment status (`u[g,t] == 0`), that redundant
bound is actively wrong — it would force an offline unit's post-contingency
generation up to `limits.min` even though it deployed nothing.
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractActivePowerModel},
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {
    T <: PostContingencyActivePowerGenerationLimitsConstraint,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    con_ub = add_constraints_container!(
        container,
        T(),
        R,
        outage_ids,
        PSY.get_name.(contributing_devices),
        time_steps;
        meta = "$(service_name)_ub",
    )
    expressions =
        get_expression(container, PostContingencyActivePowerGeneration(), R, service_name)
    jump_model = get_jump_model(container)
    for device in contributing_devices
        name = PSY.get_name(device)
        limits = PSY.get_active_power_limits(device)
        con_device_pos = con_ub.lookup[2][name]
        expr_device_pos = expressions.lookup[2][name]
        for outage_id in outage_ids
            gen_outaged = device in outaged_gens[outage_id]
            con_outage_pos = con_ub.lookup[1][outage_id]
            expr_outage_pos = expressions.lookup[1][outage_id]
            for t in time_steps
                if gen_outaged
                    con_ub.data[con_outage_pos, con_device_pos, t] = JuMP.@constraint(
                        jump_model,
                        expressions.data[expr_outage_pos, expr_device_pos, t] == 0.0,
                    )
                    continue
                end
                con_ub.data[con_outage_pos, con_device_pos, t] = JuMP.@constraint(
                    jump_model,
                    expressions.data[expr_outage_pos, expr_device_pos, t] <= limits.max,
                )
            end
        end
    end
    return
end

# Per-(outage, area, t) area balance with no tie term, used when the template
# models no `AreaInterchange`: with no modeled tie there is no Δf variable, so
# every area must cover the outages sited in it out of its own deployment.
function _add_local_area_balance_constraints!(
    container::OptimizationContainer,
    con::JuMPConstraintArray,
    contingency_expression::DenseAxisArray,
    devices::IS.FlattenIteratorWrapper{PSY.Area},
    outage_ids::Vector{String},
)
    jump_model = get_jump_model(container)
    for outage_id in outage_ids
        con_outage_pos = con.lookup[1][outage_id]
        expr_outage_pos = contingency_expression.lookup[1][outage_id]
        for area in devices
            area_name = PSY.get_name(area)
            con_area_pos = con.lookup[2][area_name]
            expr_area_pos = contingency_expression.lookup[2][area_name]
            for t in get_time_steps(container)
                con.data[con_outage_pos, con_area_pos, t] = JuMP.@constraint(
                    jump_model,
                    contingency_expression.data[expr_outage_pos, expr_area_pos, t] == 0.0,
                )
            end
        end
    end
    return
end

"""
Per-(outage, area, t) area balance for the `AreaBalancePowerModel`. See the
`PostContingencyCopperPlateBalanceConstraint` docstring for the formula: the
`PostContingencyAreaActivePowerDeployment` expression (deployed reserves in
the area minus outaged generation sited there), adjusted for the
`PostContingencyAreaInterchangeFlowDeviationVariable` on every modeled tie
touching the area, must sum to zero. Never references the pre-contingency
`ActivePowerBalance` expression: contributing devices may sit in a different
area than the outaged generator, and the flow deviation carries the response
across the tie(s) between them.
"""
function add_constraints!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{<:AreaBalancePowerModel},
    outage_ids::Vector{String},
) where {
    T <: PostContingencyCopperPlateBalanceConstraint,
    U <: PostContingencyAreaActivePowerDeployment,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    devices = get_available_components(network_model, PSY.Area, sys)
    area_names = PSY.get_name.(devices)
    service_name = PSY.get_name(service)
    con = add_constraints_container!(
        container,
        T(),
        R,
        outage_ids,
        area_names,
        time_steps;
        meta = service_name,
    )
    contingency_expression = get_expression(container, U(), R, service_name)

    # The tie maps, the Δf axis and the monitored post-contingency flow
    # expression all read this one set, so they cannot reference a tie without
    # a Δf term (or a Δf term without a pre-contingency flow).
    modeled_names = Set{String}(_modeled_area_interchange_names(container))
    if isempty(modeled_names)
        _add_local_area_balance_constraints!(
            container, con, contingency_expression, devices, outage_ids,
        )
        return
    end
    flow_deviation_variable = get_variable(
        container,
        PostContingencyAreaInterchangeFlowDeviationVariable(),
        R,
        service_name,
    )

    # Precompute, for each area, which AreaInterchanges touch it as the
    # from-side (−Δf) and which touch it as the to-side (+Δf); mirrors the
    # pre-contingency sign convention in `area_interchange.jl`.
    from_ties = Dict{String, Vector{String}}()
    to_ties = Dict{String, Vector{String}}()
    for area_interchange in PSY.get_components(
        x -> PSY.get_name(x) in modeled_names,
        PSY.AreaInterchange,
        sys,
    )
        name = PSY.get_name(area_interchange)
        push!(
            get!(from_ties, PSY.get_name(PSY.get_from_area(area_interchange)), String[]),
            name,
        )
        push!(
            get!(to_ties, PSY.get_name(PSY.get_to_area(area_interchange)), String[]),
            name,
        )
    end

    # Per-area tie sign/position lists don't depend on the outage, so they are
    # hoisted once here rather than rebuilt inside the outage/time loop below.
    tie_terms = Dict{String, Vector{Tuple{Float64, Int}}}()
    for area in devices
        area_name = PSY.get_name(area)
        terms = Tuple{Float64, Int}[]
        for name in get(from_ties, area_name, String[])
            push!(terms, (-1.0, flow_deviation_variable.lookup[2][name]))
        end
        for name in get(to_ties, area_name, String[])
            push!(terms, (1.0, flow_deviation_variable.lookup[2][name]))
        end
        tie_terms[area_name] = terms
    end

    jump_model = get_jump_model(container)
    for outage_id in outage_ids
        con_outage_pos = con.lookup[1][outage_id]
        expr_outage_pos = contingency_expression.lookup[1][outage_id]
        var_outage_pos = flow_deviation_variable.lookup[1][outage_id]
        for area in devices
            area_name = PSY.get_name(area)
            terms = tie_terms[area_name]
            con_area_pos = con.lookup[2][area_name]
            expr_area_pos = contingency_expression.lookup[2][area_name]
            for t in time_steps
                tie_term = JuMP.AffExpr(0.0)
                for (sign, tie_pos) in terms
                    JuMP.add_to_expression!(
                        tie_term, sign,
                        flow_deviation_variable.data[var_outage_pos, tie_pos, t],
                    )
                end
                con.data[con_outage_pos, con_area_pos, t] = JuMP.@constraint(
                    jump_model,
                    contingency_expression.data[expr_outage_pos, expr_area_pos, t] +
                    tie_term == 0.0,
                )
            end
        end
    end
    return
end

# ----------------------------------------------------------------------------
# construct_service! dispatches: argument + model construct stages for
# (SecurityConstrainedContingencyReserve, SecurityConstrainedRampReserve) ×
# (PTDF, CopperPlate, AreaBalance).
# ----------------------------------------------------------------------------

# Whether `F` adds `RampConstraint`s in `_construct_service_pre_contingency!`.
_includes_ramp_constraints(::Type{SecurityConstrainedContingencyReserve}) = false
_includes_ramp_constraints(::Type{SecurityConstrainedRampReserve}) = true

# Whether `F` always requires the reserve-requirement time series, regardless
# of whether `model` happens to map one for `service`.
_requires_requirement_ts(::Type{SecurityConstrainedContingencyReserve}) = false
_requires_requirement_ts(::Type{SecurityConstrainedRampReserve}) = true

# Whether `model` maps `RequirementTimeSeriesParameter` to a time series name
# that `service` actually carries. A service may hold unrelated time series
# (e.g. a cost forecast) without carrying a requirement series at all, so the
# mapped name must be checked against the service's own series, not against
# `length(get_time_series_keys(service)) > 0`.
function _has_mapped_requirement_ts(
    sys::PSY.System,
    service::PSY.AbstractReserve,
    model::ServiceModel,
)
    ts_names = get_time_series_names(model)
    haskey(ts_names, RequirementTimeSeriesParameter) || return false
    return PSY.has_time_series(
        service,
        get_deterministic_time_series_type(sys),
        ts_names[RequirementTimeSeriesParameter],
    )
end

# Whether the pre-contingency `ActivePowerReserveVariable` — and, downstream,
# its matching `RequirementConstraint` — should be built for `service` under
# formulation `F`: either `F` always requires the reserve-requirement time
# series, or `model` happens to map one that `service` actually carries.
# Shared by the arguments stage (decides whether to add the variable) and the
# model-stage prologue (decides whether to add the constraint referencing
# it), so the two decisions cannot drift out of sync.
function _service_requires_requirement_ts(
    sys::PSY.System,
    service::PSY.AbstractReserve,
    model::ServiceModel{SR, F},
) where {SR <: PSY.AbstractReserve, F <: AbstractSecurityConstrainedReservesFormulation}
    return _requires_requirement_ts(F) || _has_mapped_requirement_ts(sys, service, model)
end

# Shared ArgumentConstructStage helper used by both formulations: builds
# pre-contingency reserve variable + post-contingency deployment variable.
function _construct_service_arguments_sc!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
    devices_template::Dict{Symbol, DeviceModel},
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    has_requirement_ts = _service_requires_requirement_ts(sys, service, model)
    if has_requirement_ts
        add_parameters!(container, RequirementTimeSeriesParameter, service, model)
        add_variables!(
            container,
            ActivePowerReserveVariable,
            service,
            contributing_devices,
            F(),
        )
        add_to_expression!(container, ActivePowerReserveVariable, model, devices_template)
    end
    add_feedforward_arguments!(container, model, service)

    outage_ids = _service_outage_ids(model)
    if isempty(outage_ids)
        @warn "Service $(SR)('$name'): `service_model.outages` is empty; the \
               security-constrained formulation $(F) will not add any \
               post-contingency variables or constraints."
        return
    end

    attribute_device_map = PSY.get_component_supplemental_attribute_pairs(
        PSY.Generator, PSY.Outage, sys,
    )
    outaged_gens = _outaged_generators_by_outage_id(outage_ids, attribute_device_map)
    add_variables!(
        container,
        PostContingencyActivePowerReserveDeploymentVariable,
        service,
        model,
        contributing_devices,
        F(),
        outage_ids,
        outaged_gens,
    )
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, F},
    devices_template::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractActivePowerModel},
) where {SR <: PSY.AbstractReserve, F <: AbstractSecurityConstrainedReservesFormulation}
    _construct_service_arguments_sc!(container, sys, model, devices_template)
    return
end

# Shared ModelConstructStage helper for the pre-contingency requirement,
# ramp, participation, objective, feedforward and dual hookups.
function _construct_service_pre_contingency!(
    container::OptimizationContainer,
    sys::PSY.System,
    service::PSY.AbstractReserve,
    contributing_devices,
    model::ServiceModel{SR, F},
    has_requirement_ts::Bool,
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    if has_requirement_ts
        add_constraints!(
            container,
            RequirementConstraint,
            service,
            contributing_devices,
            model,
        )
        _includes_ramp_constraints(F) && add_constraints!(
            container, RampConstraint, service, contributing_devices, model,
        )
        add_constraints!(
            container,
            ParticipationFractionConstraint,
            service,
            contributing_devices,
            model,
        )
        objective_function!(container, service, model)
    end
    add_feedforward_constraints!(container, model, service)
    add_constraint_dual!(container, sys, model)
    return
end

# Shared helper for the post-contingency power-balance + generation-balance
# expression/constraint stack that's common to the PTDF and CopperPlate
# network models. Not called under the AreaBalance path: the system-wide
# `PostContingencyActivePowerBalance` expression and the generation-balance
# constraint built from it are both redundant there (summing the per-area
# balance over all areas recovers it, since the flow deviations cancel), so
# `_construct_service_model_areabalance!` skips this helper entirely rather
# than build O(outages × devices × T) of dead work with no constraint or
# result reader left to consume it.
function _construct_service_post_contingency_balance!(
    container::OptimizationContainer,
    service::PSY.AbstractReserve,
    contributing_devices,
    model::ServiceModel{SR, F},
    network_model::NetworkModel,
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
    attribute_device_map::_AttributeDeviceMap,
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    add_to_expression!(
        container, PostContingencyActivePowerBalance,
        PostContingencyActivePowerReserveDeploymentVariable,
        contributing_devices, service, model, network_model, outage_ids, outaged_gens,
    )
    add_to_expression!(
        container, PostContingencyActivePowerBalance, ActivePowerVariable,
        attribute_device_map, service, model, network_model,
    )
    add_constraints!(
        container, PostContingencyGenerationBalanceConstraint,
        PostContingencyActivePowerBalance,
        contributing_devices, service, model, network_model, outage_ids,
    )
    return
end

"""
Shared prologue for the three network-model service constructors below
(PTDF, CopperPlate, AreaBalance): resolve the service, build the
pre-contingency requirement/ramp/participation/objective stack, and resolve
the outages the service claims along with the outage→generator map and the
`attribute_device_map` scan, both computed once here and threaded down to
every consumer in the calling network-model branch.

If the service is unavailable, returns an empty `contributing_devices` and
`outage_ids` without building anything; callers rely on a single
`isempty(outage_ids) && return` to cover both that case and a service with no
outages attached, rather than a separate nothing-sentinel signal.
"""
function _construct_service_model_prologue!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    if !PSY.get_available(service)
        return service, PSY.Component[], false, String[],
        Dict{String, Set{PSY.Generator}}(), _AttributeDeviceMap()
    end
    contributing_devices = get_contributing_devices(model)

    has_requirement_ts = _service_requires_requirement_ts(sys, service, model)
    _construct_service_pre_contingency!(
        container, sys, service, contributing_devices, model, has_requirement_ts,
    )

    outage_ids = _service_outage_ids(model)
    if isempty(outage_ids)
        return service, contributing_devices, has_requirement_ts, outage_ids,
        Dict{String, Set{PSY.Generator}}(), _AttributeDeviceMap()
    end
    attribute_device_map = PSY.get_component_supplemental_attribute_pairs(
        PSY.Generator, PSY.Outage, sys,
    )
    outaged_gens = _outaged_generators_by_outage_id(outage_ids, attribute_device_map)
    return service, contributing_devices, has_requirement_ts, outage_ids, outaged_gens,
    attribute_device_map
end

"""
Shared tail for the three network-model service constructors below: the
post-contingency reserve-deployment limit, applied either as a constraint
against the pre-contingency `ActivePowerReserveVariable` (when a requirement
time series is modeled) or as a direct generation-limits constraint on the
post-contingency generation expression (when it is not — see
`PostContingencyActivePowerGeneration`'s docstring for why no requirement
time series changes which bound applies).
"""
function _construct_service_deployment_limits!(
    container::OptimizationContainer,
    contributing_devices,
    service::PSY.AbstractReserve,
    model::ServiceModel{SR, F},
    network_model::NetworkModel,
    has_requirement_ts::Bool,
    outage_ids::Vector{String},
    outaged_gens::Dict{String, Set{PSY.Generator}},
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    if has_requirement_ts
        add_constraints!(
            container,
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            ActivePowerReserveVariable,
            PostContingencyActivePowerReserveDeploymentVariable,
            contributing_devices, service, model, network_model, outage_ids, outaged_gens,
        )
    else
        add_to_expression!(
            container, PostContingencyActivePowerGeneration,
            contributing_devices, service, model, network_model, outage_ids, outaged_gens,
        )
        add_constraints!(
            container, PostContingencyActivePowerGenerationLimitsConstraint,
            contributing_devices, service, model, network_model, outage_ids, outaged_gens,
        )
    end
    return
end

# ----- PTDF (DC) network model -----

function _construct_service_model_ptdf!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
    network_model::NetworkModel{<:PM.AbstractDCPModel},
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    service, contributing_devices, has_requirement_ts, outage_ids, outaged_gens,
    attribute_device_map = _construct_service_model_prologue!(container, sys, model)
    isempty(outage_ids) && return

    _construct_service_post_contingency_balance!(
        container, service, contributing_devices, model, network_model, outage_ids,
        outaged_gens, attribute_device_map,
    )
    relevant_buses = _injection_relevant_buses(
        get_network_reduction(network_model), contributing_devices, outaged_gens,
    )
    add_to_expression!(
        container, PostContingencyNodalActivePowerDeployment,
        PostContingencyActivePowerReserveDeploymentVariable,
        contributing_devices, relevant_buses, service, model, network_model,
        outage_ids, outaged_gens,
    )
    add_to_expression!(
        container, PostContingencyNodalActivePowerDeployment, ActivePowerVariable,
        attribute_device_map, service, model, network_model,
    )
    resolved_arcs = _resolve_service_monitored_arcs(
        model, get_network_reduction(network_model),
    )
    add_post_contingency_flow_expressions!(
        container, PostContingencyBranchFlow, service, model, network_model,
        resolved_arcs,
    )
    add_constraints!(
        container, PostContingencyFlowRateConstraint, PostContingencyBranchFlow,
        service, model, network_model, resolved_arcs,
    )

    _construct_service_deployment_limits!(
        container, contributing_devices, service, model, network_model,
        has_requirement_ts, outage_ids, outaged_gens,
    )
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, F},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:PM.AbstractDCPModel},
) where {SR <: PSY.AbstractReserve, F <: AbstractSecurityConstrainedReservesFormulation}
    _construct_service_model_ptdf!(container, sys, model, network_model)
    return
end

# ----- CopperPlate network model -----

function _construct_service_model_copperplate!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
    network_model::NetworkModel{<:CopperPlatePowerModel},
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    service, contributing_devices, has_requirement_ts, outage_ids, outaged_gens,
    attribute_device_map = _construct_service_model_prologue!(container, sys, model)
    isempty(outage_ids) && return

    _construct_service_post_contingency_balance!(
        container, service, contributing_devices, model, network_model, outage_ids,
        outaged_gens, attribute_device_map,
    )

    _construct_service_deployment_limits!(
        container, contributing_devices, service, model, network_model,
        has_requirement_ts, outage_ids, outaged_gens,
    )
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, F},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:CopperPlatePowerModel},
) where {SR <: PSY.AbstractReserve, F <: AbstractSecurityConstrainedReservesFormulation}
    _construct_service_model_copperplate!(container, sys, model, network_model)
    return
end

# ----- AreaBalance network model -----

function _construct_service_model_areabalance!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
    network_model::NetworkModel{<:AreaBalancePowerModel},
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    service, contributing_devices, has_requirement_ts, outage_ids, outaged_gens,
    attribute_device_map = _construct_service_model_prologue!(container, sys, model)
    isempty(outage_ids) && return

    # The system-wide `PostContingencyActivePowerBalance` expression built by
    # `_construct_service_post_contingency_balance!` is redundant here (the
    # per-area balance below recovers it on summation), so it is not called;
    # `attribute_device_map` came from the shared prologue scan instead.
    add_to_expression!(
        container, sys, PostContingencyAreaActivePowerDeployment,
        PostContingencyActivePowerReserveDeploymentVariable,
        contributing_devices, service, model, network_model, outage_ids, outaged_gens,
    )
    add_to_expression!(
        container, PostContingencyAreaActivePowerDeployment, ActivePowerVariable,
        attribute_device_map, service, model, network_model,
    )
    add_post_contingency_area_interchange_flow_deviation_variables!(
        container, sys, service, model, network_model, outage_ids,
    )
    add_constraints!(
        container, sys, PostContingencyCopperPlateBalanceConstraint,
        PostContingencyAreaActivePowerDeployment,
        service, model, network_model, outage_ids,
    )
    resolved_area_interchanges =
        _resolve_service_monitored_area_interchanges(sys, model)
    add_post_contingency_flow_expressions!(
        container, sys, PostContingencyAreaInterchangeFlow,
        service, model, network_model, resolved_area_interchanges,
    )
    add_constraints!(
        container, sys, PostContingencyFlowRateConstraint,
        PostContingencyAreaInterchangeFlow,
        service, model, network_model, resolved_area_interchanges,
    )

    _construct_service_deployment_limits!(
        container, contributing_devices, service, model, network_model,
        has_requirement_ts, outage_ids, outaged_gens,
    )
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, F},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:AreaBalancePowerModel},
) where {SR <: PSY.AbstractReserve, F <: AbstractSecurityConstrainedReservesFormulation}
    _construct_service_model_areabalance!(container, sys, model, network_model)
    return
end
