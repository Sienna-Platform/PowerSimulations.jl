# ----------------------------------------------------------------------------
# Security-constrained reserve service formulations (G-1 with reserve
# deployment + monitored-branch post-contingency flow constraints).
#
# Sparse + monitored counterpart to the legacy dense-per-branch implementation
# in `older_static_injection_security_constrained_models.jl`. Mirrors the
# device-side ac_transmission_security_constrained_models.jl: post-contingency
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

#! format: off
get_variable_upper_bound(
    ::PostContingencyFlowActivePowerSlackUpperBound,
    ::PSY.ACTransmission,
    ::AbstractSecurityConstrainedReservesFormulation,
) = nothing
get_variable_lower_bound(
    ::PostContingencyFlowActivePowerSlackUpperBound,
    ::PSY.ACTransmission,
    ::AbstractSecurityConstrainedReservesFormulation,
) = 0.0
get_variable_upper_bound(
    ::PostContingencyFlowActivePowerSlackLowerBound,
    ::PSY.ACTransmission,
    ::AbstractSecurityConstrainedReservesFormulation,
) = nothing
get_variable_lower_bound(
    ::PostContingencyFlowActivePowerSlackLowerBound,
    ::PSY.ACTransmission,
    ::AbstractSecurityConstrainedReservesFormulation,
) = 0.0
#! format: on

# ----------------------------------------------------------------------------
# Helpers: monitored-arc resolution + sparse container scaffolding
# ----------------------------------------------------------------------------

"""
Resolve every monitored component in `service_model.outages` to a container
name and arc tuple in the active network reduction. Mirrors the device-side
`_resolve_monitored_arcs` but operates on a `ServiceModel`. Outages whose
monitored component types are not modeled in the network are skipped.

Returns
`Vector{Pair{UUID, Vector{Tuple{DataType, String, Tuple{Int,Int}, String}}}}`
where each inner tuple is `(monitored_type, container_name, arc,
reduction_kind)`. Outages are sorted by UUID for deterministic axes.
"""
function _resolve_service_monitored_arcs(
    service_model::ServiceModel,
    net_reduction_data::PNM.NetworkReductionData,
)
    name_to_arc_maps = PNM.get_name_to_arc_maps(net_reduction_data)
    component_to_reduction_maps =
        PNM.get_component_to_reduction_name_map(net_reduction_data)
    resolved =
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}}[]
    for (uuid, per_type) in get_outages(service_model)
        kept = Tuple{DataType, String, Tuple{Int, Int}, String}[]
        for (T, names) in per_type
            haskey(name_to_arc_maps, T) || continue
            name_to_arc = name_to_arc_maps[T]
            component_to_reduction =
                get(component_to_reduction_maps, T, Dict{String, String}())
            seen = Set{Tuple{Int, Int}}()
            for name in sort!(collect(names))
                if haskey(name_to_arc, name)
                    container_name = name
                elseif haskey(component_to_reduction, name)
                    container_name = component_to_reduction[name]
                else
                    error(
                        "Monitored component \"$name\" (type $T) for outage $uuid is " *
                        "absent from both the network-reduction name-to-arc map and " *
                        "the component-to-reduction map. Verify the component exists " *
                        "in the system and is modeled with a branch formulation that " *
                        "produces a PTDFBranchFlow expression.",
                    )
                end
                arc, reduction_kind = name_to_arc[container_name]
                arc in seen && continue
                push!(seen, arc)
                push!(kept, (T, container_name, arc, reduction_kind))
            end
        end
        isempty(kept) && continue
        push!(resolved, uuid => kept)
    end
    sort!(resolved; by = first)
    return resolved
end

"""
Resolve the outages claimed by `service_model.outages` to the `PSY.Outage`
supplemental attribute objects attached to the reserve service in `sys`.
Returned vector is sorted by UUID for deterministic axes.

The element type is `PSY.Outage` (rather than `PSY.UnplannedOutage`) so that
the `"include_planned_outages"` opt-in honored by
`_build_service_model_outages!` in `template_validation.jl` can flow through
without a `MethodError` when a `PSY.PlannedOutage` is claimed.

This is the service-side counterpart to iterating `get_outages(device_model)`
on the AC-branch side: outages are attached to the reserve service (and,
typically, also to the outaged generator), and resolution requires a UUID
lookup against the system. Callers use the resolved objects to query
`PSY.get_associated_components(sys, outage; component_type = PSY.Generator)`
and pin the outaged generator's deployment variable to zero.
"""
function _service_outages(sys::PSY.System, service_model::ServiceModel)
    outage_uuids = sort!(collect(keys(get_outages(service_model))))
    return PSY.Outage[
        PSY.get_supplemental_attribute(sys, uuid) for uuid in outage_uuids
    ]
end

"""
Pre-allocate a `SparseAxisArray` keyed by
`(outage_id::String, monitored_name::String, t::Int)` holding zero `AffExpr`s
for the resolved monitored arcs. Registered on `container.expressions` under
`ExpressionKey(T, R; meta = service_name)`.
"""
function _add_service_post_contingency_sparse_expression!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{R},
    service_name::String,
    resolved::Vector{
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}},
    },
    time_steps::UnitRange{Int},
) where {T <: PostContingencyExpressions, R <: PSY.AbstractReserve}
    contents = Dict{Tuple{String, String, Int}, JuMP.AffExpr}()
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (_, name, _, _) in entries, t in time_steps
            contents[(outage_id, name, t)] = zero(JuMP.AffExpr)
        end
    end
    expr_container = SparseAxisArray(contents)
    _assign_container!(
        container.expressions,
        ExpressionKey(T, R, service_name),
        expr_container,
    )
    return expr_container
end

"""
Register an empty `SparseAxisArray` keyed by
`(outage_id::String, monitored_name::String, t::Int)` for the given
post-contingency constraint type / meta tag.
"""
function _add_service_post_contingency_sparse_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{R},
    service_name::String;
    meta_suffix::String,
) where {T <: ConstraintType, R <: PSY.AbstractReserve}
    cons_container =
        SparseAxisArray(Dict{Tuple{String, String, Int}, JuMP.ConstraintRef}())
    _assign_container!(
        container.constraints,
        ConstraintKey(T, R, "$(service_name)_$(meta_suffix)"),
        cons_container,
    )
    return cons_container
end

"""
Sparse slack variable container keyed by
`(outage_id::String, monitored_name::String, t::Int)`. Each entry is a
non-negative `JuMP.VariableRef` whose objective contribution is
`POST_CONTINGENCY_CONSTRAINT_VIOLATION_SLACK_COST`. Built directly via
`@variable`/`_assign_container!` so the axes can be sparse.
"""
function add_post_contingency_slack_variables!(
    container::OptimizationContainer,
    ::Type{T},
    service::R,
    service_name::String,
    resolved::Vector{
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}},
    },
    ::AbstractSecurityConstrainedReservesFormulation,
) where {T <: AbstractContingencySlackVariableType, R <: PSY.AbstractReserve}
    time_steps = get_time_steps(container)
    jump_model = get_jump_model(container)
    contents = Dict{Tuple{String, String, Int}, JuMP.VariableRef}()
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (_, name, _, _) in entries
            for t in time_steps
                v = JuMP.@variable(
                    jump_model,
                    base_name = "$(T)_$(R)_$(service_name)_{$(outage_id), $(name), $(t)}",
                    lower_bound = 0.0,
                    start = 0.0,
                )
                contents[(outage_id, name, t)] = v
                add_to_objective_invariant_expression!(
                    container,
                    v * POST_CONTINGENCY_CONSTRAINT_VIOLATION_SLACK_COST,
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

# ----------------------------------------------------------------------------
# Reserve deployment variable per (outage, contributing device, t)
# ----------------------------------------------------------------------------

function add_variables!(
    container::OptimizationContainer,
    sys::PSY.System,
    variable_type::Type{T},
    service::R,
    service_model::ServiceModel{R, <:AbstractSecurityConstrainedReservesFormulation},
    contributing_devices::Vector{V},
    formulation::AbstractSecurityConstrainedReservesFormulation,
) where {
    T <: AbstractContingencyVariableType,
    R <: PSY.AbstractReserve,
    V <: PSY.StaticInjection,
}
    @assert !isempty(contributing_devices)
    time_steps = get_time_steps(container)
    binary = get_variable_binary(variable_type(), R, formulation)
    service_name = PSY.get_name(service)

    # Outages claimed by this service live on `service_model.outages` (UUID
    # keys). Resolve them to the supplemental attribute objects so we can
    # query the associated outaged generators and pin their deployment
    # variables to zero under their own contingency.
    associated_outages = _service_outages(sys, service_model)
    outage_ids = string.(IS.get_uuid.(associated_outages))

    variable = lazy_container_addition!(
        container,
        variable_type(),
        R,
        outage_ids,
        [PSY.get_name(d) for d in contributing_devices],
        time_steps;
        meta = service_name,
    )

    for outage in associated_outages
        outage_id = string(IS.get_uuid(outage))
        associated_devices =
            PSY.get_associated_components(sys, outage; component_type = PSY.Generator)
        for device in contributing_devices
            name = PSY.get_name(device)
            device_outaged = device in associated_devices
            for t in time_steps
                v = JuMP.@variable(
                    get_jump_model(container),
                    base_name = "$(T)_$(R)_$(service_name)_{$(outage_id), $(name), $(t)}",
                    binary = binary,
                )
                variable[outage_id, name, t] = v
                if device_outaged
                    # The outaged generator cannot deploy reserves for its own
                    # contingency; force the variable to zero.
                    JuMP.set_upper_bound(v, 0.0)
                    JuMP.set_lower_bound(v, 0.0)
                    JuMP.set_start_value(v, 0.0)
                    continue
                end
                ub = get_variable_upper_bound(variable_type(), service, device, formulation)
                ub === nothing || JuMP.set_upper_bound(v, ub)
                lb = get_variable_lower_bound(variable_type(), service, device, formulation)
                (lb === nothing || binary) || JuMP.set_lower_bound(v, lb)
                init = get_variable_warm_start_value(variable_type(), device, formulation)
                init === nothing || JuMP.set_start_value(v, init)
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
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PostContingencyActivePowerBalance,
    U <: AbstractContingencyVariableType,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    associated_outages = _service_outages(sys, service_model)
    expression = lazy_container_addition!(
        container,
        T(),
        R,
        string.(IS.get_uuid.(associated_outages)),
        time_steps;
        meta = service_name,
    )
    reserve_deployment_variable = get_variable(container, U(), R, service_name)
    mult_default = get_variable_multiplier(U(), R, F())
    for outage in associated_outages
        associated_devices =
            PSY.get_associated_components(sys, outage; component_type = PSY.Generator)
        outage_id = string(IS.get_uuid(outage))
        for device in contributing_devices
            name = PSY.get_name(device)
            mult = device in associated_devices ? 0.0 : mult_default
            for t in time_steps
                _add_to_jump_expression!(
                    expression[outage_id, t],
                    reserve_deployment_variable[outage_id, name, t],
                    mult,
                )
            end
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    attribute_device_map::Vector{
        NamedTuple{(:component, :supplemental_attribute), Tuple{V, PSY.Outage}},
    },
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PostContingencyActivePowerBalance,
    U <: VariableType,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    associated_outages = Set(_service_outages(sys, service_model))
    expression = get_expression(container, T(), R, service_name)
    for (d, outage) in attribute_device_map
        outage in associated_outages || continue
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

function add_to_expression!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{N},
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
    associated_outages = _service_outages(sys, service_model)
    ptdf = get_PTDF_matrix(network_model)
    bus_numbers = PNM.get_bus_axis(ptdf)
    expression = lazy_container_addition!(
        container,
        T(),
        R,
        string.(IS.get_uuid.(associated_outages)),
        bus_numbers,
        time_steps;
        meta = service_name,
    )
    reserve_deployment_variable = get_variable(container, U(), R, service_name)
    mult_default = get_variable_multiplier(U(), R, F())
    network_reduction = get_network_reduction(network_model)
    for outage in associated_outages
        associated_devices =
            PSY.get_associated_components(sys, outage; component_type = PSY.Generator)
        outage_id = string(IS.get_uuid(outage))
        for device in contributing_devices
            mult = device in associated_devices ? 0.0 : mult_default
            name = PSY.get_name(device)
            bus_number =
                PNM.get_mapped_bus_number(network_reduction, PSY.get_bus(device))
            for t in time_steps
                _add_to_jump_expression!(
                    expression[outage_id, bus_number, t],
                    reserve_deployment_variable[outage_id, name, t],
                    mult,
                )
            end
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    attribute_device_map::Vector{
        NamedTuple{(:component, :supplemental_attribute), Tuple{V, PSY.Outage}},
    },
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{N},
) where {
    T <: PostContingencyNodalActivePowerDeployment,
    U <: VariableType,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
    N <: AbstractPTDFModel,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    associated_outages = Set(_service_outages(sys, service_model))
    expression = get_expression(container, T(), R, service_name)
    network_reduction = get_network_reduction(network_model)
    for (device, outage) in attribute_device_map
        outage in associated_outages || continue
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
    ::NetworkModel{<:AreaBalancePowerModel},
) where {
    T <: PostContingencyAreaActivePowerDeployment,
    U <: AbstractContingencyVariableType,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    associated_outages = _service_outages(sys, service_model)
    area_names = PSY.get_name.(PSY.get_components(PSY.Area, sys))
    expression = lazy_container_addition!(
        container,
        T(),
        R,
        string.(IS.get_uuid.(associated_outages)),
        area_names,
        time_steps;
        meta = service_name,
    )
    reserve_deployment_variable = get_variable(container, U(), R, service_name)
    mult_default = get_variable_multiplier(U(), R, F())
    for outage in associated_outages
        associated_devices =
            PSY.get_associated_components(sys, outage; component_type = PSY.Generator)
        outage_id = string(IS.get_uuid(outage))
        for device in contributing_devices
            mult = device in associated_devices ? 0.0 : mult_default
            name = PSY.get_name(device)
            area_name = PSY.get_name(PSY.get_area(PSY.get_bus(device)))
            for t in time_steps
                _add_to_jump_expression!(
                    expression[outage_id, area_name, t],
                    reserve_deployment_variable[outage_id, name, t],
                    mult,
                )
            end
        end
    end
    return
end

function add_to_expression!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:AreaBalancePowerModel},
) where {
    T <: PostContingencyAreaActivePowerDeployment,
    U <: VariableType,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    attribute_device_map = PSY.get_component_supplemental_attribute_pairs(
        PSY.Generator,
        PSY.UnplannedOutage,
        sys,
    )
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    associated_outages = Set(_service_outages(sys, service_model))
    expression = get_expression(container, T(), R, service_name)
    for (device, outage) in attribute_device_map
        outage in associated_outages || continue
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
    sys::PSY.System,
    ::Type{T},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: PostContingencyActivePowerGeneration,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    associated_outages = _service_outages(sys, service_model)
    expression = add_expression_container!(
        container,
        T(),
        R,
        string.(IS.get_uuid.(associated_outages)),
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
        for outage in associated_outages
            associated_devices = PSY.get_associated_components(
                sys, outage; component_type = PSY.Generator,
            )
            outage_id = string(IS.get_uuid(outage))
            gen_outaged = device in associated_devices
            for t in time_steps
                _add_to_jump_expression!(
                    expression[outage_id, gen_name, t],
                    reserve_deployment_variable[outage_id, gen_name, t],
                    1.0,
                )
                gen_outaged && continue
                _add_to_jump_expression!(
                    expression[outage_id, gen_name, t],
                    gen_var[gen_name, t],
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
) where {
    T <: PostContingencyBranchFlow,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
    N <: AbstractPTDFModel,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    net_reduction_data = network_model.network_reduction
    resolved = _resolve_service_monitored_arcs(service_model, net_reduction_data)
    expression_container = _add_service_post_contingency_sparse_expression!(
        container, T, R, service_name, resolved, time_steps,
    )
    isempty(resolved) && return expression_container

    nodal_deployment = get_expression(
        container,
        PostContingencyNodalActivePowerDeployment(),
        R,
        service_name,
    )
    ptdf = get_PTDF_matrix(network_model)

    # Cache PTDF columns per (monitored_type, arc) — multiple outages may
    # monitor the same arc.
    pre_flow_cache = Dict{DataType, Any}()
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        # Positional slice over the bus/time axes; matches `ptdf_col`'s
        # positional indexing and avoids keyed lookup mismatches when the
        # nodal expression's bus axis is a subset of the PTDF column space
        # (e.g. AreaPTDFPowerModel).
        post_cont_expr = nodal_deployment[outage_id, :, :].data
        for (entry_type, name, arc, _) in entries
            pre_flow = get!(pre_flow_cache, entry_type) do
                get_expression(container, PTDFBranchFlow(), entry_type)
            end
            ptdf_col = ptdf[arc, :]
            for t in time_steps
                acc = JuMP.AffExpr(0.0)
                JuMP.add_to_expression!(acc, pre_flow[name, t])
                @inbounds for b in eachindex(ptdf_col)
                    coef = ptdf_col[b]
                    abs(coef) < PTDF_ZERO_TOL && continue
                    JuMP.add_to_expression!(acc, coef, post_cont_expr[b, t])
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
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    ::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {
    T <: PostContingencyGenerationBalanceConstraint,
    U <: PostContingencyActivePowerBalance,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    associated_outages = _service_outages(sys, service_model)
    expressions = get_expression(container, U(), R, service_name)
    constraint = add_constraints_container!(
        container,
        T(),
        R,
        [string(IS.get_uuid(o)) for o in associated_outages],
        time_steps;
        meta = service_name,
    )
    jump_model = get_jump_model(container)
    for outage in associated_outages, t in time_steps
        outage_id = string(IS.get_uuid(outage))
        constraint[outage_id, t] =
            JuMP.@constraint(jump_model, expressions[outage_id, t] == 0)
    end
    return
end

"""
Sparse-monitored post-contingency branch flow inequalities. The container is
keyed by `(outage_id::String, monitored_name::String, t::Int)` and only
entries resolved from `service_model.outages` are populated. Limits use the
monitored branch's emergency rating. Optional non-negative slacks relax the
inequalities at `POST_CONTINGENCY_CONSTRAINT_VIOLATION_SLACK_COST`.
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    service::R,
    service_model::ServiceModel{R, F},
    network_model::NetworkModel{<:AbstractPTDFModel},
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
    resolved = _resolve_service_monitored_arcs(service_model, net_reduction_data)

    con_lb = _add_service_post_contingency_sparse_constraints!(
        container, T, R, service_name; meta_suffix = "lb",
    )
    con_ub = _add_service_post_contingency_sparse_constraints!(
        container, T, R, service_name; meta_suffix = "ub",
    )
    isempty(resolved) && return

    post_cont_flow = get_expression(container, U(), R, service_name)
    jump_model = get_jump_model(container)

    use_slacks = get_use_slacks(service_model)
    slack_ub = if use_slacks
        add_post_contingency_slack_variables!(
            container,
            PostContingencyFlowActivePowerSlackUpperBound,
            service,
            service_name,
            resolved,
            F(),
        )
    else
        nothing
    end
    slack_lb = if use_slacks
        add_post_contingency_slack_variables!(
            container,
            PostContingencyFlowActivePowerSlackLowerBound,
            service,
            service_name,
            resolved,
            F(),
        )
    else
        nothing
    end

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
# AreaBalance network model: post-contingency AreaInterchange flow expression
# and rate-limit constraints. The expression is keyed by
# `(outage_id::String, area_interchange_name::String, t::Int)` and represents
# the from→to flow after the contingency:
#     post_flow = pre_flow + Σ_{g ∈ from} deploy_g - Σ_{g ∈ to} deploy_g
#                 - sign(outaged_side) * P_outaged
# where `sign(outaged_side)` is `+1` if the outaged generator sits in the
# from-area and `-1` if it sits in the to-area. Deployment variables for the
# outaged generator are pinned to zero, so iterating contributing devices is
# safe. Only the AreaInterchanges named in `service_model.outages[uuid]` are
# instantiated.
# ----------------------------------------------------------------------------

"""
Resolve every monitored `PSY.AreaInterchange` carried by
`service_model.outages` to its system component. Mirrors
`_resolve_service_monitored_arcs` but for the AreaBalance path where the
monitored object is an AreaInterchange (not a branch arc) and the only
information needed downstream is the component itself. Outages with no
monitored AreaInterchanges are skipped; the returned vector is sorted by
UUID for deterministic axes.
"""
function _resolve_service_monitored_area_interchanges(
    sys::PSY.System,
    service_model::ServiceModel,
)
    resolved = Pair{Base.UUID, Vector{Tuple{String, PSY.AreaInterchange}}}[]
    for (uuid, per_type) in get_outages(service_model)
        kept = Tuple{String, PSY.AreaInterchange}[]
        names = get(per_type, PSY.AreaInterchange, nothing)
        if names !== nothing
            for name in sort!(collect(names))
                comp = PSY.get_component(PSY.AreaInterchange, sys, name)
                comp === nothing && continue
                push!(kept, (name, comp))
            end
        end
        isempty(kept) && continue
        push!(resolved, uuid => kept)
    end
    sort!(resolved; by = first)
    return resolved
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
) where {
    T <: PostContingencyAreaInterchangeFlow,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    resolved = _resolve_service_monitored_area_interchanges(sys, service_model)

    contents = Dict{Tuple{String, String, Int}, JuMP.AffExpr}()
    for (uuid, entries) in resolved
        outage_id = string(uuid)
        for (name, _) in entries, t in time_steps
            contents[(outage_id, name, t)] = zero(JuMP.AffExpr)
        end
    end
    expression_container = SparseAxisArray(contents)
    _assign_container!(
        container.expressions,
        ExpressionKey(T, R, service_name),
        expression_container,
    )
    isempty(resolved) && return expression_container

    # Baseline flow variable for the AreaInterchange (from→to convention).
    flow_var = get_variable(container, FlowActivePowerVariable(), PSY.AreaInterchange)

    # Reserve-deployment variable keyed by (outage_id, gen_name, t).
    reserve_deployment_variable = get_variable(
        container,
        PostContingencyActivePowerReserveDeploymentVariable(),
        R,
        service_name,
    )
    contributing_devices = get_contributing_devices(service_model)

    # Pre-compute area assignment for each contributing device so we can
    # apply +1 for from-area and -1 for to-area without `isa` checks.
    device_areas = Dict{String, String}()
    for device in contributing_devices
        device_areas[PSY.get_name(device)] =
            PSY.get_name(PSY.get_area(PSY.get_bus(device)))
    end

    associated_outages = _service_outages(sys, service_model)
    outage_by_uuid = Dict(IS.get_uuid(o) => o for o in associated_outages)
    for (uuid, entries) in resolved
        outage = outage_by_uuid[uuid]
        outage_id = string(uuid)
        outaged_gens = PSY.get_associated_components(
            sys, outage; component_type = PSY.Generator,
        )
        for (name, area_interchange) in entries
            from_area = PSY.get_name(PSY.get_from_area(area_interchange))
            to_area = PSY.get_name(PSY.get_to_area(area_interchange))
            for t in time_steps
                expr = expression_container[outage_id, name, t]
                JuMP.add_to_expression!(expr, flow_var[name, t])
                for device in contributing_devices
                    gen_name = PSY.get_name(device)
                    gen_area = device_areas[gen_name]
                    coef = if gen_area == from_area
                        1.0
                    elseif gen_area == to_area
                        -1.0
                    else
                        0.0
                    end
                    coef == 0.0 && continue
                    JuMP.add_to_expression!(
                        expr,
                        coef,
                        reserve_deployment_variable[outage_id, gen_name, t],
                    )
                end
                # Subtract the outaged generation contribution on the
                # outaged side: +pre-contingency power if in from-area,
                # -pre-contingency power if in to-area.
                for outaged_gen in outaged_gens
                    outaged_area =
                        PSY.get_name(PSY.get_area(PSY.get_bus(outaged_gen)))
                    coef = if outaged_area == from_area
                        -1.0
                    elseif outaged_area == to_area
                        1.0
                    else
                        0.0
                    end
                    coef == 0.0 && continue
                    gen_var = get_variable(
                        container, ActivePowerVariable(), typeof(outaged_gen),
                    )
                    JuMP.add_to_expression!(
                        expr, coef, gen_var[PSY.get_name(outaged_gen), t],
                    )
                end
                expression_container[outage_id, name, t] = expr
            end
        end
    end
    return expression_container
end

"""
Per-(outage, area_interchange, t) flow-rate inequalities under the
AreaBalance network model. Limits come from
`PSY.get_flow_limits(area_interchange)` as `[-from_to, +to_from]`. Optional
non-negative slacks relax the inequalities at
`POST_CONTINGENCY_CONSTRAINT_VIOLATION_SLACK_COST`.
"""
function add_constraints!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:AreaBalancePowerModel},
) where {
    T <: PostContingencyFlowRateConstraint,
    U <: PostContingencyAreaInterchangeFlow,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    resolved = _resolve_service_monitored_area_interchanges(sys, service_model)

    con_lb = _add_service_post_contingency_sparse_constraints!(
        container, T, R, service_name; meta_suffix = "lb",
    )
    con_ub = _add_service_post_contingency_sparse_constraints!(
        container, T, R, service_name; meta_suffix = "ub",
    )
    isempty(resolved) && return

    post_cont_flow = get_expression(container, U(), R, service_name)
    jump_model = get_jump_model(container)

    use_slacks = get_use_slacks(service_model)
    # Adapt the existing sparse-slack helper: it expects the PTDF-style
    # resolved tuple shape `(type, name, arc, reduction_kind)`. Build an
    # equivalent shape on the fly with placeholder arc/reduction values so
    # the slack containers are keyed by the same `(outage_id, name, t)`.
    slack_resolved =
        Pair{Base.UUID, Vector{Tuple{DataType, String, Tuple{Int, Int}, String}}}[
            uuid => Tuple{DataType, String, Tuple{Int, Int}, String}[
                (PSY.AreaInterchange, name, (0, 0), "") for (name, _) in entries
            ] for (uuid, entries) in resolved
        ]
    slack_ub = if use_slacks
        add_post_contingency_slack_variables!(
            container,
            PostContingencyFlowActivePowerSlackUpperBound,
            service,
            service_name,
            slack_resolved,
            F(),
        )
    else
        nothing
    end
    slack_lb = if use_slacks
        add_post_contingency_slack_variables!(
            container,
            PostContingencyFlowActivePowerSlackLowerBound,
            service,
            service_name,
            slack_resolved,
            F(),
        )
    else
        nothing
    end

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
pre-contingency reserve variable. Outaged generators are pinned to zero.
"""
function add_constraints!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{X},
    ::Type{U},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractPowerModel},
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
    associated_outages = _service_outages(sys, service_model)
    constraint = add_constraints_container!(
        container,
        T(),
        R,
        [string(IS.get_uuid(o)) for o in associated_outages],
        [PSY.get_name(d) for d in contributing_devices],
        time_steps;
        meta = service_name,
    )
    variable = get_variable(container, X(), R, service_name)
    variable_outage = get_variable(container, U(), R, service_name)
    jump_model = get_jump_model(container)
    for outage in associated_outages
        associated_devices = PSY.get_associated_components(
            sys, outage; component_type = PSY.Generator,
        )
        outage_id = string(IS.get_uuid(outage))
        for device in contributing_devices
            name = PSY.get_name(device)
            gen_outaged = device in associated_devices
            for t in time_steps
                if gen_outaged
                    constraint[outage_id, name, t] = JuMP.@constraint(
                        jump_model,
                        variable_outage[outage_id, name, t] == 0.0,
                    )
                    continue
                end
                constraint[outage_id, name, t] = JuMP.@constraint(
                    jump_model,
                    variable_outage[outage_id, name, t] <= variable[name, t],
                )
            end
        end
    end
    return
end

"""
Per-(outage, generator, t) min/max bounds on the
`PostContingencyActivePowerGeneration` expression. Used when the service
has no reserve requirement time series.
"""
function add_constraints!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    contributing_devices::Union{IS.FlattenIteratorWrapper{V}, Vector{V}},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:PM.AbstractActivePowerModel},
) where {
    T <: PostContingencyActivePowerGenerationLimitsConstraint,
    V <: PSY.Generator,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    associated_outages = _service_outages(sys, service_model)
    con_lb = add_constraints_container!(
        container,
        T(),
        R,
        string.(IS.get_uuid.(associated_outages)),
        PSY.get_name.(contributing_devices),
        time_steps;
        meta = "$(service_name)_lb",
    )
    con_ub = add_constraints_container!(
        container,
        T(),
        R,
        string.(IS.get_uuid.(associated_outages)),
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
        for outage in associated_outages
            associated_devices = PSY.get_associated_components(
                sys, outage; component_type = PSY.Generator,
            )
            outage_id = string(IS.get_uuid(outage))
            gen_outaged = device in associated_devices
            for t in time_steps
                if gen_outaged
                    con_ub[outage_id, name, t] = JuMP.@constraint(
                        jump_model, expressions[outage_id, name, t] == 0.0,
                    )
                    con_lb[outage_id, name, t] = JuMP.@constraint(
                        jump_model, expressions[outage_id, name, t] == 0.0,
                    )
                    continue
                end
                con_ub[outage_id, name, t] = JuMP.@constraint(
                    jump_model, expressions[outage_id, name, t] <= limits.max,
                )
                con_lb[outage_id, name, t] = JuMP.@constraint(
                    jump_model, expressions[outage_id, name, t] >= limits.min,
                )
            end
        end
    end
    return
end

"""
Per-(outage, area, t) area balance for the `AreaBalancePowerModel`: the
post-contingency area-deployment expression plus the pre-contingency area
`ActivePowerBalance` must close to zero.
"""
function add_constraints!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::Type{T},
    ::Type{U},
    ::Type{Y},
    service::R,
    service_model::ServiceModel{R, F},
    ::NetworkModel{<:AreaBalancePowerModel},
) where {
    T <: PostContingencyCopperPlateBalanceConstraint,
    U <: PostContingencyAreaActivePowerDeployment,
    Y <: ActivePowerBalance,
    R <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    time_steps = get_time_steps(container)
    devices = PSY.get_components(PSY.Area, sys)
    area_names = PSY.get_name.(devices)
    service_name = PSY.get_name(service)
    associated_outages = _service_outages(sys, service_model)
    con = add_constraints_container!(
        container,
        T(),
        R,
        string.(IS.get_uuid.(associated_outages)),
        area_names,
        time_steps;
        meta = service_name,
    )
    contingency_expression = get_expression(container, U(), R, service_name)
    area_expression = get_expression(container, Y(), PSY.Area)
    jump_model = get_jump_model(container)
    for outage in associated_outages
        outage_id = string(IS.get_uuid(outage))
        for area in devices
            area_name = PSY.get_name(area)
            for t in time_steps
                con[outage_id, area_name, t] = JuMP.@constraint(
                    jump_model,
                    contingency_expression[outage_id, area_name, t] +
                    area_expression[area_name, t] == 0.0,
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

# Shared ArgumentConstructStage helper used by both formulations: builds
# pre-contingency reserve variable + post-contingency deployment variable.
function _construct_service_arguments_sc!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
    devices_template::Dict{Symbol, DeviceModel},
    require_ts::Bool,
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return false
    contributing_devices = get_contributing_devices(model)

    has_requirement_ts =
        haskey(get_time_series_names(model), RequirementTimeSeriesParameter) &&
        length(PSY.get_time_series_keys(service)) > 0
    if has_requirement_ts || require_ts
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

    associated_outages = _service_outages(sys, model)
    if isempty(associated_outages)
        @warn "Service $(SR)('$name'): `service_model.outages` is empty; the \
               security-constrained formulation $(F) will not add any \
               post-contingency variables or constraints."
        return false
    end

    add_variables!(
        container,
        sys,
        PostContingencyActivePowerReserveDeploymentVariable,
        service,
        model,
        contributing_devices,
        F(),
    )
    return true
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, SecurityConstrainedContingencyReserve},
    devices_template::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractActivePowerModel},
) where {SR <: PSY.AbstractReserve}
    _construct_service_arguments_sc!(container, sys, model, devices_template, false)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ArgumentConstructStage,
    model::ServiceModel{SR, SecurityConstrainedRampReserve},
    devices_template::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    ::NetworkModel{<:PM.AbstractActivePowerModel},
) where {SR <: PSY.AbstractReserve}
    # Ramp reserve formulation always requires the requirement time series.
    _construct_service_arguments_sc!(container, sys, model, devices_template, true)
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
    include_ramp::Bool,
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
        include_ramp && add_constraints!(
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
# expression/constraint stack that's common to every network model.
function _construct_service_post_contingency_balance!(
    container::OptimizationContainer,
    sys::PSY.System,
    service::PSY.AbstractReserve,
    contributing_devices,
    model::ServiceModel{SR, F},
    network_model::NetworkModel,
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    add_to_expression!(
        container, sys, PostContingencyActivePowerBalance,
        PostContingencyActivePowerReserveDeploymentVariable,
        contributing_devices, service, model, network_model,
    )
    attribute_device_map = PSY.get_component_supplemental_attribute_pairs(
        PSY.Generator, PSY.Outage, sys,
    )
    add_to_expression!(
        container, sys, PostContingencyActivePowerBalance, ActivePowerVariable,
        attribute_device_map, service, model, network_model,
    )
    add_constraints!(
        container, sys, PostContingencyGenerationBalanceConstraint,
        PostContingencyActivePowerBalance,
        contributing_devices, service, model, network_model,
    )
    return attribute_device_map
end

# ----- PTDF (DC) network model -----

function _construct_service_model_ptdf!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
    network_model::NetworkModel{<:PM.AbstractDCPModel},
    has_requirement_ts_default::Bool,
    include_ramp::Bool,
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    has_requirement_ts =
        has_requirement_ts_default || (
            haskey(get_time_series_names(model), RequirementTimeSeriesParameter) &&
            length(PSY.get_time_series_keys(service)) > 0
        )
    _construct_service_pre_contingency!(
        container, sys, service, contributing_devices, model,
        has_requirement_ts, include_ramp,
    )

    associated_outages = _service_outages(sys, model)
    isempty(associated_outages) && return

    attribute_device_map = _construct_service_post_contingency_balance!(
        container, sys, service, contributing_devices, model, network_model,
    )
    add_to_expression!(
        container, sys, PostContingencyNodalActivePowerDeployment,
        PostContingencyActivePowerReserveDeploymentVariable,
        contributing_devices, service, model, network_model,
    )
    add_to_expression!(
        container, sys, PostContingencyNodalActivePowerDeployment, ActivePowerVariable,
        attribute_device_map, service, model, network_model,
    )
    add_post_contingency_flow_expressions!(
        container, PostContingencyBranchFlow, service, model, network_model,
    )
    add_constraints!(
        container, PostContingencyFlowRateConstraint, PostContingencyBranchFlow,
        service, model, network_model,
    )

    if has_requirement_ts
        add_constraints!(
            container, sys,
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            ActivePowerReserveVariable,
            PostContingencyActivePowerReserveDeploymentVariable,
            contributing_devices, service, model, network_model,
        )
    else
        add_to_expression!(
            container, sys, PostContingencyActivePowerGeneration,
            contributing_devices, service, model, network_model,
        )
        add_constraints!(
            container, sys, PostContingencyActivePowerGenerationLimitsConstraint,
            contributing_devices, service, model, network_model,
        )
    end
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, SecurityConstrainedContingencyReserve},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:PM.AbstractDCPModel},
) where {SR <: PSY.AbstractReserve}
    _construct_service_model_ptdf!(container, sys, model, network_model, false, true)
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, SecurityConstrainedRampReserve},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:PM.AbstractDCPModel},
) where {SR <: PSY.AbstractReserve}
    _construct_service_model_ptdf!(container, sys, model, network_model, true, true)
    return
end

# ----- CopperPlate network model -----

function _construct_service_model_copperplate!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
    network_model::NetworkModel{<:CopperPlatePowerModel},
    has_requirement_ts_default::Bool,
    include_ramp::Bool,
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    has_requirement_ts =
        has_requirement_ts_default || (
            haskey(get_time_series_names(model), RequirementTimeSeriesParameter) &&
            length(PSY.get_time_series_keys(service)) > 0
        )
    _construct_service_pre_contingency!(
        container, sys, service, contributing_devices, model,
        has_requirement_ts, include_ramp,
    )

    associated_outages = _service_outages(sys, model)
    isempty(associated_outages) && return

    _construct_service_post_contingency_balance!(
        container, sys, service, contributing_devices, model, network_model,
    )

    if has_requirement_ts
        add_constraints!(
            container, sys,
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            ActivePowerReserveVariable,
            PostContingencyActivePowerReserveDeploymentVariable,
            contributing_devices, service, model, network_model,
        )
    else
        add_to_expression!(
            container, sys, PostContingencyActivePowerGeneration,
            contributing_devices, service, model, network_model,
        )
        add_constraints!(
            container, sys, PostContingencyActivePowerGenerationLimitsConstraint,
            contributing_devices, service, model, network_model,
        )
    end
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, SecurityConstrainedContingencyReserve},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:CopperPlatePowerModel},
) where {SR <: PSY.AbstractReserve}
    _construct_service_model_copperplate!(
        container, sys, model, network_model, false, true,
    )
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, SecurityConstrainedRampReserve},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:CopperPlatePowerModel},
) where {SR <: PSY.AbstractReserve}
    _construct_service_model_copperplate!(
        container, sys, model, network_model, true, true,
    )
    return
end

# ----- AreaBalance network model -----

function _construct_service_model_areabalance!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{SR, F},
    network_model::NetworkModel{<:AreaBalancePowerModel},
    has_requirement_ts_default::Bool,
    include_ramp::Bool,
) where {
    SR <: PSY.AbstractReserve,
    F <: AbstractSecurityConstrainedReservesFormulation,
}
    name = get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    !PSY.get_available(service) && return
    contributing_devices = get_contributing_devices(model)

    has_requirement_ts =
        has_requirement_ts_default || (
            haskey(get_time_series_names(model), RequirementTimeSeriesParameter) &&
            length(PSY.get_time_series_keys(service)) > 0
        )
    _construct_service_pre_contingency!(
        container, sys, service, contributing_devices, model,
        has_requirement_ts, include_ramp,
    )

    associated_outages = _service_outages(sys, model)
    isempty(associated_outages) && return

    _construct_service_post_contingency_balance!(
        container, sys, service, contributing_devices, model, network_model,
    )
    add_to_expression!(
        container, sys, PostContingencyAreaActivePowerDeployment,
        PostContingencyActivePowerReserveDeploymentVariable,
        contributing_devices, service, model, network_model,
    )
    add_to_expression!(
        container, sys, PostContingencyAreaActivePowerDeployment, ActivePowerVariable,
        service, model, network_model,
    )
    add_constraints!(
        container, sys, PostContingencyCopperPlateBalanceConstraint,
        PostContingencyAreaActivePowerDeployment, ActivePowerBalance,
        service, model, network_model,
    )
    add_post_contingency_flow_expressions!(
        container, sys, PostContingencyAreaInterchangeFlow,
        service, model, network_model,
    )
    add_constraints!(
        container, sys, PostContingencyFlowRateConstraint,
        PostContingencyAreaInterchangeFlow,
        service, model, network_model,
    )

    if has_requirement_ts
        add_constraints!(
            container, sys,
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            ActivePowerReserveVariable,
            PostContingencyActivePowerReserveDeploymentVariable,
            contributing_devices, service, model, network_model,
        )
    else
        add_to_expression!(
            container, sys, PostContingencyActivePowerGeneration,
            contributing_devices, service, model, network_model,
        )
        add_constraints!(
            container, sys, PostContingencyActivePowerGenerationLimitsConstraint,
            contributing_devices, service, model, network_model,
        )
    end
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, SecurityConstrainedContingencyReserve},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:AreaBalancePowerModel},
) where {SR <: PSY.AbstractReserve}
    _construct_service_model_areabalance!(
        container, sys, model, network_model, false, true,
    )
    return
end

function construct_service!(
    container::OptimizationContainer,
    sys::PSY.System,
    ::ModelConstructStage,
    model::ServiceModel{SR, SecurityConstrainedRampReserve},
    ::Dict{Symbol, DeviceModel},
    ::Set{<:DataType},
    network_model::NetworkModel{<:AreaBalancePowerModel},
) where {SR <: PSY.AbstractReserve}
    _construct_service_model_areabalance!(
        container, sys, model, network_model, true, true,
    )
    return
end
