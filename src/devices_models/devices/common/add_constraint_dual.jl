function add_constraint_dual!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::DeviceModel{T, D},
) where {T <: PSY.Component, D <: AbstractDeviceFormulation}
    if !isempty(get_duals(model))
        devices = get_available_components(model, sys)
        for constraint_type in get_duals(model)
            assign_dual_variable!(container, constraint_type, devices, D)
        end
    end
    return
end

function add_constraint_dual!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::NetworkModel{T},
) where {T <: PM.AbstractPowerModel}
    if !isempty(get_duals(model))
        devices = get_available_components(model, PSY.ACBus, sys)
        for constraint_type in get_duals(model)
            assign_dual_variable!(container, constraint_type, devices, model)
        end
    end
    return
end

function add_constraint_dual!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::NetworkModel{T},
) where {T <: Union{CopperPlatePowerModel, AbstractPTDFModel}}
    if !isempty(get_duals(model))
        for constraint_type in get_duals(model)
            assign_dual_variable!(container, constraint_type, sys, model)
        end
    end
    return
end

function add_constraint_dual!(
    container::OptimizationContainer,
    sys::PSY.System,
    model::ServiceModel{T, D},
) where {T <: PSY.Service, D <: AbstractServiceFormulation}
    if !isempty(get_duals(model))
        service = get_available_components(model, sys)
        for constraint_type in get_duals(model)
            assign_dual_variable!(container, constraint_type, service, D)
        end
    end
    return
end

function assign_dual_variable!(
    container::OptimizationContainer,
    constraint_type::Type{<:ConstraintType},
    service::D,
    ::Type{<:AbstractServiceFormulation},
) where {D <: PSY.Service}
    time_steps = get_time_steps(container)
    service_name = PSY.get_name(service)
    add_dual_container!(
        container,
        constraint_type,
        D,
        [service_name],
        time_steps;
        meta = service_name,
    )
    return
end

function assign_dual_variable!(
    container::OptimizationContainer,
    constraint_type::Type{<:ConstraintType},
    devices::U,
    ::Type{<:AbstractDeviceFormulation},
) where {U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}}} where {D <: PSY.Device}
    @assert !isempty(devices)
    time_steps = get_time_steps(container)
    metas = _existing_constraint_metas(container, constraint_type, D)
    if isempty(metas)
        device_names = PSY.get_name.(devices)
        add_dual_container!(container, constraint_type, D, device_names, time_steps)
    else
        # Reuse the existing constraint container's row axis so the dual axis
        # matches the constraint exactly. Network reductions (radial /
        # degree-two) drop branches that pass the device-model filter, so the
        # constraint axis is a strict subset of PSY.get_name.(devices). Sizing
        # the dual from the device list would leave the dual broadcast in
        # process_duals incompatible with the constraint matrix.
        for meta in metas
            existing =
                get_constraint(container, ConstraintKey(constraint_type, D, meta))
            row_axis = axes(existing)[1]
            add_dual_container!(
                container,
                constraint_type,
                D,
                row_axis,
                time_steps;
                meta = meta,
            )
        end
    end
    return
end

function _existing_constraint_metas(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{D},
) where {T <: ConstraintType, D}
    metas = String[]
    for key in get_constraint_keys(container)
        if IS.Optimization.get_entry_type(key) === T &&
           IS.Optimization.get_component_type(key) === D
            push!(metas, key.meta)
        end
    end
    return metas
end

function assign_dual_variable!(
    container::OptimizationContainer,
    constraint_type::Type{<:ConstraintType},
    devices::U,
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}}} where {D <: PSY.ACBus}
    @assert !isempty(devices)
    time_steps = get_time_steps(container)
    add_dual_container!(
        container,
        constraint_type,
        D,
        PSY.get_name.(devices),
        time_steps,
    )
    return
end

function assign_dual_variable!(
    container::OptimizationContainer,
    constraint_type::Type{CopperPlateBalanceConstraint},
    ::U,
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {U <: PSY.System}
    time_steps = get_time_steps(container)
    ref_buses = get_reference_buses(network_model)
    add_dual_container!(container, constraint_type, U, ref_buses, time_steps)
    return
end
