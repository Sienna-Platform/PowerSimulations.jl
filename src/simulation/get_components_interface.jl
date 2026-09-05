# Analogous to `src/get_components_interface.jl` in PowerSystems.jl, see comments there.

# get_components
"""
Calling `get_components` on a `Results` is the same as calling
[`get_available_components`] on the system attached to the results.
"""
PSY.get_components(
    ::Type{T},
    res::IS.Outputs;
    subsystem_name = nothing,
) where {T <: IS.InfrastructureSystemsComponent} =
    IS.get_components(T, res; subsystem_name = subsystem_name)

PSY.get_components(res::IS.Outputs, attribute::IS.SupplementalAttribute) =
    IS.get_components(res, attribute)

PSY.get_components(
    filter_func::Function,
    ::Type{T},
    res::IS.Outputs;
    subsystem_name = nothing,
) where {T <: IS.InfrastructureSystemsComponent} =
    IS.get_components(filter_func, T, res; subsystem_name = subsystem_name)

PSY.get_components(
    scope_limiter::Union{Function, Nothing},
    selector::IS.ComponentSelector,
    res::IS.Outputs,
) =
    IS.get_components(scope_limiter, selector, res)

PSY.get_components(selector::IS.ComponentSelector, res::IS.Outputs) =
    IS.get_components(selector, res)

# get_component
"""
Calling `get_component` on a `Results` is the same as calling
[`get_available_component`] on the system attached to the results.
"""
PSY.get_component(res::IS.Outputs, uuid::Base.UUID) = IS.get_component(res, uuid)
PSY.get_component(res::IS.Outputs, uuid::String) = IS.get_component(res, uuid)

PSY.get_component(
    ::Type{T},
    res::IS.Outputs,
    name::AbstractString,
) where {T <: IS.InfrastructureSystemsComponent} =
    IS.get_component(T, res, name)

PSY.get_component(
    scope_limiter::Union{Function, Nothing},
    selector::IS.SingularComponentSelector,
    res::IS.Outputs,
) =
    IS.get_component(scope_limiter, selector, res)

PSY.get_component(selector::IS.SingularComponentSelector, res::IS.Outputs) =
    IS.get_component(selector, res)

# get_groups
"""
Calling `get_groups` on a `Results` is the same as calling [`get_available_groups`] on
the system attached to the results.
"""
PSY.get_groups(
    scope_limiter::Union{Function, Nothing},
    selector::IS.ComponentSelector,
    res::IS.Outputs,
) =
    IS.get_groups(scope_limiter, selector, res)

PSY.get_groups(selector::IS.ComponentSelector, res::IS.Outputs) =
    IS.get_groups(selector, res)
