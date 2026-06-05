"""
Add variables to the OptimizationContainer for any component.
"""
function add_variables!(
    container::OptimizationContainer,
    ::Type{T},
    devices::Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    formulation::Union{AbstractServiceFormulation, AbstractDeviceFormulation},
) where {T <: VariableType, U <: PSY.Component}
    add_variable!(container, T(), devices, formulation)
    return
end

#=
# TODO: Contingency. Is this needed?
"""
Add Contingency-related variables to the OptimizationContainer for any component.
"""
function add_variables!(
    container::OptimizationContainer,
    ::Type{T},
    devices::Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    devices_outages::Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    formulation::Union{AbstractServiceFormulation, AbstractDeviceFormulation},
) where {T <: VariableType, U <: PSY.Component}
    add_variable!(container, T(), devices, devices_outages, formulation)
    return
end
=#

"""
Add variables to the OptimizationContainer for a service.
"""
function add_variables!(
    container::OptimizationContainer,
    ::Type{T},
    service::U,
    contributing_devices::Union{Vector{V}, IS.FlattenIteratorWrapper{V}},
    formulation::AbstractReservesFormulation,
) where {T <: VariableType, U <: PSY.AbstractReserve, V <: PSY.Component}
    # PERF: compilation hotspot. Switch to TSC.
    add_service_variable!(container, T(), service, contributing_devices, formulation)
    return
end

@doc raw"""
Adds a variable to the optimization model and to the affine expressions contained
in the optimization_container model according to the specified sign. Based on the inputs, the variable can
be specified as binary.

# Bounds

``` lb_value_function <= varstart[name, t] <= ub_value_function ```

If binary = true:

``` varstart[name, t] in {0,1} ```

# LaTeX

``  lb \ge x^{device}_t \le ub \forall t ``

``  x^{device}_t \in {0,1} \forall t iff \text{binary = true}``

# Arguments
* container::OptimizationContainer : the optimization_container model built in PowerSimulations
* devices : Vector or Iterator with the devices
* var_key::VariableKey : Base Name for the variable
* binary::Bool : Select if the variable is binary
* expression_name::Symbol : Expression_name name stored in container.expressions to add the variable
* sign::Float64 : sign of the addition of the variable to the expression_name. Default Value is 1.0

# Accepted Keyword Arguments
* ub_value : Provides the function over device to obtain the value for a upper_bound
* lb_value : Provides the function over device to obtain the value for a lower_bound. If the variable is meant to be positive define lb = x -> 0.0
* initial_value : Provides the function over device to obtain the warm start value

"""
function add_variable!(
    container::OptimizationContainer,
    variable_type::T,
    devices::U,
    formulation,
) where {
    T <: VariableType,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Component}
    @assert !isempty(devices)
    time_steps = get_time_steps(container)
    settings = get_settings(container)
    binary = get_variable_binary(variable_type, D, formulation)

    variable = add_variable_container!(
        container,
        variable_type,
        D,
        PSY.get_name.(devices),
        time_steps,
    )

    for t in time_steps, d in devices
        name = PSY.get_name(d)
        variable[name, t] = JuMP.@variable(
            get_jump_model(container),
            base_name = "$(T)_$(D)_{$(name), $(t)}",
            binary = binary
        )
        ub = get_variable_upper_bound(variable_type, d, formulation)
        !isnothing(ub) && JuMP.set_upper_bound(variable[name, t], ub)

        lb = get_variable_lower_bound(variable_type, d, formulation)
        !isnothing(lb) && JuMP.set_lower_bound(variable[name, t], lb)

        if get_warm_start(settings)
            init = get_variable_warm_start_value(variable_type, d, formulation)
            !isnothing(init) && JuMP.set_start_value(variable[name, t], init)
        end
    end

    return
end

function add_service_variable!(
    container::OptimizationContainer,
    variable_type::T,
    service::U,
    contributing_devices::V,
    formulation::AbstractServiceFormulation,
) where {
    T <: VariableType,
    U <: PSY.Service,
    V <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Component}
    @assert !isempty(contributing_devices)
    time_steps = get_time_steps(container)

    binary = get_variable_binary(variable_type, U, formulation)

    variable = add_variable_container!(
        container,
        variable_type,
        U,
        PSY.get_name(service),
        [PSY.get_name(d) for d in contributing_devices],
        time_steps,
    )

    for t in time_steps, d in contributing_devices
        name = PSY.get_name(d)
        variable[name, t] = JuMP.@variable(
            get_jump_model(container),
            base_name = "$(T)_$(U)_$(PSY.get_name(service))_{$(name), $(t)}",
            binary = binary
        )

        ub = get_variable_upper_bound(variable_type, service, d, formulation)
        !isnothing(ub) && JuMP.set_upper_bound(variable[name, t], ub)

        lb = get_variable_lower_bound(variable_type, service, d, formulation)
        !isnothing(lb) && !binary && JuMP.set_lower_bound(variable[name, t], lb)

        init = get_variable_warm_start_value(variable_type, d, formulation)
        !isnothing(init) && JuMP.set_start_value(variable[name, t], init)
    end

    return
end
