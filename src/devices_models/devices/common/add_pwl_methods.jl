"""
    _get_breakpoints_for_pwl_function(min_val, max_val, f; num_segments = DEFAULT_INTERPOLATION_LENGTH)

Generate breakpoints for piecewise linear (PWL) approximation of a nonlinear function.

This function creates equally-spaced breakpoints over the specified domain [min_val, max_val]
and evaluates the given function at each breakpoint to construct a piecewise linear approximation.
The breakpoints are used in optimization problems to linearize nonlinear constraints or objectives.

# Arguments
- `min_val::Float64`: Minimum value of the domain for the PWL approximation
- `max_val::Float64`: Maximum value of the domain for the PWL approximation  
- `f`: Function to be approximated (must be callable with Float64 input)
- `num_segments::Int`: Number of linear segments in the PWL approximation (default: DEFAULT_INTERPOLATION_LENGTH)

# Returns
- `Tuple{Vector{Float64}, Vector{Float64}}`: A tuple containing:
  - `x_bkpts`: Vector of x-coordinates (breakpoints) in the domain
  - `y_bkpts`: Vector of y-coordinates (function values at breakpoints)

# Notes
- The number of breakpoints is `num_segments + 1`
- Breakpoints are equally spaced across the domain
- The first breakpoint is always at `min_val` and the last at `max_val`
"""
function _get_breakpoints_for_pwl_function(
    min_val::Float64,
    max_val::Float64,
    f;
    num_segments = DEFAULT_INTERPOLATION_LENGTH,
)
    # Calculate total number of breakpoints (one more than segments)
    # num_segments is the number of linear segments in the PWL approximation
    # num_bkpts is the total number of breakpoints needed for the segments
    num_bkpts = num_segments + 1

    # Calculate step size for equally-spaced breakpoints
    step = (max_val - min_val) / num_segments

    # Pre-allocate vectors for breakpoint coordinates
    x_bkpts = Vector{Float64}(undef, num_bkpts)  # Domain values (x-coordinates)
    y_bkpts = Vector{Float64}(undef, num_bkpts)  # Function values (y-coordinates)

    # Set the first breakpoint at the minimum domain value
    x_bkpts[1] = min_val
    y_bkpts[1] = f(min_val)

    # Generate remaining breakpoints by stepping through the domain
    for i in 1:num_segments
        x_val = min_val + step * i  # Calculate x-coordinate of current breakpoint
        x_bkpts[i + 1] = x_val
        y_bkpts[i + 1] = f(x_val)  # Evaluate function at current breakpoint
    end
    return x_bkpts, y_bkpts
end

"""
    add_sparse_pwl_interpolation_variables!(container, devices, ::T, model, num_segments = DEFAULT_INTERPOLATION_LENGTH)

Add piecewise linear interpolation variables to an optimization container.

This function creates the necessary variables for piecewise linear (PWL) approximation in optimization models.
It adds either continuous interpolation variables (δ) or binary interpolation variables (z) depending on the
variable type `T`. These variables are used in the incremental method for PWL approximation where:

- **Interpolation variables (δ)**: Continuous variables ∈ [0,1] that represent weights for each segment
- **Binary interpolation variables (z)**: Binary variables that enforce ordering constraints in incremental method

The function creates a 3-dimensional variable structure indexed by (device_name, segment_index, time_step).
For binary variables, the number of variables is one less than for continuous variables since they control
transitions between segments.

# Arguments
- `container::OptimizationContainer`: The optimization container to add variables to
- `devices`: Collection of devices for which to create PWL variables
- `::T`: Type parameter specifying the variable type (InterpolationVariableType or BinaryInterpolationVariableType)
- `model::DeviceModel{U, V}`: Device model containing formulation information for bounds
- `num_segments::Int`: Number of linear segments in the PWL approximation (default: DEFAULT_INTERPOLATION_LENGTH)

# Type Parameters
- `T <: Union{InterpolationVariableType, BinaryInterpolationVariableType}`: Variable type to create
- `U <: PSY.Component`: Component type for devices
- `V <: AbstractDeviceFormulation`: Device formulation type for bounds

# Notes
- Binary variables have `num_segments - 1` variables (control transitions between segments)
- Continuous variables have `num_segments` variables (one per segment)
- Variable bounds are set based on the device formulation if available
- Variables are created for all devices and time steps in the optimization horizon

# See Also
- `_add_generic_incremental_interpolation_constraint!`: Function that uses these variables in constraints
"""
function add_sparse_pwl_interpolation_variables!(
    container::OptimizationContainer,
    ::T,
    devices,
    model::DeviceModel{U, V},
    num_segments = DEFAULT_INTERPOLATION_LENGTH,
) where {
    T <: Union{InterpolationVariableType, BinaryInterpolationVariableType},
    U <: PSY.Component,
    V <: AbstractDeviceFormulation,
}
    # TODO: Implement approach for deciding segment length
    # Extract time steps from the optimization container
    time_steps = get_time_steps(container)

    # Create variable container using lazy initialization
    var_container = lazy_container_addition!(container, T(), U)
    # Determine if this variable type should be binary based on type, component, and formulation
    binary_flag = get_variable_binary(T(), U, V())
    # Calculate number of segments based on variable type:
    # - Binary variables: (num_segments - 1) to control transitions between segments
    # - Continuous variables: num_segments (one per segment)
    len_segs = binary_flag ? (num_segments - 1) : num_segments

    # Iterate over all devices to create PWL variables
    for d in devices
        name = PSY.get_name(d)
        # Create variables for each time step
        for t in time_steps
            # Pre-allocate array to store variable references for this device and time step
            pwlvars = Array{JuMP.VariableRef}(undef, len_segs)

            # Create individual PWL variables for each segment
            for i in 1:len_segs
                # Create JuMP variable with descriptive name and store in both arrays
                pwlvars[i] =
                    var_container[(name, i, t)] = JuMP.@variable(
                        get_jump_model(container),
                        base_name = "$(T)_$(name)_{pwl_$(i), $(t)}",  # Descriptive variable name
                        binary = binary_flag  # Set as binary if this is a binary variable type
                    )

                # Set upper bound if specified by the device formulation
                ub = get_variable_upper_bound(T(), d, V())
                !isnothing(ub) && JuMP.set_upper_bound(var_container[name, i, t], ub)

                # Set lower bound if specified by the device formulation  
                lb = get_variable_lower_bound(T(), d, V())
                !isnothing(lb) && JuMP.set_lower_bound(var_container[name, i, t], lb)
            end
        end
    end
    return
end

"""
    _add_generic_incremental_interpolation_constraint!(container, ::R, ::S, ::T, ::U, ::V, devices, dic_var_bkpts, dic_function_bkpts; meta)

Add incremental piecewise linear interpolation constraints to an optimization container.

This function implements the incremental method for piecewise linear approximation in optimization models.
It creates constraints that relate the original variable (x) to its piecewise linear approximation (y = f(x))
using interpolation variables (δ) and binary variables (z) to ensure proper ordering.

The incremental method represents each segment of the PWL function as:
- x = x₁ + Σᵢ δᵢ(xᵢ₊₁ - xᵢ) where δᵢ ∈ [0,1]
- y = y₁ + Σᵢ δᵢ(yᵢ₊₁ - yᵢ) where yᵢ = f(xᵢ)

Binary variables z ensure the incremental property: δᵢ₊₁ ≤ zᵢ ≤ δᵢ for adjacent segments.

# Arguments
- `container::OptimizationContainer`: The optimization container to add constraints to
- `::R`: Type parameter for the original variable (x)
- `::S`: Type parameter for the approximated variable (y = f(x))  
- `::T`: Type parameter for the interpolation variables (δ)
- `::U`: Type parameter for the binary interpolation variables (z)
- `::V`: Type parameter for the constraint type
- `devices::IS.FlattenIteratorWrapper{W}`: Collection of devices to apply constraints to
- `dic_var_bkpts::Dict{String, Vector{Float64}}`: Breakpoints in the domain (x-coordinates) for each device
- `dic_function_bkpts::Dict{String, Vector{Float64}}`: Function values at breakpoints (y-coordinates) for each device
- `meta`: Metadata for constraint naming (default: empty)

# Type Parameters
- `R <: VariableType`: Original variable type
- `S <: VariableType`: Approximated variable type  
- `T <: VariableType`: Interpolation variable type
- `U <: VariableType`: Binary interpolation variable type
- `V <: ConstraintType`: Constraint type
- `W <: PSY.Component`: Component type for devices

# Notes
- Creates two types of constraints: variable interpolation and function interpolation
- Adds ordering constraints for binary variables to ensure incremental property
- All constraints are applied for each device and time step
"""
function _add_generic_incremental_interpolation_constraint!(
    container::OptimizationContainer,
    ::R, # original var : x
    ::S, # approximated var : y = f(x)
    ::T, # interpolation var : δ
    ::U, # binary interpolation var : z
    ::V, # constraint
    devices::IS.FlattenIteratorWrapper{W},
    dic_var_bkpts::Dict{String, Vector{Float64}},
    dic_function_bkpts::Dict{String, Vector{Float64}};
    meta = IS.Optimization.CONTAINER_KEY_EMPTY_META,
) where {
    R <: VariableType,
    S <: VariableType,
    T <: VariableType,
    U <: VariableType,
    V <: ConstraintType,
    W <: PSY.Component,
}
    # Extract time steps and device names for constraint indexing
    time_steps = get_time_steps(container)
    names = [PSY.get_name(d) for d in devices]
    JuMPmodel = get_jump_model(container)

    # Retrieve all required variables from the optimization container
    # Retrieve original variable for DCVoltage from the Bus
    x_var = if (R <: DCVoltage)
        get_variable(container, R(), PSY.DCBus)  # Original variable (domain of function)
    else
        get_variable(container, R(), W)  # Original variable (domain of function)
    end  # Original variable (domain of function)
    y_var = get_variable(container, S(), W)  # Approximated variable (range of function)
    δ_var = get_variable(container, T(), W)  # Interpolation variables (weights for segments)
    z_var = get_variable(container, U(), W)  # Binary variables (ordering constraints)

    # Create containers for the two main constraint types
    # Container for variable interpolation constraints: x = x₁ + Σᵢ δᵢ(xᵢ₊₁ - xᵢ)
    const_container_var = add_constraints_container!(
        container,
        V(),
        W,
        names,
        time_steps;
        meta = "$(meta)pwl_variable",
    )

    # Container for function interpolation constraints: y = y₁ + Σᵢ δᵢ(yᵢ₊₁ - yᵢ)
    const_container_function = add_constraints_container!(
        container,
        V(),
        W,
        names,
        time_steps;
        meta = "$(meta)pwl_function",
    )

    # Iterate over all devices to add constraints for each device and time step
    for d in devices
        name = PSY.get_name(d)
        bus_name = PSY.get_name(PSY.get_dc_bus(d))
        # Get proper name for x variable (if is DCVoltage or not)
        x_name = (R <: DCVoltage) ? bus_name : name
        var_bkpts = dic_var_bkpts[name]        # Breakpoints in domain (x-values)
        function_bkpts = dic_function_bkpts[name]  # Function values at breakpoints (y-values)
        num_segments = length(var_bkpts) - 1   # Number of linear segments

        for t in time_steps
            # Variable interpolation constraint: x = x₁ + Σᵢ δᵢ(xᵢ₊₁ - xᵢ)
            # This ensures the original variable is expressed as a convex combination
            # of breakpoint intervals weighted by interpolation variables
            const_container_var[name, t] = JuMP.@constraint(
                JuMPmodel,
                x_var[x_name, t] ==
                var_bkpts[1] + sum(
                    δ_var[name, i, t] * (var_bkpts[i + 1] - var_bkpts[i]) for
                    i in 1:num_segments
                )
            )

            # Function interpolation constraint: y = y₁ + Σᵢ δᵢ(yᵢ₊₁ - yᵢ)
            # This defines the piecewise linear approximation of the function
            const_container_function[name, t] = JuMP.@constraint(
                JuMPmodel,
                y_var[name, t] ==
                function_bkpts[1] + sum(
                    δ_var[name, i, t] * (function_bkpts[i + 1] - function_bkpts[i]) for
                    i in 1:num_segments
                )
            )

            # Incremental ordering constraints using binary variables (SOS2)
            # These ensure that δᵢ₊₁ ≤ zᵢ ≤ δᵢ, which maintains the incremental property:
            # segments must be filled in order (δ₁ before δ₂, δ₂ before δ₃, etc.)
            for i in 1:(num_segments - 1)
                # z[i] must be >= δ[i+1]: can't activate later segment without current one
                JuMP.@constraint(JuMPmodel, z_var[name, i, t] >= δ_var[name, i + 1, t])
                # z[i] must be <= δ[i]: can't be more activated than current segment
                JuMP.@constraint(JuMPmodel, z_var[name, i, t] <= δ_var[name, i, t])
            end
        end
    end
    return
end
