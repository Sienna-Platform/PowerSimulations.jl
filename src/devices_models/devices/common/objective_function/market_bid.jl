##################################################
################ PWL Parameters  #################
##################################################

# Helper functions to manage incremental (false) vs. decremental (true) cases
get_initial_input_maybe_decremental(::Val{true}, device::PSY.StaticInjection) =
    PSY.get_decremental_initial_input(PSY.get_operation_cost(device))
get_initial_input_maybe_decremental(::Val{false}, device::PSY.StaticInjection) =
    PSY.get_incremental_initial_input(PSY.get_operation_cost(device))

get_offer_curves_maybe_decremental(::Val{true}, device::PSY.StaticInjection) =
    get_input_offer_curves(PSY.get_operation_cost(device))
get_offer_curves_maybe_decremental(::Val{false}, device::PSY.StaticInjection) =
    get_output_offer_curves(PSY.get_operation_cost(device))

# Dictionaries to handle more incremental (false) vs. decremental (true) cases
const _SLOPE_PARAMS::Dict{Bool, Type{<:AbstractPiecewiseLinearSlopeParameter}} = Dict(
    false => IncrementalPiecewiseLinearSlopeParameter,
    true => DecrementalPiecewiseLinearSlopeParameter)
const _BREAKPOINT_PARAMS::Dict{Bool, Type{<:AbstractPiecewiseLinearBreakpointParameter}} =
    Dict(
        false => IncrementalPiecewiseLinearBreakpointParameter,
        true => DecrementalPiecewiseLinearBreakpointParameter)
const _PIECEWISE_BLOCK_VARS::Dict{Bool, Type{<:AbstractPiecewiseLinearBlockOffer}} = Dict(
    false => PiecewiseLinearBlockIncrementalOffer,
    true => PiecewiseLinearBlockDecrementalOffer)
const _PIECEWISE_BLOCK_CONSTRAINTS::Dict{
    Bool,
    Type{<:AbstractPiecewiseLinearBlockOfferConstraint},
} = Dict(
    false => PiecewiseLinearBlockIncrementalOfferConstraint,
    true => PiecewiseLinearBlockDecrementalOfferConstraint)

# Determines whether we care about various types of costs, given the formulation
# NOTE: currently works based on what has already been added to the container;
# alternatively we could dispatch on the formulation directly

_consider_parameter(
    ::StartupCostParameter,
    container::OptimizationContainer,
    ::DeviceModel{T, D},
) where {T, D} =
    any(has_container_key.([container], [StartVariable, MULTI_START_VARIABLES...], [T]))

_consider_parameter(
    ::ShutdownCostParameter,
    container::OptimizationContainer,
    ::DeviceModel{T, D},
) where {T, D} = has_container_key(container, StopVariable, T)

# FIXME storage doesn't currently have an OnVariable. should it have one?
_consider_parameter(
    ::AbstractCostAtMinParameter,
    container::OptimizationContainer,
    ::DeviceModel{T, D},
) where {T, D} = has_container_key(container, OnVariable, T)

# For slopes and breakpoints, the relevant variables won't have been created yet, so we'll
# just check all components for the presence of the relevant time series
_consider_parameter(
    ::AbstractPiecewiseLinearSlopeParameter,
    ::OptimizationContainer,
    ::DeviceModel{T, D},
) where {T, D} = true

_consider_parameter(
    ::AbstractPiecewiseLinearBreakpointParameter,
    ::OptimizationContainer,
    ::DeviceModel{T, D},
) where {T, D} = true

_has_market_bid_cost(device::PSY.StaticInjection) =
    PSY.get_operation_cost(device) isa PSY.MarketBidCost

_has_market_bid_cost(::PSY.RenewableNonDispatch) = false

_has_market_bid_cost(::PSY.PowerLoad) = false # PowerLoads don't even have operation cost.
_has_market_bid_cost(device::PSY.ControllableLoad) =
    PSY.get_operation_cost(device) isa PSY.MarketBidCost

_has_import_export_cost(device::PSY.Source) =
    PSY.get_operation_cost(device) isa PSY.ImportExportCost

_has_import_export_cost(::PSY.StaticInjection) = false

_has_offer_curve_cost(device::PSY.Component) =
    _has_market_bid_cost(device) || _has_import_export_cost(device)

_has_parameter_time_series(::StartupCostParameter, device::PSY.StaticInjection) =
    is_time_variant(PSY.get_start_up(PSY.get_operation_cost(device)))

_has_parameter_time_series(::ShutdownCostParameter, device::PSY.StaticInjection) =
    is_time_variant(PSY.get_shut_down(PSY.get_operation_cost(device)))

_has_parameter_time_series(
    ::T,
    device::PSY.StaticInjection,
) where {T <: AbstractCostAtMinParameter} =
    _has_offer_curve_cost(device) &&
    is_time_variant(_get_parameter_field(T(), PSY.get_operation_cost(device)))

_has_parameter_time_series(
    ::T,
    device::PSY.StaticInjection,
) where {T <: AbstractPiecewiseLinearSlopeParameter} =
    _has_offer_curve_cost(device) &&
    is_time_variant(_get_parameter_field(T(), PSY.get_operation_cost(device)))

_has_parameter_time_series(
    ::T,
    device::PSY.StaticInjection,
) where {T <: AbstractPiecewiseLinearBreakpointParameter} =
    _has_offer_curve_cost(device) &&
    is_time_variant(_get_parameter_field(T(), PSY.get_operation_cost(device)))

function validate_initial_input_time_series(device::PSY.StaticInjection, decremental::Bool)
    initial_input = get_initial_input_maybe_decremental(Val(decremental), device)
    initial_is_ts = is_time_variant(initial_input)
    variable_is_ts = is_time_variant(
        get_offer_curves_maybe_decremental(Val(decremental), device))
    label = decremental ? "decremental" : "incremental"

    (initial_is_ts && !variable_is_ts) &&
        @warn "In `MarketBidCost` for $(get_name(device)), found time series for `$(label)_initial_input` but non-time-series `$(label)_offer_curves`; will ignore `initial_input` of `$(label)_offer_curves"
    (variable_is_ts && !initial_is_ts) &&
        throw(
            ArgumentError(
                "In `MarketBidCost` for $(get_name(device)), if providing time series for `$(label)_offer_curves`, must also provide time series for `$(label)_initial_input`",
            ),
        )

    if !variable_is_ts && !initial_is_ts
        _validate_eltype(
            Union{Float64, Nothing}, device, initial_input, " initial_input",
        )
    else
        _validate_eltype(
            Float64, device, initial_input, " initial_input",
        )
    end
end

function validate_occ_breakpoints_slopes(device::PSY.StaticInjection, decremental::Bool)
    offer_curves = get_offer_curves_maybe_decremental(Val(decremental), device)
    device_name = get_name(device)
    is_ts = is_time_variant(offer_curves)
    expected_type = if is_ts
        IS.PiecewiseStepData
    else
        PSY.CostCurve{PSY.PiecewiseIncrementalCurve}
    end
    p1 = nothing
    apply_maybe_across_time_series(device, offer_curves) do x
        curve_type = decremental ? "decremental" : "incremental"
        _validate_eltype(expected_type, device, x, " $curve_type offer curves")
        if decremental
            PSY.is_concave(x) ||
                throw(
                    ArgumentError(
                        "Decremental $(nameof(typeof(PSY.get_operation_cost(device)))) for component $(device_name) is non-concave",
                    ),
                )
        else
            PSY.is_convex(x) ||
                throw(
                    ArgumentError(
                        "Incremental $(nameof(typeof(PSY.get_operation_cost(device)))) for component $(device_name) is non-convex",
                    ),
                )
        end

        # Different specific validations for MBC versus IEC
        p1 = _validate_occ_subtype(
            PSY.get_operation_cost(device),
            decremental,
            is_ts,
            x,
            device_name,
            p1,
        )
    end
end

function _validate_occ_subtype(
    ::PSY.MarketBidCost,
    decremental,
    is_ts,
    curve::PSY.PiecewiseStepData,
    device_name::String,
    p1::Union{Nothing, Float64},
)
    @assert is_ts
    my_p1 = first(PSY.get_x_coords(curve))
    if isnothing(p1)
        p1 = my_p1
    elseif !isapprox(p1, my_p1)
        throw(
            ArgumentError(
                "Inconsistent minimum breakpoint values in time series MarketBidCost for $(device_name) offer curves. For time-variable MarketBidCost, all first x-coordinates must be equal across the entire time series.",
            ),
        )
    end
    return p1
end

_validate_occ_subtype(
    ::PSY.MarketBidCost,
    decremental,
    is_ts,
    ::PSY.CostCurve,
    args...,
) =
    @assert !is_ts

function _validate_occ_subtype(
    cost::PSY.ImportExportCost,
    decremental,
    is_ts,
    curve::PSY.CostCurve,
    args...,
)
    # In the non-time-variable case, a VOM cost and initial input are represented; these must be zero
    @assert !is_ts
    !iszero(PSY.get_vom_cost(curve)) && throw(
        ArgumentError(
            "For ImportExportCost, VOM cost must be zero.",
        ),
    )
    vc = PSY.get_value_curve(curve)
    !iszero(PSY.get_initial_input(curve)) && throw(
        ArgumentError(
            "For ImportExportCost, initial input must be zero.",
        ),
    )
    _validate_occ_subtype(cost, decremental, true, PSY.get_function_data(vc))  # also do the FunctionData validations
end

function _validate_occ_subtype(
    ::PSY.ImportExportCost,
    decremental,
    is_ts,
    curve::PSY.PiecewiseStepData,
    args...,
)
    # In the time-variable case, VOM cost and initial input cannot be represented, so they cannot be nonzero
    @assert is_ts
    if !iszero(first(PSY.get_x_coords(curve)))
        throw(
            ArgumentError(
                "For ImportExportCost, the first breakpoint must be zero.",
            ),
        )
    end
end

# Warn if hot/warm/cold startup costs are given for non-`ThermalMultiStart`
function validate_occ_component(
    ::StartupCostParameter,
    device::PSY.ThermalMultiStart,
)
    startup = PSY.get_start_up(PSY.get_operation_cost(device))
    _validate_eltype(
        Union{Float64, NTuple{3, Float64}, StartUpStages},
        device,
        startup,
        " startup cost",
    )
end

function validate_occ_component(::StartupCostParameter, device::PSY.StaticInjection)
    startup = PSY.get_start_up(PSY.get_operation_cost(device))
    contains_multistart = false
    apply_maybe_across_time_series(device, startup) do x
        if x isa Float64
            return
        elseif x isa Union{NTuple{3, Float64}, StartUpStages}
            contains_multistart = true
        else
            location =
                is_time_variant(startup) ? " in time series $(get_name(startup))" : ""
            throw(
                ArgumentError(
                    "Expected Float64 or NTuple{3, Float64} or StartUpStages startup cost but got $(typeof(x))$location for $(get_name(device))",
                ),
            )
        end
    end
    if contains_multistart
        location = is_time_variant(startup) ? " in time series $(get_name(startup))" : ""
        @warn "Multi-start costs detected$location for non-multi-start unit $(get_name(device)), will take the maximum"
    end
    return
end

# Validate eltype of shutdown costs
function validate_occ_component(::ShutdownCostParameter, device::PSY.StaticInjection)
    shutdown = PSY.get_shut_down(PSY.get_operation_cost(device))
    _validate_eltype(Float64, device, shutdown, " for shutdown cost")
end

# Renewable-specific validations that warn when costs are nonzero.
# There warnings are captured by the with_logger, though, so we don't actually see them.
function validate_occ_component(
    ::StartupCostParameter,
    device::Union{PSY.RenewableDispatch, PSY.Storage},
)
    startup = PSY.get_start_up(PSY.get_operation_cost(device))
    apply_maybe_across_time_series(device, startup) do x
        if x != PSY.single_start_up_to_stages(0.0)
            #println(
            @warn "Nonzero startup cost detected for renewable generation or storage device $(get_name(device))."
            # )
        end
    end
end

function validate_occ_component(
    ::ShutdownCostParameter,
    device::Union{PSY.RenewableDispatch, PSY.Storage},
)
    shutdown = PSY.get_shut_down(PSY.get_operation_cost(device))
    apply_maybe_across_time_series(device, shutdown) do x
        if x != 0.0
            #println(
            @warn "Nonzero shutdown cost detected for renewable generation or storage device $(get_name(device))."
            #)
        end
    end
end

function validate_occ_component(
    ::IncrementalCostAtMinParameter,
    device::Union{PSY.RenewableDispatch, PSY.Storage},
)
    no_load_cost = PSY.get_no_load_cost(PSY.get_operation_cost(device))
    if !isnothing(no_load_cost)
        apply_maybe_across_time_series(device, no_load_cost) do x
            if x != 0.0
                #println(
                @warn "Nonzero no-load cost detected for renewable generation or storage device $(get_name(device))."
                #)
            end
        end
    end
end

function validate_occ_component(
    ::DecrementalCostAtMinParameter,
    device::PSY.Storage,
)
    no_load_cost = PSY.get_no_load_cost(PSY.get_operation_cost(device))
    if !isnothing(no_load_cost)
        apply_maybe_across_time_series(device, no_load_cost) do x
            if x != 0.0
                #println(
                @warn "Nonzero no-load cost detected for storage device $(get_name(device))."
                #)
            end
        end
    end
end

# Validate that initial input ts always appears if variable ts appears, warn if initial input ts appears without variable ts
validate_occ_component(
    ::IncrementalCostAtMinParameter,
    device::PSY.StaticInjection,
) =
    validate_initial_input_time_series(device, false)
validate_occ_component(
    ::DecrementalCostAtMinParameter,
    device::PSY.StaticInjection,
) =
    validate_initial_input_time_series(device, true)

# Validate convexity/concavity of cost curves as appropriate, verify P1 = min gen power
validate_occ_component(
    ::IncrementalPiecewiseLinearBreakpointParameter,
    device::PSY.StaticInjection,
) =
    validate_occ_breakpoints_slopes(device, false)
validate_occ_component(
    ::DecrementalPiecewiseLinearBreakpointParameter,
    device::PSY.StaticInjection,
) =
    validate_occ_breakpoints_slopes(device, true)

# Slope and breakpoint validations are done together, nothing to do here
validate_occ_component(
    ::AbstractPiecewiseLinearSlopeParameter,
    device::PSY.StaticInjection,
) = nothing

# Validates and adds parameters for a given OfferCurveCost-related ParameterType
# PERF: could switch to a TSC here.
function _process_occ_parameters_helper(
    ::P,
    container::OptimizationContainer,
    model,
    devices,
) where {P <: ParameterType}
    param_instance = P()
    for device in devices
        validate_occ_component(param_instance, device)
    end
    if _consider_parameter(param_instance, container, model)
        ts_devices =
            filter(device -> _has_parameter_time_series(param_instance, device), devices)
        (length(ts_devices) > 0) && add_parameters!(container, P, ts_devices, model)
    end
end

"Validate MarketBidCosts and add the appropriate parameters"
function process_market_bid_parameters!(
    container::OptimizationContainer,
    devices_in,
    model::DeviceModel,
    incremental::Bool = true,
    decremental::Bool = false,
)
    devices = filter(_has_market_bid_cost, collect(devices_in))  # https://github.com/Sienna-Platform/InfrastructureSystems.jl/issues/460
    isempty(devices) && return

    # Validate and add the parameters:
    for param in (
        StartupCostParameter(),
        ShutdownCostParameter(),
    )
        _process_occ_parameters_helper(param, container, model, devices)
    end
    if incremental
        for param in (
            IncrementalCostAtMinParameter(),
            IncrementalPiecewiseLinearSlopeParameter(),
            IncrementalPiecewiseLinearBreakpointParameter(),
        )
            _process_occ_parameters_helper(param, container, model, devices)
        end
    end
    if decremental
        for param in (
            DecrementalCostAtMinParameter(),
            DecrementalPiecewiseLinearSlopeParameter(),
            DecrementalPiecewiseLinearBreakpointParameter(),
        )
            _process_occ_parameters_helper(param, container, model, devices)
        end
    end
end

##################################################
################# PWL Variables ##################
##################################################

# For Market Bid
function _add_pwl_variables!(
    container::OptimizationContainer,
    ::Type{T},
    component_name::String,
    time_period::Int,
    n_tranches::Int,
    ::Type{U},
) where {
    T <: PSY.Component,
    U <: AbstractPiecewiseLinearBlockOffer,
}
    var_container = lazy_container_addition!(container, U(), T)
    # length(PiecewiseStepData) gets number of segments, here we want number of points
    pwlvars = Array{JuMP.VariableRef}(undef, n_tranches)
    for i in 1:n_tranches
        pwlvars[i] =
            var_container[(component_name, i, time_period)] = JuMP.@variable(
                get_jump_model(container),
                base_name = "$(nameof(U))_$(component_name)_{pwl_$(i), $time_period}",
                lower_bound = 0.0,
            )
    end
    return pwlvars
end

##################################################
################# PWL Constraints ################
##################################################

# without this, you get "variable OnVariable__RenewableDispatch is not stored"
# TODO: really this falls along the divide of
# commitment (OnVariable + ActivePower) vs dispatch (ActivePower only)
_include_min_gen_power_in_constraint(
    ::PSY.RenewableDispatch,
    ::ActivePowerVariable,
    ::AbstractDeviceFormulation,
) = false
_include_min_gen_power_in_constraint(
    ::PSY.Generator,
    ::ActivePowerVariable,
    ::AbstractDeviceFormulation,
) = true
_include_min_gen_power_in_constraint(
    ::PSY.ControllableLoad,
    ::ActivePowerVariable,
    ::PowerLoadInterruption,
) = true
_include_min_gen_power_in_constraint(
    ::PSY.ControllableLoad,
    ::ActivePowerVariable,
    ::PowerLoadDispatch,
) = false
_include_min_gen_power_in_constraint(
    ::Any,
    ::PowerAboveMinimumVariable,
    ::AbstractDeviceFormulation,
) = false

# add the minimum generation power to the PWL constraint, as a constant. Returns true for
# formulations where there's nonzero minimum power (first breakpoint), but no OnVariable.
# TODO: cleaner way? e.g. can we just do this whenever there's no OnVariable?
_include_constant_min_gen_power_in_constraint(
    ::PSY.ControllableLoad,
    ::ActivePowerVariable,
    ::PowerLoadDispatch,
) = true
_include_constant_min_gen_power_in_constraint(
    ::PSY.ControllableLoad,
    ::ActivePowerVariable,
    ::PowerLoadInterruption,
) = false
_include_constant_min_gen_power_in_constraint(
    ::PSY.RenewableGen,
    ::ActivePowerVariable,
    ::AbstractRenewableDispatchFormulation,
) = true
_include_constant_min_gen_power_in_constraint(
    ::Any,
    ::VariableType,
    ::AbstractDeviceFormulation,
) = false

"""
Implement the constraints for PWL Block Offer variables. That is:

```math
\\sum_{k\\in\\mathcal{K}} \\delta_{k,t} = p_t \\\\
\\sum_{k\\in\\mathcal{K}} \\delta_{k,t} <= P_{k+1,t}^{max} - P_{k,t}^{max}
```
"""
function _add_pwl_constraint!(
    container::OptimizationContainer,
    component::T,
    ::U,
    ::D,
    break_points::Vector{<:JuMPOrFloat},
    period::Int,
    ::Type{V},
    ::Type{W},
) where {T <: PSY.Component, U <: VariableType,
    D <: AbstractDeviceFormulation,
    V <: AbstractPiecewiseLinearBlockOffer,
    W <: AbstractPiecewiseLinearBlockOfferConstraint}
    variables = get_variable(container, U(), T)
    const_container = lazy_container_addition!(
        container,
        W(),
        T,
        axes(variables)...,
    )
    len_cost_data = length(break_points) - 1
    jump_model = get_jump_model(container)
    pwl_vars = get_variable(container, V(), T)
    name = PSY.get_name(component)
    sum_pwl_vars = sum(pwl_vars[name, ix, period] for ix in 1:len_cost_data)

    # As detailed in https://github.com/Sienna-Platform/PowerSimulations.jl/issues/1318,
    # time-variable P1 is problematic, so for now we require P1 to be constant. Thus we can
    # just look up what it is currently fixed to and use that here without worrying about
    # updating.
    if _include_constant_min_gen_power_in_constraint(component, U(), D())
        # TODO this seems kind of redundant with the
        sum_pwl_vars += jump_fixed_value(first(break_points))::Float64
    elseif _include_min_gen_power_in_constraint(component, U(), D())
        on_vars = get_variable(container, OnVariable(), T)
        p1::Float64 = jump_fixed_value(first(break_points))
        sum_pwl_vars += p1 * on_vars[name, period]
    end

    const_container[name, period] = JuMP.@constraint(
        jump_model,
        variables[name, period] == sum_pwl_vars
    )

    for ix in 1:len_cost_data
        JuMP.@constraint(
            jump_model,
            pwl_vars[name, ix, period] <= break_points[ix + 1] - break_points[ix]
        )
    end
    return
end

"""
Implement the constraints for PWL Block Offer variables for ORDC. That is:

```math
\\sum_{k\\in\\mathcal{K}} \\delta_{k,t} = p_t \\\\
\\sum_{k\\in\\mathcal{K}} \\delta_{k,t} <= P_{k+1,t}^{max} - P_{k,t}^{max}
```
"""
function _add_pwl_constraint!(
    container::OptimizationContainer,
    component::T,
    ::U,
    break_points::Vector{Float64},
    sos_status::SOSStatusVariable,
    period::Int,
) where {T <: PSY.ReserveDemandCurve, U <: ServiceRequirementVariable}
    name = PSY.get_name(component)
    variables = get_variable(container, U(), T, name)
    const_container = lazy_container_addition!(
        container,
        PiecewiseLinearBlockIncrementalOfferConstraint(),
        T,
        axes(variables)...;
        meta = name,
    )
    len_cost_data = length(break_points) - 1
    jump_model = get_jump_model(container)
    pwl_vars = get_variable(container, PiecewiseLinearBlockIncrementalOffer(), T)
    const_container[name, period] = JuMP.@constraint(
        jump_model,
        variables[name, period] ==
        sum(pwl_vars[name, ix, period] for ix in 1:len_cost_data)
    )

    for ix in 1:len_cost_data
        JuMP.@constraint(
            jump_model,
            pwl_vars[name, ix, period] <= break_points[ix + 1] - break_points[ix]
        )
    end
    return
end

##################################################
################ PWL Expressions #################
##################################################

get_offer_curves_for_var(var, comp::PSY.Component) =
    get_offer_curves_for_var(var, get_operation_cost(comp))
get_offer_curves_for_var(::PiecewiseLinearBlockIncrementalOffer, cost::PSY.MarketBidCost) =
    get_offer_curves_maybe_decremental(Val(false), cost)
get_offer_curves_for_var(::PiecewiseLinearBlockDecrementalOffer, cost::PSY.MarketBidCost) =
    get_offer_curves_maybe_decremental(Val(true), cost)

get_multiplier_for_var(::PiecewiseLinearBlockIncrementalOffer) = OBJECTIVE_FUNCTION_POSITIVE
get_multiplier_for_var(::PiecewiseLinearBlockDecrementalOffer) = OBJECTIVE_FUNCTION_NEGATIVE

function _get_pwl_cost_expression(
    container::OptimizationContainer,
    component::T,
    time_period::Int,
    slopes_normalized::Vector{Float64},
    ::U,
    ::V,
    ::W,
) where {
    T <: PSY.Component,
    U <: VariableType,
    V <: AbstractDeviceFormulation,
    W <: AbstractPiecewiseLinearBlockOffer,
}
    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR
    multiplier = get_multiplier_for_var(W()) * dt

    name = PSY.get_name(component)
    pwl_var_container = get_variable(container, W(), T)
    gen_cost = JuMP.AffExpr(0.0)
    for (i, cost) in enumerate(slopes_normalized)
        JuMP.add_to_expression!(
            gen_cost,
            (cost * multiplier),
            pwl_var_container[(name, i, time_period)],
        )
    end
    return gen_cost
end

"""
Get cost expression for StepwiseCostReserve
"""
function _get_pwl_cost_expression(
    container::OptimizationContainer,
    component::T,
    time_period::Int,
    slopes_normalized::Vector{Float64},
    multiplier::Float64,
) where {T <: PSY.ReserveDemandCurve}
    name = PSY.get_name(component)
    pwl_var_container = get_variable(container, PiecewiseLinearBlockIncrementalOffer(), T)
    ordc_cost = JuMP.AffExpr(0.0)
    for (i, slope) in enumerate(slopes_normalized)
        JuMP.add_to_expression!(
            ordc_cost,
            slope * multiplier,
            pwl_var_container[(name, i, time_period)],
        )
    end
    return ordc_cost
end

###############################################
######## MarketBidCost: Fixed Curves ##########
###############################################

# Serves a similar role as _lookup_maybe_time_variant_param, but needs extra logic
function _get_pwl_data(
    is_decremental::Bool,
    container::OptimizationContainer,
    component::T,
    time::Int,
) where {T <: PSY.Component}
    cost_data = get_offer_curves_maybe_decremental(Val(is_decremental), component)

    if is_time_variant(cost_data)
        name = PSY.get_name(component)

        SlopeParam = _SLOPE_PARAMS[is_decremental]
        slope_param_arr = get_parameter_array(container, SlopeParam(), T)
        slope_param_mult = get_parameter_multiplier_array(container, SlopeParam(), T)
        @assert size(slope_param_arr) == size(slope_param_mult)  # multiplier arrays should be 3D too
        slope_cost_component =
            slope_param_arr[name, :, time] .* slope_param_mult[name, :, time]
        slope_cost_component = slope_cost_component.data

        BreakpointParam = _BREAKPOINT_PARAMS[is_decremental]
        breakpoint_param_container = get_parameter(container, BreakpointParam(), T)
        breakpoint_param_arr = get_parameter_column_refs(breakpoint_param_container, name)  # performs component -> time series many-to-one mapping
        breakpoint_param_mult = get_multiplier_array(breakpoint_param_container)
        @assert size(breakpoint_param_arr) == size(breakpoint_param_mult[name, :, :])
        breakpoint_cost_component =
            breakpoint_param_arr[:, time] .* breakpoint_param_mult[name, :, time]
        breakpoint_cost_component = breakpoint_cost_component.data

        @assert_op length(slope_cost_component) == length(breakpoint_cost_component) - 1
        # PSY's cost_function_timeseries.jl says this will always be natural units
        unit_system = PSY.UnitSystem.NATURAL_UNITS
    else
        cost_component = PSY.get_function_data(PSY.get_value_curve(cost_data))
        breakpoint_cost_component = PSY.get_x_coords(cost_component)
        slope_cost_component = PSY.get_y_coords(cost_component)
        unit_system = PSY.get_power_units(cost_data)
    end

    breakpoints, slopes = get_piecewise_curve_per_system_unit(
        breakpoint_cost_component,
        slope_cost_component,
        unit_system,
        get_base_power(container),
        PSY.get_base_power(component),
    )

    return breakpoints, slopes
end

"""
Add PWL cost terms for data coming from the MarketBidCost
with a fixed incremental offer curve
"""
function add_pwl_term!(
    is_decremental::Bool,
    container::OptimizationContainer,
    component::T,
    ::PSY.OfferCurveCost,
    ::U,
    ::V,
) where {T <: PSY.Component, U <: VariableType, V <: AbstractDeviceFormulation}
    name = PSY.get_name(component)
    W = _PIECEWISE_BLOCK_VARS[is_decremental]
    X = _PIECEWISE_BLOCK_CONSTRAINTS[is_decremental]

    name = PSY.get_name(component)
    time_steps = get_time_steps(container)
    for t in time_steps
        breakpoints, slopes = _get_pwl_data(is_decremental, container, component, t)
        _add_pwl_variables!(container, T, name, t, length(slopes), W)
        _add_pwl_constraint!(
            container,
            component,
            U(),
            V(),
            breakpoints,
            t,
            W,
            X,
        )
        pwl_cost = _get_pwl_cost_expression(
            container,
            component,
            t,
            slopes,
            U(),
            V(),
            W(),
        )

        add_to_expression!(
            container,
            ProductionCostExpression,
            pwl_cost,
            component,
            t,
        )

        if is_time_variant(
            get_offer_curves_maybe_decremental(Val(is_decremental), component),
        )
            add_to_objective_variant_expression!(container, pwl_cost)
        else
            add_to_objective_invariant_expression!(container, pwl_cost)
        end
    end
end

##################################################
########## PWL for StepwiseCostReserve  ##########
##################################################

# Not touching this in PR #1303, TODO figure it out later -GKS
function _add_pwl_term!(
    container::OptimizationContainer,
    component::T,
    cost_data::PSY.CostCurve{PSY.PiecewiseIncrementalCurve},
    ::U,
    ::V,
) where {T <: PSY.Component, U <: VariableType, V <: AbstractServiceFormulation}
    multiplier = objective_function_multiplier(U(), V())
    resolution = get_resolution(container)
    dt = Dates.value(Dates.Second(resolution)) / SECONDS_IN_HOUR
    base_power = get_base_power(container)
    value_curve = PSY.get_value_curve(cost_data)
    power_units = PSY.get_power_units(cost_data)
    cost_component = PSY.get_function_data(value_curve)
    device_base_power = PSY.get_base_power(component)
    data = get_piecewise_curve_per_system_unit(
        cost_component,
        power_units,
        base_power,
        device_base_power,
    )
    name = PSY.get_name(component)
    time_steps = get_time_steps(container)
    pwl_cost_expressions = Vector{JuMP.AffExpr}(undef, time_steps[end])
    sos_val = _get_sos_value(container, V, component)
    for t in time_steps
        break_points = PSY.get_x_coords(data)
        _add_pwl_variables!(
            container,
            T,
            name,
            t,
            length(IS.get_y_coords(data)),
            PiecewiseLinearBlockIncrementalOffer,
        )
        _add_pwl_constraint!(container, component, U(), break_points, sos_val, t)
        pwl_cost = _get_pwl_cost_expression(
            container,
            component,
            t,
            IS.get_y_coords(data),
            multiplier * dt,
        )
        pwl_cost_expressions[t] = pwl_cost
    end
    return pwl_cost_expressions
end

############################################################
######## MarketBidCost: PiecewiseIncrementalCurve ##########
############################################################

"""
Creates piecewise linear market bid function using a sum of variables and expression for market participants.
Decremental offers are not accepted for most components, except Storage systems and loads.

# Arguments

  - container::OptimizationContainer : the optimization_container model built in PowerSimulations
  - var_key::VariableKey: The variable name
  - component_name::String: The component_name of the variable container
  - cost_function::MarketBidCost : container for market bid cost
"""
function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    cost_function::PSY.OfferCurveCost,
    ::U,
) where {T <: VariableType, U <: AbstractDeviceFormulation}
    component_name = PSY.get_name(component)
    @debug "Market Bid" _group = LOG_GROUP_COST_FUNCTIONS component_name
    if !isnothing(get_input_offer_curves(cost_function))
        error("Component $(component_name) is not allowed to participate as a demand.")
    end
    add_pwl_term!(
        false, # I suspect this is a problem. could very well be decremental, storage
        container,
        component,
        cost_function,
        T(),
        U(),
    )
    return
end

function _add_variable_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    cost_function::PSY.OfferCurveCost,
    ::U,
) where {T <: VariableType,
    U <: AbstractControllablePowerLoadFormulation}
    component_name = PSY.get_name(component)
    @debug "Market Bid" _group = LOG_GROUP_COST_FUNCTIONS component_name
    if !(isnothing(get_output_offer_curves(cost_function)))
        error("Component $(component_name) is not allowed to participate as a supply.")
    end
    add_pwl_term!(
        true,
        container,
        component,
        cost_function,
        T(),
        U(),
    )
    return
end

function _add_service_bid_cost!(
    container::OptimizationContainer,
    component::PSY.Component,
    service::T,
) where {T <: PSY.Reserve{<:PSY.ReserveDirection}}
    time_steps = get_time_steps(container)
    initial_time = get_initial_time(container)
    base_power = get_base_power(container)
    forecast_data = PSY.get_services_bid(
        component,
        PSY.get_operation_cost(component),
        service;
        start_time = initial_time,
        len = length(time_steps),
    )
    forecast_data_values = PSY.get_cost.(TimeSeries.values(forecast_data))
    # Single Price Bid
    if eltype(forecast_data_values) == Float64
        data_values = forecast_data_values
        # Single Price/Quantity Bid
    elseif eltype(forecast_data_values) == Vector{NTuple{2, Float64}}
        data_values = [v[1][1] for v in forecast_data_values]
    else
        error("$(eltype(forecast_data_values)) not supported for MarketBidCost")
    end

    reserve_variable =
        get_variable(container, ActivePowerReserveVariable(), T, PSY.get_name(service))
    component_name = PSY.get_name(component)
    for t in time_steps
        add_to_objective_invariant_expression!(
            container,
            data_values[t] * base_power * reserve_variable[component_name, t],
        )
    end
    return
end

function _add_service_bid_cost!(::OptimizationContainer, ::PSY.Component, ::PSY.Service) end

# "copy-paste and change incremental to decremental" here. Refactor?
function _add_vom_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    op_cost::PSY.OfferCurveCost,
    ::U,
) where {T <: VariableType, U <: AbstractDeviceFormulation}
    incremental_cost_curves = get_output_offer_curves(op_cost)
    if is_time_variant(incremental_cost_curves)
        # TODO this might imply a change to the MBC struct?
        @warn "Incremental curves are time variant, there is no VOM cost source. Skipping VOM cost."
        return
    end
    _add_vom_cost_to_objective_helper!(
        container,
        T(),
        component,
        op_cost,
        incremental_cost_curves,
        U(),
    )
    return
end

function _add_vom_cost_to_objective!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    op_cost::PSY.OfferCurveCost,
    ::U,
) where {T <: VariableType,
    U <: AbstractControllablePowerLoadFormulation}
    decremental_cost_curves = get_input_offer_curves(op_cost)
    if is_time_variant(decremental_cost_curves)
        # TODO this might imply a change to the MBC struct?
        @warn "Decremental curves are time variant, there is no VOM cost source. Skipping VOM cost."
        return
    end
    _add_vom_cost_to_objective_helper!(
        container,
        T(),
        component,
        op_cost,
        decremental_cost_curves,
        U(),
    )
    return
end

function _add_vom_cost_to_objective_helper!(
    container::OptimizationContainer,
    ::T,
    component::PSY.Component,
    ::PSY.OfferCurveCost,
    cost_data::PSY.CostCurve{PSY.PiecewiseIncrementalCurve},
    ::U,
) where {T <: VariableType,
    U <: AbstractDeviceFormulation}
    power_units = PSY.get_power_units(cost_data)
    vom_cost = PSY.get_vom_cost(cost_data)
    multiplier = 1.0 # VOM Cost is always positive
    cost_term = PSY.get_proportional_term(vom_cost)
    iszero(cost_term) && return
    base_power = get_base_power(container)
    device_base_power = PSY.get_base_power(component)
    cost_term_normalized = get_proportional_cost_per_system_unit(cost_term,
        power_units,
        base_power,
        device_base_power)
    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR
    for t in get_time_steps(container)
        exp = _add_proportional_term!(
            container, T(), component, cost_term_normalized * multiplier * dt, t)
        add_to_expression!(container, VOMCostExpression, exp, component, t)
    end
    return
end
