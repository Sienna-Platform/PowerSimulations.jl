function _update_parameter_values!(
    parameter_array::DenseAxisArray,
    ::T,
    parameter_multiplier::JuMPFloatArray,
    attributes::CostFunctionAttributes,
    ::Type{V},
    model::DecisionModel,
    ::DatasetContainer{InMemoryDataset},
) where {T <: ObjectiveFunctionParameter, V <: PSY.Component}
    initial_forecast_time = get_current_time(model) # Function not well defined for DecisionModels
    time_steps = get_time_steps(get_optimization_container(model))
    horizon = time_steps[end]
    container = get_optimization_container(model)
    is_synchronized(container) && error(
        "Cannot update $(T) on a synchronized container; objective expressions would " *
        "be re-augmented and double-counted",
    )
    template = get_template(model)
    device_model = get_model(template, V)
    components = IOM.get_available_components(device_model, get_system(model))
    ts_type = get_deterministic_time_series_type(get_system(model))
    for component in components
        name = PSY.get_name(component)
        op_cost = PSY.get_operation_cost(component)
        # `handle_variable_cost_parameter` is responsible for figuring out whether there is
        # actually time variance for this particular component and, if so, performing the update
        handle_variable_cost_parameter(
            T(),
            op_cost,
            component,
            name,
            parameter_array,
            parameter_multiplier,
            attributes,
            model,
            initial_forecast_time,
            horizon,
            ts_type,
        )
    end
    return
end

# POM keys a cost parameter of every reserve of type `V` into one container with an empty
# `meta`, the reserves along its first axis, so each reserve on that axis is updated in turn.
function _update_service_cost_parameter_values!(
    parameter_array::DenseAxisArray,
    ::T,
    parameter_multiplier::JuMPFloatArray,
    attributes::CostFunctionAttributes,
    ::Type{V},
    model::DecisionModel,
    ::DatasetContainer{InMemoryDataset},
) where {T <: ObjectiveFunctionParameter, V <: PSY.Service}
    initial_forecast_time = get_current_time(model)
    time_steps = get_time_steps(get_optimization_container(model))
    horizon = time_steps[end]
    container = get_optimization_container(model)
    is_synchronized(container) && error(
        "Cannot update $(T) on a synchronized container; objective expressions would " *
        "be re-augmented and double-counted",
    )
    system = get_system(model)
    ts_type = get_deterministic_time_series_type(system)
    for service_name in axes(parameter_array, 1)
        service = PSY.get_component(V, system, service_name)
        isnothing(service) && error(
            "Service $(V) named '$(service_name)' is no longer in the system; " *
            "cannot update parameter $(T)",
        )
        handle_variable_cost_parameter(
            T(),
            service,
            service_name,
            parameter_array,
            parameter_multiplier,
            attributes,
            model,
            initial_forecast_time,
            horizon,
            ts_type,
        )
    end
    return
end

# Only PSY.OfferCurveCost has time-series cost parameters to update; every other
# OperationalCost is a no-op. OfferCurveCost is dispatched to the specialized
# methods below (it is more specific than PSY.OperationalCost), so these
# fallbacks are only reached for the no-op cases — no `isa` guard needed.
handle_variable_cost_parameter(
    ::Union{StartupCostParameter, ShutdownCostParameter, AbstractCostAtMinParameter},
    ::PSY.OperationalCost, args...) = nothing
handle_variable_cost_parameter(
    ::Union{
        AbstractPiecewiseLinearSlopeParameter,
        AbstractPiecewiseLinearBreakpointParameter,
    },
    ::PSY.OperationalCost, args...) = nothing

# Slope and breakpoint parameters share the same raw time series; the slope's resolved value
# is an objective term, the breakpoint's re-parametrizes the block-width constraint's RHS.
const _AnyPiecewiseLinearParameter =
    Union{AbstractPiecewiseLinearSlopeParameter, AbstractPiecewiseLinearBreakpointParameter}

# The block axis is sized for the longest curve in the series. A shorter curve is padded to it
# the way `unwrap_for_param` pads the parameter values and the build pins the blocks: zero-width
# blocks at the last breakpoint with zero slope. The update then rewrites every block's width
# and cost, so a block past the curve's end cannot keep the width of an earlier, longer curve.
_n_blocks(::AbstractPiecewiseLinearSlopeParameter, additional_axes) =
    length(only(additional_axes))
_n_blocks(::AbstractPiecewiseLinearBreakpointParameter, additional_axes) =
    length(only(additional_axes)) - 1

function _pad_to_blocks(value::PSY.PiecewiseStepData, n_blocks::Int)
    x_coords = PSY.get_x_coords(value)
    y_coords = PSY.get_y_coords(value)
    n_pad = n_blocks - length(y_coords)
    n_pad == 0 && return value
    n_pad > 0 || error(
        "PiecewiseStepData has $(length(y_coords)) segments, more than the $(n_blocks) " *
        "blocks the model was built with",
    )
    return PSY.PiecewiseStepData(
        vcat(x_coords, fill(last(x_coords), n_pad)),
        vcat(y_coords, zeros(n_pad)),
    )
end

_linear_block_width_constraint(::Type{IncrementalPiecewiseLinearBreakpointParameter}) =
    PiecewiseLinearBlockIncrementalWidthConstraint
_linear_block_width_constraint(::Type{DecrementalPiecewiseLinearBreakpointParameter}) =
    PiecewiseLinearBlockDecrementalWidthConstraint

"""
Set each per-block width constraint's RHS from updated breakpoints. Mirrors
`_update_pwl_cost_expression` below (the slope-side counterpart): both read a converted
`PSY.PiecewiseStepData` and walk its segments, but this one calls `JuMP.set_normalized_rhs`
on the stored width `ConstraintRef`s (IOM `objective_function_pwl_delta.jl`) rather than
rebuilding an objective expression.
"""
function _update_pwl_width_constraint!(
    ::P,
    container::OptimizationContainer,
    ::Type{T},
    component_name::String,
    time_period::Int,
    cost_data::PSY.PiecewiseStepData,
) where {P <: AbstractPiecewiseLinearBreakpointParameter, T <: PSY.Component}
    width_container = get_constraint(container, _linear_block_width_constraint(P), T)
    breakpoints = PSY.get_x_coords(cost_data)
    for ix in 1:(length(breakpoints) - 1)
        JuMP.set_normalized_rhs(
            width_container[(component_name, ix, time_period)],
            breakpoints[ix + 1] - breakpoints[ix],
        )
    end
    return
end

# Breakpoints re-parametrize the block-width constraint's RHS in place; no objective term changes.
function update_variable_cost!(
    breakpoint_param::AbstractPiecewiseLinearBreakpointParameter,
    container::OptimizationContainer,
    function_data::PSY.PiecewiseStepData,
    ::JuMPFloatArray,
    ::CostFunctionAttributes,
    component::T,
    time_period::Int,
    power_units::IS.AbstractUnitSystem,
) where {T <: PSY.Component}
    component_name = PSY.get_name(component)
    converted_data =
        _converted_piecewise_data(container, function_data, component, power_units)
    _update_pwl_width_constraint!(
        breakpoint_param, container, T, component_name, time_period, converted_data)
    return
end

# A device's curve also anchors its block-offer linking row at the first breakpoint, so a
# device update re-anchors that row under formulation `D`.
function update_variable_cost!(
    breakpoint_param::AbstractPiecewiseLinearBreakpointParameter,
    container::OptimizationContainer,
    function_data::PSY.PiecewiseStepData,
    parameter_multiplier::JuMPFloatArray,
    attributes::CostFunctionAttributes,
    component::T,
    time_period::Int,
    power_units::IS.AbstractUnitSystem,
    ::Type{D},
) where {T <: PSY.Component, D <: AbstractDeviceFormulation}
    update_variable_cost!(
        breakpoint_param,
        container,
        function_data,
        parameter_multiplier,
        attributes,
        component,
        time_period,
        power_units,
    )
    converted_data =
        _converted_piecewise_data(container, function_data, component, power_units)
    _update_pwl_offset!(
        breakpoint_param, container, component, time_period, converted_data, D)
    return
end

# A slope update rewrites objective terms only, so the formulation a device update
# carries for the breakpoint's row anchoring is not needed here.
update_variable_cost!(
    slope_param::AbstractPiecewiseLinearSlopeParameter,
    container::OptimizationContainer,
    function_data::PSY.PiecewiseStepData,
    parameter_multiplier::JuMPFloatArray,
    attributes::CostFunctionAttributes,
    component::PSY.Component,
    time_period::Int,
    power_units::IS.AbstractUnitSystem,
    ::Type{<:AbstractDeviceFormulation},
) = update_variable_cost!(
    slope_param,
    container,
    function_data,
    parameter_multiplier,
    attributes,
    component,
    time_period,
    power_units,
)

_converted_piecewise_data(
    container::OptimizationContainer,
    function_data::PSY.PiecewiseStepData,
    component::PSY.Component,
    power_units::IS.AbstractUnitSystem,
) = get_piecewise_curve_per_system_unit(
    function_data,
    power_units,
    get_model_base_power(container),
    PSY.get_base_power(component),
)

# The block-offer linking row a breakpoint parameter anchors, paired the way
# `_linear_block_width_constraint` pairs the width rows.
_linear_block_offer_constraint(::Type{IncrementalPiecewiseLinearBreakpointParameter}) =
    PiecewiseLinearBlockIncrementalOfferConstraint
_linear_block_offer_constraint(::Type{DecrementalPiecewiseLinearBreakpointParameter}) =
    PiecewiseLinearBlockDecrementalOfferConstraint

"""
Re-anchor the block-offer linking row at the curve's refreshed first breakpoint.
`add_pwl_constraint_delta!` (IOM `objective_function_pwl_delta.jl`) builds the row as
`power == Σ δ + p1 * on` for a formulation whose offset rides the `OnVariable`, as
`power == Σ δ + p1` for one with a constant minimum, and as `power == Σ δ` for one with
no offset, with `p1` the first breakpoint of the curve the model was built with. A curve
whose first breakpoint moves between solves must move that offset too, or the row keeps
the device at the old curve's start while the width rows describe the new one. The
branch mirrors the build's dispatch on the power variable the row links, so a formulation
without an offset is left alone.
"""
function _update_pwl_offset!(
    ::P,
    container::OptimizationContainer,
    component::T,
    time_period::Int,
    cost_data::PSY.PiecewiseStepData,
    ::Type{D},
) where {
    P <: AbstractPiecewiseLinearBreakpointParameter,
    T <: PSY.Component,
    D <: AbstractDeviceFormulation,
}
    component_name = PSY.get_name(component)
    row = get_constraint(container, _linear_block_offer_constraint(P), T)[
        component_name,
        time_period,
    ]
    U = _linked_power_variable(container, row, T, component_name, time_period)
    p1 = first(PSY.get_x_coords(cost_data))
    if IOM._include_constant_min_gen_power_in_constraint(T, U, D)
        JuMP.set_normalized_rhs(row, p1)
    elseif IOM._include_min_gen_power_in_constraint(T, U, D)
        on_vars = get_variable(container, OnVariable, T)
        if component_name in axes(on_vars, 1)
            JuMP.set_normalized_coefficient(row, on_vars[component_name, time_period], -p1)
        else
            JuMP.set_normalized_rhs(row, p1)
        end
    end
    return
end

# The cost parameter's attributes name only the block variables, so the power variable the
# linking row ties them to is read off the row: the one `T` variable entering it with
# coefficient one.
function _linked_power_variable(
    container::OptimizationContainer,
    row::JuMP.ConstraintRef,
    ::Type{T},
    component_name::String,
    time_period::Int,
) where {T <: PSY.Component}
    for (key, variables) in get_variables(container)
        IOM.get_component_type(key) === T || continue
        V = IOM.get_entry_type(key)
        V === OnVariable && continue
        ndims(variables) == 2 || continue
        component_name in axes(variables, 1) || continue
        v = variables[component_name, time_period]
        JuMP.normalized_coefficient(row, v) == 1.0 && return V
    end
    error("No power variable of $(T) links the block-offer row of $(component_name)")
end

_maybe_tuple(::StartupCostParameter, value) = Tuple(value)
_maybe_tuple(::ShutdownCostParameter, value) = value
_maybe_tuple(::AbstractCostAtMinParameter, value) = value

# Time-series key backing a parameter's raw cost-object field, and the time-series name
# resolved from it. Mirrors POM's build-time `_get_time_series_name` /
# `_device_offer_curve_ts_key` (PowerOperationsModels.jl/src/common_models/add_parameters.jl).
_cost_ts_key(
    ::T,
    op_cost::PSY.OfferCurveCost,
) where {T <: Union{StartupCostParameter, AbstractCostAtMinParameter}} =
    _get_parameter_field(T, op_cost)
_cost_ts_key(::T, op_cost::PSY.OfferCurveCost) where {T <: ShutdownCostParameter} =
    IS.get_time_series_key(_get_parameter_field(T, op_cost))
_cost_ts_key(::T, op_cost::PSY.OfferCurveCost) where {T <: _AnyPiecewiseLinearParameter} =
    IS.get_time_series_key(PSY.get_value_curve(_get_parameter_field(T, op_cost)))

# A key carries only its store-minted association id; the name lives in the catalog.
_ts_name_from_key(owner::PSY.Component, key::IS.TimeSeriesKey) =
    IS.get_name(IS.get_time_series_metadata(owner, key))

_cost_ts_name(param, owner, op_cost::PSY.OfferCurveCost) =
    _ts_name_from_key(owner, _cost_ts_key(param, op_cost))

function handle_variable_cost_parameter(
    param::T,
    op_cost::PSY.OfferCurveCost,
    component,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    model::DecisionModel,
    initial_forecast_time,
    horizon,
    ts_type,
) where {
    T <: Union{StartupCostParameter, ShutdownCostParameter, AbstractCostAtMinParameter},
}
    is_time_variant(_get_parameter_field(T, op_cost)) || return
    container = get_optimization_container(model)
    ts_name = _cost_ts_name(param, component, op_cost)
    raw_values = get_time_series_values!(
        ts_type,
        model,
        component,
        ts_name,
        initial_forecast_time,
        horizon,
    )
    _update_scalar_cost_values!(
        param,
        container,
        raw_values,
        parameter_array,
        parameter_multiplier,
        attributes,
        component,
        name,
    )
    return
end

# Function barrier: `raw_values` comes back from the time-series read abstractly typed, so the
# per-step loop is specialized here on its concrete runtime element type.
function _update_scalar_cost_values!(
    param::T,
    container::OptimizationContainer,
    raw_values,
    parameter_array,
    parameter_multiplier,
    attributes,
    component,
    name,
) where {
    T <: Union{StartupCostParameter, ShutdownCostParameter, AbstractCostAtMinParameter},
}
    additional_axes = lookup_additional_axes(parameter_array)
    for (t, raw_value) in enumerate(raw_values)
        value = unwrap_for_param(param, raw_value, additional_axes)
        _set_param_value!(parameter_array, _maybe_tuple(param, value), name, t)
        update_variable_cost!(
            param,
            container,
            parameter_array,
            parameter_multiplier,
            attributes,
            component,
            t,
        )
    end
    return
end

function handle_variable_cost_parameter(
    param::T,
    op_cost::PSY.OfferCurveCost,
    component,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    model::DecisionModel,
    initial_forecast_time,
    horizon,
    ts_type,
) where {T <: _AnyPiecewiseLinearParameter}
    offer_curve = _get_parameter_field(T, op_cost)
    is_time_variant(offer_curve) || return
    container = get_optimization_container(model)
    ts_name = _cost_ts_name(param, component, op_cost)
    power_units = IS.get_power_units(offer_curve)
    formulation = IOM.get_formulation(get_model(get_template(model), typeof(component)))
    raw_values = get_time_series_values!(
        ts_type,
        model,
        component,
        ts_name,
        initial_forecast_time,
        horizon,
    )
    additional_axes = lookup_additional_axes(parameter_array)
    n_blocks = _n_blocks(param, additional_axes)
    for (t, value::PSY.PiecewiseStepData) in enumerate(raw_values)
        unwrapped_value = unwrap_for_param(T(), value, additional_axes)
        _set_param_value!(parameter_array, unwrapped_value, name, t)
        update_variable_cost!(
            param,
            container,
            _pad_to_blocks(value, n_blocks),
            parameter_multiplier,
            attributes,
            component,
            t,
            power_units,
            formulation,
        )
    end
    return
end

# Every reserve kind carries its demand curve in `variable`, and POM builds the curve's cost
# parameters for any `AbstractReserve` with one (on-line, off-line and group reserves alike).
function handle_variable_cost_parameter(
    param::T,
    component::PSY.AbstractReserve,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    model::DecisionModel,
    initial_forecast_time,
    horizon,
    ts_type,
) where {T <: _AnyPiecewiseLinearParameter}
    offer_curve = PSY.get_variable(component)
    is_time_variant(offer_curve) || return
    container = get_optimization_container(model)
    ts_key = IS.get_time_series_key(PSY.get_value_curve(offer_curve))
    ts_name = _ts_name_from_key(component, ts_key)
    power_units = IS.get_power_units(offer_curve)
    raw_values = get_time_series_values!(
        ts_type,
        model,
        component,
        ts_name,
        initial_forecast_time,
        horizon,
    )
    additional_axes = lookup_additional_axes(parameter_array)
    n_blocks = _n_blocks(param, additional_axes)
    for (t, value::PSY.PiecewiseStepData) in enumerate(raw_values)
        unwrapped_value = unwrap_for_param(T(), value, additional_axes)
        _set_param_value!(parameter_array, unwrapped_value, name, t)
        update_variable_cost!(
            param,
            container,
            _pad_to_blocks(value, n_blocks),
            parameter_multiplier,
            attributes,
            component,
            t,
            power_units,
        )
    end
    return
end

# A `FuelCurve`'s time-varying fuel cost lives in `fuel_cost_time_series` (mutually
# exclusive with the fixed `fuel_cost` field); a plain `CostCurve` has no fuel cost at all.
_has_time_variant_fuel_cost(fuel_curve::PSY.FuelCurve) =
    is_time_variant(PSY.get_fuel_cost_time_series(fuel_curve))
_has_time_variant_fuel_cost(::PSY.CostCurve) = false

function handle_variable_cost_parameter(
    ::FuelCostParameter,
    op_cost::PSY.ThermalGenerationCost,
    component,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    model::DecisionModel,
    initial_forecast_time,
    horizon,
    ts_type,
)
    fuel_curve = PSY.get_variable_operation_cost(op_cost)
    _has_time_variant_fuel_cost(fuel_curve) || return
    # No formulation sets uses_compact_power=true on a FuelCostParameter's attributes today;
    # error loudly rather than silently resolving to an undefined conversion if that changes.
    IOM.get_uses_compact_power(attributes) && error(
        "Compact-power fuel cost conversion is not implemented for $(name); " *
        "no current formulation sets uses_compact_power=true for FuelCostParameter.",
    )
    container = get_optimization_container(model)
    device_model = get_model(get_template(model), typeof(component))
    ts_name = IOM.get_time_series_names(device_model)[FuelCostParameter]
    raw_values = get_time_series_values!(
        ts_type,
        model,
        component,
        ts_name,
        initial_forecast_time,
        horizon,
    )
    _update_fuel_cost_values!(
        container,
        raw_values,
        parameter_array,
        parameter_multiplier,
        attributes,
        component,
        fuel_curve,
        name,
    )
    return
end

# Function barrier: `raw_values` comes back from the time-series read abstractly typed, so the
# per-step loop is specialized here on its concrete runtime element type.
function _update_fuel_cost_values!(
    container::OptimizationContainer,
    raw_values,
    parameter_array,
    parameter_multiplier,
    attributes,
    component,
    fuel_curve,
    name,
)
    for (t, value) in enumerate(raw_values)
        _set_param_value!(parameter_array, value, name, t)
        update_variable_cost!(
            FuelCostParameter(),
            container,
            parameter_array,
            parameter_multiplier,
            attributes,
            component,
            fuel_curve,
            t,
        )
    end
    return
end

_linear_block_param(::Type{IncrementalPiecewiseLinearSlopeParameter}) =
    PiecewiseLinearBlockIncrementalOffer
_linear_block_param(::Type{DecrementalPiecewiseLinearSlopeParameter}) =
    PiecewiseLinearBlockDecrementalOffer

function _update_pwl_cost_expression(
    ::P,
    container::OptimizationContainer,
    ::Type{T},
    component_name::String,
    time_period::Int,
    cost_data::PSY.PiecewiseStepData,
) where {P <: AbstractPiecewiseLinearSlopeParameter, T <: PSY.Component}
    pwl_var_container = get_variable(container, _linear_block_param(P), T)
    resolution = get_resolution(container)
    dt = Dates.value(resolution) / MILLISECONDS_IN_HOUR
    gen_cost = JuMP.AffExpr(0.0)
    slopes = PSY.get_y_coords(cost_data)
    for i in 1:length(cost_data)
        JuMP.add_to_expression!(
            gen_cost,
            slopes[i] * dt,
            pwl_var_container[(component_name, i, time_period)],
        )
    end
    return gen_cost
end

# For multi-start variables, we need to get a subset of the parameter
_index_into_param(cost_data, ::T) where {T <: Union{StartVariable, MultiStartVariable}} =
    start_up_cost(cost_data, T)
_index_into_param(cost_data, ::VariableType) = cost_data

get_update_multiplier(::DecrementalCostAtMinParameter) = -1.0
get_update_multiplier(::IncrementalCostAtMinParameter) = 1.0
get_update_multiplier(::ObjectiveFunctionParameter) = 1.0

# Mirrors the per-component decomposition done at build time, so recurrent solves
# update the same constituent expression that contributed to ProductionCostExpression.
_constituent_cost_expression(::StartupCostParameter) = StartUpCostExpression
_constituent_cost_expression(::ShutdownCostParameter) = ShutDownCostExpression
_constituent_cost_expression(::AbstractCostAtMinParameter) = FixedCostExpression

# General case
function update_variable_cost!(
    parameter::ObjectiveFunctionParameter,
    container::OptimizationContainer,
    parameter_array::DenseAxisArray{T},
    parameter_multiplier::JuMPFloatArray,
    attributes::CostFunctionAttributes{T},
    component::U,
    time_period::Int,
) where {T, U <: PSY.Component}
    component_name = PSY.get_name(component)
    cost_data = parameter_array[component_name, time_period]
    mult_ = parameter_multiplier[component_name, time_period]
    mult2 = get_update_multiplier(parameter)
    constituent_type = _constituent_cost_expression(parameter)
    for MyVariableType in get_variable_types(attributes)
        variable = get_variable(container, MyVariableType, U)
        my_cost_data = _index_into_param(cost_data, MyVariableType())
        iszero(my_cost_data) && continue
        cost_expr = variable[component_name, time_period] * my_cost_data * mult_ * mult2
        add_to_objective_variant_expression!(container, cost_expr)
        set_expression!(
            container,
            ProductionCostExpression,
            cost_expr,
            component,
            time_period,
        )
        set_expression!(container, constituent_type, cost_expr, component, time_period)
    end
    return
end

get_update_multiplier(::IncrementalPiecewiseLinearSlopeParameter) = 1.0
get_update_multiplier(::DecrementalPiecewiseLinearSlopeParameter) = -1.0

# Special case for PiecewiseStepData
function update_variable_cost!(
    slope_param::AbstractPiecewiseLinearSlopeParameter,
    container::OptimizationContainer,
    function_data::PSY.PiecewiseStepData,
    parameter_multiplier::JuMPFloatArray,
    ::CostFunctionAttributes,
    component::T,
    time_period::Int,
    power_units::IS.AbstractUnitSystem,
) where {T <: PSY.Component}
    component_name = PSY.get_name(component)
    # TODO handle per-tranche multiplier if necessary
    mult_ = 1.0 # parameter_multiplier[component_name, time_period, 1]
    mult2 = get_update_multiplier(slope_param)
    converted_data = get_piecewise_curve_per_system_unit(
        function_data,
        power_units,
        get_model_base_power(container),
        PSY.get_base_power(component),
    )
    gen_cost =
        _update_pwl_cost_expression(
            slope_param,
            container,
            T,
            component_name,
            time_period,
            converted_data,
        )
    signed_cost = mult2 * mult_ * gen_cost
    add_to_objective_variant_expression!(container, signed_cost)
    set_expression!(
        container,
        ProductionCostExpression,
        signed_cost,
        component,
        time_period,
    )
    set_expression!(container, FuelCostExpression, signed_cost, component, time_period)
    return
end

# Special case for fuel cost
function update_variable_cost!(
    ::FuelCostParameter,
    container::OptimizationContainer,
    parameter_array::JuMPFloatArray,
    parameter_multiplier::JuMPFloatArray,
    ::CostFunctionAttributes{Float64},
    component::T,
    fuel_curve::PSY.FuelCurve,
    time_period::Int,
) where {T <: PSY.Component}
    component_name = PSY.get_name(component)
    fuel_cost = parameter_array[component_name, time_period]
    iszero(fuel_cost) && return
    mult_ = parameter_multiplier[component_name, time_period]
    expression = get_expression(container, FuelConsumptionExpression, T)
    cost_expr = expression[component_name, time_period] * fuel_cost * mult_
    add_to_objective_variant_expression!(container, cost_expr)
    set_expression!(container, ProductionCostExpression, cost_expr, component, time_period)
    set_expression!(container, FuelCostExpression, cost_expr, component, time_period)
    return
end
