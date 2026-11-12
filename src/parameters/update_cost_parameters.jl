# The update edits the objective and rows in place only if the model does not rebuild.
# A rebuilt model reads its parameter values in the next build. The old build can lack
# the terms of a new step's curves, for example a bid offered only on a later day.
# A released model has an empty JuMP model.
_edits_in_place(container::OptimizationContainer) =
    !IOM.get_rebuild_model(IOM.get_settings(container))

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
    _edits_in_place(container) && is_synchronized(container) &&
        error(
            "Cannot update $(T) on a synchronized container; objective expressions would " *
            "be re-augmented and double-counted",
        )
    system = get_system(model)
    ts_type = get_deterministic_time_series_type(system)
    # The build made rows only for components with the series; availability can change since.
    for name in axes(parameter_array, 1)
        component = PSY.get_component(V, system, name)
        isnothing(component) && error(
            "Component $(V) named '$(name)' is no longer in the system; " *
            "cannot update parameter $(T)",
        )
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
    _edits_in_place(container) && is_synchronized(container) &&
        error(
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

# Pad a shorter curve with zero-width, zero-slope blocks, so no block keeps an older curve's width.
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

function _converted_piecewise_data(
    container::OptimizationContainer,
    function_data::PSY.PiecewiseStepData,
    component::PSY.Component,
    power_units::IS.AbstractUnitSystem,
)
    return get_piecewise_curve_per_system_unit(
        function_data,
        power_units,
        get_model_base_power(container),
        PSY.get_base_power(component),
    )
end

_linear_block_offer_constraint(::Type{IncrementalPiecewiseLinearBreakpointParameter}) =
    PiecewiseLinearBlockIncrementalOfferConstraint
_linear_block_offer_constraint(::Type{DecrementalPiecewiseLinearBreakpointParameter}) =
    PiecewiseLinearBlockDecrementalOfferConstraint

_update_pwl_offsets!(::AbstractPiecewiseLinearSlopeParameter, args...) = nothing

"""
Move the block-offer rows' offsets to the new first breakpoints. IOM's `update_pwl_offset!`
shares the build's offset logic, so the update matches `add_pwl_constraint_delta!`.
"""
function _update_pwl_offsets!(
    ::P,
    container::OptimizationContainer,
    component::T,
    raw_values,
    power_units::IS.AbstractUnitSystem,
    ::Type{D},
) where {
    P <: AbstractPiecewiseLinearBreakpointParameter,
    T <: PSY.Component,
    D <: AbstractDeviceFormulation,
}
    component_name = PSY.get_name(component)
    rows = get_constraint(container, _linear_block_offer_constraint(P), T)
    U = _linked_power_variable(container, rows[component_name, 1], T, component_name, 1)
    _update_pwl_offset_rows!(container, rows, component, raw_values, power_units, U, D)
    return
end

# Function barrier: `U` comes from a runtime search of the container, so the per-step loop
# is specialized here on the concrete variable type.
function _update_pwl_offset_rows!(
    container::OptimizationContainer,
    rows,
    component::PSY.Component,
    raw_values,
    power_units::IS.AbstractUnitSystem,
    ::Type{U},
    ::Type{D},
) where {U <: VariableType, D <: AbstractDeviceFormulation}
    component_name = PSY.get_name(component)
    for (t, value::PSY.PiecewiseStepData) in enumerate(raw_values)
        converted = _converted_piecewise_data(container, value, component, power_units)
        IOM.update_pwl_offset!(
            container,
            rows[component_name, t],
            component,
            U,
            D,
            t,
            first(PSY.get_x_coords(converted)),
        )
    end
    return
end

# The attributes name only the block variables: find the `T` variable with coefficient one.
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
    ts_name = _cost_ts_name(param, component, op_cost)
    raw_values = get_time_series_values!(
        ts_type,
        model,
        component,
        ts_name,
        initial_forecast_time,
        horizon,
    )
    _update_scalar_cost_values!(param, raw_values, parameter_array, name)
    return
end

# Function barrier: `raw_values` comes back from the time-series read abstractly typed, so the
# per-step loop is specialized here on its concrete runtime element type.
function _update_scalar_cost_values!(
    param::T,
    raw_values,
    parameter_array,
    name,
) where {
    T <: Union{StartupCostParameter, ShutdownCostParameter, AbstractCostAtMinParameter},
}
    additional_axes = lookup_additional_axes(parameter_array)
    for (t, raw_value) in enumerate(raw_values)
        value = unwrap_for_param(param, raw_value, additional_axes)
        _set_param_value!(parameter_array, _maybe_tuple(param, value), name, t)
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
    _update_piecewise_cost_values!(
        param,
        container,
        raw_values,
        parameter_array,
        parameter_multiplier,
        attributes,
        component,
        name,
        power_units,
    )
    _edits_in_place(container) || return
    _update_pwl_offsets!(param, container, component, raw_values, power_units, formulation)
    return
end

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
    _update_piecewise_cost_values!(
        param,
        container,
        raw_values,
        parameter_array,
        parameter_multiplier,
        attributes,
        component,
        name,
        power_units,
    )
    return
end

# Function barrier, as in `_update_scalar_cost_values!`.
function _update_piecewise_cost_values!(
    param::T,
    container::OptimizationContainer,
    raw_values,
    parameter_array,
    parameter_multiplier,
    attributes,
    component,
    name,
    power_units::IS.AbstractUnitSystem,
) where {T <: _AnyPiecewiseLinearParameter}
    additional_axes = lookup_additional_axes(parameter_array)
    n_blocks = _n_blocks(param, additional_axes)
    for (t, value::PSY.PiecewiseStepData) in enumerate(raw_values)
        unwrapped_value = unwrap_for_param(T(), value, additional_axes)
        _set_param_value!(parameter_array, unwrapped_value, name, t)
    end
    _edits_in_place(container) || return
    for (t, value::PSY.PiecewiseStepData) in enumerate(raw_values)
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
    _update_fuel_cost_values!(raw_values, parameter_array, name)
    return
end

# Function barrier: `raw_values` comes back from the time-series read abstractly typed, so the
# per-step loop is specialized here on its concrete runtime element type.
function _update_fuel_cost_values!(raw_values, parameter_array, name)
    for (t, value) in enumerate(raw_values)
        _set_param_value!(parameter_array, value, name, t)
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
    IOM.add_variant_cost_to_expression!(
        container, ProductionCostExpression, signed_cost, T, component_name, time_period)
    return
end

"""
Re-add the time-variant scalar cost terms after the reset that precedes an in-place update.
Calls the build's own writers, which read the parameter values just written, so the update
keeps the build's `dt` scaling, must-run skips and no-load terms.
"""
function rewrite_variant_costs!(model::IOM.AbstractOptimizationModel)
    container = get_optimization_container(model)
    system = get_system(model)
    for device_model in values(IOM.get_device_models(get_template(model)))
        _rewrite_variant_costs!(container, device_model, system)
    end
    return
end

_rewrite_variant_costs!(::OptimizationContainer, ::DeviceModel, ::PSY.System) = nothing

function _rewrite_variant_costs!(
    container::OptimizationContainer,
    ::DeviceModel{T, D},
    system::PSY.System,
) where {T <: PSY.Device, D <: AbstractDeviceFormulation}
    if has_container_key(container, StartupCostParameter, T)
        devices = _built_components(container, StartupCostParameter, T, system)
        for U in _cost_variable_types(container, StartupCostParameter, T)
            IOM.add_start_up_cost!(container, U, devices, D)
        end
    end
    if has_container_key(container, ShutdownCostParameter, T)
        devices = _built_components(container, ShutdownCostParameter, T, system)
        for U in _cost_variable_types(container, ShutdownCostParameter, T)
            IOM.add_shut_down_cost!(container, U, devices, D)
        end
    end
    # Cost at min and a fuel price both price the OnVariable term; write it once per device.
    on_devices = T[]
    for P in
        (IncrementalCostAtMinParameter, DecrementalCostAtMinParameter, FuelCostParameter)
        has_container_key(container, P, T) || continue
        append!(on_devices, _built_components(container, P, T, system))
    end
    unique!(on_devices)
    filter!(d -> IOM.is_time_variant_proportional(PSY.get_operation_cost(d)), on_devices)
    if !isempty(on_devices) && has_container_key(container, OnVariable, T)
        POM.add_proportional_cost!(container, OnVariable, on_devices, D)
    end
    if has_container_key(container, FuelCostParameter, T)
        for d in _built_components(container, FuelCostParameter, T, system)
            fuel_curve = PSY.get_variable_operation_cost(PSY.get_operation_cost(d))
            IOM._add_time_varying_fuel_variable_cost!(
                container, ActivePowerVariable, d,
                PSY.get_fuel_cost_time_series(fuel_curve))
        end
    end
    return
end

# Components with a row in the parameter `P` of `T`.
_built_components(
    container::OptimizationContainer,
    ::Type{P},
    ::Type{T},
    system::PSY.System,
) where {P, T} =
    T[
        PSY.get_component(T, system, name) for
        name in axes(get_parameter_array(container, P, T), 1)
    ]

_cost_variable_types(container::OptimizationContainer, ::Type{P}, ::Type{T}) where {P, T} =
    IOM.get_variable_types(get_parameter_attributes(container, ParameterKey(P, T)))
