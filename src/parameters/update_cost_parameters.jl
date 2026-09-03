# IOM's `OutputsByTime` inner constructor validates `column_names` against the stored
# `DenseAxisArray` via `_check_column_consistency`, but only has methods for 2-D arrays
# (component, time); the 3-D tranche-axis arrays this file's Incremental/Decremental
# PiecewiseLinear{Slope,Breakpoint}Parameter values are stored in (component, tranche, time)
# have no matching method, so reading a time-varying PWL cost parameter back out of a
# `Simulation`'s store errors in `src/simulation/decision_model_simulation_results.jl`'s
# `_get_store_value`. That file is out of this task's scope; teach IOM's generic function
# about the 3-D case here instead — mirrors its own 2-D method (`outputs_by_time.jl`), which
# already loops `cols` generically over however many axes it is given.
function IOM._check_column_consistency(
    data::SortedDict{Dates.DateTime, <:DenseAxisArray{Float64, 3}},
    cols::NTuple{2, Vector{String}},
)
    for val in values(data)
        for (i, col) in enumerate(cols)
            if axes(val)[i] != col
                error(
                    "Mismatch in DenseAxisArray axis $i column names: $(axes(val)[i]) $col",
                )
            end
        end
    end
    return
end

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
        )
    end
    return
end

function _update_service_cost_parameter_values!(
    parameter_array::DenseAxisArray,
    ::T,
    parameter_multiplier::JuMPFloatArray,
    attributes::CostFunctionAttributes,
    ::Type{V},
    model::DecisionModel,
    ::DatasetContainer{InMemoryDataset},
    service_name::String,
) where {T <: ObjectiveFunctionParameter, V <: PSY.Service}
    initial_forecast_time = get_current_time(model)
    time_steps = get_time_steps(get_optimization_container(model))
    horizon = time_steps[end]
    container = get_optimization_container(model)
    is_synchronized(container) && error(
        "Cannot update $(T) on a synchronized container; objective expressions would " *
        "be re-augmented and double-counted",
    )
    service = PSY.get_component(V, get_system(model), service_name)
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
    )
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

# Slope and breakpoint parameters share the same raw time series (a `PiecewiseStepData`
# carries both x- and y-coordinates) and so share the read/unwrap/set-parameter-value path
# below; only the slope parameter's resolved value additionally contributes an objective
# term (breakpoints re-parametrize constraint bounds elsewhere, contributing nothing to the
# objective directly), so `update_variable_cost!` is a no-op for breakpoint parameters.
const _AnyPiecewiseLinearParameter =
    Union{AbstractPiecewiseLinearSlopeParameter, AbstractPiecewiseLinearBreakpointParameter}
update_variable_cost!(::AbstractPiecewiseLinearBreakpointParameter, args...) = nothing

# Mirrors POM's build-time `IOM._get_parameter_field(::Type{<:T}, op_cost)` dispatch table
# (PowerOperationsModels.jl/src/common_models/market_bid_plumbing.jl) one-for-one, but keyed
# on a parameter INSTANCE rather than a Type: update time always has a live parameter object
# in hand (never just its Type), so this is a genuinely different call convention from
# build time's, not a redundant copy of it. Only the 1-arg (raw field, no time resolution)
# form is needed here; per-step values are read from the underlying time series directly
# (see `_cost_ts_key`/`_cost_ts_name` below) rather than through PSY's curve-object-resolving
# convenience accessors, matching how POM's build-time parameter population reads them.
IOM._get_parameter_field(::StartupCostParameter, op_cost::PSY.OfferCurveCost) =
    PSY.get_start_up(op_cost)
IOM._get_parameter_field(::ShutdownCostParameter, op_cost::PSY.OfferCurveCost) =
    PSY.get_shut_down(op_cost)
IOM._get_parameter_field(::IncrementalCostAtMinParameter, op_cost::PSY.OfferCurveCost) =
    IS.get_initial_input(PSY.get_value_curve(get_output_offer_curves(op_cost)))
IOM._get_parameter_field(::DecrementalCostAtMinParameter, op_cost::PSY.OfferCurveCost) =
    IS.get_initial_input(PSY.get_value_curve(get_input_offer_curves(op_cost)))
IOM._get_parameter_field(
    ::Union{
        IncrementalPiecewiseLinearSlopeParameter,
        IncrementalPiecewiseLinearBreakpointParameter,
    },
    op_cost::PSY.OfferCurveCost,
) = get_output_offer_curves(op_cost)
IOM._get_parameter_field(
    ::Union{
        DecrementalPiecewiseLinearSlopeParameter,
        DecrementalPiecewiseLinearBreakpointParameter,
    },
    op_cost::PSY.OfferCurveCost,
) = get_input_offer_curves(op_cost)

# `PSY.StartUpStages` (a plain `NamedTuple`) is the static, non-time-varying `start_up`
# field of `MarketBidCost`; it carries no time-series backing of its own (the time-varying
# counterpart is a bare `StartUpStagesKey`, already covered by `IS.is_time_series_backed`).
# IOM's generic `is_time_variant` doesn't know about PSY types, so teach it here — mirrors
# how POM teaches `is_time_variant_proportional` about PSY cost types
# (market_bid_plumbing.jl).
IOM.is_time_variant(::PSY.StartUpStages) = false

_maybe_tuple(::StartupCostParameter, value) = Tuple(value)
_maybe_tuple(::ShutdownCostParameter, value) = value
_maybe_tuple(::AbstractCostAtMinParameter, value) = value

# Time-series key backing a parameter's raw cost-object field, and the time-series NAME
# resolved from it. Mirrors POM's build-time `_get_time_series_name` /
# `_device_offer_curve_ts_key` (PowerOperationsModels.jl/src/common_models/add_parameters.jl)
# exactly, reusing its private `_ts_name_from_key` rather than re-deriving it, so a name
# resolves identically whether the parameter is being built or refreshed between steps.
_cost_ts_key(
    param::Union{StartupCostParameter, AbstractCostAtMinParameter},
    op_cost::PSY.OfferCurveCost,
) = IOM._get_parameter_field(param, op_cost)
_cost_ts_key(param::ShutdownCostParameter, op_cost::PSY.OfferCurveCost) =
    IS.get_time_series_key(IOM._get_parameter_field(param, op_cost))
_cost_ts_key(param::_AnyPiecewiseLinearParameter, op_cost::PSY.OfferCurveCost) =
    IS.get_time_series_key(PSY.get_value_curve(IOM._get_parameter_field(param, op_cost)))

_cost_ts_name(param, owner, op_cost::PSY.OfferCurveCost) =
    POM._ts_name_from_key(owner, _cost_ts_key(param, op_cost))

function handle_variable_cost_parameter(
    param::Union{StartupCostParameter, ShutdownCostParameter, AbstractCostAtMinParameter},
    op_cost::PSY.OfferCurveCost,
    component,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    model::DecisionModel,
    initial_forecast_time,
    horizon,
)
    is_time_variant(IOM._get_parameter_field(param, op_cost)) || return
    container = get_optimization_container(model)
    ts_type = get_deterministic_time_series_type(get_system(model))
    ts_name = _cost_ts_name(param, component, op_cost)
    raw_values = IOM.get_time_series_values!(
        ts_type,
        model,
        component,
        ts_name,
        initial_forecast_time,
        horizon,
    )
    for (t, raw_value) in enumerate(raw_values)
        value = unwrap_for_param(param, raw_value, lookup_additional_axes(parameter_array))
        # startup needs Tuple(value), rest just value. (slight type instability)
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
) where {T <: _AnyPiecewiseLinearParameter}
    is_time_variant(IOM._get_parameter_field(param, op_cost)) || return
    container = get_optimization_container(model)
    ts_type = get_deterministic_time_series_type(get_system(model))
    ts_name = _cost_ts_name(param, component, op_cost)
    raw_values = IOM.get_time_series_values!(
        ts_type,
        model,
        component,
        ts_name,
        initial_forecast_time,
        horizon,
    )
    for (t, value::PSY.PiecewiseStepData) in enumerate(raw_values)
        unwrapped_value =
            unwrap_for_param(T(), value, lookup_additional_axes(parameter_array))
        _set_param_value!(parameter_array, unwrapped_value, name, t)
        update_variable_cost!(
            param,
            container,
            value,  # intentionally passing the PiecewiseStepData here, not the unwrapped
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
    component::PSY.OnlineReserve,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    model::DecisionModel,
    initial_forecast_time,
    horizon,
) where {T <: _AnyPiecewiseLinearParameter}
    offer_curve = PSY.get_variable(component)
    is_time_variant(offer_curve) || return
    container = get_optimization_container(model)
    ts_type = get_deterministic_time_series_type(get_system(model))
    ts_key = IS.get_time_series_key(PSY.get_value_curve(offer_curve))
    ts_name = POM._ts_name_from_key(component, ts_key)
    raw_values = IOM.get_time_series_values!(
        ts_type,
        model,
        component,
        ts_name,
        initial_forecast_time,
        horizon,
    )
    for (t, value::PSY.PiecewiseStepData) in enumerate(raw_values)
        unwrapped_value =
            unwrap_for_param(T(), value, lookup_additional_axes(parameter_array))
        _set_param_value!(parameter_array, unwrapped_value, name, t)
        update_variable_cost!(
            param,
            container,
            value,
            parameter_multiplier,
            attributes,
            component,
            t,
        )
    end
    return
end

function handle_variable_cost_parameter(
    ::FuelCostParameter,
    op_cost::PSY.ThermalGenerationCost,
    component,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    container,
    initial_forecast_time,
    horizon,
)
    fuel_curve = PSY.get_variable(op_cost)
    # Nothing to update for this component if we don't have a fuel cost time series
    (fuel_curve isa PSY.FuelCurve && is_time_variant(PSY.get_fuel_cost(fuel_curve))) ||
        return

    ts_vector = PSY.get_fuel_cost(
        component;
        start_time = initial_forecast_time,
        len = horizon,
    )
    fuel_cost_forecast_values = TimeSeries.values(ts_vector)
    for (t, value) in enumerate(fuel_cost_forecast_values)
        # `_convert_variable_cost` split a pre-psy6 `PSY.VariableCost` into no-load and
        # compact-power components; it was removed when PSY moved to ValueCurve/FunctionData
        # cost types and never replaced (see `main` history, commit 70e6a3109). No formulation
        # in IOM/POM sets `uses_compact_power = true` on a `FuelCostParameter`'s
        # `CostFunctionAttributes` today (`add_param_container!` always defaults it to
        # `false`), so this branch is unreachable. Error loudly instead of resolving to an
        # undefined function if that ever changes.
        attributes.uses_compact_power && error(
            "Compact-power fuel cost conversion is not implemented for $(name); " *
            "no current formulation sets uses_compact_power=true for FuelCostParameter.",
        )
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
) where {T <: PSY.Component}
    component_name = PSY.get_name(component)
    # TODO handle per-tranche multiplier if necessary
    mult_ = 1.0 # parameter_multiplier[component_name, time_period, 1]
    mult2 = get_update_multiplier(slope_param)
    converted_data = get_piecewise_curve_per_system_unit(
        function_data,
        IS.NaturalUnit(),  # PSY's cost_function_timeseries.jl says this will always be natural units
        IOM.get_model_base_power(container),
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
    add_to_objective_variant_expression!(container, mult2 * mult_ * gen_cost)
    set_expression!(container, ProductionCostExpression, gen_cost, component, time_period)
    set_expression!(container, FuelCostExpression, gen_cost, component, time_period)
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
    if all(iszero.(last.(fuel_cost)))
        return
    end
    mult_ = parameter_multiplier[component_name, time_period]
    expression = get_expression(container, FuelConsumptionExpression, T)
    cost_expr = expression[component_name, time_period] * fuel_cost * mult_
    add_to_objective_variant_expression!(container, cost_expr)
    set_expression!(container, ProductionCostExpression, cost_expr, component, time_period)
    set_expression!(container, FuelCostExpression, cost_expr, component, time_period)
    return
end
