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
    @assert !is_synchronized(container)
    template = get_template(model)
    device_model = get_model(template, V)
    components = get_available_components(device_model, get_system(model))
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
            container,
            initial_forecast_time,
            horizon,
        )
    end
    return
end

# We only support certain time series costs for PSY.OfferCurveCost, nothing to do for all the others
# We group them this way because we implement them that way: avoids method ambiguity issues
handle_variable_cost_parameter(
    ::Union{StartupCostParameter, ShutdownCostParameter, AbstractCostAtMinParameter},
    op_cost::PSY.OperationalCost, args...) = @assert !(op_cost isa PSY.OfferCurveCost)
handle_variable_cost_parameter(
    ::AbstractPiecewiseLinearSlopeParameter,
    op_cost::PSY.OperationalCost, args...) = @assert !(op_cost isa PSY.OfferCurveCost)

# typically used just with 1 arg, _get_parameter_field(T(), operation_cost).
_get_parameter_field(::StartupCostParameter, args...; kwargs...) =
    PSY.get_start_up(args...; kwargs...)
_get_parameter_field(::ShutdownCostParameter, args...; kwargs...) =
    PSY.get_shut_down(args...; kwargs...)
_get_parameter_field(::IncrementalCostAtMinParameter, args...; kwargs...) =
    PSY.get_incremental_initial_input(args...; kwargs...)
_get_parameter_field(::DecrementalCostAtMinParameter, args...; kwargs...) =
    PSY.get_decremental_initial_input(args...; kwargs...)
_get_parameter_field(
    ::Union{
        IncrementalPiecewiseLinearSlopeParameter,
        IncrementalPiecewiseLinearBreakpointParameter,
    },
    args...;
    kwargs...,
) =
    get_output_offer_curves(args...; kwargs...)
_get_parameter_field(
    ::Union{
        DecrementalPiecewiseLinearSlopeParameter,
        DecrementalPiecewiseLinearBreakpointParameter,
    },
    args...;
    kwargs...,
) =
    get_input_offer_curves(args...; kwargs...)

_maybe_tuple(::StartupCostParameter, value) = Tuple(value)
_maybe_tuple(::ShutdownCostParameter, value) = value
_maybe_tuple(::AbstractCostAtMinParameter, value) = value

function handle_variable_cost_parameter(
    param::Union{StartupCostParameter, ShutdownCostParameter, AbstractCostAtMinParameter},
    op_cost::PSY.OfferCurveCost,
    component,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    container,
    initial_forecast_time,
    horizon,
)
    is_time_variant(_get_parameter_field(param, op_cost)) || return
    ts_vector = _get_parameter_field(param, component, op_cost;
        start_time = initial_forecast_time,
        len = horizon,
    )
    for (t, value) in enumerate(TimeSeries.values(ts_vector))
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
    slope_param::T,
    op_cost::PSY.OfferCurveCost,
    component,
    name,
    parameter_array,
    parameter_multiplier,
    attributes,
    container,
    initial_forecast_time,
    horizon,
) where {T <: AbstractPiecewiseLinearSlopeParameter}
    is_time_variant(_get_parameter_field(slope_param, op_cost)) || return
    ts_vector = _get_parameter_field(slope_param,
        component, op_cost;
        start_time = initial_forecast_time,
        len = horizon,
    )
    for (t, value::PSY.PiecewiseStepData) in enumerate(TimeSeries.values(ts_vector))
        unwrapped_value =
            _unwrap_for_param(T(), value, lookup_additional_axes(parameter_array))
        _set_param_value!(parameter_array, unwrapped_value, name, t)
        update_variable_cost!(
            slope_param,
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
        # TODO: MBC Is this compact power attribute being used?
        if attributes.uses_compact_power
            # TODO implement this
            value, _ = _convert_variable_cost(value)
        end
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
    PiecewiseLinearBlockIncrementalOffer()
_linear_block_param(::Type{DecrementalPiecewiseLinearSlopeParameter}) =
    PiecewiseLinearBlockDecrementalOffer()

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
    start_up_cost(cost_data, T())
_index_into_param(cost_data, ::VariableType) = cost_data

get_update_multiplier(::DecrementalCostAtMinParameter) = -1.0
get_update_multiplier(::IncrementalCostAtMinParameter) = 1.0
get_update_multiplier(::ObjectiveFunctionParameter) = 1.0

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
    for MyVariableType in get_variable_types(attributes)
        variable = get_variable(container, MyVariableType(), U)
        my_cost_data = _index_into_param(cost_data, MyVariableType())
        iszero(my_cost_data) && continue
        cost_expr = variable[component_name, time_period] * my_cost_data * mult_ * mult2
        add_to_objective_variant_expression!(container, cost_expr)
        set_expression!(
            container,
            ProductionCostExpression, # for loads, this should be...?
            cost_expr,
            component,
            time_period,
        )
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
        PSY.UnitSystem.NATURAL_UNITS,  # PSY's cost_function_timeseries.jl says this will always be natural units
        get_base_power(container),
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
    expression = get_expression(container, FuelConsumptionExpression(), T)
    cost_expr = expression[component_name, time_period] * fuel_cost * mult_
    add_to_objective_variant_expression!(container, cost_expr)
    set_expression!(container, ProductionCostExpression, cost_expr, component, time_period)
    set_expression!(container, FuelCostExpression, cost_expr, component, time_period)
    return
end
