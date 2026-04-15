"""
Convert the internal `Dates.Millisecond` interval (where `UNSET_INTERVAL` means
unset) to the `Union{Nothing, Dates.Period}` form the IS / PSY time-series API
expects. Internal hot paths carry a concrete `Dates.Millisecond` to stay
type-stable; this helper performs the boundary conversion.
"""
_to_is_interval(interval::Dates.Millisecond) =
    interval == UNSET_INTERVAL ? nothing : interval

function get_available_components(
    model::DeviceModel{T, <:AbstractDeviceFormulation},
    sys::PSY.System,
) where {T <: PSY.Component}
    subsystem = get_subsystem(model)
    filter_function = get_attribute(model, "filter_function")
    if filter_function === nothing
        return PSY.get_components(
            PSY.get_available,
            T,
            sys;
            subsystem_name = subsystem,
        )
    else
        return PSY.get_components(
            x -> PSY.get_available(x) && filter_function(x),
            T,
            sys;
            subsystem_name = subsystem,
        )
    end
end

function get_available_components(
    model::ServiceModel{T, <:AbstractServiceFormulation},
    sys::PSY.System,
) where {T <: PSY.Component}
    subsystem = get_subsystem(model)
    filter_function = get_attribute(model, "filter_function")
    if filter_function === nothing
        return PSY.get_components(
            PSY.get_available,
            T,
            sys;
            subsystem_name = subsystem,
        )
    else
        return PSY.get_components(
            x -> PSY.get_available(x) && filter_function(x),
            T,
            sys;
            subsystem_name = subsystem,
        )
    end
end

_filter_function(x::PSY.ACBus) =
    PSY.get_bustype(x) != PSY.ACBusTypes.ISOLATED && PSY.get_available(x)

function get_available_components(
    model::NetworkModel,
    ::Type{PSY.ACBus},
    sys::PSY.System,
)
    subsystem = get_subsystem(model)
    return PSY.get_components(
        _filter_function,
        PSY.ACBus,
        sys;
        subsystem_name = subsystem,
    )
end

function get_available_components(
    model::NetworkModel,
    ::Type{T},
    sys::PSY.System,
) where {T <: PSY.Component}
    subsystem = get_subsystem(model)
    return PSY.get_components(
        T,
        sys;
        subsystem_name = subsystem,
    )
end

#=
function get_available_components(
    ::Type{PSY.RegulationDevice{T}},
    sys::PSY.System,
) where {T <: PSY.Component}
    return PSY.get_components(
        x -> (PSY.get_available(x) && PSY.has_service(x, PSY.AGC)),
        PSY.RegulationDevice{T},
        sys,
    )
end
=#

make_system_filename(sys::PSY.System) = make_system_filename(IS.get_uuid(sys))
make_system_filename(sys_uuid::Union{Base.UUID, AbstractString}) = "system-$(sys_uuid).json"

function check_hvdc_line_limits_consistency(
    d::Union{PSY.TwoTerminalHVDC, PSY.TModelHVDCLine},
)
    from_min = PSY.get_active_power_limits_from(d).min
    to_min = PSY.get_active_power_limits_to(d).min
    from_max = PSY.get_active_power_limits_from(d).max
    to_max = PSY.get_active_power_limits_to(d).max

    if from_max < to_min
        throw(
            IS.ConflictingInputsError(
                "From Max $(from_max) can't be a smaller value than To Min $(to_min)",
            ),
        )
    elseif to_max < from_min
        throw(
            IS.ConflictingInputsError(
                "To Max $(to_max) can't be a smaller value than From Min $(from_min)",
            ),
        )
    end
    return
end

function check_hvdc_line_limits_unidirectional(d::PSY.TwoTerminalHVDC)
    from_min = PSY.get_active_power_limits_from(d).min
    to_min = PSY.get_active_power_limits_to(d).min
    from_max = PSY.get_active_power_limits_from(d).max
    to_max = PSY.get_active_power_limits_to(d).max

    if from_min < 0 || to_min < 0 || from_max < 0 || to_max < 0
        throw(
            IS.ConflictingInputsError(
                "Changing flow direction on HVDC Line $(PSY.get_name(d)) is not compatible with non-linear network formulations. \
                Bi-directional models with losses are only compatible with linear network models like DCPPowerModel.",
            ),
        )
    end
    return
end

##################################################
########### Cost Function Utilities ##############
##################################################

"""
Obtain proportional (marginal or slope) cost data in system base per unit
depending on the specified power units
"""
function get_proportional_cost_per_system_unit(
    cost_term::Float64,
    unit_system::PSY.UnitSystem,
    system_base_power::Float64,
    device_base_power::Float64,
)
    return _get_proportional_cost_per_system_unit(
        cost_term,
        Val{unit_system}(),
        system_base_power,
        device_base_power,
    )
end

function _get_proportional_cost_per_system_unit(
    cost_term::Float64,
    ::Val{PSY.UnitSystem.SYSTEM_BASE},
    system_base_power::Float64,
    device_base_power::Float64,
)
    return cost_term
end

function _get_proportional_cost_per_system_unit(
    cost_term::Float64,
    ::Val{PSY.UnitSystem.DEVICE_BASE},
    system_base_power::Float64,
    device_base_power::Float64,
)
    return cost_term * (system_base_power / device_base_power)
end

function _get_proportional_cost_per_system_unit(
    cost_term::Float64,
    ::Val{PSY.UnitSystem.NATURAL_UNITS},
    system_base_power::Float64,
    device_base_power::Float64,
)
    return cost_term * system_base_power
end

"""
Obtain quadratic cost data in system base per unit
depending on the specified power units
"""
function get_quadratic_cost_per_system_unit(
    cost_term::Float64,
    unit_system::PSY.UnitSystem,
    system_base_power::Float64,
    device_base_power::Float64,
)
    return _get_quadratic_cost_per_system_unit(
        cost_term,
        Val{unit_system}(),
        system_base_power,
        device_base_power,
    )
end

function _get_quadratic_cost_per_system_unit(
    cost_term::Float64,
    ::Val{PSY.UnitSystem.SYSTEM_BASE}, # SystemBase Unit
    system_base_power::Float64,
    device_base_power::Float64,
)
    return cost_term
end

function _get_quadratic_cost_per_system_unit(
    cost_term::Float64,
    ::Val{PSY.UnitSystem.DEVICE_BASE}, # DeviceBase Unit
    system_base_power::Float64,
    device_base_power::Float64,
)
    return cost_term * (system_base_power / device_base_power)^2
end

function _get_quadratic_cost_per_system_unit(
    cost_term::Float64,
    ::Val{PSY.UnitSystem.NATURAL_UNITS}, # Natural Units
    system_base_power::Float64,
    device_base_power::Float64,
)
    return cost_term * system_base_power^2
end

"""
Obtain the normalized PiecewiseLinear cost data in system base per unit
depending on the specified power units.

Note that the costs (y-axis) are always in \$/h so
they do not require transformation
"""
function get_piecewise_pointcurve_per_system_unit(
    cost_component::PSY.PiecewiseLinearData,
    unit_system::PSY.UnitSystem,
    system_base_power::Float64,
    device_base_power::Float64,
)
    return _get_piecewise_pointcurve_per_system_unit(
        cost_component,
        Val{unit_system}(),
        system_base_power,
        device_base_power,
    )
end

function _get_piecewise_pointcurve_per_system_unit(
    cost_component::PSY.PiecewiseLinearData,
    ::Val{PSY.UnitSystem.SYSTEM_BASE},
    system_base_power::Float64,
    device_base_power::Float64,
)
    return cost_component
end

function _get_piecewise_pointcurve_per_system_unit(
    cost_component::PSY.PiecewiseLinearData,
    ::Val{PSY.UnitSystem.DEVICE_BASE},
    system_base_power::Float64,
    device_base_power::Float64,
)
    points = cost_component.points
    points_normalized = Vector{NamedTuple{(:x, :y)}}(undef, length(points))
    for (ix, point) in enumerate(points)
        points_normalized[ix] =
            (x = point.x * (device_base_power / system_base_power), y = point.y)
    end
    return PSY.PiecewiseLinearData(points_normalized)
end

function _get_piecewise_pointcurve_per_system_unit(
    cost_component::PSY.PiecewiseLinearData,
    ::Val{PSY.UnitSystem.NATURAL_UNITS},
    system_base_power::Float64,
    device_base_power::Float64,
)
    points = cost_component.points
    points_normalized = Vector{NamedTuple{(:x, :y)}}(undef, length(points))
    for (ix, point) in enumerate(points)
        points_normalized[ix] = (x = point.x / system_base_power, y = point.y)
    end
    return PSY.PiecewiseLinearData(points_normalized)
end

"""
Obtain the normalized PiecewiseStepData in system base per unit depending on the specified
power units.

Note that the costs (y-axis) are in \$/MWh, \$/(sys pu h) or \$/(device pu h), so they also
require transformation.
"""
function get_piecewise_curve_per_system_unit(
    cost_component::PSY.PiecewiseStepData,
    unit_system::PSY.UnitSystem,
    system_base_power::Float64,
    device_base_power::Float64,
)
    return PSY.PiecewiseStepData(
        get_piecewise_curve_per_system_unit(
            PSY.get_x_coords(cost_component),
            PSY.get_y_coords(cost_component),
            unit_system,
            system_base_power,
            device_base_power,
        )...,
    )
end

function get_piecewise_curve_per_system_unit(
    x_coords::AbstractVector,
    y_coords::AbstractVector,
    unit_system::PSY.UnitSystem,
    system_base_power::Float64,
    device_base_power::Float64,
)
    return _get_piecewise_curve_per_system_unit(
        x_coords,
        y_coords,
        Val{unit_system}(),
        system_base_power,
        device_base_power,
    )
end

function _get_piecewise_curve_per_system_unit(
    x_coords::AbstractVector,
    y_coords::AbstractVector,
    ::Val{PSY.UnitSystem.SYSTEM_BASE},
    system_base_power::Float64,
    device_base_power::Float64,
)
    return x_coords, y_coords
end

function _get_piecewise_curve_per_system_unit(
    x_coords::AbstractVector,
    y_coords::AbstractVector,
    ::Val{PSY.UnitSystem.DEVICE_BASE},
    system_base_power::Float64,
    device_base_power::Float64,
)
    ratio = device_base_power / system_base_power
    x_coords_normalized = x_coords .* ratio
    y_coords_normalized = y_coords ./ ratio
    return x_coords_normalized, y_coords_normalized
end

function _get_piecewise_curve_per_system_unit(
    x_coords::AbstractVector,
    y_coords::AbstractVector,
    ::Val{PSY.UnitSystem.NATURAL_UNITS},
    system_base_power::Float64,
    device_base_power::Float64,
)
    x_coords_normalized = x_coords ./ system_base_power
    y_coords_normalized = y_coords .* system_base_power
    return x_coords_normalized, y_coords_normalized
end

is_time_variant(::IS.TimeSeriesKey) = true
is_time_variant(::Any) = false

function create_temporary_cost_function_in_system_per_unit(
    original_cost_function::PSY.CostCurve,
    new_data::PSY.PiecewiseLinearData,
)
    return PSY.CostCurve(
        PSY.PiecewisePointCurve(new_data),
        PSY.UnitSystem.SYSTEM_BASE,
        PSY.get_vom_cost(original_cost_function),
    )
end

function create_temporary_cost_function_in_system_per_unit(
    original_cost_function::PSY.FuelCurve,
    new_data::PSY.PiecewiseLinearData,
)
    return PSY.FuelCurve(
        PSY.PiecewisePointCurve(new_data),
        PSY.UnitSystem.SYSTEM_BASE,
        PSY.get_fuel_cost(original_cost_function),
        PSY.LinearCurve(0.0),  # setting fuel offtake cost to default value of 0
        PSY.get_vom_cost(original_cost_function),
    )
end

"""
Return the set of distinct forecast intervals present in the system.
"""
function get_forecast_intervals(sys::PSY.System)
    table = PSY.get_forecast_summary_table(sys)
    return Set(row.interval for row in eachrow(table) if row.interval !== nothing)
end

"""
Automatically transform `SingleTimeSeries` into `DeterministicSingleTimeSeries` for a
given (horizon, interval) when a DecisionModel is built with these settings and the
system contains only static time series. Uses `delete_existing=false` so multiple
models may share the same system with different transforms.

Does nothing when:

  - The model's `horizon` or `interval` are unset.
  - The system has no `SingleTimeSeries` to transform.
  - The system already has forecast data (auto-transform only applies to pure SingleTimeSeries systems).
"""
function auto_transform_time_series!(sys::PSY.System, settings::Settings)
    model_interval = get_interval(settings)
    model_horizon = get_horizon(settings)
    if model_interval == UNSET_INTERVAL || model_horizon == UNSET_HORIZON
        return
    end

    counts = PSY.get_time_series_counts(sys)
    if counts.static_time_series_count < 1
        return
    end
    if counts.forecast_count > 0 && model_interval in get_forecast_intervals(sys)
        return
    end

    model_resolution = get_resolution(settings)
    resolution_kwarg =
        model_resolution == UNSET_RESOLUTION ? (;) : (; resolution = model_resolution)

    @info "Auto-transforming SingleTimeSeries to DeterministicSingleTimeSeries" horizon =
        Dates.canonicalize(model_horizon) interval = Dates.canonicalize(model_interval)
    PSY.transform_single_time_series!(
        sys,
        model_horizon,
        model_interval;
        delete_existing = false,
        resolution_kwarg...,
    )
    return
end

function get_deterministic_time_series_type(sys::PSY.System)
    time_series_types = IS.get_time_series_counts_by_type(sys.data)
    existing_types = Set(d["type"] for d in time_series_types)
    if Set(["Deterministic", "DeterministicSingleTimeSeries"]) ∈ existing_types
        error(
            "The System contains a combination of forecast data and transformed time series data. Currently this is not supported.",
        )
    end
    if "Deterministic" ∈ existing_types
        return PSY.Deterministic
    elseif "DeterministicSingleTimeSeries" ∈ existing_types
        return PSY.DeterministicSingleTimeSeries
    else
        error(
            "The System does not contain any forecast data or transformed time series data.",
        )
    end
end
