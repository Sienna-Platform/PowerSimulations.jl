function get_copied_line(
    line::PSY.Line,
)
    copied_line = Line(;
        name = PSY.get_name(line) * "_copy",
        available = PSY.get_available(line),
        active_power_flow = PSY.get_active_power_flow(line),
        reactive_power_flow = PSY.get_reactive_power_flow(line),
        arc = PSY.get_arc(line),
        r = PSY.get_r(line),
        x = PSY.get_x(line),
        b = PSY.get_b(line),
        rating = PSY.get_rating(line),
        angle_limits = PSY.get_angle_limits(line),
        rating_b = PSY.get_rating_b(line),
        rating_c = PSY.get_rating_c(line),
        g = PSY.get_g(line),
        services = PSY.get_services(line),
        ext = PSY.get_ext(line),
    )
    return copied_line
end

function get_copied_bus(
    bus::PSY.ACBus,
)
    copied_bus = ACBus(;
        number = PSY.get_number(bus) + 1000, #Add 1000 to avoid name conflicts
        name = PSY.get_name(bus) * "_copy",
        available = PSY.get_available(bus),
        bustype = ACBusTypes.PQ,
        angle = PSY.get_angle(bus),
        magnitude = PSY.get_magnitude(bus),
        voltage_limits = PSY.get_voltage_limits(bus),
        base_voltage = PSY.get_base_voltage(bus),
        area = PSY.get_area(bus),
        load_zone = PSY.get_load_zone(bus),
    )
    return copied_bus
end

function add_equivalent_ac_transmission_with_series_parallel_circuits!(
    sys::System,
    ac_transmission::PSY.Line,
    ::Type{T},
) where {T <: PSY.Line}

    #Create intermediate Bus
    old_arc = PSY.get_arc(ac_transmission)
    original_bus_to = PSY.get_to(old_arc)
    original_bus_from = PSY.get_from(old_arc)
    intermediate_bus = get_copied_bus(original_bus_to)
    add_component!(sys, intermediate_bus)

    #Remove old arc
    remove_component!(sys, old_arc)

    #add new Arcs
    arc1 = Arc(; from = original_bus_from, to = intermediate_bus)
    arc2 = Arc(; from = intermediate_bus, to = original_bus_to)
    add_component!(sys, arc1)
    add_component!(sys, arc2)
    #Update Arc original Line
    set_arc!(ac_transmission, arc1)

    #make Parallel circuits
    original_rating = PSY.get_rating(ac_transmission)
    original_r = PSY.get_r(ac_transmission)
    original_x = PSY.get_x(ac_transmission)
    rating_new_parallel = PSY.get_rating(ac_transmission) / 2
    new_r_parallel = PSY.get_r(ac_transmission) * 2
    new_x_parallel = PSY.get_x(ac_transmission) * 2
    ac_transmission_copy_parallel = get_copied_line(ac_transmission)
    ac_transmission_copy_series = get_copied_line(ac_transmission_copy_parallel)

    set_rating!(ac_transmission, rating_new_parallel)
    set_rating!(ac_transmission_copy_parallel, rating_new_parallel)
    set_r!(ac_transmission, original_r)
    set_r!(ac_transmission_copy_parallel, new_r_parallel)
    set_x!(ac_transmission, new_x_parallel)
    set_x!(ac_transmission_copy_parallel, new_x_parallel)
    add_component!(sys, ac_transmission_copy_parallel)

    #Add new series Line with same parameters
    set_arc!(ac_transmission_copy_series, arc2)
    add_component!(sys, ac_transmission_copy_series)
    set_x!(ac_transmission_copy_series, 1e-9)
    set_r!(ac_transmission_copy_series, 1e-9)
end

function add_equivalent_ac_transmission_with_parallel_circuits!(
    sys::System,
    ac_transmission::PSY.Line,
    ::Type{T},
) where {T <: PSY.Line}
    rating_new = PSY.get_rating(ac_transmission) / 2
    x_new = PSY.get_x(ac_transmission) * 2
    r_new = PSY.get_r(ac_transmission) * 2
    ac_transmission_copy_parallel = get_copied_line(ac_transmission)

    #Set ratings the half so the case remains equivalent to the original
    set_rating!(ac_transmission, rating_new)
    set_rating!(ac_transmission_copy_parallel, rating_new)
    set_x!(ac_transmission, x_new)
    set_x!(ac_transmission_copy_parallel, x_new)
    set_r!(ac_transmission, r_new)
    set_r!(ac_transmission_copy_parallel, r_new)
    add_component!(sys, ac_transmission_copy_parallel)
end

function add_equivalent_ac_transmission_with_parallel_circuits!(
    sys::System,
    ac_transmission::PSY.ACTransmission,
    ::Type{T},
    ::Type{PSY.MonitoredLine},
) where {T <: PSY.Line}
    rating_new = PSY.get_rating(ac_transmission) / 2
    x_new = PSY.get_x(ac_transmission) * 2
    r_new = PSY.get_r(ac_transmission) * 2
    ac_transmission_copy = MonitoredLine(;
        name = PSY.get_name(ac_transmission) * "_copy",
        available = PSY.get_available(ac_transmission),
        active_power_flow = PSY.get_active_power_flow(ac_transmission),
        reactive_power_flow = PSY.get_reactive_power_flow(ac_transmission),
        arc = PSY.get_arc(ac_transmission),
        r = PSY.get_r(ac_transmission),
        x = PSY.get_x(ac_transmission),
        b = PSY.get_b(ac_transmission),
        flow_limits = (
            from_to = rating_new,
            to_from = rating_new,
        ),
        rating = rating_new,
        angle_limits = PSY.get_angle_limits(ac_transmission),
        rating_b = PSY.get_rating_b(ac_transmission),
        rating_c = PSY.get_rating_c(ac_transmission),
        g = PSY.get_g(ac_transmission),
        services = PSY.get_services(ac_transmission),
        ext = PSY.get_ext(ac_transmission))

    #Set ratings the half so the case remains equivalent to the original
    set_rating!(ac_transmission, rating_new)
    set_x!(ac_transmission, x_new)
    set_r!(ac_transmission, r_new)
    add_component!(sys, ac_transmission_copy)
end

function add_reserve_product_without_requirement_time_series!(
    sys::PSY.System,
    name::String,
    direction::String,
    contributing_devices::Union{
        IS.FlattenIteratorWrapper{<:PSY.Generator},
        Vector{<:PSY.Generator},
    },
)
    AS_DIRECTION_MAP = Dict(
        "Up" => ReserveUp,
        "Down" => ReserveDown,
    )
    as_direction = AS_DIRECTION_MAP[direction]
    reserve_instance = VariableReserve{as_direction}(;
        name = name,
        available = true,
        time_frame = 0.0,
        requirement = 0.0,
        sustained_time = 3600,
        max_output_fraction = 1.0,
        max_participation_factor = 0.25,
        deployed_fraction = 0.0,
    )
    add_service!(sys, reserve_instance, contributing_devices)
end

"""
    add_renewable_dispatch_with_time_series!(
        sys,
        reference_generator,
        availability;
        name = "renewable_dispatch",
        curtailment_cost = 0.0,
    )

Adds a `RenewableDispatch` with the same static maximum active power as
`reference_generator` and an hourly deterministic `max_active_power` forecast.
`availability` values are per unit of the renewable's static maximum active
power.

# Arguments

    - `sys::PSY.System`: System that owns the thermal generator.
    - `reference_generator::PSY.ThermalStandard`: Generator that supplies the
        renewable's bus, static maximum active power, and base power.
  - `availability::Vector{Float64}`: Per-unit availability values.
  - `name::String`: Name assigned to the renewable generator.
    - `curtailment_cost::Float64`: Linear curtailment cost in \$/MWh.
"""
function add_renewable_dispatch_with_time_series!(
    sys::PSY.System,
    reference_generator::PSY.ThermalStandard,
    availability::Vector{Float64};
    name::String = "renewable_dispatch",
    curtailment_cost::Float64 = 0.0,
)
    renewable = PSY.RenewableDispatch(;
        name = name,
        available = PSY.get_available(reference_generator),
        bus = PSY.get_bus(reference_generator),
        active_power = PSY.get_active_power(reference_generator),
        reactive_power = PSY.get_reactive_power(reference_generator),
        rating = PSY.get_rating(reference_generator),
        prime_mover_type = PSY.PrimeMovers.PVe,
        reactive_power_limits = PSY.get_reactive_power_limits(reference_generator),
        power_factor = 0.9,
        operation_cost = PSY.RenewableGenerationCost(
            ;
            variable = PSY.CostCurve(PSY.LinearCurve(0.0)),
            curtailment_cost = PSY.CostCurve(PSY.LinearCurve(curtailment_cost)),
        ),
        base_power = PSY.get_base_power(reference_generator),
    )
    load = first(PSY.get_components(PSY.PowerLoad, sys))
    load_time_series = PSY.get_time_series(PSY.Deterministic, load, "max_active_power")
    availability_data = Dict(
        timestamp => availability for timestamp in keys(IS.get_data(load_time_series))
    )
    availability_forecast = PSY.Deterministic(
        "max_active_power",
        availability_data,
        Hour(1);
        scaling_factor_multiplier = PSY.get_max_active_power,
        interval = IS.get_interval(load_time_series),
    )
    PSY.add_component!(sys, renewable)
    PSY.add_time_series!(sys, renewable, availability_forecast)
    return renewable
end
