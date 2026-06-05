const PM_MAP_TUPLE =
    NamedTuple{(:from_to, :to_from), Tuple{Tuple{Int, Int, Int}, Tuple{Int, Int, Int}}}

const PM_BUSTYPES = Dict{PSY.ACBusTypes, Int}(
    PSY.ACBusTypes.ISOLATED => 4,
    PSY.ACBusTypes.PQ => 1,
    PSY.ACBusTypes.PV => 2,
    PSY.ACBusTypes.REF => 3,
    PSY.ACBusTypes.SLACK => 3,
)

struct PMmap
    bus::Dict{Int, PSY.ACBus}
    arcs::Dict{Tuple{Int, Int}, PM_MAP_TUPLE}
    arcs_dc::Dict{PM_MAP_TUPLE, PSY.TwoTerminalHVDC}
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.PhaseShiftingTransformer,
    ::Type{PhaseAngleControl},
    ::Type{<:PM.AbstractDCPModel},
    ::DeviceModel,
)
    # we allocate the transformer shunt values to primary side only
    yt = PSY.get_primary_shunt(branch)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "rate_a" => PSY.get_rating(branch),
        "shift" => PSY.get_α(branch),
        "rate_b" => PSY.get_rating(branch),
        "br_x" => PSY.get_x(branch),
        "rate_c" => PSY.get_rating(branch),
        "g_to" => 0.0,
        "b_to" => 0.0,
        "g_fr" => real(yt),
        "b_fr" => imag(yt),
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => 0.0, # Turn off the branch while keeping the function type stable
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => PSY.get_tap(branch),
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.PhaseShiftingTransformer,
    ::Type{D},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
) where {D <: AbstractBranchFormulation}
    # we allocate the transformer shunt values to primary side only
    yt = PSY.get_primary_shunt(branch)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "rate_a" => PSY.get_rating(branch),
        "shift" => PSY.get_α(branch),
        "rate_b" => PSY.get_rating(branch),
        "br_x" => PSY.get_x(branch),
        "rate_c" => PSY.get_rating(branch),
        "g_to" => 0.0,
        "b_to" => 0.0,
        "g_fr" => real(yt),
        "b_fr" => imag(yt),
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => PSY.get_tap(branch),
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.PhaseShiftingTransformer,
    ::Type{StaticBranchUnbounded},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    # we allocate the transformer shunt values to primary side only
    yt = PSY.get_primary_shunt(branch)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "shift" => PSY.get_α(branch),
        "br_x" => PSY.get_x(branch),
        "g_to" => 0.0,
        "b_to" => 0.0,
        "g_fr" => real(yt),
        "b_fr" => imag(yt),
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => PSY.get_tap(branch),
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.Transformer2W,
    ::Type{<:AbstractBranchFormulation},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    # we allocate the transformer shunt values to primary side only
    yt = PSY.get_primary_shunt(branch)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "rate_a" => PSY.get_rating(branch),
        "shift" => 0.0,
        "rate_b" => PSY.get_rating(branch),
        "br_x" => PSY.get_x(branch),
        "rate_c" => PSY.get_rating(branch),
        "g_to" => 0.0,
        "b_to" => 0.0,
        "g_fr" => real(yt),
        "b_fr" => imag(yt),
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => 1.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.Transformer2W,
    ::Type{StaticBranchUnbounded},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    # we allocate the transformer shunt values to primary side only
    yt = PSY.get_primary_shunt(branch)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "shift" => 0.0,
        "br_x" => PSY.get_x(branch),
        "g_to" => 0.0,
        "b_to" => 0.0,
        "g_fr" => real(yt),
        "b_fr" => imag(yt),
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => 1.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.TapTransformer,
    ::Type{<:AbstractBranchFormulation},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    # we allocate the transformer shunt values to primary side only
    yt = PSY.get_primary_shunt(branch)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "rate_a" => PSY.get_rating(branch),
        "shift" => 0.0,
        "rate_b" => PSY.get_rating(branch),
        "br_x" => PSY.get_x(branch),
        "rate_c" => PSY.get_rating(branch),
        "g_to" => 0.0,
        "b_to" => 0.0,
        "g_fr" => real(yt),
        "b_fr" => imag(yt),
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => PSY.get_tap(branch),
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.TapTransformer,
    ::Type{StaticBranchUnbounded},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    # we allocate the transformer shunt values to primary side only
    yt = PSY.get_primary_shunt(branch)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "shift" => 0.0,
        "br_x" => PSY.get_x(branch),
        "g_to" => 0.0,
        "b_to" => 0.0,
        "g_fr" => real(yt),
        "b_fr" => imag(yt),
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => PSY.get_tap(branch),
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.ACTransmission,
    ::Type{<:AbstractBranchFormulation},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "rate_a" => PSY.get_rating(branch),
        "shift" => 0.0,
        "rate_b" => PSY.get_rating(branch),
        "br_x" => PSY.get_x(branch),
        "rate_c" => PSY.get_rating(branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PSY.get_b(branch).from,
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "b_to" => PSY.get_b(branch).to,
        "index" => ix,
        "angmin" => PSY.get_angle_limits(branch).min,
        "angmax" => PSY.get_angle_limits(branch).max,
        "transformer" => false,
        "tap" => 1.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    ::Tuple{Int, Int},
    branch::PSY.ACTransmission,
    ::Type{StaticBranchUnbounded},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    PM_branch = Dict{String, Any}(
        "br_r" => PSY.get_r(branch),
        "shift" => 0.0,
        "br_x" => PSY.get_x(branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PSY.get_b(branch).from,
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "b_to" => PSY.get_b(branch).to,
        "index" => ix,
        "angmin" => PSY.get_angle_limits(branch).min,
        "angmax" => PSY.get_angle_limits(branch).max,
        "transformer" => false,
        "tap" => 1.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    arc_tuple::Tuple{Int, Int},
    branch::PNM.ThreeWindingTransformerWinding{PSY.Transformer3W},
    ::Type{StaticBranchUnbounded},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    arc_tuple
    PM_branch = Dict{String, Any}(
        "br_r" => PNM.get_equivalent_r(branch),
        "shift" => 0.0,
        "br_x" => PNM.get_equivalent_x(branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PNM.get_equivalent_b(branch).from,
        "f_bus" => arc_tuple[1],
        "br_status" => Float64(PNM.get_equivalent_available(branch)),
        "t_bus" => arc_tuple[2],
        "b_to" => PNM.get_equivalent_b(branch).to,
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => 1.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    arc_tuple::Tuple{Int, Int},
    branch::PNM.ThreeWindingTransformerWinding{PSY.Transformer3W},
    ::Type{<:AbstractBranchFormulation},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    PM_branch = Dict{String, Any}(
        "br_r" => PNM.get_equivalent_r(branch),
        "rate_a" => PNM.get_equivalent_rating(branch),
        "shift" => 0.0,
        "rate_b" => PNM.get_equivalent_rating(branch),
        "br_x" => PNM.get_equivalent_x(branch),
        "rate_c" => PNM.get_equivalent_rating(branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PNM.get_equivalent_b(branch).from,
        "f_bus" => arc_tuple[1],
        "br_status" => Float64(PNM.get_equivalent_available(branch)),
        "t_bus" => arc_tuple[2],
        "b_to" => PNM.get_equivalent_b(branch).to,
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => 1.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    arc_tuple::Tuple{Int, Int},
    branch::PNM.ThreeWindingTransformerWinding{PSY.PhaseShiftingTransformer3W},
    ::Type{<:AbstractBranchFormulation},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    PM_branch = Dict{String, Any}(
        "br_r" => PNM.get_equivalent_r(branch),
        "rate_a" => PNM.get_equivalent_rating(branch),
        "shift" => 0.0,
        "rate_b" => PNM.get_equivalent_rating(branch),
        "br_x" => PNM.get_equivalent_x(branch),
        "rate_c" => PNM.get_equivalent_rating(branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PNM.get_equivalent_b(branch).from,
        "f_bus" => arc_tuple[1],
        "br_status" => Float64(PNM.get_equivalent_available(branch)),
        "t_bus" => arc_tuple[2],
        "b_to" => PNM.get_equivalent_b(branch).to,
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => PNM.get_equivalent_tap(branch),
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    arc_tuple::Tuple{Int, Int},
    branch::PNM.ThreeWindingTransformerWinding{PSY.PhaseShiftingTransformer3W},
    ::Type{StaticBranchUnbounded},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    PM_branch = Dict{String, Any}(
        "br_r" => PNM.get_equivalent_r(branch),
        "shift" => 0.0,
        "br_x" => PNM.get_equivalent_x(branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PNM.get_equivalent_b(branch).from,
        "f_bus" => arc_tuple[1],
        "br_status" => Float64(PNM.get_equivalent_available(branch)),
        "t_bus" => arc_tuple[2],
        "b_to" => PNM.get_equivalent_b(branch).to,
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => true,
        "tap" => PNM.get_equivalent_tap(branch),
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    arc_tuple::Tuple{Int, Int},
    double_circuit::PNM.AbstractBranchesParallel,
    T::Type{<:AbstractBranchFormulation},
    U::Type{<:PM.AbstractPowerModel},
    device_model::DeviceModel,
)
    equivalent_branch = PNM.get_equivalent_physical_branch_parameters(double_circuit)
    rating = _get_parallel_branch_max_rating(device_model, double_circuit)
    PM_branch = Dict{String, Any}(
        "br_r" => PNM.get_equivalent_r(equivalent_branch),
        "shift" => 0.0,
        "rate_a" => rating,
        "rate_b" => rating,
        "rate_c" => rating,
        "br_x" => PNM.get_equivalent_x(equivalent_branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PNM.get_equivalent_b_from(equivalent_branch),
        "f_bus" => arc_tuple[1],
        "br_status" => Float64(PNM.get_equivalent_available(double_circuit)),
        "t_bus" => arc_tuple[2],
        "b_to" => PNM.get_equivalent_b_to(equivalent_branch),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => false,
        "tap" => 1.0,
    )

    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    arc_tuple::Tuple{Int, Int},
    double_circuit::PNM.AbstractBranchesParallel,
    T::Type{StaticBranchUnbounded},
    U::Type{<:PM.AbstractPowerModel},
    device_model::DeviceModel,
)
    equivalent_branch = PNM.get_equivalent_physical_branch_parameters(double_circuit)
    rating = _get_parallel_branch_max_rating(device_model, double_circuit)
    PM_branch = Dict{String, Any}(
        "br_r" => PNM.get_equivalent_r(equivalent_branch),
        "shift" => 0.0,
        "br_x" => PNM.get_equivalent_x(equivalent_branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "rate_a" => rating,
        "rate_b" => rating,
        "rate_c" => rating,
        "b_fr" => PNM.get_equivalent_b_from(equivalent_branch),
        "f_bus" => arc_tuple[1],
        "br_status" => Float64(PNM.get_equivalent_available(double_circuit)),
        "t_bus" => arc_tuple[2],
        "b_to" => PNM.get_equivalent_b_to(equivalent_branch),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => false,
        "tap" => 1.0,
    )

    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    arc_tuple::Tuple{Int, Int},
    series_chain::PNM.BranchesSeries,
    T::Type{<:AbstractBranchFormulation},
    U::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    equivalent_branch = PNM.get_equivalent_physical_branch_parameters(series_chain)
    PM_branch = Dict{String, Any}(
        "br_r" => PNM.get_equivalent_r(equivalent_branch),
        "shift" => 0.0,
        "br_x" => PNM.get_equivalent_x(equivalent_branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PNM.get_equivalent_b_from(equivalent_branch),
        "f_bus" => arc_tuple[1],
        "br_status" => Float64(PNM.get_equivalent_available(series_chain)),
        "t_bus" => arc_tuple[2],
        "b_to" => PNM.get_equivalent_b_to(equivalent_branch),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => false,
        "tap" => 1.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    arc_tuple::Tuple{Int, Int},
    series_chain::PNM.BranchesSeries,
    T::Type{StaticBranchUnbounded},
    U::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    equivalent_branch = PNM.get_equivalent_physical_branch_parameters(series_chain)
    PM_branch = Dict{String, Any}(
        "br_r" => PNM.get_equivalent_r(equivalent_branch),
        "shift" => 0.0,
        "br_x" => PNM.get_equivalent_x(equivalent_branch),
        "g_to" => 0.0,
        "g_fr" => 0.0,
        "b_fr" => PNM.get_equivalent_b_from(equivalent_branch),
        "f_bus" => arc_tuple[1],
        "br_status" => Float64(PNM.get_equivalent_available(double_circuit)),
        "t_bus" => arc_tuple[2],
        "b_to" => PNM.get_equivalent_b_to(equivalent_branch),
        "index" => ix,
        "angmin" => -π / 2,
        "angmax" => π / 2,
        "transformer" => false,
        "tap" => 1.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    branch::PSY.TwoTerminalGenericHVDCLine,
    ::Type{HVDCTwoTerminalDispatch},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    check_hvdc_line_limits_unidirectional(branch)
    PM_branch = Dict{String, Any}(
        "loss1" => PSY.get_proportional_term(PSY.get_loss(branch)),
        "mp_pmax" => PSY.get_reactive_power_limits_from(branch).max,
        "model" => 2,
        "shutdown" => 0.0,
        "pmaxt" => PSY.get_active_power_limits_to(branch).max,
        "pmaxf" => PSY.get_active_power_limits_from(branch).max,
        "startup" => 0.0,
        "loss0" => PSY.get_constant_term(PSY.get_loss(branch)),
        "pt" => 0.0,
        "vt" => PSY.get_magnitude(PSY.get_arc(branch).to),
        "qmaxf" => PSY.get_reactive_power_limits_from(branch).max,
        "pmint" => PSY.get_active_power_limits_to(branch).min,
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "mp_pmin" => PSY.get_reactive_power_limits_from(branch).min,
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "qmint" => PSY.get_reactive_power_limits_to(branch).min,
        "qf" => 0.0,
        "cost" => 0.0,
        "pminf" => PSY.get_active_power_limits_from(branch).min,
        "qt" => 0.0,
        "qminf" => PSY.get_reactive_power_limits_from(branch).min,
        "vf" => PSY.get_magnitude(PSY.get_arc(branch).from),
        "qmaxt" => PSY.get_reactive_power_limits_to(branch).max,
        "ncost" => 0,
        "pf" => 0.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    branch::PSY.TwoTerminalGenericHVDCLine,
    ::Type{<:AbstractTwoTerminalDCLineFormulation},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    PM_branch = Dict{String, Any}(
        "loss1" => PSY.get_proportional_term(PSY.get_loss(branch)),
        "mp_pmax" => PSY.get_reactive_power_limits_from(branch).max,
        "model" => 2,
        "shutdown" => 0.0,
        "pmaxt" => PSY.get_active_power_limits_to(branch).max,
        "pmaxf" => PSY.get_active_power_limits_from(branch).max,
        "startup" => 0.0,
        "loss0" => PSY.get_constant_term(PSY.get_loss(branch)),
        "pt" => 0.0,
        "vt" => PSY.get_magnitude(PSY.get_arc(branch).to),
        "qmaxf" => PSY.get_reactive_power_limits_from(branch).max,
        "pmint" => PSY.get_active_power_limits_to(branch).min,
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "mp_pmin" => PSY.get_reactive_power_limits_from(branch).min,
        "br_status" => Float64(PSY.get_available(branch)),
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "qmint" => PSY.get_reactive_power_limits_to(branch).min,
        "qf" => 0.0,
        "cost" => 0.0,
        "pminf" => PSY.get_active_power_limits_from(branch).min,
        "qt" => 0.0,
        "qminf" => PSY.get_reactive_power_limits_from(branch).min,
        "vf" => PSY.get_magnitude(PSY.get_arc(branch).from),
        "qmaxt" => PSY.get_reactive_power_limits_to(branch).max,
        "ncost" => 0,
        "pf" => 0.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    branch::PSY.TwoTerminalGenericHVDCLine,
    ::Type{HVDCTwoTerminalDispatch},
    ::Type{<:PM.AbstractDCPModel},
    ::DeviceModel,
)
    PM_branch = Dict{String, Any}(
        "loss1" => PSY.get_proportional_term(PSY.get_loss(branch)),
        "mp_pmax" => PSY.get_reactive_power_limits_from(branch).max,
        "model" => 2,
        "shutdown" => 0.0,
        "pmaxt" => PSY.get_active_power_limits_to(branch).max,
        "pmaxf" => PSY.get_active_power_limits_from(branch).max,
        "startup" => 0.0,
        "loss0" => PSY.get_constant_term(PSY.get_loss(branch)),
        "pt" => 0.0,
        "vt" => PSY.get_magnitude(PSY.get_arc(branch).to),
        "qmaxf" => PSY.get_reactive_power_limits_from(branch).max,
        "pmint" => PSY.get_active_power_limits_to(branch).min,
        "f_bus" => PSY.get_number(PSY.get_arc(branch).from),
        "mp_pmin" => PSY.get_reactive_power_limits_from(branch).min,
        "br_status" => 0.0,
        "t_bus" => PSY.get_number(PSY.get_arc(branch).to),
        "index" => ix,
        "qmint" => PSY.get_reactive_power_limits_to(branch).min,
        "qf" => 0.0,
        "cost" => 0.0,
        "pminf" => PSY.get_active_power_limits_from(branch).min,
        "qt" => 0.0,
        "qminf" => PSY.get_reactive_power_limits_from(branch).min,
        "vf" => PSY.get_magnitude(PSY.get_arc(branch).from),
        "qmaxt" => PSY.get_reactive_power_limits_to(branch).max,
        "ncost" => 0,
        "pf" => 0.0,
    )
    return PM_branch
end

function get_branch_to_pm(
    ix::Int,
    branch::PSY.TwoTerminalLCCLine,
    ::Type{HVDCTwoTerminalLCC},
    ::Type{<:PM.AbstractPowerModel},
    ::DeviceModel,
)
    return Dict{String, Any}()
end

function get_branches_to_pm(
    sys::PSY.System,
    network_model::NetworkModel{S},
    ::Type{T},
    branch_template::BranchModelContainer,
    start_idx = 0,
) where {T <: PSY.ACTransmission, S <: PM.AbstractPowerModel}
    PM_branches = Dict{String, Any}()
    PMmap_br = Dict{Tuple{Int, Int}, PM_MAP_TUPLE}()
    net_reduction_data = get_network_reduction(network_model)
    all_branch_maps_by_type = net_reduction_data.all_branch_maps_by_type
    name_to_arc_maps = PNM.get_name_to_arc_maps(net_reduction_data)
    ix = 1
    @assert !isempty(branch_template)
    modeled_arc_tuples = Set{Tuple{Int, Int}}()
    for (d, device_model) in branch_template
        comp_type = get_component_type(device_model)
        if comp_type <: PSY.TwoTerminalHVDC || !haskey(name_to_arc_maps, comp_type)
            @info "No $d Branches to process in PowerModels data."
            continue
        end
        name_to_arc_map = PNM.get_name_to_arc_map(net_reduction_data, comp_type)
        for (_, (arc_tuple, reduction)) in name_to_arc_map
            arc_tuple ∈ modeled_arc_tuples && continue # This is the PowerModels equivalent of the branch and constraint tracker.
            reduction_entry = all_branch_maps_by_type[reduction][comp_type][arc_tuple]
            PM_branches["$(ix)"] = get_branch_to_pm(
                ix,
                arc_tuple,
                reduction_entry,
                get_formulation(device_model),
                S,
                device_model,
            )
            if PM_branches["$(ix)"]["br_status"] == true
                f = PM_branches["$(ix)"]["f_bus"]
                t = PM_branches["$(ix)"]["t_bus"]
                PMmap_br[arc_tuple] = (from_to = (ix, f, t), to_from = (ix, t, f))
            end
            push!(modeled_arc_tuples, arc_tuple)
            ix += 1
        end
    end
    return PM_branches, PMmap_br
end

function get_branches_to_pm(
    sys::PSY.System,
    network_model::NetworkModel{S},
    ::Type{T},
    branch_template::BranchModelContainer,
    start_idx = 0,
) where {T <: PSY.TwoTerminalHVDC, S <: PM.AbstractPowerModel}
    PM_branches = Dict{String, Any}()
    PMmap_br = Dict{PM_MAP_TUPLE, T}()

    for (d, device_model) in branch_template
        comp_type = get_component_type(device_model)
        !(comp_type <: T) && continue
        if comp_type <: PSY.TwoTerminalLCCLine &&
           get_formulation(device_model) <: HVDCTwoTerminalLCC
            continue
        end
        start_idx += length(PM_branches)
        for (i, branch) in enumerate(get_available_components(device_model, sys))
            ix = i + start_idx
            PM_branches["$(ix)"] = get_branch_to_pm(
                ix,
                branch,
                get_formulation(device_model),
                S,
                device_model,
            )
            if PM_branches["$(ix)"]["br_status"] == true
                f = PM_branches["$(ix)"]["f_bus"]
                t = PM_branches["$(ix)"]["t_bus"]
                PMmap_br[(from_to = (ix, f, t), to_from = (ix, t, f))] = branch
            end
        end
    end
    return PM_branches, PMmap_br
end

function get_buses_to_pm(buses::IS.FlattenIteratorWrapper{PSY.ACBus})
    PM_buses = Dict{String, Any}()
    PMmap_buses = Dict{Int, PSY.ACBus}()

    for bus in buses
        if PSY.get_bustype(bus) == PSY.ACBusTypes.ISOLATED
            continue
        end
        number = PSY.get_number(bus)
        PM_bus = Dict{String, Any}(
            "zone" => 1,
            "bus_i" => number,
            "bus_type" => PM_BUSTYPES[PSY.get_bustype(bus)],
            "vmax" => PSY.get_voltage_limits(bus).max,
            "area" => 1,
            "vmin" => PSY.get_voltage_limits(bus).min,
            "index" => PSY.get_number(bus),
            "va" => PSY.get_angle(bus),
            "vm" => PSY.get_magnitude(bus),
            "base_kv" => PSY.get_base_voltage(bus),
            "inj_p" => 0.0,
            "inj_q" => 0.0,
            "name" => PSY.get_name(bus),
        )
        PM_buses["$(number)"] = PM_bus
        PMmap_buses[number] = bus
    end
    return PM_buses, PMmap_buses
end

function pass_to_pm(sys::PSY.System, template::ProblemTemplate, time_periods::Int)
    ac_lines, PMmap_ac = get_branches_to_pm(
        sys,
        get_network_model(template),
        PSY.ACTransmission,
        template.branches,
    )
    two_terminal_dc_lines, PMmap_dc = get_branches_to_pm(
        sys,
        get_network_model(template),
        PSY.TwoTerminalHVDC,
        template.branches,
        length(ac_lines),
    )
    network_model = get_network_model(template)
    buses = get_available_components(network_model, PSY.ACBus, sys)
    pm_buses, PMmap_buses = get_buses_to_pm(buses)
    PM_translation = Dict{String, Any}(
        "bus" => pm_buses,
        "branch" => ac_lines,
        "baseMVA" => PSY.get_base_power(sys),
        "per_unit" => true,
        "storage" => Dict{String, Any}(),
        "dcline" => two_terminal_dc_lines,
        "gen" => Dict{String, Any}(),
        "switch" => Dict{String, Any}(),
        "shunt" => Dict{String, Any}(),
        "load" => Dict{String, Any}(),
    )

    # TODO: this function adds overhead in large number of time_steps
    # We can do better later.

    PM_translation = PM.replicate(PM_translation, time_periods)

    PM_map = PMmap(PMmap_buses, PMmap_ac, PMmap_dc)

    return PM_translation, PM_map
end
