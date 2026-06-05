
# Note: Any future concrete formulation requires the definition of

# construct_device!(
#     ::OptimizationContainer,
#     ::PSY.System,
#     ::DeviceModel{<:PSY.ACTransmission, MyNewFormulation},
#     ::Union{Type{CopperPlatePowerModel}, Type{AreaBalancePowerModel}},
# ) = nothing

#

# Not implemented yet
# struct TapControl <: AbstractBranchFormulation end

#################################### Branch Variables ##################################################
# Because of the way we integrate with PowerModels, most of the time PowerSimulations will create variables
# for the branch flows either in AC or DC.

#! format: off
get_variable_binary(::FlowActivePowerVariable, ::Type{<:PSY.ACTransmission}, ::AbstractBranchFormulation,) = false
get_variable_binary(::PhaseShifterAngle, ::Type{PSY.PhaseShiftingTransformer}, ::AbstractBranchFormulation,) = false

get_parameter_multiplier(::FixValueParameter, ::PSY.ACTransmission, ::AbstractBranchFormulation) = 1.0
get_parameter_multiplier(::LowerBoundValueParameter, ::PSY.ACTransmission, ::AbstractBranchFormulation) = 1.0
get_parameter_multiplier(::UpperBoundValueParameter, ::PSY.ACTransmission, ::AbstractBranchFormulation) = 1.0

get_variable_multiplier(::PhaseShifterAngle, d::PSY.PhaseShiftingTransformer, ::PhaseAngleControl) = 1.0/PSY.get_x(d)

get_multiplier_value(::AbstractBranchRatingTimeSeriesParameter, d::PSY.ACTransmission, ::StaticBranch) = PSY.get_rating(d)
get_multiplier_value(::AbstractBranchRatingTimeSeriesParameter, d::PSY.ACTransmission, ::AbstractSecurityConstrainedStaticBranch) = PSY.get_rating(d)


get_initial_conditions_device_model(::OperationModel, ::DeviceModel{T, U}) where {T <: PSY.ACTransmission, U <: AbstractBranchFormulation} = DeviceModel(T, U)

#### Properties of slack variables
get_variable_binary(::FlowActivePowerSlackUpperBound, ::Type{<:PSY.ACTransmission}, ::AbstractBranchFormulation,) = false
get_variable_binary(::FlowActivePowerSlackLowerBound, ::Type{<:PSY.ACTransmission}, ::AbstractBranchFormulation,) = false
# These two methods are defined to avoid ambiguities
get_variable_upper_bound(::FlowActivePowerSlackUpperBound, ::PSY.ACTransmission, ::AbstractBranchFormulation) = nothing
get_variable_lower_bound(::FlowActivePowerSlackUpperBound, ::PSY.ACTransmission, ::AbstractBranchFormulation) = 0.0
get_variable_upper_bound(::FlowActivePowerSlackLowerBound, ::PSY.ACTransmission, ::AbstractBranchFormulation) = nothing
get_variable_lower_bound(::FlowActivePowerSlackLowerBound, ::PSY.ACTransmission, ::AbstractBranchFormulation) = 0.0
get_variable_upper_bound(::FlowActivePowerVariable, ::PNM.BranchesSeries, ::AbstractBranchFormulation) = nothing
get_variable_lower_bound(::FlowActivePowerVariable, ::PNM.BranchesSeries, ::AbstractBranchFormulation) = nothing
get_variable_upper_bound(::FlowActivePowerVariable, ::PNM.BranchesParallel, ::AbstractBranchFormulation) = nothing
get_variable_lower_bound(::FlowActivePowerVariable, ::PNM.BranchesParallel, ::AbstractBranchFormulation) = nothing
get_variable_upper_bound(::FlowActivePowerVariable, ::PNM.ThreeWindingTransformerWinding, ::AbstractBranchFormulation) = nothing
get_variable_lower_bound(::FlowActivePowerVariable, ::PNM.ThreeWindingTransformerWinding, ::AbstractBranchFormulation) = nothing

#! format: on
function get_default_time_series_names(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.ACTransmission, V <: AbstractBranchFormulation}
    return Dict{Type{<:TimeSeriesParameter}, String}()
end

"""
DeviceModel attribute key selecting which `PowerNetworkMatrices` function aggregates
the individual circuit ratings of a `PNM.BranchesParallel` into a single maximum flow
limit. Valid values: `"single_element_contingency"` (default; N-1, post-trip surviving
capacity), `"sum_of_max"` (plain Σ Sᵢ), `"impedance_averaged"` (susceptance-weighted
average). `PNM.MixedBranchesParallel` groups always use `sum_of_max`.
"""
const PARALLEL_BRANCH_MAX_RATING_KEY = "parallel_branch_max_rating_method"

function get_default_attributes(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.ACTransmission, V <: AbstractBranchFormulation}
    return Dict{String, Any}(PARALLEL_BRANCH_MAX_RATING_KEY => "single_element_contingency")
end

function get_default_attributes(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.ACTransmission, V <: AbstractSecurityConstrainedStaticBranch}
    return Dict{String, Any}(
        PARALLEL_BRANCH_MAX_RATING_KEY => "single_element_contingency",
        "include_planned_outages" => false,
    )
end

# Resolve the per-DeviceModel attribute to one of the explicit PNM rating functions.
# `MixedBranchesParallel` ignores the attribute and always uses the plain sum, since
# the constituent branches may carry different DeviceModel preferences and there is
# no defensible way to pick one.
function _get_parallel_branch_max_rating(model::DeviceModel, bp::PNM.BranchesParallel)
    name = get_attribute(model, PARALLEL_BRANCH_MAX_RATING_KEY)
    name == "single_element_contingency" &&
        return PNM.get_single_element_contingency_rating(bp)
    name == "sum_of_max" && return PNM.get_sum_of_max_rating(bp)
    name == "impedance_averaged" && return PNM.get_impedance_averaged_rating(bp)
    error(
        "Unknown $PARALLEL_BRANCH_MAX_RATING_KEY value: $(repr(name)). " *
        "Valid: \"single_element_contingency\", \"sum_of_max\", \"impedance_averaged\".",
    )
end

function _get_parallel_branch_max_rating(::DeviceModel, mbp::PNM.MixedBranchesParallel)
    return PNM.get_sum_of_max_rating(mbp)
end

# Parameter multiplier at build (`add_parameters.jl`). Non-branch-rating params
# use `get_multiplier_value`; branch-rating series use the same type-aware
# aggregation as the static `branch_rating` path. The parallel arms are the
# exception: a series on one member can't be split across the group. See the
# "Branch Rating Limits" explanation page.
_resolve_branch_multiplier(p, d, f, ::DeviceModel) = get_multiplier_value(p, d, f)

function _resolve_branch_multiplier(
    ::BranchRatingTimeSeriesParameter,
    d::PNM.AbstractBranchesParallel,
    ::Union{StaticBranch, AbstractSecurityConstrainedStaticBranch},
    ::DeviceModel,
)
    @warn "Parallel reduction $(PNM.get_name(d)) has a member with a branch rating \
           time series; using sum_of_max as the multiplier, regardless of the \
           `parallel_branch_max_rating_method` attribute."
    return PNM.get_sum_of_max_rating(d)
end

function _resolve_branch_multiplier(
    ::PostContingencyBranchRatingTimeSeriesParameter,
    d::PNM.AbstractBranchesParallel,
    ::Union{StaticBranch, AbstractSecurityConstrainedStaticBranch},
    ::DeviceModel,
)
    @warn "Parallel reduction $(PNM.get_name(d)) has a member with a \
           post-contingency branch rating time series; using the summed emergency \
           rating as the multiplier, regardless of the \
           `parallel_branch_max_rating_method` attribute."
    return PNM.get_equivalent_emergency_rating(d)
end

# Non-parallel entries: same aggregation as the static `branch_rating` path.
# Every PNM reduction wrapper is `<: PSY.ACTransmission`; the parallel methods
# above are more specific (`<: AbstractBranchesParallel`), so they win for groups.
function _resolve_branch_multiplier(
    ::BranchRatingTimeSeriesParameter,
    entry::PSY.ACTransmission,
    ::Union{StaticBranch, AbstractSecurityConstrainedStaticBranch},
    ::DeviceModel,
)
    return PNM.get_equivalent_rating(entry)
end

function _resolve_branch_multiplier(
    ::PostContingencyBranchRatingTimeSeriesParameter,
    entry::PSY.ACTransmission,
    ::Union{StaticBranch, AbstractSecurityConstrainedStaticBranch},
    ::DeviceModel,
)
    return PNM.get_equivalent_emergency_rating(entry)
end
#################################### Flow Variable Bounds ##################################################

# `AbstractPTDFModel <: PM.AbstractPowerModel`, so this single method also
# covers the PTDF network models; the PTDF `StaticBranchUnbounded` no-op
# override below is more specific on the formulation argument and still wins.
function add_variables!(
    container::OptimizationContainer,
    ::Type{T},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
    devices::IS.FlattenIteratorWrapper{U},
    formulation::AbstractBranchFormulation,
) where {T <: AbstractACActivePowerFlow, U <: PSY.ACTransmission}
    net_reduction_data = network_model.network_reduction
    time_steps = get_time_steps(container)
    branch_names = get_branch_argument_variable_axis(net_reduction_data, devices)
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    variable_container =
        add_variable_container!(container, T(), U, branch_names, time_steps)

    for (name, (arc, reduction)) in PNM.get_name_to_arc_map(net_reduction_data, U)
        reduction_entry = all_branch_maps_by_type[reduction][U][arc]
        has_entry, tracker_container =
            search_for_reduced_branch_argument!(reduced_branch_tracker, arc, T)
        if has_entry
            @assert !isempty(tracker_container) name arc reduction
        end
        ub = get_variable_upper_bound(T(), reduction_entry, formulation)
        lb = get_variable_lower_bound(T(), reduction_entry, formulation)
        for t in time_steps
            if !has_entry
                tracker_container[t] = JuMP.@variable(
                    get_jump_model(container),
                    base_name = "$(T)_$(U)_$(reduction)_{$(name), $(t)}",
                )
                !isnothing(ub) && JuMP.set_upper_bound(tracker_container[t], ub)
                !isnothing(lb) && JuMP.set_lower_bound(tracker_container[t], lb)
            end
            variable_container[name, t] = tracker_container[t]
        end
    end
    return
end

function add_variables!(
    ::OptimizationContainer,
    ::Type{T},
    network_model::NetworkModel{<:AbstractPTDFModel},
    devices::IS.FlattenIteratorWrapper{U},
    formulation::StaticBranchUnbounded,
) where {T <: AbstractACActivePowerFlow, U <: PSY.ACTransmission}
    @debug "PTDF Branch Flows with StaticBranchUnbounded do not require flow variables $T. Flow values are given by PTDFBranchFlow expression."
    return
end

function add_variables!(
    container::OptimizationContainer,
    ::Type{S},
    network_model::NetworkModel{CopperPlatePowerModel},
    devices::IS.FlattenIteratorWrapper{T},
    formulation::U,
) where {
    S <: AbstractACActivePowerFlow,
    T <: PSY.ACTransmission,
    U <: AbstractBranchFormulation,
}
    @debug "AC Branches of type $(T) do not require flow variables $S in CopperPlatePowerModel."
    return
end

function _get_flow_variable_vector(
    container::OptimizationContainer,
    ::NetworkModel{<:PM.AbstractDCPModel},
    ::Type{B},
) where {B <: PSY.ACTransmission}
    return [get_variable(container, FlowActivePowerVariable(), B)]
end

function _get_flow_variable_vector(
    container::OptimizationContainer,
    ::NetworkModel{<:PM.AbstractPowerModel},
    ::Type{B},
) where {B <: PSY.ACTransmission}
    return [
        get_variable(container, FlowActivePowerFromToVariable(), B),
        get_variable(container, FlowActivePowerToFromVariable(), B),
    ]
end

function branch_rate_bounds!(
    container::OptimizationContainer,
    device_model::DeviceModel{B, T},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
) where {B <: PSY.ACTransmission, T <: AbstractBranchFormulation}
    time_steps = get_time_steps(container)
    net_reduction_data = get_network_reduction(network_model)
    all_branch_maps_by_type = net_reduction_data.all_branch_maps_by_type
    for var in _get_flow_variable_vector(container, network_model, B)
        for (name, (arc, reduction)) in PNM.get_name_to_arc_map(net_reduction_data, B)
            # TODO: entry is not type stable here, it can return any type ACTransmission.
            # It might have performance implications. Possibly separate this into other functions
            reduction_entry = all_branch_maps_by_type[reduction][B][arc]
            # Use the same limit values as FlowRateConstraint for consistency.
            limits = min_max_flow_limits(reduction_entry, device_model)
            for t in time_steps
                @assert limits.min <= limits.max "Infeasible rate limits for branch $(name)"
                JuMP.set_upper_bound(var[name, t], limits.max)
                JuMP.set_lower_bound(var[name, t], limits.min)
            end
        end
    end
    return
end

################################## PWL Loss Variables ##################################

function _check_pwl_loss_model(devices)
    first_loss = PSY.get_loss(first(devices))
    first_loss_type = typeof(first_loss)
    for d in devices
        loss = PSY.get_loss(d)
        if !isa(loss, first_loss_type)
            error(
                "Not all TwoTerminal HVDC lines have the same loss model data. Check that all loss models are LinearCurve or PiecewiseIncrementalCurve",
            )
        end
        if isa(first_loss, PSY.PiecewiseIncrementalCurve)
            len_first_loss = length(PSY.get_slopes(first_loss))
            len_loss = length(PSY.get_slopes(loss))
            if len_first_loss != len_loss
                error(
                    "Different length of PWL segments for TwoTerminal HVDC losses are not supported. Check that all HVDC data have the same amount of PWL segments.",
                )
            end
        end
    end
    return
end

################################## Rate Limits constraint_infos ############################

"""
Scalar branch rating for a reduction entry — the single source of truth for
branch flow ratings. Parallel groups use the `PARALLEL_BRANCH_MAX_RATING_KEY`
attribute; every other entry uses `PNM.get_equivalent_rating`. Extend that (not
this) for new types. See the "Branch Rating Limits" explanation page.
"""
function branch_rating(double_circuit::PNM.AbstractBranchesParallel, model::DeviceModel)
    return _get_parallel_branch_max_rating(model, double_circuit)
end

function branch_rating(entry, ::DeviceModel)
    return PNM.get_equivalent_rating(entry)
end

"""
Symmetric `(min, max)` flow limits from [`branch_rating`](@ref). Prefer this
over the formulation-only `get_min_max_limits` when the `DeviceModel` is in
scope.
"""
function min_max_flow_limits(entry, model::DeviceModel)
    rating = branch_rating(entry, model)
    return (min = -rating, max = rating)
end

# `MonitoredLine` has explicit, possibly asymmetric `flow_limits`; defer to its
# own `get_min_max_limits` instead of the symmetric `branch_rating` path.
function min_max_flow_limits(device::PSY.MonitoredLine, ::DeviceModel)
    return get_min_max_limits(device, FlowRateConstraint, AbstractBranchFormulation)
end

# Formulation-typed adapter used by the range-constraint framework (e.g.
# `PhaseShiftingTransformer` under `FlowLimitConstraint`). `MonitoredLine`
# overrides this below.
function get_min_max_limits(
    device::PSY.ACTransmission,
    ::Type{<:ConstraintType},
    ::Type{<:AbstractBranchFormulation},
)
    rating = PNM.get_equivalent_rating(device)
    return (min = -rating, max = rating)
end

"""
Min and max limits for Abstract Branch Formulation
"""
function get_min_max_limits(
    ::PSY.PhaseShiftingTransformer,
    ::Type{PhaseAngleControlLimit},
    ::Type{PhaseAngleControl},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    return (min = -π / 2, max = π / 2)
end

function _add_flow_rate_constraint!(
    container::OptimizationContainer,
    arc::Tuple{Int, Int},
    use_slacks::Bool,
    con_lb::DenseAxisArray,
    con_ub::DenseAxisArray,
    var::DenseAxisArray,
    branch_maps_by_type::Dict,
    name::String,
    device_model::DeviceModel{T},
) where {T <: PSY.ACTransmission}
    reduction_entry = branch_maps_by_type[arc]
    time_steps = get_time_steps(container)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), T)[name, :]
        slack_lb = get_variable(container, FlowActivePowerSlackLowerBound(), T)[name, :]
    end
    limits = min_max_flow_limits(reduction_entry, device_model)
    for t in time_steps
        con_ub[name, t] = JuMP.@constraint(
            get_jump_model(container),
            var[name, t] - (use_slacks ? slack_ub[t] : 0.0) <= limits.max
        )
        con_lb[name, t] = JuMP.@constraint(
            get_jump_model(container),
            var[name, t] + (use_slacks ? slack_lb[t] : 0.0) >= limits.min
        )
    end
    return
end

"""
Add branch rate limit constraints for ACBranch with AbstractActivePowerModel
"""
function add_constraints!(
    container::OptimizationContainer,
    cons_type::Type{FlowRateConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, U},
    network_model::NetworkModel{V},
) where {
    T <: PSY.ACTransmission,
    U <: AbstractBranchFormulation,
    V <: PM.AbstractActivePowerModel,
}
    time_steps = get_time_steps(container)
    net_reduction_data = network_model.network_reduction
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    branch_names = get_branch_argument_constraint_axis(
        net_reduction_data,
        reduced_branch_tracker,
        devices,
        cons_type,
    )
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    con_lb = add_constraints_container!(
        container,
        cons_type(),
        T,
        branch_names,
        time_steps;
        meta = "lb",
    )
    con_ub = add_constraints_container!(
        container,
        cons_type(),
        T,
        branch_names,
        time_steps;
        meta = "ub",
    )

    array = get_variable(container, FlowActivePowerVariable(), T)

    use_slacks = get_use_slacks(device_model)
    # Gate on the parameter container actually existing, not merely on the
    # time-series name being configured: when the name is set but no branch of
    # this type carries the series, `add_parameters!` skips creating the
    # container and `get_parameter_array` would throw. An empty
    # `ts_branch_names` then routes every arc through the static-rating path,
    # which is the intended fallback. `name in ts_branch_names` is
    # self-sufficient at the call site.
    ts_branch_names = String[]
    if has_container_key(container, BranchRatingTimeSeriesParameter, T)
        ts_name = get_time_series_names(device_model)[BranchRatingTimeSeriesParameter]
        param = get_parameter_array(container, BranchRatingTimeSeriesParameter(), T)
        ts_branch_names = axes(param, 1)
    end

    for (name, (arc, reduction)) in
        get_constraint_map_by_type(reduced_branch_tracker)[FlowRateConstraint][T]
        if name in ts_branch_names
            _add_flow_rate_constraint_with_parameters!(
                container,
                T,
                arc,
                use_slacks,
                con_lb,
                con_ub,
                array,
                all_branch_maps_by_type[reduction][T],
                name,
                ts_name,
            )
        else
            _add_flow_rate_constraint!(
                container,
                arc,
                use_slacks,
                con_lb,
                con_ub,
                array,
                all_branch_maps_by_type[reduction][T],
                name,
                device_model,
            )
        end
    end
    return
end

function _add_flow_rate_constraint_with_parameters!(
    container::OptimizationContainer,
    ::Type{T},
    arc::Tuple{Int, Int},
    use_slacks::Bool,
    con_lb::DenseAxisArray,
    con_ub::DenseAxisArray,
    var::DenseAxisArray,
    branch_maps_by_type::Dict,
    name::String,
    ts_name::String,
) where {T <: PSY.ACTransmission}
    time_steps = get_time_steps(container)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), T)[name, :]
        slack_lb = get_variable(container, FlowActivePowerSlackLowerBound(), T)[name, :]
    end
    param_container = get_parameter(container, BranchRatingTimeSeriesParameter(), T)
    param = get_parameter_column_refs(param_container, name)
    mult = get_multiplier_array(param_container)[name, :]

    for t in time_steps
        @debug "Branch rating time series applied for branch $(name) at time step $(t)"
        con_ub[name, t] = JuMP.@constraint(
            get_jump_model(container),
            var[name, t] - (use_slacks ? slack_ub[t] : 0.0) <= param[t] * mult[t]
        )
        con_lb[name, t] = JuMP.@constraint(
            get_jump_model(container),
            var[name, t] + (use_slacks ? slack_lb[t] : 0.0) >= -1.0 * param[t] * mult[t]
        )
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    cons_type::Type{FlowRateConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, U},
    network_model::NetworkModel{V},
) where {T <: PSY.ACTransmission, U <: AbstractBranchFormulation, V <: AbstractPTDFModel}
    time_steps = get_time_steps(container)
    net_reduction_data = network_model.network_reduction
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    branch_names = get_branch_argument_constraint_axis(
        net_reduction_data,
        reduced_branch_tracker,
        devices,
        cons_type,
    )
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    con_lb = add_constraints_container!(
        container,
        cons_type(),
        T,
        branch_names,
        time_steps;
        meta = "lb",
    )
    con_ub = add_constraints_container!(
        container,
        cons_type(),
        T,
        branch_names,
        time_steps;
        meta = "ub",
    )

    array = get_expression(container, PTDFBranchFlow(), T)

    use_slacks = get_use_slacks(device_model)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), T)
        slack_lb = get_variable(container, FlowActivePowerSlackLowerBound(), T)
    end
    for (name, (arc, reduction)) in
        get_constraint_map_by_type(reduced_branch_tracker)[FlowRateConstraint][T]
        # TODO: entry is not type stable here, it can return any type ACTransmission.
        # It might have performance implications. Possibly separate this into other functions
        reduction_entry = all_branch_maps_by_type[reduction][T][arc]
        limits = min_max_flow_limits(reduction_entry, device_model)
        for t in time_steps
            con_ub[name, t] = JuMP.@constraint(
                get_jump_model(container),
                array[name, t] - (use_slacks ? slack_ub[name, t] : 0.0) <= limits.max
            )
            con_lb[name, t] = JuMP.@constraint(
                get_jump_model(container),
                array[name, t] + (use_slacks ? slack_lb[name, t] : 0.0) >= limits.min
            )
        end
    end
    return
end

function add_flow_rate_constraint_with_parameters!(
    container::OptimizationContainer,
    cons_type::Type{FlowRateConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, U},
    network_model::NetworkModel{V},
) where {T <: PSY.ACTransmission, U <: StaticBranch, V <: AbstractPTDFModel}
    time_steps = get_time_steps(container)
    net_reduction_data = network_model.network_reduction
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    # `get_constraint_map_by_type[FlowRateConstraint][T]` drops arcs already
    # claimed by another branch type's static FlowRateConstraint pass, which
    # would silently skip the TS bound on shared parallel arcs. Walk
    # `name_to_arc_map[T]` directly, matching `_add_time_series_parameters!`.
    name_to_arc_map = PNM.get_name_to_arc_map(net_reduction_data, T)
    branch_names = collect(keys(name_to_arc_map))

    con_lb = add_constraints_container!(
        container,
        cons_type(),
        T,
        branch_names,
        time_steps;
        meta = "lb",
    )
    con_ub = add_constraints_container!(
        container,
        cons_type(),
        T,
        branch_names,
        time_steps;
        meta = "ub",
    )

    var_array = get_expression(container, PTDFBranchFlow(), T)

    ts_name = get_time_series_names(device_model)[BranchRatingTimeSeriesParameter]
    ts_type = get_default_time_series_type(container)
    use_slacks = get_use_slacks(device_model)
    # Mark each arc as claimed so a later static `add_constraints!(FlowRateConstraint, ...)`
    # for a different branch type sharing the same reduced arc skips it. Without this,
    # a duplicate static cap could be added on top of the time-varying limit and the
    # static value would silently win whenever the TS multiplier exceeds it.
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    arc_tuples_with_constraints =
        get!(get_constraint_dict(reduced_branch_tracker), cons_type, Set{Tuple{Int, Int}}())
    for (name, (arc, reduction)) in name_to_arc_map
        branch_map_T = all_branch_maps_by_type[reduction][T]
        if PNM.has_time_series(branch_map_T[arc], ts_type, ts_name)
            _add_flow_rate_constraint_with_parameters!(
                container,
                T,
                arc,
                use_slacks,
                con_lb,
                con_ub,
                var_array,
                branch_map_T,
                name,
                ts_name,
            )
        else
            _add_flow_rate_constraint!(
                container,
                arc,
                use_slacks,
                con_lb,
                con_ub,
                var_array,
                branch_map_T,
                name,
                device_model,
            )
        end
        push!(arc_tuples_with_constraints, arc)
    end
    return
end

"""
Add rate limit from to constraints for ACBranch with AbstractPowerModel.

When `BranchRatingTimeSeriesParameter` is configured on the `device_model`,
the per-timestep rate is `parameter_value * multiplier` (where the multiplier
is the static rating); otherwise the static rating from the reduction entry
is used.
"""
function add_constraints!(
    container::OptimizationContainer,
    cons_type::Type{FlowRateConstraintFromTo},
    devices::IS.FlattenIteratorWrapper{B},
    device_model::DeviceModel{B, <:AbstractBranchFormulation},
    network_model::NetworkModel{T},
) where {B <: PSY.ACTransmission, T <: PM.AbstractPowerModel}
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    net_reduction_data = get_network_reduction(network_model)
    all_branch_maps_by_type = net_reduction_data.all_branch_maps_by_type
    device_names = get_branch_argument_constraint_axis(
        net_reduction_data,
        reduced_branch_tracker,
        devices,
        cons_type,
    )
    time_steps = get_time_steps(container)
    var1 = get_variable(container, FlowActivePowerFromToVariable(), B)
    var2 = get_variable(container, FlowReactivePowerFromToVariable(), B)
    add_constraints_container!(container, cons_type(), B, device_names, time_steps)
    constraint = get_constraint(container, cons_type(), B)

    use_slacks = get_use_slacks(device_model)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), B)
    end

    # Gate on the parameter container actually existing, not merely on the
    # time-series name being configured: when the name is set but no branch of
    # this type carries the series, `add_parameters!` skips creating the
    # container and `get_parameter_array` would throw. An empty
    # `ts_branch_names` then routes every arc through the static-rating path,
    # which is the intended fallback. `name in ts_branch_names` is
    # self-sufficient at the call site.
    ts_branch_names = String[]
    if has_container_key(container, BranchRatingTimeSeriesParameter, B)
        param = get_parameter_array(container, BranchRatingTimeSeriesParameter(), B)
        mult =
            get_parameter_multiplier_array(container, BranchRatingTimeSeriesParameter(), B)
        ts_branch_names = axes(param, 1)
    end

    for (name, (arc, reduction)) in
        get_constraint_map_by_type(reduced_branch_tracker)[FlowRateConstraintFromTo][B]
        # TODO: entry is not type stable here, it can return any type ACTransmission.
        # It might have performance implications. Possibly separate this into other functions
        reduction_entry = all_branch_maps_by_type[reduction][B][arc]
        # Per-name (not per-timestep): the TS membership does not depend on `t`.
        # The time-series `param * mult` is built to equal `rating^2` directly, so
        # it is NOT squared here; the static path squares the scalar rating.
        if name in ts_branch_names
            for t in time_steps
                constraint[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    var1[name, t]^2 + var2[name, t]^2 -
                    (use_slacks ? slack_ub[name, t] : 0.0) <=
                    param[name, t] * mult[name, t]
                )
            end
        else
            branch_rate = branch_rating(reduction_entry, device_model)
            for t in time_steps
                constraint[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    var1[name, t]^2 + var2[name, t]^2 -
                    (use_slacks ? slack_ub[name, t] : 0.0) <= branch_rate^2
                )
            end
        end
    end
    return
end

"""
Add rate limit to from constraints for ACBranch with AbstractPowerModel.

When `BranchRatingTimeSeriesParameter` is configured on the `device_model`,
the per-timestep rate is `parameter_value * multiplier` (where the multiplier
is the static rating); otherwise the static rating from the reduction entry
is used.
"""
function add_constraints!(
    container::OptimizationContainer,
    cons_type::Type{FlowRateConstraintToFrom},
    devices::IS.FlattenIteratorWrapper{B},
    device_model::DeviceModel{B, <:AbstractBranchFormulation},
    network_model::NetworkModel{T},
) where {B <: PSY.ACTransmission, T <: PM.AbstractPowerModel}
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    net_reduction_data = get_network_reduction(network_model)
    all_branch_maps_by_type = net_reduction_data.all_branch_maps_by_type
    time_steps = get_time_steps(container)
    device_names = get_branch_argument_constraint_axis(
        net_reduction_data,
        reduced_branch_tracker,
        devices,
        cons_type,
    )
    var1 = get_variable(container, FlowActivePowerToFromVariable(), B)
    var2 = get_variable(container, FlowReactivePowerToFromVariable(), B)
    add_constraints_container!(container, cons_type(), B, device_names, time_steps)
    constraint = get_constraint(container, cons_type(), B)
    use_slacks = get_use_slacks(device_model)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), B)
    end

    # Gate on the parameter container actually existing, not merely on the
    # time-series name being configured: when the name is set but no branch of
    # this type carries the series, `add_parameters!` skips creating the
    # container and `get_parameter_array` would throw. An empty
    # `ts_branch_names` then routes every arc through the static-rating path,
    # which is the intended fallback. `name in ts_branch_names` is
    # self-sufficient at the call site.
    ts_branch_names = String[]
    if has_container_key(container, BranchRatingTimeSeriesParameter, B)
        # In this case the value of the multiplier and the param need to equal to rating^2.
        # The updating needs to happen in a clever way to avoid performance issues. The param and multiplier are
        # stored separately to allow the time series to be updated without needing to rebuild the multiplier, which is more expensive to update since it requires updating all entries instead of just the ones in the time series.
        param = get_parameter_array(container, BranchRatingTimeSeriesParameter(), B)
        mult =
            get_parameter_multiplier_array(container, BranchRatingTimeSeriesParameter(), B)
        ts_branch_names = axes(param, 1)
    end

    for (name, (arc, reduction)) in
        get_constraint_map_by_type(reduced_branch_tracker)[FlowRateConstraintToFrom][B]
        # TODO: entry is not type stable here, it can return any type ACTransmission.
        # It might have performance implications. Possibly separate this into other functions
        reduction_entry = all_branch_maps_by_type[reduction][B][arc]
        # Per-name (not per-timestep): the TS membership does not depend on `t`.
        if name in ts_branch_names
            for t in time_steps
                constraint[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    var1[name, t]^2 + var2[name, t]^2 -
                    (use_slacks ? slack_ub[name, t] : 0.0) <=
                    param[name, t] * mult[name, t]
                )
            end
        else
            branch_rate = branch_rating(reduction_entry, device_model)
            for t in time_steps
                constraint[name, t] = JuMP.@constraint(
                    get_jump_model(container),
                    var1[name, t]^2 + var2[name, t]^2 -
                    (use_slacks ? slack_ub[name, t] : 0.0) <= branch_rate^2
                )
            end
        end
    end
    return
end

"""
Error if a PTDF/MODF column length differs from the nodal-balance bus
dimension. Prevents a downstream `@inbounds` out-of-bounds read; a mismatch
means the matrix and container used different network reductions.
"""
function _assert_flow_expression_dimensions(
    name::AbstractString,
    n_col::Int,
    nodal_balance_expressions::Matrix{JuMP.AffExpr},
)
    n_bus = size(nodal_balance_expressions, 1)
    if n_col != n_bus
        error(
            "Flow-expression dimension mismatch for branch/arc '$name': " *
            "PTDF/MODF column has $n_col entries but the nodal-balance " *
            "expression has $n_bus buses. PTDF and MODF must be built with " *
            "the same network reduction as the optimization container.",
        )
    end
    return
end

function _make_flow_expressions!(
    name::String,
    time_steps::UnitRange{Int},
    ptdf_col::Vector{Float64},
    nodal_balance_expressions::Matrix{JuMP.AffExpr},
)
    @debug "Making Flow Expression on thread $(Threads.threadid()) for branch $name"
    _assert_flow_expression_dimensions(name, length(ptdf_col), nodal_balance_expressions)
    nz_idx = [i for i in eachindex(ptdf_col) if abs(ptdf_col[i]) > PTDF_ZERO_TOL]
    hint = length(nz_idx)
    expressions = Vector{JuMP.AffExpr}(undef, length(time_steps))
    for t in time_steps
        acc = get_hinted_aff_expr(hint)
        @inbounds for i in nz_idx
            JuMP.add_to_expression!(acc, ptdf_col[i], nodal_balance_expressions[i, t])
        end
        expressions[t] = acc
    end
    return name, expressions
end

function _make_flow_expressions!(
    name::String,
    time_steps::UnitRange{Int},
    ptdf_col::SparseArrays.SparseVector{Float64, Int},
    nodal_balance_expressions::Matrix{JuMP.AffExpr},
)
    @debug "Making Flow Expression on thread $(Threads.threadid()) for branch $name"
    _assert_flow_expression_dimensions(name, length(ptdf_col), nodal_balance_expressions)
    nz_idx = SparseArrays.nonzeroinds(ptdf_col)
    nz_val = SparseArrays.nonzeros(ptdf_col)
    hint = length(nz_idx)
    expressions = Vector{JuMP.AffExpr}(undef, length(time_steps))
    for t in time_steps
        acc = get_hinted_aff_expr(hint)
        @inbounds for k in eachindex(nz_idx)
            JuMP.add_to_expression!(acc, nz_val[k], nodal_balance_expressions[nz_idx[k], t])
        end
        expressions[t] = acc
    end
    return name, expressions
end

function _add_expression_to_container!(
    branch_flow_expr::JuMPAffineExpressionDArrayStringInt,
    jump_model::JuMP.Model,
    time_steps::UnitRange{Int},
    ptdf_col::AbstractVector{Float64},
    nodal_balance_expressions::JuMPAffineExpressionDArrayIntInt,
    reduction_entry::T,
    branches::Vector{String},
) where {T <: PSY.ACTransmission}
    name = PSY.get_name(reduction_entry)
    if name in branches
        branch_flow_expr[name, :] .= _make_flow_expressions!(
            name,
            time_steps,
            ptdf_col,
            nodal_balance_expressions.data,
        )
    end
    return
end

function _add_expression_to_container!(
    branch_flow_expr::JuMPAffineExpressionDArrayStringInt,
    jump_model::JuMP.Model,
    time_steps::UnitRange{Int},
    ptdf_col::AbstractVector{Float64},
    nodal_balance_expressions::JuMPAffineExpressionDArrayIntInt,
    reduction_entry::Vector{Any},
    branches::Vector{String},
)
    names = _get_branch_names(reduction_entry)
    for name in names
        if name in branches
            branch_flow_expr[name, :] .= _make_flow_expressions!(
                name,
                time_steps,
                ptdf_col,
                nodal_balance_expressions.data,
            )
            #Only one constraint added per arc; once it is found can return
            return
        end
    end
end

function _add_expression_to_container!(
    branch_flow_expr::JuMPAffineExpressionDArrayStringInt,
    jump_model::JuMP.Model,
    time_steps::UnitRange{Int},
    ptdf_col::AbstractVector{Float64},
    nodal_balance_expressions::JuMPAffineExpressionDArrayIntInt,
    reduction_entry::Set{PSY.ACTransmission},
    branches::Vector{String},
)
    names = _get_branch_names(reduction_entry)
    for name in names
        if name in branches
            branch_flow_expr[name, :] .= _make_flow_expressions!(
                name,
                time_steps,
                ptdf_col,
                nodal_balance_expressions.data,
            )
            #Only one constraint added per arc; once it is found can return
            return
        end
    end
end

function add_expressions!(
    container::OptimizationContainer,
    ::Type{PTDFBranchFlow},
    devices::IS.FlattenIteratorWrapper{B},
    model::DeviceModel{B, <:AbstractBranchFormulation},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {B <: PSY.ACTransmission}
    time_steps = get_time_steps(container)
    ptdf = get_PTDF_matrix(network_model)
    net_reduction_data = network_model.network_reduction
    # This might need to be changed to something else
    branch_names = get_branch_argument_variable_axis(net_reduction_data, devices)
    # Needs to be a vector to use multi-threading
    name_to_arc_map = collect(PNM.get_name_to_arc_map(net_reduction_data, B))
    nodal_balance_expressions = get_expression(container, ActivePowerBalance(), PSY.ACBus)

    branch_flow_expr =
        add_expression_container!(container, PTDFBranchFlow(), B, branch_names, time_steps)

    jump_model = get_jump_model(container)

    # `ptdf[arc, :]` is a KLU forward+backward solve. libklu cannot run safely
    # under concurrent calls (even with distinct Numeric/Symbolic/Common per
    # thread; see `_LIBKLU_LOCK` in PowerNetworkMatrices), so the solves run
    # serially on the dispatcher and only the JuMP `AffExpr` build is
    # parallelized via `Threads.@spawn`. The try/catch surfaces the inner
    # exception — `build!`'s default error handler shows only the wrapping
    # `TaskFailedException`, which makes spawn-task failures undebuggable.
    tasks = map(name_to_arc_map) do pair
        (name, (arc, _)) = pair
        ptdf_col = ptdf[arc, :]
        Threads.@spawn try
            _make_flow_expressions!(name, time_steps, ptdf_col, nodal_balance_expressions.data)
        catch e
            @error "PTDF flow-expression task failed" name = name arc = arc exception =
                (e, catch_backtrace())
            rethrow()
        end
    end
    for task in tasks
        name, expressions = fetch(task)
        # ptdf[arc,:] uses the representative's orientation; flip series members
        # back to native from→to. sign == 1.0 fast path skips a broadcast-multiply.
        orientation_sign = get_ptdf_orientation_sign(net_reduction_data, B, name)
        if orientation_sign == 1.0
            branch_flow_expr[name, :] .= expressions
        else
            branch_flow_expr[name, :] .= orientation_sign .* expressions
        end
    end
    #= Leaving serial code commented out for debugging purposes in the future
    for (name, (arc, reduction)) in name_to_arc_map
        reduction_entry = all_branch_maps_by_type[reduction][B][arc]
        network_reduction_map = all_branch_maps_by_type[map]
        !haskey(network_reduction_map, branch_Type) && continue
        for (arc_tuple, reduction_entry) in network_reduction_map[branch_Type]
            ptdf_col = ptdf[arc_tuple, :]
            _add_expression_to_container!(
                branch_flow_expr,
                jump_model,
                time_steps,
                ptdf_col,
                nodal_balance_expressions,
                reduction_entry,
                name,
            )
        end
    end
    =#
    return
end

"""
Add network flow constraints for ACBranch and NetworkModel with <: AbstractPTDFModel
"""
function add_constraints!(
    container::OptimizationContainer,
    cons_type::Type{NetworkFlowConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, StaticBranchBounds},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.ACTransmission}
    time_steps = get_time_steps(container)
    branch_flow_expr = get_expression(container, PTDFBranchFlow(), T)
    flow_variables = get_variable(container, FlowActivePowerVariable(), T)
    net_reduction_data = network_model.network_reduction
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    branches = get_branch_argument_constraint_axis(
        net_reduction_data,
        reduced_branch_tracker,
        devices,
        cons_type,
    )
    branch_flow = add_constraints_container!(
        container,
        NetworkFlowConstraint(),
        T,
        branches,
        time_steps,
    )
    jump_model = get_jump_model(container)

    use_slacks = get_use_slacks(device_model)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), T)
        slack_lb = get_variable(container, FlowActivePowerSlackLowerBound(), T)
    end

    for name in branches
        for t in time_steps
            branch_flow[name, t] = JuMP.@constraint(
                jump_model,
                branch_flow_expr[name, t] - flow_variables[name, t] ==
                (use_slacks ? slack_ub[name, t] - slack_lb[name, t] : 0.0)
            )
        end
    end
    return
end

function add_constraints!(
    ::OptimizationContainer,
    cons_type::Type{NetworkFlowConstraint},
    ::IS.FlattenIteratorWrapper{B},
    ::DeviceModel{B, T},
    ::NetworkModel{<:AbstractPTDFModel},
) where {B <: PSY.ACTransmission, T <: Union{StaticBranchUnbounded, StaticBranch}}
    @debug "PTDF Branch Flows with $T do not require network flow constraints $cons_type. Flow values are given by PTDFBranchFlow."
    return
end

"""
Add network flow constraints for PhaseShiftingTransformer and NetworkModel with <: AbstractPTDFModel
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{NetworkFlowConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, PhaseAngleControl},
    network_model::NetworkModel{<:AbstractPTDFModel},
) where {T <: PSY.PhaseShiftingTransformer}
    ptdf = get_PTDF_matrix(network_model)
    branches = PSY.get_name.(devices)
    time_steps = get_time_steps(container)
    branch_flow = add_constraints_container!(
        container,
        NetworkFlowConstraint(),
        T,
        branches,
        time_steps,
    )
    nodal_balance_expressions = get_expression(container, ActivePowerBalance(), PSY.ACBus)
    flow_variables = get_variable(container, FlowActivePowerVariable(), T)
    angle_variables = get_variable(container, PhaseShifterAngle(), T)
    jump_model = get_jump_model(container)
    for br in devices
        arc = PNM.get_arc_tuple(br)
        name = PSY.get_name(br)
        ptdf_col = ptdf[arc, :]
        inv_x = 1 / PSY.get_x(br)
        for t in time_steps
            branch_flow[name, t] = JuMP.@constraint(
                jump_model,
                sum(
                    ptdf_col[i] * nodal_balance_expressions.data[i, t] for
                    i in 1:length(ptdf_col)
                ) + inv_x * angle_variables[name, t] - flow_variables[name, t] == 0.0
            )
        end
    end
    return
end

# `MonitoredLine.flow_limits` may be asymmetric; the symmetric/min-based
# `get_min_max_limits` methods below collapse it to one value and warn once.
function _warn_unequal_monitored_flow_limits(device::PSY.MonitoredLine)
    flow_limits = PSY.get_flow_limits(device)
    if flow_limits.to_from != flow_limits.from_to
        @warn "Flow limits in Line $(PSY.get_name(device)) aren't equal; the \
               minimum will be used."
    end
    return
end

"""
Min and max limits for monitored line
"""
function get_min_max_limits(
    device::PSY.MonitoredLine,
    ::Type{<:ConstraintType},
    ::Type{<:AbstractBranchFormulation},
)
    _warn_unequal_monitored_flow_limits(device)
    limit = min(
        PSY.get_rating(device),
        PSY.get_flow_limits(device).to_from,
        PSY.get_flow_limits(device).from_to,
    )
    minmax = (min = -1 * limit, max = limit)
    return minmax
end

############################## Flow Limits Constraints #####################################
"""
Add branch flow constraints for monitored lines with DC Power Model
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{FlowLimitConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, U},
    ::NetworkModel{V},
) where {
    T <: Union{PSY.PhaseShiftingTransformer, PSY.MonitoredLine},
    U <: AbstractBranchFormulation,
    V <: PM.AbstractDCPModel,
}
    add_range_constraints!(
        container,
        FlowLimitConstraint,
        FlowActivePowerVariable,
        devices,
        model,
        V,
    )
    return
end

"""
Don't add branch flow constraints for monitored lines if formulation is StaticBranchUnbounded
"""
function add_constraints!(
    ::OptimizationContainer,
    ::Type{FlowRateConstraintFromTo},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, U},
    ::NetworkModel{V},
) where {
    T <: PSY.MonitoredLine,
    U <: StaticBranchUnbounded,
    V <: PM.AbstractActivePowerModel,
}
    return
end

"""
Min and max limits for flow limit from-to constraint
"""
function get_min_max_limits(
    device::PSY.MonitoredLine,
    ::Type{FlowLimitFromToConstraint},
    ::Type{<:AbstractBranchFormulation},
)
    _warn_unequal_monitored_flow_limits(device)
    return (
        min = -1 * PSY.get_flow_limits(device).from_to,
        max = PSY.get_flow_limits(device).from_to,
    )
end

"""
Min and max limits for flow limit to-from constraint
"""
function get_min_max_limits(
    device::PSY.MonitoredLine,
    ::Type{FlowLimitToFromConstraint},
    ::Type{<:AbstractBranchFormulation},
)
    _warn_unequal_monitored_flow_limits(device)
    return (
        min = -1 * PSY.get_flow_limits(device).to_from,
        max = PSY.get_flow_limits(device).to_from,
    )
end

"""
Don't add branch flow constraints for monitored lines if formulation is StaticBranchUnbounded
"""
function add_constraints!(
    ::OptimizationContainer,
    ::Type{FlowLimitToFromConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, U},
    ::NetworkModel{V},
) where {
    T <: PSY.MonitoredLine,
    U <: StaticBranchUnbounded,
    V <: PM.AbstractActivePowerModel,
}
    return
end

"""
Add phase angle limits for phase shifters
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{PhaseAngleControlLimit},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, PhaseAngleControl},
    ::NetworkModel{U},
) where {T <: PSY.PhaseShiftingTransformer, U <: PM.AbstractActivePowerModel}
    add_range_constraints!(
        container,
        PhaseAngleControlLimit,
        PhaseShifterAngle,
        devices,
        model,
        U,
    )
    return
end

"""
Add network flow constraints for PhaseShiftingTransformer and NetworkModel with PM.DCPPowerModel
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{NetworkFlowConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    model::DeviceModel{T, PhaseAngleControl},
    ::NetworkModel{PM.DCPPowerModel},
) where {T <: PSY.PhaseShiftingTransformer}
    time_steps = get_time_steps(container)
    flow_variables = get_variable(container, FlowActivePowerVariable(), T)
    ps_angle_variables = get_variable(container, PhaseShifterAngle(), T)
    bus_angle_variables = get_variable(container, VoltageAngle(), PSY.ACBus)
    jump_model = get_jump_model(container)
    branch_flow = add_constraints_container!(
        container,
        NetworkFlowConstraint(),
        T,
        axes(flow_variables)[1],
        time_steps,
    )

    for br in devices
        name = PSY.get_name(br)
        inv_x = 1.0 / PSY.get_x(br)
        flow_variables_ = flow_variables[name, :]
        from_bus = PSY.get_name(PSY.get_from(PSY.get_arc(br)))
        to_bus = PSY.get_name(PSY.get_to(PSY.get_arc(br)))
        angle_variables_ = ps_angle_variables[name, :]
        bus_angle_from = bus_angle_variables[from_bus, :]
        bus_angle_to = bus_angle_variables[to_bus, :]
        @assert inv_x > 0.0
        for t in time_steps
            branch_flow[name, t] = JuMP.@constraint(
                jump_model,
                flow_variables_[t] ==
                inv_x * (bus_angle_from[t] - bus_angle_to[t] + angle_variables_[t])
            )
        end
    end
    return
end

function objective_function!(
    container::OptimizationContainer,
    ::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, <:AbstractBranchFormulation},
    ::Type{<:PM.AbstractPowerModel},
) where {T <: PSY.ACTransmission}
    if get_use_slacks(device_model)
        variable_up = get_variable(container, FlowActivePowerSlackUpperBound(), T)
        # Use device names because there might be a network reduction
        for name in axes(variable_up, 1)
            for t in get_time_steps(container)
                add_to_objective_invariant_expression!(
                    container,
                    variable_up[name, t] * CONSTRAINT_VIOLATION_SLACK_COST,
                )
            end
        end
    end
    return
end

function objective_function!(
    container::OptimizationContainer,
    ::IS.FlattenIteratorWrapper{T},
    device_model::DeviceModel{T, <:AbstractBranchFormulation},
    ::Type{<:PM.AbstractActivePowerModel},
) where {T <: PSY.ACTransmission}
    if get_use_slacks(device_model)
        variable_up = get_variable(container, FlowActivePowerSlackUpperBound(), T)
        variable_dn = get_variable(container, FlowActivePowerSlackLowerBound(), T)
        # Use device names because there might be a network reduction
        for name in axes(variable_up, 1)
            for t in get_time_steps(container)
                add_to_objective_invariant_expression!(
                    container,
                    (variable_dn[name, t] + variable_up[name, t]) *
                    CONSTRAINT_VIOLATION_SLACK_COST,
                )
            end
        end
    end
    return
end
