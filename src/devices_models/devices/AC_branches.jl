
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

get_multiplier_value(::AbstractDynamicBranchRatingTimeSeriesParameter, d::PSY.ACTransmission, ::StaticBranch) = PSY.get_rating(d)
get_multiplier_value(::AbstractDynamicBranchRatingTimeSeriesParameter, d::PNM.BranchesParallel, ::StaticBranch) = PNM.get_equivalent_rating(d)


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
    return Dict{Type{<:TimeSeriesParameter}, String}(
        DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
    )
end

function get_default_attributes(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.ACTransmission, V <: AbstractBranchFormulation}
    return Dict{String, Any}()
end
#################################### Flow Variable Bounds ##################################################

function add_variables!(
    container::OptimizationContainer,
    ::Type{T},
    network_model::NetworkModel{<:AbstractPTDFModel},
    devices::IS.FlattenIteratorWrapper{U},
    formulation::AbstractBranchFormulation,
) where {
    T <: AbstractACActivePowerFlow,
    U <: PSY.ACTransmission}
    time_steps = get_time_steps(container)
    net_reduction_data = network_model.network_reduction
    branch_names = get_branch_argument_variable_axis(net_reduction_data, devices)
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    variable_container = add_variable_container!(
        container,
        T(),
        U,
        branch_names,
        time_steps,
    )

    for (name, (arc, reduction)) in PNM.get_name_to_arc_map(net_reduction_data, U)
        # TODO: entry is not type stable here, it can return any type ACTransmission.
        # It might have performance implications. Possibly separate this into other functions
        reduction_entry = all_branch_maps_by_type[reduction][U][arc]
        has_entry, tracker_container = search_for_reduced_branch_argument!(
            reduced_branch_tracker,
            arc,
            T,
        )
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
                ub !== nothing && JuMP.set_upper_bound(tracker_container[t], ub)
                lb !== nothing && JuMP.set_lower_bound(tracker_container[t], lb)
            end
            variable_container[name, t] = tracker_container[t]
        end
    end
    return
end

function add_variables!(
    container::OptimizationContainer,
    ::Type{T},
    network_model::NetworkModel{<:PM.AbstractPowerModel},
    devices::IS.FlattenIteratorWrapper{U},
    formulation::AbstractBranchFormulation,
) where {
    T <: AbstractACActivePowerFlow,
    U <: PSY.ACTransmission}
    net_reduction_data = network_model.network_reduction
    time_steps = get_time_steps(container)
    branch_names = get_branch_argument_variable_axis(net_reduction_data, devices)
    reduced_branch_tracker = get_reduced_branch_tracker(network_model)
    all_branch_maps_by_type = PNM.get_all_branch_maps_by_type(net_reduction_data)

    variable_container = add_variable_container!(
        container,
        T(),
        U,
        branch_names,
        time_steps,
    )

    for (name, (arc, reduction)) in PNM.get_name_to_arc_map(net_reduction_data, U)
        reduction_entry = all_branch_maps_by_type[reduction][U][arc]
        has_entry, tracker_container = search_for_reduced_branch_argument!(
            reduced_branch_tracker,
            arc,
            T,
        )
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
                ub !== nothing && JuMP.set_upper_bound(tracker_container[t], ub)
                lb !== nothing && JuMP.set_lower_bound(tracker_container[t], lb)
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
) where {
    T <: AbstractACActivePowerFlow,
    U <: PSY.ACTransmission}
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
    ::DeviceModel{B, T},
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
            limits = get_min_max_limits(reduction_entry, FlowRateConstraint, T)
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

function get_rating(double_circuit::PNM.BranchesParallel)
    return sum([PSY.get_rating(circuit) for circuit in double_circuit])
end
function get_rating(series_chain::PNM.BranchesSeries)
    return minimum([get_rating(segment) for segment in series_chain])
end
function get_rating(device::T) where {T <: PSY.ACTransmission}
    return PSY.get_rating(device)
end
function get_rating(
    device::PNM.ThreeWindingTransformerWinding{T},
) where {T <: PSY.ThreeWindingTransformer}
    return PNM.get_equivalent_rating(device)
end

"""
Min and max limits for Abstract Branch Formulation
"""
function get_min_max_limits(
    double_circuit::PNM.BranchesParallel{<:PSY.ACTransmission},
    constraint_type::Type{<:ConstraintType},
    branch_formulation::Type{<:AbstractBranchFormulation},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    min_max_by_circuit = [
        get_min_max_limits(device, constraint_type, branch_formulation) for
        device in double_circuit
    ]
    min_by_circuit = [x.min for x in min_max_by_circuit]
    max_by_circuit = [x.max for x in min_max_by_circuit]
    # Limit by most restictive circuit:
    return (min = maximum(min_by_circuit), max = minimum(max_by_circuit))
end

"""
Min and max limits for Abstract Branch Formulation
"""
function get_min_max_limits(
    transformer_entry::PNM.ThreeWindingTransformerWinding,
    constraint_type::Type{<:ConstraintType},
    branch_formulation::Type{<:AbstractBranchFormulation},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    transformer = PNM.get_transformer(transformer_entry)
    winding_number = PNM.get_winding_number(transformer_entry)
    if winding_number == 1
        limits = (
            min = -1 * PSY.get_rating_primary(transformer),
            max = PSY.get_rating_primary(transformer),
        )
    elseif winding_number == 2
        limits = (
            min = -1 * PSY.get_rating_secondary(transformer),
            max = PSY.get_rating_secondary(transformer),
        )
    elseif winding_number == 3
        limits = (
            min = -1 * PSY.get_rating_tertiary(transformer),
            max = PSY.get_rating_tertiary(transformer),
        )
    end
    return limits
end

"""
Min and max limits for Abstract Branch Formulation
"""
function get_min_max_limits(
    series_chain::PNM.BranchesSeries,
    constraint_type::Type{<:ConstraintType},
    branch_formulation::Type{<:AbstractBranchFormulation},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    min_max_by_segment = [
        get_min_max_limits(segment, constraint_type, branch_formulation) for
        segment in series_chain
    ]
    min_by_segment = [x.min for x in min_max_by_segment]
    max_by_segment = [x.max for x in min_max_by_segment]
    # Limit by most restictive segment:
    return (min = maximum(min_by_segment), max = minimum(max_by_segment))
end

"""
Min and max limits for Abstract Branch Formulation
"""
function get_min_max_limits(
    device::PSY.ACTransmission,
    ::Type{<:ConstraintType},
    ::Type{<:AbstractBranchFormulation},
) #  -> Union{Nothing, NamedTuple{(:min, :max), Tuple{Float64, Float64}}}
    return (
        min = -1 * PNM.get_equivalent_rating(device),
        max = PNM.get_equivalent_rating(device),
    )
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
    ::Type{T},
    arc::Tuple{Int, Int},
    use_slacks::Bool,
    con_lb::DenseAxisArray,
    con_ub::DenseAxisArray,
    var::DenseAxisArray,
    branch_maps_by_type::Dict,
    name::String,
) where {T <: PSY.ACTransmission}
    reduction_entry = branch_maps_by_type[arc]
    time_steps = get_time_steps(container)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), T)[name, :]
        slack_lb = get_variable(container, FlowActivePowerSlackLowerBound(), T)[name, :]
    end
    limits = get_min_max_limits(reduction_entry, FlowRateConstraint, StaticBranch)
    for t in time_steps
        con_ub[name, t] =
            JuMP.@constraint(
                get_jump_model(container),
                var[name, t] - (use_slacks ? slack_ub[t] : 0.0) <= limits.max
            )
        con_lb[name, t] =
            JuMP.@constraint(
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

    con_lb =
        add_constraints_container!(
            container,
            cons_type(),
            T,
            branch_names,
            time_steps;
            meta = "lb",
        )
    con_ub =
        add_constraints_container!(
            container,
            cons_type(),
            T,
            branch_names,
            time_steps;
            meta = "ub",
        )

    array = get_variable(container, FlowActivePowerVariable(), T)

    use_slacks = get_use_slacks(device_model)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), T)
        slack_lb = get_variable(container, FlowActivePowerSlackLowerBound(), T)
    end
    for (name, (arc, reduction)) in
        get_constraint_map_by_type(reduced_branch_tracker)[FlowRateConstraint][T]
        _add_flow_rate_constraint!(
            container,
            T,
            arc,
            use_slacks,
            con_lb,
            con_ub,
            array,
            all_branch_maps_by_type[reduction][T],
            name,
        )
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
    param_container =
        get_parameter(container, DynamicBranchRatingTimeSeriesParameter(), T)
    param = get_parameter_column_refs(param_container, name)
    mult = get_multiplier_array(param_container)[name, :]

    for t in time_steps
        @debug "Dynamic Branch Rating applied for branch $(name) at time step $(t)"
        con_ub[name, t] =
            JuMP.@constraint(
                get_jump_model(container),
                var[name, t] - (use_slacks ? slack_ub[t] : 0.0) <= param[t] * mult[t]
            )
        con_lb[name, t] =
            JuMP.@constraint(
                get_jump_model(container),
                var[name, t] + (use_slacks ? slack_lb[t] : 0.0) >=
                -1.0 * param[t] * mult[t]
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
) where {
    T <: PSY.ACTransmission,
    U <: AbstractBranchFormulation,
    V <: AbstractPTDFModel,
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

    con_lb =
        add_constraints_container!(
            container,
            cons_type(),
            T,
            branch_names,
            time_steps;
            meta = "lb",
        )
    con_ub =
        add_constraints_container!(
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
        limits = get_min_max_limits(reduction_entry, FlowRateConstraint, U)
        for t in time_steps
            con_ub[name, t] =
                JuMP.@constraint(get_jump_model(container),
                    array[name, t] -
                    (use_slacks ? slack_ub[name, t] : 0.0) <=
                    limits.max)
            con_lb[name, t] =
                JuMP.@constraint(get_jump_model(container),
                    array[name, t] +
                    (use_slacks ? slack_lb[name, t] : 0.0) >=
                    limits.min)
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
) where {
    T <: PSY.ACTransmission,
    U <: StaticBranch,
    V <: AbstractPTDFModel,
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

    con_lb =
        add_constraints_container!(
            container,
            cons_type(),
            T,
            branch_names,
            time_steps;
            meta = "lb",
        )
    con_ub =
        add_constraints_container!(
            container,
            cons_type(),
            T,
            branch_names,
            time_steps;
            meta = "ub",
        )

    var_array = get_expression(container, PTDFBranchFlow(), T)

    ts_name = get_time_series_names(device_model)[DynamicBranchRatingTimeSeriesParameter]
    ts_type = get_default_time_series_type(container)
    use_slacks = get_use_slacks(device_model)
    for (name, (arc, reduction)) in
        get_constraint_map_by_type(reduced_branch_tracker)[FlowRateConstraint][T]
        if PNM.has_time_series(
            all_branch_maps_by_type[reduction][T][arc],
            ts_type,
            ts_name,
        )
            _add_flow_rate_constraint_with_parameters!(
                container,
                T,
                arc,
                use_slacks,
                con_lb,
                con_ub,
                var_array,
                all_branch_maps_by_type[reduction][T],
                name,
                ts_name,
            )
        else
            _add_flow_rate_constraint!(
                container,
                T,
                arc,
                use_slacks,
                con_lb,
                con_ub,
                var_array,
                all_branch_maps_by_type[reduction][T],
                name,
            )
        end
    end
    return
end

"""
Add rate limit from to constraints for ACBranch with AbstractPowerModel
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
    add_constraints_container!(
        container,
        cons_type(),
        B,
        device_names,
        time_steps,
    )
    constraint = get_constraint(container, cons_type(), B)

    use_slacks = get_use_slacks(device_model)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), B)
    end
    for (name, (arc, reduction)) in
        get_constraint_map_by_type(reduced_branch_tracker)[FlowRateConstraintFromTo][B]
        # TODO: entry is not type stable here, it can return any type ACTransmission.
        # It might have performance implications. Possibly separate this into other functions
        reduction_entry = all_branch_maps_by_type[reduction][B][arc]
        branch_rate = get_rating(reduction_entry)
        for t in time_steps
            constraint[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var1[name, t]^2 + var2[name, t]^2 -
                (use_slacks ? slack_ub[name, t] : 0.0) <= branch_rate^2
            )
        end
    end
    return
end

"""
Add rate limit to from constraints for ACBranch with AbstractPowerModel
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
    add_constraints_container!(
        container,
        cons_type(),
        B,
        device_names,
        time_steps,
    )
    constraint = get_constraint(container, cons_type(), B)
    use_slacks = get_use_slacks(device_model)
    if use_slacks
        slack_ub = get_variable(container, FlowActivePowerSlackUpperBound(), B)
    end
    for (name, (arc, reduction)) in
        get_constraint_map_by_type(reduced_branch_tracker)[FlowRateConstraintToFrom][B]
        # TODO: entry is not type stable here, it can return any type ACTransmission.
        # It might have performance implications. Possibly separate this into other functions
        reduction_entry = all_branch_maps_by_type[reduction][B][arc]
        branch_rate = get_rating(reduction_entry)
        for t in time_steps
            constraint[name, t] = JuMP.@constraint(
                get_jump_model(container),
                var1[name, t]^2 + var2[name, t]^2 -
                (use_slacks ? slack_ub[name, t] : 0.0) <= branch_rate^2
            )
        end
    end
    return
end

function _make_flow_expressions!(
    jump_model::JuMP.Model,
    name::String,
    time_steps::UnitRange{Int},
    ptdf_col::AbstractVector{Float64},
    nodal_balance_expressions::Matrix{JuMP.AffExpr},
)
    @debug "Making Flow Expression on thread $(Threads.threadid()) for branch $name"
    expressions = Vector{JuMP.AffExpr}(undef, length(time_steps))
    for t in time_steps
        expressions[t] = JuMP.@expression(
            jump_model,
            sum(
                ptdf_col[i] * nodal_balance_expressions[i, t] for
                i in 1:length(ptdf_col)
            )
        )
    end
    return name, expressions
    # change when using the not concurrent version
    # return expressions
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
            jump_model,
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
                jump_model,
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
                jump_model,
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
    nodal_balance_expressions = get_expression(
        container,
        ActivePowerBalance(),
        PSY.ACBus,
    )

    branch_flow_expr = add_expression_container!(container,
        PTDFBranchFlow(),
        B,
        branch_names,
        time_steps,
    )

    jump_model = get_jump_model(container)

    tasks = map(collect(name_to_arc_map)) do pair
        (name, (arc, _)) = pair
        ptdf_col = ptdf[arc, :]
        Threads.@spawn _make_flow_expressions!(
            jump_model,
            name,
            time_steps,
            ptdf_col,
            nodal_balance_expressions.data,
        )
    end
    for task in tasks
        name, expressions = fetch(task)
        branch_flow_expr[name, :] .= expressions
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
                branch_flow_expr[name, t] -
                flow_variables[name, t]
                ==
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

"""
Min and max limits for monitored line
"""
function get_min_max_limits(
    device::PSY.MonitoredLine,
    ::Type{<:ConstraintType},
    ::Type{<:AbstractBranchFormulation},
)
    if PSY.get_flow_limits(device).to_from != PSY.get_flow_limits(device).from_to
        @warn(
            "Flow limits in Line $(PSY.get_name(device)) aren't equal. The minimum will be used in formulation $(T)"
        )
    end
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
    if PSY.get_flow_limits(device).to_from != PSY.get_flow_limits(device).from_to
        @warn(
            "Flow limits in Line $(PSY.get_name(device)) aren't equal. The minimum will be used in formulation $(T)"
        )
    end
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
    if PSY.get_flow_limits(device).to_from != PSY.get_flow_limits(device).from_to
        @warn(
            "Flow limits in Line $(PSY.get_name(device)) aren't equal. The minimum will be used in formulation $(T)"
        )
    end
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
