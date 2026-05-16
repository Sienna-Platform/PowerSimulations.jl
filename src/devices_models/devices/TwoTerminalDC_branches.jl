#################################### Branch Variables ##################################################
#! format: off
get_variable_binary(::FlowActivePowerSlackUpperBound, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation,) = false
get_variable_binary(::FlowActivePowerSlackLowerBound, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation,) = false
get_variable_binary(::HVDCPiecewiseLossVariable, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation,) = false
get_variable_binary(::HVDCActivePowerReceivedFromVariable, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation,) = false
get_variable_binary(::HVDCActivePowerReceivedToVariable, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation,) = false
get_variable_binary(::HVDCPiecewiseBinaryLossVariable, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation,) = true
get_variable_binary(_, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation) = false
get_variable_binary(::FlowActivePowerVariable, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation) = false
get_variable_binary(::HVDCFlowDirectionVariable, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation) = true
get_variable_multiplier(::FlowActivePowerVariable, ::Type{<:PSY.TwoTerminalHVDC}, _) = NaN
get_parameter_multiplier(::FixValueParameter, ::PSY.TwoTerminalHVDC, ::AbstractTwoTerminalDCLineFormulation) = 1.0
get_variable_multiplier(::FlowActivePowerFromToVariable, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation) = -1.0
get_variable_multiplier(::FlowActivePowerToFromVariable, ::Type{<:PSY.TwoTerminalHVDC}, ::AbstractTwoTerminalDCLineFormulation) = -1.0

function get_variable_multiplier(
    ::HVDCLosses,
    d::PSY.TwoTerminalHVDC,
    ::HVDCTwoTerminalDispatch,
)
    loss = PSY.get_loss(d)
    if !isa(loss, PSY.LinearCurve)
        error(
            "HVDCTwoTerminalDispatch of branch $(PSY.get_name(d)) only accepts LinearCurve for loss models.",
        )
    end
    l1 = PSY.get_proportional_term(loss)
    l0 = PSY.get_constant_term(loss)
    if l1 == 0.0 && l0 == 0.0
        return 0.0
    else
        return -1.0
    end
end

get_variable_lower_bound(::FlowActivePowerVariable, d::PSY.TwoTerminalHVDC, ::HVDCTwoTerminalUnbounded) = nothing
get_variable_upper_bound(::FlowActivePowerVariable, d::PSY.TwoTerminalHVDC, ::HVDCTwoTerminalUnbounded) = nothing
get_variable_lower_bound(::FlowActivePowerVariable, d::PSY.TwoTerminalHVDC, ::AbstractTwoTerminalDCLineFormulation) = nothing
get_variable_upper_bound(::FlowActivePowerVariable, d::PSY.TwoTerminalHVDC, ::AbstractTwoTerminalDCLineFormulation) = nothing
get_variable_lower_bound(::HVDCLosses, d::PSY.TwoTerminalHVDC, ::HVDCTwoTerminalDispatch) = 0.0
get_variable_upper_bound(::FlowActivePowerFromToVariable, d::PSY.TwoTerminalHVDC, ::HVDCTwoTerminalDispatch) = PSY.get_active_power_limits_from(d).max
get_variable_lower_bound(::FlowActivePowerFromToVariable, d::PSY.TwoTerminalHVDC, ::HVDCTwoTerminalDispatch) = PSY.get_active_power_limits_from(d).min
get_variable_upper_bound(::FlowActivePowerToFromVariable, d::PSY.TwoTerminalHVDC, ::HVDCTwoTerminalDispatch) = PSY.get_active_power_limits_to(d).max
get_variable_lower_bound(::FlowActivePowerToFromVariable, d::PSY.TwoTerminalHVDC, ::HVDCTwoTerminalDispatch) = PSY.get_active_power_limits_to(d).min
get_variable_upper_bound(::HVDCActivePowerReceivedFromVariable, d::PSY.TwoTerminalHVDC, ::AbstractTwoTerminalDCLineFormulation) = PSY.get_active_power_limits_from(d).max
get_variable_lower_bound(::HVDCActivePowerReceivedFromVariable, d::PSY.TwoTerminalHVDC, ::AbstractTwoTerminalDCLineFormulation) = PSY.get_active_power_limits_from(d).min
get_variable_upper_bound(::HVDCActivePowerReceivedToVariable, d::PSY.TwoTerminalHVDC, ::AbstractTwoTerminalDCLineFormulation) = PSY.get_active_power_limits_to(d).max
get_variable_lower_bound(::HVDCActivePowerReceivedToVariable, d::PSY.TwoTerminalHVDC, ::AbstractTwoTerminalDCLineFormulation) = PSY.get_active_power_limits_to(d).min

function get_variable_upper_bound(
    ::HVDCLosses,
    d::PSY.TwoTerminalHVDC,
    ::HVDCTwoTerminalDispatch,
)
    loss = PSY.get_loss(d)
    if !isa(loss, PSY.LinearCurve)
        error(
            "HVDCTwoTerminalDispatch of branch $(PSY.get_name(d)) only accepts LinearCurve for loss models.",
        )
    end
    l1 = PSY.get_proportional_term(loss)
    l0 = PSY.get_constant_term(loss)
    if l1 == 0.0 && l0 == 0.0
        return 0.0
    else
        return
    end
end

get_variable_upper_bound(::HVDCPiecewiseLossVariable, d::PSY.TwoTerminalHVDC, ::Union{HVDCTwoTerminalDispatch, HVDCTwoTerminalPiecewiseLoss}) = 1.0
get_variable_lower_bound(::HVDCPiecewiseLossVariable, d::PSY.TwoTerminalHVDC, ::Union{HVDCTwoTerminalDispatch, HVDCTwoTerminalPiecewiseLoss}) = 0.0

#################################### LCC ##################################################
get_variable_binary(::HVDCActivePowerReceivedFromVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCActivePowerReceivedToVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCReactivePowerReceivedFromVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCReactivePowerReceivedToVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCRectifierDelayAngleVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCInverterExtinctionAngleVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCRectifierPowerFactorAngleVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCInverterPowerFactorAngleVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCRectifierOverlapAngleVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCInverterOverlapAngleVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCRectifierDCVoltageVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCInverterDCVoltageVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCRectifierACCurrentVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCInverterACCurrentVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::DCLineCurrentFlowVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCRectifierTapSettingVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_binary(::HVDCInverterTapSettingVariable, ::Type{PSY.TwoTerminalLCCLine}, ::HVDCTwoTerminalLCC) = false
get_variable_upper_bound(::HVDCRectifierDelayAngleVariable, d::PSY.TwoTerminalLCCLine, ::HVDCTwoTerminalLCC) = PSY.get_rectifier_delay_angle_limits(d).max
get_variable_lower_bound(::HVDCRectifierDelayAngleVariable, d::PSY.TwoTerminalLCCLine, ::HVDCTwoTerminalLCC) = PSY.get_rectifier_delay_angle_limits(d).min
get_variable_upper_bound(::HVDCInverterExtinctionAngleVariable, d::PSY.TwoTerminalLCCLine, ::HVDCTwoTerminalLCC) = PSY.get_inverter_extinction_angle_limits(d).max
get_variable_lower_bound(::HVDCInverterExtinctionAngleVariable, d::PSY.TwoTerminalLCCLine, ::HVDCTwoTerminalLCC) = PSY.get_inverter_extinction_angle_limits(d).min
get_variable_upper_bound(::HVDCRectifierTapSettingVariable, d::PSY.TwoTerminalLCCLine, ::HVDCTwoTerminalLCC) = PSY.get_rectifier_tap_limits(d).max
get_variable_lower_bound(::HVDCRectifierTapSettingVariable, d::PSY.TwoTerminalLCCLine, ::HVDCTwoTerminalLCC) = PSY.get_rectifier_tap_limits(d).min
get_variable_upper_bound(::HVDCInverterTapSettingVariable, d::PSY.TwoTerminalLCCLine, ::HVDCTwoTerminalLCC) = PSY.get_inverter_tap_limits(d).max
get_variable_lower_bound(::HVDCInverterTapSettingVariable, d::PSY.TwoTerminalLCCLine, ::HVDCTwoTerminalLCC) = PSY.get_inverter_tap_limits(d).min
#! format: on
##########################################################
function get_default_time_series_names(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.TwoTerminalHVDC, V <: AbstractTwoTerminalDCLineFormulation}
    return Dict{Type{<:TimeSeriesParameter}, String}()
end

function get_default_attributes(
    ::Type{U},
    ::Type{V},
) where {U <: PSY.TwoTerminalHVDC, V <: AbstractTwoTerminalDCLineFormulation}
    return Dict{String, Any}()
end

get_initial_conditions_device_model(
    ::OperationModel,
    ::DeviceModel{T, U},
) where {T <: PSY.TwoTerminalHVDC, U <: AbstractTwoTerminalDCLineFormulation} =
    DeviceModel(T, U)

####################################### PWL Constraints #######################################################

function _get_range_segments(::PSY.TwoTerminalHVDC, loss::PSY.LinearCurve)
    return 1:4
end

function _get_range_segments(
    ::PSY.TwoTerminalHVDC,
    loss::PSY.PiecewiseIncrementalCurve,
)
    loss_factors = PSY.get_slopes(loss)
    return 1:(2 * length(loss_factors) + 2)
end

function _add_dense_pwl_loss_variables!(
    container::OptimizationContainer,
    devices,
    model::DeviceModel{D, HVDCTwoTerminalPiecewiseLoss},
) where {D <: PSY.TwoTerminalHVDC}
    # Check if type and length of PWL loss model are the same for all devices
    _check_pwl_loss_model(devices)

    # Create Variables
    time_steps = get_time_steps(container)
    settings = get_settings(container)
    formulation = HVDCTwoTerminalPiecewiseLoss()
    T = HVDCPiecewiseLossVariable
    binary = get_variable_binary(T(), D, formulation)
    first_loss = PSY.get_loss(first(devices))
    if isa(first_loss, PSY.LinearCurve)
        len_segments = 4 # 2*1 + 2
    elseif isa(first_loss, PSY.PiecewiseIncrementalCurve)
        len_segments = 2 * length(PSY.get_slopes(first_loss)) + 2
    else
        error("Should not be here")
    end

    segments = ["pwl_$i" for i in 1:len_segments]
    T = HVDCPiecewiseLossVariable
    variable = add_variable_container!(
        container,
        T(),
        D,
        PSY.get_name.(devices),
        segments,
        time_steps,
    )

    for t in time_steps, s in segments, d in devices
        name = PSY.get_name(d)
        variable[name, s, t] = JuMP.@variable(
            get_jump_model(container),
            base_name = "$(T)_$(D)_{$(name), $(s), $(t)}",
            binary = binary
        )
        ub = get_variable_upper_bound(T(), d, formulation)
        !isnothing(ub) && JuMP.set_upper_bound(variable[name, s, t], ub)

        lb = get_variable_lower_bound(T(), d, formulation)
        !isnothing(lb) && JuMP.set_lower_bound(variable[name, s, t], lb)

        if get_warm_start(settings)
            init = get_variable_warm_start_value(T(), d, formulation)
            !isnothing(init) && JuMP.set_start_value(variable[name, s, t], init)
        end
    end
end

# Full Binary
function _add_sparse_pwl_loss_variables!(
    container::OptimizationContainer,
    devices,
    ::DeviceModel{D, HVDCTwoTerminalPiecewiseLoss},
) where {D <: PSY.TwoTerminalHVDC}
    # Check if type and length of PWL loss model are the same for all devices
    #_check_pwl_loss_model(devices)

    # Create Variables
    time_steps = get_time_steps(container)
    settings = get_settings(container)
    formulation = HVDCTwoTerminalPiecewiseLoss()
    T = HVDCPiecewiseLossVariable
    binary_T = get_variable_binary(T(), D, formulation)
    U = HVDCPiecewiseBinaryLossVariable
    binary_U = get_variable_binary(U(), D, formulation)
    first_loss = PSY.get_loss(first(devices))
    if isa(first_loss, PSY.LinearCurve)
        len_segments = 3 # 2*1 + 1
    elseif isa(first_loss, PSY.PiecewiseIncrementalCurve)
        len_segments = 2 * length(PSY.get_slopes(first_loss)) + 1
    else
        error("Should not be here")
    end

    var_container = lazy_container_addition!(container, T(), D)
    var_container_binary = lazy_container_addition!(container, U(), D)

    for d in devices
        name = PSY.get_name(d)
        for t in time_steps
            pwlvars = Array{JuMP.VariableRef}(undef, len_segments)
            pwlvars_bin = Array{JuMP.VariableRef}(undef, len_segments)
            for i in 1:len_segments
                pwlvars[i] =
                    var_container[(name, i, t)] = JuMP.@variable(
                        get_jump_model(container),
                        base_name = "$(T)_$(name)_{pwl_$(i), $(t)}",
                        binary = binary_T
                    )
                ub = get_variable_upper_bound(T(), d, formulation)
                !isnothing(ub) && JuMP.set_upper_bound(var_container[name, i, t], ub)

                lb = get_variable_lower_bound(T(), d, formulation)
                !isnothing(lb) && JuMP.set_lower_bound(var_container[name, i, t], lb)

                pwlvars_bin[i] =
                    var_container_binary[(name, i, t)] = JuMP.@variable(
                        get_jump_model(container),
                        base_name = "$(U)_$(name)_{pwl_$(i), $(t)}",
                        binary = binary_U
                    )
            end
        end
    end
end

function _get_pwl_loss_params(d::PSY.TwoTerminalHVDC, loss::PSY.LinearCurve)
    from_to_loss_params = Vector{Float64}(undef, 4)
    to_from_loss_params = Vector{Float64}(undef, 4)
    loss_factor = PSY.get_proportional_term(loss)
    P_send0 = PSY.get_constant_term(loss)
    P_max_ft = PSY.get_active_power_limits_from(d).max
    P_max_tf = PSY.get_active_power_limits_to(d).max
    if P_max_ft != P_max_tf
        error(
            "HVDC Line $(PSY.get_name(d)) has non-symmetrical limits for from and to, that are not supported in the HVDCTwoTerminalPiecewiseLoss formulation",
        )
    end
    P_sendS = P_max_ft
    ### Update Params Vectors ###
    from_to_loss_params[1] = -P_sendS - P_send0
    from_to_loss_params[2] = -P_send0
    from_to_loss_params[3] = 0.0
    from_to_loss_params[4] = P_sendS * (1 - loss_factor)

    to_from_loss_params[1] = P_sendS * (1 - loss_factor)
    to_from_loss_params[2] = 0.0
    to_from_loss_params[3] = -P_send0
    to_from_loss_params[4] = -P_sendS - P_send0

    return from_to_loss_params, to_from_loss_params
end

function _get_pwl_loss_params(
    d::PSY.TwoTerminalHVDC,
    loss::PSY.PiecewiseIncrementalCurve,
)
    p_breakpoints = PSY.get_x_coords(loss)
    loss_factors = PSY.get_slopes(loss)
    len_segments = length(loss_factors)
    len_variables = 2 * len_segments + 2
    from_to_loss_params = Vector{Float64}(undef, len_variables)
    to_from_loss_params = similar(from_to_loss_params)
    P_max_ft = PSY.get_active_power_limits_from(d).max
    P_max_tf = PSY.get_active_power_limits_to(d).max
    if P_max_ft != P_max_tf
        error(
            "HVDC Line $(PSY.get_name(d)) has non-symmetrical limits for from and to, that are not supported in the HVDCTwoTerminalPiecewiseLoss formulation",
        )
    end
    if P_max_ft != last(p_breakpoints)
        error(
            "Maximum power limit $P_max_ft of HVDC Line $(PSY.get_name(d)) has different value of last breakpoint from Loss data $(last(p_breakpoints)).",
        )
    end
    ### Update Params Vectors ###
    ## Update from 1 to S
    for i in 1:len_segments
        from_to_loss_params[i] = -p_breakpoints[2 + len_segments - i] - p_breakpoints[1] # for i = 1: P_end, for i = len_segments: P_2
        to_from_loss_params[i] =
            p_breakpoints[2 + len_segments - i] * (1 - loss_factors[len_segments + 1 - i])
    end
    ## Update from S+1 and S+2
    from_to_loss_params[len_segments + 1] = -p_breakpoints[1] # P_send0
    from_to_loss_params[len_segments + 2] = 0.0
    to_from_loss_params[len_segments + 1] = 0.0
    to_from_loss_params[len_segments + 2] = -p_breakpoints[1] # P_send0
    ## Update from S+3 to 2S+2
    for i in 1:len_segments
        from_to_loss_params[2 + len_segments + i] =
            p_breakpoints[i + 1] * (1 - loss_factors[i])
        to_from_loss_params[2 + len_segments + i] = -p_breakpoints[i + 1] - p_breakpoints[1]
    end

    return from_to_loss_params, to_from_loss_params
end

function add_variables!(
    container::OptimizationContainer,
    ::Type{FlowActivePowerVariable},
    network_model::NetworkModel{CopperPlatePowerModel},
    devices::IS.FlattenIteratorWrapper{T},
    formulation::U,
) where {T <: PSY.TwoTerminalHVDC, U <: AbstractBranchFormulation}
    inter_network_branches = T[]
    for d in devices
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        if ref_bus_from != ref_bus_to
            push!(inter_network_branches, d)
        else
            @warn(
                "HVDC Line $(PSY.get_name(d)) is in the same subnetwork, so the line will not be modeled."
            )
        end
    end
    if !isempty(inter_network_branches)
        add_variables!(container, FlowActivePowerVariable, inter_network_branches, U())
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    ::DeviceModel{U, HVDCTwoTerminalPiecewiseLoss},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: HVDCFlowCalculationConstraint, U <: PSY.TwoTerminalHVDC}
    var_pwl = get_variable(container, HVDCPiecewiseLossVariable(), U)
    var_pwl_bin = get_variable(container, HVDCPiecewiseBinaryLossVariable(), U)
    names = PSY.get_name.(devices)
    time_steps = get_time_steps(container)
    flow_ft = get_variable(container, HVDCActivePowerReceivedFromVariable(), U)
    flow_tf = get_variable(container, HVDCActivePowerReceivedToVariable(), U)

    constraint_from_to =
        add_constraints_container!(container, T(), U, names, time_steps; meta = "ft")
    constraint_to_from =
        add_constraints_container!(container, T(), U, names, time_steps; meta = "tf")
    constraint_binary =
        add_constraints_container!(container, T(), U, names, time_steps; meta = "bin")
    for d in devices
        name = PSY.get_name(d)
        loss = PSY.get_loss(d)
        from_to_params, to_from_params = _get_pwl_loss_params(d, loss)
        range_segments = 1:(length(from_to_params) - 1) # 1:(2S+1)
        for t in time_steps
            ## Add Equality Constraints ##
            constraint_from_to[name, t] = JuMP.@constraint(
                get_jump_model(container),
                flow_ft[name, t] ==
                sum(
                    var_pwl_bin[name, ix, t] * from_to_params[ix] for
                    ix in range_segments
                ) + sum(
                    var_pwl[name, ix, t] * (from_to_params[ix + 1] - from_to_params[ix]) for
                    ix in range_segments
                )
            )
            constraint_to_from[name, t] = JuMP.@constraint(
                get_jump_model(container),
                flow_tf[name, t] ==
                sum(
                    var_pwl_bin[name, ix, t] * to_from_params[ix] for
                    ix in range_segments
                ) + sum(
                    var_pwl[name, ix, t] * (to_from_params[ix + 1] - to_from_params[ix]) for
                    ix in range_segments
                )
            )
            ## Add Binary Bound ###
            constraint_binary[name, t] = JuMP.@constraint(
                get_jump_model(container),
                sum(var_pwl_bin[name, ix, t] for ix in range_segments) == 1.0
            )
            ## Add Bounds for Continuous ##
            for ix in range_segments
                JuMP.@constraint(
                    get_jump_model(container),
                    var_pwl[name, ix, t] <= var_pwl_bin[name, ix, t]
                )
                if ix == div(length(range_segments) + 1, 2)
                    JuMP.fix(var_pwl[name, ix, t], 0.0; force = true)
                end
            end
        end
    end
    return
end

#################################### Rate Limits Constraints ##################################################
function _get_flow_bounds(d::PSY.TwoTerminalHVDC)
    check_hvdc_line_limits_consistency(d)
    from_min = PSY.get_active_power_limits_from(d).min
    to_min = PSY.get_active_power_limits_to(d).min
    from_max = PSY.get_active_power_limits_from(d).max
    to_max = PSY.get_active_power_limits_to(d).max

    if from_min >= 0.0 && to_min >= 0.0
        min_rate = min(from_min, to_min)
    elseif from_min <= 0.0 && to_min <= 0.0
        min_rate = max(from_min, to_min)
    elseif from_min <= 0.0 && to_min >= 0.0
        min_rate = from_min
    elseif to_min <= 0.0 && from_min >= 0.0
        min_rate = to_min
    end

    if from_max >= 0.0 && to_max >= 0.0
        max_rate = min(from_max, to_max)
    elseif from_max <= 0.0 && to_max <= 0.0
        max_rate = max(from_max, to_max)
    elseif from_max <= 0.0 && to_max >= 0.0
        max_rate = from_max
    elseif from_max >= 0.0 && to_max <= 0.0
        max_rate = to_max
    end

    return min_rate, max_rate
end

add_constraints!(
    ::OptimizationContainer,
    ::Type{<:Union{FlowRateConstraintFromTo, FlowRateConstraintToFrom}},
    ::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, HVDCTwoTerminalUnbounded},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.TwoTerminalHVDC} = nothing

add_constraints!(
    ::OptimizationContainer,
    ::Type{FlowRateConstraint},
    ::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, HVDCTwoTerminalUnbounded},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: PSY.TwoTerminalHVDC} = nothing

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    ::DeviceModel{U, HVDCTwoTerminalLossless},
    ::NetworkModel{<:PM.AbstractPowerModel},
) where {T <: FlowRateConstraint, U <: PSY.TwoTerminalHVDC}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)

    var = get_variable(container, FlowActivePowerVariable(), U)
    constraint_ub =
        add_constraints_container!(container, T(), U, names, time_steps; meta = "ub")
    constraint_lb =
        add_constraints_container!(container, T(), U, names, time_steps; meta = "lb")
    for d in devices
        min_rate, max_rate = _get_flow_bounds(d)
        for t in time_steps
            constraint_ub[PSY.get_name(d), t] = JuMP.@constraint(
                get_jump_model(container),
                var[PSY.get_name(d), t] <= max_rate
            )
            constraint_lb[PSY.get_name(d), t] = JuMP.@constraint(
                get_jump_model(container),
                min_rate <= var[PSY.get_name(d), t]
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::Union{Vector{U}, IS.FlattenIteratorWrapper{U}},
    ::DeviceModel{U, HVDCTwoTerminalLossless},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {T <: FlowRateConstraint, U <: PSY.TwoTerminalHVDC}
    time_steps = get_time_steps(container)
    names = String[]
    modeled_devices = U[]

    for d in devices
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        if ref_bus_from != ref_bus_to
            push!(names, PSY.get_name(d))
            push!(modeled_devices, d)
        end
    end

    var = get_variable(container, FlowActivePowerVariable(), U)
    constraint_ub =
        add_constraints_container!(container, T(), U, names, time_steps; meta = "ub")
    constraint_lb =
        add_constraints_container!(container, T(), U, names, time_steps; meta = "lb")
    for d in modeled_devices
        min_rate, max_rate = _get_flow_bounds(d)
        for t in time_steps
            constraint_ub[PSY.get_name(d), t] = JuMP.@constraint(
                get_jump_model(container),
                var[PSY.get_name(d), t] <= max_rate
            )
            constraint_lb[PSY.get_name(d), t] = JuMP.@constraint(
                get_jump_model(container),
                min_rate <= var[PSY.get_name(d), t]
            )
        end
    end
    return
end

function _add_hvdc_flow_constraints!(
    container::OptimizationContainer,
    devices::Union{Vector{T}, IS.FlattenIteratorWrapper{T}},
    constraint::FlowRateConstraintFromTo,
) where {T <: PSY.TwoTerminalHVDC}
    _add_hvdc_flow_constraints!(
        container,
        devices,
        FlowActivePowerFromToVariable(),
        constraint,
    )
end

function _add_hvdc_flow_constraints!(
    container::OptimizationContainer,
    devices::Union{Vector{T}, IS.FlattenIteratorWrapper{T}},
    constraint::FlowRateConstraintToFrom,
) where {T <: PSY.TwoTerminalHVDC}
    _add_hvdc_flow_constraints!(
        container,
        devices,
        FlowActivePowerToFromVariable(),
        constraint,
    )
end

function _add_hvdc_flow_constraints!(
    container::OptimizationContainer,
    devices::Union{Vector{T}, IS.FlattenIteratorWrapper{T}},
    var::Union{
        FlowActivePowerFromToVariable,
        FlowActivePowerToFromVariable,
        HVDCActivePowerReceivedFromVariable,
        HVDCActivePowerReceivedToVariable,
    },
    constraint::Union{FlowRateConstraintFromTo, FlowRateConstraintToFrom},
) where {T <: PSY.TwoTerminalHVDC}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)

    variable = get_variable(container, var, T)
    constraint_ub =
        add_constraints_container!(container, constraint, T, names, time_steps; meta = "ub")
    constraint_lb =
        add_constraints_container!(container, constraint, T, names, time_steps; meta = "lb")
    for d in devices
        check_hvdc_line_limits_consistency(d)
        max_rate = get_variable_upper_bound(var, d, HVDCTwoTerminalDispatch())
        min_rate = get_variable_lower_bound(var, d, HVDCTwoTerminalDispatch())
        name = PSY.get_name(d)
        for t in time_steps
            constraint_ub[name, t] = JuMP.@constraint(
                get_jump_model(container),
                variable[name, t] <= max_rate
            )
            constraint_lb[name, t] = JuMP.@constraint(
                get_jump_model(container),
                min_rate <= variable[name, t]
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, HVDCTwoTerminalDispatch},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: Union{FlowRateConstraintFromTo, FlowRateConstraintToFrom},
    U <: PSY.TwoTerminalHVDC,
}
    inter_network_branches = U[]
    for d in devices
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        if ref_bus_from != ref_bus_to
            push!(inter_network_branches, d)
        end
    end
    if !isempty(inter_network_branches)
        _add_hvdc_flow_constraints!(container, devices, T())
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    ::DeviceModel{U, HVDCTwoTerminalDispatch},
    ::NetworkModel{<:PM.AbstractDCPModel},
) where {
    T <: Union{FlowRateConstraintToFrom, FlowRateConstraintFromTo},
    U <: PSY.TwoTerminalHVDC,
}
    _add_hvdc_flow_constraints!(container, devices, T())
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    ::DeviceModel{U, HVDCTwoTerminalDispatch},
    ::NetworkModel{<:AbstractPTDFModel},
) where {
    T <: Union{FlowRateConstraintToFrom, FlowRateConstraintFromTo},
    U <: PSY.TwoTerminalHVDC,
}
    _add_hvdc_flow_constraints!(container, devices, T())
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    model::DeviceModel{U, V},
    network_model::NetworkModel{CopperPlatePowerModel},
) where {
    T <: Union{FlowRateConstraintFromTo, FlowRateConstraintToFrom},
    U <: PSY.TwoTerminalHVDC,
    V <: HVDCTwoTerminalPiecewiseLoss,
}
    inter_network_branches = U[]
    for d in devices
        ref_bus_from = get_reference_bus(network_model, PSY.get_arc(d).from)
        ref_bus_to = get_reference_bus(network_model, PSY.get_arc(d).to)
        if ref_bus_from != ref_bus_to
            push!(inter_network_branches, d)
        end
    end
    if !isempty(inter_network_branches)
        if T <: FlowRateConstraintFromTo
            _add_hvdc_flow_constraints!(
                container,
                devices,
                HVDCActivePowerReceivedFromVariable(),
                T(),
            )
        else
            _add_hvdc_flow_constraints!(
                container,
                devices,
                HVDCActivePowerReceivedToVariable(),
                T(),
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{T},
    devices::IS.FlattenIteratorWrapper{U},
    ::DeviceModel{U, V},
    ::NetworkModel{<:AbstractPTDFModel},
) where {
    T <: Union{FlowRateConstraintFromTo, FlowRateConstraintToFrom},
    U <: PSY.TwoTerminalHVDC,
    V <: HVDCTwoTerminalPiecewiseLoss,
}
    if T <: FlowRateConstraintFromTo
        _add_hvdc_flow_constraints!(
            container,
            devices,
            HVDCActivePowerReceivedFromVariable(),
            T(),
        )
    else
        _add_hvdc_flow_constraints!(
            container,
            devices,
            HVDCActivePowerReceivedToVariable(),
            T(),
        )
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCPowerBalance},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:AbstractTwoTerminalDCLineFormulation},
    ::NetworkModel{<:PM.AbstractDCPModel},
) where {T <: PSY.TwoTerminalHVDC}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    tf_var = get_variable(container, FlowActivePowerToFromVariable(), T)
    ft_var = get_variable(container, FlowActivePowerFromToVariable(), T)
    direction_var = get_variable(container, HVDCFlowDirectionVariable(), T)
    losses = get_variable(container, HVDCLosses(), T)

    constraint_ft_ub = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "ft_ub",
    )
    constraint_tf_ub = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "tf_ub",
    )
    constraint_ft_lb = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "tf_lb",
    )
    constraint_tf_lb = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "ft_lb",
    )
    constraint_loss = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "loss",
    )
    constraint_loss_aux1 = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "loss_aux1",
    )
    constraint_loss_aux2 = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "loss_aux2",
    )
    constraint_loss_aux3 = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "loss_aux3",
    )
    constraint_loss_aux4 = add_constraints_container!(
        container,
        HVDCPowerBalance(),
        T,
        names,
        time_steps;
        meta = "loss_aux4",
    )
    for d in devices
        name = PSY.get_name(d)
        loss = PSY.get_loss(d)
        if !isa(loss, PSY.LinearCurve)
            error(
                "HVDCTwoTerminalDispatch of branch $(name) only accepts LinearCurve for loss models.",
            )
        end
        l1 = PSY.get_proportional_term(loss)
        l0 = PSY.get_constant_term(loss)
        R_min_from, R_max_from = PSY.get_active_power_limits_from(d)
        R_min_to, R_max_to = PSY.get_active_power_limits_to(d)
        for t in get_time_steps(container)
            constraint_tf_ub[name, t] = JuMP.@constraint(
                get_jump_model(container),
                tf_var[name, t] <= R_max_to * direction_var[name, t]
            )
            constraint_tf_lb[name, t] = JuMP.@constraint(
                get_jump_model(container),
                tf_var[name, t] >= R_min_to * (1 - direction_var[name, t])
            )
            constraint_ft_ub[name, t] = JuMP.@constraint(
                get_jump_model(container),
                ft_var[name, t] <= R_max_from * (1 - direction_var[name, t])
            )
            constraint_ft_lb[name, t] = JuMP.@constraint(
                get_jump_model(container),
                ft_var[name, t] >= R_min_from * direction_var[name, t]
            )
            constraint_loss[name, t] = JuMP.@constraint(
                get_jump_model(container),
                tf_var[name, t] + ft_var[name, t] == losses[name, t]
            )
            constraint_loss_aux1[name, t] = JuMP.@constraint(
                get_jump_model(container),
                losses[name, t] >= l0 + l1 * ft_var[name, t]
            )
            constraint_loss_aux2[name, t] = JuMP.@constraint(
                get_jump_model(container),
                losses[name, t] >= l0 + l1 * tf_var[name, t]
            )
            constraint_loss_aux3[name, t] = JuMP.@constraint(
                get_jump_model(container),
                losses[name, t] <=
                l0 + l1 * ft_var[name, t] + M_VALUE * direction_var[name, t]
            )
            constraint_loss_aux4[name, t] = JuMP.@constraint(
                get_jump_model(container),
                losses[name, t] <=
                l0 + l1 * tf_var[name, t] + M_VALUE * (1 - direction_var[name, t])
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCRectifierDCLineVoltageConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    rect_dc_voltage_var = get_variable(container, HVDCRectifierDCVoltageVariable(), T)
    rect_ac_voltage_bus_var = get_variable(container, VoltageMagnitude(), PSY.ACBus)
    rect_delay_angle_var = get_variable(container, HVDCRectifierDelayAngleVariable(), T)
    rect_tap_setting_var = get_variable(container, HVDCRectifierTapSettingVariable(), T)
    dc_line_current_var = get_variable(container, DCLineCurrentFlowVariable(), T)

    constraint_rect_dc_volt = add_constraints_container!(
        container,
        HVDCRectifierDCLineVoltageConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)
        rect_bridges = PSY.get_rectifier_bridges(d)
        dc_rect_com_reactance = PSY.get_rectifier_xc(d)
        rect_tap_ratio = PSY.get_rectifier_transformer_ratio(d)
        bus_from = PSY.get_arc(d).from
        bus_from_name = PSY.get_name(bus_from)

        for t in get_time_steps(container)
            constraint_rect_dc_volt[name, t] = JuMP.@constraint(
                get_jump_model(container),
                rect_dc_voltage_var[name, t] ==
                (3 * rect_bridges / pi) * (
                    sqrt(2) * (
                        rect_tap_ratio *
                        rect_ac_voltage_bus_var[bus_from_name, t] *
                        cos(rect_delay_angle_var[name, t])
                    ) / rect_tap_setting_var[name, t] -
                    dc_rect_com_reactance * dc_line_current_var[name, t]
                )
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCInverterDCLineVoltageConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    inv_dc_voltage_var = get_variable(container, HVDCInverterDCVoltageVariable(), T)
    inv_ac_voltage_bus_var = get_variable(container, VoltageMagnitude(), PSY.ACBus)
    inv_extinction_angle_var =
        get_variable(container, HVDCInverterExtinctionAngleVariable(), T)
    inv_tap_setting_var = get_variable(container, HVDCInverterTapSettingVariable(), T)
    dc_line_current_var = get_variable(container, DCLineCurrentFlowVariable(), T)

    constraint_inv_dc_volt = add_constraints_container!(
        container,
        HVDCInverterDCLineVoltageConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)
        inv_bridges = PSY.get_inverter_bridges(d)
        dc_inv_com_reactance = PSY.get_inverter_xc(d)
        inv_tap_ratio = PSY.get_inverter_transformer_ratio(d)
        bus_to = PSY.get_arc(d).to
        bus_to_name = PSY.get_name(bus_to)

        for t in get_time_steps(container)
            constraint_inv_dc_volt[name, t] = JuMP.@constraint(
                get_jump_model(container),
                inv_dc_voltage_var[name, t] ==
                (3 * inv_bridges / pi) * (
                    sqrt(2) * (
                        inv_tap_ratio *
                        inv_ac_voltage_bus_var[bus_to_name, t] *
                        cos(inv_extinction_angle_var[name, t])
                    ) / inv_tap_setting_var[name, t] -
                    dc_inv_com_reactance * dc_line_current_var[name, t]
                )
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCRectifierOverlapAngleConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    rect_ac_voltage_bus_var = get_variable(container, VoltageMagnitude(), PSY.ACBus)
    rect_delay_angle_var = get_variable(container, HVDCRectifierDelayAngleVariable(), T)
    rect_overlap_angle_var = get_variable(container, HVDCRectifierOverlapAngleVariable(), T)
    rect_tap_setting_var = get_variable(container, HVDCRectifierTapSettingVariable(), T)
    dc_line_current_var = get_variable(container, DCLineCurrentFlowVariable(), T)

    constraint_rect_over_ang = add_constraints_container!(
        container,
        HVDCRectifierOverlapAngleConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)
        dc_rect_com_reactance = PSY.get_rectifier_xc(d)
        rect_tap_ratio = PSY.get_rectifier_transformer_ratio(d)
        bus_from = PSY.get_arc(d).from
        bus_from_name = PSY.get_name(bus_from)

        for t in get_time_steps(container)
            constraint_rect_over_ang[name, t] = JuMP.@constraint(
                get_jump_model(container),
                rect_overlap_angle_var[name, t] == (
                    acos(
                        cos(rect_delay_angle_var[name, t])
                        -
                        (
                            (
                                sqrt(2) * dc_rect_com_reactance *
                                dc_line_current_var[name, t] *
                                rect_tap_setting_var[name, t]
                            )
                            /
                            (
                                rect_tap_ratio *
                                rect_ac_voltage_bus_var[bus_from_name, t]
                            )
                        ),
                    )
                    -
                    rect_delay_angle_var[name, t]
                )
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCInverterOverlapAngleConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    inv_ac_voltage_bus_var = get_variable(container, VoltageMagnitude(), PSY.ACBus)
    inv_extinction_angle_var =
        get_variable(container, HVDCInverterExtinctionAngleVariable(), T)
    inv_overlap_angle_var = get_variable(container, HVDCInverterOverlapAngleVariable(), T)
    inv_tap_setting_var = get_variable(container, HVDCInverterTapSettingVariable(), T)
    dc_line_current_var = get_variable(container, DCLineCurrentFlowVariable(), T)

    constraint_inv_over_ang = add_constraints_container!(
        container,
        HVDCInverterOverlapAngleConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)
        dc_inv_com_reactance = PSY.get_inverter_xc(d)
        inv_tap_ratio = PSY.get_inverter_transformer_ratio(d)
        bus_to = PSY.get_arc(d).to
        bus_to_name = PSY.get_name(bus_to)

        for t in get_time_steps(container)
            constraint_inv_over_ang[name, t] = JuMP.@constraint(
                get_jump_model(container),
                inv_overlap_angle_var[name, t] == (
                    acos(
                        cos(inv_extinction_angle_var[name, t])
                        -
                        (
                            (
                                sqrt(2) * dc_inv_com_reactance *
                                dc_line_current_var[name, t] *
                                inv_tap_setting_var[name, t]
                            )
                            /
                            (
                                inv_tap_ratio *
                                inv_ac_voltage_bus_var[bus_to_name, t]
                            )
                        ),
                    )
                    -
                    inv_extinction_angle_var[name, t]
                )
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCRectifierPowerFactorAngleConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    rect_delay_angle_var = get_variable(container, HVDCRectifierDelayAngleVariable(), T)
    rect_overlap_angle_var = get_variable(container, HVDCRectifierOverlapAngleVariable(), T)
    rect_power_factor_var =
        get_variable(container, HVDCRectifierPowerFactorAngleVariable(), T)

    constraint_rect_power_factor_ang = add_constraints_container!(
        container,
        HVDCRectifierPowerFactorAngleConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)

        for t in get_time_steps(container)
            constraint_rect_power_factor_ang[name, t] = JuMP.@constraint(
                get_jump_model(container),
                # Full equation not working with Ipopt
                # rect_power_factor_var[name, t] *
                #     (
                #         cos(2 * rect_delay_angle_var[name, t]) - cos(
                #             2(
                #                 rect_overlap_angle_var[name, t] +
                #                 rect_delay_angle_var[name, t]
                #             ),
                #         )
                #     ) == atan(
                #     (
                #         - 2 * rect_overlap_angle_var[name, t] +
                #         - sin(2 * rect_delay_angle_var[name, t]) + sin(
                #             2 * (
                #                 rect_overlap_angle_var[name, t] +
                #                 rect_delay_angle_var[name, t]
                #             ),
                #         )
                #     )
                # )

                # Approximation of rectifier power factor calculation
                rect_power_factor_var[name, t] == acos(
                    0.5 * cos(rect_delay_angle_var[name, t]) +
                    0.5 * cos(
                        rect_delay_angle_var[name, t] + rect_overlap_angle_var[name, t],
                    ),
                )
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCInverterPowerFactorAngleConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    inv_extinction_angle_var =
        get_variable(container, HVDCInverterExtinctionAngleVariable(), T)
    inv_overlap_angle_var = get_variable(container, HVDCInverterOverlapAngleVariable(), T)
    inv_power_factor_var =
        get_variable(container, HVDCInverterPowerFactorAngleVariable(), T)

    constraint_inv_power_factor_ang = add_constraints_container!(
        container,
        HVDCInverterPowerFactorAngleConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)

        for t in get_time_steps(container)
            constraint_inv_power_factor_ang[name, t] = JuMP.@constraint(
                get_jump_model(container),
                # Full equation not working with Ipopt
                # inv_power_factor_var[name, t] *
                #     (
                #         cos(2 * inv_extinction_angle_var[name, t]) - cos(
                #             2(
                #                 inv_overlap_angle_var[name, t] +
                #                 inv_extinction_angle_var[name, t]
                #             ),
                #         )
                #     ) == atan(
                #     (
                #         - 2 * inv_overlap_angle_var[name, t] +
                #         - sin(2 * inv_extinction_angle_var[name, t]) + sin(
                #             2 * (
                #                 inv_overlap_angle_var[name, t] +
                #                 inv_extinction_angle_var[name, t]
                #             ),
                #         )
                #     )
                # )

                # Approximation of inverter power factor calculation
                inv_power_factor_var[name, t] == acos(
                    0.5 * cos(inv_extinction_angle_var[name, t]) +
                    0.5 * cos(
                        inv_extinction_angle_var[name, t] + inv_overlap_angle_var[name, t],
                    ),
                )
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCRectifierACCurrentFlowConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    rect_ac_current_var = get_variable(container, HVDCRectifierACCurrentVariable(), T)
    dc_line_current_var = get_variable(container, DCLineCurrentFlowVariable(), T)

    constraint_rect_ac_current = add_constraints_container!(
        container,
        HVDCRectifierACCurrentFlowConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)
        rect_bridges = PSY.get_rectifier_bridges(d)

        for t in get_time_steps(container)
            constraint_rect_ac_current[name, t] = JuMP.@constraint(
                get_jump_model(container),
                rect_ac_current_var[name, t] ==
                sqrt(6) * rect_bridges * dc_line_current_var[name, t] / pi
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCInverterACCurrentFlowConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    inv_ac_current_var = get_variable(container, HVDCInverterACCurrentVariable(), T)
    dc_line_current_var = get_variable(container, DCLineCurrentFlowVariable(), T)

    constraint_inv_ac_current = add_constraints_container!(
        container,
        HVDCInverterACCurrentFlowConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)
        inv_bridges = PSY.get_inverter_bridges(d)

        for t in get_time_steps(container)
            constraint_inv_ac_current[name, t] = JuMP.@constraint(
                get_jump_model(container),
                inv_ac_current_var[name, t] ==
                sqrt(6) * inv_bridges * dc_line_current_var[name, t] / pi
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCRectifierPowerCalculationConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    rect_ac_ppower_var = get_variable(container, HVDCActivePowerReceivedFromVariable(), T)
    rect_ac_qpower_var = get_variable(container, HVDCReactivePowerReceivedFromVariable(), T)
    rect_ac_current_var = get_variable(container, HVDCRectifierACCurrentVariable(), T)
    rect_ac_voltage_bus_var = get_variable(container, VoltageMagnitude(), PSY.ACBus)
    rect_power_factor_var =
        get_variable(container, HVDCRectifierPowerFactorAngleVariable(), T)
    rect_tap_setting_var = get_variable(container, HVDCRectifierTapSettingVariable(), T)

    constraint_ft_p = add_constraints_container!(
        container,
        HVDCRectifierPowerCalculationConstraint(),
        T,
        names,
        time_steps;
        meta = "active",
    )
    constraint_ft_q = add_constraints_container!(
        container,
        HVDCRectifierPowerCalculationConstraint(),
        T,
        names,
        time_steps;
        meta = "reactive",
    )

    for d in devices
        name = PSY.get_name(d)
        rect_tap_ratio = PSY.get_rectifier_transformer_ratio(d)
        bus_from = PSY.get_arc(d).from
        bus_from_name = PSY.get_name(bus_from)

        for t in get_time_steps(container)
            constraint_ft_p[name, t] = JuMP.@constraint(
                get_jump_model(container),
                rect_ac_ppower_var[name, t] ==
                (
                    rect_tap_ratio * sqrt(3) * rect_ac_current_var[name, t]
                    * rect_ac_voltage_bus_var[bus_from_name, t] *
                    cos(rect_power_factor_var[name, t])
                ) / rect_tap_setting_var[name, t],
            )
            constraint_ft_q[name, t] = JuMP.@constraint(
                get_jump_model(container),
                rect_ac_qpower_var[name, t] ==
                (
                    rect_tap_ratio * sqrt(3) * rect_ac_current_var[name, t]
                    * rect_ac_voltage_bus_var[bus_from_name, t] *
                    sin(rect_power_factor_var[name, t])
                ) / rect_tap_setting_var[name, t],
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCInverterPowerCalculationConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    inv_ac_ppower_var = get_variable(container, HVDCActivePowerReceivedToVariable(), T)
    inv_ac_qpower_var = get_variable(container, HVDCReactivePowerReceivedToVariable(), T)
    inv_ac_current_var = get_variable(container, HVDCInverterACCurrentVariable(), T)
    inv_ac_voltage_bus_var = get_variable(container, VoltageMagnitude(), PSY.ACBus)
    inv_power_factor_var =
        get_variable(container, HVDCInverterPowerFactorAngleVariable(), T)
    inv_tap_setting_var = get_variable(container, HVDCInverterTapSettingVariable(), T)

    constraint_ft_p = add_constraints_container!(
        container,
        HVDCInverterPowerCalculationConstraint(),
        T,
        names,
        time_steps;
        meta = "active",
    )
    constraint_ft_q = add_constraints_container!(
        container,
        HVDCInverterPowerCalculationConstraint(),
        T,
        names,
        time_steps;
        meta = "reactive",
    )

    for d in devices
        name = PSY.get_name(d)
        inv_tap_ratio = PSY.get_inverter_transformer_ratio(d)
        bus_to = PSY.get_arc(d).to
        bus_to_name = PSY.get_name(bus_to)

        for t in get_time_steps(container)
            constraint_ft_p[name, t] = JuMP.@constraint(
                get_jump_model(container),
                inv_ac_ppower_var[name, t] ==
                (
                    inv_tap_ratio * sqrt(3) * inv_ac_current_var[name, t]
                    * inv_ac_voltage_bus_var[bus_to_name, t] *
                    cos(inv_power_factor_var[name, t])
                ) / inv_tap_setting_var[name, t],
            )
            constraint_ft_q[name, t] = JuMP.@constraint(
                get_jump_model(container),
                inv_ac_qpower_var[name, t] ==
                (
                    inv_tap_ratio * sqrt(3) * inv_ac_current_var[name, t]
                    * inv_ac_voltage_bus_var[bus_to_name, t] *
                    sin(inv_power_factor_var[name, t])
                ) / inv_tap_setting_var[name, t],
            )
        end
    end
    return
end

function add_constraints!(
    container::OptimizationContainer,
    ::Type{HVDCTransmissionDCLineConstraint},
    devices::IS.FlattenIteratorWrapper{T},
    ::DeviceModel{T, <:HVDCTwoTerminalLCC},
    ::NetworkModel{PM.ACPPowerModel},
) where {T <: PSY.TwoTerminalLCCLine}
    time_steps = get_time_steps(container)
    names = PSY.get_name.(devices)
    rect_dc_voltage_var = get_variable(container, HVDCRectifierDCVoltageVariable(), T)
    inv_dc_voltage_var = get_variable(container, HVDCInverterDCVoltageVariable(), T)
    dc_line_current_var = get_variable(container, DCLineCurrentFlowVariable(), T)

    constraint_tl_c = add_constraints_container!(
        container,
        HVDCTransmissionDCLineConstraint(),
        T,
        names,
        time_steps;
    )

    for d in devices
        name = PSY.get_name(d)
        dc_line_resistance = PSY.get_r(d)

        for t in get_time_steps(container)
            constraint_tl_c[name, t] = JuMP.@constraint(
                get_jump_model(container),
                inv_dc_voltage_var[name, t] ==
                rect_dc_voltage_var[name, t] -
                dc_line_resistance * dc_line_current_var[name, t]
            )
        end
    end
    return
end
