function add_feedforward_arguments!(
    container::OptimizationContainer,
    model::DeviceModel,
    devices::IS.FlattenIteratorWrapper{V},
) where {V <: PSY.Component}
    for ff in get_feedforwards(model)
        @debug "arguments" ff V _group = LOG_GROUP_FEEDFORWARDS_CONSTRUCTION
        _add_feedforward_arguments!(container, model, devices, ff)
    end
    return
end

function add_feedforward_arguments!(
    container::OptimizationContainer,
    model::ServiceModel,
    service::V,
) where {V <: PSY.AbstractReserve}
    for ff in get_feedforwards(model)
        @debug "arguments" ff V _group = LOG_GROUP_FEEDFORWARDS_CONSTRUCTION
        contributing_devices = get_contributing_devices(model)
        _add_feedforward_arguments!(container, model, contributing_devices, ff)
    end
    return
end

function add_feedforward_arguments!(
    ::OptimizationContainer,
    model::ServiceModel,
    ::PSY.TransmissionInterface,
)
    # Currently we do not support feedforwards for TransmissionInterface
    ffs = get_feedforwards(model)
    if !isempty(ffs)
        throw(
            ArgumentError(
                "TransmissionInterface data types currently do not support feedforwards.",
            ),
        )
    end
    return
end

function _add_feedforward_arguments!(
    container::OptimizationContainer,
    model::DeviceModel{T, U},
    devices::IS.FlattenIteratorWrapper{T},
    ff::AbstractAffectFeedforward,
) where {T <: PSY.Device, U <: AbstractDeviceFormulation}
    parameter_type = get_default_parameter_type(ff, T)
    add_parameters!(container, parameter_type, ff, model, devices)
    return
end

function _add_feedforward_arguments!(
    container::OptimizationContainer,
    model::ServiceModel{T, U},
    contributing_devices::Vector,
    ff::AbstractAffectFeedforward,
) where {T <: PSY.AbstractReserve, U <: AbstractServiceFormulation}
    parameter_type = get_default_parameter_type(ff, U)
    add_parameters!(container, parameter_type, ff, model, contributing_devices)
    return
end

function _add_feedforward_slack_variables!(container::OptimizationContainer,
    ::T,
    ff::Union{LowerBoundFeedforward, UpperBoundFeedforward},
    model::ServiceModel{U, V},
    devices::Vector,
) where {
    T <: Union{LowerBoundFeedForwardSlack, UpperBoundFeedForwardSlack},
    U <: PSY.AbstractReserve,
    V <: AbstractReservesFormulation,
}
    time_steps = get_time_steps(container)
    for var in get_affected_values(ff)
        variable = get_variable(container, var)
        device_name_set, set_time = JuMP.axes(variable)
        device_names = PSY.get_name.(devices)
        @assert issetequal(device_name_set, device_names)
        IS.@assert_op set_time == time_steps
        service_name = get_service_name(model)
        var_type = get_entry_type(var)
        variable_container = add_variable_container!(
            container,
            T(),
            U,
            device_names,
            time_steps;
            meta = "$(var_type)_$(service_name)",
        )

        for t in time_steps, name in device_name_set
            variable_container[name, t] = JuMP.@variable(
                get_jump_model(container),
                base_name = "$(T)_$(U)_{$(name), $(t)}",
                lower_bound = 0.0
            )
            add_to_objective_invariant_expression!(
                container,
                variable_container[name, t] * BALANCE_SLACK_COST,
            )
        end
    end
    return
end

function _add_feedforward_slack_variables!(
    container::OptimizationContainer,
    ::T,
    ff::Union{LowerBoundFeedforward, UpperBoundFeedforward},
    model::DeviceModel{U, V},
    devices::IS.FlattenIteratorWrapper{U},
) where {
    T <: Union{LowerBoundFeedForwardSlack, UpperBoundFeedForwardSlack},
    U <: PSY.Device,
    V <: AbstractDeviceFormulation,
}
    time_steps = get_time_steps(container)
    for var in get_affected_values(ff)
        variable = get_variable(container, var)
        device_name_set, set_time = JuMP.axes(variable)
        devices_names = PSY.get_name.(devices)
        @assert issetequal(device_name_set, devices_names)
        IS.@assert_op set_time == time_steps

        var_type = get_entry_type(var)
        variable = add_variable_container!(
            container,
            T(),
            U,
            devices_names,
            time_steps;
            meta = "$(var_type)",
        )

        for t in time_steps, name in device_name_set
            variable[name, t] = JuMP.@variable(
                get_jump_model(container),
                base_name = "$(T)_$(U)_{$(name), $(t)}",
                lower_bound = 0.0
            )
        end
    end
    return
end

function _add_feedforward_arguments!(
    container::OptimizationContainer,
    model::DeviceModel{T, U},
    devices::IS.FlattenIteratorWrapper{T},
    ff::UpperBoundFeedforward,
) where {T <: PSY.Device, U <: AbstractDeviceFormulation}
    parameter_type = get_default_parameter_type(ff, T)
    add_parameters!(container, parameter_type, ff, model, devices)
    if get_slacks(ff)
        _add_feedforward_slack_variables!(
            container,
            UpperBoundFeedForwardSlack(),
            ff,
            model,
            devices,
        )
    end
    return
end

function _add_feedforward_arguments!(
    container::OptimizationContainer,
    model::ServiceModel{T, U},
    contributing_devices::Vector,
    ff::UpperBoundFeedforward,
) where {T <: PSY.AbstractReserve, U <: AbstractServiceFormulation}
    parameter_type = get_default_parameter_type(ff, SR)
    add_parameters!(container, parameter_type, ff, model, contributing_devices)
    if get_slacks(ff)
        _add_feedforward_slack_variables!(
            container,
            UpperBoundFeedForwardSlack(),
            ff,
            model,
            contributing_devices,
        )
    end
    return
end

function _add_feedforward_arguments!(
    container::OptimizationContainer,
    model::DeviceModel{T, U},
    devices::IS.FlattenIteratorWrapper{T},
    ff::LowerBoundFeedforward,
) where {T <: PSY.Device, U <: AbstractDeviceFormulation}
    parameter_type = get_default_parameter_type(ff, T)
    add_parameters!(container, parameter_type, ff, model, devices)
    if get_slacks(ff)
        _add_feedforward_slack_variables!(
            container,
            LowerBoundFeedForwardSlack(),
            ff,
            model,
            devices,
        )
    end
    return
end

function _add_feedforward_arguments!(
    container::OptimizationContainer,
    model::ServiceModel{T, U},
    contributing_devices::Vector{V},
    ff::LowerBoundFeedforward,
) where {T <: PSY.AbstractReserve, U <: AbstractReservesFormulation, V <: PSY.Component}
    parameter_type = get_default_parameter_type(ff, T)
    add_parameters!(container, parameter_type, ff, model, contributing_devices)
    if get_slacks(ff)
        _add_feedforward_slack_variables!(
            container,
            LowerBoundFeedForwardSlack(),
            ff,
            model,
            contributing_devices,
        )
    end
    return
end

function _add_feedforward_arguments!(
    container::OptimizationContainer,
    model::DeviceModel{T, U},
    devices::IS.FlattenIteratorWrapper{T},
    ff::SemiContinuousFeedforward,
) where {T <: PSY.Device, U <: AbstractDeviceFormulation}
    parameter_type = get_default_parameter_type(ff, T)
    add_parameters!(container, parameter_type, ff, model, devices)
    add_to_expression!(
        container,
        ActivePowerRangeExpressionUB,
        parameter_type(),
        devices,
        model,
    )
    add_to_expression!(
        container,
        ActivePowerRangeExpressionLB,
        parameter_type(),
        devices,
        model,
    )
    return
end
