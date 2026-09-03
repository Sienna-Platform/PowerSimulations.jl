function _default_devices_uc()
    return [
        DeviceModel(PSY.ThermalStandard, ThermalBasicUnitCommitment),
        DeviceModel(PSY.RenewableDispatch, RenewableFullDispatch),
        DeviceModel(PSY.RenewableNonDispatch, FixedOutput),
        DeviceModel(PSY.PowerLoad, StaticPowerLoad),
        DeviceModel(PSY.InterruptiblePowerLoad, PowerLoadInterruption),
        DeviceModel(PSY.Line, StaticBranch),
        DeviceModel(PSY.TwoWindingTransformer, StaticBranch),
        DeviceModel(PSY.TwoTerminalGenericHVDCLine, HVDCTwoTerminalDispatch),
    ]
end

function _default_devices_dispatch()
    default = _default_devices_uc()
    default[1] = DeviceModel(PSY.ThermalStandard, ThermalBasicDispatch)
    return default
end

function _default_services()
    return [
        ServiceModel(PSY.OnlineReserve{PSY.ReserveUp}, RangeReserve),
        ServiceModel(PSY.OnlineReserve{PSY.ReserveDown}, RangeReserve),
    ]
end

"""
    template_unit_commitment(; kwargs...)

Creates a `PowerOperationsProblemTemplate` with default DeviceModels for a Unit Commitment
problem.

# Example

template = template_unit_commitment()

```

# Accepted Key Words
- `network::Type{<:AbstractNetworkModel}` : override default network model settings
- `devices::Vector{DeviceModel}` : override default `DeviceModel` settings
- `services::Vector{ServiceModel}` : override default `ServiceModel` settings
```
"""
function template_unit_commitment(; kwargs...)
    network = get(kwargs, :network, CopperPlateNetworkModel)
    template = PowerOperationsProblemTemplate(network)
    for model in get(kwargs, :devices, _default_devices_uc())
        set_device_model!(template, model)
    end

    for model in get(kwargs, :services, _default_services())
        set_service_model!(template, model)
    end
    return template
end

"""
    template_economic_dispatch(; kwargs...)

Creates a `PowerOperationsProblemTemplate` with default DeviceModels for an Economic Dispatch
problem.

# Example

template = template_economic_dispatch()

```

# Accepted Key Words
- `network::Type{<:AbstractNetworkModel}` : override default network model settings
- `devices::Vector{DeviceModel}` : override default `DeviceModel` settings
- `services::Vector{ServiceModel}` : override default `ServiceModel` settings
```
"""
function template_economic_dispatch(; kwargs...)
    network = get(kwargs, :network, CopperPlateNetworkModel)
    template = PowerOperationsProblemTemplate(network)
    for model in get(kwargs, :devices, _default_devices_dispatch())
        set_device_model!(template, model)
    end

    for model in get(kwargs, :services, _default_services())
        set_service_model!(template, model)
    end

    return template
end
