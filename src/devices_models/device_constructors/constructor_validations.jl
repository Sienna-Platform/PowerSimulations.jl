function validate_available_devices(
    model::DeviceModel{T, <:AbstractDeviceFormulation},
    system::PSY.System,
) where {T <: PSY.Device}
    devices = get_available_components(model, system)
    return !isempty(devices)
end