struct DeviceStatusInitialConditionTestDevice <: PSY.Device
    name::String
end

PSY.get_name(device::DeviceStatusInitialConditionTestDevice) = device.name

function _make_initial_condition_update_fixture(source_type::Symbol, value::Float64)
    device = DeviceStatusInitialConditionTestDevice("test-device")
    initial_condition = PSI.InitialCondition{PSI.DeviceStatus, Float64}(device, 0.0)
    initial_conditions = Vector{
        Union{
            PSI.InitialCondition{PSI.DeviceStatus, Float64},
            PSI.InitialCondition{PSI.DeviceStatus, Nothing},
        },
    }(
        undef,
        1,
    )
    initial_conditions[1] = initial_condition

    key = PSI.VariableKey(PSI.OnVariable, typeof(device))
    values = DenseAxisArray(
        reshape([value], 1, 1),
        [PSY.get_name(device)],
        1:1,
    )
    dataset = PSI.InMemoryDataset(values)
    PSI.set_last_recorded_row!(dataset, 1)

    source = if source_type == :simulation_state
        state = PSI.SimulationState()
        PSI.set_dataset!(PSI.get_system_states(state), key, dataset)
        state
    elseif source_type == :in_memory_store
        store = PSI.EmulationModelStore()
        PSI.set_dataset!(store.data_container, key, dataset)
        store
    else
        error("Unsupported source type $source_type")
    end

    PSI.update_initial_conditions!(initial_conditions, source, Dates.Millisecond(0))
    return PSI.get_condition(initial_conditions[1])
end

@testset "DeviceStatus initial condition updates round OnVariable values" begin
    value = 0.9999997
    for source_type in (:simulation_state, :in_memory_store)
        result = _make_initial_condition_update_fixture(source_type, value)
        @test result === 1.0
    end
end
