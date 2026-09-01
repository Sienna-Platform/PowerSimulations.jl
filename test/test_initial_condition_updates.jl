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
    return initial_conditions[1], dataset
end

@testset "DeviceStatus initial condition updates round OnVariable values" begin
    value = 0.9999997
    for source_type in (:simulation_state, :in_memory_store)
        initial_condition, dataset =
            _make_initial_condition_update_fixture(source_type, value)
        @test PSI.get_condition(initial_condition) === 1.0
        @test PSI.get_last_recorded_value(dataset)["test-device"] === value
    end
end

@testset "DeviceStatus initial condition writes round OnVariable values" begin
    device = DeviceStatusInitialConditionTestDevice("test-device")
    model = JuMP.Model()
    variable = JuMP.@variable(model, status)
    initial_condition = PSI.InitialCondition{PSI.DeviceStatus, JuMP.VariableRef}(
        device,
        variable,
    )

    PSI.set_ic_quantity!(initial_condition, 0.9999997)

    @test JuMP.fix_value(variable) === 1.0
end
