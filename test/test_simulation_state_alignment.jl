@testset "align_to_dataset_grid" begin
    grid_start = DateTime("2024-01-01T00:00:00")
    resolution = Millisecond(Hour(1))
    dataset = PSI.InMemoryDataset(0.0, grid_start, resolution, 0, 5, ["gen"])

    # on-grid timestamps are fixed points
    on_grid = DateTime("2024-01-01T02:00:00")
    @test PSI.align_to_dataset_grid(dataset, on_grid) == on_grid
    @test PSI.find_aligned_timestamp_index(dataset, on_grid) == 3

    # off-grid timestamps floor to the containing row
    @test PSI.align_to_dataset_grid(dataset, DateTime("2024-01-01T00:05:00")) == grid_start
    @test PSI.align_to_dataset_grid(dataset, DateTime("2024-01-01T00:59:00")) == grid_start
    @test PSI.align_to_dataset_grid(dataset, DateTime("2024-01-01T01:00:00")) ==
          DateTime("2024-01-01T01:00:00")
    @test PSI.find_aligned_timestamp_index(dataset, DateTime("2024-01-01T00:05:00")) == 1
    @test PSI.find_aligned_timestamp_index(dataset, DateTime("2024-01-01T00:59:00")) == 1
    @test PSI.find_aligned_timestamp_index(dataset, DateTime("2024-01-01T01:00:00")) == 2

    # a timestamp before the grid start has no containing row
    @test_throws ErrorException PSI.align_to_dataset_grid(
        dataset,
        DateTime("2023-12-31T23:00:00"),
    )
end

@testset "duration aux variables scale to the system-state resolution for every device type" begin
    # A decision model at hourly resolution writes TimeDurationOn in hours; the system state
    # runs at the finest simulation resolution (5 minutes here), so the copied value must be
    # rescaled by 12 regardless of the device type carrying the key.
    ts = DateTime("2024-01-01T00:00:00")
    for device_type in (ThermalStandard, ThermalMultiStart)
        key = PSI.AuxVarKey(PSI.TimeDurationOn, device_type)
        decision = PSI.DatasetContainer{PSI.InMemoryDataset}()
        PSI.set_dataset!(
            decision,
            key,
            PSI.InMemoryDataset(0.0, ts, Millisecond(Hour(1)), 0, 3, ["gen"]),
        )
        PSI.get_dataset(decision, key).values["gen", 1] = 3.0
        system = PSI.DatasetContainer{PSI.InMemoryDataset}()
        PSI.set_dataset!(
            system,
            key,
            PSI.make_system_state(ts, Millisecond(Minute(5)), (["gen"],)),
        )
        PSI.update_system_state!(system, key, decision, ts)
        @test PSI.get_dataset_values(system, key)["gen", 1] == 36.0
    end
end

@testset "3-D decision state takes outputs with an integer second axis" begin
    # The state names its second axis with strings; a block or segment index in the output
    # is an integer axis.
    ts = DateTime("2024-01-01T00:00:00")
    resolution = Millisecond(Hour(1))
    key = PSI.VariableKey(ActivePowerVariable, ThermalStandard)
    state = PSI.SimulationState()
    PSI.set_dataset!(
        PSI.get_decision_states(state),
        key,
        PSI.InMemoryDataset(0.0, ts, resolution, 1, 2, (["gen"], ["1", "2"])),
    )
    store_data = DenseAxisArray(reshape([1.0, 2.0, 3.0, 4.0], 1, 2, 2), ["gen"], 1:2, 1:2)
    params = PSI.ModelStoreParams(1, 2, resolution, resolution, 100.0, UUIDs.uuid4())
    PSI.update_decision_state!(state, key, store_data, ts, params)
    values = PSI.get_decision_state_value(state, key)
    @test values["gen", "1", :].data == [1.0, 3.0]
    @test values["gen", "2", :].data == [2.0, 4.0]
end
