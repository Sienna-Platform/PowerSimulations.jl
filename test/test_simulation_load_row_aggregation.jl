# Two static loads share bus 2, so their series are summed into one parameter per step.
function _shared_bus_load_simulation(rebuild)
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; force_build = true)
    load = PSY.get_component(PowerLoad, sys, "Bus2")
    twin = PowerLoad(;
        name = "Bus2_twin",
        available = true,
        bus = PSY.get_bus(load),
        active_power = 0.5,
        reactive_power = 0.1,
        base_power = 100.0,
        max_active_power = 0.5,
        max_reactive_power = 0.1,
        input_basis = u"CU",
    )
    PSY.add_component!(sys, twin)
    PSY.copy_time_series!(twin, load)
    models = SimulationModels([
        DecisionModel(
            get_template_nomin_ed_simulation(),
            sys;
            name = "ED",
            optimizer = HiGHS_optimizer,
            rebuild_model = rebuild,
        ),
    ])
    sim = Simulation(;
        name = "row_aggregation",
        steps = 2,
        models = models,
        sequence = SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
        ),
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return sim, load, twin
end

function _realized_thermal(sim)
    outputs = get_decision_problem_results(SimulationResults(sim), "ED")
    return read_realized_variables(outputs; table_format = TableFormat.WIDE)["ActivePowerVariable__ThermalStandard"]
end

@testset "Aggregated load parameters refresh each simulation step" begin
    recurrent, load, twin = _shared_bus_load_simulation(false)
    rebuilt, _, _ = _shared_bus_load_simulation(true)

    model = PSI.get_simulation_model(recurrent, :ED)
    param_container = IOM.get_parameter(
        IOM.get_optimization_container(model),
        POM.ActivePowerTimeSeriesParameter,
        PowerLoad,
    )
    bus_number = PSY.get_number(PSY.get_bus(load))
    refs, coefficient = IOM.get_row_aggregate_refs(param_container, bus_number)
    @test eltype(refs) == JuMP.VariableRef
    values = collect(
        IOM.get_parameter_column_values(param_container, PSY.get_name(load)),
    )
    multiplier = IOM.get_multiplier_array(param_container)
    expected = [
        values[t] * (multiplier[PSY.get_name(load), t] + multiplier[PSY.get_name(twin), t])
        for t in eachindex(values)
    ]
    @test coefficient .* JuMP.fix_value.(refs) ≈ expected

    @test Matrix(_realized_thermal(recurrent)[!, 2:end]) ≈
          Matrix(_realized_thermal(rebuilt)[!, 2:end])
end
