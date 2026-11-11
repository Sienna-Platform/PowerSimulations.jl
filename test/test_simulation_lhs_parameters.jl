# A deployed fraction is a constraint coefficient, so a model with one is rebuilt every step.

const _LHS_HORIZON = Hour(4)
const _LHS_FRACTION = 0.5

function _lhs_storage_system(profile::Vector{Float64})
    sys = PSB.build_system(
        PSITestSystems,
        "c_sys5_bat";
        add_single_time_series = true,
        add_reserves = true,
    )
    for r in PSY.get_components(PSY.has_demand_curve, PSY.OnlineReserve, sys)
        PSY.set_available!(r, false)
    end
    reserve = only([
        r for r in PSY.get_components(PSY.OnlineReserve{PSY.ReserveUp}, sys) if
        PSY.get_available(r)
    ])
    PSY.set_deployed_fraction!(reserve, _LHS_FRACTION)
    stamps = collect(
        range(DateTime("2024-01-01T00:00:00"); step = Hour(1), length = length(profile)),
    )
    PSY.add_time_series!(
        sys,
        reserve,
        PSY.SingleTimeSeries("deployed_fraction", TimeArray(stamps, profile)),
    )
    PSY.transform_single_time_series!(sys, _LHS_HORIZON, _LHS_HORIZON)
    return sys, reserve
end

function _lhs_storage_template()
    template = PowerOperationsProblemTemplate(
        NetworkModel(CopperPlateNetworkModel; use_slacks = true),
    )
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(
        template,
        DeviceModel(EnergyReservoirStorage, StorageDispatchWithReserves),
    )
    set_service_model!(
        template,
        ServiceModel(OnlineReserve{ReserveUp}, RangeReserve; use_slacks = true),
    )
    set_service_model!(
        template,
        ServiceModel(OnlineReserve{ReserveDown}, RangeReserve; use_slacks = true),
    )
    return template
end

function _run_lhs_simulation(models; steps = 2)
    sequence = SimulationSequence(;
        models = models,
        ini_cond_chronology = InterProblemChronology(),
    )
    sim = Simulation(;
        name = "lhs",
        steps = steps,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return sim
end

"The deployed fraction the model last solved with, per time step, read from its coefficients."
function _deployed_fraction_in_model(model, reserve)
    container = IOM.get_optimization_container(model)
    V = PSY.EnergyReservoirStorage
    U = POM.AncillaryServiceVariableDischarge
    T = POM.StorageReserveBalanceExpression{
        PSY.ReserveUp,
        POM.DeployedReserve,
        POM.DischargeSide,
    }
    device = PSY.get_component(V, PSI.get_system(model), "Bat")
    base =
        POM.get_variable_multiplier(U, T, device, POM.StorageDispatchWithReserves, reserve)
    expression = IOM.get_expression(container, T, V)
    awards = IOM.get_variable(container, U, V, POM._service_container_meta(reserve))
    return [
        JuMP.coefficient(expression["Bat", t], awards["Bat", t]) / base for
        t in IOM.get_time_steps(container)
    ]
end

for rebuild in (false, true)
    @testset "Deployed fraction refreshes each step (rebuild_model = $rebuild)" begin
        profile = collect(range(0.2, 0.9; length = 48))
        sys, reserve = _lhs_storage_system(profile)
        models = SimulationModels([
            DecisionModel(
                _lhs_storage_template(),
                sys;
                name = "ED",
                optimizer = HiGHS_optimizer,
                rebuild_model = rebuild,
            ),
        ])
        sim = _run_lhs_simulation(models)
        model = PSI.get_simulation_model(sim, :ED)
        @test IOM.get_rebuild_model(IOM.get_settings(model))
        @test _deployed_fraction_in_model(model, reserve) ≈ _LHS_FRACTION .* profile[5:8]

        outputs = get_decision_problem_results(SimulationResults(sim), "ED")
        fractions = read_parameter(
            outputs,
            POM.DeployedFractionParameter,
            OnlineReserve{ReserveUp},
        )
        step_2 = fractions[DateTime("2024-01-01T04:00:00")]
        @test step_2[!, :value] ≈ _LHS_FRACTION .* profile[5:8]
    end
end

@testset "A deployed fraction that is zero at build becomes nonzero" begin
    profile = vcat(zeros(4), fill(0.6, 44))
    sys, reserve = _lhs_storage_system(profile)
    models = SimulationModels([
        DecisionModel(
            _lhs_storage_template(),
            sys;
            name = "ED",
            optimizer = HiGHS_optimizer,
        ),
    ])
    sim = _run_lhs_simulation(models)
    @test _deployed_fraction_in_model(PSI.get_simulation_model(sim, :ED), reserve) ≈
          fill(_LHS_FRACTION * 0.6, 4)
end

@testset "EmulationModel refreshes the deployed fraction at t = 1" begin
    profile = collect(range(0.2, 0.9; length = 48))
    sys_ed, _ = _lhs_storage_system(profile)
    sys_em, reserve_em = _lhs_storage_system(profile)
    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                _lhs_storage_template(),
                sys_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
            ),
        ],
        emulation_model = EmulationModel(
            _lhs_storage_template(),
            sys_em;
            name = "EM",
            optimizer = HiGHS_optimizer,
        ),
    )
    # The emulator advances one hour per execution: four executions per 4-hour ED step.
    sim = _run_lhs_simulation(models; steps = 1)
    emulator = PSI.get_simulation_model(sim, :EM)
    @test IOM.get_rebuild_model(IOM.get_settings(emulator))
    @test only(_deployed_fraction_in_model(emulator, reserve_em)) ≈
          _LHS_FRACTION * profile[4]
end

function _lhs_emulation_models(profile; release_model = false)
    sys_ed, _ = _lhs_storage_system(profile)
    sys_em, _ = _lhs_storage_system(profile)
    return SimulationModels(;
        decision_models = [
            DecisionModel(
                _lhs_storage_template(),
                sys_ed;
                name = "ED",
                optimizer = HiGHS_optimizer,
                release_model = release_model,
            ),
        ],
        emulation_model = EmulationModel(
            _lhs_storage_template(),
            sys_em;
            name = "EM",
            optimizer = HiGHS_optimizer,
            release_model = release_model,
        ),
    )
end

# The battery's award of a reserve splits between its charge and discharge sides; the split
# is degenerate, so separate builds may solve to different splits of the same total. The
# total per reserve is compared instead.
function _realized_emulator_values(sim)
    results = get_emulation_problem_results(SimulationResults(sim))
    values = Dict{String, Vector{Float64}}()
    for key in list_variable_names(results)
        df = sort(read_realized_variable(results, key), [:DateTime, :name])
        total = replace(
            key,
            "AncillaryServiceVariableCharge" => "AncillaryServiceVariable",
            "AncillaryServiceVariableDischarge" => "AncillaryServiceVariable",
        )
        values[total] = get(values, total, zeros(nrow(df))) .+ df.value
    end
    return values
end

@testset "A released emulator keeps its results" begin
    profile = collect(range(0.2, 0.9; length = 48))
    # The emulator rebuilds and runs four times per step.
    kept = _run_lhs_simulation(_lhs_emulation_models(profile); steps = 2)
    released =
        _run_lhs_simulation(_lhs_emulation_models(profile; release_model = true); steps = 2)
    released_values = _realized_emulator_values(released)
    kept_values = _realized_emulator_values(kept)
    @test keys(released_values) == keys(kept_values)
    for (key, values) in kept_values
        @test isapprox(released_values[key], values; atol = 1e-6)
    end
    for name in (:ED, :EM)
        @test IOM.is_released(PSI.get_simulation_model(released, name))
        @test !IOM.is_released(PSI.get_simulation_model(kept, name))
    end
end
