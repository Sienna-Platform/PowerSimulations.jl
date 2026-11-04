# Line outage events under DCPNetworkModel with StaticBranchBounds. The emulator dispatches
# with ThermalDispatchNoMin and no feedforward, so each emulator step is an independent
# economic dispatch: its optimum depends only on that hour's data and network. That makes
# two independent references: during outage hours the emulator must match a simulation of
# the same system with the line deleted, and otherwise one with the line intact.

const LINE_OUTAGE_NAME = "1"  # nodeA -> nodeB; not a bridge, and the dispatch uses it

function line_outage_system(; outage_profile = nothing, remove_line = false)
    sys = build_system(PSITestSystems, "c_sys5_events")
    line = get_component(Line, sys, LINE_OUTAGE_NAME)
    # Event lines may not carry an angle-difference constraint; open the window in every
    # variant so the intact reference sees the same network.
    set_angle_limits!(line, (min = -π, max = π))
    if remove_line
        remove_component!(sys, line)
    elseif !isnothing(outage_profile)
        outage = PSY.FixedForcedOutage(; outage_status = 0.0)
        add_supplemental_attribute!(sys, line, outage)
        PSY.add_time_series!(
            sys,
            outage,
            PSY.SingleTimeSeries("outage_profile", outage_profile),
        )
    end
    return sys
end

function run_line_outage_simulation(; outage_profile = nothing, remove_line = false)
    sys_d1 = line_outage_system(; outage_profile, remove_line)
    transform_single_time_series!(sys_d1, Day(2), Day(1))
    sys_em = line_outage_system(; outage_profile, remove_line)

    template_d1 = get_template_basic_uc_simulation()
    set_network_model!(template_d1, NetworkModel(DCPNetworkModel))
    set_device_model!(template_d1, Line, StaticBranchBounds)
    template_em = get_template_nomin_ed_simulation(DCPNetworkModel)
    set_device_model!(template_em, Line, StaticBranchBounds)

    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_d1,
                sys_d1;
                name = "D1",
                initialize_model = false,
                optimizer = HiGHS_optimizer,
            ),
        ],
        emulation_model = EmulationModel(
            template_em,
            sys_em;
            name = "EM",
            optimizer = HiGHS_optimizer,
        ),
    )
    events = if isnothing(outage_profile)
        POM.EventModel[]
    else
        [
            EventModel(
                FixedForcedOutage,
                POM.ContinuousCondition();
                timeseries_mapping = Dict(:outage_status => "outage_profile"),
            ),
        ]
    end
    sequence = SimulationSequence(;
        models = models,
        ini_cond_chronology = InterProblemChronology(),
        events = events,
    )
    sim = Simulation(;
        name = "line_events",
        steps = 1,
        models = models,
        sequence = sequence,
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim; console_level = Logging.Error) == SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == IOM.RunStatus.SUCCESSFULLY_FINALIZED
    return get_emulation_problem_results(SimulationResults(sim; ignore_status = true))
end

read_em(em, name) = read_realized_variable(em, name; table_format = TableFormat.WIDE)
# Emulator optimizer stats are not stored, so sum the realized production cost of every
# device type: with no slacks this is the emulator's objective at each step.
function em_objectives(em)
    names = filter(startswith("ProductionCostExpression__"), list_expression_names(em))
    @test "ProductionCostExpression__ThermalStandard" in names
    return sum(
        sum.(
            eachrow(
                read_realized_expression(
                    em,
                    name;
                    table_format = TableFormat.WIDE,
                )[
                    !,
                    Not(:DateTime),
                ],
            )
        ) for name in names
    )
end

@testset "Line outage in a simulation matches removing the line" begin
    dates = collect(DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"))
    outage_data = zeros(length(dates))
    outage_data[3] = 1.0
    outage_data[10:12] .= 1.0
    outage_data[20] = 1.0
    em = run_line_outage_simulation(; outage_profile = TimeArray(dates, outage_data))
    em_removed = run_line_outage_simulation(; remove_line = true)
    em_intact = run_line_outage_simulation()

    status = read_em(em, "AvailableStatusParameter__Line")[!, LINE_OUTAGE_NAME]
    n_steps = length(status)
    # A profile value of 1 means the line is out at that step (see the thermal
    # FixedForcedOutage testset in test_events.jl).
    @test status == 1.0 .- outage_data[1:n_steps]
    out = status .== 0.0
    @test any(out) && !all(out)

    flow = read_em(em, "FlowActivePowerVariable__Line")[!, LINE_OUTAGE_NAME]
    va = read_em(em, "VoltageAngle__ACBus")
    dtheta = va[!, "nodeA"] .- va[!, "nodeB"]
    @test isapprox(flow[out], zeros(count(out)); atol = 1e-6)
    # The open line does not tie its terminal angles together...
    @test all(abs.(dtheta[out]) .> 1e-4)
    # ...and once restored it follows Ohm's law with one fixed susceptance again.
    in_service = .!out .& (abs.(dtheta) .> 1e-4)
    ratios = flow[in_service] ./ dtheta[in_service]
    @test all(isapprox.(ratios, first(ratios); rtol = 1e-6))

    objectives = em_objectives(em)
    @test isapprox(objectives[out], em_objectives(em_removed)[out]; rtol = 1e-6)
    @test isapprox(objectives[.!out], em_objectives(em_intact)[.!out]; rtol = 1e-6)
    # Guard against a fixture where losing the line changes nothing.
    @test !isapprox(objectives[out], em_objectives(em_intact)[out]; rtol = 1e-4)
end
