# Line outage events under DCPNetworkModel with StaticBranchBounds. The emulator dispatches
# with ThermalDispatchNoMin and no feedforward, so each emulator step is an independent
# economic dispatch: its optimum depends only on that hour's data and network. That makes
# two independent references: during outage hours the emulator must match a simulation of
# the same system with the line deleted, and otherwise one with the line intact.

const LINE_OUTAGE_NAME = "1"  # nodeA -> nodeB; not a bridge, and the dispatch uses it
# nodeB -> nodeC and nodeC -> nodeD: together these are nodeC's only connections, so
# losing both islands nodeC (Solitude, PVBus5, and load Bus3) from the rest of the system.
const ISLANDING_LINE_NAMES = ["4", "5"]
# The helper below splits LINE_OUTAGE_NAME into two identical halves; this is the new one.
const PARALLEL_LINE_NAME = LINE_OUTAGE_NAME * "_copy"
# nodeA -> nodeE and nodeC -> nodeD: losing both leaves the network connected and
# feasible, and each line, alone or together, changes the dispatch.
const NON_ISLANDING_LINE_NAMES = ["3", "5"]

# `line_names` are the lines in play; `outages` pairs a set of those lines with the profile
# of one FixedForcedOutage they all share; `removed` lines are deleted from the system.
function line_outage_system(;
    line_names = [LINE_OUTAGE_NAME],
    outages = Pair{Vector{String}, <:TimeArray}[],
    removed = String[],
    parallel_pair = false,
)
    sys = build_system(PSITestSystems, "c_sys5_events")
    if parallel_pair
        line = get_component(Line, sys, LINE_OUTAGE_NAME)
        # Set before splitting so the copy inherits the open window too.
        set_angle_limits!(line, (min = -π, max = π))
        add_equivalent_ac_transmission_with_parallel_circuits!(sys, line, Line)
    end
    # Event lines may not carry an angle-difference constraint; open the window in every
    # variant so the intact reference sees the same network.
    for line_name in line_names
        set_angle_limits!(get_component(Line, sys, line_name), (min = -π, max = π))
    end
    for (outage_lines, profile) in outages
        outage = PSY.FixedForcedOutage(; outage_status = 0.0)
        for line_name in outage_lines
            add_supplemental_attribute!(sys, get_component(Line, sys, line_name), outage)
        end
        PSY.add_time_series!(sys, outage, PSY.SingleTimeSeries("outage_profile", profile))
    end
    for line_name in removed
        remove_component!(sys, get_component(Line, sys, line_name))
    end
    return sys
end

# PowerNetworkMatrices names a parallel group after its arc's bus numbers.
function parallel_group_name()
    sys = line_outage_system(; parallel_pair = true)
    arc = get_arc(get_component(Line, sys, LINE_OUTAGE_NAME))
    return "$(get_number(get_from(arc)))_$(get_number(get_to(arc)))_double_circuit"
end

function build_line_outage_simulation(;
    line_names = [LINE_OUTAGE_NAME],
    outages = Pair{Vector{String}, <:TimeArray}[],
    removed = String[],
    parallel_pair = false,
)
    sys_d1 = line_outage_system(; line_names, outages, removed, parallel_pair)
    transform_single_time_series!(sys_d1, Day(2), Day(1))
    sys_em = line_outage_system(; line_names, outages, removed, parallel_pair)

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
    events = if isempty(outages)
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
    return sim
end

function run_line_outage_simulation(; kwargs...)
    sim = build_line_outage_simulation(; kwargs...)
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
    em = run_line_outage_simulation(;
        outages = [[LINE_OUTAGE_NAME] => TimeArray(dates, outage_data)],
    )
    em_removed = run_line_outage_simulation(; removed = [LINE_OUTAGE_NAME])
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

@testset "Line outages that island a bus match removing the lines" begin
    dates = collect(DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"))
    outage_data = zeros(length(dates))
    outage_data[5:8] .= 1.0
    outage_data[15] = 1.0
    profile = TimeArray(dates, outage_data)
    line_names = ISLANDING_LINE_NAMES
    # One outage per line, sharing a profile.
    outages = [[name] => profile for name in line_names]
    em = run_line_outage_simulation(; line_names, outages)
    em_removed = run_line_outage_simulation(; line_names, removed = line_names)
    em_intact = run_line_outage_simulation(; line_names)

    status = read_em(em, "AvailableStatusParameter__Line")
    n_steps = nrow(status)
    for name in line_names
        @test status[!, name] == 1.0 .- outage_data[1:n_steps]
    end
    out = status[!, first(line_names)] .== 0.0
    @test any(out) && !all(out)

    flow = read_em(em, "FlowActivePowerVariable__Line")
    for name in line_names
        @test isapprox(flow[out, name], zeros(count(out)); atol = 1e-6)
    end

    # While islanded, nodeC must serve its own load with its own generation.
    thermal = read_em(em, "ActivePowerVariable__ThermalStandard")
    thermal_removed = read_em(em_removed, "ActivePowerVariable__ThermalStandard")
    @test isapprox(thermal[out, "Solitude"], thermal_removed[out, "Solitude"]; rtol = 1e-6)

    objectives = em_objectives(em)
    @test isapprox(objectives[out], em_objectives(em_removed)[out]; rtol = 1e-6)
    @test isapprox(objectives[.!out], em_objectives(em_intact)[.!out]; rtol = 1e-6)
    @test !isapprox(objectives[out], em_objectives(em_intact)[out]; rtol = 1e-4)
end

@testset "Outage of one line in a parallel pair matches removing that line" begin
    dates = collect(DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"))
    outage_data = zeros(length(dates))
    outage_data[4:6] .= 1.0
    outage_data[18] = 1.0
    profile = TimeArray(dates, outage_data)
    em = run_line_outage_simulation(;
        parallel_pair = true,
        outages = [[LINE_OUTAGE_NAME] => profile],
    )
    em_removed = run_line_outage_simulation(;
        parallel_pair = true,
        removed = [LINE_OUTAGE_NAME],
    )
    em_intact = run_line_outage_simulation(; parallel_pair = true)

    status = read_em(em, "AvailableStatusParameter__Line")
    n_steps = nrow(status)
    @test status[!, LINE_OUTAGE_NAME] == 1.0 .- outage_data[1:n_steps]
    out = status[!, LINE_OUTAGE_NAME] .== 0.0
    @test any(out) && !all(out)

    # The pair is reported as one corridor flow, not per circuit.
    flow = read_em(em, "FlowActivePowerVariable__Line")
    @test LINE_OUTAGE_NAME ∉ names(flow) && PARALLEL_LINE_NAME ∉ names(flow)
    corridor = flow[!, parallel_group_name()]
    # The surviving circuit keeps the corridor connected.
    @test all(abs.(corridor[out]) .> 1e-6)

    # Losing one of two identical circuits halves the corridor's susceptance.
    va = read_em(em, "VoltageAngle__ACBus")
    dtheta = va[!, "nodeA"] .- va[!, "nodeB"]
    moving = abs.(dtheta) .> 1e-4
    @test all(moving[out])
    ratios_out = corridor[out] ./ dtheta[out]
    ratios_in = corridor[.!out .& moving] ./ dtheta[.!out .& moving]
    @test all(isapprox.(ratios_in, first(ratios_in); rtol = 1e-6))
    @test all(isapprox.(ratios_out, first(ratios_in) / 2; rtol = 1e-6))

    objectives = em_objectives(em)
    @test isapprox(objectives[out], em_objectives(em_removed)[out]; rtol = 1e-6)
    @test isapprox(objectives[.!out], em_objectives(em_intact)[.!out]; rtol = 1e-6)
    @test !isapprox(objectives[out], em_objectives(em_intact)[out]; rtol = 1e-4)
end

@testset "One outage on several lines matches removing all of them" begin
    dates = collect(DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"))
    outage_data = zeros(length(dates))
    outage_data[6:9] .= 1.0
    outage_data[17] = 1.0
    line_names = NON_ISLANDING_LINE_NAMES
    em = run_line_outage_simulation(;
        line_names,
        outages = [line_names => TimeArray(dates, outage_data)],
    )
    em_removed = run_line_outage_simulation(; line_names, removed = line_names)
    em_intact = run_line_outage_simulation(; line_names)

    status = read_em(em, "AvailableStatusParameter__Line")
    n_steps = nrow(status)
    for name in line_names
        @test status[!, name] == 1.0 .- outage_data[1:n_steps]
    end
    out = status[!, first(line_names)] .== 0.0
    @test any(out) && !all(out)

    flow = read_em(em, "FlowActivePowerVariable__Line")
    for name in line_names
        @test isapprox(flow[out, name], zeros(count(out)); atol = 1e-6)
    end

    objectives = em_objectives(em)
    @test isapprox(objectives[out], em_objectives(em_removed)[out]; rtol = 1e-6)
    @test isapprox(objectives[.!out], em_objectives(em_intact)[.!out]; rtol = 1e-6)
    @test !isapprox(objectives[out], em_objectives(em_intact)[out]; rtol = 1e-4)
end

@testset "Overlapping outages on different lines each start and end on their own" begin
    dates = collect(DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"))
    line_1, line_2 = NON_ISLANDING_LINE_NAMES
    # Outage 1 starts, then outage 2, then outage 1 ends, then outage 2 ends.
    outage_1 = zeros(length(dates))
    outage_1[4:11] .= 1.0
    outage_2 = zeros(length(dates))
    outage_2[8:15] .= 1.0
    line_names = NON_ISLANDING_LINE_NAMES
    em = run_line_outage_simulation(;
        line_names,
        outages = [
            [line_1] => TimeArray(dates, outage_1),
            [line_2] => TimeArray(dates, outage_2),
        ],
    )
    em_only_1 = run_line_outage_simulation(; line_names, removed = [line_1])
    em_only_2 = run_line_outage_simulation(; line_names, removed = [line_2])
    em_both = run_line_outage_simulation(; line_names, removed = line_names)
    em_intact = run_line_outage_simulation(; line_names)

    status = read_em(em, "AvailableStatusParameter__Line")
    n_steps = nrow(status)
    @test status[!, line_1] == 1.0 .- outage_1[1:n_steps]
    @test status[!, line_2] == 1.0 .- outage_2[1:n_steps]
    out_1 = status[!, line_1] .== 0.0
    out_2 = status[!, line_2] .== 0.0
    only_1 = out_1 .& .!out_2
    only_2 = out_2 .& .!out_1
    both = out_1 .& out_2
    neither = .!out_1 .& .!out_2
    @test any(only_1) && any(only_2) && any(both) && any(neither)

    flow = read_em(em, "FlowActivePowerVariable__Line")
    @test isapprox(flow[out_1, line_1], zeros(count(out_1)); atol = 1e-6)
    @test isapprox(flow[out_2, line_2], zeros(count(out_2)); atol = 1e-6)

    objectives = em_objectives(em)
    for (mask, reference) in (
        only_1 => em_only_1,
        only_2 => em_only_2,
        both => em_both,
        neither => em_intact,
    )
        @test isapprox(objectives[mask], em_objectives(reference)[mask]; rtol = 1e-6)
    end
    # Guard against a fixture where the overlap is indistinguishable from either single
    # outage, which would let a stuck or early-cleared status pass.
    @test !isapprox(objectives[both], em_objectives(em_only_1)[both]; rtol = 1e-4)
    @test !isapprox(objectives[both], em_objectives(em_only_2)[both]; rtol = 1e-4)
    @test !isapprox(objectives[only_1], em_objectives(em_intact)[only_1]; rtol = 1e-4)
    @test !isapprox(objectives[only_2], em_objectives(em_intact)[only_2]; rtol = 1e-4)
end

@testset "Line outages that island an unserved load make the emulator infeasible" begin
    # nodeA -> nodeB and nodeB -> nodeC are nodeB's only connections; nodeB carries load
    # Bus2 and no generation, so with no slacks the island cannot balance.
    line_names = ["1", "4"]
    dates = collect(DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00"))
    # Out for the whole horizon so the last emulator solve is islanded regardless of
    # when within a step the event is applied.
    profile = TimeArray(dates, ones(length(dates)))
    sim = build_line_outage_simulation(;
        line_names,
        outages = [[name] => profile for name in line_names],
    )
    @test execute!(sim; in_memory = true) == IOM.RunStatus.FAILED
    em_model = PSI.get_emulation_model(PSI.get_models(sim))
    @test JuMP.termination_status(IOM.get_jump_model(em_model)) == JuMP.INFEASIBLE
end
