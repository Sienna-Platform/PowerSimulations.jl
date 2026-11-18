# Line outage events in a simulation with StaticBranchBounds. The emulator dispatches with
# ThermalDispatchNoMin and no feedforward, so each emulator step is an independent economic
# dispatch whose optimum depends only on that hour's data and network. At every step the
# emulator must therefore match a simulation of the same system with the lines out at that
# step deleted.

const LINE_OUTAGE_NETWORKS = (DCPNetworkModel, PTDFNetworkModel)
const OUTAGE_DATES = DateTime("2024-01-01T00:00:00"):Hour(1):DateTime("2024-01-02T23:00:00")

# Event lines may not carry an angle-difference constraint; open the window on every line in
# every variant so the outage and removal references see the same network.
function c_sys5_events()
    sys = PSB.build_system(PSITestSystems, "c_sys5_events")
    for line in get_components(Line, sys)
        set_angle_limits!(line, (min = -π, max = π))
    end
    return sys
end

# c_sys5_events with line `parallel_to` split into two parallel circuits.
function c_sys5_events_with_parallel_circuit(parallel_to)
    sys = c_sys5_events()
    add_equivalent_ac_transmission_with_parallel_circuits!(
        sys,
        get_component(Line, sys, parallel_to),
        Line,
    )
    return sys
end

# 1.0 (out of service) at the steps in `window`, 0.0 elsewhere.
function outage_status(window = eachindex(OUTAGE_DATES))
    status = zeros(length(OUTAGE_DATES))
    status[window] .= 1.0
    return status
end

lines_out_at(outages, t) =
    Set{String}(name for (lines, status) in outages if isone(status[t]) for name in lines)

# Each `(lines, status)` in `outages` is one FixedForcedOutage shared by `lines`.
function attach_line_outages!(sys, outages)
    for (lines, status) in outages
        outage = PSY.FixedForcedOutage(; outage_status = 0.0)
        for name in lines
            add_supplemental_attribute!(sys, get_component(Line, sys, name), outage)
        end
        PSY.add_time_series!(
            sys,
            outage,
            PSY.SingleTimeSeries("outage_profile", TimeArray(OUTAGE_DATES, status)),
        )
    end
    return sys
end

function build_line_outage_simulation(
    network_model,
    build_sys;
    outages = Tuple{Vector{String}, Vector{Float64}}[],
    removed = String[],
)
    sys_d1, sys_em = map(1:2) do _
        sys = attach_line_outages!(build_sys(), outages)
        for name in removed
            remove_component!(sys, get_component(Line, sys, name))
        end
        sys
    end
    transform_single_time_series!(sys_d1, Day(2), Day(1))

    template_d1 = get_template_basic_uc_simulation()
    set_network_model!(template_d1, NetworkModel(network_model))
    set_device_model!(template_d1, Line, StaticBranchBounds)
    template_em = get_template_nomin_ed_simulation(network_model)
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

function run_line_outage_simulation(network_model, build_sys; kwargs...)
    sim = build_line_outage_simulation(network_model, build_sys; kwargs...)
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
                read_realized_expression(em, name; table_format = TableFormat.WIDE)[
                    !,
                    Not(:DateTime),
                ],
            )
        ) for name in names
    )
end

# Returns the emulator results and the removal reference for each set of lines out, which
# always includes the intact network.
function test_outages_match_removal(network_model, build_sys, outages)
    em = run_line_outage_simulation(network_model, build_sys; outages)
    status = read_em(em, "AvailableStatusParameter__Line")
    steps = 1:nrow(status)
    for (lines, profile) in outages, name in lines
        @test status[!, name] == 1.0 .- profile[steps]
    end

    out_sets = [lines_out_at(outages, t) for t in steps]
    references = Dict(
        out => run_line_outage_simulation(network_model, build_sys; removed = collect(out))
        for out in union(unique(out_sets), [Set{String}()])
    )
    objectives = em_objectives(em)
    reference_objectives = Dict(out => em_objectives(ref) for (out, ref) in references)
    for (out, reference) in reference_objectives
        mask = out_sets .== Ref(out)
        any(mask) || continue
        @test isapprox(objectives[mask], reference[mask]; rtol = 1e-6)
        # Guard against a fixture where two network states are indistinguishable, which
        # would let a stuck or early-cleared status pass.
        for (other, other_reference) in reference_objectives
            other == out && continue
            @test !isapprox(objectives[mask], other_reference[mask]; rtol = 1e-4)
        end
    end

    for (out, reference) in references
        mask = findall(out_sets .== Ref(out))
        isempty(mask) || test_matches_reference(em, reference, mask, out)
    end
    return em, references
end

function test_matches_reference(em, reference, steps, out)
    p = read_em(em, "ActivePowerVariable__ThermalStandard")
    p_ref = read_em(reference, "ActivePowerVariable__ThermalStandard")
    for name in names(p, Not(:DateTime))
        @test isapprox(p[steps, name], p_ref[steps, name]; atol = 1e-6)
    end
    flow = read_em(em, "FlowActivePowerVariable__Line")
    flow_ref = read_em(reference, "FlowActivePowerVariable__Line")
    for name in names(flow, Not(:DateTime))
        if name in out
            @test isapprox(flow[steps, name], zeros(length(steps)); atol = 1e-6)
        elseif name in names(flow_ref)
            @test isapprox(flow[steps, name], flow_ref[steps, name]; atol = 1e-6)
        end
    end
    return
end

@testset "Line outage that never occurs" begin
    for network_model in LINE_OUTAGE_NETWORKS
        status = outage_status(Int[])
        test_outages_match_removal(network_model, c_sys5_events, [(["1"], status)])
    end
end

@testset "Line outage over the whole horizon" begin
    for network_model in LINE_OUTAGE_NETWORKS
        # TODO(acostarelli): emulator can only apply outages to next step
        status = outage_status(2:length(OUTAGE_DATES))
        test_outages_match_removal(network_model, c_sys5_events, [(["1"], status)])
    end
end

@testset "Line outages starting and ending mid-horizon" begin
    for network_model in LINE_OUTAGE_NETWORKS
        status = outage_status([3; 10:12; 20])
        test_outages_match_removal(network_model, c_sys5_events, [(["1"], status)])
    end
end

@testset "Outage of one circuit of a parallel pair" begin
    for network_model in LINE_OUTAGE_NETWORKS
        # Line "1" (A-B) is split into "1" and "1_copy"; the outage takes out "1_copy"
        # only, leaving "1" in service.
        build_sys = () -> c_sys5_events_with_parallel_circuit("1")
        status = outage_status([4:6; 18])
        em, _ = test_outages_match_removal(network_model, build_sys, [(["1_copy"], status)])

        # The pair is reported as one corridor flow, named by PowerNetworkMatrices after its
        # arc's bus numbers, and the surviving circuit keeps it carrying power.
        arc = get_arc(get_component(Line, build_sys(), "1"))
        corridor_name = "$(get_number(get_from(arc)))_$(get_number(get_to(arc)))_double_circuit"
        flow = read_em(em, "FlowActivePowerVariable__Line")
        @test "1" ∉ names(flow) && "1_copy" ∉ names(flow)
        out = isone.(status[1:nrow(flow)])
        @test all(abs.(flow[out, corridor_name]) .> 1e-6)
    end
end

@testset "One outage taking out several lines" begin
    for network_model in LINE_OUTAGE_NETWORKS
        # A single outage attribute on lines 3 (A-E) and 5 (C-D); the rest stays connected.
        status = outage_status([6:9; 17])
        test_outages_match_removal(network_model, c_sys5_events, [(["3", "5"], status)])
    end
end

@testset "Staggered outages of separate lines" begin
    for network_model in LINE_OUTAGE_NETWORKS
        # Independent outage attributes on lines 3 (A-E) and 5 (C-D): outage 3 starts, then
        # 5, then 3 ends, then 5 ends, so the network passes through {}, {3}, {3, 5}, {5}.
        test_outages_match_removal(
            network_model,
            c_sys5_events,
            [(["3"], outage_status(4:11)), (["5"], outage_status(8:15))],
        )
    end
end

@testset "Line outages islanding a bus with generation and load" begin
    for network_model in LINE_OUTAGE_NETWORKS
        # Opening lines 4 (B-C) and 5 (C-D) leaves nodeC on its own with Solitude, PVBus5
        # and Bus3, which must serve their load themselves.
        status = outage_status([5:8; 15])
        test_outages_match_removal(network_model, c_sys5_events, [(["4", "5"], status)])
    end
end

@testset "Line outages islanding a bus with load only are infeasible" begin
    for network_model in LINE_OUTAGE_NETWORKS
        # Opening lines 1 (A-B) and 4 (B-C) leaves nodeB with Bus2's load and no generation.
        # Out for the whole horizon so the last emulator solve is islanded regardless of
        # when within a step the event is applied.
        sim = build_line_outage_simulation(
            network_model,
            c_sys5_events;
            outages = [(["1", "4"], outage_status())],
        )
        @test execute!(sim; in_memory = true) == IOM.RunStatus.FAILED
        em_model = PSI.get_emulation_model(PSI.get_models(sim))
        @test JuMP.termination_status(IOM.get_jump_model(em_model)) == JuMP.INFEASIBLE
    end
end
