#! /usr/bin/env julia
# Verifies that PowerFlows AC solvers with the reactive-power controls (voltage-controlling
# taps, switched shunts, FACTS) are usable THROUGH PowerSimulations on the RTS system, both
# single- and multi-period. Drives real DecisionModel build!/solve! calls (PF-in-the-loop),
# then inspects the resulting PowerFlowData for convergence, voltage-bound violations, and
# regulated-bus setpoint tracking.
#
# Usage:
#   julia --project=test scripts/verify_pf_reactive_control_psi.jl [subset]
#
# `subset` selects a slice of the matrix:
#   smoke    - NewtonRaphsonACPowerFlow, discrete control, single-period only (fast).
#   discrete - the full discrete-control block (NR/TR, single- and multi-period).
#   qlimit   - the full Q-limit block (all AC solvers, single- and multi-period).
#   full     - discrete + qlimit (default when no ARGS given).

import PowerSystems as PSY
import PowerSimulations as PSI
import PowerSystemCaseBuilder as PSB
import PowerNetworkMatrices as PNM
using PowerFlows
const PFS = PowerFlows
using HydroPowerSimulations
using StorageSystemsSimulations
import JuMP
import HiGHS
import Dates

const HiGHS_optimizer = JuMP.optimizer_with_attributes(
    HiGHS.Optimizer,
    "time_limit" => 100.0,
    "random_seed" => 12345,
    "log_to_console" => false,
)

const DISCRETE_SOLVERS = (PFS.NewtonRaphsonACPowerFlow, PFS.TrustRegionACPowerFlow)
const QLIMIT_SOLVERS = (
    PFS.NewtonRaphsonACPowerFlow,
    PFS.TrustRegionACPowerFlow,
    PFS.LevenbergMarquardtACPowerFlow,
    PFS.RobustHomotopyPowerFlow,
    PFS.FastDecoupledACPowerFlow,
)
const HORIZONS = (1, 3)
const V_MIN = 0.9
const V_MAX = 1.1

# --- 1. Build + make PF-friendly (mirrors PowerFlows' create_pf_friendly_rts_gmlc) ---

function build_pf_friendly_rts()
    sys = PSB.build_system(PSB.PSISystems, "RTS_GMLC_DA_sys")
    PSY.remove_component!(sys, only(PSY.get_components(PSY.TwoTerminalHVDC, sys)))
    for (component_type, component_name, new_limits) in [
        (PSY.RenewableDispatch, "113_PV_1", (min = -30.0, max = 30.0)),
        (PSY.ThermalStandard, "115_STEAM_3", (min = -50.0, max = 100.0)),
        (PSY.ThermalStandard, "207_CT_1", (min = -70.0, max = 70.0)),
        (PSY.RenewableDispatch, "215_PV_1", (min = -40.0, max = 40.0)),
        (PSY.ThermalStandard, "307_CT_1", (min = -70.0, max = 70.0)),
        (PSY.ThermalStandard, "315_CT_8", (min = 0.0, max = 80.0)),
    ]
        PSY.set_reactive_power_limits!(
            PSY.get_component(component_type, sys, component_name),
            new_limits,
        )
    end
    return sys
end

# --- 2. Augment with controllable devices ---

function is_bridging_grid_tx(t, gen_buses::Set{Int})
    arc = PSY.get_arc(t)
    vf = PSY.get_base_voltage(PSY.get_from(arc))
    vt = PSY.get_base_voltage(PSY.get_to(arc))
    if isapprox(vf, vt)
        return false
    end
    fb = PSY.get_number(PSY.get_from(arc))
    tb = PSY.get_number(PSY.get_to(arc))
    return iszero((fb in gen_buses) + (tb in gen_buses))
end

function mv_terminal_bus(t)
    arc = PSY.get_arc(t)
    vf = PSY.get_base_voltage(PSY.get_from(arc))
    if isapprox(vf, 138.0; atol = 1.0)
        return PSY.get_from(arc)
    end
    return PSY.get_to(arc)
end

"Set first-class control fields on HV<->MV bridging TapTransformers to make them
voltage-controlling (regulating their own 138 kV terminal). Returns the controlled names."
function control_bridging_transformers!(sys::PSY.System)
    gen_buses = Set(
        PSY.get_number(PSY.get_bus(g)) for
        g in PSY.get_available_components(PSY.Generator, sys)
    )
    controlled = String[]
    for tx in PSY.get_available_components(PSY.TapTransformer, sys)
        is_bridging_grid_tx(tx, gen_buses) || continue
        mv_bus = mv_terminal_bus(tx)
        PSY.set_control_objective!(tx, PSY.TransformerControlObjective.VOLTAGE)
        PSY.set_tap_limits!(tx, (min = 0.9, max = 1.1))
        PSY.set_number_of_tap_positions!(tx, 33)
        PSY.set_regulated_bus_number!(tx, PSY.get_number(mv_bus))
        PSY.set_voltage_setpoint!(tx, 1.0)
        push!(controlled, PSY.get_name(tx))
    end
    return controlled
end

function first_138kv_pq_load_bus(sys::PSY.System)
    for load in PSY.get_available_components(PSY.PowerLoad, sys)
        bus = PSY.get_bus(load)
        isapprox(PSY.get_base_voltage(bus), 138.0; atol = 1.0) || continue
        PSY.get_bustype(bus) == PSY.ACBusTypes.PQ || continue
        return bus
    end
    error("No 138 kV PQ load bus found in the RTS system")
end

"Add one SwitchedAdmittance at a 138 kV load bus, regulating that bus continuously.
RTS ships none, so this is a pure addition."
function add_switched_shunt!(sys::PSY.System)
    bus = first_138kv_pq_load_bus(sys)
    shunt = PSY.SwitchedAdmittance(;
        name = "verify_switched_shunt",
        available = true,
        bus = bus,
        Y = 0.0 + 0.0im,
        initial_status = [0],
        number_of_steps = [12],
        Y_increase = [0.0 + 0.1im],
        admittance_limits = (min = 0.995, max = 1.005),
        control_mode = PSY.SwitchedAdmittanceControlMode.CONTINUOUS_VOLTAGE,
    )
    PSY.add_component!(sys, shunt)
    return shunt
end

function first_230kv_non_ref_bus(sys::PSY.System)
    for bus in PSY.get_available_components(PSY.ACBus, sys)
        isapprox(PSY.get_base_voltage(bus), 230.0; atol = 1.0) || continue
        PSY.get_bustype(bus) == PSY.ACBusTypes.REF && continue
        return bus
    end
    error("No non-REF 230 kV bus found in the RTS system")
end

"Add one FACTSControlDevice (SVC) at a 230 kV bus, regulating that bus locally."
function add_facts!(sys::PSY.System)
    bus = first_230kv_non_ref_bus(sys)
    facts = PSY.FACTSControlDevice(;
        name = "verify_facts_svc",
        available = true,
        bus = bus,
        control_mode = PSY.FACTSOperationModes.NML,
        voltage_setpoint = 1.0,
        max_shunt_current = 100.0,
        max_reactive_power = 9999.0,
        shunt_control_type = PSY.FACTSShuntControlType.SVC,
        regulated_bus_number = 0,
    )
    PSY.add_component!(sys, facts)
    return facts
end

struct VerificationSystem
    sys::PSY.System
    controlled_taps::Vector{String}
    shunt::PSY.SwitchedAdmittance
    facts::PSY.FACTSControlDevice
    ptdf::PNM.PTDF
end

function build_verification_system()
    sys = build_pf_friendly_rts()
    controlled_taps = control_bridging_transformers!(sys)
    shunt = add_switched_shunt!(sys)
    facts = add_facts!(sys)
    ptdf = PNM.PTDF(sys)
    return VerificationSystem(sys, controlled_taps, shunt, facts, ptdf)
end

# --- 3. Solver x control matrix, driven through PSI ---

function build_pf_evaluation(
    ::Type{ACSolver},
    control_discrete_devices::Bool,
    check_reactive_power_limits::Bool,
) where {ACSolver <: PFS.ACPowerFlowSolverType}
    return PFS.ACPolarPowerFlow{ACSolver}(;
        control_discrete_devices = control_discrete_devices,
        check_reactive_power_limits = check_reactive_power_limits,
    )
end

"ThermalBasicUnitCommitment, not the `ThermalBasicDispatch` that `template_economic_dispatch`
defaults to: without a commitment binary every RTS thermal unit is pinned at or above its own
P_min, and the P_min sum (3745 MW) exceeds the load in the low hours this script runs (3337 MW
at t=1), so the balance is infeasible from over-generation. Commitment lets units decommit.
It also mirrors the RTS AC-PF-in-the-loop idiom in test_power_flow_in_the_loop.jl.
The Hydro/EnergyReservoirStorage models are here for fleet fidelity — the built-in templates
cover only a subset of RTS's generation — not for feasibility."
function build_template(pf_evaluation, ptdf::PNM.PTDF)
    template = PSI.ProblemTemplate(
        PSI.NetworkModel(
            PSI.PTDFPowerModel;
            PTDF_matrix = ptdf,
            power_flow_evaluation = pf_evaluation,
        ),
    )
    PSI.set_device_model!(template, PSY.ThermalStandard, PSI.ThermalBasicUnitCommitment)
    PSI.set_device_model!(template, PSY.RenewableDispatch, PSI.RenewableFullDispatch)
    PSI.set_device_model!(template, PSY.RenewableNonDispatch, PSI.FixedOutput)
    PSI.set_device_model!(template, PSY.PowerLoad, PSI.StaticPowerLoad)
    PSI.set_device_model!(template, PSY.Line, PSI.StaticBranch)
    PSI.set_device_model!(template, PSY.Transformer2W, PSI.StaticBranch)
    PSI.set_device_model!(template, PSY.TapTransformer, PSI.StaticBranch)
    PSI.set_device_model!(template, PSY.HydroDispatch, HydroDispatchRunOfRiver)
    PSI.set_device_model!(template, PSY.HydroTurbine, HydroTurbineEnergyDispatch)
    PSI.set_device_model!(template, PSY.HydroReservoir, HydroEnergyModelReservoir)
    PSI.set_device_model!(template, PSY.EnergyReservoirStorage, StorageDispatchWithReserves)
    return template
end

# --- 4. Record + report a matrix ---

struct ComboResult
    label::String
    status::Symbol   # :PASS, :FAIL, :SKIPPED
    note::String
    converged::Bool
    max_v_violation::Float64
    max_setpoint_error::Float64
    device_settings_varied::Bool
    aux_vars_ingested::Bool   # PF device results readable as PSI aux variables
end

# Verify the PF solve's device results were ingested into PSI as aux variables: every
# (family, name, time_step, final/delivered) row of `get_controlled_device_results` must be
# readable back, value-identical, from the model's aux-variable containers. FACTS delivered Q
# is MVAr in the results table but stored p.u. in the aux variable (hence the divisor).
function verify_aux_var_ingestion(model::PSI.DecisionModel, sys::PSY.System, data)::Bool
    container = PSI.get_optimization_container(model)
    df = PFS.get_controlled_device_results(data)
    base_power = PSY.get_base_power(sys)
    for (family, comp_type, aux_var, col, divisor) in (
        ("TapTransformer", PSY.TapTransformer, PSI.PowerFlowTapRatio, :final, 1.0),
        (
            "SwitchedAdmittance",
            PSY.SwitchedAdmittance,
            PSI.PowerFlowSwitchedShuntSusceptance,
            :final,
            1.0,
        ),
        (
            "FACTSControlDevice",
            PSY.FACTSControlDevice,
            PSI.PowerFlowFACTSReactivePower,
            :delivered_q_mvar,
            base_power,
        ),
    )
        rows = df[df.family .== family, :]
        isempty(rows) && continue
        aux = PSI.get_aux_variable(container, aux_var(), comp_type)
        for row in eachrow(rows)
            if aux[row.name, row.time_step] != row[col] / divisor
                @error "aux-var ingestion mismatch" family row.name row.time_step
                return false
            end
        end
    end
    return true
end

function regulated_bus_number(t::PSY.TapTransformer)
    reg = PSY.get_regulated_bus_number(t)
    if iszero(reg)
        return PSY.get_number(PSY.get_to(PSY.get_arc(t)))
    end
    return reg
end

function regulated_bus_number(s::PSY.SwitchedAdmittance)
    reg = PSY.get_regulated_bus_number(s)
    if iszero(reg)
        return PSY.get_number(PSY.get_bus(s))
    end
    return reg
end

function regulated_bus_number(f::PSY.FACTSControlDevice)
    reg = PSY.get_regulated_bus_number(f)
    if iszero(reg)
        return PSY.get_number(PSY.get_bus(f))
    end
    return reg
end

device_setpoint(t::PSY.TapTransformer) = PSY.get_voltage_setpoint(t)
device_setpoint(f::PSY.FACTSControlDevice) = PSY.get_voltage_setpoint(f)
function device_setpoint(s::PSY.SwitchedAdmittance)
    lims = PSY.get_admittance_limits(s)
    return (lims.min + lims.max) / 2.0
end

"|V - setpoint| at the device's regulated bus, using the FINAL time step. Returns `missing`
when the regulated bus is not present in `bus_lookup` (e.g. the tap was not enrolled)."
function setpoint_error(d, data, bus_lookup::Dict{Int, Int})
    bus_number = regulated_bus_number(d)
    haskey(bus_lookup, bus_number) || return missing
    bus_ix = bus_lookup[bus_number]
    v = PFS.get_bus_magnitude(data)[bus_ix, end]
    return abs(v - device_setpoint(d))
end

function max_voltage_violation(data)
    v = PFS.get_bus_magnitude(data)
    return maximum(max.(0.0, max.(v .- V_MAX, V_MIN .- v)))
end

"True when any enrolled tap/shunt/FACTS device's solved setting differs across time steps
(only meaningful for multi-period discrete-control runs). Reads the public per-time-step
results frame rather than device struct internals."
function device_settings_varied(data)
    df = PFS.get_controlled_device_results(data)
    finals_by_device = Dict{Tuple{String, String}, Vector{Float64}}()
    families = df[:, "family"]
    names = df[:, "name"]
    finals = df[:, "final"]
    for i in eachindex(families)
        key = (families[i], names[i])
        push!(get!(finals_by_device, key, Float64[]), finals[i])
    end
    for vals in values(finals_by_device)
        length(vals) <= 1 && continue
        lo, hi = extrema(vals)
        if !isapprox(lo, hi; atol = 1e-6)
            return true
        end
    end
    return false
end

function run_combo(
    vsys::VerificationSystem,
    ::Type{ACSolver},
    control_discrete_devices::Bool,
    check_reactive_power_limits::Bool,
    horizon_hours::Int,
    label::String,
) where {ACSolver <: PFS.ACPowerFlowSolverType}
    pf_evaluation =
        build_pf_evaluation(ACSolver, control_discrete_devices, check_reactive_power_limits)
    template = build_template(pf_evaluation, vsys.ptdf)
    model = PSI.DecisionModel(
        template,
        vsys.sys;
        optimizer = HiGHS_optimizer,
        horizon = Dates.Hour(horizon_hours),
        name = label,
    )
    build_status = PSI.build!(model; output_dir = mktempdir(; cleanup = true))
    if build_status != PSI.ModelBuildStatus.BUILT
        return ComboResult(
            label, :FAIL, "build! -> $build_status", false, NaN, NaN, false, false)
    end
    run_status = PSI.solve!(model)
    if run_status != PSI.RunStatus.SUCCESSFULLY_FINALIZED
        return ComboResult(
            label, :FAIL, "solve! -> $run_status", false, NaN, NaN, false, false)
    end

    data = PSI.get_power_flow_data(
        only(PSI.get_power_flow_evaluation_data(PSI.get_optimization_container(model))),
    )
    converged = all(PFS.get_converged(data))
    max_v_violation = max_voltage_violation(data)
    bus_lookup = PFS.get_bus_lookup(data)

    setpoint_errors = Float64[]
    if control_discrete_devices
        for d in (vsys.shunt, vsys.facts)
            e = setpoint_error(d, data, bus_lookup)
            ismissing(e) || push!(setpoint_errors, e)
        end
        for tap in PSY.get_available_components(PSY.TapTransformer, vsys.sys)
            PSY.get_name(tap) in vsys.controlled_taps || continue
            e = setpoint_error(tap, data, bus_lookup)
            ismissing(e) || push!(setpoint_errors, e)
        end
    end
    max_setpoint_error = NaN
    if !isempty(setpoint_errors)
        max_setpoint_error = maximum(setpoint_errors)
    end

    varied = false
    if control_discrete_devices && horizon_hours > 1
        varied = device_settings_varied(data)
    end

    aux_ingested = true
    if control_discrete_devices
        aux_ingested = verify_aux_var_ingestion(model, vsys.sys, data)
    end

    status = :FAIL
    note = ""
    if !converged
        note = "not all time steps converged"
    elseif !aux_ingested
        note = "PF device results not ingested as PSI aux variables"
    else
        status = :PASS
    end
    return ComboResult(
        label,
        status,
        note,
        converged,
        max_v_violation,
        max_setpoint_error,
        varied,
        aux_ingested,
    )
end

# --- 5. Structure for partial runs ---

function discrete_combos(vsys::VerificationSystem; horizons = HORIZONS)
    results = ComboResult[]
    for solver in DISCRETE_SOLVERS
        for h in horizons
            label = "discrete/$(nameof(solver))/h$(h)"
            push!(results, run_combo(vsys, solver, true, false, h, label))
        end
    end
    return results
end

function qlimit_combos(vsys::VerificationSystem; horizons = HORIZONS)
    results = ComboResult[]
    for solver in QLIMIT_SOLVERS
        for h in horizons
            label = "qlimit/$(nameof(solver))/h$(h)"
            push!(results, run_combo(vsys, solver, false, true, h, label))
        end
    end
    return results
end

function smoke_combo(vsys::VerificationSystem)
    return [run_combo(vsys, PFS.NewtonRaphsonACPowerFlow, true, false, 1, "smoke/NR/h1")]
end

function get_subset()
    if isempty(ARGS)
        return :full
    end
    return Symbol(ARGS[1])
end

function print_results(results::Vector{ComboResult})
    println()
    println(rpad("LABEL", 26), rpad("STATUS", 10), rpad("CONVERGED", 11),
        rpad("MAX |V| VIOL", 14), rpad("MAX SETPT ERR", 15), rpad("VARIED", 8),
        rpad("AUX INGESTED", 14), "NOTE")
    println("-"^114)
    for r in results
        println(
            rpad(r.label, 26),
            rpad(String(r.status), 10),
            rpad(string(r.converged), 11),
            rpad(string(round(r.max_v_violation; digits = 5)), 14),
            rpad(string(round(r.max_setpoint_error; digits = 5)), 15),
            rpad(string(r.device_settings_varied), 8),
            rpad(string(r.aux_vars_ingested), 14),
            r.note,
        )
    end
    println()
end

function main()
    subset = get_subset()
    @info "Building RTS verification system (subset=$subset)"
    vsys = build_verification_system()
    @info "Controlled taps: $(vsys.controlled_taps)"
    @info "Switched shunt: $(PSY.get_name(vsys.shunt)) at bus $(PSY.get_number(PSY.get_bus(vsys.shunt)))"
    @info "FACTS SVC: $(PSY.get_name(vsys.facts)) at bus $(PSY.get_number(PSY.get_bus(vsys.facts)))"

    results = ComboResult[]
    if subset == :smoke
        append!(results, smoke_combo(vsys))
    elseif subset == :discrete
        append!(results, discrete_combos(vsys))
    elseif subset == :qlimit
        append!(results, qlimit_combos(vsys))
    elseif subset == :full
        append!(results, discrete_combos(vsys))
        append!(results, qlimit_combos(vsys))
    else
        error("Unknown subset '$subset'; expected smoke, discrete, qlimit, or full.")
    end

    print_results(results)

    n_fail = count(r -> r.status == :FAIL, results)
    if n_fail > 0
        @error "$n_fail combo(s) FAILED"
        exit(1)
    end
    @info "All expected-supported combos PASSED (or were SKIPPED by design)."
    exit(0)
end

main()
