# An in-place cost update must leave the model as a fresh build at the same time would:
# same objective coefficients and same cost expressions.

const _COST_EXPRESSIONS = (
    ProductionCostExpression,
    FuelCostExpression,
    StartUpCostExpression,
    ShutDownCostExpression,
    FixedCostExpression,
)

_named_terms(expr::JuMP.GenericQuadExpr) = _named_terms(expr.aff)
function _named_terms(expr::JuMP.GenericAffExpr)
    terms = Dict{String, Float64}(
        JuMP.name(v) => c for (c, v) in JuMP.linear_terms(expr) if !iszero(c))
    if !iszero(JuMP.constant(expr))
        terms["(constant)"] = JuMP.constant(expr)
    end
    return terms
end

# Entries of `a` and `b` that differ, as readable strings.
function _term_mismatches(a::Dict, b::Dict, label)
    out = String[]
    for k in union(keys(a), keys(b))
        va, vb = get(a, k, 0.0), get(b, k, 0.0)
        isapprox(va, vb; rtol = 1e-8, atol = 1e-8) ||
            push!(out, "$(label) $(k): updated $(va), fresh build $(vb)")
    end
    return out
end

function _cost_mismatches(updated::DecisionModel, fresh::DecisionModel, ::Type{T}) where {T}
    out = _term_mismatches(
        _named_terms(JuMP.objective_function(PSI.get_jump_model(updated))),
        _named_terms(JuMP.objective_function(PSI.get_jump_model(fresh))),
        "objective",
    )
    c_up = PSI.get_optimization_container(updated)
    c_fr = PSI.get_optimization_container(fresh)
    for E in _COST_EXPRESSIONS
        PSI.has_container_key(c_fr, E, T) || continue
        e_up = PSI.get_expression(c_up, E, T)
        e_fr = PSI.get_expression(c_fr, E, T)
        for name in axes(e_fr, 1), t in axes(e_fr, 2)
            append!(
                out,
                _term_mismatches(
                    _named_terms(e_up[name, t]), _named_terms(e_fr[name, t]),
                    "$(E)[$(name), $(t)]"),
            )
        end
    end
    return out
end

"""
Solve `make_model(sys; initial_time)` for two steps in place, then build it fresh at the
second window and return the cost mismatches between the two.
"""
function update_vs_fresh_build(make_model, sys, types::Type...)
    initial_times = collect(PSY.get_forecast_initial_times(sys))
    updated = make_model(sys; initial_time = first(initial_times))
    models = SimulationModels(; decision_models = [updated])
    sequence = SimulationSequence(;
        models = models, ini_cond_chronology = InterProblemChronology())
    sim = Simulation(; name = "cost_update", steps = 2, models = models,
        sequence = sequence, initial_time = first(initial_times),
        simulation_folder = mktempdir())
    @test build!(sim; console_level = Logging.Error) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim; in_memory = true) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    fresh = make_model(sys; initial_time = initial_times[2])
    @test build!(fresh; output_dir = mktempdir(), console_level = Logging.Error) ==
          PSI.ModelBuildStatus.BUILT
    return reduce(vcat, [_cost_mismatches(updated, fresh, T) for T in types])
end

function _mbc_model(sys; initial_time, multistart = false)
    template = PSI.get_template(build_generic_mbc_model(sys; multistart = multistart))
    return DecisionModel(template, sys; name = "UC", store_variable_names = true,
        optimizer = HiGHS_optimizer_small_gap, initial_time = initial_time)
end

_uc_model(sys; initial_time) = DecisionModel(
    test_template_unit_commitment(CopperPlateNetworkModel), sys; name = "UC",
    store_variable_names = true, optimizer = HiGHS_optimizer_small_gap,
    initial_time = initial_time)

function _ordc_template()
    template = POM.PowerOperationsProblemTemplate(
        NetworkModel(CopperPlateNetworkModel; use_slacks = true))
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_service_model!(template, ServiceModel(OnlineReserve{ReserveDown}, RangeReserve))
    set_service_model!(
        template, ServiceModel(OnlineReserve{ReserveUp}, StepwiseCostReserve))
    set_service_model!(template, ServiceModel(OfflineReserve, StepwiseCostReserve))
    return template
end

@testset "Cost update matches a fresh build" begin
    @testset "Time-series offer at 5-minute resolution" begin
        sys = load_and_fix_system(PSITestSystems, "c_sys5_ed")
        unit = get_component(ThermalStandard, sys, "Park City")
        add_mbc!(sys, make_selector(ThermalStandard, "Park City"))
        set_operation_cost!(
            unit,
            to_market_bid_ts_cost(sys, unit, get_operation_cost(unit)),
        )
        @test isempty(update_vs_fresh_build(_mbc_model, sys, ThermalStandard))
    end

    @testset "Time-series startup and shutdown" begin
        sys = load_and_fix_system(PSITestSystems, "c_fixed_market_bid_cost")
        tweak_for_startup_shutdown!(sys)
        add_startup_shutdown_ts_a!(sys, true)
        @test isempty(update_vs_fresh_build(_mbc_model, sys, ThermalStandard))
    end

    @testset "Must-run unit with time-series startup and shutdown" begin
        sys = load_and_fix_system(PSITestSystems, "c_fixed_market_bid_cost")
        tweak_for_startup_shutdown!(sys)
        add_startup_shutdown_ts_a!(sys, true)
        PSY.set_commitment_mode!(
            get_component(ThermalStandard, sys, "Test Unit1"), PSY.CommitmentModes.MUST_RUN,
        )
        @test isempty(update_vs_fresh_build(_mbc_model, sys, ThermalStandard))
    end

    @testset "Time-series startup, multi-start" begin
        sys = create_multistart_sys(true, (4:16...,))
        mismatches = update_vs_fresh_build(
            (s; initial_time) -> _mbc_model(s; initial_time, multistart = true),
            sys, ThermalMultiStart)
        @test isempty(mismatches)
    end

    @testset "Offer slopes, breakpoints and cost at min that vary" begin
        @test isempty(
            update_vs_fresh_build(
                _mbc_model, build_sys_incr(true, true, true), ThermalStandard),
        )
        @test isempty(
            update_vs_fresh_build(
                _mbc_model, build_sys_decr2(true, true, true), InterruptiblePowerLoad),
        )
    end

    @testset "Time-series reserve demand curves" begin
        sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
        static_ordc = first(get_components(PSY.has_demand_curve, PSY.OnlineReserve, sys))
        _add_ts_ordc!(sys, "ORDC_TS1", static_ordc;
            incrs_x = (0.03, 0.13, 0.07), incrs_y = (0.03, 0.13, 0.07))
        _add_ts_ordc!(sys, "NSPIN_TS", static_ordc; reserve_type = OfflineReserve,
            incrs_x = (0.03, 0.13, 0.07), incrs_y = (0.02, 0.14, 0.08))
        reserve_model(s; initial_time) = DecisionModel(
            _ordc_template(), s; name = "UC", store_variable_names = true,
            optimizer = HiGHS_optimizer, initial_time = initial_time)
        @test isempty(
            update_vs_fresh_build(
                reserve_model, sys, OnlineReserve{ReserveUp}, OfflineReserve),
        )
    end

    @testset "Time-series fuel cost with a no-load term" begin
        sys, _, _, _ = fuel_cost_time_series_system()
        @test isempty(update_vs_fresh_build(_uc_model, sys, ThermalStandard))
    end
end
