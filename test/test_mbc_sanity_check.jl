# Test setup
test_path = mktempdir()

"""
Create cost curves with varying slope breakpoints, slope magnitudes, and cost at minimum generation
for InterruptibleLoad market bid cost testing. Each test scenario should produce measurably 
different objective values when the load curtailment levels change.
"""
function build_test_systems_with_different_curves()
    systems = Dict{String, PSY.System}()

    # Base system
    base_sys = PSB.build_system(PSITestSystems, "c_sys5_il")
    load_selector = make_selector(PSY.InterruptiblePowerLoad, "IloadBus4")

    # Test Case 1: Baseline - moderate slopes, moderate cost at min
    sys1 = deepcopy(base_sys)
    x_coords = [0.0, 25.0, 50.0, 75.0, 100.0]  # MW breakpoints
    slopes = [40.0, 25.0, 15.0, 10.0]  # $/MWh slopes (decreasing for decremental)

    initial_input = 8.0  # $/h cost at minimum generation
    curve1 = PiecewiseIncrementalCurve(0.0, initial_input, x_coords, slopes)
    add_mbc_inner!(sys1, load_selector; decr_curve = curve1)
    systems["baseline"] = sys1

    # Test Case 2: Steeper slopes - should make curtailment more expensive
    sys2 = deepcopy(base_sys)
    steep_slopes = [80.0, 50.0, 30.0, 20.0]  # Steeper decreasing slopes
    curve2 = PiecewiseIncrementalCurve(0.0, initial_input, x_coords, steep_slopes)
    add_mbc_inner!(sys2, load_selector; decr_curve = curve2)
    systems["steep_slopes"] = sys2

    # Test Case 3: Different breakpoints - earlier steep decrease
    sys3 = deepcopy(base_sys)
    early_steep_x = [0.0, 10.0, 30.0, 60.0, 100.0]  # Earlier transition to low cost
    early_steep_slopes = [80.0, 60.0, 40.0, 5.0]  # High initial, then very low
    curve3 =
        PiecewiseIncrementalCurve(0.0, initial_input, early_steep_x, early_steep_slopes)
    add_mbc_inner!(sys3, load_selector; decr_curve = curve3)
    systems["early_steep"] = sys3

    # Test Case 4: Higher cost at minimum generation
    sys4 = deepcopy(base_sys)
    high_min_cost = 25.0  # Much higher fixed cost
    curve4 = PiecewiseIncrementalCurve(0.0, high_min_cost, x_coords, slopes)  # Use the correct decreasing slopes
    add_mbc_inner!(sys4, load_selector; decr_curve = curve4)
    systems["high_min_cost"] = sys4

    # Test Case 5: Very flat curve - cheap curtailment
    sys5 = deepcopy(base_sys)
    flat_slopes = [5.0, 4.0, 3.0, 2.0]  # Very shallow decreasing slopes
    low_min_cost = 2.0  # Low fixed cost
    curve5 = PiecewiseIncrementalCurve(0.0, low_min_cost, x_coords, flat_slopes)
    add_mbc_inner!(sys5, load_selector; decr_curve = curve5)
    systems["cheap_curtailment"] = sys5

    return systems
end

function run_test_simulations(systems)
    results = Dict{String, Any}()

    for (name, sys) in systems

        # Build model
        # TODO move run_generic_mbc_sim from test_market_bid_cost.jl into test_utils
        # to prevent against include order "undefined function" errors.
        _, res = run_generic_mbc_sim(sys)

        # Extract key metrics
        obj_value = read_optimizer_stats(res)[1, "objective_value"]

        il = first(get_components(InterruptiblePowerLoad, sys))
        il_ts = get_time_series(il, first(get_time_series_keys(il)))
        @assert !isnothing(il) "InterruptibleLoad component not found in system"
        @assert length(PSY.get_components(InterruptiblePowerLoad, sys)) == 1 "Expected exactly one InterruptibleLoad component"

        load_power = read_variable(res, ActivePowerVariable, InterruptiblePowerLoad)
        load_curtailments = Dict{DateTime, TimeArray}()
        total_curtailment = 0.0

        for window in iterate_windows(il_ts)
            initial_time = first(TimeSeries.timestamp(window))
            load_curtailments[initial_time] = TimeArray(
                window .- load_power[initial_time][!, :value];
                colnames = [:value],
            )
            total_curtailment += sum(TimeSeries.values(load_curtailments[initial_time]))
        end

        # Get load cost expression if available
        load_cost = read_expression(res, "ProductionCostExpression__InterruptiblePowerLoad")
        load_cost_out = Dict(
            k => TimeArray(v[!, [:DateTime, :value]]; timestamp = :DateTime) for
            (k, v) in load_cost
        )

        total_load_cost = sum(sum(v[!, :value]) for v in values(load_cost))

        results[name] = Dict(
            "objective" => obj_value,
            "curtailment" => load_curtailments,
            "total_curtailment" => total_curtailment,
            "load_cost" => load_cost_out,
            "total_load_cost" => total_load_cost,
        )
    end

    return results
end

function analyze_results(results)
    baseline_obj = results["baseline"]["objective"]
    baseline_curtail = results["baseline"]["total_curtailment"]

    for (name, data) in results
        name == "baseline" && continue

        obj_diff = data["objective"] - baseline_obj
        curtail_diff = data["total_curtailment"] - baseline_curtail
        obj_pct = (obj_diff / baseline_obj) * 100

        # we're minimizing the objective function
        if name == "steep_slopes"
            @test obj_diff < 0 # steeper demand curve => more $ of benefit per MWh => lower objective
            @test curtail_diff < 0 # Steeper slopes should reduce curtailment
        elseif name == "high_min_cost"
            @test obj_diff < 0 # more benefit per MWh => lower objective
        elseif name == "cheap_curtailment"
            @test obj_diff > 0 # less $ benefit per MWh => higher objective
            @test curtail_diff > 0 # Cheaper curtailment should increase curtailment amount
        elseif name == "early_steep"
            # The early steep curve should affect behavior differently depending on curtailment levels
        end
    end
end

@testset "MBC Sanity Check" begin
    systems = build_test_systems_with_different_curves()
    results = run_test_simulations(systems)
    analyze_results(results)
end
