# WARNING: included in HydroPowerSimulations's tests as well.
# If you make changes, run those tests too!
const TIME1 = DateTime("2024-01-01T00:00:00")
test_path = mktempdir()
const FormulationDict =
    Dict{Type{<:PSY.Device}, Union{DeviceModel, Type{<:PSI.AbstractDeviceFormulation}}}
# TODO could replace with PSI's defaults, template_unit_commitment
const DEFAULT_FORMULATIONS =
    FormulationDict(
        ThermalStandard => ThermalBasicUnitCommitment,
        PowerLoad => StaticPowerLoad,
        InterruptiblePowerLoad => PowerLoadInterruption,
        RenewableDispatch => RenewableFullDispatch,
        # I include this file in the tests of SSS and HPS, which error on these formulations.
        # HydroDispatch => HydroCommitmentRunOfRiver,
        # EnergyReservoirStorage => StorageDispatchWithReserves,
    )

# debugging code for inspecting objective functions -- ignore
function format_objective_function_file(filepath::String)
    if !isfile(filepath)
        println("Error: File '$filepath' does not exist.")
        exit(1)
    end

    try
        content = read(filepath, String)
        content = replace(content, "+" => "+\n")
        content = replace(content, "-" => "-\n")
        write(filepath, content)
    catch e
        println("Error processing file '$filepath': $e")
        exit(1)
    end
end

function save_objective_function(model::DecisionModel, filepath::String)
    open(filepath, "w") do file
        println(file, "invariant_terms:")
        println(file, model.internal.container.objective_function.invariant_terms)
        println(file, "variant_terms:")
        println(file, model.internal.container.objective_function.variant_terms)
    end
    format_objective_function_file(filepath)
end

function save_constraints(model::DecisionModel, filepath::String)
    open(filepath, "w") do file
        for (k, v) in model.internal.container.constraints
            println(file, "Constraint Type: $(k)")
            println(file, v)
        end
    end
end
# end debugging code

function set_formulations!(template::ProblemTemplate,
    sys::PSY.System,
    device_to_formulation::FormulationDict,
)
    for (device, formulation) in device_to_formulation
        if !isempty(get_components(device, sys))
            _set_formulations_helper(template, device, formulation)
        end
    end
    for (device, formulation) in DEFAULT_FORMULATIONS
        if !haskey(device_to_formulation, device) && !isempty(get_components(device, sys))
            _set_formulations_helper(template, device, formulation)
        end
    end
end

_set_formulations_helper(
    template::ProblemTemplate,
    device::Type{<:PSY.Device},
    formulation::Type{<:PSI.AbstractDeviceFormulation},
) =
    set_device_model!(template, device, formulation)
_set_formulations_helper(template::ProblemTemplate, _, device_model::DeviceModel) =
    set_device_model!(template, device_model)

# Layer of indirection to upgrade problem results to look like simulation results
_maybe_upgrade_to_dict(input::AbstractDict) = input
_maybe_upgrade_to_dict(input::DataFrame) =
    SortedDict{DateTime, DataFrame}(first(input[!, :DateTime]) => input)

read_variable_dict(
    res::IS.Results,
    var_name::Type{<:PSI.VariableType},
    comp_type::Type{<:PSY.Component},
) =
    _maybe_upgrade_to_dict(read_variable(res, var_name, comp_type))
read_parameter_dict(
    res::IS.Results,
    par_name::Type{<:PSI.ParameterType},
    comp_type::Type{<:PSY.Component},
) =
    _maybe_upgrade_to_dict(read_parameter(res, par_name, comp_type))

function _read_one_value(res, var_name, gentype, unit_name)
    df = @chain begin
        vcat(values(read_variable_dict(res, var_name, gentype))...)
        @rsubset(:name == unit_name)
        @combine(:value = sum(:value))
    end
    return df[1, 1]
end

function build_generic_mbc_model(sys::System;
    multistart::Bool = false,
    standard::Bool = false,
    device_to_formulation = FormulationDict(),
)
    template = ProblemTemplate(
        NetworkModel(
            CopperPlatePowerModel;
            duals = [CopperPlateBalanceConstraint],
        ),
    )

    set_formulations!(
        template,
        sys,
        device_to_formulation,
    )
    if standard
        set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
    end
    if multistart
        set_device_model!(template, ThermalMultiStart, ThermalMultiStartUnitCommitment)
    end

    model = DecisionModel(
        template,
        sys;
        name = "UC",
        store_variable_names = true,
        optimizer = HiGHS_optimizer_small_gap,
    )
    return model
end

function run_generic_mbc_prob(
    sys::System;
    multistart::Bool = false,
    standard = false,
    test_success = true,
    filename::Union{String, Nothing} = nothing,
    is_decremental::Bool = false,
    device_to_formulation = FormulationDict(),
)
    model = build_generic_mbc_model(
        sys;
        multistart = multistart,
        standard = standard,
        device_to_formulation = device_to_formulation,
    )
    test_path = mktempdir()
    build_result = build!(model; output_dir = test_path)
    test_success && @test build_result == PSI.ModelBuildStatus.BUILT
    solve_result = solve!(model)
    test_success && @test solve_result == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    res = OptimizationProblemResults(model)
    if !isnothing(filename)
        adj = is_decremental ? "decr" : "incr"
        save_objective_function(
            model,
            "$(filename)_$(adj)_prob_objective_function.txt",
        )
        save_constraints(
            model,
            "$(filename)_$(adj)_prob_constraints.txt",
        )
    end
    return model, res
end

function run_generic_mbc_sim(
    sys::System;
    multistart::Bool = false,
    in_memory_store::Bool = false,
    standard::Bool = false,
    test_success = true,
    filename::Union{String, Nothing} = nothing,
    is_decremental::Bool = false,
    device_to_formulation = FormulationDict(),
)
    model = build_generic_mbc_model(
        sys;
        multistart = multistart,
        standard = standard,
        device_to_formulation = device_to_formulation,
    )
    models = SimulationModels(;
        decision_models = [
            model,
        ],
    )
    sequence = SimulationSequence(;
        models = models,
        feedforwards = Dict(
        ),
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = "compact_sim",
        steps = 2,
        models = models,
        sequence = sequence,
        initial_time = TIME1,
        simulation_folder = mktempdir(),
    )

    test_success && @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    test_success &&
        @test execute!(sim; in_memory = in_memory_store) ==
              PSI.RunStatus.SUCCESSFULLY_FINALIZED

    sim_res = SimulationResults(sim)
    res = get_decision_problem_results(sim_res, "UC")
    if !isnothing(filename)
        adj = is_decremental ? "decr" : "incr"
        save_objective_function(
            model,
            "$(filename)_$(adj)_sim_objective_function.txt",
        )
        save_constraints(
            model,
            "$(filename)_$(adj)_sim_constraints.txt",
        )
    end
    return model, res
end

"""
Run a simple simulation with the system and return information useful for testing
time-varying startup and shutdown functionality.  Pass `simulation = false` to use a single
decision model, `true` for a full simulation.
"""
function run_mbc_sim(
    sys::System,
    comp_name::String,
    ::Type{T};
    has_initial_input::Bool = true,
    is_decremental::Bool = false,
    simulation = true,
    in_memory_store = false,
    standard = false,
    filename::Union{String, Nothing} = nothing,
    device_to_formulation = FormulationDict(),
) where {T <: PSY.Component}
    model, res = if simulation
        run_generic_mbc_sim(
            sys;
            in_memory_store = in_memory_store,
            standard = standard,
            filename = filename,
            is_decremental = is_decremental,
            device_to_formulation = device_to_formulation,
        )
    else
        run_generic_mbc_prob(
            sys;
            standard = standard,
            filename = filename,
            is_decremental = is_decremental,
            device_to_formulation = device_to_formulation,
        )
    end

    # TODO make this more general as to which variables we're reading.
    # e.g. hydro.

    # Test that breakpoint and slope parameters read from results match the
    # ground truth from the system's offer curve time series.
    # The PowerLoadDispatch device formulation doesn't have
    # DecrementalCostAtMinParameter nor OnVariable.

    if is_decremental
        bp_param_type = PSI.DecrementalPiecewiseLinearBreakpointParameter
        sl_param_type = PSI.DecrementalPiecewiseLinearSlopeParameter
        oc_getter = get_decremental_offer_curves
    else
        bp_param_type = PSI.IncrementalPiecewiseLinearBreakpointParameter
        sl_param_type = PSI.IncrementalPiecewiseLinearSlopeParameter
        oc_getter = get_incremental_offer_curves
    end
    # Both bp_param and sl_param are SortedDict{DateTime, DataFrame}.
    # Columns: :DateTime, :name ("Test Unit1"), :name2 ("tranche_1"),
    # and :value (breakpoints for bp_param, slopes for sl_param).
    bp_param = _maybe_upgrade_to_dict(read_parameter(res, bp_param_type, T))
    sl_param = _maybe_upgrade_to_dict(read_parameter(res, sl_param_type, T))

    # We can compare the raw PiecewiseStepData values directly against read_parameter
    # results because: (1) time-variant offer curve time series are always in natural units
    # (see PSY's cost_function_timeseries.jl), and (2) the parameter multiplier is 1.0 for
    # both slopes and breakpoints (see default_interface_methods.jl). Unit conversion via
    # get_piecewise_curve_per_system_unit only happens later when building expressions.
    for (step_dt, bp_step_df) in pairs(bp_param)
        sl_step_df = sl_param[step_dt]
        for gen_name in unique(bp_step_df.name)
            comp = get_component(T, sys, gen_name)
            cost = PSY.get_operation_cost(comp)
            oc_ts = oc_getter(comp, cost; start_time = step_dt)
            gen_bp = @rsubset(bp_step_df, :name == gen_name)
            gen_sl = @rsubset(sl_step_df, :name == gen_name)
            for (ts, psd) in zip(TimeSeries.timestamp(oc_ts), TimeSeries.values(oc_ts))
                expected_bp = get_x_coords(psd)
                expected_sl = get_y_coords(psd)
                actual_bp = sort(@rsubset(gen_bp, :DateTime == ts), :name2).value
                actual_sl = sort(@rsubset(gen_sl, :DateTime == ts), :name2).value
                # actual may be longer than expected due to padding (see _unwrap_for_param)
                @test length(actual_bp) >= length(expected_bp)
                @test length(actual_sl) >= length(expected_sl)
                @test all(isapprox.(actual_bp[1:length(expected_bp)], expected_bp))
                @test all(isapprox.(actual_sl[1:length(expected_sl)], expected_sl))
            end
        end
    end

    if has_initial_input
        if is_decremental
            param_type = PSI.DecrementalCostAtMinParameter
            initial_getter = get_decremental_initial_input
        else  # Default to incremental for ThermalStandard and other types
            param_type = PSI.IncrementalCostAtMinParameter
            initial_getter = get_incremental_initial_input
        end
        init_param = read_parameter_dict(res, param_type, T)
        for (step_dt, step_df) in pairs(init_param)
            for gen_name in unique(step_df.name)
                comp = get_component(T, sys, gen_name)
                ii_comp = initial_getter(
                    comp,
                    PSY.get_operation_cost(comp);
                    start_time = step_dt,
                )
                @test all(step_df[!, :DateTime] .== TimeSeries.timestamp(ii_comp))
                @test all(
                    isapprox.(
                        @rsubset(step_df, :name == gen_name).value,
                        TimeSeries.values(ii_comp),
                    ),
                )
            end
        end
    end
    # NOTE this could be rewritten nicely using PowerAnalytics
    # Select component based on comp_type - fallback to legacy behavior if needed
    sel = make_selector(T, comp_name)
    @assert !isnothing(first(get_components(sel, sys)))
    if has_initial_input
        decisions = (
            _read_one_value(res, PSI.OnVariable, T, comp_name),
            _read_one_value(res, PSI.ActivePowerVariable, T, comp_name),
        )
    else
        decisions = (
            1.0, # placeholder so return type is consistent.
            _read_one_value(res, PSI.ActivePowerVariable, T, comp_name),
        )
    end
    return model, res, decisions, ()
end

function cost_due_to_time_varying_mbc(
    sys::System,
    res::IS.Results,
    ::Type{T};
    is_decremental = false,
    has_initial_input = true,
    device_to_formulation::Any, #unused
) where {T <: PSY.Device}
    power_vars = read_variable_dict(res, PSI.ActivePowerVariable, T)
    result = SortedDict{DateTime, DataFrame}()
    if has_initial_input
        on_vars = read_variable_dict(res, PSI.OnVariable, T)
        @assert all(keys(on_vars) .== keys(power_vars))
        @assert !isempty(keys(on_vars))
    end
    for step_dt in keys(power_vars)
        power_df = power_vars[step_dt]
        step_df = DataFrame(:DateTime => unique(power_df.DateTime))
        gen_names = unique(power_df.name)
        @assert !isempty(gen_names)
        @assert any([
            get_operation_cost(comp) isa MarketBidCost for
            comp in get_components(T, sys)
        ])
        if has_initial_input
            on_df = on_vars[step_dt]
            @assert names(on_df) == names(power_df)
            @assert on_df[!, :DateTime] == power_df[!, :DateTime]
        else
            # assumption: all devices are on.
            on_df = DataFrame(
                :DateTime => power_df.DateTime,
                :name => power_df.name,
                :value => ones(nrow(power_df)),
            )
        end
        for gen_name in gen_names
            comp = get_component(T, sys, gen_name)
            cost = PSY.get_operation_cost(comp)
            (cost isa MarketBidCost) || continue
            step_df[!, gen_name] .= 0.0
            ii_getter = if is_decremental
                get_decremental_initial_input
            else
                get_incremental_initial_input
            end
            if PSI.is_time_variant(ii_getter(cost))
                # initial cost: initial input time series multiplied by OnVariable value.
                ii_ts = ii_getter(comp, cost; start_time = step_dt)
                @assert all(unique(on_df.DateTime) .== TimeSeries.timestamp(ii_ts))
                step_df[!, gen_name] .+=
                    @rsubset(on_df, :name == gen_name).value .*
                    TimeSeries.values(ii_ts)
            end
            oc_getter =
                is_decremental ?
                get_decremental_offer_curves :
                get_incremental_offer_curves
            if PSI.is_time_variant(oc_getter(cost))
                vc_ts = oc_getter(comp, cost; start_time = step_dt)
                @assert all(unique(power_df.DateTime) .== TimeSeries.timestamp(vc_ts))
                # variable cost: cost function time series evaluated at ActivePowerVariable value.
                step_df[!, gen_name] .+=
                    _calc_pwi_cost.(
                        @rsubset(power_df, :name == gen_name).value,
                        TimeSeries.values(vc_ts),
                    ) # could replace with direct evaluation, now that it is implemented in IS. (https://github.com/NREL-Sienna/PowerSimulations.jl/issues/1430)
            end
        end
        measure_vars = [x for x in names(step_df) if x != "DateTime"]
        # rows represent: [time, component, time-varying MBC cost for {component} at {time}]
        result[step_dt] =
            DataFrames.stack(
                step_df,
                measure_vars;
                variable_name = :name,
                value_name = :value,
            )
    end
    return result
end

# See run_startup_shutdown_obj_fun_test for explanation
function obj_fun_test_helper(
    ground_truth_1,
    ground_truth_2,
    res1,
    res2;
    is_decremental = false,
)
    @assert all(keys(ground_truth_1) .== keys(ground_truth_2))
    # total cost due to time-varying MBCs in each scenario
    total1 =
        [only(@combine(df, :total = sum(:value)).total) for df in values(ground_truth_1)]
    total2 =
        [only(@combine(df, :total = sum(:value)).total) for df in values(ground_truth_2)]
    if !is_decremental
        ground_truth_diff = total2 .- total1  # How much did the cost increase between simulation 1 and simulation 2 for each step
    else
        # objective = cost - benefit. higher load prices => more willing to pay, more benefit.
        # so we get an extra negative sign, since we're increasing benefit, not cost.
        ground_truth_diff = total1 .- total2
    end

    obj1 = PSI.read_optimizer_stats(res1)[!, "objective_value"]
    obj2 = PSI.read_optimizer_stats(res2)[!, "objective_value"]
    obj_diff = obj2 .- obj1

    # Make sure there is some real difference between the two scenarios
    @assert !any(isapprox.(ground_truth_diff, 0.0; atol = 0.0001))
    # Make sure the difference is reflected correctly in the objective value
    if !all(isapprox.(obj_diff, ground_truth_diff; atol = 0.0001))
        @error obj_diff
        @error ground_truth_diff
    end

    comparison_passes = all(isapprox.(obj_diff, ground_truth_diff; atol = 0.0001))

    # An assumption in this line of testing is that our perturbations are small enough that
    # they don't actually change the decisions, just slightly alter the cost. If the
    # comparison isn't passing, one reason could be that this assumption is violated. In
    # that case, we want an `AssertionError` rather than a test failure.
    if !comparison_passes
        @assert isapprox(obj1, obj2; atol = 10, rtol = 0.01) "obj1 ($obj1) and obj2 ($obj2) are supposed to differ, but they differ by an improbably large amount ($obj_diff) -- the perturbations are likely affecting the decisions. Ground truth difference is $ground_truth_diff; the test would fail, but we have some reason to believe it is instead broken."
    end

    # At this point, we've eliminiated many 'innocent' sources of error, so if the
    # comparison isn't passing here we can have some confidence that it's actually pointing
    # to a bug in the implementation
    @test comparison_passes
    return comparison_passes
end

# See run_startup_shutdown_obj_fun_test for explanation
function run_mbc_obj_fun_test(
    sys1,
    sys2,
    comp_name::String,
    comp_type::Type{T};
    is_decremental::Bool = false,
    has_initial_input::Bool = true,
    simulation = true,
    in_memory_store = false,
    filename::Union{String, Nothing} = nothing,
    device_to_formulation = FormulationDict(),
) where {T <: PSY.Component}
    # at the moment, nullable_decisions are empty tuples, but keep them for future-proofing.
    # look at run_startup_shutdown_test for explanation: non-nullable should be approx_geq_1.
    kwargs = Dict(
        :is_decremental => is_decremental,
        :has_initial_input => has_initial_input,
        :simulation => simulation,
        :in_memory_store => in_memory_store,
        :filename => filename,
        :device_to_formulation => device_to_formulation,
    )
    filename_in = get(kwargs, :filename, nothing)
    if !isnothing(filename_in)
        kwargs[:filename] = filename_in * get_name(sys1)
    end
    _, res1, decisions1, nullable_decisions1 =
        run_mbc_sim(
            sys1,
            comp_name,
            comp_type;
            kwargs...,
        )
    if !isnothing(filename_in)
        kwargs[:filename] = filename_in * get_name(sys2)
    end
    _, res2, decisions2, nullable_decisions2 =
        run_mbc_sim(
            sys2,
            comp_name,
            comp_type;
            kwargs...,
        )
    all_decisions1 = (decisions1..., nullable_decisions1...)
    all_decisions2 = (decisions2..., nullable_decisions2...)
    if !all(isapprox.(all_decisions1, all_decisions2))
        @error all_decisions1
        @error all_decisions2
    end
    @assert all(isapprox.(all_decisions1, all_decisions2))

    ground_truth_1 =
        cost_due_to_time_varying_mbc(sys1, res1, T; is_decremental = is_decremental,
            has_initial_input = has_initial_input,
            device_to_formulation = device_to_formulation)
    ground_truth_2 =
        cost_due_to_time_varying_mbc(sys2, res2, T; is_decremental = is_decremental,
            has_initial_input = has_initial_input,
            device_to_formulation = device_to_formulation)

    success = obj_fun_test_helper(
        ground_truth_1,
        ground_truth_2,
        res1,
        res2;
        is_decremental = is_decremental,
    )
    #=
    if !success
        @error ground_truth_1
        @error ground_truth_2
        obj1 = PSI.read_optimizer_stats(res1)[!, "objective_value"]
        obj2 = PSI.read_optimizer_stats(res2)[!, "objective_value"]
        @error obj1
        @error obj2
    end=#
    return decisions1, decisions2
end

# TODO for https://github.com/NREL-Sienna/PowerSimulations.jl/issues/1430, reimplement this
# by converting the implied IncrementalCurve into an InputOutputCurve and then evaluating
# *its* `FunctionData`
function _calc_pwi_cost(active_power::Float64, pwi::PiecewiseStepData)
    isapprox(active_power, 0.0) && return 0.0
    breakpoints = get_x_coords(pwi)
    slopes = get_y_coords(pwi)
    above_min =
        isapprox(active_power, first(breakpoints)) || active_power > first(breakpoints)
    below_max =
        isapprox(active_power, last(breakpoints)) || active_power < last(breakpoints)
    @assert above_min && below_max "Active power ($active_power) is outside the range of breakpoints ($(first(breakpoints)) to $(last(breakpoints))) for the piecewise step data."
    active_power = clamp(active_power, first(breakpoints), last(breakpoints))
    i_leq = findlast(<=(active_power), breakpoints)
    cost =
        sum(slopes[1:(i_leq - 1)] .* (breakpoints[2:i_leq] .- breakpoints[1:(i_leq - 1)]))
    (active_power > breakpoints[i_leq]) &&
        (cost += slopes[i_leq] * (active_power - breakpoints[i_leq]))
    return cost
end

"Test that the two systems (typically one without time series and one with constant time series) simulate the same"
function test_generic_mbc_equivalence(sys0, sys1; kwargs...)
    for runner in (run_generic_mbc_prob, run_generic_mbc_sim)  # test with both a single problem and a full simulation
        filename_in = get(kwargs, :filename, nothing)
        # Create a mutable copy of kwargs
        kwargs_dict = Dict(kwargs)
        if !isnothing(filename_in)
            kwargs_dict[:filename] = filename_in * get_name(sys0)
        end
        _, res0 = runner(sys0; kwargs_dict...)
        if !isnothing(filename_in)
            kwargs_dict[:filename] = filename_in * get_name(sys1)
        end
        _, res1 = runner(sys1; kwargs_dict...)
        obj_val_0 = PSI.read_optimizer_stats(res0)[!, "objective_value"]
        obj_val_1 = PSI.read_optimizer_stats(res1)[!, "objective_value"]
        @test isapprox(obj_val_0, obj_val_1; atol = 0.0001)
    end
end

approx_geq_1(x; kwargs...) = (x >= 1.0) || isapprox(x, 1.0; kwargs...)
