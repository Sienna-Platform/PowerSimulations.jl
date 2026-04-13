precompile_time = @timed using PowerSimulations

using PowerSimulations
import PowerSimulations as PSI
using PowerSystems
import PowerSystems as PSY
using Logging
using PowerSystemCaseBuilder
using PowerNetworkMatrices
using HydroPowerSimulations
using HiGHS
using Dates
using PowerFlows

@info pkgdir(PowerSimulations)

function is_running_on_ci()
    return get(ENV, "CI", "false") == "true" || haskey(ENV, "GITHUB_ACTIONS")
end

open("precompile_time.txt", "a") do io
    if length(ARGS) == 0 && !is_running_on_ci()
        push!(ARGS, "Local Test")
    end
    write(io, "| $(ARGS[1]) | $(precompile_time.time) |\n")
end

function set_device_models!(template::ProblemTemplate, uc::Bool = true)
    if uc
        # unique to UC
        set_device_model!(template, ThermalMultiStart, ThermalStandardUnitCommitment)
        set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
        set_device_model!(template, HydroDispatch, FixedOutput)
    else
        # unique to ED
        set_device_model!(template, ThermalMultiStart, ThermalBasicDispatch)
        set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
        set_device_model!(template, HydroDispatch, HydroDispatchRunOfRiver)
    end

    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, DeviceModel(Line, StaticBranch))
    set_device_model!(template, Transformer2W, StaticBranchUnbounded)
    set_device_model!(template, TapTransformer, StaticBranchUnbounded)
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveUp}, RangeReserve),
    )
    set_service_model!(
        template,
        ServiceModel(VariableReserve{ReserveDown}, RangeReserve),
    )
    return template
end

try
    sys_rts_da = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys_rts_rt = build_system(PSISystems, "modified_RTS_GMLC_RT_sys")
    sys_rts_realization = build_system(PSISystems, "modified_RTS_GMLC_realization_sys")

    for sys in [sys_rts_da, sys_rts_rt, sys_rts_realization]
        g = get_component(ThermalStandard, sys, "121_NUCLEAR_1")
        set_must_run!(g, true)
    end

    for i in 1:2
        template_uc = ProblemTemplate(
            NetworkModel(
                PTDFPowerModel;
                use_slacks = true,
                PTDF_matrix = PTDF(sys_rts_da),
                duals = [CopperPlateBalanceConstraint],
                power_flow_evaluation = DCPowerFlow(),
            ),
        )
        set_device_models!(template_uc)

        template_ed = ProblemTemplate(
            NetworkModel(
                PTDFPowerModel;
                use_slacks = true,
                PTDF_matrix = PTDF(sys_rts_da),
                duals = [CopperPlateBalanceConstraint],
                power_flow_evaluation = DCPowerFlow(),
            ),
        )
        set_device_models!(template_ed, false)

        template_em = ProblemTemplate(
            NetworkModel(
                PTDFPowerModel;
                use_slacks = true,
                PTDF_matrix = PTDF(sys_rts_da),
                duals = [CopperPlateBalanceConstraint],
            ),
        )
        set_device_models!(template_em, false)
        empty!(template_em.services)

        models = SimulationModels(;
            decision_models = [
                DecisionModel(
                    template_uc,
                    sys_rts_da;
                    name = "UC",
                    optimizer = optimizer_with_attributes(HiGHS.Optimizer,
                        "mip_rel_gap" => 0.01),
                    initialize_model = true,
                    optimizer_solve_log_print = false,
                    direct_mode_optimizer = true,
                    check_numerical_bounds = false,
                ),
                DecisionModel(
                    template_ed,
                    sys_rts_rt;
                    name = "ED",
                    optimizer = optimizer_with_attributes(HiGHS.Optimizer,
                        "mip_rel_gap" => 0.01),
                    initialize_model = true,
                    check_numerical_bounds = false,
                    #export_pwl_vars = true,
                ),
            ],
            emulation_model = EmulationModel(
                template_em,
                sys_rts_realization;
                name = "PF",
                optimizer = optimizer_with_attributes(HiGHS.Optimizer),
            ),
        )

        sequence = SimulationSequence(;
            models = models,
            feedforwards = Dict(
                "ED" => [
                    SemiContinuousFeedforward(;
                        component_type = ThermalStandard,
                        source = OnVariable,
                        affected_values = [ActivePowerVariable],
                    ),
                ],
                "PF" => [
                    SemiContinuousFeedforward(;
                        component_type = ThermalStandard,
                        source = OnVariable,
                        affected_values = [ActivePowerVariable],
                    ),
                ],
            ),
            ini_cond_chronology = InterProblemChronology(),
        )

        sim = Simulation(;
            name = "compact_sim",
            steps = 3,
            models = models,
            sequence = sequence,
            initial_time = DateTime("2020-01-01T00:00:00"),
            simulation_folder = mktempdir(; cleanup = true),
        )

        build_out, time_build, _, _ =
            @timed build!(sim; console_level = Logging.Error)

        if build_out == PSI.SimulationBuildStatus.BUILT
            name = i > 1 ? "Postcompile" : "Precompile"
            open("build_time.txt", "a") do io
                write(io, "| $(ARGS[1])-Build Time $name | $(time_build) |\n")
            end
        else
            open("build_time.txt", "a") do io
                write(io, "| $(ARGS[1])- Build Time $name | FAILED TO TEST |\n")
            end
        end

        solve_out, time_solve, _, _ = @timed execute!(sim; enable_progress_bar = false)

        if solve_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED
            name = i > 1 ? "Postcompile" : "Precompile"
            open("solve_time.txt", "a") do io
                write(io, "| $(ARGS[1])-Solve Time $name | $(time_solve) |\n")
            end
        else
            open("solve_time.txt", "a") do io
                write(io, "| $(ARGS[1])- Solve Time $name | FAILED TO TEST |\n")
            end
        end
    end
catch e
    rethrow(e)
    open("build_time.txt", "a") do io
        write(io, "| $(ARGS[1])- Build Time | FAILED TO TEST |\n")
    end
end

if !is_running_on_ci()
    for file in ["precompile_time.txt", "build_time.txt", "solve_time.txt"]
        name = replace(file, "_" => " ")[begin:(end - 4)]
        println("$name:")
        for line in eachline(open(file))
            println("\t", line)
        end
    end
end
