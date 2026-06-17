function get_outage_total_power_by_step_dict(
    sys::PSY.System,
    variables::Dict{String, DataFrame},
    var_name::String,
    associated_outages::Vector{PSY.UnplannedOutage};
    col_name::String = "name",
)
    required_variables = variables[var_name]
    total_variable_dict = Dict{String, Vector{Float64}}()
    for outage in associated_outages
        outage_name = string(IS.get_uuid(outage))
        outage_power_v = Vector{Float64}()
        devices = PSY.get_associated_components(
            sys,
            outage;
            component_type = PSY.Generator,
        )
        for (i, device) in enumerate(devices)
            device_name = PSY.get_name(device)
            current_v =
                filter(x -> x[col_name] == device_name, required_variables)[!, "value"]
            if i == 1
                outage_power_v = current_v
            else
                outage_power_v .+= current_v
            end
        end
        total_variable_dict[outage_name] = outage_power_v
    end
    return total_variable_dict
end

function get_reserve_total_power_by_step_dict(
    variables::Dict{String, DataFrame},
    var_name::String,
    associated_outages::Vector{PSY.UnplannedOutage},
    contributing_devices::Union{
        IS.FlattenIteratorWrapper{<:PSY.Generator},
        Vector{<:PSY.Generator},
    };
    col_name::String = "name2",
)
    required_variables = variables[var_name]
    total_variable_dict = Dict{String, Vector{Float64}}()
    for outage in associated_outages
        outage_name = string(IS.get_uuid(outage))
        outage_power_v = Vector{Float64}()
        for (i, device) in enumerate(contributing_devices)
            device_name = PSY.get_name(device)
            current_v =
                filter(x -> x[col_name] == device_name, required_variables)[!, "value"]
            if i == 1
                outage_power_v = current_v
            else
                outage_power_v .+= current_v
            end
        end
        total_variable_dict[outage_name] = outage_power_v
    end
    return total_variable_dict
end

function test_reserves_deployment(
    power_outage::Float64,
    reserve_deployment::Float64;
    tol::Float64 = 1e-3,
)
    @test isapprox(power_outage, reserve_deployment, atol = tol)
end

function compare_outage_power_and_deployed_reserves(
    sys::PSY.System,
    res::OptimizationProblemResults,
    service::PSY.VariableReserve;
    tolerance::Float64 = 1e-3,
)
    variablesdict = read_variables(res)
    associated_outages =
        collect(PSY.get_supplemental_attributes(PSY.UnplannedOutage, service))
    # Fall back: in the new G-1 pattern, outages are attached to the outaged
    # generator, not the reserve service. Resolve from system if empty.
    if isempty(associated_outages)
        all_outages = collect(PSY.get_supplemental_attributes(PSY.UnplannedOutage, sys))
        associated_outages = all_outages
    end
    outage_dict = get_outage_total_power_by_step_dict(
        sys,
        variablesdict,
        "ActivePowerVariable__ThermalStandard",
        associated_outages;
        col_name = "name",
    )
    contributing_devices = PSY.get_contributing_devices(sys, service)
    service_name = PSY.get_name(service)
    reserve_dict = get_reserve_total_power_by_step_dict(
        variablesdict,
        "PostContingencyActivePowerReserveDeploymentVariable__VariableReserve__ReserveUp__" *
        service_name,
        associated_outages,
        contributing_devices;
        col_name = "name2",
    )
    for outage in associated_outages
        outage_name = string(IS.get_uuid(outage))
        for i in 1:length(outage_dict[outage_name])
            test_reserves_deployment(
                outage_dict[outage_name][i],
                reserve_dict[outage_name][i],
            )
        end
    end
end

@testset "G-n with Ramp reserve deliverability constraints Dispatch with responding reserves only up, including reduction of parallel circuits" begin
    for add_parallel_line in [true, false]
        c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
        if add_parallel_line
            l4 = get_component(Line, c_sys5, "4")
            add_equivalent_ac_transmission_with_parallel_circuits!(c_sys5, l4, PSY.Line)
        end
        systems = [c_sys5]
        objfuncs = [GAEVF, GQEVF, GQEVF]
        constraint_keys = [
            PSI.ConstraintKey(
                ActivePowerVariableLimitsConstraint,
                PSY.ThermalStandard,
                "lb",
            ),
            PSI.ConstraintKey(
                ActivePowerVariableLimitsConstraint,
                PSY.ThermalStandard,
                "ub",
            ),
            PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
            PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
            PSI.ConstraintKey(
                PostContingencyFlowRateConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1_lb",
            ),
            PSI.ConstraintKey(
                PostContingencyFlowRateConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1_ub",
            ),
            PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
            #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
            PSI.ConstraintKey(
                RequirementConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
            PSI.ConstraintKey(
                RampConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
            PSI.ConstraintKey(
                PostContingencyGenerationBalanceConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
            PSI.ConstraintKey(
                PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
        ]
        PTDF_ref = IdDict{System, PTDF}(
            c_sys5 => PTDF(c_sys5),
        )
        test_results = IdDict{System, Vector{Int}}(
            c_sys5 => [360, 0, 600, 432, 72],
        )
        test_obj_values = IdDict{System, Float64}(
            c_sys5 => 329000.0,
        )
        components_outages_cases = IdDict{System, Vector{String}}(
            c_sys5 => ["Alta"],
        )
        for (ix, sys) in enumerate(systems)
            gen = get_component(ThermalStandard, sys, "Solitude")
            set_ramp_limits!(gen, (up = 0.4, down = 0.4)) #Increase ramp limits to make the problem feasible
            components_outages_names = components_outages_cases[sys]
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
            for component_name in components_outages_names
                # --- Create Outage Data ---
                transition_data = GeometricDistributionForcedOutage(;
                    mean_time_to_recovery = 10,
                    outage_transition_probability = 0.9999,
                    monitored_components = collect(get_components(ACTransmission, sys)),
                )
                # --- Add Outage Supplemental attribute to device and services that should respond ---
                component = get_component(ThermalStandard, sys, component_name)
                add_supplemental_attribute!(sys, component, transition_data)
                add_supplemental_attribute!(sys, reserve_up, transition_data)
            end
            template = get_thermal_dispatch_template_network(
                NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]),
            )
            set_service_model!(template,
                ServiceModel(
                    VariableReserve{ReserveUp},
                    SecurityConstrainedRampReserve,
                    "Reserve1",
                ))

            ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            psi_constraint_test(ps_model, constraint_keys)
            moi_tests(
                ps_model,
                test_results[sys]...,
                false,
            )
            psi_checkobjfun_test(ps_model, objfuncs[ix])
            psi_checksolve_test(
                ps_model,
                [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
                test_obj_values[sys],
                10000,
            )
            res = OptimizationProblemResults(ps_model)
            compare_outage_power_and_deployed_reserves(
                sys,
                res,
                reserve_up)
        end
    end
end

# Exercises the per-service line-scoping path in
# `_monitored_components_by_modeled_type` and the downstream
# `PostContingencyFlowRateConstraint` build for `PTDFPowerModel` when the
# reserve service monitors a strict subset of the system's AC lines instead of
# every line. The constraint key meta does not change (it remains keyed by
# service name) but the per-outage flow constraint container ends up with
# fewer entries, which lowers the MOI counts compared to the all-lines variant.
@testset "G-n with Ramp reserve deliverability constraints PTDFPowerModel with monitored line subset" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    monitored_line_names = ["1", "2"]
    systems = [c_sys5]
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(
            ActivePowerVariableLimitsConstraint,
            PSY.ThermalStandard,
            "lb",
        ),
        PSI.ConstraintKey(
            ActivePowerVariableLimitsConstraint,
            PSY.ThermalStandard,
            "ub",
        ),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_ub",
        ),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
    ]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5 => PTDF(c_sys5),
    )
    # Counts are smaller than the all-lines baseline `[360, 0, 600, 432, 72]`
    # because only the monitored subset contributes
    # `PostContingencyFlowRateConstraint` rows per outage step.
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [360, 0, 504, 336, 72],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 329000.0,
    )
    components_outages_cases = IdDict{System, Vector{String}}(
        c_sys5 => ["Alta"],
    )
    for (ix, sys) in enumerate(systems)
        gen = get_component(ThermalStandard, sys, "Solitude")
        set_ramp_limits!(gen, (up = 0.4, down = 0.4)) #Increase ramp limits to make the problem feasible
        components_outages_names = components_outages_cases[sys]
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
        monitored_subset =
            [get_component(Line, sys, n) for n in monitored_line_names]
        for component_name in components_outages_names
            # --- Create Outage Data with a hand-picked monitored subset ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = monitored_subset,
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end
        template = get_thermal_dispatch_template_network(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve1",
            ))

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
        res = OptimizationProblemResults(ps_model)
        compare_outage_power_and_deployed_reserves(
            sys,
            res,
            reserve_up)
    end
end

@testset "G-n with contingency reserves deliverability constraints including responding reserves only up, reserve requirement, and reduction of parallel circuits" begin
    for add_parallel_line in [true, false]
        c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

        if add_parallel_line
            l4 = get_component(Line, c_sys5, "4")
            add_equivalent_ac_transmission_with_parallel_circuits!(c_sys5, l4, PSY.Line)
        end
        systems = [c_sys5]
        objfuncs = [GAEVF, GQEVF, GQEVF]
        constraint_keys = [
            PSI.ConstraintKey(
                ActivePowerVariableLimitsConstraint,
                PSY.ThermalStandard,
                "lb",
            ),
            PSI.ConstraintKey(
                ActivePowerVariableLimitsConstraint,
                PSY.ThermalStandard,
                "ub",
            ),
            PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
            PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
            PSI.ConstraintKey(
                PostContingencyFlowRateConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1_lb",
            ),
            PSI.ConstraintKey(
                PostContingencyFlowRateConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1_ub",
            ),
            PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
            #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
            PSI.ConstraintKey(
                RequirementConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
            PSI.ConstraintKey(
                RampConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
            PSI.ConstraintKey(
                PostContingencyGenerationBalanceConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
            PSI.ConstraintKey(
                PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
        ]
        PTDF_ref = IdDict{System, PTDF}(
            c_sys5 => PTDF(c_sys5),
        )
        test_results = IdDict{System, Vector{Int}}(
            c_sys5 => [360, 0, 600, 432, 72],
        )
        test_obj_values = IdDict{System, Float64}(
            c_sys5 => 329000.0,
        )
        components_outages_cases = IdDict{System, Vector{String}}(
            c_sys5 => ["Alta"],
        )
        for (ix, sys) in enumerate(systems)
            gen = get_component(ThermalStandard, sys, "Solitude")
            set_ramp_limits!(gen, (up = 0.4, down = 0.4)) #Increase ramp limits to make the problem feasible
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")

            components_outages_names = components_outages_cases[sys]
            for component_name in components_outages_names
                # --- Create Outage Data ---
                transition_data = GeometricDistributionForcedOutage(;
                    mean_time_to_recovery = 10,
                    outage_transition_probability = 0.9999,
                    monitored_components = collect(get_components(ACTransmission, sys)),
                )
                # --- Add Outage Supplemental attribute to device and services that should respond ---
                component = get_component(ThermalStandard, sys, component_name)
                add_supplemental_attribute!(sys, component, transition_data)
                add_supplemental_attribute!(sys, reserve_up, transition_data)
            end
            template = get_thermal_dispatch_template_network(
                NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]),
            )
            set_service_model!(template,
                ServiceModel(
                    VariableReserve{ReserveUp},
                    SecurityConstrainedContingencyReserve,
                    "Reserve1",
                ))

            ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            psi_constraint_test(ps_model, constraint_keys)
            moi_tests(
                ps_model,
                test_results[sys]...,
                false,
            )
            psi_checkobjfun_test(ps_model, objfuncs[ix])
            psi_checksolve_test(
                ps_model,
                [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
                test_obj_values[sys],
                10000,
            )
            res = OptimizationProblemResults(ps_model)
            compare_outage_power_and_deployed_reserves(
                sys,
                res,
                reserve_up)
        end
    end
end

@testset "G-n with contingency reserves deliverability constraints including responding reserves only up, NO reserve requirement, and reduction of parallel circuits" begin
    for add_parallel_line in [true, false]
        c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

        if add_parallel_line
            l4 = get_component(Line, c_sys5, "4")
            add_equivalent_ac_transmission_with_parallel_circuits!(c_sys5, l4, PSY.Line)
        end
        systems = [c_sys5]
        objfuncs = [GAEVF, GQEVF, GQEVF]
        constraint_keys = [
            PSI.ConstraintKey(
                ActivePowerVariableLimitsConstraint,
                PSY.ThermalStandard,
                "lb",
            ),
            PSI.ConstraintKey(
                ActivePowerVariableLimitsConstraint,
                PSY.ThermalStandard,
                "ub",
            ),
            PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
            PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
            PSI.ConstraintKey(
                PostContingencyFlowRateConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1_lb",
            ),
            PSI.ConstraintKey(
                PostContingencyFlowRateConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1_ub",
            ),
            PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
            #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
            PSI.ConstraintKey(
                PostContingencyGenerationBalanceConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1",
            ),
            PSI.ConstraintKey(
                PostContingencyActivePowerGenerationLimitsConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1_lb",
            ),
            PSI.ConstraintKey(
                PostContingencyActivePowerGenerationLimitsConstraint,
                PSY.VariableReserve{ReserveUp},
                "Reserve1_ub",
            ),
        ]
        PTDF_ref = IdDict{System, PTDF}(
            c_sys5 => PTDF(c_sys5),
        )
        test_results = IdDict{System, Vector{Int}}(
            c_sys5 => [240, 0, 504, 504, 96],
        )
        test_obj_values = IdDict{System, Float64}(
            c_sys5 => 329000.0,
        )
        components_outages_cases = IdDict{System, Vector{String}}(
            c_sys5 => ["Alta"],
        )
        for (ix, sys) in enumerate(systems)
            gen = get_component(ThermalStandard, sys, "Solitude")
            set_ramp_limits!(gen, (up = 0.4, down = 0.4)) #Increase ramp limits to make the problem feasible
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
            remove_time_series!(
                sys,
                Deterministic,
                reserve_up,
                "requirement",
            )
            components_outages_names = components_outages_cases[sys]
            for component_name in components_outages_names
                # --- Create Outage Data ---
                transition_data = GeometricDistributionForcedOutage(;
                    mean_time_to_recovery = 10,
                    outage_transition_probability = 0.9999,
                    monitored_components = collect(get_components(ACTransmission, sys)),
                )
                # --- Add Outage Supplemental attribute to device and services that should respond ---
                component = get_component(ThermalStandard, sys, component_name)
                add_supplemental_attribute!(sys, component, transition_data)
                add_supplemental_attribute!(sys, reserve_up, transition_data)
            end
            template = get_thermal_dispatch_template_network(
                NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]),
            )
            set_service_model!(template,
                ServiceModel(
                    VariableReserve{ReserveUp},
                    SecurityConstrainedContingencyReserve,
                    "Reserve1",
                ))

            ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            psi_constraint_test(ps_model, constraint_keys)
            moi_tests(
                ps_model,
                test_results[sys]...,
                false,
            )
            psi_checkobjfun_test(ps_model, objfuncs[ix])
            psi_checksolve_test(
                ps_model,
                [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
                test_obj_values[sys],
                10000,
            )
            res = OptimizationProblemResults(ps_model)
            compare_outage_power_and_deployed_reserves(
                sys,
                res,
                reserve_up)
        end
    end
end

#This test ensures that the security constrained models build even when there are devices without set_device_model!()
@testset "Test if G-n with Ramp reserve deliverability constraints builds when there is a device without set_device_model!()" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    l4 = get_component(Line, c_sys5, "4")
    add_equivalent_ac_transmission_with_parallel_circuits!(
        c_sys5,
        l4,
        PSY.Line,
        PSY.MonitoredLine,
    )
    remove_component!(c_sys5, l4)

    systems = [c_sys5]

    PTDF_ref = IdDict{System, PTDF}(
        c_sys5 => PTDF(c_sys5),
    )

    components_outages_cases = IdDict{System, Vector{String}}(
        c_sys5 => ["Alta"],
    )
    for (ix, sys) in enumerate(systems)
        components_outages_names = components_outages_cases[sys]
        for component_name in components_outages_names
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end

        template =
            ProblemTemplate(NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]))
        set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
        set_device_model!(template, PowerLoad, StaticPowerLoad)
        #set_device_model!(template, MonitoredLine, StaticBranchBounds)
        set_device_model!(template, Line, StaticBranch)
        set_device_model!(template, Transformer2W, StaticBranch)
        set_device_model!(template, TapTransformer, StaticBranch)
        set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)

        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve1",
            ))

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
    end
end
@testset "Test SecurityConstrainedContingencyReserve with different BranchFormulations" begin
    for line_formulation in [StaticBranch, StaticBranchUnbounded, StaticBranchBounds]
        c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
        l4 = get_component(Line, c_sys5, "4")
        add_equivalent_ac_transmission_with_parallel_circuits!(
            c_sys5,
            l4,
            PSY.Line,
            PSY.MonitoredLine,
        )
        remove_component!(c_sys5, l4)

        systems = [c_sys5]

        PTDF_ref = IdDict{System, PTDF}(
            c_sys5 => PTDF(c_sys5),
        )

        components_outages_cases = IdDict{System, Vector{String}}(
            c_sys5 => ["Alta"],
        )
        for (ix, sys) in enumerate(systems)
            components_outages_names = components_outages_cases[sys]
            for component_name in components_outages_names
                # --- Create Outage Data ---
                transition_data = GeometricDistributionForcedOutage(;
                    mean_time_to_recovery = 10,
                    outage_transition_probability = 0.9999,
                    monitored_components = collect(get_components(ACTransmission, sys)),
                )
                # --- Add Outage Supplemental attribute to device and services that should respond ---
                component = get_component(ThermalStandard, sys, component_name)
                reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
                add_supplemental_attribute!(sys, component, transition_data)
                add_supplemental_attribute!(sys, reserve_up, transition_data)
            end

            template =
                ProblemTemplate(NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]))
            set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
            set_device_model!(template, PowerLoad, StaticPowerLoad)
            #set_device_model!(template, MonitoredLine, StaticBranchBounds)
            set_device_model!(template, Line, line_formulation)
            set_device_model!(template, Transformer2W, StaticBranch)
            set_device_model!(template, TapTransformer, StaticBranch)
            set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)

            set_service_model!(template,
                ServiceModel(
                    VariableReserve{ReserveUp},
                    SecurityConstrainedContingencyReserve,
                    "Reserve1",
                ))

            ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

            @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
                  PSI.ModelBuildStatus.BUILT
            constraints = ps_model.internal.container.constraints
            flow_rate_cons = constraints[PSI.ConstraintKey{
                PostContingencyFlowRateConstraint,
                VariableReserve{ReserveUp},
            }(
                "Reserve1_lb",
            )]
            @test length(flow_rate_cons) == 1 * 5 * 24
        end
    end
end

@testset "Test if G-n with Contingency reserve deliverability constraints builds when there is a device without set_device_model!()" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    l4 = get_component(Line, c_sys5, "4")
    add_equivalent_ac_transmission_with_parallel_circuits!(
        c_sys5,
        l4,
        PSY.Line,
        PSY.MonitoredLine,
    )
    remove_component!(c_sys5, l4)

    systems = [c_sys5]

    PTDF_ref = IdDict{System, PTDF}(
        c_sys5 => PTDF(c_sys5),
    )

    components_outages_cases = IdDict{System, Vector{String}}(
        c_sys5 => ["Alta"],
    )
    for (ix, sys) in enumerate(systems)
        components_outages_names = components_outages_cases[sys]
        for component_name in components_outages_names
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end

        template =
            ProblemTemplate(NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]))
        set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
        set_device_model!(template, PowerLoad, StaticPowerLoad)
        #set_device_model!(template, MonitoredLine, StaticBranchBounds)
        set_device_model!(template, Line, StaticBranch)
        set_device_model!(template, Transformer2W, StaticBranch)
        set_device_model!(template, TapTransformer, StaticBranch)
        set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)

        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1",
            ))

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
    end
end

@testset "G-n with Ramp reserve deliverability constraints UC allowing 2 reserve products to respond" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    systems = [c_sys5]
    objfuncs = [GAEVF, GQEVF, GQEVF]
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve11_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve11_ub",
        ),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve11",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve11",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve11",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve11",
        ),
    ]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5 => PTDF(c_sys5),
    )
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [960, 0, 1296, 600, 240],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 254242.0,
    )
    components_outages_cases = IdDict{System, Vector{String}}(
        c_sys5 => ["Alta"],
    )
    for (ix, sys) in enumerate(systems)
        components_outages_names = components_outages_cases[sys]
        for component_name in components_outages_names
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
            reserve_up2 = get_component(VariableReserve{ReserveUp}, sys, "Reserve11")
            add_supplemental_attribute!(sys, reserve_up2, transition_data)
        end

        template = get_thermal_dispatch_template_network(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF_ref[sys]),
        )

        set_device_model!(
            template,
            ThermalStandard,
            ThermalStandardUnitCommitment,
        )

        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve1",
            ))

        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve11",
            ))

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[sys]...,
            true,
        )
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
    end
end

@testset "G-n with Ramp reserve deliverability constraints with AreaPTDFPowerModel w/wo Reserve Slacks" begin
    reserve_slacks = [false, true]
    objfuncs = [GAEVF]
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
    ]
    test_results = IdDict{Bool, Vector{Int}}(
        reserve_slacks[1] => [744, 0, 1536, 1200, 168],
        reserve_slacks[2] => [2040, 0, 1536, 1200, 168],
    )
    test_obj_values = IdDict{Bool, Float64}(
        reserve_slacks[1] => 497000.0,
        reserve_slacks[2] => 497000.0,
    )
    components_outages_cases = (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for reserve_slack in reserve_slacks
        sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
        transform_single_time_series!(sys, Hour(24), Hour(1))

        components_outages_names, reserve_names = components_outages_cases
        for (component_name, reserve_name) in
            zip(components_outages_names, reserve_names)
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end

        template = get_thermal_dispatch_template_network(
            NetworkModel(AreaPTDFPowerModel; PTDF_matrix = PTDF(sys)),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve1_1";
                use_slacks = reserve_slack,
            ))
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve1_2";
                use_slacks = reserve_slack,
            ))
        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[reserve_slack]...,
            false,
        )
        psi_checkobjfun_test(ps_model, objfuncs[1])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[reserve_slack],
            10000,
        )
        res = OptimizationProblemResults(ps_model)
        for reserve_name in reserve_names
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            compare_outage_power_and_deployed_reserves(
                sys,
                res,
                reserve_up)
        end
    end
end

# Exercises the per-service line-scoping path in
# `_monitored_components_by_modeled_type` for the `AreaPTDFPowerModel`
# network model. Each reserve service monitors a different hand-picked
# subset of AC lines, which keeps the constraint key meta keyed by service
# name but reduces the number of `PostContingencyFlowRateConstraint` rows
# compared to the all-lines baseline.
@testset "G-n with Ramp reserve deliverability constraints with AreaPTDFPowerModel and monitored line subset" begin
    objfuncs = [GAEVF]
    monitored_line_names_per_service = (["1_1", "2_1"], ["1_2", "2_2"])
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
    ]
    # Counts are smaller than the all-lines baseline `[744, 0, 1536, 1200, 168]`
    # because each service monitors only two AC lines instead of all 13.
    test_results = [744, 0, 1008, 672, 168]
    test_obj_value = 497000.0
    components_outages_cases = (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(sys, Hour(24), Hour(1))

    components_outages_names, reserve_names = components_outages_cases
    for (component_name, reserve_name, monitored_names) in zip(
        components_outages_names,
        reserve_names,
        monitored_line_names_per_service,
    )
        monitored_subset = [get_component(Line, sys, n) for n in monitored_names]
        # --- Create Outage Data with a hand-picked monitored subset ---
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
            monitored_components = monitored_subset,
        )
        # --- Add Outage Supplemental attribute to device and services that should respond ---
        component = get_component(ThermalStandard, sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        add_supplemental_attribute!(sys, component, transition_data)
        add_supplemental_attribute!(sys, reserve_up, transition_data)
    end

    template = get_thermal_dispatch_template_network(
        NetworkModel(AreaPTDFPowerModel; PTDF_matrix = PTDF(sys)),
    )
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedRampReserve,
            "Reserve1_1",
        ))
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedRampReserve,
            "Reserve1_2",
        ))
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    psi_constraint_test(ps_model, constraint_keys)
    moi_tests(
        ps_model,
        test_results...,
        false,
    )
    psi_checkobjfun_test(ps_model, objfuncs[1])
    psi_checksolve_test(
        ps_model,
        [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
        test_obj_value,
        10000,
    )
    res = OptimizationProblemResults(ps_model)
    for reserve_name in reserve_names
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        compare_outage_power_and_deployed_reserves(
            sys,
            res,
            reserve_up)
    end
end

@testset "G-n with Contingency reserve deliverability constraints with AreaPTDFPowerModel, reserves only up, reserve requirement" begin
    reserve_slacks = [false, true]
    objfuncs = [GAEVF]
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
    ]
    test_results = IdDict{Bool, Vector{Int}}(
        reserve_slacks[1] => [744, 0, 1536, 1200, 168],
        reserve_slacks[2] => [2040, 0, 1536, 1200, 168],
    )
    test_obj_values = IdDict{Bool, Float64}(
        reserve_slacks[1] => 497000.0,
        reserve_slacks[2] => 497000.0,
    )
    components_outages_cases = (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for reserve_slack in reserve_slacks
        sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
        transform_single_time_series!(sys, Hour(24), Hour(1))
        components_outages_names, reserve_names = components_outages_cases
        for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end

        template = get_thermal_dispatch_template_network(
            NetworkModel(AreaPTDFPowerModel; PTDF_matrix = PTDF(sys)),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1_1";
                use_slacks = reserve_slack,
            ))
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1_2";
                use_slacks = reserve_slack,
            ))
        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[reserve_slack]...,
            false,
        )
        psi_checkobjfun_test(ps_model, objfuncs[1])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[reserve_slack],
            10000,
        )
        res = OptimizationProblemResults(ps_model)
        for reserve_name in reserve_names
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            compare_outage_power_and_deployed_reserves(
                sys,
                res,
                reserve_up)
        end
    end
end

@testset "G-n with Contingency reserve deliverability constraints with AreaPTDFPowerModel, reserves only up, NO reserve requirement" begin
    c_sys5_2area = PSB.build_system(PSISystems, "two_area_pjm_DA")
    transform_single_time_series!(c_sys5_2area, Hour(24), Hour(1))
    systems = [c_sys5_2area]
    objfuncs = [GAEVF]
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
    ]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5_2area => PTDF(c_sys5_2area),
    )
    test_results = IdDict{System, Vector{Int}}(
        c_sys5_2area => [504, 0, 1344, 1344, 216],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5_2area => 497000.0,
    )
    components_outages_cases = IdDict{System, Tuple{Vector{String}, Vector{String}}}(
        c_sys5_2area => (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"]),
    )
    for (ix, sys) in enumerate(systems)
        components_outages_names, reserve_names = components_outages_cases[sys]
        contributing_devices = get_components(
            g -> get_name(get_area(get_bus(g))) == "Area1",
            ThermalStandard,
            sys,
        )
        add_reserve_product_without_requirement_time_series!(
            sys,
            "Reserve1_1",
            "Up",
            contributing_devices,
        )
        contributing_devices = get_components(
            g -> get_name(get_area(get_bus(g))) == "Area2",
            ThermalStandard,
            sys,
        )
        add_reserve_product_without_requirement_time_series!(
            sys,
            "Reserve1_2",
            "Up",
            contributing_devices,
        )

        for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end

        template = get_thermal_dispatch_template_network(
            NetworkModel(AreaPTDFPowerModel; PTDF_matrix = PTDF_ref[sys]),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1_1",
            ))
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1_2",
            ))
        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
        res = OptimizationProblemResults(ps_model)
        for reserve_name in reserve_names
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            compare_outage_power_and_deployed_reserves(
                sys,
                res,
                reserve_up)
        end
    end
end

@testset "G-n with Ramp reserve deliverability constraints with CopperPlatePowerModel" begin
    c_sys5_2area = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(c_sys5_2area, Hour(24), Hour(1))
    systems = [c_sys5_2area]
    objfuncs = [GAEVF]
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
    ]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5_2area => PTDF(c_sys5_2area),
    )
    test_results = IdDict{System, Vector{Int}}(
        c_sys5_2area => [720, 0, 624, 288, 120],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5_2area => 497494.48,
    )
    components_outages_cases = IdDict{System, Tuple{Vector{String}, Vector{String}}}(
        c_sys5_2area => (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"]),
    )
    for (ix, sys) in enumerate(systems)
        components_outages_names, reserve_names = components_outages_cases[sys]
        for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end

        template = get_thermal_dispatch_template_network(
            NetworkModel(CopperPlatePowerModel; PTDF_matrix = PTDF_ref[sys]),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve1_1",
            ))
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve1_2",
            ))
        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
        res = OptimizationProblemResults(ps_model)
        for reserve_name in reserve_names
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            compare_outage_power_and_deployed_reserves(
                sys,
                res,
                reserve_up)
        end
    end
end

@testset "G-n with Contingency reserve deliverability constraints with CopperPlatePowerModel with Reserve Requirement" begin
    c_sys5_2area = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(c_sys5_2area, Hour(24), Hour(1))
    systems = [c_sys5_2area]
    objfuncs = [GAEVF]
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
    ]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5_2area => PTDF(c_sys5_2area),
    )
    test_results = IdDict{System, Vector{Int}}(
        c_sys5_2area => [720, 0, 624, 288, 120],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5_2area => 497494.48,
    )
    components_outages_cases = IdDict{System, Tuple{Vector{String}, Vector{String}}}(
        c_sys5_2area => (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"]),
    )
    for (ix, sys) in enumerate(systems)
        components_outages_names, reserve_names = components_outages_cases[sys]
        for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end

        template = get_thermal_dispatch_template_network(
            NetworkModel(CopperPlatePowerModel; PTDF_matrix = PTDF_ref[sys]),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1_1",
            ))
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1_2",
            ))
        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
        res = OptimizationProblemResults(ps_model)
        for reserve_name in reserve_names
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            compare_outage_power_and_deployed_reserves(
                sys,
                res,
                reserve_up)
        end
    end
end

@testset "G-n with Contingency reserve deliverability constraints with CopperPlatePowerModel with NO Reserve Requirement" begin
    c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    systems = [c_sys5]
    objfuncs = [GAEVF]
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_ub",
        )]
    PTDF_ref = IdDict{System, PTDF}(
        c_sys5 => PTDF(c_sys5),
    )
    test_results = IdDict{System, Vector{Int}}(
        c_sys5 => [240, 0, 216, 216, 96],
    )
    test_obj_values = IdDict{System, Float64}(
        c_sys5 => 329000.0,
    )
    components_outages_cases = IdDict{System, Vector{String}}(
        c_sys5 => ["Alta"],
    )
    for (ix, sys) in enumerate(systems)
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
        remove_time_series!(
            sys,
            Deterministic,
            reserve_up,
            "requirement",
        )

        components_outages_names = components_outages_cases[sys]
        for component_name in components_outages_names
            # --- Create Outage Data ---
            transition_data = GeometricDistributionForcedOutage(;
                mean_time_to_recovery = 10,
                outage_transition_probability = 0.9999,
                monitored_components = collect(get_components(ACTransmission, sys)),
            )
            # --- Add Outage Supplemental attribute to device and services that should respond ---
            component = get_component(ThermalStandard, sys, component_name)
            add_supplemental_attribute!(sys, component, transition_data)
            add_supplemental_attribute!(sys, reserve_up, transition_data)
        end

        template = get_thermal_dispatch_template_network(
            NetworkModel(CopperPlatePowerModel; PTDF_matrix = PTDF_ref[sys]),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1",
            ))

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        psi_constraint_test(ps_model, constraint_keys)
        moi_tests(
            ps_model,
            test_results[sys]...,
            false,
        )
        psi_checkobjfun_test(ps_model, objfuncs[ix])
        psi_checksolve_test(
            ps_model,
            [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
            test_obj_values[sys],
            10000,
        )
        res = OptimizationProblemResults(ps_model)
        compare_outage_power_and_deployed_reserves(
            sys,
            res,
            reserve_up)
    end
end

@testset "G-n with Ramp reserve deliverability constraints with AreaBalance PowerModel" begin
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyCopperPlateBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyCopperPlateBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
    ]

    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(c_sys, Hour(24), Hour(1))
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        # --- Create Outage Data ---
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
            monitored_components = vcat(
                collect(get_components(ACTransmission, c_sys)),
                collect(get_components(AreaInterchange, c_sys)),
            ),
        )
        # --- Add Outage Supplemental attribute to device and services that should respond ---
        component = get_component(ThermalStandard, c_sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        add_supplemental_attribute!(c_sys, component, transition_data)
        add_supplemental_attribute!(c_sys, reserve_up, transition_data)
    end

    template = get_thermal_dispatch_template_network(NetworkModel(AreaBalancePowerModel))
    set_device_model!(template, AreaInterchange, StaticBranch)

    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedRampReserve,
            "Reserve1_1",
        ))
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedRampReserve,
            "Reserve1_2",
        ))

    ps_model =
        DecisionModel(template, c_sys; resolution = Hour(1), optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    psi_constraint_test(ps_model, constraint_keys)

    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 744, 0, 696, 360, 240, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 497494.4871638, 1)

    results = OptimizationProblemResults(ps_model)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    # The values for these tests come from the data
    @test all(interarea_flow[!, "1_2"] .<= 150 + 1e-6)
    @test all(interarea_flow[!, "1_2"] .>= -150 - 1e-6)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Bus4_1", "Bus3_1", "Bus2_1"]]))
    zone_1_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_1", "Park City_1", "Sundance_1", "Brighton_1", "Alta_1"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_1_gen .+ zone_1_load .- interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    zone_2_load = sum(eachcol(load[!, ["Bus4_2", "Bus3_2", "Bus2_2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_2", "Park City_2", "Sundance_2", "Brighton_2", "Alta_2"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_2_gen .+ zone_2_load .+ interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    res = OptimizationProblemResults(ps_model)
    for reserve_name in reserve_names
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        compare_outage_power_and_deployed_reserves(
            c_sys,
            res,
            reserve_up)
    end
end

@testset "G-n with Contingency reserve deliverability constraints with AreaBalancePowerModel with Reserve Requirement" begin
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RequirementConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            RampConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyCopperPlateBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyCopperPlateBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
    ]

    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(c_sys, Hour(24), Hour(1))
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        # --- Create Outage Data ---
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
            monitored_components = vcat(
                collect(get_components(ACTransmission, c_sys)),
                collect(get_components(AreaInterchange, c_sys)),
            ),
        )
        # --- Add Outage Supplemental attribute to device and services that should respond ---
        component = get_component(ThermalStandard, c_sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        add_supplemental_attribute!(c_sys, component, transition_data)
        add_supplemental_attribute!(c_sys, reserve_up, transition_data)
    end

    template = get_thermal_dispatch_template_network(NetworkModel(AreaBalancePowerModel))
    set_device_model!(template, AreaInterchange, StaticBranch)

    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1_1",
        ))
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1_2",
        ))

    ps_model =
        DecisionModel(template, c_sys; resolution = Hour(1), optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    psi_constraint_test(ps_model, constraint_keys)

    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 744, 0, 696, 360, 240, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 497494.4871638, 1)

    results = OptimizationProblemResults(ps_model)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    # The values for these tests come from the data
    @test all(interarea_flow[!, "1_2"] .<= 150 + 1e-6)
    @test all(interarea_flow[!, "1_2"] .>= -150 - 1e-6)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Bus4_1", "Bus3_1", "Bus2_1"]]))
    zone_1_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_1", "Park City_1", "Sundance_1", "Brighton_1", "Alta_1"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_1_gen .+ zone_1_load .- interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    zone_2_load = sum(eachcol(load[!, ["Bus4_2", "Bus3_2", "Bus2_2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_2", "Park City_2", "Sundance_2", "Brighton_2", "Alta_2"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_2_gen .+ zone_2_load .+ interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    res = OptimizationProblemResults(ps_model)
    for reserve_name in reserve_names
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        compare_outage_power_and_deployed_reserves(
            c_sys,
            res,
            reserve_up)
    end
end

@testset "G-n with Contingency reserve deliverability constraints with AreaBalancePowerModel with NO Reserve Requirement" begin
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyActivePowerGenerationLimitsConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyCopperPlateBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
        PSI.ConstraintKey(
            PostContingencyCopperPlateBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1_ub",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_lb",
        ),
        PSI.ConstraintKey(
            PostContingencyFlowRateConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_2_ub",
        ),
    ]

    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)

    reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, "Reserve1_1")
    remove_time_series!(
        c_sys,
        SingleTimeSeries,
        reserve_up,
        "requirement",
    )
    reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, "Reserve1_2")
    remove_time_series!(
        c_sys,
        SingleTimeSeries,
        reserve_up,
        "requirement",
    )

    transform_single_time_series!(c_sys, Hour(24), Hour(1))
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        # --- Create Outage Data ---
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.9999,
            monitored_components = vcat(
                collect(get_components(ACTransmission, c_sys)),
                collect(get_components(AreaInterchange, c_sys)),
            ),
        )
        # --- Add Outage Supplemental attribute to device and services that should respond ---
        component = get_component(ThermalStandard, c_sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        add_supplemental_attribute!(c_sys, component, transition_data)
        add_supplemental_attribute!(c_sys, reserve_up, transition_data)
    end

    template = get_thermal_dispatch_template_network(NetworkModel(AreaBalancePowerModel))
    set_device_model!(template, AreaInterchange, StaticBranch)

    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1_1",
        ))
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1_2",
        ))

    ps_model =
        DecisionModel(template, c_sys; resolution = Hour(1), optimizer = HiGHS_optimizer)

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    psi_constraint_test(ps_model, constraint_keys)

    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    moi_tests(ps_model, 504, 0, 504, 504, 288, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 482055.7647083302, 1)

    results = OptimizationProblemResults(ps_model)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    # The values for these tests come from the data
    @test all(interarea_flow[!, "1_2"] .<= 150)
    @test all(interarea_flow[!, "1_2"] .>= -150)

    load = read_parameter(
        results,
        "ActivePowerTimeSeriesParameter__PowerLoad";
        table_format = TableFormat.WIDE,
    )
    thermal_gen = read_variable(
        results,
        "ActivePowerVariable__ThermalStandard";
        table_format = TableFormat.WIDE,
    )

    zone_1_load = sum(eachcol(load[!, ["Bus4_1", "Bus3_1", "Bus2_1"]]))
    zone_1_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_1", "Park City_1", "Sundance_1", "Brighton_1", "Alta_1"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_1_gen .+ zone_1_load .- interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    zone_2_load = sum(eachcol(load[!, ["Bus4_2", "Bus3_2", "Bus2_2"]]))
    zone_2_gen = sum(
        eachcol(
            thermal_gen[
                !,
                ["Solitude_2", "Park City_2", "Sundance_2", "Brighton_2", "Alta_2"],
            ],
        ),
    )
    @test all(
        isapprox.(
            sum(zone_2_gen .+ zone_2_load .+ interarea_flow[!, "1_2"]; dims = 2),
            0.0;
            atol = 1e-3,
        ),
    )

    res = OptimizationProblemResults(ps_model)
    for reserve_name in reserve_names
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        compare_outage_power_and_deployed_reserves(
            c_sys,
            res,
            reserve_up)
    end
end

# Regression test for per-service outage scoping under the
# attachment-as-the-rule contract: a security-constrained reserve service
# responds to exactly the outages attached to it via
# `add_supplemental_attribute!(sys, service, outage)`. Generator attachment
# is required for the post-contingency build (so the outaged generator can
# be pinned to zero deployment), but it is the *service* attachment that
# selects which `ServiceModel` claims the outage. Membership in the
# service's contributing-devices set is irrelevant to the selection.
@testset "SC reserve outage attachment scopes responding services" begin
    sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(sys, Hour(24), Hour(1))

    reserve1 = get_component(VariableReserve{ReserveUp}, sys, "Reserve1_1")

    # Attach an UnplannedOutage to a single Area1 generator and to the
    # reserve that should respond. Reserve1_2 is intentionally NOT attached.
    alta1 = get_component(ThermalStandard, sys, "Alta_1")
    transition_data = GeometricDistributionForcedOutage(;
        mean_time_to_recovery = 10,
        outage_transition_probability = 0.9999,
        monitored_components = collect(get_components(ACTransmission, sys)),
    )
    add_supplemental_attribute!(sys, alta1, transition_data)
    add_supplemental_attribute!(sys, reserve1, transition_data)
    outage_uuid = IS.get_uuid(transition_data)

    template = get_thermal_dispatch_template_network(
        NetworkModel(AreaPTDFPowerModel; PTDF_matrix = PTDF(sys)),
    )
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedRampReserve,
            "Reserve1_1",
        ))
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedRampReserve,
            "Reserve1_2",
        ))

    # --- Unit-level: attachment scoping populates only the responding ServiceModel ---
    PSI._build_service_model_outages!(template, sys)

    services = PSI.get_service_models(template)
    sm1 = services[("Reserve1_1", Symbol(VariableReserve{ReserveUp}))]
    sm2 = services[("Reserve1_2", Symbol(VariableReserve{ReserveUp}))]
    @test haskey(sm1.outages, outage_uuid)
    @test !haskey(sm2.outages, outage_uuid)

    # --- Build-level: post-contingency constraints fire only on Reserve1_1 ---
    ps_model =
        DecisionModel(template, sys; resolution = Hour(1), optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    @test PSI.has_container_key(
        container,
        PostContingencyGenerationBalanceConstraint,
        PSY.VariableReserve{ReserveUp},
        "Reserve1_1",
    )
    @test !PSI.has_container_key(
        container,
        PostContingencyGenerationBalanceConstraint,
        PSY.VariableReserve{ReserveUp},
        "Reserve1_2",
    )
    cons_resp = PSI.get_constraint(
        container,
        PSI.ConstraintKey(
            PostContingencyGenerationBalanceConstraint,
            PSY.VariableReserve{ReserveUp},
            "Reserve1_1",
        ),
    )
    @test size(cons_resp) == (1, 24)
end

# Regression test for the single-reserve case: when only one SC reserve
# service is in the template, an outage attached to both the outaged
# generator and the reserve must end up in that ServiceModel.outages dict.
@testset "SC reserve outage attachment covers single-reserve case" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    transition_data = GeometricDistributionForcedOutage(;
        mean_time_to_recovery = 10,
        outage_transition_probability = 0.9999,
        monitored_components = collect(get_components(ACTransmission, sys)),
    )
    alta = get_component(ThermalStandard, sys, "Alta")
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    add_supplemental_attribute!(sys, alta, transition_data)
    add_supplemental_attribute!(sys, reserve_up, transition_data)
    outage_uuid = IS.get_uuid(transition_data)

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
    )
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedRampReserve,
            "Reserve1",
        ))

    PSI._build_service_model_outages!(template, sys)

    services = PSI.get_service_models(template)
    sm = services[("Reserve1", Symbol(VariableReserve{ReserveUp}))]
    @test haskey(sm.outages, outage_uuid)
end
