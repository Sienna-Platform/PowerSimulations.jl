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
        outage_rows = filter(x -> x["name"] == outage_name, required_variables)
        outage_power_v = Vector{Float64}()
        for (i, device) in enumerate(contributing_devices)
            device_name = PSY.get_name(device)
            current_v =
                filter(x -> x[col_name] == device_name, outage_rows)[!, "value"]
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
    require_positive_outage::Bool = true,
)
    variablesdict = read_variables(res)
    associated_outages =
        collect(PSY.get_supplemental_attributes(PSY.UnplannedOutage, service))
    isempty(associated_outages) &&
        error("service $(PSY.get_name(service)) has no attached outages")
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
        # Guard against a vacuous `0 == 0` pass: if the outaged unit never
        # dispatches, both sides of the identity are trivially zero and the
        # check below verifies nothing. `require_positive_outage = false`
        # opts a testset out where zero outaged dispatch is expected.
        if require_positive_outage
            @test maximum(outage_dict[outage_name]) > tolerance
        end
        for i in 1:length(outage_dict[outage_name])
            test_reserves_deployment(
                outage_dict[outage_name][i],
                reserve_dict[outage_name][i],
            )
        end
    end
end

function compare_outage_power_and_deployed_reserves(
    sys::PSY.System,
    res::OptimizationProblemResults,
    reserve_names::Vector{String};
    tolerance::Float64 = 1e-3,
    require_positive_outage::Bool = true,
)
    for reserve_name in reserve_names
        reserve = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        compare_outage_power_and_deployed_reserves(
            sys,
            res,
            reserve;
            tolerance = tolerance,
            require_positive_outage = require_positive_outage,
        )
    end
    return nothing
end

# Shorthand for the `PSI.ConstraintKey(T, PSY.VariableReserve{ReserveUp}, name)`
# pattern that dominates the constraint-key lists below.
function sc_key(constraint_type::DataType, name::String)
    return PSI.ConstraintKey(constraint_type, PSY.VariableReserve{ReserveUp}, name)
end

# Shared tail for the SC reserve deliverability testsets: build, check
# constraint keys, MOI counts, objective function type, solve status/value,
# and (when `reserve_names` is non-empty) that deployed reserves match the
# outaged power.
function run_sc_reserve_case!(
    ps_model::DecisionModel,
    sys::PSY.System,
    constraint_keys::Vector{<:PSI.ConstraintKey},
    moi_counts::Vector{Int},
    has_binaries::Bool,
    objfunc::DataType,
    obj_value::Float64,
    reserve_names::Vector{String};
    obj_tol::Float64 = 10000.0,
    require_positive_outage::Bool = true,
)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    psi_constraint_test(ps_model, constraint_keys)
    moi_tests(ps_model, moi_counts..., has_binaries)
    psi_checkobjfun_test(ps_model, objfunc)
    psi_checksolve_test(
        ps_model,
        [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL],
        obj_value,
        obj_tol,
    )
    if isempty(reserve_names)
        return nothing
    end
    res = OptimizationProblemResults(ps_model)
    compare_outage_power_and_deployed_reserves(
        sys,
        res,
        reserve_names;
        require_positive_outage = require_positive_outage,
    )
    return nothing
end

# Shared zone-balance verification for the `AreaBalancePowerModel` testsets:
# the monitored "1_2" tie stays within its flow limits, and each area's
# generation plus load nets to zero against the tie flow.
function verify_zone_balance!(
    results::OptimizationProblemResults;
    flow_tol::Float64 = 1e-6,
)
    interarea_flow = read_variable(
        results,
        "FlowActivePowerVariable__AreaInterchange";
        table_format = TableFormat.WIDE,
    )
    @test all(interarea_flow[!, "1_2"] .<= 150 + flow_tol)
    @test all(interarea_flow[!, "1_2"] .>= -150 - flow_tol)

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
    return nothing
end

@testset "G-n with Ramp reserve deliverability constraints Dispatch with responding reserves only up, including reduction of parallel circuits" begin
    for add_parallel_line in [true, false]
        sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
        if add_parallel_line
            l4 = get_component(Line, sys, "4")
            add_equivalent_ac_transmission_with_parallel_circuits!(sys, l4, PSY.Line)
        end
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
            sc_key(PostContingencyFlowRateConstraint, "Reserve1_lb"),
            sc_key(PostContingencyFlowRateConstraint, "Reserve1_ub"),
            PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
            #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
            sc_key(RequirementConstraint, "Reserve1"),
            sc_key(RampConstraint, "Reserve1"),
            sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1"),
            sc_key(
                PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
                "Reserve1",
            ),
        ]
        gen = get_component(ThermalStandard, sys, "Solitude")
        set_ramp_limits!(gen, (up = 0.4, down = 0.4)) #Increase ramp limits to make the problem feasible
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
        component = get_component(ThermalStandard, sys, "Alta")
        attach_geometric_outage!(sys, component, [reserve_up])

        template = get_thermal_dispatch_template_network(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedRampReserve,
                "Reserve1",
            ))

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        run_sc_reserve_case!(
            ps_model,
            sys,
            constraint_keys,
            [360, 0, 600, 432, 48],
            false,
            GAEVF,
            329000.0,
            ["Reserve1"],
        )
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
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    monitored_line_names = ["1", "2"]
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
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        sc_key(RequirementConstraint, "Reserve1"),
        sc_key(RampConstraint, "Reserve1"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1",
        ),
    ]
    # Counts are smaller than the all-lines baseline `[360, 0, 600, 432, 72]`
    # because only the monitored subset contributes
    # `PostContingencyFlowRateConstraint` rows per outage step.
    gen = get_component(ThermalStandard, sys, "Solitude")
    set_ramp_limits!(gen, (up = 0.4, down = 0.4)) #Increase ramp limits to make the problem feasible
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    monitored_subset = [get_component(Line, sys, n) for n in monitored_line_names]
    component = get_component(ThermalStandard, sys, "Alta")
    attach_geometric_outage!(
        sys,
        component,
        [reserve_up];
        monitored_components = monitored_subset,
    )

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
    )
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedRampReserve,
            "Reserve1",
        ))

    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

    run_sc_reserve_case!(
        ps_model,
        sys,
        constraint_keys,
        [360, 0, 504, 336, 48],
        false,
        GAEVF,
        329000.0,
        ["Reserve1"],
    )
end

@testset "G-n with contingency reserves deliverability constraints including responding reserves only up, reserve requirement, and reduction of parallel circuits" begin
    for add_parallel_line in [true, false]
        sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

        if add_parallel_line
            l4 = get_component(Line, sys, "4")
            add_equivalent_ac_transmission_with_parallel_circuits!(sys, l4, PSY.Line)
        end
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
            sc_key(PostContingencyFlowRateConstraint, "Reserve1_lb"),
            sc_key(PostContingencyFlowRateConstraint, "Reserve1_ub"),
            PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
            #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
            sc_key(RequirementConstraint, "Reserve1"),
            sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1"),
            sc_key(
                PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
                "Reserve1",
            ),
        ]
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
        component = get_component(ThermalStandard, sys, "Alta")
        attach_geometric_outage!(sys, component, [reserve_up])

        template = get_thermal_dispatch_template_network(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1",
            ))

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        run_sc_reserve_case!(
            ps_model,
            sys,
            constraint_keys,
            [360, 0, 504, 432, 48],
            false,
            GAEVF,
            329000.0,
            ["Reserve1"],
        )
    end
end

@testset "G-n with contingency reserves deliverability constraints including responding reserves only up, NO reserve requirement, and reduction of parallel circuits" begin
    for add_parallel_line in [true, false]
        c_sys5 = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

        if add_parallel_line
            l4 = get_component(Line, c_sys5, "4")
            add_equivalent_ac_transmission_with_parallel_circuits!(c_sys5, l4, PSY.Line)
        end
        sys = c_sys5
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
            sc_key(PostContingencyFlowRateConstraint, "Reserve1_lb"),
            sc_key(PostContingencyFlowRateConstraint, "Reserve1_ub"),
            PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
            #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
            sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1"),
            sc_key(PostContingencyActivePowerGenerationLimitsConstraint, "Reserve1_ub"),
        ]
        gen = get_component(ThermalStandard, sys, "Solitude")
        set_ramp_limits!(gen, (up = 0.4, down = 0.4)) #Increase ramp limits to make the problem feasible
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
        remove_time_series!(
            sys,
            Deterministic,
            reserve_up,
            "requirement",
        )
        component = get_component(ThermalStandard, sys, "Alta")
        attach_geometric_outage!(sys, component, [reserve_up])

        template = get_thermal_dispatch_template_network(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
        )
        set_service_model!(template,
            ServiceModel(
                VariableReserve{ReserveUp},
                SecurityConstrainedContingencyReserve,
                "Reserve1",
            ))

        ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

        run_sc_reserve_case!(
            ps_model,
            sys,
            constraint_keys,
            [240, 0, 504, 408, 72],
            false,
            GAEVF,
            329000.0,
            ["Reserve1"],
        )
    end
end

#This test ensures that the security constrained models build even when there are devices without set_device_model!()
@testset "Test if G-n with Ramp reserve deliverability constraints builds when there is a device without set_device_model!()" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    l4 = get_component(Line, sys, "4")
    add_equivalent_ac_transmission_with_parallel_circuits!(
        sys,
        l4,
        PSY.Line,
        PSY.MonitoredLine,
    )
    remove_component!(sys, l4)

    component = get_component(ThermalStandard, sys, "Alta")
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    attach_geometric_outage!(sys, component, [reserve_up])

    template = ProblemTemplate(NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)))
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
@testset "Test SecurityConstrainedContingencyReserve with different BranchFormulations" begin
    for line_formulation in [StaticBranch, StaticBranchUnbounded, StaticBranchBounds]
        sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
        l4 = get_component(Line, sys, "4")
        add_equivalent_ac_transmission_with_parallel_circuits!(
            sys,
            l4,
            PSY.Line,
            PSY.MonitoredLine,
        )
        remove_component!(sys, l4)

        component = get_component(ThermalStandard, sys, "Alta")
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
        attach_geometric_outage!(sys, component, [reserve_up])

        template = ProblemTemplate(NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)))
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

@testset "Test if G-n with Contingency reserve deliverability constraints builds when there is a device without set_device_model!()" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    l4 = get_component(Line, sys, "4")
    add_equivalent_ac_transmission_with_parallel_circuits!(
        sys,
        l4,
        PSY.Line,
        PSY.MonitoredLine,
    )
    remove_component!(sys, l4)

    component = get_component(ThermalStandard, sys, "Alta")
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    attach_geometric_outage!(sys, component, [reserve_up])

    template = ProblemTemplate(NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)))
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

@testset "G-n with Ramp reserve deliverability constraints UC allowing 2 reserve products to respond" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve11_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve11_ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
        sc_key(RequirementConstraint, "Reserve1"),
        sc_key(RequirementConstraint, "Reserve11"),
        sc_key(RampConstraint, "Reserve1"),
        sc_key(RampConstraint, "Reserve11"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve11"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1",
        ),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve11",
        ),
    ]
    component = get_component(ThermalStandard, sys, "Alta")
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    reserve_up2 = get_component(VariableReserve{ReserveUp}, sys, "Reserve11")
    attach_geometric_outage!(sys, component, [reserve_up, reserve_up2])

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
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

    run_sc_reserve_case!(
        ps_model,
        sys,
        constraint_keys,
        [960, 0, 1296, 600, 192],
        true,
        GAEVF,
        254242.0,
        String[],
    )
end

@testset "G-n with Ramp reserve deliverability constraints with AreaPTDFPowerModel w/wo Reserve Slacks" begin
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        sc_key(RequirementConstraint, "Reserve1_1"),
        sc_key(RequirementConstraint, "Reserve1_2"),
        sc_key(RampConstraint, "Reserve1_1"),
        sc_key(RampConstraint, "Reserve1_2"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_2"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_1",
        ),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_2",
        ),
    ]
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for reserve_slack in [false, true]
        sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
        transform_single_time_series!(sys, Hour(24), Hour(1))

        for (component_name, reserve_name) in
            zip(components_outages_names, reserve_names)
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            attach_geometric_outage!(sys, component, [reserve_up])
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

        if reserve_slack
            moi_counts = [2040, 0, 1536, 1200, 120]
        else
            moi_counts = [744, 0, 1536, 1200, 120]
        end
        run_sc_reserve_case!(
            ps_model,
            sys,
            constraint_keys,
            moi_counts,
            false,
            GAEVF,
            497000.0,
            reserve_names,
        )
    end
end

# Exercises the per-service line-scoping path in
# `_monitored_components_by_modeled_type` for the `AreaPTDFPowerModel`
# network model. Each reserve service monitors a different hand-picked
# subset of AC lines, which keeps the constraint key meta keyed by service
# name but reduces the number of `PostContingencyFlowRateConstraint` rows
# compared to the all-lines baseline.
@testset "G-n with Ramp reserve deliverability constraints with AreaPTDFPowerModel and monitored line subset" begin
    monitored_line_names_per_service = (["1_1", "2_1"], ["1_2", "2_2"])
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        sc_key(RequirementConstraint, "Reserve1_1"),
        sc_key(RequirementConstraint, "Reserve1_2"),
        sc_key(RampConstraint, "Reserve1_1"),
        sc_key(RampConstraint, "Reserve1_2"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_2"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_1",
        ),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_2",
        ),
    ]
    # Counts are smaller than the all-lines baseline `[744, 0, 1536, 1200, 120]`
    # because each service monitors only two AC lines instead of all 13.
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(sys, Hour(24), Hour(1))

    for (component_name, reserve_name, monitored_names) in zip(
        components_outages_names,
        reserve_names,
        monitored_line_names_per_service,
    )
        monitored_subset = [get_component(Line, sys, n) for n in monitored_names]
        component = get_component(ThermalStandard, sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        attach_geometric_outage!(
            sys,
            component,
            [reserve_up];
            monitored_components = monitored_subset,
        )
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

    run_sc_reserve_case!(
        ps_model,
        sys,
        constraint_keys,
        [744, 0, 1008, 672, 120],
        false,
        GAEVF,
        497000.0,
        reserve_names,
    )
end

@testset "G-n with Contingency reserve deliverability constraints with AreaPTDFPowerModel, reserves only up, reserve requirement" begin
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        sc_key(RequirementConstraint, "Reserve1_1"),
        sc_key(RequirementConstraint, "Reserve1_2"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_2"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_1",
        ),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_2",
        ),
    ]
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for reserve_slack in [false, true]
        sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
        transform_single_time_series!(sys, Hour(24), Hour(1))
        for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
            component = get_component(ThermalStandard, sys, component_name)
            reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
            attach_geometric_outage!(sys, component, [reserve_up])
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

        if reserve_slack
            moi_counts = [2040, 0, 1344, 1200, 120]
        else
            moi_counts = [744, 0, 1344, 1200, 120]
        end
        run_sc_reserve_case!(
            ps_model,
            sys,
            constraint_keys,
            moi_counts,
            false,
            GAEVF,
            497000.0,
            reserve_names,
        )
    end
end

@testset "G-n with Contingency reserve deliverability constraints with AreaPTDFPowerModel, reserves only up, NO reserve requirement" begin
    sys = PSB.build_system(PSISystems, "two_area_pjm_DA")
    transform_single_time_series!(sys, Hour(24), Hour(1))
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "lb"),
        PSI.ConstraintKey(FlowRateConstraint, PSY.Line, "ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_ub"),
        PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        #PSI.ConstraintKey(NetworkFlowConstraint, PSY.Line),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_2"),
        sc_key(PostContingencyActivePowerGenerationLimitsConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyActivePowerGenerationLimitsConstraint, "Reserve1_2_ub"),
    ]
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

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
        component = get_component(ThermalStandard, sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        attach_geometric_outage!(sys, component, [reserve_up])
    end

    template = get_thermal_dispatch_template_network(
        NetworkModel(AreaPTDFPowerModel; PTDF_matrix = PTDF(sys)),
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

    run_sc_reserve_case!(
        ps_model,
        sys,
        constraint_keys,
        [504, 0, 1344, 1152, 168],
        false,
        GAEVF,
        497000.0,
        reserve_names,
    )
end

@testset "G-n with Ramp reserve deliverability constraints with CopperPlatePowerModel" begin
    sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(sys, Hour(24), Hour(1))
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        sc_key(RequirementConstraint, "Reserve1_1"),
        sc_key(RequirementConstraint, "Reserve1_2"),
        sc_key(RampConstraint, "Reserve1_1"),
        sc_key(RampConstraint, "Reserve1_2"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_2"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_1",
        ),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_2",
        ),
    ]
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        component = get_component(ThermalStandard, sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        attach_geometric_outage!(sys, component, [reserve_up])
    end

    template = get_thermal_dispatch_template_network(
        NetworkModel(CopperPlatePowerModel; PTDF_matrix = PTDF(sys)),
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

    run_sc_reserve_case!(
        ps_model,
        sys,
        constraint_keys,
        [720, 0, 624, 288, 72],
        false,
        GAEVF,
        497494.48,
        reserve_names,
    )
end

@testset "G-n with Contingency reserve deliverability constraints with CopperPlatePowerModel with Reserve Requirement" begin
    sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(sys, Hour(24), Hour(1))
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        sc_key(RequirementConstraint, "Reserve1_1"),
        sc_key(RequirementConstraint, "Reserve1_2"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_2"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_1",
        ),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_2",
        ),
    ]
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        component = get_component(ThermalStandard, sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        attach_geometric_outage!(sys, component, [reserve_up])
    end

    template = get_thermal_dispatch_template_network(
        NetworkModel(CopperPlatePowerModel; PTDF_matrix = PTDF(sys)),
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

    run_sc_reserve_case!(
        ps_model,
        sys,
        constraint_keys,
        [720, 0, 432, 288, 72],
        false,
        GAEVF,
        482055.92,
        reserve_names,
    )
end

@testset "G-n with Contingency reserve deliverability constraints with CopperPlatePowerModel with NO Reserve Requirement" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.System),
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1"),
        sc_key(PostContingencyActivePowerGenerationLimitsConstraint, "Reserve1_ub")]
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    remove_time_series!(
        sys,
        Deterministic,
        reserve_up,
        "requirement",
    )
    component = get_component(ThermalStandard, sys, "Alta")
    attach_geometric_outage!(sys, component, [reserve_up])

    template = get_thermal_dispatch_template_network(
        NetworkModel(CopperPlatePowerModel; PTDF_matrix = PTDF(sys)),
    )
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1",
        ))

    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)

    run_sc_reserve_case!(
        ps_model,
        sys,
        constraint_keys,
        [240, 0, 216, 120, 72],
        false,
        GAEVF,
        329000.0,
        ["Reserve1"],
    )
end

@testset "G-n with Ramp reserve deliverability constraints with AreaBalance PowerModel" begin
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        sc_key(RequirementConstraint, "Reserve1_1"),
        sc_key(RequirementConstraint, "Reserve1_2"),
        sc_key(RampConstraint, "Reserve1_1"),
        sc_key(RampConstraint, "Reserve1_2"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_1",
        ),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_2",
        ),
        sc_key(PostContingencyCopperPlateBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyCopperPlateBalanceConstraint, "Reserve1_2"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_ub"),
    ]

    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(c_sys, Hour(24), Hour(1))
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    monitored_components = vcat(
        collect(get_components(ACTransmission, c_sys)),
        collect(get_components(AreaInterchange, c_sys)),
    )
    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        component = get_component(ThermalStandard, c_sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        attach_geometric_outage!(
            c_sys,
            component,
            [reserve_up];
            monitored_components = monitored_components,
        )
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

    moi_tests(ps_model, 792, 0, 696, 360, 144, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 497494.4871638, 1)

    results = OptimizationProblemResults(ps_model)
    verify_zone_balance!(results; flow_tol = 1e-6)

    compare_outage_power_and_deployed_reserves(c_sys, results, reserve_names)
end

@testset "G-n with Contingency reserve deliverability constraints with AreaBalancePowerModel with Reserve Requirement" begin
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        sc_key(RequirementConstraint, "Reserve1_1"),
        sc_key(RequirementConstraint, "Reserve1_2"),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_1",
        ),
        sc_key(
            PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint,
            "Reserve1_2",
        ),
        sc_key(PostContingencyCopperPlateBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyCopperPlateBalanceConstraint, "Reserve1_2"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_ub"),
    ]

    c_sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(c_sys, Hour(24), Hour(1))
    components_outages_names, reserve_names =
        (["Alta_1", "Alta_2"], ["Reserve1_1", "Reserve1_2"])

    monitored_components = vcat(
        collect(get_components(ACTransmission, c_sys)),
        collect(get_components(AreaInterchange, c_sys)),
    )
    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        component = get_component(ThermalStandard, c_sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        attach_geometric_outage!(
            c_sys,
            component,
            [reserve_up];
            monitored_components = monitored_components,
        )
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

    moi_tests(ps_model, 792, 0, 504, 360, 144, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 482055.9151334, 1)

    results = OptimizationProblemResults(ps_model)
    verify_zone_balance!(results; flow_tol = 1e-6)

    compare_outage_power_and_deployed_reserves(c_sys, results, reserve_names)
end

@testset "G-n with Contingency reserve deliverability constraints with AreaBalancePowerModel with NO Reserve Requirement" begin
    constraint_keys = [
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "lb"),
        PSI.ConstraintKey(ActivePowerVariableLimitsConstraint, PSY.ThermalStandard, "ub"), PSI.ConstraintKey(CopperPlateBalanceConstraint, PSY.Area),
        sc_key(PostContingencyActivePowerGenerationLimitsConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyActivePowerGenerationLimitsConstraint, "Reserve1_2_ub"),
        sc_key(PostContingencyCopperPlateBalanceConstraint, "Reserve1_1"),
        sc_key(PostContingencyCopperPlateBalanceConstraint, "Reserve1_2"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_1_ub"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_lb"),
        sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_ub"),
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

    monitored_components = vcat(
        collect(get_components(ACTransmission, c_sys)),
        collect(get_components(AreaInterchange, c_sys)),
    )
    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        component = get_component(ThermalStandard, c_sys, component_name)
        reserve_up = get_component(VariableReserve{ReserveUp}, c_sys, reserve_name)
        attach_geometric_outage!(
            c_sys,
            component,
            [reserve_up];
            monitored_components = monitored_components,
        )
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

    moi_tests(ps_model, 552, 0, 504, 312, 192, false)

    opt_container = PSI.get_optimization_container(ps_model)
    copper_plate_constraints =
        PSI.get_constraint(opt_container, CopperPlateBalanceConstraint(), PSY.Area)
    @test size(copper_plate_constraints) == (2, 24)

    psi_checksolve_test(ps_model, [MOI.OPTIMAL], 482055.7647083302, 1)

    results = OptimizationProblemResults(ps_model)
    verify_zone_balance!(results; flow_tol = 0.0)

    compare_outage_power_and_deployed_reserves(c_sys, results, reserve_names)
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
    transition_data = attach_geometric_outage!(sys, alta1, [reserve1])
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
        sc_key(PostContingencyGenerationBalanceConstraint, "Reserve1_1"),
    )
    @test size(cons_resp) == (1, 24)
end

# Regression test for the single-reserve case: when only one SC reserve
# service is in the template, an outage attached to both the outaged
# generator and the reserve must end up in that ServiceModel.outages dict.
@testset "SC reserve outage attachment covers single-reserve case" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)

    alta = get_component(ThermalStandard, sys, "Alta")
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    transition_data = attach_geometric_outage!(sys, alta, [reserve_up])
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

# ----------------------------------------------------------------------------
# Regression tests for the AreaBalancePowerModel post-contingency
# reformulation (PR #1617 review remediation, task 1): the per-area balance
# no longer references the pre-contingency `ActivePowerBalance` expression,
# and inter-area rebalancing flows through a free
# `PostContingencyAreaInterchangeFlowDeviationVariable` (Δf) on every
# `AreaInterchange`. Prior to the fix, areas without the outaged unit had
# their deployment pinned to zero by construction, so these tests fail on
# unmodified code (either with a missing-variable error, since Δf did not
# exist, or because the below assertions on nonzero cross-area deployment
# do not hold).
# ----------------------------------------------------------------------------

# `PostContingencyAreaInterchangeFlowDeviationVariable` is dense over
# (outage_id, area_interchange_name, t), so `read_variable` returns separate
# "name"/"name2" columns (outage, tie) like the reserve deployment variable.
# Filter to one (outage, tie) pair and return the value series sorted by time.
function _sc_flow_deviation_series(
    df::DataFrame,
    outage_id::String,
    monitored_name::String,
)
    rows = filter(x -> x["name"] == outage_id && x["name2"] == monitored_name, df)
    sorted = sort(rows, :DateTime)
    return sorted[!, "value"]
end

# Sum `PostContingencyActivePowerReserveDeploymentVariable` across
# contributing devices for one outage, sorted by time. Shared by the
# cross-area deployment/Δf identity checks below.
function total_deployment_series(df::DataFrame, outage_id::String)
    outage_rows = filter(x -> x["name"] == outage_id, df)
    total_by_t = combine(groupby(outage_rows, :DateTime), :value => sum => :dep)
    sort!(total_by_t, :DateTime)
    return total_by_t.dep
end

# Adds a second, parallel `AreaInterchange` between Area1 and Area2 of a
# `two_area_pjm_DA`-derived system, named "1_2_b" to distinguish it from the
# original "1_2" tie. Shared by the multi-tie G-1 testsets below, which vary
# only in `available`.
function add_parallel_tie!(sys::PSY.System; available::Bool = true)
    area1 = get_component(Area, sys, "Area1")
    area2 = get_component(Area, sys, "Area2")
    tie = AreaInterchange(;
        name = "1_2_b",
        available = available,
        active_power_flow = 0.0,
        from_area = area1,
        to_area = area2,
        flow_limits = (from_to = 1.5, to_from = 1.5),
    )
    add_component!(sys, tie)
    return tie
end

# Shared setup for the AreaBalancePowerModel G-1 testsets below: builds
# `two_area_pjm_DA`, optionally adds the parallel "1_2_b" tie
# (`add_parallel_tie = false` skips it), attaches a `FixedForcedOutage` on
# `outaged_gen_name` to "Reserve1_2" with the caller-supplied monitored
# components (a function of `sys`, since some callers need components created
# inside this setup), and builds the AreaBalancePowerModel template/service
# model/decision model. `set_interchange_model!` registers the
# `AreaInterchange` DeviceModel; callers override it to attach a
# `filter_function` or to omit the DeviceModel entirely.
function _two_area_g1_setup!(;
    add_parallel_tie::Bool = false,
    tie_available::Bool = true,
    monitored::Function,
    use_slacks::Bool = false,
    outaged_gen_name::String = "Brighton_1",
    set_interchange_model!::Function = t ->
        set_device_model!(t, AreaInterchange, StaticBranch),
)
    sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(sys, Hour(24), Hour(1))
    if add_parallel_tie
        add_parallel_tie!(sys; available = tie_available)
    end

    outaged_gen = get_component(ThermalStandard, sys, outaged_gen_name)
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1_2")
    attach_fixed_outage!(
        sys,
        outaged_gen,
        [reserve_up];
        monitored_components = monitored(sys),
    )

    template = get_thermal_dispatch_template_network(NetworkModel(AreaBalancePowerModel))
    set_interchange_model!(template)
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1_2";
            use_slacks = use_slacks,
        ))

    ps_model =
        DecisionModel(template, sys; resolution = Hour(1), optimizer = HiGHS_optimizer)
    return ps_model, sys, reserve_up
end

@testset "G-1 AreaBalancePowerModel: cross-area reserve deployment covers out-of-area outage" begin
    # Outage sited in Area1; the only responding service, Reserve1_2, has
    # contributing devices exclusively in Area2 (see reserve setup in
    # `two_area_pjm_DA`). Deployment must cross the "1_2" tie.
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        monitored = s -> vcat(
            collect(get_components(ACTransmission, s)),
            collect(get_components(AreaInterchange, s)),
        ),
    )
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    res = OptimizationProblemResults(ps_model)
    variables = read_variables(res)

    # The outaged generator must dispatch normally, not be pinned to zero by
    # a stray "areas without the outage get zero deployment" side effect.
    gen_power = filter(
        x -> x["name"] == "Brighton_1",
        variables["ActivePowerVariable__ThermalStandard"],
    )[
        !,
        "value",
    ]
    @test maximum(gen_power) > 1.0

    # Σ Δrsv (Area2 devices) == p_out (Brighton_1), reusing the existing
    # outage/deployment comparison helper.
    compare_outage_power_and_deployed_reserves(sys, res, reserve_up)

    # The monitored tie's flow deviation carries exactly the deployed amount
    # (opposite sign, since Area2 → Area1 support reduces the from(Area1)→
    # to(Area2) flow).
    resolved_outages =
        collect(PSY.get_supplemental_attributes(PSY.UnplannedOutage, reserve_up))
    @test length(resolved_outages) == 1
    outage_id = string(IS.get_uuid(only(resolved_outages)))

    deployment =
        variables["PostContingencyActivePowerReserveDeploymentVariable__VariableReserve__ReserveUp__Reserve1_2"]
    total_deployment = total_deployment_series(deployment, outage_id)

    flow_dev =
        variables["PostContingencyAreaInterchangeFlowDeviationVariable__VariableReserve__ReserveUp__Reserve1_2"]
    tie_flow_dev = _sc_flow_deviation_series(flow_dev, outage_id, "1_2")

    @test all(total_deployment .> 0.0)
    @test maximum(abs.(total_deployment .+ tie_flow_dev)) < 1e-3
end

@testset "G-1 AreaBalancePowerModel: multi-tie area splits response without over-counting" begin
    # Add a second, parallel AreaInterchange between Area1 and Area2 so the
    # post-contingency response can split across two ties.
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        add_parallel_tie = true,
        monitored = s -> vcat(
            collect(get_components(ACTransmission, s)),
            collect(get_components(AreaInterchange, s)),
        ),
    )
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    res = OptimizationProblemResults(ps_model)
    variables = read_variables(res)
    compare_outage_power_and_deployed_reserves(sys, res, reserve_up)

    resolved_outages =
        collect(PSY.get_supplemental_attributes(PSY.UnplannedOutage, reserve_up))
    outage_id = string(IS.get_uuid(only(resolved_outages)))

    deployment =
        variables["PostContingencyActivePowerReserveDeploymentVariable__VariableReserve__ReserveUp__Reserve1_2"]
    total_deployment = total_deployment_series(deployment, outage_id)

    flow_dev =
        variables["PostContingencyAreaInterchangeFlowDeviationVariable__VariableReserve__ReserveUp__Reserve1_2"]
    tie1_flow_dev = _sc_flow_deviation_series(flow_dev, outage_id, "1_2")
    tie2_flow_dev = _sc_flow_deviation_series(flow_dev, outage_id, "1_2_b")

    # Both ties are from(Area1)→to(Area2); the per-area balance requires the
    # *sum* of their deviations to carry the response, not each tie
    # independently reporting the full amount (the pre-fix over-count).
    @test maximum(
        abs.(total_deployment .+ tie1_flow_dev .+ tie2_flow_dev),
    ) < 1e-3
end

@testset "G-1 AreaBalancePowerModel: unmonitored tie has no rate constraint despite dense Δf axis" begin
    # Add a second, available parallel tie but monitor only the original
    # "1_2" tie. Δf must still span both (it is dense over every tie in the
    # AreaInterchange DeviceModel's set), while the rate constraint is
    # strictly opt-in and must cover only the monitored one.
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        add_parallel_tie = true,
        monitored = s -> [get_component(AreaInterchange, s, "1_2")],
    )
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    flow_dev = PSI.get_variable(
        container,
        PSI.PostContingencyAreaInterchangeFlowDeviationVariable(),
        VariableReserve{ReserveUp},
        "Reserve1_2",
    )
    @test Set(axes(flow_dev, 2)) == Set(["1_2", "1_2_b"])

    for suffix in ("lb", "ub")
        # `SparseAxisArray` does not forward `Base.keys` to the container
        # axes, so index its `.data` dict directly to get the actual
        # `(outage_id, name, t)` tuples that were written.
        cons = PSI.get_constraint(
            container,
            sc_key(PostContingencyFlowRateConstraint, "Reserve1_2_$(suffix)"),
        )
        @test Set(k[2] for k in keys(cons.data)) == Set(["1_2"])
    end
end

@testset "G-1 AreaBalancePowerModel: binding monitored-tie limit uses post-contingency flow slack" begin
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        monitored = s -> vcat(
            collect(get_components(ACTransmission, s)),
            collect(get_components(AreaInterchange, s)),
        ),
        use_slacks = true,
    )

    # Tighten the "1_2" tie well below the cross-area response so the
    # post-contingency flow-rate constraint binds.
    tie = get_component(AreaInterchange, sys, "1_2")
    set_flow_limits!(tie, (from_to = 0.1, to_from = 0.1))

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    res = OptimizationProblemResults(ps_model)
    variables = read_variables(res)

    # The balance still holds exactly even though the tie's physical limit
    # cannot accommodate the full response: Δf is decoupled from the
    # flow-rate constraint by the slack.
    compare_outage_power_and_deployed_reserves(sys, res, reserve_up)

    slack_lb =
        variables["PostContingencyFlowActivePowerSlackLowerBound__VariableReserve__ReserveUp__Reserve1_2"]
    @test maximum(slack_lb[!, "value"]) > 1e-3
end

@testset "G-1 AreaBalancePowerModel: unavailable AreaInterchange excluded from Δf axis" begin
    # Add a second, parallel AreaInterchange between Area1 and Area2, then
    # disable it. The original "1_2" tie stays available, so it remains the
    # sole path for cross-area response. Monitor only available
    # AreaInterchanges: a monitored-but-unavailable tie is a separate error
    # path (`_resolve_service_monitored_area_interchanges`), not what this
    # test targets.
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        add_parallel_tie = true,
        tie_available = false,
        monitored = s -> vcat(
            collect(get_components(ACTransmission, s)),
            collect(get_components(PSY.get_available, AreaInterchange, s)),
        ),
    )
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    flow_deviation_var = PSI.get_variable(
        container,
        PSI.PostContingencyAreaInterchangeFlowDeviationVariable(),
        VariableReserve{ReserveUp},
        "Reserve1_2",
    )
    interchange_axis = axes(flow_deviation_var, 2)
    @test "1_2" in interchange_axis
    @test !("1_2_b" in interchange_axis)

    # Matches pre-contingency semantics: the disabled tie is not the sole
    # path here (the original "1_2" tie stays available), so the model
    # remains solvable and the response still balances exactly.
    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    res = OptimizationProblemResults(ps_model)
    compare_outage_power_and_deployed_reserves(sys, res, reserve_up)
end

@testset "G-1 AreaBalancePowerModel: unavailable monitored AreaInterchange rejected at validation" begin
    # Monitor every AreaInterchange, including the disabled one: an admissible
    # tie must be both available and in the network model's scope.
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        add_parallel_tie = true,
        tie_available = false,
        monitored = s -> collect(get_components(AreaInterchange, s)),
    )
    template = PSI.get_template(ps_model)

    # `build!` catches this internally and reports `ModelBuildStatus.FAILED`
    # rather than propagating it, so the throw is only observable by calling
    # the template-validation step directly (mirrors the `ReserveDown`
    # rejection testset below).
    @test_throws IS.ConflictingInputsError PSI._build_service_model_outages!(
        template,
        sys,
    )

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.FAILED
end

# The AreaInterchange `DeviceModel`'s set — not the network model's scope — is
# what Δf must span: a tie excluded by that DeviceModel's `filter_function`
# has no pre-contingency `FlowActivePowerVariable`, so a Δf term on it would
# be an unanchored transfer path in the per-area balance (and the monitored
# post-contingency flow expression would hit a `KeyError`).
#
# Both testsets below build under `AreaBalancePowerModel`, so
# `ignores_branch_filtering(AreaBalancePowerModel) = true` makes
# `_validate_branch_models` log a stray "Branch filtering is ignored for
# network model AreaBalancePowerModel" warning during `build!`. That warning
# is inaccurate for this DeviceModel: filtering IS honored here (Δf's axis
# comes straight from the filtered `FlowActivePowerVariable__AreaInterchange`
# container, asserted below). Do not "fix" these tests by removing
# `filter_function` — the warning is a known false positive in the unrelated
# branch-filtering validation path, not a signal that filtering was ignored.
_exclude_parallel_tie_model!(template::PSI.ProblemTemplate) = set_device_model!(
    template,
    DeviceModel(
        AreaInterchange,
        StaticBranch;
        attributes = Dict("filter_function" => x -> get_name(x) != "1_2_b"),
    ),
)

@testset "G-1 AreaBalancePowerModel: filter_function-excluded tie gets no Δf term" begin
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        add_parallel_tie = true,
        monitored = s -> [get_component(AreaInterchange, s, "1_2")],
        set_interchange_model! = _exclude_parallel_tie_model!,
    )
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    flow_var =
        PSI.get_variable(container, PSI.FlowActivePowerVariable(), AreaInterchange)
    flow_deviation_var = PSI.get_variable(
        container,
        PSI.PostContingencyAreaInterchangeFlowDeviationVariable(),
        VariableReserve{ReserveUp},
        "Reserve1_2",
    )
    # Δf spans exactly the pre-contingency flow variable's ties.
    @test Set(axes(flow_var, 1)) == Set(["1_2"])
    @test Set(axes(flow_deviation_var, 2)) == Set(["1_2"])

    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    res = OptimizationProblemResults(ps_model)
    variables = read_variables(res)
    compare_outage_power_and_deployed_reserves(sys, res, reserve_up)

    outage_id = string(
        IS.get_uuid(
            only(collect(PSY.get_supplemental_attributes(PSY.UnplannedOutage, reserve_up))),
        ),
    )
    total_deployment = total_deployment_series(
        variables["PostContingencyActivePowerReserveDeploymentVariable__VariableReserve__ReserveUp__Reserve1_2"],
        outage_id,
    )
    tie_flow_dev = _sc_flow_deviation_series(
        variables["PostContingencyAreaInterchangeFlowDeviationVariable__VariableReserve__ReserveUp__Reserve1_2"],
        outage_id,
        "1_2",
    )
    # Per-area balance residual: the single modeled tie carries the whole
    # cross-area response, with no free deviation on the excluded tie.
    @test all(total_deployment .> 0.0)
    @test maximum(abs.(total_deployment .+ tie_flow_dev)) < 1e-3
end

@testset "G-1 AreaBalancePowerModel: monitored tie excluded by filter_function rejected at validation" begin
    # Both ties are available and in the network model's scope, so only the
    # DeviceModel-inclusion check can reject the monitored "1_2_b".
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        add_parallel_tie = true,
        monitored = s -> collect(get_components(AreaInterchange, s)),
        set_interchange_model! = _exclude_parallel_tie_model!,
    )
    template = PSI.get_template(ps_model)

    @test_throws IS.ConflictingInputsError PSI._build_service_model_outages!(
        template,
        sys,
    )

    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.FAILED
end

@testset "G-1 AreaBalancePowerModel: no AreaInterchange device model warns and covers in-area" begin
    # No AreaInterchange DeviceModel means no modeled tie at all: the per-area
    # post-contingency balance reduces to in-area coverage, so the outage is
    # sited in Area2 where Reserve1_2's contributing devices live.
    ps_model, sys, reserve_up = _two_area_g1_setup!(;
        outaged_gen_name = "Solitude_2",
        monitored = s -> collect(get_components(Line, s)),
        set_interchange_model! = t -> nothing,
    )
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    @test !PSI.has_container_key(
        container,
        PSI.FlowActivePowerVariable,
        AreaInterchange,
    )
    # No modeled tie, so no Δf container at all: the per-area balance reduces
    # to in-area coverage.
    @test !PSI.has_container_key(
        container,
        PSI.PostContingencyAreaInterchangeFlowDeviationVariable,
        VariableReserve{ReserveUp},
        "Reserve1_2",
    )

    # `build!` installs its own logger, so the construct-time warning is only
    # observable in the model's log file; it must be emitted exactly once.
    log_text = read(PSI.get_log_file(ps_model), String)
    @test length(
        collect(eachmatch(r"no PSY\.AreaInterchange device model in the", log_text)),
    ) == 1

    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    res = OptimizationProblemResults(ps_model)
    compare_outage_power_and_deployed_reserves(sys, res, reserve_up)
end

@testset "SecurityConstrainedContingencyReserve rejects Reserve{ReserveDown}" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveDown},
            SecurityConstrainedContingencyReserve,
            "Reserve2",
        ))

    # `build!` catches this internally and reports `ModelBuildStatus.FAILED`
    # rather than propagating it (see `build_impl!`), so the throw is only
    # observable by calling the template-validation step directly — mirrors
    # the sibling AC-transmission test file's convention for internal
    # validation checks.
    @test_throws IS.ConflictingInputsError PSI._build_service_model_outages!(
        template,
        sys,
    )
end

@testset "SC guard rails: fail-fast on unresolved monitored components" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    template = get_thermal_dispatch_template_network(PTDFPowerModel)
    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    network_model = PSI.get_network_model(PSI.get_template(ps_model))
    net_reduction_data = network_model.network_reduction
    uuid = IS.get_uuid(sys)

    # (a) device-side: an ACTransmission-type monitor absent from the
    # network-reduction's branch-arc maps (this system has no MonitoredLine
    # components) raises, rather than being silently dropped.
    device_model = DeviceModel(MonitoredLine, SecurityConstrainedStaticBranch)
    device_model.outages[uuid] =
        Dict{DataType, Set{String}}(MonitoredLine => Set(["fake_line"]))
    @test_throws ErrorException PSI._resolve_monitored_arcs(
        device_model,
        net_reduction_data,
    )

    # (a) service-side counterpart.
    service_model = ServiceModel(
        VariableReserve{ReserveUp},
        SecurityConstrainedContingencyReserve,
        "Reserve1",
    )
    service_model.outages[uuid] =
        Dict{DataType, Set{String}}(MonitoredLine => Set(["fake_line"]))
    @test_throws ErrorException PSI._resolve_service_monitored_arcs(
        service_model,
        net_reduction_data,
    )

    # (b) AreaInterchange monitors have no branch-arc entry under a PTDF
    # network model; dropped with a warning naming the component instead of
    # being silently skipped.
    service_model_ai = ServiceModel(
        VariableReserve{ReserveUp},
        SecurityConstrainedContingencyReserve,
        "Reserve1",
    )
    service_model_ai.outages[uuid] =
        Dict{DataType, Set{String}}(AreaInterchange => Set(["tie1"]))
    @test_logs (:warn, r"AreaInterchange monitor.*tie1.*dropped") PSI._resolve_service_monitored_arcs(
        service_model_ai,
        net_reduction_data,
    )

    # (c) an AreaInterchange monitor naming a component absent from the
    # system raises, rather than being silently skipped.
    service_model_missing = ServiceModel(
        VariableReserve{ReserveUp},
        SecurityConstrainedContingencyReserve,
        "Reserve1",
    )
    service_model_missing.outages[uuid] =
        Dict{DataType, Set{String}}(AreaInterchange => Set(["not_a_real_tie"]))
    @test_throws ErrorException PSI._resolve_service_monitored_area_interchanges(
        sys,
        service_model_missing,
    )
end

# Regression for A-1 (PR #1617 review remediation, simplify pass 2):
# `_monitored_components_by_modeled_type` admits `PSY.AreaInterchange` for the
# service consumer only. On the device-side path an AreaInterchange monitor
# must be reported via `_warn_uncovered_monitored_types` and excluded from
# `device_model.outages`, not silently dropped later inside
# `_resolve_monitored_arcs`.
@testset "Device-side AreaInterchange monitor is reported, not silently dropped" begin
    sys = PSB.build_system(PSISystems, "two_area_pjm_DA"; add_reserves = true)
    transform_single_time_series!(sys, Hour(24), Hour(1))

    line = first(collect(get_components(Line, sys)))
    tie = get_component(AreaInterchange, sys, "1_2")
    attach_fixed_outage!(
        sys,
        line,
        PSY.Service[];
        monitored_components = [line, tie],
    )

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
    )
    set_device_model!(template, Line, SecurityConstrainedStaticBranch)
    set_device_model!(template, AreaInterchange, StaticBranch)

    @test_logs (
        :warn,
        r"AreaInterchange.*is not a modeled ACTransmission branch type",
    ) PSI._build_device_model_outages!(template, sys)

    device_model = only(PSI._sc_branch_models(template))
    outage_uuid = only(keys(device_model.outages))
    per_type = device_model.outages[outage_uuid]
    @test haskey(per_type, Line)
    @test !haskey(per_type, AreaInterchange)
end

@testset "PostContingencyBranchFlow applies PTDF orientation sign for series-reduced monitored branch" begin
    # Regression for M4: `RADIAL1-RADIAL2-i_1` is a `:ToFrom`-oriented member
    # of a degree-two series reduction (orientation_sign == -1.0) — mirrors
    # the `case10_radial_series_reductions` fixture pattern used for the
    # equivalent device-side check in
    # test_ac_transmission_security_constrained_models.jl.
    sys_red = PSB.build_system(PSITestSystems, "case10_radial_series_reductions")

    dummy_forecast = Deterministic(
        "max_active_power",
        Dict(DateTime("2020-01-01T00:00:00") => ones(24)),
        Hour(1),
    )
    load = first(collect(get_components(StandardLoad, sys_red)))
    add_time_series!(sys_red, load, dummy_forecast)

    gens = collect(get_components(ThermalStandard, sys_red))
    add_service!(sys_red,
        VariableReserve{ReserveUp}(;
            name = "Reserve1",
            available = true,
            time_frame = 0.0,
            requirement = 0.0,
            sustained_time = 3600,
            max_output_fraction = 1.0,
            max_participation_factor = 0.25,
            deployed_fraction = 0.0,
        ),
        gens)
    reserve_up = get_component(VariableReserve{ReserveUp}, sys_red, "Reserve1")

    outaged_gen = gens[1]
    monitored_name = "RADIAL1-RADIAL2-i_1"
    monitored_line = get_component(Line, sys_red, monitored_name)
    outage = attach_fixed_outage!(
        sys_red,
        outaged_gen,
        [reserve_up];
        monitored_components = [monitored_line],
    )

    ptdf_red = PTDF(sys_red; network_reductions = NetworkReduction[DegreeTwoReduction()])
    template = ProblemTemplate(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = ptdf_red,
            reduce_degree_two_branches = true,
        ),
    )
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, StandardLoad, StaticPowerLoad)
    set_device_model!(template, Line, StaticBranch)
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1",
        ))

    ps_model = DecisionModel(template, sys_red; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    network_model = PSI.get_network_model(PSI.get_template(ps_model))
    net_reduction_data = network_model.network_reduction
    orientation_sign =
        PSI.get_ptdf_orientation_sign(net_reduction_data, Line, monitored_name)
    @test orientation_sign == -1.0   # guard: fixture must exercise the :ToFrom member

    time_steps = PSI.get_time_steps(container)
    outage_id = string(IS.get_uuid(outage))
    actual_container = PSI.get_expression(
        container,
        PSI.PostContingencyBranchFlow(),
        VariableReserve{ReserveUp},
        "Reserve1",
    )
    pre_flow = PSI.get_expression(container, PSI.PTDFBranchFlow(), Line)
    nodal_deployment = PSI.get_expression(
        container,
        PSI.PostContingencyNodalActivePowerDeployment(),
        VariableReserve{ReserveUp},
        "Reserve1",
    )
    # `nodal_deployment`'s bus axis is scoped to injection-relevant buses
    # (contributing-device + outaged-generator buses), not the full PTDF bus
    # axis — buses outside that subset always carry a zero deployment
    # expression, so they're skipped here rather than indexed positionally.
    name_to_arc_maps = PNM.get_name_to_arc_maps(net_reduction_data)
    arc, _ = name_to_arc_maps[Line][monitored_name]
    ptdf_col = PSI.get_PTDF_matrix(network_model)[arc, :]
    full_bus_axis = PNM.get_bus_axis(PSI.get_PTDF_matrix(network_model))
    relevant_buses = Set(nodal_deployment.axes[2])

    for t in time_steps
        actual = actual_container[outage_id, monitored_name, t]
        correctly_signed = copy(pre_flow[monitored_name, t])
        naively_signed = copy(pre_flow[monitored_name, t])
        for b in eachindex(ptdf_col)
            coef = ptdf_col[b]
            abs(coef) < PSI.PTDF_ZERO_TOL && continue
            bus_number = full_bus_axis[b]
            bus_number in relevant_buses || continue
            JuMP.add_to_expression!(
                correctly_signed, orientation_sign * coef,
                nodal_deployment[outage_id, bus_number, t],
            )
            JuMP.add_to_expression!(
                naively_signed, coef, nodal_deployment[outage_id, bus_number, t],
            )
        end
        @test aff_exprs_approx_equal(actual, correctly_signed; atol = 1e-8)
        @test !aff_exprs_approx_equal(actual, naively_signed; atol = 1e-8)
    end
end

@testset "PostContingencyActivePowerGenerationLimitsConstraint has no lower bound (UC, no-requirement path)" begin
    # Regression for M5: for an up-reserve formulation, `p + Δ >= limits.min`
    # is redundant when the unit is committed (`p >= limits.min * u`, `Δ >=
    # 0`) and actively wrong when it is not (`u == 0` forces `p == 0`, so the
    # old lower bound demanded a positive `Δ` from an offline unit just to
    # reach `limits.min`). Fixing the offline unit's on-status AND its
    # deployment to zero must remain feasible: under the dropped lower bound
    # this is `0 >= limits.min`, which is false whenever `limits.min > 0` —
    # provably infeasible under the pre-fix constraint.
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    remove_time_series!(sys, Deterministic, reserve_up, "requirement")

    outaged_gen = get_component(ThermalStandard, sys, "Alta")
    offline_gen = get_component(ThermalStandard, sys, "Park City")
    @test PSY.get_active_power_limits(offline_gen).min > 0.0   # guard: fix-to-zero must bind the old lb

    outage = attach_fixed_outage!(
        sys,
        outaged_gen,
        [reserve_up];
        monitored_components = collect(get_components(ACTransmission, sys)),
    )

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
    )
    set_device_model!(template, ThermalStandard, ThermalStandardUnitCommitment)
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1",
        ))

    ps_model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    on_var = PSI.get_variable(container, OnVariable(), ThermalStandard)
    deployment_var = PSI.get_variable(
        container,
        PostContingencyActivePowerReserveDeploymentVariable(),
        VariableReserve{ReserveUp},
        "Reserve1",
    )
    outage_id = string(IS.get_uuid(outage))
    for t in PSI.get_time_steps(container)
        JuMP.fix(on_var["Park City", t], 0.0; force = true)
        JuMP.fix(deployment_var[outage_id, "Park City", t], 0.0; force = true)
    end

    @test solve!(ps_model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "has_requirement_ts is gated on the mapped requirement series, not any time series" begin
    # Regression for M12: a service carrying only an unrelated time series
    # (not the name mapped to `RequirementTimeSeriesParameter`) must take the
    # no-requirement path — no `RequirementConstraint` should be built.
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc"; add_reserves = true)
    reserve_up = get_component(VariableReserve{ReserveUp}, sys, "Reserve1")
    remove_time_series!(sys, Deterministic, reserve_up, "requirement")

    unrelated_forecast = Deterministic(
        "unrelated_series",
        Dict(DateTime("2020-01-01T00:00:00") => ones(24)),
        Hour(1),
    )
    add_time_series!(sys, reserve_up, unrelated_forecast)

    outaged_gen = get_component(ThermalStandard, sys, "Alta")
    attach_fixed_outage!(
        sys,
        outaged_gen,
        [reserve_up];
        monitored_components = collect(get_components(ACTransmission, sys)),
    )

    template = get_thermal_dispatch_template_network(
        NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
    )
    set_service_model!(template,
        ServiceModel(
            VariableReserve{ReserveUp},
            SecurityConstrainedContingencyReserve,
            "Reserve1",
        ))

    ps_model =
        DecisionModel(template, sys; optimizer = HiGHS_optimizer, interval = Hour(24))
    @test build!(ps_model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    container = PSI.get_optimization_container(ps_model)
    @test !PSI.has_container_key(
        container,
        RequirementConstraint,
        VariableReserve{ReserveUp},
    )
end
