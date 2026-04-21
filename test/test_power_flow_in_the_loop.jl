@testset "AC Power Flow in the loop for PhaseShiftingTransformer" begin
    system = build_system(PSITestSystems, "c_sys5_uc")

    line = get_component(Line, system, "1")
    arc = get_arc(line)
    remove_component!(system, line)

    ps = PhaseShiftingTransformer(;
        name = get_name(line),
        available = true,
        active_power_flow = 0.0,
        reactive_power_flow = 0.0,
        r = get_r(line),
        x = get_x(line),
        primary_shunt = 0.0,
        tap = 1.0,
        α = 0.0,
        rating = get_rating(line),
        arc = arc,
        base_power = get_base_power(system),
    )

    add_component!(system, ps)

    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(system),
            power_flow_evaluation = ACPowerFlow(),
        ),
    )
    set_device_model!(template, DeviceModel(PhaseShiftingTransformer, PhaseAngleControl))
    model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT

    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results = OptimizationProblemResults(model_m)
    vd = read_variables(results)

    data = PSI.get_power_flow_data(
        only(PSI.get_power_flow_evaluation_data(PSI.get_optimization_container(model_m))),
    )
    base_power = get_base_power(system)
    phase_results = vd["FlowActivePowerVariable__PhaseShiftingTransformer"]

    # cannot easily test for the "from" bus because of the generators "Park City" and "Alta"
    bus_lookup = PFS.get_bus_lookup(data)
    @test isapprox(
        data.bus_active_power_injections[bus_lookup[get_number(get_to(arc))], :] *
        base_power,
        filter(row -> row[:name] == get_name(line), phase_results)[!, :value],
        atol = 1e-9,
        rtol = 0,
    )
end

@testset "AC Power Flow in the loop with parallel lines" begin
    original_line_flow, parallel_line_flow = zero(ComplexF64), zero(ComplexF64)
    for replace_line in (true, false)
        system = build_system(PSITestSystems, "c_sys5_uc")

        line = get_component(Line, system, "1")
        # split line into 2 parallel lines.
        if replace_line
            original_impedance = get_r(line) + im * get_x(line)
            original_shunt = get_b(line)
            remove_component!(system, line)
            split_impedance = original_impedance * 2
            split_shunt = (from = 0.5 * original_shunt.from, to = 0.5 * original_shunt.to)
            for i in 1:2
                l = Line(;
                    name = get_name(line) * "_$i",
                    available = true,
                    active_power_flow = 0.0,
                    reactive_power_flow = 0.0,
                    arc = get_arc(line),
                    r = real(split_impedance),
                    x = imag(split_impedance),
                    b = split_shunt,
                    angle_limits = get_angle_limits(line),
                    rating = get_rating(line),
                )
                add_component!(system, l)
            end
        end
        template = get_template_dispatch_with_network(
            NetworkModel(
                PTDFPowerModel;
                PTDF_matrix = PTDF(system),
                power_flow_evaluation = ACPowerFlow(),
            ),
        )
        model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT

        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
        results = OptimizationProblemResults(model_m)
        vd = read_aux_variables(results)
        active_power_ft = vd["PowerFlowBranchActivePowerFromTo__Line"]
        reactive_power_ft = vd["PowerFlowBranchReactivePowerFromTo__Line"]
        if replace_line
            name = "$(get_name(line))_1"
            parallel_line_flow =
                filter(row -> row[:name] == name, active_power_ft)[1, :value][1] +
                im * filter(row -> row[:name] == name, reactive_power_ft)[1, :value][1]
        else
            name = get_name(line)
            original_line_flow =
                filter(row -> row[:name] == name, active_power_ft)[1, :value][1] +
                im * filter(row -> row[:name] == name, reactive_power_ft)[1, :value][1]
        end
    end

    @test isapprox(
        2 * parallel_line_flow,
        original_line_flow,
        atol = 1e-3,
    )
end

@testset "AC Power Flow in the loop with a breaker-switch" begin
    system = build_system(PSITestSystems, "c_sys5_uc")
    # we choose a line to replace such that the arc lookup of a different line changes.
    line = get_component(Line, system, "2")
    remove_component!(system, line)
    bs = PSY.DiscreteControlledACBranch(
        ;
        name = get_name(line),
        available = true,
        active_power_flow = 0.0,
        reactive_power_flow = 0.0,
        arc = get_arc(line),
        r = 0.0,
        x = 0.0,
        rating = get_rating(line),
        discrete_branch_type = PSY.DiscreteControlledBranchType.BREAKER,
        branch_status = PSY.DiscreteControlledBranchStatus.CLOSED,
    )
    add_component!(system, bs)
    # these lines end up being parallel, so we set their impedances to be the same
    line3 = get_component(Line, system, "3")
    line6 = get_component(Line, system, "6")
    PSY.set_r!(line3, PSY.get_r(line6))
    PSY.set_x!(line3, PSY.get_x(line6))
    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(system),
            power_flow_evaluation = ACPowerFlow(),
        ),
    )
    model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)

    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    # the interface currently doesn't allow for power flow in-the-loop on networks with
    # reductions. we'd have to pass kwargs all the way down to add_power_flow_data!.
end

# already has a TwoTerminalGenericHVDCLine
replace_hvdc!(::PSY.System, ::Type{TwoTerminalGenericHVDCLine}) = nothing

function replace_hvdc!(sys::PSY.System, ::Type{TwoTerminalVSCLine})
    # required fields for constructor:
    # name, available, arc, active_power_flow, rating, active_power_limits_from,
    # active_power_limits_to
    old_hvdc = only(get_components(TwoTerminalGenericHVDCLine, sys))
    remove_component!(sys, old_hvdc)
    hvdc = TwoTerminalVSCLine(;
        name = get_name(old_hvdc),
        available = true,
        arc = get_arc(old_hvdc),
        active_power_flow = get_active_power_flow(old_hvdc),
        rating = 100.0, # arbitrary
        active_power_limits_from = get_active_power_limits_from(old_hvdc),
        active_power_limits_to = get_active_power_limits_to(old_hvdc),
    )
    add_component!(sys, hvdc)
end

function replace_hvdc!(sys::PSY.System, ::Type{TwoTerminalLCCLine})
    # required fields for constructor (yikes):
    # name, available, arc, active_power_flow, r, transfer_setpoint, scheduled_dc_voltage,
    # rectifier_bridges, rectifier_delay_angle_limits, rectifier_rc, rectifier_xc, 
    #rectifier_base_voltage, inverter_bridges, inverter_extinction_angle_limits, 
    # inverter_rc, inverter_xc, inverter_base_voltage, 
    old_hvdc = only(get_components(TwoTerminalGenericHVDCLine, sys))
    remove_component!(sys, old_hvdc)
    # hand-tuned parameters.
    r = 0.01
    xr = 0.01
    xi = 0.01
    hvdc = TwoTerminalLCCLine(;
        name = get_name(old_hvdc),
        available = true,
        arc = get_arc(old_hvdc),
        active_power_flow = get_active_power_flow(old_hvdc),
        r = r,
        transfer_setpoint = 50,
        scheduled_dc_voltage = 200.0,
        rectifier_bridges = 1,
        rectifier_delay_angle_limits = (min = 0.0, max = π / 2),
        rectifier_rc = 0.0,
        rectifier_xc = xr,
        rectifier_base_voltage = 100.0,
        inverter_bridges = 1,
        inverter_extinction_angle_limits = (min = 0, max = π / 2),
        inverter_rc = 0.0,
        inverter_xc = xi,
        inverter_base_voltage = 100.0,
        # rest are optional.
        #=power_mode = true,
        switch_mode_voltage = 0.0,
        compounding_resistance = 0.0,
        min_compounding_voltage = 0.0,
        rectifier_transformer_ratio = 1.0,
        rectifier_tap_setting = 1.0,
        rectifier_tap_limits = (min = 0.5, max = 1.5),
        rectifier_tap_step = 0.05,
        rectifier_delay_angle = 0.01,
        rectifier_capacitor_reactance = 0.0,
        inverter_transformer_ratio = 1.0,
        inverter_tap_setting = 1.0,
        inverter_tap_limits = (min = 0.5, max = 1.5),
        inverter_tap_step = 0.05,
        inverter_extinction_angle = 0.0,
        inverter_capacitor_reactance = 0.0,
        active_power_limits_from = (min = 0.0, max = 0.0),
        active_power_limits_to = (min = 0.0, max = 0.0),
        reactive_power_limits_from = (min = 0.0, max = 0.0),
        reactive_power_limits_to = (min = 0.0, max = 0.0),=#
    )
    add_component!(sys, hvdc)
end

@testset "HVDCs with DC PF in the loop" begin
    for hvdc_type in (TwoTerminalGenericHVDCLine, TwoTerminalLCCLine, TwoTerminalVSCLine)
        sys = build_system(PSISystems, "2Area 5 Bus System")
        replace_hvdc!(sys, hvdc_type)

        template_uc =
            ProblemTemplate(
                NetworkModel(PTDFPowerModel; power_flow_evaluation = DCPowerFlow()),
            )

        set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
        set_device_model!(template_uc, RenewableDispatch, RenewableFullDispatch)
        set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
        set_device_model!(template_uc, DeviceModel(Line, StaticBranch))

        if hvdc_type == TwoTerminalVSCLine
            set_device_model!(
                template_uc,
                # regardless of formulation, PowerFlows.jl always takes losses into account...
                DeviceModel(hvdc_type, HVDCTwoTerminalLossless),
            )
        else
            set_device_model!(
                template_uc,
                DeviceModel(hvdc_type, HVDCTwoTerminalDispatch),
            )
        end

        model = DecisionModel(template_uc, sys; name = "UC", optimizer = HiGHS_optimizer)

        @test build!(model; output_dir = mktempdir()) == PSI.ModelBuildStatus.BUILT
        @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    end
end

@testset "LCC HVDC with AC PF in the loop" begin
    sys5 = build_system(PSISystems, "2Area 5 Bus System")
    hvdc = first(get_components(TwoTerminalGenericHVDCLine, sys5))
    lcc = TwoTerminalLCCLine(;
        name = "lcc",
        available = true,
        arc = hvdc.arc,
        active_power_flow = 0.1,
        r = 0.000189,
        transfer_setpoint = -100.0,
        scheduled_dc_voltage = 7.5,
        rectifier_bridges = 2,
        rectifier_delay_angle_limits = (min = 0.31590, max = 1.570),
        rectifier_rc = 2.6465e-5,
        rectifier_xc = 0.001092,
        rectifier_base_voltage = 230.0,
        inverter_bridges = 2,
        inverter_extinction_angle_limits = (min = 0.3037, max = 1.57076),
        inverter_rc = 2.6465e-5,
        inverter_xc = 0.001072,
        inverter_base_voltage = 230.0,
        power_mode = true,
        switch_mode_voltage = 0.0,
        compounding_resistance = 0.0,
        min_compounding_voltage = 0.0,
        rectifier_transformer_ratio = 0.09772,
        rectifier_tap_setting = 1.0,
        rectifier_tap_limits = (min = 1, max = 1),
        rectifier_tap_step = 0.00624,
        rectifier_delay_angle = 0.31590,
        rectifier_capacitor_reactance = 0.1,
        inverter_transformer_ratio = 0.07134,
        inverter_tap_setting = 1.0,
        inverter_tap_limits = (min = 1, max = 1),
        inverter_tap_step = 0.00625,
        inverter_extinction_angle = 0.31416,
        inverter_capacitor_reactance = 0.0,
        active_power_limits_from = (min = -3.0, max = 3.0),
        active_power_limits_to = (min = -3.0, max = 3.0),
        reactive_power_limits_from = (min = -3.0, max = 3.0),
        reactive_power_limits_to = (min = -3.0, max = 3.0),
    )

    add_component!(sys5, lcc)
    remove_component!(sys5, hvdc)

    template = get_thermal_dispatch_template_network(
        NetworkModel(
            ACPPowerModel;
            use_slacks = false,
            power_flow_evaluation = ACPowerFlow(),
        ),
    )

    set_device_model!(template, TwoTerminalLCCLine, PSI.HVDCTwoTerminalLCC)
    set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)

    model = DecisionModel(
        template,
        sys5;
        optimizer = optimizer_with_attributes(Ipopt.Optimizer),
        horizon = Hour(2),
    )
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
end

@testset "generic HVDC with AC PF in the loop" begin
    # TODO replace RTS with something smaller, so this test case doesn't take so long.
    sys = build_system(PSISystems, "RTS_GMLC_DA_sys")

    hvdc = only(get_components(TwoTerminalGenericHVDCLine, sys))
    from = get_from(get_arc(hvdc))
    to = get_to(get_arc(hvdc))

    # remove components that impact total bus power at the HVDC line buses.
    components = collect(
        get_components(
            x -> get_number(get_bus(x)) ∈ (get_number(from), get_number(to)),
            StaticInjection,
            sys,
        ),
    )
    foreach(x -> remove_component!(sys, x), components)
    change_to_PQ = ["Chifa", "Arne"]
    for bus_name in change_to_PQ
        bus = get_component(PSY.ACBus, sys, bus_name)
        @assert !isnothing(bus) "bus does not exist"
        set_bustype!(bus, PSY.ACBusTypes.PQ)
    end

    set_bustype!(get_component(ACBus, sys, "Arthur"), ACBusTypes.REF)

    template_uc =
        ProblemTemplate(NetworkModel(PTDFPowerModel; power_flow_evaluation = ACPowerFlow()))

    set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template_uc, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
    set_device_model!(template_uc, DeviceModel(Line, StaticBranch))
    set_device_model!(
        template_uc,
        DeviceModel(TwoTerminalGenericHVDCLine, HVDCTwoTerminalDispatch),
    )

    model = DecisionModel(template_uc, sys; name = "UC", optimizer = HiGHS_optimizer)

    @test build!(model; output_dir = mktempdir()) == PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results = OptimizationProblemResults(model)
    vd = read_variables(results)
    ad = read_aux_variables(results)

    data = PSI.get_power_flow_data(
        only(PSI.get_power_flow_evaluation_data(PSI.get_optimization_container(model))),
    )
    base_power = get_base_power(sys)

    # test that the power flow results for the HVDC buses match the HVDC power transfer from the simulation
    bus_lookup = PFS.get_bus_lookup(data)

    from_to = vd["FlowActivePowerFromToVariable__TwoTerminalGenericHVDCLine"][:, :value]
    to_from = vd["FlowActivePowerToFromVariable__TwoTerminalGenericHVDCLine"][:, :value]

    @test isapprox(
        data.bus_active_power_injections[bus_lookup[get_number(from)], :] * base_power,
        -1 .* from_to,
        atol = 1e-9,
        rtol = 0,
    )
    # verify the line loss curve is exactly 10% so the loss-ratio check below is meaningful
    hvdc_loss_curve = get_loss(hvdc)
    @assert hvdc_loss_curve isa PSY.LinearCurve
    @assert get_proportional_term(hvdc_loss_curve) == 0.1
    @assert get_constant_term(hvdc_loss_curve) == 0.0
    nonzeros = (abs.(from_to) .> 1e-9) .| (abs.(to_from) .> 1e-9)
    loss_ratios = (from_to .+ to_from) ./ maximum.(zip(abs.(from_to), abs.(to_from)))
    ten_percent_loss = abs.(loss_ratios .- 0.1) .< 1e-9
    @test all(ten_percent_loss[nonzeros])

    @test isapprox(
        data.bus_active_power_injections[bus_lookup[get_number(to)], :] * base_power,
        -1 .* to_from,
        atol = 1e-9,
        rtol = 0,
    )
end

@testset "Test AC power flow in the loop: small system UCED, PSS/E export" for calculate_loss_factors in
                                                                               (true, false)
    for calculate_voltage_stability_factors in (true, false)
        file_path = mktempdir(; cleanup = true)
        export_path = mktempdir(; cleanup = true)
        pf_path = mktempdir(; cleanup = true)
        c_sys5_hy_uc = PSB.build_system(PSITestSystems, "c_sys5_hy_uc")
        c_sys5_hy_ed = PSB.build_system(PSITestSystems, "c_sys5_hy_ed")
        sim = run_simulation(
            c_sys5_hy_uc,
            c_sys5_hy_ed,
            file_path,
            export_path;
            ed_network_model = NetworkModel(
                CopperPlatePowerModel;
                duals = [CopperPlateBalanceConstraint],
                use_slacks = true,
                power_flow_evaluation =
                ACPowerFlow(;
                    exporter = PSSEExportPowerFlow(:v33, pf_path; write_comments = true),
                    calculate_loss_factors = calculate_loss_factors,
                    calculate_voltage_stability_factors = calculate_voltage_stability_factors,
                ),
            ),
        )
        results = SimulationResults(sim)
        results_ed = get_decision_problem_results(results, "ED")
        thermal_results = first(
            values(
                PSI.read_results_with_keys(results_ed,
                    [PSI.VariableKey(ActivePowerVariable, ThermalStandard)]),
            ),
        )
        min_time = minimum(thermal_results.DateTime)
        max_time = maximum(thermal_results.DateTime)
        first_result = filter(row -> row[:DateTime] == min_time, thermal_results)
        last_result = filter(row -> row[:DateTime] == max_time, thermal_results)

        available_aux_variables = list_aux_variable_keys(results_ed)
        loss_factors_aux_var_key = PSI.AuxVarKey(PowerFlowLossFactors, ACBus)
        voltage_stability_aux_var_key =
            PSI.AuxVarKey(PowerFlowVoltageStabilityFactors, ACBus)

        # here we check if the loss factors are stored in the results, the values are tested in PowerFlows.jl
        if calculate_loss_factors
            @test loss_factors_aux_var_key ∈ available_aux_variables
            loss_factors = first(
                values(
                    PSI.read_results_with_keys(results_ed,
                        [loss_factors_aux_var_key]),
                ),
            )
            @test !isnothing(loss_factors)
            # count distinct time periods
            @test length(unique(loss_factors.DateTime)) == 48 * 12
        else
            @test loss_factors_aux_var_key ∉ available_aux_variables
        end

        if calculate_voltage_stability_factors
            @test voltage_stability_aux_var_key ∈ available_aux_variables
            voltage_stability = first(
                values(
                    PSI.read_results_with_keys(results_ed,
                        [voltage_stability_aux_var_key];
                        table_format = TableFormat.LONG),
                ),
            )
            @test !isnothing(voltage_stability)
            @test length(unique(voltage_stability.DateTime)) == 48 * 12
        else
            @test voltage_stability_aux_var_key ∉ available_aux_variables
        end

        @test length(filter(x -> isdir(joinpath(pf_path, x)), readdir(pf_path))) == 48 * 12
        # this now returns a system?!
        first_export = load_pf_export(pf_path, "export_1_1")
        last_export = load_pf_export(pf_path, "export_48_12")

        # Test that the active powers written to the first and last exports line up with the real simulation results
        for gen_name in get_name.(get_components(ThermalStandard, c_sys5_hy_ed))
            this_first_result =
                filter(row -> row[:name] == gen_name, first_result)[1, :value]
            this_first_exported =
                get_active_power(get_component(ThermalStandard, first_export, gen_name))
            @test isapprox(this_first_result, this_first_exported)

            this_last_result = filter(row -> row[:name] == gen_name, last_result)[1, :value]
            this_last_exported =
                get_active_power(get_component(ThermalStandard, last_export, gen_name))
            @test isapprox(this_last_result, this_last_exported)
        end
    end
end

@testset "AC Power Flow line active power loss auxiliary variable" begin
    system = build_system(PSITestSystems, "c_sys5_uc")

    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(system),
            power_flow_evaluation = ACPowerFlow(),
        ),
    )
    model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    results = OptimizationProblemResults(model_m)
    ad = read_aux_variables(results)

    active_power_ft = ad["PowerFlowBranchActivePowerFromTo__Line"]
    active_power_tf = ad["PowerFlowBranchActivePowerToFrom__Line"]
    active_power_loss = ad["PowerFlowBranchActivePowerLoss__Line"]

    for line_name in unique(active_power_loss.name)
        ft_vals = filter(row -> row[:name] == line_name, active_power_ft)[!, :value]
        tf_vals = filter(row -> row[:name] == line_name, active_power_tf)[!, :value]
        loss_vals = filter(row -> row[:name] == line_name, active_power_loss)[!, :value]
        @test isapprox(loss_vals, ft_vals .+ tf_vals; atol = 1e-9)
    end
end
