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
            power_flow_evaluation = ACPolarPowerFlow(),
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
                power_flow_evaluation = ACPolarPowerFlow(),
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
            power_flow_evaluation = ACPolarPowerFlow(),
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
            power_flow_evaluation = ACPolarPowerFlow(),
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
        ProblemTemplate(
            NetworkModel(PTDFPowerModel; power_flow_evaluation = ACPolarPowerFlow()),
        )

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
                ACPolarPowerFlow(;
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
            power_flow_evaluation = ACPolarPowerFlow(),
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

@testset "AC Power Flow in the loop with headroom-proportional slack" begin
    system = build_system(PSITestSystems, "c_sys5_uc")

    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(system),
            power_flow_evaluation = ACPolarPowerFlow(;
                distribute_slack_proportional_to_headroom = true,
                correct_bustypes = true,
            ),
        ),
    )
    model = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = PSI.get_optimization_container(model)
    pf_e_data = only(PSI.get_power_flow_evaluation_data(container))
    data = PSI.get_power_flow_data(pf_e_data)

    computed_gspf = PFS.get_computed_gspf(data)
    n_time_steps = length(PSI.get_time_steps(container))
    bus_lookup = PFS.get_bus_lookup(data)
    base_power = get_base_power(system)

    # Headroom factors should be populated for every time step
    @test length(computed_gspf) == n_time_steps
    @test all(!isempty(d) for d in computed_gspf)
    @test all(all(v > 0.0 for v in values(d)) for d in computed_gspf)

    # bus_active_power_range should equal the sum of generator headroom per bus per time step
    for t in 1:n_time_steps
        bus_headroom_check = zeros(size(data.bus_active_power_range, 1))
        for ((comp_type, comp_name), headroom) in computed_gspf[t]
            comp = get_component(comp_type, system, comp_name)
            bus_number = get_number(get_bus(comp))
            bus_ix = bus_lookup[bus_number]
            bus_headroom_check[bus_ix] += headroom
        end
        @test isapprox(
            data.bus_active_power_range[:, t],
            bus_headroom_check;
            atol = 1e-10,
        )
    end

    # bus_slack_participation_factors should match bus_active_power_range at participating buses
    bus_slack_pf = PFS.get_bus_slack_participation_factors(data)
    for t in 1:n_time_steps
        for bus_ix in axes(data.bus_active_power_range, 1)
            R_k = data.bus_active_power_range[bus_ix, t]
            if R_k > 0.0
                @test bus_slack_pf[bus_ix, t] == R_k
            end
        end
    end

    # Independently recompute expected headroom from optimization results and system limits,
    # then verify it matches computed_gspf exactly.
    # NOTE: c_sys5_uc thermals have fixed active power limits (no ActivePowerTimeSeriesParameter),
    # so this exercises the static P_max path. The time-varying P_max path (using
    # min(static_limit, ts_param_value)) would need a system with renewable time series at a
    # REF/PV bus to be exercised.
    for t in 1:n_time_steps
        for ((comp_type, comp_name), headroom) in computed_gspf[t]
            comp = get_component(comp_type, system, comp_name)
            p_max_sys = PFS.get_active_power_limits_for_power_flow(comp).max

            # Look up the optimization set point for this generator at this time step
            var_key = PSI.VariableKey(PSI.ActivePowerVariable, comp_type)
            result_data = PSI.lookup_value(container, var_key)
            p_setpoint = JuMP.value(result_data[comp_name, t])

            expected_headroom = p_max_sys - p_setpoint
            @test expected_headroom > 0.0
            @test isapprox(headroom, expected_headroom; atol = 1e-10)
        end
    end
end

@testset "Headroom proportional slack excludes FixedOutput generators" begin
    # c_sys5_uc_re has renewables at PV/REF buses, so they would normally participate
    # in headroom slack. Setting them to FixedOutput should exclude them.
    system = build_system(PSITestSystems, "c_sys5_uc_re")

    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(system),
            use_slacks = true,
            power_flow_evaluation = ACPolarPowerFlow(;
                distribute_slack_proportional_to_headroom = true,
                correct_bustypes = true,
            ),
        ),
    )
    set_device_model!(template, RenewableDispatch, FixedOutput)

    model = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = PSI.get_optimization_container(model)
    pf_e_data = only(PSI.get_power_flow_evaluation_data(container))
    data = PSI.get_power_flow_data(pf_e_data)
    computed_gspf = PFS.get_computed_gspf(data)

    # The renewable is at a PV bus but uses FixedOutput, so it must not appear in the
    # headroom factors — only ThermalStandard generators should participate.
    for t in 1:length(PSI.get_time_steps(container))
        for ((comp_type, _), _) in computed_gspf[t]
            @test comp_type <: ThermalStandard
        end
    end
end

@testset "Headroom proportional slack with time-varying active power limits" begin
    # c_sys5_uc_re has renewables at PV/REF buses with time series data.
    system = build_system(PSITestSystems, "c_sys5_uc_re")
    re_gen = first(get_components(RenewableDispatch, system))
    re_name = get_name(re_gen)
    # Force device_base != system_base so the unit-handling path is exercised. If headroom
    # ever silently re-introduces a `* device_base / system_base` factor, the recomputed
    # expected_headroom below will mismatch by 0.5×, failing the assertion.
    PSY.set_units_base_system!(system, "SYSTEM_BASE")
    PSY.set_base_power!(re_gen, get_base_power(re_gen) / 2)

    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(system),
            use_slacks = true,
            power_flow_evaluation = ACPolarPowerFlow(;
                distribute_slack_proportional_to_headroom = true,
                correct_bustypes = true,
            ),
        ),
    )
    set_device_model!(template, RenewableDispatch, RenewableFullDispatch)

    model = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = PSI.get_optimization_container(model)
    pf_e_data = only(PSI.get_power_flow_evaluation_data(container))
    data = PSI.get_power_flow_data(pf_e_data)
    computed_gspf = PFS.get_computed_gspf(data)
    n_time_steps = length(PSI.get_time_steps(container))

    # Verify the renewable's headroom uses min(static_limit, ts_param) at each time step
    re_type = typeof(re_gen)
    p_max_static = PFS.get_active_power_limits_for_power_flow(re_gen).max

    var_key = PSI.VariableKey(PSI.ActivePowerVariable, re_type)
    var_values = PSI.lookup_value(container, var_key)
    ts_key = PSI.ParameterKey(PSI.ActivePowerTimeSeriesParameter, re_type)
    ts_values = PSI.lookup_value(container, ts_key)

    for t in 1:n_time_steps
        p_setpoint = JuMP.value(var_values[re_name, t])
        p_max_ts = ts_values[re_name, t]
        p_max_t = min(p_max_static, p_max_ts)
        expected_headroom = p_max_t - p_setpoint

        entry = get(computed_gspf[t], (re_type, re_name), nothing)
        if expected_headroom > 0.0
            @test !isnothing(entry)
            @test isapprox(entry, expected_headroom; atol = 1e-10)
        else
            @test isnothing(entry)
        end
    end

    # The time series should cause P_max to vary, producing different headroom across steps
    re_ts_vals = [ts_values[re_name, t] for t in 1:n_time_steps]
    @test !all(isapprox.(re_ts_vals, re_ts_vals[1]; atol = 1e-10))
end

@testset "Power Flow in the loop with separate in/out active power variables" begin
    # Build a system with a Source component that uses ImportExportSourceModel,
    # which creates separate ActivePowerInVariable and ActivePowerOutVariable.
    sys = make_5_bus_with_import_export(; add_single_time_series = false)

    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(sys),
            power_flow_evaluation = PTDFDCPowerFlow(),
        ),
    )
    set_device_model!(
        template,
        DeviceModel(
            Source,
            ImportExportSourceModel;
            attributes = Dict("reservation" => false),
        ),
    )
    model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = PSI.get_optimization_container(model)
    pf_e_data =
        only(PSI.get_power_flow_evaluation_data(container))
    input_key_map = PSI.get_input_key_map(pf_e_data)

    # Verify that active_power_in and active_power_out categories are present in the map
    @test haskey(input_key_map, :active_power_in)
    @test haskey(input_key_map, :active_power_out)

    # Check that ActivePowerInVariable and ActivePowerOutVariable for Source are mapped
    in_keys = collect(keys(input_key_map[:active_power_in]))
    out_keys = collect(keys(input_key_map[:active_power_out]))
    @test any(
        k ->
            PSI.get_entry_type(k) == PSI.ActivePowerInVariable &&
                PSI.get_component_type(k) == Source,
        in_keys,
    )
    @test any(
        k ->
            PSI.get_entry_type(k) == PSI.ActivePowerOutVariable &&
                PSI.get_component_type(k) == Source,
        out_keys,
    )

    # Verify the PowerFlowData bus injections reflect the net power (out - in)
    data = PSI.get_power_flow_data(pf_e_data)
    base_power = get_base_power(sys)
    bus_lookup = PFS.get_bus_lookup(data)

    source = get_component(Source, sys, "source")
    source_bus_ix = bus_lookup[get_number(get_bus(source))]

    results = OptimizationProblemResults(model)
    vd = read_variables(results)
    p_out_results = vd["ActivePowerOutVariable__Source"]
    p_in_results = vd["ActivePowerInVariable__Source"]

    source_p_out = filter(row -> row[:name] == "source", p_out_results)[!, :value]
    source_p_in = filter(row -> row[:name] == "source", p_in_results)[!, :value]

    @test length(source_p_out) > 0
    @test length(source_p_in) > 0

    # The net bus injection (= injections − withdrawals) at the source bus must equal
    # the sum of every `:active_power` contribution at that bus plus the source's
    # (out − in). Loads are routed to `bus_active_power_withdrawals` under the same
    # category, so subtracting withdrawals folds them back into the comparison.
    other_injection_at_bus = zeros(length(source_p_out))
    for (key, comp_map) in input_key_map[:active_power]
        result_data = PSI.lookup_value(container, key)
        for (dev_name, bus_ix) in comp_map
            bus_ix == source_bus_ix || continue
            for t in eachindex(other_injection_at_bus)
                other_injection_at_bus[t] += PSI.jump_value(result_data[dev_name, t])
            end
        end
    end
    source_net_pu = (source_p_out .- source_p_in) ./ base_power
    @test isapprox(
        data.bus_active_power_injections[source_bus_ix, :] .-
        data.bus_active_power_withdrawals[source_bus_ix, :],
        other_injection_at_bus .+ source_net_pu;
        atol = 1e-9,
    )
end

@testset "Headroom proportional slack with in/out active power variables (Source)" begin
    # nodeC is a PV bus in c_sys5_uc, so a Source there can participate in headroom slack.
    sys = make_5_bus_with_import_export(; add_single_time_series = false)

    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(sys),
            power_flow_evaluation = ACPolarPowerFlow(;
                distribute_slack_proportional_to_headroom = true,
                correct_bustypes = true,
            ),
        ),
    )
    set_device_model!(
        template,
        DeviceModel(
            Source,
            ImportExportSourceModel;
            attributes = Dict("reservation" => false),
        ),
    )
    model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = PSI.get_optimization_container(model)
    pf_e_data = only(PSI.get_power_flow_evaluation_data(container))
    data = PSI.get_power_flow_data(pf_e_data)
    computed_gspf = PFS.get_computed_gspf(data)
    n_time_steps = length(PSI.get_time_steps(container))

    source = get_component(Source, sys, "source")
    p_max_out = PSY.get_active_power_limits(source).max

    in_key = PSI.VariableKey(PSI.ActivePowerInVariable, Source)
    out_key = PSI.VariableKey(PSI.ActivePowerOutVariable, Source)
    p_in_data = PSI.lookup_value(container, in_key)
    p_out_data = PSI.lookup_value(container, out_key)

    # When net = p_out - p_in is negative the source is charging and per spec gets zero
    # headroom (omitted from `computed_gspf`); when net >= 0 the headroom is p_max_out - net.
    for t in 1:n_time_steps
        net = JuMP.value(p_out_data["source", t]) - JuMP.value(p_in_data["source", t])
        if net < 0.0
            @test !haskey(computed_gspf[t], (Source, "source"))
        else
            @test isapprox(
                computed_gspf[t][(Source, "source")],
                p_max_out - net;
                atol = 1e-10,
            )
        end
    end
    # Guard against a regression that silently drops the in/out accumulation path.
    @test any(haskey(d, (Source, "source")) for d in computed_gspf)
end

@testset "Power Flow in the loop with Source FixedOutput (parameter path)" begin
    # Source under FixedOutput emits ActivePowerIn/OutTimeSeriesParameter and
    # no In/Out Variables. Verifies that the parameter keys are picked up by
    # PF_INPUT_KEY_PRECEDENCES (Luke Kiernan, PR #1612), and that the resulting
    # bus injection follows the same out − in allocation rule as the variable path.
    sys = make_5_bus_with_import_export(; add_single_time_series = true)
    source = get_component(Source, sys, "source")

    load = first(get_components(PowerLoad, sys))
    tstamp = TimeSeries.timestamp(
        get_time_series_array(SingleTimeSeries, load, "max_active_power"),
    )
    day_data = [
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
        0.9, 0.85, 0.95, 0.2, 0.0, 0.0,
    ]
    ts_data = repeat(day_data, 2)
    ts_out = SingleTimeSeries(
        "max_active_power_out",
        TimeArray(tstamp, ts_data);
        scaling_factor_multiplier = get_max_active_power,
    )
    ts_in = SingleTimeSeries(
        "max_active_power_in",
        TimeArray(tstamp, ts_data);
        scaling_factor_multiplier = get_max_active_power,
    )
    add_time_series!(sys, source, ts_out)
    add_time_series!(sys, source, ts_in)
    transform_single_time_series!(sys, Hour(24), Hour(24))

    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(sys),
            power_flow_evaluation = PTDFDCPowerFlow(),
        ),
    )
    set_device_model!(template, DeviceModel(Source, FixedOutput))
    model = DecisionModel(template, sys; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = PSI.get_optimization_container(model)
    pf_e_data = only(PSI.get_power_flow_evaluation_data(container))
    input_key_map = PSI.get_input_key_map(pf_e_data)

    @test haskey(input_key_map, :active_power_in)
    @test haskey(input_key_map, :active_power_out)

    in_keys = collect(keys(input_key_map[:active_power_in]))
    out_keys = collect(keys(input_key_map[:active_power_out]))
    @test any(
        k ->
            PSI.get_entry_type(k) == PSI.ActivePowerInTimeSeriesParameter &&
                PSI.get_component_type(k) == Source,
        in_keys,
    )
    @test any(
        k ->
            PSI.get_entry_type(k) == PSI.ActivePowerOutTimeSeriesParameter &&
                PSI.get_component_type(k) == Source,
        out_keys,
    )

    # PF data injections at the source bus should equal the sum of every other
    # device's contribution at that bus plus (out_param − in_param) from the
    # FixedOutput source — same allocation rule as the in/out variable path.
    data = PSI.get_power_flow_data(pf_e_data)
    bus_lookup = PFS.get_bus_lookup(data)
    source_bus_ix = bus_lookup[get_number(get_bus(source))]
    n_time_steps = length(PSI.get_time_steps(container))

    in_param = PSI.lookup_value(
        container, PSI.ParameterKey(PSI.ActivePowerInTimeSeriesParameter, Source),
    )
    out_param = PSI.lookup_value(
        container, PSI.ParameterKey(PSI.ActivePowerOutTimeSeriesParameter, Source),
    )

    other_injection_at_bus = zeros(n_time_steps)
    for (key, comp_map) in input_key_map[:active_power]
        result_data = PSI.lookup_value(container, key)
        for (dev_name, bus_ix) in comp_map
            bus_ix == source_bus_ix || continue
            for t in 1:n_time_steps
                other_injection_at_bus[t] += PSI.jump_value(result_data[dev_name, t])
            end
        end
    end

    source_net = [
        PSI.jump_value(out_param["source", t]) - PSI.jump_value(in_param["source", t])
        for t in 1:n_time_steps
    ]
    # Net bus injection = injections − withdrawals; loads route to withdrawals under
    # the same `:active_power` category and the difference folds them back in.
    @test isapprox(
        data.bus_active_power_injections[source_bus_ix, 1:n_time_steps] .-
        data.bus_active_power_withdrawals[source_bus_ix, 1:n_time_steps],
        other_injection_at_bus .+ source_net;
        atol = 1e-9,
    )

    # Guard against a regression that drives both parameters to zero — the test
    # above would then pass trivially without exercising the parameter path.
    @test !all(isapprox.(source_net, 0.0; atol = 1e-10))
end

# PowerFlows 0.17 split the AC formulation into `ACPolarPowerFlow` and
# `ACRectangularPowerFlow` (the legacy `ACPowerFlow` is now an alias for the
# polar formulation). This testset drives the same model through the
# power-flow-in-the-loop path with each formulation and asserts the rectangular
# formulation both converges and lands on the same physical solution as polar.
@testset "AC Power Flow in the loop: polar vs rectangular formulation" begin
    pf_data_for(pf_eval) = begin
        system = build_system(PSITestSystems, "c_sys5_uc")
        template = get_template_dispatch_with_network(
            NetworkModel(
                PTDFPowerModel;
                PTDF_matrix = PTDF(system),
                power_flow_evaluation = pf_eval,
            ),
        )
        model_m = DecisionModel(template, system; optimizer = HiGHS_optimizer)
        @test build!(model_m; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        @test solve!(model_m) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
        return PSI.get_power_flow_data(
            only(
                PSI.get_power_flow_evaluation_data(
                    PSI.get_optimization_container(model_m),
                ),
            ),
        )
    end

    polar_data = pf_data_for(ACPolarPowerFlow())
    rect_data = pf_data_for(ACRectangularPowerFlow())

    # Both formulations must converge through the in-the-loop solve
    # (`get_converged` returns a per-period vector for the multi-period path).
    @test all(PFS.get_converged(polar_data))
    @test all(PFS.get_converged(rect_data))

    # Polar and rectangular are different Newton parametrizations of the same
    # physical AC power flow, so the recovered state must agree.
    @test isapprox(polar_data.bus_magnitude, rect_data.bus_magnitude;
        atol = 1e-6, rtol = 0)
    @test isapprox(polar_data.bus_angles, rect_data.bus_angles;
        atol = 1e-6, rtol = 0)
    @test isapprox(
        polar_data.bus_active_power_injections,
        rect_data.bus_active_power_injections;
        atol = 1e-6,
        rtol = 0,
    )
    @test isapprox(
        polar_data.bus_reactive_power_injections,
        rect_data.bus_reactive_power_injections;
        atol = 1e-6,
        rtol = 0,
    )
end

# Regression test for issue: AC power flow evaluator on an active-power-only
# network model (e.g. `PTDFPowerModel`) used to ignore reactive power because
# `AbstractActivePowerModel` device constructors don't add
# `ReactivePowerTimeSeriesParameter` to the optimization container, leaving
# `_make_pf_input_map!` with zero input keys for `:reactive_power`.
@testset "AC Power Flow in the loop populates reactive power on PTDFPowerModel" begin
    system = build_system(PSITestSystems, "c_sys5_uc")
    template = get_template_dispatch_with_network(
        NetworkModel(
            PTDFPowerModel;
            PTDF_matrix = PTDF(system),
            power_flow_evaluation = ACPolarPowerFlow(),
        ),
    )
    model = DecisionModel(template, system; optimizer = HiGHS_optimizer)
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = PSI.get_optimization_container(model)
    pf_e_data = only(PSI.get_power_flow_evaluation_data(container))
    input_key_map = PSI.get_input_key_map(pf_e_data)

    # The :reactive_power category must have a route from the
    # ReactivePowerTimeSeriesParameter for PowerLoad components even though the
    # PTDFPowerModel optimization formulation itself doesn't model reactive power.
    @test haskey(input_key_map, :reactive_power)
    reactive_keys = collect(keys(input_key_map[:reactive_power]))
    @test any(
        k ->
            PSI.get_entry_type(k) == PSI.ReactivePowerTimeSeriesParameter &&
                PSI.get_component_type(k) == PowerLoad,
        reactive_keys,
    )

    # The PowerFlowData should have non-zero reactive power withdrawals at the
    # buses that host the loads (loads in c_sys5_uc carry a reactive component).
    data = PSI.get_power_flow_data(pf_e_data)
    @test any(!iszero, data.bus_reactive_power_withdrawals)

    # And the parameter should now exist in the container so the optimization
    # path can route it to the PF (used to be missing under PTDFPowerModel).
    @test PSI.has_container_key(
        container,
        PSI.ReactivePowerTimeSeriesParameter,
        PowerLoad,
    )
end
