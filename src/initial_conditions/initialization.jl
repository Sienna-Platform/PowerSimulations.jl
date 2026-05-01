function get_initial_conditions_template(model::OperationModel, number_of_steps::Int)
    # This is done to avoid passing the duals but also not re-allocating the PTDF when it
    # exists

    network_model = NetworkModel(
        get_network_formulation(model.template);
        use_slacks = get_use_slacks(get_network_model(model.template)),
        PTDF_matrix = get_PTDF_matrix(get_network_model(model.template)),
        MODF_matrix = get_MODF_matrix(get_network_model(model.template)),
        reduce_radial_branches = get_reduce_radial_branches(
            get_network_model(model.template),
        ),
    )
    set_hvdc_network_model!(
        network_model,
        deepcopy(get_hvdc_network_model(model.template)),
    )
    network_model.network_reduction =
        deepcopy(get_network_reduction(get_network_model(model.template)))
    network_model.subnetworks = get_subnetworks(get_network_model(model.template))
    # Initialization does not support PowerFlow evaluation
    network_model.power_flow_evaluation = Vector{PFS.PowerFlowEvaluationModel}[]
    bus_area_map = get_bus_area_map(get_network_model(model.template))

    if !isempty(bus_area_map)
        network_model.bus_area_map = get_bus_area_map(get_network_model(model.template))
    end
    network_model.modeled_ac_branch_types =
        get_network_model(model.template).modeled_ac_branch_types
    ic_template = ProblemTemplate(network_model)
    # Do not copy events here for initialization
    for device_model in values(model.template.devices)
        base_model = get_initial_conditions_device_model(model, device_model)
        base_model.use_slacks = device_model.use_slacks
        base_model.time_series_names = device_model.time_series_names
        base_model.attributes = device_model.attributes
        set_device_model!(ic_template, base_model)
    end
    for device_model in values(model.template.branches)
        base_model = get_initial_conditions_device_model(model, device_model)
        base_model.use_slacks = device_model.use_slacks
        base_model.time_series_names = device_model.time_series_names
        base_model.attributes = device_model.attributes
        set_device_model!(ic_template, base_model)
    end

    for service_model in values(model.template.services)
        base_model = get_initial_conditions_service_model(model, service_model)
        base_model.service_name = service_model.service_name
        base_model.contributing_devices_map = service_model.contributing_devices_map
        base_model.use_slacks = service_model.use_slacks
        base_model.time_series_names = service_model.time_series_names
        base_model.attributes = service_model.attributes
        set_service_model!(ic_template, get_service_name(service_model), base_model)
    end
    set_number_of_steps!(network_model.reduced_branch_tracker, number_of_steps)
    if !isempty(model.template.services)
        _add_services_to_device_model!(ic_template)
    end
    return ic_template
end

function _make_init_jump_model(ic_settings::Settings)
    optimizer = get_optimizer(ic_settings)
    JuMPmodel = JuMP.Model(optimizer)
    warm_start_enabled = get_warm_start(ic_settings)
    solver_supports_warm_start = _validate_warm_start_support(JuMPmodel, warm_start_enabled)
    set_warm_start!(ic_settings, solver_supports_warm_start)
    if get_optimizer_solve_log_print(ic_settings)
        JuMP.unset_silent(JuMPmodel)
        @debug "optimizer unset to silent" _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    else
        JuMP.set_silent(JuMPmodel)
        @debug "optimizer set to silent" _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    end
    return JuMPmodel
end

function build_initial_conditions_model!(model::T) where {T <: OperationModel}
    internal = get_internal(model)
    ISOPT.set_initial_conditions_model_container!(
        internal,
        deepcopy(get_optimization_container(model)),
    )
    ic_container = ISOPT.get_initial_conditions_model_container(internal)
    ic_settings = deepcopy(get_settings(ic_container))
    main_problem_horizon = get_horizon(ic_settings)
    # TODO: add an interface to allow user to configure initial_conditions problem
    ic_container.JuMPmodel = _make_init_jump_model(ic_settings)
    resolution = get_resolution(ic_settings)
    init_horizon = INITIALIZATION_PROBLEM_HORIZON_COUNT * resolution
    number_of_steps = min(init_horizon, main_problem_horizon)
    template = get_initial_conditions_template(model, number_of_steps ÷ resolution)
    ic_container.settings = ic_settings
    ic_container.built_for_recurrent_solves = false
    set_horizon!(ic_settings, number_of_steps)
    init_optimization_container!(
        ISOPT.get_initial_conditions_model_container(internal),
        get_network_model(get_template(model)),
        get_system(model),
    )
    JuMP.set_string_names_on_creation(
        get_jump_model(ISOPT.get_initial_conditions_model_container(internal)),
        false,
    )
    TimerOutputs.disable_timer!(BUILD_PROBLEMS_TIMER)

    build_impl!(
        model.internal.initial_conditions_model_container,
        template,
        get_system(model),
    )
    TimerOutputs.enable_timer!(BUILD_PROBLEMS_TIMER)
    return
end
