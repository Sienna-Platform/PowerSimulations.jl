struct PrimalValuesCache
    variables_cache::Dict{VariableKey, AbstractArray}
    expressions_cache::Dict{ExpressionKey, AbstractArray}
end

function PrimalValuesCache()
    return PrimalValuesCache(
        Dict{VariableKey, AbstractArray}(),
        Dict{ExpressionKey, AbstractArray}(),
    )
end

function Base.isempty(pvc::PrimalValuesCache)
    return isempty(pvc.variables_cache) && isempty(pvc.expressions_cache)
end

mutable struct ObjectiveFunction
    invariant_terms::JuMP.AbstractJuMPScalar
    variant_terms::GAE
    synchronized::Bool
    sense::MOI.OptimizationSense
    function ObjectiveFunction(invariant_terms::JuMP.AbstractJuMPScalar,
        variant_terms::GAE,
        synchronized::Bool,
        sense::MOI.OptimizationSense = MOI.MIN_SENSE)
        new(invariant_terms, variant_terms, synchronized, sense)
    end
end

get_invariant_terms(v::ObjectiveFunction) = v.invariant_terms
get_variant_terms(v::ObjectiveFunction) = v.variant_terms
function get_objective_expression(v::ObjectiveFunction)
    if iszero(v.variant_terms)
        return v.invariant_terms
    else
        # JuMP doesn't support expression conversion from Affn to QuadExpressions
        if isa(v.invariant_terms, JuMP.GenericQuadExpr)
            # Avoid mutation of invariant term
            temp_expr = JuMP.QuadExpr()
            JuMP.add_to_expression!(temp_expr, v.invariant_terms)
            return JuMP.add_to_expression!(temp_expr, v.variant_terms)
        else
            # This will mutate the variant terms, but these are reseted at each step.
            return JuMP.add_to_expression!(v.variant_terms, v.invariant_terms)
        end
    end
end
get_sense(v::ObjectiveFunction) = v.sense
is_synchronized(v::ObjectiveFunction) = v.synchronized
set_synchronized_status!(v::ObjectiveFunction, value) = v.synchronized = value
reset_variant_terms(v::ObjectiveFunction) = v.variant_terms = zero(JuMP.AffExpr)
has_variant_terms(v::ObjectiveFunction) = !iszero(v.variant_terms)
set_sense!(v::ObjectiveFunction, sense::MOI.OptimizationSense) = v.sense = sense

function ObjectiveFunction()
    return ObjectiveFunction(
        zero(JuMP.GenericAffExpr{Float64, JuMP.VariableRef}),
        zero(JuMP.AffExpr),
        true,
    )
end

mutable struct OptimizationContainer <: ISOPT.AbstractOptimizationContainer
    JuMPmodel::JuMP.Model
    time_steps::UnitRange{Int}
    settings::Settings
    variables::OrderedDict{VariableKey, AbstractArray}
    aux_variables::OrderedDict{AuxVarKey, AbstractArray}
    duals::OrderedDict{ConstraintKey, AbstractArray}
    constraints::OrderedDict{ConstraintKey, AbstractArray}
    objective_function::ObjectiveFunction
    expressions::OrderedDict{ExpressionKey, AbstractArray}
    parameters::OrderedDict{ParameterKey, ParameterContainer}
    primal_values_cache::PrimalValuesCache
    initial_conditions::OrderedDict{InitialConditionKey, Vector{<:InitialCondition}}
    initial_conditions_data::InitialConditionsData
    infeasibility_conflict::Dict{Symbol, Array}
    pm::Union{Nothing, PM.AbstractPowerModel}
    base_power::Float64
    optimizer_stats::OptimizerStats
    built_for_recurrent_solves::Bool
    metadata::ISOPT.OptimizationContainerMetadata
    default_time_series_type::Type{<:PSY.TimeSeriesData}
    power_flow_evaluation_data::Vector{PowerFlowEvaluationData}
end

function OptimizationContainer(
    sys::PSY.System,
    settings::Settings,
    jump_model::Union{Nothing, JuMP.Model},
    ::Type{T},
) where {T <: PSY.TimeSeriesData}
    if isabstracttype(T)
        error("Default Time Series Type $V can't be abstract")
    end

    if jump_model !== nothing && get_direct_mode_optimizer(settings)
        throw(
            IS.ConflictingInputsError(
                "Externally provided JuMP models are not compatible with the direct model keyword argument. Use JuMP.direct_model before passing the custom model",
            ),
        )
    end

    return OptimizationContainer(
        jump_model === nothing ? JuMP.Model() : jump_model,
        1:1,
        settings,
        OrderedDict{VariableKey, AbstractArray}(),
        OrderedDict{AuxVarKey, AbstractArray}(),
        OrderedDict{ConstraintKey, AbstractArray}(),
        OrderedDict{ConstraintKey, AbstractArray}(),
        ObjectiveFunction(),
        OrderedDict{ExpressionKey, AbstractArray}(),
        OrderedDict{ParameterKey, ParameterContainer}(),
        PrimalValuesCache(),
        OrderedDict{InitialConditionKey, Vector{InitialCondition}}(),
        InitialConditionsData(),
        Dict{Symbol, Array}(),
        nothing,
        PSY.get_base_power(sys),
        OptimizerStats(),
        false,
        ISOPT.OptimizationContainerMetadata(),
        T,
        Vector{PowerFlowEvaluationData}[],
    )
end

built_for_recurrent_solves(container::OptimizationContainer) =
    container.built_for_recurrent_solves

get_aux_variables(container::OptimizationContainer) = container.aux_variables
get_base_power(container::OptimizationContainer) = container.base_power
get_constraints(container::OptimizationContainer) = container.constraints

function cost_function_unsynch(container::OptimizationContainer)
    obj_func = get_objective_expression(container)
    if has_variant_terms(obj_func) && is_synchronized(container)
        set_synchronized_status!(obj_func, false)
        reset_variant_terms(obj_func)
    end
    return
end

function get_container_keys(container::OptimizationContainer)
    return Iterators.flatten(keys(getfield(container, f)) for f in STORE_CONTAINERS)
end

get_default_time_series_type(container::OptimizationContainer) =
    container.default_time_series_type
get_duals(container::OptimizationContainer) = container.duals
get_expressions(container::OptimizationContainer) = container.expressions
get_infeasibility_conflict(container::OptimizationContainer) =
    container.infeasibility_conflict
get_initial_conditions(container::OptimizationContainer) = container.initial_conditions
get_initial_conditions_data(container::OptimizationContainer) =
    container.initial_conditions_data
get_initial_time(container::OptimizationContainer) = get_initial_time(container.settings)
get_jump_model(container::OptimizationContainer) = container.JuMPmodel
get_metadata(container::OptimizationContainer) = container.metadata
get_optimizer_stats(container::OptimizationContainer) = container.optimizer_stats
get_parameters(container::OptimizationContainer) = container.parameters
get_power_flow_evaluation_data(container::OptimizationContainer) =
    container.power_flow_evaluation_data
get_resolution(container::OptimizationContainer) = get_resolution(container.settings)
get_settings(container::OptimizationContainer) = container.settings
get_time_steps(container::OptimizationContainer) = container.time_steps
get_variables(container::OptimizationContainer) = container.variables

set_initial_conditions_data!(container::OptimizationContainer, data) =
    container.initial_conditions_data = data
get_objective_expression(container::OptimizationContainer) = container.objective_function
is_synchronized(container::OptimizationContainer) =
    container.objective_function.synchronized
set_time_steps!(container::OptimizationContainer, time_steps::UnitRange{Int64}) =
    container.time_steps = time_steps

function reset_power_flow_is_solved!(container::OptimizationContainer)
    for pf_e_data in get_power_flow_evaluation_data(container)
        pf_e_data.is_solved = false
    end
end

function has_container_key(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ExpressionType, U <: Union{PSY.Component, PSY.System}}
    key = ExpressionKey(T, U, meta)
    return haskey(container.expressions, key)
end

function has_container_key(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    key = VariableKey(T, U, meta)
    return haskey(container.variables, key)
end

function has_container_key(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: AuxVariableType, U <: Union{PSY.Component, PSY.System}}
    key = AuxVarKey(T, U, meta)
    return haskey(container.aux_variables, key)
end

function has_container_key(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    key = ConstraintKey(T, U, meta)
    return haskey(container.constraints, key)
end

function has_container_key(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    key = ParameterKey(T, U, meta)
    return haskey(container.parameters, key)
end

function has_container_key(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: InitialConditionType, U <: Union{PSY.Component, PSY.System}}
    key = InitialConditionKey(T, U, meta)
    return haskey(container.initial_conditions, key)
end

function is_milp(container::OptimizationContainer)::Bool
    !supports_milp(container) && return false
    if !isempty(
        JuMP.all_constraints(
            PSI.get_jump_model(container),
            JuMP.VariableRef,
            JuMP.MOI.ZeroOne,
        ),
    )
        return true
    end
    return false
end

function supports_milp(container::OptimizationContainer)
    jump_model = get_jump_model(container)
    return supports_milp(jump_model)
end

function _validate_warm_start_support(JuMPmodel::JuMP.Model, warm_start_enabled::Bool)
    !warm_start_enabled && return warm_start_enabled
    solver_supports_warm_start =
        MOI.supports(JuMP.backend(JuMPmodel), MOI.VariablePrimalStart(), MOI.VariableIndex)
    if !solver_supports_warm_start
        solver_name = JuMP.solver_name(JuMPmodel)
        @warn("$(solver_name) does not support warm start")
    end
    return solver_supports_warm_start
end

function _finalize_jump_model!(container::OptimizationContainer, settings::Settings)
    @debug "Instantiating the JuMP model" _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    if built_for_recurrent_solves(container) && get_optimizer(settings) === nothing
        throw(
            IS.ConflictingInputsError(
                "Optimizer can not be nothing when building for recurrent solves",
            ),
        )
    end

    if get_direct_mode_optimizer(settings)
        optimizer = () -> MOI.instantiate(get_optimizer(settings))
        container.JuMPmodel = JuMP.direct_model(optimizer())
    elseif get_optimizer(settings) === nothing
        @debug "The optimization model has no optimizer attached" _group =
            LOG_GROUP_OPTIMIZATION_CONTAINER
    else
        JuMP.set_optimizer(PSI.get_jump_model(container), get_optimizer(settings))
    end

    JuMPmodel = PSI.get_jump_model(container)
    warm_start_enabled = get_warm_start(settings)
    solver_supports_warm_start = _validate_warm_start_support(JuMPmodel, warm_start_enabled)
    set_warm_start!(settings, solver_supports_warm_start)

    JuMP.set_string_names_on_creation(JuMPmodel, get_store_variable_names(settings))

    @debug begin
        JuMP.set_string_names_on_creation(JuMPmodel, true)
    end
    if get_optimizer_solve_log_print(settings)
        JuMP.unset_silent(JuMPmodel)
        @debug "optimizer unset to silent" _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    else
        JuMP.set_silent(JuMPmodel)
        @debug "optimizer set to silent" _group = LOG_GROUP_OPTIMIZATION_CONTAINER
    end
    return
end

function init_optimization_container!(
    container::OptimizationContainer,
    network_model::NetworkModel{T},
    sys::PSY.System,
) where {T <: PM.AbstractPowerModel}
    PSY.set_units_base_system!(sys, "SYSTEM_BASE")
    # The order of operations matter
    settings = get_settings(container)

    if get_initial_time(settings) == UNSET_INI_TIME
        if get_default_time_series_type(container) <: PSY.AbstractDeterministic
            set_initial_time!(settings, PSY.get_forecast_initial_timestamp(sys))
        elseif get_default_time_series_type(container) <: PSY.SingleTimeSeries
            ini_time, _ = PSY.check_time_series_consistency(sys, PSY.SingleTimeSeries)
            set_initial_time!(settings, ini_time)
        end
    end

    if get_resolution(settings) == UNSET_RESOLUTION
        error("Resolution not set in the model. Can't continue with the build.")
    end

    horizon_count = (get_horizon(settings) ÷ get_resolution(settings))
    @assert horizon_count > 0
    container.time_steps = 1:horizon_count

    if T <: CopperPlatePowerModel || T <: AreaBalancePowerModel
        total_number_of_devices =
            length(get_available_components(network_model, PSY.Device, sys))
    else
        total_number_of_devices =
            length(get_available_components(network_model, PSY.Device, sys))
        total_number_of_devices +=
            length(get_available_components(network_model, PSY.ACBranch, sys))
    end

    # The 10e6 limit is based on the sizes of the lp benchmark problems http://plato.asu.edu/ftp/lpcom.html
    # The maximum numbers of constraints and variables in the benchmark problems is 1,918,399 and 1,259,121,
    # respectively. See also https://prod-ng.sandia.gov/techlib-noauth/access-control.cgi/2013/138847.pdf
    variable_count_estimate = length(container.time_steps) * total_number_of_devices

    if variable_count_estimate > 10e6
        @warn(
            "The lower estimate of total number of variables that will be created in the model is $(variable_count_estimate). \\
            The total number of variables might be larger than 10e6 and could lead to large build or solve times."
        )
    end

    stats = get_optimizer_stats(container)
    stats.detailed_stats = get_detailed_optimizer_stats(settings)

    _finalize_jump_model!(container, settings)
    return
end

function reset_optimization_model!(container::OptimizationContainer)
    for field in [:variables, :aux_variables, :constraints, :expressions, :duals]
        empty!(getfield(container, field))
    end
    container.initial_conditions_data = InitialConditionsData()
    container.objective_function = ObjectiveFunction()
    container.primal_values_cache = PrimalValuesCache()
    JuMP.empty!(PSI.get_jump_model(container))
    return
end

function check_parameter_multiplier_values(multiplier_array::DenseAxisArray)
    return !all(isnan.(multiplier_array.data))
end

function check_parameter_multiplier_values(multiplier_array::SparseAxisArray)
    return !all(isnan.(values(multiplier_array.data)))
end

function check_optimization_container(container::OptimizationContainer)
    for (k, param_container) in container.parameters
        valid = check_parameter_multiplier_values(param_container.multiplier_array)
        if !valid
            error("The model container has invalid values in $(encode_key_as_string(k))")
        end
    end
    return
end

function get_problem_size(container::OptimizationContainer)
    model = get_jump_model(container)
    vars = JuMP.num_variables(model)
    cons = 0
    for (exp, c_type) in JuMP.list_of_constraint_types(model)
        cons += JuMP.num_constraints(model, exp, c_type)
    end
    return "The current total number of variables is $(vars) and total number of constraints is $(cons)"
end

function _make_container_array(ax...)
    return remove_undef!(DenseAxisArray{GAE}(undef, ax...))
end

function _make_system_expressions!(
    container::OptimizationContainer,
    subnetworks::Dict{Int, Set{Int}},
    ::Vector{Int},
    ::Type{<:PM.AbstractPowerModel},
    bus_reduction_map::Dict{Int64, Set{Int64}},
)
    time_steps = get_time_steps(container)
    if isempty(bus_reduction_map)
        ac_bus_numbers = collect(Iterators.flatten(values(subnetworks)))
    else
        ac_bus_numbers = collect(keys(bus_reduction_map))
    end
    container.expressions = Dict(
        ExpressionKey(ActivePowerBalance, PSY.ACBus) =>
            _make_container_array(ac_bus_numbers, time_steps),
        ExpressionKey(ReactivePowerBalance, PSY.ACBus) =>
            _make_container_array(ac_bus_numbers, time_steps),
    )
    return
end

function _make_system_expressions!(
    container::OptimizationContainer,
    subnetworks::Dict{Int, Set{Int}},
    ::Vector{Int},
    ::Type{<:PM.AbstractActivePowerModel},
    bus_reduction_map::Dict{Int64, Set{Int64}},
)
    time_steps = get_time_steps(container)
    if isempty(bus_reduction_map)
        ac_bus_numbers = collect(Iterators.flatten(values(subnetworks)))
    else
        ac_bus_numbers = collect(keys(bus_reduction_map))
    end
    container.expressions = Dict(
        ExpressionKey(ActivePowerBalance, PSY.ACBus) =>
            _make_container_array(ac_bus_numbers, time_steps),
    )
    return
end

function _make_system_expressions!(
    container::OptimizationContainer,
    subnetworks::Dict{Int, Set{Int}},
    ::Vector{Int},
    ::Type{CopperPlatePowerModel},
    bus_reduction_map::Dict{Int64, Set{Int64}},
)
    time_steps = get_time_steps(container)
    subnetworks_ref_buses = collect(keys(subnetworks))
    container.expressions = Dict(
        ExpressionKey(ActivePowerBalance, PSY.System) =>
            _make_container_array(subnetworks_ref_buses, time_steps),
    )
    return
end

function _make_system_expressions!(
    container::OptimizationContainer,
    subnetworks::Dict{Int, Set{Int}},
    ::Vector{Int},
    ::Type{T},
    bus_reduction_map::Dict{Int64, Set{Int64}},
) where {T <: Union{PTDFPowerModel}}
    time_steps = get_time_steps(container)
    if isempty(bus_reduction_map)
        ac_bus_numbers = collect(Iterators.flatten(values(subnetworks)))
    else
        ac_bus_numbers = collect(keys(bus_reduction_map))
    end
    subnetworks = collect(keys(subnetworks))
    container.expressions = Dict(
        ExpressionKey(ActivePowerBalance, PSY.System) =>
            _make_container_array(subnetworks, time_steps),
        ExpressionKey(ActivePowerBalance, PSY.ACBus) =>
        # Bus numbers are sorted to guarantee consistency in the order between the
        # containers
            _make_container_array(sort!(ac_bus_numbers), time_steps),
    )
    return
end

function _make_system_expressions!(
    container::OptimizationContainer,
    subnetworks::Dict{Int, Set{Int}},
    ::Type{AreaBalancePowerModel},
    areas::IS.FlattenIteratorWrapper{PSY.Area},
)
    time_steps = get_time_steps(container)
    container.expressions = Dict(
        ExpressionKey(ActivePowerBalance, PSY.Area) =>
            _make_container_array(PSY.get_name.(areas), time_steps),
    )
    return
end

function _make_system_expressions!(
    container::OptimizationContainer,
    subnetworks::Dict{Int, Set{Int}},
    ::Vector{Int},
    ::Type{AreaPTDFPowerModel},
    areas::IS.FlattenIteratorWrapper{PSY.Area},
    bus_reduction_map::Dict{Int64, Set{Int64}},
)
    time_steps = get_time_steps(container)
    if isempty(bus_reduction_map)
        ac_bus_numbers = collect(Iterators.flatten(values(subnetworks)))
    else
        ac_bus_numbers = collect(keys(bus_reduction_map))
    end
    container.expressions = Dict(
        # Enforces the balance by Area
        ExpressionKey(ActivePowerBalance, PSY.Area) =>
            _make_container_array(PSY.get_name.(areas), time_steps),
        # Keeps track of the Injections by bus.
        ExpressionKey(ActivePowerBalance, PSY.ACBus) =>
        # Bus numbers are sorted to guarantee consistency in the order between the
        # containers
            _make_container_array(sort!(ac_bus_numbers), time_steps),
    )

    if length(subnetworks) > 1
        @warn "The system contains $(length(subnetworks)) synchronous regions. \
               When combined with AreaPTDFPowerModel, the model can be infeasible if the data doesn't \
               have a well defined topology"
        subnetworks_ref_buses = collect(keys(subnetworks))
        container.expressions[ExpressionKey(ActivePowerBalance, PSY.System)] =
            _make_container_array(subnetworks_ref_buses, time_steps)
    end

    return
end

function initialize_system_expressions!(
    container::OptimizationContainer,
    network_model::NetworkModel{T},
    subnetworks::Dict{Int, Set{Int}},
    ::BranchModelContainer,
    system::PSY.System,
    bus_reduction_map::Dict{Int64, Set{Int64}},
) where {T <: PM.AbstractPowerModel}
    dc_bus_numbers = [
        PSY.get_number(b) for
        b in get_available_components(network_model, PSY.DCBus, system)
    ]
    _make_system_expressions!(container, subnetworks, dc_bus_numbers, T, bus_reduction_map)
    return
end

function _verify_area_subnetwork_topology(sys::PSY.System, subnetworks::Dict{Int, Set{Int}})
    if length(subnetworks) < 1
        @debug "Only one subnetwork detected in the system. Area - Subnetwork topology check is valid."
        return
    end

    @warn "More than one subnetwork detected in AreaBalancePowerModel. Topology consistency checks must be conducted."

    area_map = PSY.get_aggregation_topology_mapping(PSY.Area, sys)
    for (area, buses) in area_map
        bus_numbers =
            [
                PSY.get_number(b) for
                b in buses if PSY.get_bustype(b) != PSY.ACBusTypes.ISOLATED
            ]
        subnets = Int[]
        for (subnet, subnet_bus_numbers) in subnetworks
            if !isdisjoint(bus_numbers, subnet_bus_numbers)
                push!(subnets, subnet)
            end
        end
        if length(subnets) > 1
            @error "Area $(PSY.get_name(area)) is connected to multiple subnetworks $(subnets)."
            throw(
                IS.ConflictingInputsError(
                    "AreaBalancePowerModel doesn't support systems with Areas distributed across multiple asynchronous areas",
                ))
        end
    end
    return
end

function initialize_system_expressions!(
    container::OptimizationContainer,
    network_model::NetworkModel{AreaBalancePowerModel},
    subnetworks::Dict{Int, Set{Int}},
    branch_models::BranchModelContainer,
    system::PSY.System,
    ::Dict{Int64, Set{Int64}},
)
    areas = get_available_components(network_model, PSY.Area, system)
    if isempty(areas)
        throw(
            IS.ConflictingInputsError(
                "AreaBalancePowerModel doesn't support systems with no defined Areas",
            ),
        )
    end
    area_interchanges = PSY.get_available_components(PSY.AreaInterchange, system)
    if isempty(area_interchanges) || !haskey(branch_models, :AreaInterchange)
        @warn "The system does not contain any AreaInterchanges. The model won't have any power flowing between the areas."
    end
    if !isempty(area_interchanges) && !haskey(branch_models, :AreaInterchange)
        @warn "AreaInterchanges are not included in the model template. The model won't have any power flowing between the areas."
    end
    _verify_area_subnetwork_topology(system, subnetworks)
    _make_system_expressions!(container, subnetworks, AreaBalancePowerModel, areas)
    return
end

function initialize_system_expressions!(
    container::OptimizationContainer,
    network_model::NetworkModel{T},
    subnetworks::Dict{Int, Set{Int}},
    ::BranchModelContainer,
    system::PSY.System,
    bus_reduction_map::Dict{Int64, Set{Int64}},
) where {T <: AreaPTDFPowerModel}
    areas = get_available_components(network_model, PSY.Area, system)
    if isempty(areas)
        throw(
            IS.ConflictingInputsError(
                "AreaPTDFPowerModel doesn't support systems with no Areas",
            ),
        )
    end
    dc_bus_numbers = [
        PSY.get_number(b) for
        b in get_available_components(network_model, PSY.DCBus, system)
    ]
    _make_system_expressions!(
        container,
        subnetworks,
        dc_bus_numbers,
        AreaPTDFPowerModel,
        areas,
        bus_reduction_map,
    )
    return
end

function initialize_hvdc_system!(
    container::OptimizationContainer,
    network_model::NetworkModel{T},
    dc_model::Nothing,
    system::PSY.System,
) where {T <: PM.AbstractPowerModel}
    dc_buses = get_available_components(network_model, PSY.DCBus, system)
    if !isempty(dc_buses)
        @warn "HVDC Network Model is set to 'Nothing' but DC Buses are present in the system. \
               Consider adding an HVDC Network Model or removing DC Buses from the system."
    end
    return
end

function initialize_hvdc_system!(
    container::OptimizationContainer,
    network_model::NetworkModel{T},
    dc_model::U,
    system::PSY.System,
) where {T <: PM.AbstractPowerModel, U <: TransportHVDCNetworkModel}
    dc_buses = get_available_components(network_model, PSY.DCBus, system)
    @assert !isempty(dc_buses) "No DC buses found in the system. Consider adding DC Buses or removing HVDC network model."
    dc_bus_numbers = sort(PSY.get_number.(dc_buses))
    container.expressions[ExpressionKey(ActivePowerBalance, PSY.DCBus)] =
        _make_container_array(dc_bus_numbers, get_time_steps(container))
    return
end

function initialize_hvdc_system!(
    container::OptimizationContainer,
    network_model::NetworkModel{T},
    dc_model::U,
    system::PSY.System,
) where {T <: PM.AbstractPowerModel, U <: VoltageDispatchHVDCNetworkModel}
    dc_buses = get_available_components(network_model, PSY.DCBus, system)
    @assert !isempty(dc_buses) "No DC buses found in the system. Consider adding DC Buses or removing HVDC network model."
    dc_bus_numbers = sort(PSY.get_number.(dc_buses))
    container.expressions[ExpressionKey(DCCurrentBalance, PSY.DCBus)] =
        _make_container_array(dc_bus_numbers, get_time_steps(container))
    add_variable!(container, DCVoltage(), dc_buses, dc_model)
    return
end

function build_impl!(
    container::OptimizationContainer,
    template::ProblemTemplate,
    sys::PSY.System,
)
    transmission = get_network_formulation(template)
    transmission_model = get_network_model(template)
    hvdc_model = get_hvdc_network_model(template)

    initialize_system_expressions!(
        container,
        get_network_model(template),
        transmission_model.subnetworks,
        get_branch_models(template),
        sys,
        transmission_model.network_reduction.bus_reduction_map)

    initialize_hvdc_system!(
        container,
        transmission_model,
        hvdc_model,
        sys,
    )

    # Order is required
    for device_model in values(template.devices)
        @debug "Building Arguments for $(get_component_type(device_model)) with $(get_formulation(device_model)) formulation" _group =
            LOG_GROUP_OPTIMIZATION_CONTAINER
        TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "$(get_component_type(device_model))" begin
            construct_device!(
                container,
                sys,
                ArgumentConstructStage(),
                device_model,
                transmission_model,
            )
            @debug "Problem size:" get_problem_size(container) _group =
                LOG_GROUP_OPTIMIZATION_CONTAINER
        end
    end

    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Services" begin
        construct_services!(
            container,
            sys,
            ArgumentConstructStage(),
            get_service_models(template),
            get_device_models(template),
            transmission_model,
        )
    end

    # # Sort branch models so that those with time series parameters (e.g. DLR) come first.
    # # This ensures that DLR-aware constraint builders claim shared arcs before static builders,
    # # preventing static constraints from overriding DLR constraints for parallel branches of
    # # different types sharing the same arc.
    # sorted_branch_models = sort(
    #     collect(values(template.branches));
    #     by = b -> isempty(get_time_series_names(b)) ? 1 : 0,
    # )
    for branch_model in values(template.branches)
        @debug "Building Arguments for $(get_component_type(branch_model)) with $(get_formulation(branch_model)) formulation" _group =
            LOG_GROUP_OPTIMIZATION_CONTAINER
        TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "$(get_component_type(branch_model))" begin
            construct_device!(
                container,
                sys,
                ArgumentConstructStage(),
                branch_model,
                transmission_model,
            )
            @debug "Problem size:" get_problem_size(container) _group =
                LOG_GROUP_OPTIMIZATION_CONTAINER
        end
    end
    for device_model in values(template.devices)
        @debug "Building Model for $(get_component_type(device_model)) with $(get_formulation(device_model)) formulation" _group =
            LOG_GROUP_OPTIMIZATION_CONTAINER
        TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "$(get_component_type(device_model))" begin
            construct_device!(
                container,
                sys,
                ModelConstructStage(),
                device_model,
                transmission_model,
            )
            @debug "Problem size:" get_problem_size(container) _group =
                LOG_GROUP_OPTIMIZATION_CONTAINER
        end
    end

    # This function should be called after construct_device ModelConstructStage
    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "$(transmission)" begin
        @debug "Building $(transmission) network formulation" _group =
            LOG_GROUP_OPTIMIZATION_CONTAINER
        construct_network!(container, sys, transmission_model, template)
        construct_hvdc_network!(container, sys, transmission_model, hvdc_model, template)
        @debug "Problem size:" get_problem_size(container) _group =
            LOG_GROUP_OPTIMIZATION_CONTAINER
    end
    for branch_model in values(template.branches)
        @debug "Building Model for $(get_component_type(branch_model)) with $(get_formulation(branch_model)) formulation" _group =
            LOG_GROUP_OPTIMIZATION_CONTAINER
        TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "$(get_component_type(branch_model))" begin
            construct_device!(
                container,
                sys,
                ModelConstructStage(),
                branch_model,
                transmission_model,
            )
            @debug "Problem size:" get_problem_size(container) _group =
                LOG_GROUP_OPTIMIZATION_CONTAINER
        end
    end

    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Services" begin
        construct_services!(
            container,
            sys,
            ModelConstructStage(),
            get_service_models(template),
            get_device_models(template),
            transmission_model,
        )
    end

    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Objective" begin
        @debug "Building Objective" _group = LOG_GROUP_OPTIMIZATION_CONTAINER
        update_objective_function!(container)
    end
    @debug "Total operation count $(PSI.get_jump_model(container).operator_counter)" _group =
        LOG_GROUP_OPTIMIZATION_CONTAINER

    TimerOutputs.@timeit BUILD_PROBLEMS_TIMER "Power Flow Initialization" begin
        add_power_flow_data!(container, get_power_flow_evaluation(transmission_model), sys)
    end
    check_optimization_container(container)
    return
end

function update_objective_function!(container::OptimizationContainer)
    JuMP.@objective(
        get_jump_model(container),
        get_sense(container.objective_function),
        get_objective_expression(container.objective_function)
    )
    return
end

"""
Default solve method for OptimizationContainer
"""
function solve_impl!(container::OptimizationContainer, system::PSY.System)
    optimizer_stats = get_optimizer_stats(container)

    jump_model = get_jump_model(container)

    model_status = MOI.NO_SOLUTION::MOI.ResultStatusCode
    conflict_status = MOI.COMPUTE_CONFLICT_NOT_CALLED

    try_count = 0
    while model_status != MOI.FEASIBLE_POINT::MOI.ResultStatusCode
        _,
        optimizer_stats.timed_solve_time,
        optimizer_stats.solve_bytes_alloc,
        optimizer_stats.sec_in_gc = @timed JuMP.optimize!(jump_model)
        model_status = JuMP.primal_status(jump_model)

        if model_status != MOI.FEASIBLE_POINT::MOI.ResultStatusCode
            if get_calculate_conflict(get_settings(container))
                @warn "Optimizer returned $model_status computing conflict"
                conflict_status = compute_conflict!(container)
                if conflict_status == MOI.CONFLICT_FOUND
                    return RunStatus.FAILED
                end
            else
                @warn "Optimizer returned $model_status trying optimize! again"
            end

            try_count += 1
            if try_count > MAX_OPTIMIZE_TRIES
                @error "Optimizer returned $model_status after $MAX_OPTIMIZE_TRIES optimize! attempts"
                return RunStatus.FAILED
            end
        end
    end

    # Order is important because if a dual is needed then it could move the results to the
    # temporary primal container
    _, optimizer_stats.timed_calculate_aux_variables =
        @timed calculate_aux_variables!(container, system)

    # Needs to be called here to avoid issues when getting duals from MILPs
    write_optimizer_stats!(container)

    _, optimizer_stats.timed_calculate_dual_variables =
        @timed calculate_dual_variables!(container, system, is_milp(container))

    return RunStatus.SUCCESSFULLY_FINALIZED
end

function compute_conflict!(container::OptimizationContainer)
    jump_model = get_jump_model(container)
    settings = get_settings(container)
    JuMP.unset_silent(jump_model)
    jump_model.is_model_dirty = false
    conflict = container.infeasibility_conflict
    try
        JuMP.compute_conflict!(jump_model)
        conflict_status = MOI.get(jump_model, MOI.ConflictStatus())
        if conflict_status != MOI.CONFLICT_FOUND
            @error "No conflict could be found for the model. Status: $conflict_status"
            if !get_optimizer_solve_log_print(settings)
                JuMP.set_silent(jump_model)
            end
            return conflict_status
        end

        for (key, field_container) in get_constraints(container)
            conflict_indices = check_conflict_status(jump_model, field_container)
            if isempty(conflict_indices)
                @info "Conflict Index returned empty for $key"
                continue
            else
                conflict[ISOPT.encode_key(key)] = conflict_indices
            end
        end

        msg = IOBuffer()
        for (k, v) in conflict
            PrettyTables.pretty_table(msg, v; header = [k])
        end

        @error "Constraints participating in conflict basis (IIS) \n\n$(String(take!(msg)))"

        return conflict_status
    catch e
        jump_model.is_model_dirty = true
        if isa(e, MethodError)
            @info "Can't compute conflict, check that your optimizer supports conflict refining/IIS"
        else
            @error "Can't compute conflict" exception = (e, catch_backtrace())
        end
    end

    return MOI.NO_CONFLICT_EXISTS
end

function write_optimizer_stats!(container::OptimizationContainer)
    write_optimizer_stats!(get_optimizer_stats(container), get_jump_model(container))
    return
end

"""
Exports the OpModel JuMP object in MathOptFormat
"""
function serialize_optimization_model(container::OptimizationContainer, save_path::String)
    serialize_jump_optimization_model(get_jump_model(container), save_path)
    return
end

function serialize_metadata!(container::OptimizationContainer, output_dir::String)
    for key in Iterators.flatten((
        keys(container.constraints),
        keys(container.duals),
        keys(container.parameters),
        keys(container.variables),
        keys(container.aux_variables),
        keys(container.expressions),
    ))
        encoded_key = encode_key_as_string(key)
        if ISOPT.has_container_key(container.metadata, encoded_key)
            # Constraints and Duals can store the same key.
            IS.@assert_op key ==
                          ISOPT.get_container_key(container.metadata, encoded_key)
        end
        ISOPT.add_container_key!(container.metadata, encoded_key, key)
    end

    filename = ISOPT._make_metadata_filename(output_dir)
    Serialization.serialize(filename, container.metadata)
    @debug "Serialized container keys to $filename" _group = IS.LOG_GROUP_SERIALIZATION
end

function deserialize_metadata!(
    container::OptimizationContainer,
    output_dir::String,
    model_name,
)
    merge!(
        container.metadata.container_key_lookup,
        deserialize_metadata(
            ISOPT.OptimizationContainerMetadata,
            output_dir,
            model_name,
        ),
    )
    return
end

# PERF: compilation hotspot. from string conversion at the container[key] = value line?
function _assign_container!(container::OrderedDict, key::OptimizationContainerKey, value)
    if haskey(container, key)
        @error "$(ISOPT.encode_key(key)) is already stored" sort!(
            ISOPT.encode_key.(keys(container)),
        )
        throw(IS.InvalidValue("$key is already stored"))
    end
    container[key] = value
    @debug "Added container entry $(typeof(key)) $(ISOPT.encode_key(key))" _group =
        LOG_GROUP_OPTIMZATION_CONTAINER
    return
end

####################################### Variable Container #################################
function _add_variable_container!(
    container::OptimizationContainer,
    var_key::VariableKey{T, U},
    sparse::Bool,
    axs...,
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    if sparse
        var_container = sparse_container_spec(JuMP.VariableRef, axs...)
    else
        var_container = container_spec(JuMP.VariableRef, axs...)
    end
    _assign_container!(container.variables, var_key, var_container)
    return var_container
end

function add_variable_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    var_key = VariableKey(T, U, meta)
    return _add_variable_container!(container, var_key, sparse, axs...)
end

function add_variable_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta::String,
    axs...;
    sparse = false,
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    var_key = VariableKey(T, U, meta)
    return _add_variable_container!(container, var_key, sparse, axs...)
end

function _get_pwl_variables_container()
    contents = Dict{Tuple{String, Int, Int}, Any}()
    return SparseAxisArray(contents)
end

function add_variable_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U};
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: SparseVariableType, U <: Union{PSY.Component, PSY.System}}
    var_key = VariableKey(T, U, meta)
    _assign_container!(container.variables, var_key, _get_pwl_variables_container())
    return container.variables[var_key]
end

function get_variable_keys(container::OptimizationContainer)
    return collect(keys(container.variables))
end

function get_variable(container::OptimizationContainer, key::VariableKey)
    var = get(container.variables, key, nothing)
    if var === nothing
        name = ISOPT.encode_key(key)
        keys = ISOPT.encode_key.(get_variable_keys(container))
        throw(IS.InvalidValue("variable $name is not stored. $keys"))
    end
    return var
end

function get_variable(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta::String = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    return get_variable(container, VariableKey(T, U, meta))
end

##################################### AuxVariable Container ################################
function add_aux_variable_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: AuxVariableType, U <: PSY.Component}
    var_key = AuxVarKey(T, U, meta)
    if sparse
        aux_variable_container = sparse_container_spec(Float64, axs...)
    else
        aux_variable_container = container_spec(Float64, axs...)
    end
    _assign_container!(container.aux_variables, var_key, aux_variable_container)
    return aux_variable_container
end

function get_aux_variable_keys(container::OptimizationContainer)
    return collect(keys(container.aux_variables))
end

function get_aux_variable(container::OptimizationContainer, key::AuxVarKey)
    aux = get(container.aux_variables, key, nothing)
    if aux === nothing
        name = ISOPT.encode_key(key)
        keys = ISOPT.encode_key.(get_aux_variable_keys(container))
        throw(IS.InvalidValue("Auxiliary variable $name is not stored. $keys"))
    end
    return aux
end

function get_aux_variable(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta::String = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: AuxVariableType, U <: PSY.Component}
    return get_aux_variable(container, AuxVarKey(T, U, meta))
end

##################################### DualVariable Container ################################
function add_dual_container!(
    container::OptimizationContainer,
    ::Type{T},
    ::Type{U},
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    if is_milp(container)
        @warn("The model has resulted in a MILP, \\
              dual value retrieval requires solving an additional Linear Program \\
              which increases simulation time and the results could be innacurate.")
    end
    const_key = ConstraintKey(T, U, meta)
    if sparse
        dual_container = sparse_container_spec(Float64, axs...)
    else
        dual_container = container_spec(Float64, axs...)
    end
    _assign_container!(container.duals, const_key, dual_container)
    return dual_container
end

function get_dual_keys(container::OptimizationContainer)
    return collect(keys(container.duals))
end

##################################### Constraint Container #################################
function _add_constraints_container!(
    container::OptimizationContainer,
    cons_key::ConstraintKey,
    axs...;
    sparse = false,
)
    if sparse
        cons_container = sparse_container_spec(JuMP.ConstraintRef, axs...)
    else
        cons_container = container_spec(JuMP.ConstraintRef, axs...)
    end
    _assign_container!(container.constraints, cons_key, cons_container)
    return cons_container
end

function add_constraints_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    cons_key = ConstraintKey(T, U, meta)
    return _add_constraints_container!(container, cons_key, axs...; sparse = sparse)
end

function get_constraint_keys(container::OptimizationContainer)
    return collect(keys(container.constraints))
end

function get_constraint(container::OptimizationContainer, key::ConstraintKey)
    var = get(container.constraints, key, nothing)
    if var === nothing
        name = ISOPT.encode_key(key)
        keys = ISOPT.encode_key.(get_constraint_keys(container))
        throw(IS.InvalidValue("constraint $name is not stored. $keys"))
    end

    return var
end

function get_constraint(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta::String = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    return get_constraint(container, ConstraintKey(T, U, meta))
end

function read_duals(container::OptimizationContainer)
    return Dict(k => to_dataframe(jump_value.(v), k) for (k, v) in get_duals(container))
end

##################################### Parameter Container ##################################
function _add_param_container!(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
    attribute::VariableValueAttributes{<:OptimizationContainerKey},
    param_type::DataType,
    axs...;
    sparse = false,
) where {T <: VariableValueParameter, U <: PSY.Component}
    if sparse
        param_array = sparse_container_spec(param_type, axs...)
        multiplier_array = sparse_container_spec(Float64, axs...)
    else
        param_array = DenseAxisArray{param_type}(undef, axs...)
        multiplier_array = fill!(DenseAxisArray{Float64}(undef, axs...), NaN)
    end
    param_container = ParameterContainer(attribute, param_array, multiplier_array)
    _assign_container!(container.parameters, key, param_container)
    return param_container
end

function _add_param_container!(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
    attribute::VariableValueAttributes{<:OptimizationContainerKey},
    axs...;
    sparse = false,
) where {T <: VariableValueParameter, U <: PSY.Component}
    if built_for_recurrent_solves(container) && !get_rebuild_model(get_settings(container))
        param_type = JuMP.VariableRef
    else
        param_type = Float64
    end
    return _add_param_container!(
        container,
        key,
        attribute,
        param_type,
        axs...;
        sparse = sparse,
    )
end

function _add_param_container!(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
    attribute::TimeSeriesAttributes{V},
    param_axs,
    multiplier_axs,
    additional_axs,
    time_steps::UnitRange{Int};
    sparse = false,
) where {T <: TimeSeriesParameter, U <: PSY.Component, V <: PSY.TimeSeriesData}
    if built_for_recurrent_solves(container) && !get_rebuild_model(get_settings(container))
        param_type = JuMP.VariableRef
    else
        param_type = Float64
    end
    if sparse
        param_array =
            sparse_container_spec(param_type, param_axs, additional_axs..., time_steps)
        multiplier_array =
            sparse_container_spec(Float64, multiplier_axs, additional_axs..., time_steps)
    else
        param_array =
            DenseAxisArray{param_type}(undef, param_axs, additional_axs..., time_steps)
        multiplier_array =
            fill!(
                DenseAxisArray{Float64}(
                    undef,
                    multiplier_axs,
                    additional_axs...,
                    time_steps,
                ),
                NaN,
            )
    end
    param_container = ParameterContainer(attribute, param_array, multiplier_array)
    _assign_container!(container.parameters, key, param_container)
    return param_container
end

function _add_param_container!(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
    attribute::EventParametersAttributes{V, T},
    param_axs,
    time_steps;
    sparse = false,
) where {T <: EventParameter, U <: PSY.Component, V <: PSY.Contingency}
    if built_for_recurrent_solves(container) && !get_rebuild_model(get_settings(container))
        param_type = JuMP.VariableRef
    else
        param_type = Float64
    end

    if sparse
        error("Sparse parameter container is not supported for $V")
    else
        param_array = DenseAxisArray{param_type}(undef, param_axs, time_steps)
        multiplier_array =
            fill!(DenseAxisArray{Float64}(undef, param_axs, time_steps), NaN)
    end
    param_container = ParameterContainer(attribute, param_array, multiplier_array)
    _assign_container!(container.parameters, key, param_container)
    return param_container
end

function _add_param_container!(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
    attributes::CostFunctionAttributes{R},
    axs...;
    sparse = false,
) where {R, T <: ObjectiveFunctionParameter, U <: PSY.Component}
    if sparse
        param_array = sparse_container_spec(R, axs...)
        multiplier_array = sparse_container_spec(Float64, axs...)
    else
        param_array = DenseAxisArray{R}(undef, axs...)
        multiplier_array = fill!(DenseAxisArray{Float64}(undef, axs...), NaN)
    end
    param_container = ParameterContainer(attributes, param_array, multiplier_array)
    _assign_container!(container.parameters, key, param_container)
    return param_container
end

function add_param_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    ::Type{V},
    name::String,
    param_axs,
    multiplier_axs,
    additional_axs,
    time_steps::UnitRange{Int};
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: TimeSeriesParameter, U <: PSY.Component, V <: PSY.TimeSeriesData}
    param_key = ParameterKey(T, U, meta)
    if isabstracttype(V)
        error("$V can't be abstract: $param_key")
    end
    attributes = TimeSeriesAttributes(V, name)
    return _add_param_container!(
        container,
        param_key,
        attributes,
        param_axs,
        multiplier_axs,
        additional_axs,
        time_steps;
        sparse = sparse,
    )
end

function add_param_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    variable_types::Tuple{Vararg{Type}},
    sos_variable::SOSStatusVariable = NO_VARIABLE,
    uses_compact_power::Bool = false,
    data_type::DataType = Float64,
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ObjectiveFunctionParameter, U <: PSY.Component}
    param_key = ParameterKey(T, U, meta)
    attributes =
        CostFunctionAttributes{data_type}(variable_types, sos_variable, uses_compact_power)
    return _add_param_container!(container, param_key, attributes, axs...; sparse = sparse)
end

function add_param_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    ::Type{V},
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: EventParameter, U <: PSY.Component, V <: PSY.Contingency}
    param_key = ParameterKey(T, U, meta)
    attributes = EventParametersAttributes{V, T}(U[])
    return _add_param_container!(container, param_key, attributes, axs...; sparse = sparse)
end

function add_param_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    source_key::V,
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: VariableValueParameter, U <: PSY.Component, V <: OptimizationContainerKey}
    param_key = ParameterKey(T, U, meta)
    attributes = VariableValueAttributes(source_key)
    return _add_param_container!(container, param_key, attributes, axs...; sparse = sparse)
end

# FixValue parameters are created using Float64 since we employ JuMP.fix to fix the downstream
# variables.
function add_param_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    source_key::V,
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: FixValueParameter, U <: PSY.Component, V <: OptimizationContainerKey}
    if meta == ISOPT.CONTAINER_KEY_EMPTY_META
        error("$T parameters require passing the VariableType to the meta field")
    end
    param_key = ParameterKey(T, U, meta)
    attributes = VariableValueAttributes(source_key)
    return _add_param_container!(
        container,
        param_key,
        attributes,
        Float64,
        axs...;
        sparse = sparse,
    )
end

function get_parameter_keys(container::OptimizationContainer)
    return collect(keys(container.parameters))
end

function get_parameter(container::OptimizationContainer, key::ParameterKey)
    param_container = get(container.parameters, key, nothing)
    if param_container === nothing
        name = ISOPT.encode_key(key)
        throw(
            IS.InvalidValue(
                "parameter $name is not stored. $(collect(keys(container.parameters)))",
            ),
        )
    end
    return param_container
end

function get_parameter(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_parameter(container, ParameterKey(T, U, meta))
end

function get_parameter_array(container::OptimizationContainer, key)
    return get_parameter_array(get_parameter(container, key))
end

function get_parameter_array(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_parameter_array(get_parameter(container, key))
end

function get_parameter_multiplier_array(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_multiplier_array(get_parameter(container, key))
end

function get_parameter_attributes(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_attributes(get_parameter(container, key))
end

function get_parameter_array(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_parameter_array(container, ParameterKey(T, U, meta))
end
function get_parameter_multiplier_array(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_multiplier_array(get_parameter(container, ParameterKey(T, U, meta)))
end

function get_parameter_attributes(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ParameterType, U <: Union{PSY.Component, PSY.System}}
    return get_attributes(get_parameter(container, ParameterKey(T, U, meta)))
end

# Slow implementation not to be used in hot loops
function read_parameters(container::OptimizationContainer)
    params_dict = Dict{ParameterKey, DenseAxisArray}()
    parameters = get_parameters(container)
    (parameters === nothing || isempty(parameters)) && return params_dict
    for (k, v) in parameters
        # TODO: all functions similar to calculate_parameter_values should be in one
        # place and be consistent in behavior.
        #params_dict[k] = to_dataframe(calculate_parameter_values(v))
        param_array = get_parameter_values(v)
        multiplier_array = get_multiplier_array(v)
        params_dict[k] = _calculate_parameter_values(k, param_array, multiplier_array)
    end
    return params_dict
end

function _calculate_parameter_values(
    ::ParameterKey{<:ParameterType, <:PSY.Component},
    param_array,
    multiplier_array,
)
    return param_array .* multiplier_array
end

function _calculate_parameter_values(
    ::ParameterKey{<:ObjectiveFunctionParameter, <:PSY.Component},
    param_array,
    multiplier_array,
)
    return param_array
end
##################################### Expression Container #################################
function _add_expression_container!(
    container::OptimizationContainer,
    expr_key::ExpressionKey,
    ::Type{T},
    axs...;
    sparse = false,
) where {T <: JuMP.AbstractJuMPScalar}
    if sparse
        expr_container = sparse_container_spec(T, axs...)
    else
        expr_container = container_spec(T, axs...)
    end
    remove_undef!(expr_container)
    _assign_container!(container.expressions, expr_key, expr_container)
    return expr_container
end

function add_expression_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    axs...;
    expr_type = GAE,
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ExpressionType, U <: Union{PSY.Component, PSY.System}}
    expr_key = ExpressionKey(T, U, meta)
    return _add_expression_container!(
        container,
        expr_key,
        expr_type,
        axs...;
        sparse = sparse,
    )
end

function add_expression_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    axs...;
    sparse = false,
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: CostExpressions, U <: Union{PSY.Component, PSY.System}}
    expr_key = ExpressionKey(T, U, meta)
    expr_type = JuMP.QuadExpr
    return _add_expression_container!(
        container,
        expr_key,
        expr_type,
        axs...;
        sparse = sparse,
    )
end

function get_expression_keys(container::OptimizationContainer)
    return collect(keys(container.expressions))
end

function get_expression(container::OptimizationContainer, key::ExpressionKey)
    var = get(container.expressions, key, nothing)
    if var === nothing
        throw(
            IS.InvalidValue(
                "expression $key is not stored. $(collect(keys(container.expressions)))",
            ),
        )
    end

    return var
end

function get_expression(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ExpressionType, U <: Union{PSY.Component, PSY.System}}
    return get_expression(container, ExpressionKey(T, U, meta))
end

function read_expressions(container::OptimizationContainer)
    return Dict(
        k => to_dataframe(jump_value.(v), k) for (k, v) in get_expressions(container) if
        !(get_entry_type(k) <: SystemBalanceExpressions)
    )
end

###################################Initial Conditions Containers############################
function _add_initial_condition_container!(
    container::OptimizationContainer,
    ic_key::InitialConditionKey{T, U},
    length_devices::Int,
) where {T <: InitialConditionType, U <: Union{PSY.Component, PSY.System}}
    if built_for_recurrent_solves(container) && !get_rebuild_model(get_settings(container))
        ini_conds = Vector{
            Union{InitialCondition{T, JuMP.VariableRef}, InitialCondition{T, Nothing}},
        }(
            undef,
            length_devices,
        )
    else
        ini_conds =
            Vector{Union{InitialCondition{T, Float64}, InitialCondition{T, Nothing}}}(
                undef,
                length_devices,
            )
    end
    _assign_container!(container.initial_conditions, ic_key, ini_conds)
    return ini_conds
end

function add_initial_condition_container!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    axs;
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: InitialConditionType, U <: Union{PSY.Component, PSY.System}}
    ic_key = InitialConditionKey(T, U, meta)
    @debug "add_initial_condition_container" ic_key _group = LOG_GROUP_SERVICE_CONSTUCTORS
    return _add_initial_condition_container!(container, ic_key, length(axs))
end

function get_initial_condition(
    container::OptimizationContainer,
    ::T,
    ::Type{D},
) where {T <: InitialConditionType, D <: PSY.Component}
    return get_initial_condition(container, InitialConditionKey(T, D))
end

function get_initial_condition(container::OptimizationContainer, key::InitialConditionKey)
    initial_conditions = get(container.initial_conditions, key, nothing)
    if initial_conditions === nothing
        throw(IS.InvalidValue("initial conditions are not stored for $(key)"))
    end
    return initial_conditions
end

function get_initial_conditions_keys(container::OptimizationContainer)
    return collect(keys(container.initial_conditions))
end

function write_initial_conditions_data!(
    container::OptimizationContainer,
    ic_container::OptimizationContainer,
)
    for field in STORE_CONTAINERS
        ic_container_dict = getfield(ic_container, field)
        if field == STORE_CONTAINER_PARAMETERS
            ic_container_dict = read_parameters(ic_container)
        end
        if field == STORE_CONTAINER_EXPRESSIONS
            continue
        end
        isempty(ic_container_dict) && continue
        ic_data_dict = getfield(get_initial_conditions_data(container), field)
        for (key, field_container) in ic_container_dict
            @debug "Adding $(encode_key_as_string(key)) to InitialConditionsData" _group =
                LOG_GROUP_SERVICE_CONSTUCTORS
            if field == STORE_CONTAINER_PARAMETERS
                ic_data_dict[key] = ic_container_dict[key]
            else
                ic_data_dict[key] = jump_value.(field_container)
            end
        end
    end
    return
end

# Note: These methods aren't passing the potential meta fields in the keys
function get_initial_conditions_variable(
    container::OptimizationContainer,
    type::VariableType,
    ::Type{T},
) where {T <: Union{PSY.Component, PSY.System}}
    return get_initial_conditions_variable(get_initial_conditions_data(container), type, T)
end

function get_initial_conditions_aux_variable(
    container::OptimizationContainer,
    type::AuxVariableType,
    ::Type{T},
) where {T <: Union{PSY.Component, PSY.System}}
    return get_initial_conditions_aux_variable(
        get_initial_conditions_data(container),
        type,
        T,
    )
end

function get_initial_conditions_dual(
    container::OptimizationContainer,
    type::ConstraintType,
    ::Type{T},
) where {T <: Union{PSY.Component, PSY.System}}
    return get_initial_conditions_dual(get_initial_conditions_data(container), type, T)
end

function get_initial_conditions_parameter(
    container::OptimizationContainer,
    type::ParameterType,
    ::Type{T},
) where {T <: Union{PSY.Component, PSY.System}}
    return get_initial_conditions_parameter(get_initial_conditions_data(container), type, T)
end

function add_to_objective_invariant_expression!(
    container::OptimizationContainer,
    cost_expr::T,
) where {T <: JuMP.AbstractJuMPScalar}
    T_cf = typeof(container.objective_function.invariant_terms)
    if T_cf <: JuMP.GenericAffExpr && T <: JuMP.GenericQuadExpr
        container.objective_function.invariant_terms += cost_expr
    else
        JuMP.add_to_expression!(container.objective_function.invariant_terms, cost_expr)
    end
    return
end

function add_to_objective_variant_expression!(
    container::OptimizationContainer,
    cost_expr::JuMP.AffExpr,
)
    JuMP.add_to_expression!(container.objective_function.variant_terms, cost_expr)
    return
end

function deserialize_key(container::OptimizationContainer, name::AbstractString)
    return deserialize_key(container.metadata, name)
end

function calculate_aux_variables!(container::OptimizationContainer, system::PSY.System)
    aux_var_keys = keys(get_aux_variables(container))
    pf_aux_var_keys = filter(is_from_power_flow ∘ get_entry_type, aux_var_keys)
    non_pf_aux_var_keys = setdiff(aux_var_keys, pf_aux_var_keys)
    # We should only have power flow aux vars if we have power flow evaluators
    @assert isempty(pf_aux_var_keys) || !isempty(get_power_flow_evaluation_data(container))

    TimerOutputs.@timeit RUN_SIMULATION_TIMER "Power Flow Evaluation" begin
        reset_power_flow_is_solved!(container)
        # Power flow-related aux vars get calculated once per power flow
        for (i, pf_e_data) in enumerate(get_power_flow_evaluation_data(container))
            @debug "Processing power flow $i"
            solve_power_flow!(pf_e_data, container)
            for key in pf_aux_var_keys
                calculate_aux_variable_value!(container, key, system)
            end
        end
    end

    # Other aux vars get calculated once at the end
    for key in non_pf_aux_var_keys
        calculate_aux_variable_value!(container, key, system)
    end
    return RunStatus.SUCCESSFULLY_FINALIZED
end

function _calculate_dual_variable_value!(
    container::OptimizationContainer,
    key::ConstraintKey{CopperPlateBalanceConstraint, PSY.System},
    ::PSY.System,
)
    constraint_container = get_constraint(container, key)
    dual_variable_container = get_duals(container)[key]

    for subnet in axes(constraint_container)[1], t in axes(constraint_container)[2]
        # See https://jump.dev/JuMP.jl/stable/manual/solutions/#Dual-solution-values
        dual_variable_container[subnet, t] = jump_value(constraint_container[subnet, t])
    end
    return
end

function _calculate_dual_variable_value!(
    container::OptimizationContainer,
    key::ConstraintKey{T, D},
    ::PSY.System,
) where {T <: ConstraintType, D <: Union{PSY.Component, PSY.System}}
    constraint_duals = jump_value.(get_constraint(container, key))
    dual_variable_container = get_duals(container)[key]

    # Needs to loop since the container ordering might not match in the DenseAxisArray
    for index in Iterators.product(axes(constraint_duals)...)
        dual_variable_container[index...] = constraint_duals[index...]
    end

    return
end

function _calculate_dual_variables_continous_model!(
    container::OptimizationContainer,
    system::PSY.System,
)
    duals_vars = get_duals(container)
    for key in keys(duals_vars)
        _calculate_dual_variable_value!(container, key, system)
    end
    return RunStatus.SUCCESSFULLY_FINALIZED
end

function _calculate_dual_variables_discrete_model!(
    container::OptimizationContainer,
    ::PSY.System,
)
    return process_duals(container, container.settings.optimizer)
end

function calculate_dual_variables!(
    container::OptimizationContainer,
    sys::PSY.System,
    is_milp::Bool,
)
    isempty(get_duals(container)) && return RunStatus.SUCCESSFULLY_FINALIZED
    if is_milp
        status = _calculate_dual_variables_discrete_model!(container, sys)
    else
        status = _calculate_dual_variables_continous_model!(container, sys)
    end
    return
end

########################### Helper Functions to get keys ###################################
function get_optimization_container_key(
    ::T,
    ::Type{U},
    meta::String,
) where {T <: AuxVariableType, U <: PSY.Component}
    return AuxVarKey(T, U, meta)
end

function get_optimization_container_key(
    ::T,
    ::Type{U},
    meta::String,
) where {T <: VariableType, U <: PSY.Component}
    return VariableKey(T, U, meta)
end

function get_optimization_container_key(
    ::T,
    ::Type{U},
    meta::String,
) where {T <: ParameterType, U <: PSY.Component}
    return ParameterKey(T, U, meta)
end

function get_optimization_container_key(
    ::T,
    ::Type{U},
    meta::String,
) where {T <: ConstraintType, U <: PSY.Component}
    return ConstraintKey(T, U, meta)
end

function lazy_container_addition!(
    container::OptimizationContainer,
    var::T,
    ::Type{U},
    axs...;
    kwargs...,
) where {T <: VariableType, U <: Union{PSY.Component, PSY.System}}
    if !has_container_key(container, T, U)
        var_container = add_variable_container!(container, var, U, axs...; kwargs...)
    else
        var_container = get_variable(container, var, U)
    end
    return var_container
end

function lazy_container_addition!(
    container::OptimizationContainer,
    constraint::T,
    ::Type{U},
    axs...;
    kwargs...,
) where {T <: ConstraintType, U <: Union{PSY.Component, PSY.System}}
    meta = get(kwargs, :meta, ISOPT.CONTAINER_KEY_EMPTY_META)
    if !has_container_key(container, T, U, meta)
        cons_container =
            add_constraints_container!(container, constraint, U, axs...; kwargs...)
    else
        cons_container = get_constraint(container, constraint, U, meta)
    end
    return cons_container
end

function lazy_container_addition!(
    container::OptimizationContainer,
    expression::T,
    ::Type{U},
    axs...;
    kwargs...,
) where {T <: ExpressionType, U <: Union{PSY.Component, PSY.System}}
    meta = get(kwargs, :meta, IS.Optimization.CONTAINER_KEY_EMPTY_META)
    if !has_container_key(container, T, U, meta)
        expr_container =
            add_expression_container!(container, expression, U, axs...; kwargs...)
    else
        expr_container = get_expression(container, expression, U, meta)
    end
    return expr_container
end

function get_time_series_initial_values!(
    container::OptimizationContainer,
    ::Type{T},
    component::PSY.Component,
    time_series_name::AbstractString,
) where {T <: PSY.TimeSeriesData}
    initial_time = get_initial_time(container)
    time_steps = get_time_steps(container)
    forecast = PSY.get_time_series(
        T,
        component,
        time_series_name;
        start_time = initial_time,
        count = 1,
    )
    ts_values = IS.get_time_series_values(
        component,
        forecast;
        start_time = initial_time,
        len = length(time_steps),
        ignore_scaling_factors = true,
    )
    return ts_values
end

"""
Get the column names for the specified container in the OptimizationContainer.

# Arguments
- `container::OptimizationContainer`: The optimization container.
- `field::Symbol`: The field for which to retrieve the column names.
- `key::OptimizationContainerKey`: The key for which to retrieve the column names.

# Returns
- `Tuple`: Tuple of Vector{String}.
"""
function get_column_names(
    ::OptimizationContainer,
    field::Symbol,
    subcontainer,
    key::OptimizationContainerKey,
)
    return if field == :parameters
        # Parameters are stored in ParameterContainer.
        get_column_names(key, subcontainer)
    else
        # The others are in DenseAxisArrays.
        get_column_names_from_axis_array(key, subcontainer)
    end
end

lookup_value(container::OptimizationContainer, key::VariableKey) =
    get_variable(container, key)
lookup_value(container::OptimizationContainer, key::ParameterKey) =
    calculate_parameter_values(get_parameter(container, key))
lookup_value(container::OptimizationContainer, key::AuxVarKey) =
    get_aux_variable(container, key)
lookup_value(container::OptimizationContainer, key::ExpressionKey) =
    get_expression(container, key)
lookup_value(container::OptimizationContainer, key::ConstraintKey) =
    get_constraint(container, key)
