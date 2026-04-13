# Defines the order of precedence for each type of information that could be sent to PowerFlows.jl
const PF_INPUT_KEY_PRECEDENCES = Dict(
    :active_power => [ActivePowerVariable, PowerOutput, ActivePowerTimeSeriesParameter],
    :reactive_power => [ReactivePowerVariable, ReactivePowerTimeSeriesParameter],
    :voltage_angle_export => [PowerFlowVoltageAngle, VoltageAngle],
    :voltage_magnitude_export => [PowerFlowVoltageMagnitude, VoltageMagnitude],
    :voltage_angle_opf => [VoltageAngle],
    :voltage_magnitude_opf => [VoltageMagnitude],
    :active_power_hvdc_pst_from_to =>
        [FlowActivePowerFromToVariable, FlowActivePowerVariable],
    :active_power_hvdc_pst_to_from =>
        [FlowActivePowerFromToVariable, FlowActivePowerVariable],
)

const RELEVANT_COMPONENTS_SELECTOR =
    PSY.make_selector(Union{PSY.StaticInjection, PSY.Bus, PSY.Branch})

function _add_aux_variables!(
    container::OptimizationContainer,
    component_map::Dict{Type{<:AuxVariableType}, <:Set{<:Tuple{DataType, Any}}},
)
    for (var_type, components) in pairs(component_map)
        component_types = unique(first.(components))
        for component_type in component_types
            component_names = [v for (k, v) in components if k <: component_type]
            sort!(component_names)
            add_aux_variable_container!(
                container,
                var_type(),
                component_type,
                component_names,
                get_time_steps(container),
            )
        end
    end
end

# Trait that determines which types of information are needed for each type of power flow
pf_input_keys(::PFS.ABAPowerFlowData) =
    [:active_power]
pf_input_keys(::PFS.PTDFPowerFlowData) =
    [:active_power]
pf_input_keys(::PFS.vPTDFPowerFlowData) =
    [:active_power]
pf_input_keys(::PFS.ACPowerFlowData) =
    [:active_power, :reactive_power, :voltage_angle_opf, :voltage_magnitude_opf]
pf_input_keys(::PFS.PSSEExporter) =
    [:active_power, :reactive_power, :voltage_angle_export, :voltage_magnitude_export]
pf_input_keys_hvdc_pst(::PFS.PowerFlowData) = DataType[]
pf_input_keys_hvdc_pst(::PFS.ACPowerFlowData) =
    [:active_power_hvdc_pst_from_to, :active_power_hvdc_pst_to_from]

_get_component_bus_for_map(component::PSY.Branch, ::Val{:from}) =
    PSY.get_from_bus(component)
_get_component_bus_for_map(component::PSY.Branch, ::Val{:to}) = PSY.get_to_bus(component)
_get_component_bus_for_map(component::PSY.Component, ::Nothing) = PSY.get_bus(component)

# Generalized function to create component maps by name to the index in the PowerFlowData bus arrays
function _make_temp_component_map(
    pf_data::PFS.PowerFlowData,
    sys::PSY.System,
    component_type::DataType,
    side::Union{Val{:from}, Val{:to}, Nothing},
)
    nrd = PFS.get_network_reduction_data(pf_data)
    temp_component_map = Dict{DataType, Dict{String, Int}}()
    components = PSY.get_available_components(component_type, sys)
    bus_lookup = PFS.get_bus_lookup(pf_data)
    for comp in components
        comp_type = typeof(comp)
        bus_dict = get!(temp_component_map, comp_type, Dict{String, Int}())
        bus_number = PSY.get_number(_get_component_bus_for_map(comp, side))
        bus_dict[PSY.get_name(comp)] = PNM.get_bus_index(bus_number, bus_lookup, nrd)
    end
    return temp_component_map
end

# Maps the StaticInjection component type by name to the
# index in the PowerFlow data arrays going from Bus number to bus index
function _make_temp_component_map(pf_data::PFS.PowerFlowData, sys::PSY.System)
    temp_component_map = _make_temp_component_map(
        pf_data,
        sys,
        PSY.StaticInjection,
        nothing,
    )
    # Add ACBus components for voltage magnitude and angle export
    bus_lookup = PFS.get_bus_lookup(pf_data)
    nrd = PFS.get_network_reduction_data(pf_data)
    temp_component_map[PSY.ACBus] =
        Dict(
            PSY.get_name(c) => PNM.get_bus_index(PSY.get_number(c), bus_lookup, nrd) for
            c in get_available_components(PSY.ACBus, sys)
        )
    return temp_component_map
end

_get_temp_component_map_lhs(comp::PSY.Component) = PSY.get_name(comp)
_get_temp_component_map_lhs(comp::PSY.Bus) = PSY.get_number(comp)

# Creates dicts of components by type
function _make_temp_component_map(::PFS.SystemPowerFlowContainer, sys::PSY.System)
    temp_component_map =
        Dict{DataType, Dict{Union{String, Int64}, String}}()
    relevant_components = PSY.get_available_components(RELEVANT_COMPONENTS_SELECTOR, sys)
    for comp_type in unique(typeof.(relevant_components))
        # NOTE we avoid using bus numbers here because PSY.get_bus(system, number) is O(n)
        temp_component_map[comp_type] =
            Dict(
                _get_temp_component_map_lhs(c) => PSY.get_name(c) for
                c in relevant_components if c isa comp_type
            )
    end
    return temp_component_map
end

function _make_pf_input_map!(
    pf_e_data::PowerFlowEvaluationData,
    container::OptimizationContainer,
    sys::PSY.System,
)
    pf_data = get_power_flow_data(pf_e_data)
    temp_component_map = _make_temp_component_map(pf_data, sys)
    map_type = valtype(temp_component_map)  # Dict{String, Int} for PowerFlowData, Dict{Union{String, Int64}, String} for SystemPowerFlowContainer
    pf_e_data.input_key_map = Dict{Symbol, Dict{OptimizationContainerKey, map_type}}()

    # available_keys is a vector of Pair{OptimizationContainerKey, data} containing all possibly relevant data sources to iterate over
    available_keys = vcat(
        [
            collect(pairs(f(container))) for
            f in [get_variables, get_aux_variables, get_parameters]
        ]...,
    )
    # Separate map for each category
    for category in pf_input_keys(pf_data)
        # Map that persists to store the bus index to which the variable maps in the PowerFlowData, etc.
        pf_data_opt_container_map = Dict{OptimizationContainerKey, map_type}()
        @info "Adding input map to send $category to $(nameof(typeof(pf_data)))"
        precedence = PF_INPUT_KEY_PRECEDENCES[category]
        _add_category_to_map!(
            precedence,
            available_keys,
            temp_component_map,
            pf_data_opt_container_map,
        )
        pf_e_data.input_key_map[category] = pf_data_opt_container_map
    end
    _add_two_terminal_elements_map!(sys, pf_data, available_keys, pf_e_data.input_key_map)
    return
end

"""
    _add_category_to_map!(
        precedence::Vector{DataType},
        available_keys::Vector{Pair{OptimizationContainerKey, Any}},
        temp_component_map::Union{
            Dict{DataType, Dict{String, Int}},
            Dict{DataType, Dict{Union{Int64, String}, String}},
        },
        pf_data_opt_container_map::Union{
            Dict{OptimizationContainerKey, Dict{String, Int}},
            Dict{OptimizationContainerKey, Dict{Union{Int64, String}, String}},
        },
    )

Helper function that is used in _make_pf_input_map! and _add_two_terminal_elements_map! to configure which variables from the
optimization results get written to the PowerFlowData. For every results variable from the optimization, it finds the corresponding
mapping between the optimization variable and the PowerFlowData variable.
The mappings are added to the `pf_data_opt_container_map` Dict.
This step is executed during the build stage of the optimization. The results are written to the PowerFlowData in the
solve stage, before the power flow is solved.

# Arguments
- `precedence::Vector{DataType}`: A vector of `DataType` objects that defines the order of precedence for the variables that correspond to the category of variables (e.g. `:active_power` - first look for `ActivePowerVariable` for the component type, if not available then `PowerOutput`, and finally `ActivePowerTimeSeriesParameter`).
- `available_keys::Vector{Pair{OptimizationContainerKey, Any}}`: A vector of key-value pairs where the key is an `OptimizationContainerKey` and the value contains data associated with the key.
- `temp_component_map::Union{Dict{DataType, Dict{String, Int}}, Dict{DataType, Dict{Union{Int64, String}, String}}}`: A mapping for component types to point the component-level results (e.g. as voltage value for bus "A") to the appropriate variable in PowerFlowData (e.g. row 27 in the bus-related matrices).
- `pf_data_opt_container_map::Union{Dict{OptimizationContainerKey, Dict{String, Int}}, Dict{OptimizationContainerKey, Dict{Union{Int64, String}, String}}}`: The target Dict that contains mappings for all relevant component types.
"""
function _add_category_to_map!(
    precedence::Vector{DataType},
    available_keys::Vector{Pair{OptimizationContainerKey, Any}},
    temp_component_map::Dict{DataType, <:Dict},
    pf_data_opt_container_map::Dict{OptimizationContainerKey, <:Dict},
)
    added_injection_types = DataType[]
    for entry_type in precedence
        for (key, val) in available_keys
            if get_entry_type(key) === entry_type
                comp_type = get_component_type(key)
                # Skip types that have already been handled by something of higher precedence
                if comp_type ∈ added_injection_types || comp_type ∉ keys(temp_component_map)
                    continue
                end
                push!(added_injection_types, comp_type)

                name_bus_ix_map = valtype(temp_component_map)()
                comp_names =
                    if (key isa ParameterKey)
                        get_component_names(get_attributes(val))
                    else
                        axes(val)[1]
                    end
                for comp_name in comp_names
                    name_bus_ix_map[comp_name] =
                        temp_component_map[comp_type][comp_name]
                end
                pf_data_opt_container_map[key] = name_bus_ix_map
            end
        end
    end
end

# the function to map HVDC power transfers as bus injections is not applicable to PSSEExporter:
_add_two_terminal_elements_map!(
    ::PSY.System,
    ::PFS.PSSEExporter,
    ::Vector{Pair{OptimizationContainerKey, Any}},
    ::Dict{Symbol, <:Dict{OptimizationContainerKey, <:Dict}},
) = nothing

"""
    _add_two_terminal_elements_map!(
        sys::PSY.System,
        pf_data::PFS.PowerFlowData,
        available_keys::Vector{Pair{OptimizationContainerKey, Any}},
        input_key_map::Dict{Symbol, Dict{OptimizationContainerKey, Dict{String, Int64}}}
    )

Adds mappings for two-terminal elements (HVDC components) that connect the power flow results (from -> to, to -> from)
to be added to the mappings for all component types.
The results for these elements are added as bus injections in the `PowerFlowData` as a simplified representation of
these components.

# Arguments
- `sys::PSY.System`: `System` instance representing the power system model.
- `pf_data::PFS.PowerFlowData`: The power flow data used internally for power flow calculations.
- `available_keys::Vector{Pair{OptimizationContainerKey, Any}}`: A vector of available optimization container keys and their associated values.
- `input_key_map::Dict{Symbol, Dict{OptimizationContainerKey, Dict{String, Int64}}}`: A dictionary mapping categories to optimization container keys and their associated mappings. To be extended in this function by the mappings for the two-terminal elements to the respective buses in the `PowerFlowData` instance.
"""
function _add_two_terminal_elements_map!(
    sys::PSY.System,
    pf_data::PFS.PowerFlowData,
    available_keys::Vector{Pair{OptimizationContainerKey, Any}},
    input_key_map::Dict{Symbol, <:Dict{OptimizationContainerKey, <:Dict}},
)
    for element_type in (PSY.TwoTerminalHVDC, PSY.PhaseShiftingTransformer)
        for (category, side) in zip(
            [:active_power_hvdc_pst_from_to, :active_power_hvdc_pst_to_from],
            [Val(:from), Val(:to)],
        )
            category ∈ pf_input_keys_hvdc_pst(pf_data) || continue

            temp_component_map = _make_temp_component_map(
                pf_data,
                sys,
                element_type,
                side,
            )
            isempty(temp_component_map) && continue

            precedence = PF_INPUT_KEY_PRECEDENCES[category]
            pf_data_opt_container_map =
                Dict{OptimizationContainerKey, valtype(temp_component_map)}()
            _add_category_to_map!(
                precedence,
                available_keys,
                temp_component_map,
                pf_data_opt_container_map,
            )
            category_map = get!(
                input_key_map,
                category,
                Dict{OptimizationContainerKey, valtype(temp_component_map)}(),
            )
            merge!(category_map, pf_data_opt_container_map)
        end
    end
    return
end

# Trait that determines what branch aux vars we can get from each PowerFlowContainer
branch_aux_vars(::PFS.ACPowerFlowData) =
    [PowerFlowBranchReactivePowerFromTo, PowerFlowBranchReactivePowerToFrom,
        PowerFlowBranchActivePowerFromTo, PowerFlowBranchActivePowerToFrom,
        PowerFlowBranchActivePowerLoss]
branch_aux_vars(::PFS.ABAPowerFlowData) =
    [PowerFlowBranchActivePowerFromTo, PowerFlowBranchActivePowerToFrom]
branch_aux_vars(::PFS.PTDFPowerFlowData) =
    [PowerFlowBranchActivePowerFromTo, PowerFlowBranchActivePowerToFrom]
branch_aux_vars(::PFS.vPTDFPowerFlowData) =
    [PowerFlowBranchActivePowerFromTo, PowerFlowBranchActivePowerToFrom]
branch_aux_vars(::PFS.PSSEExporter) = DataType[]

# Same for bus aux vars
function bus_aux_vars(data::PFS.ACPowerFlowData)
    vars = [PowerFlowVoltageAngle, PowerFlowVoltageMagnitude]
    if PFS.get_calculate_loss_factors(data)
        push!(vars, PowerFlowLossFactors)
    end
    if PFS.get_calculate_voltage_stability_factors(data)
        push!(vars, PowerFlowVoltageStabilityFactors)
    end
    return vars
end

bus_aux_vars(::PFS.ABAPowerFlowData) = [PowerFlowVoltageAngle]
bus_aux_vars(::PFS.PTDFPowerFlowData) = DataType[]
bus_aux_vars(::PFS.vPTDFPowerFlowData) = DataType[]
bus_aux_vars(::PFS.PSSEExporter) = DataType[]

# TODO: Needs update for MultiTerminal HVDC
_get_branch_component_tuples(sys::PSY.System) = [
    (typeof(c), get_name(c)) for
    c in PSY.get_available_components(PSY.ACBranch, sys)
]

_get_bus_component_tuples(pfd::PFS.PowerFlowData) =
    tuple.(PSY.ACBus, keys(PFS.get_bus_lookup(pfd)))  # get_bus_type returns a ACBusTypes, not the DataType we need here

_get_bus_component_tuples(pfd::PFS.SystemPowerFlowContainer) =
    [
        (typeof(c), PSY.get_number(c)) for
        c in PSY.get_available_components(PSY.ACBus, PFS.get_system(pfd))
    ]

function _with_time_steps(pf::T, n::Int) where {T <: PFS.PowerFlowEvaluationModel}
    fields = Dict(fn => getfield(pf, fn) for fn in fieldnames(T))
    fields[:time_steps] = n
    return T(; fields...)
end

_with_time_steps(pf::PFS.PSSEExportPowerFlow, ::Int) = pf # exporter doesn't use time_steps

function add_power_flow_data!(
    container::OptimizationContainer,
    evaluators::Vector{PFS.PowerFlowEvaluationModel},
    sys::PSY.System,
)
    container.power_flow_evaluation_data = Vector{PowerFlowEvaluationData}()
    sizehint!(container.power_flow_evaluation_data, length(evaluators))
    # For each output key, what components are we working with?
    branch_aux_var_components =
        Dict{Type{<:AuxVariableType}, Set{Tuple{<:DataType, String}}}()
    bus_aux_var_components = Dict{Type{<:AuxVariableType}, Set{Tuple{<:DataType, <:Int}}}()
    # we ought to be providing the time_steps when constructing the PF evaluation model,
    # but that value isn't known until runtime (and PF evaluation model is immutable).
    n_time_steps = length(get_time_steps(container))
    for evaluator in evaluators
        evaluator = _with_time_steps(evaluator, n_time_steps)
        @info "Building PowerFlow evaluator using $(evaluator)"
        pf_data = PFS.make_power_flow_container(evaluator, sys)
        pf_e_data = PowerFlowEvaluationData(pf_data)
        my_branch_aux_vars = branch_aux_vars(pf_data)
        my_bus_aux_vars = bus_aux_vars(pf_data)

        my_branch_components = _get_branch_component_tuples(sys)
        for branch_aux_var in my_branch_aux_vars
            to_add_to = get!(
                branch_aux_var_components,
                branch_aux_var,
                Set{Tuple{<:DataType, String}}(),
            )
            push!.(Ref(to_add_to), my_branch_components)
        end

        my_bus_components = _get_bus_component_tuples(pf_data)
        for bus_aux_var in my_bus_aux_vars
            to_add_to =
                get!(bus_aux_var_components, bus_aux_var, Set{Tuple{<:DataType, <:Int}}())
            push!.(Ref(to_add_to), my_bus_components)
        end
        push!(container.power_flow_evaluation_data, pf_e_data)
    end

    _add_aux_variables!(container, branch_aux_var_components)
    _add_aux_variables!(container, bus_aux_var_components)

    # Make the input maps after adding aux vars so output of one power flow can be input of another
    for pf_e_data in get_power_flow_evaluation_data(container)
        _make_pf_input_map!(pf_e_data, container, sys)
    end
    return
end

# How to update the PowerFlowData given a component type. A bit duplicative of code in PowerFlows.jl.
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Val{:active_power},
    ::Type{<:PSY.StaticInjection},
    index,
    t,
    value,
) = (pf_data.bus_active_power_injections[index, t] += value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Val{:active_power},
    ::Type{<:PSY.ElectricLoad},
    index,
    t,
    value,
) = (pf_data.bus_active_power_withdrawals[index, t] -= value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Val{:reactive_power},
    ::Type{<:PSY.StaticInjection},
    index,
    t,
    value,
) = (pf_data.bus_reactive_power_injections[index, t] += value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Val{:reactive_power},
    ::Type{<:PSY.ElectricLoad},
    index,
    t,
    value,
) = (pf_data.bus_reactive_power_withdrawals[index, t] -= value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Union{Val{:voltage_angle_export}, Val{:voltage_angle_opf}},
    ::Type{<:PSY.ACBus},
    index,
    t,
    value,
) = (pf_data.bus_angles[index, t] = value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Union{Val{:voltage_magnitude_export}, Val{:voltage_magnitude_opf}},
    ::Type{<:PSY.ACBus},
    index,
    t,
    value,
) = (pf_data.bus_magnitude[index, t] = value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Val{:active_power_hvdc_pst_from_to},
    ::Type{<:PSY.TwoTerminalHVDC},
    index,
    t,
    value,
) = (pf_data.bus_active_power_injections[index, t] -= value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Val{:active_power_hvdc_pst_to_from},
    ::Type{<:PSY.TwoTerminalHVDC},
    index,
    t,
    value,
) = (pf_data.bus_active_power_injections[index, t] += value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Val{:active_power_hvdc_pst_from_to},
    ::Type{<:PSY.PhaseShiftingTransformer},
    index,
    t,
    value,
) = (pf_data.bus_active_power_injections[index, t] -= value)
_update_pf_data_component!(
    pf_data::PFS.PowerFlowData,
    ::Val{:active_power_hvdc_pst_to_from},
    ::Type{<:PSY.PhaseShiftingTransformer},
    index,
    t,
    value,
) = (pf_data.bus_active_power_injections[index, t] += value)

function _write_value_to_pf_data!(
    pf_data::PFS.PowerFlowData,
    category::Symbol,
    container::OptimizationContainer,
    key::OptimizationContainerKey,
    component_map)
    result = lookup_value(container, key)
    for (device_name, index) in component_map
        injection_values = result[device_name, :]
        for t in get_time_steps(container)
            value = jump_value(injection_values[t])
            _update_pf_data_component!(
                pf_data,
                Val(category),
                get_component_type(key),
                index,
                t,
                value,
            )
        end
    end
    return
end

function update_pf_data!(
    pf_e_data::PowerFlowEvaluationData{<:PFS.PowerFlowData},
    container::OptimizationContainer,
)
    pf_data = get_power_flow_data(pf_e_data)
    PFS.clear_injection_data!(pf_data)
    input_map = get_input_key_map(pf_e_data)
    for (category, inputs) in input_map
        @debug "Writing $category to $(nameof(typeof(pf_data)))"
        for (key, component_map) in inputs
            _write_value_to_pf_data!(pf_data, category, container, key, component_map)
        end
    end
    return
end

# PERF we use direct dot access here, and implement our own unit conversions, for performance and convenience
_update_component!(comp::PSY.Component, ::Val{:active_power}, value, sys_base) =
    (comp.active_power = value * sys_base / PSY.get_base_power(comp))
# Sign is flipped for loads (TODO can we rely on some existing function that encodes this information?)
_update_component!(comp::PSY.ElectricLoad, ::Val{:active_power}, value, sys_base) =
    (comp.active_power = -value * sys_base / PSY.get_base_power(comp))
_update_component!(comp::PSY.Component, ::Val{:reactive_power}, value, sys_base) =
    (comp.reactive_power = value * sys_base / PSY.get_base_power(comp))
_update_component!(comp::PSY.ElectricLoad, ::Val{:reactive_power}, value, sys_base) =
    (comp.reactive_power = -value * sys_base / PSY.get_base_power(comp))
_update_component!(
    comp::PSY.ACBus,
    ::Union{Val{:voltage_angle_export}, Val{:voltage_angle_opf}},
    value, sys_base,
) =
    comp.angle = value
_update_component!(
    comp::PSY.ACBus,
    ::Union{Val{:voltage_magnitude_export}, Val{:voltage_magnitude_opf}},
    value, sys_base,
) =
    comp.magnitude = value

function update_pf_system!(
    sys::PSY.System,
    container::OptimizationContainer,
    input_map::Dict{Symbol, <:Dict{OptimizationContainerKey, <:Any}},
    time_step::Int,
)
    for (category, inputs) in input_map
        @debug "Writing $category to (possibly internal) System"
        for (key, component_map) in inputs
            result = lookup_value(container, key)
            for (device_id, device_name) in component_map
                injection_values = result[device_id, :]
                comp = PSY.get_component(get_component_type(key), sys, device_name)
                val = jump_value(injection_values[time_step])
                _update_component!(comp, Val(category), val, get_base_power(container))
            end
        end
    end
end

"""
Update a `PowerFlowEvaluationData` containing a `PowerFlowContainer` that does not
`supports_multi_period` using a single `time_step` of the `OptimizationContainer`. To
properly keep track of outer step number, time steps must be passed in sequentially,
starting with 1.
"""
function update_pf_data!(
    pf_e_data::PowerFlowEvaluationData{PFS.PSSEExporter},
    container::OptimizationContainer,
    time_step::Int,
)
    pf_data = get_power_flow_data(pf_e_data)
    input_map = get_input_key_map(pf_e_data)
    update_pf_system!(PFS.get_system(pf_data), container, input_map, time_step)
    if !isnothing(pf_data.step)
        outer_step, _... = pf_data.step
        # time_step == 1 means we have rolled over to a new outer step
        # NOTE this is a bit brittle but there is currently no way of getting this
        # information from upstream, may change in the future
        (time_step == 1) && (outer_step += 1)
        pf_data.step = (outer_step, time_step)
    end
    return
end

"Fetch the most recently solved `PowerFlowEvaluationData`"
function latest_solved_power_flow_evaluation_data(container::OptimizationContainer)
    datas = get_power_flow_evaluation_data(container)
    return datas[findlast(x -> x.is_solved, datas)]
end

function solve_power_flow!(
    pf_e_data::PowerFlowEvaluationData,
    container::OptimizationContainer)
    pf_data = get_power_flow_data(pf_e_data)
    if PFS.supports_multi_period(pf_data)
        update_pf_data!(pf_e_data, container)
        PFS.solve_power_flow!(pf_data)
    else
        for t in get_time_steps(container)
            update_pf_data!(pf_e_data, container, t)
            PFS.solve_power_flow!(pf_data)
        end
    end
    pf_e_data.is_solved = true
    return
end

# Currently nothing to write back to the optimization container from a PSSEExporter
calculate_aux_variable_value!(::OptimizationContainer,
    ::AuxVarKey{T, <:Any} where {T <: PowerFlowAuxVariableType},
    ::PSY.System, ::PowerFlowEvaluationData{PFS.PSSEExporter}) = nothing

_get_pf_result(::Type{PowerFlowVoltageAngle}, pf_data::PFS.PowerFlowData) =
    PFS.get_bus_angles(pf_data)
_get_pf_result(::Type{PowerFlowVoltageMagnitude}, pf_data::PFS.PowerFlowData) =
    PFS.get_bus_magnitude(pf_data)
_get_pf_result(::Type{PowerFlowBranchReactivePowerFromTo}, pf_data::PFS.PowerFlowData) =
    PFS.get_arc_reactive_power_flow_from_to(pf_data)
_get_pf_result(::Type{PowerFlowBranchReactivePowerToFrom}, pf_data::PFS.PowerFlowData) =
    PFS.get_arc_reactive_power_flow_to_from(pf_data)
_get_pf_result(::Type{PowerFlowBranchActivePowerFromTo}, pf_data::PFS.PowerFlowData) =
    PFS.get_arc_active_power_flow_from_to(pf_data)
_get_pf_result(::Type{PowerFlowBranchActivePowerToFrom}, pf_data::PFS.PowerFlowData) =
    PFS.get_arc_active_power_flow_to_from(pf_data)
_get_pf_result(::Type{PowerFlowLossFactors}, pf_data::PFS.PowerFlowData) =
    PFS.get_loss_factors(pf_data)
_get_pf_result(::Type{PowerFlowVoltageStabilityFactors}, pf_data::PFS.PowerFlowData) =
    PFS.get_voltage_stability_factors(pf_data)
# PERF: unlike the others, this one requires a bit of computation.
_get_pf_result(::Type{PowerFlowBranchActivePowerLoss}, pf_data::PFS.PowerFlowData) =
    PFS.get_arc_active_power_flow_from_to(pf_data) .+
    PFS.get_arc_active_power_flow_to_from(pf_data)

function calculate_aux_variable_value!(container::OptimizationContainer,
    key::AuxVarKey{T, <:PSY.ACBus},
    ::PSY.System,
    pf_e_data::PowerFlowEvaluationData{<:PFS.PowerFlowData},
) where {T <: PowerFlowAuxVariableType}
    @debug "Updating $key from PowerFlowData"
    pf_data = get_power_flow_data(pf_e_data)
    nrd = PFS.get_network_reduction_data(pf_data)
    src = _get_pf_result(T, pf_data)
    bus_lookup = PFS.get_bus_lookup(pf_data)
    dest = get_aux_variable(container, key)
    for bus_number in axes(dest, 1)
        bus_ix = PNM.get_bus_index(bus_number, bus_lookup, nrd)
        dest[bus_number, :] = src[bus_ix, :]
    end
    return
end

function calculate_aux_variable_value!(container::OptimizationContainer,
    key::AuxVarKey{T, U},
    ::PSY.System,
    pf_e_data::PowerFlowEvaluationData{<:PFS.PowerFlowData},
) where {T <: PowerFlowAuxVariableType, U <: PSY.Branch}
    @debug "Updating $key from PowerFlowData"
    pf_data = get_power_flow_data(pf_e_data)
    src = _get_pf_result(T, pf_data)
    dest = get_aux_variable(container, key)
    nrd = PFS.get_network_reduction_data(pf_data)
    arc_lookup = PFS.get_arc_lookup(pf_data)
    # PERF: could pre-compute a Dict of branch type to arcs, then intersect the arcs
    # for the type U with the keys of the branch maps.
    for (arc, br) in PNM.get_direct_branch_map(nrd)
        if br isa U # always a concrete type, so same as: typeof(br) == U
            name = PSY.get_name(br)
            arc_ix = arc_lookup[arc]
            dest[name, :] = src[arc_ix, :]
        end
    end
    for (arc, parallel_brs) in PNM.get_parallel_branch_map(nrd) # parallel_brs is Set{ACTransmission}
        for br in parallel_brs
            sample_line = first(parallel_brs)
            impedance = PSY.get_r(sample_line) + im * PSY.get_x(sample_line)
            first_name = PSY.get_name(sample_line)
            if br isa U
                name = PSY.get_name(br)
                IS.@assert_op T <: BranchFlowAuxVariableType ||
                              (T == PowerFlowBranchActivePowerLoss)
                if !isapprox(PSY.get_r(br) + im * PSY.get_x(br), impedance)
                    @debug "Parallel branches with different impedances found: " *
                           "$name and $first_name. Check your data inputs."
                end
                multiplier = PNM.compute_parallel_multiplier(parallel_brs, name)
                arc_ix = arc_lookup[arc]
                dest[name, :] = multiplier .* src[arc_ix, :]
            end
        end
    end
    return
end

function calculate_aux_variable_value!(container::OptimizationContainer,
    key::AuxVarKey{<:PowerFlowAuxVariableType, <:PSY.Component},
    system::PSY.System)
    # Skip the aux vars that the current power flow isn't meant to update
    pf_e_data = latest_solved_power_flow_evaluation_data(container)
    pf_data = get_power_flow_data(pf_e_data)
    (key in branch_aux_vars(pf_data) || key in bus_aux_vars(pf_data)) && return
    calculate_aux_variable_value!(container, key, system, pf_e_data)
    return
end
