# Defines the order of precedence for each type of information that could be sent to PowerFlows.jl
const PF_INPUT_KEY_PRECEDENCES = Dict(
    :active_power => [ActivePowerVariable, PowerOutput, ActivePowerTimeSeriesParameter],
    :active_power_in => [ActivePowerInVariable, ActivePowerInTimeSeriesParameter],
    :active_power_out => [ActivePowerOutVariable, ActivePowerOutTimeSeriesParameter],
    :reactive_power => [ReactivePowerVariable, ReactivePowerTimeSeriesParameter],
    :voltage_angle_export => [PowerFlowVoltageAngle, VoltageAngle],
    :voltage_magnitude_export => [PowerFlowVoltageMagnitude, VoltageMagnitude],
    :voltage_angle_opf => [VoltageAngle],
    :voltage_magnitude_opf => [VoltageMagnitude],
    :active_power_hvdc_pst_from_to =>
        [FlowActivePowerFromToVariable, FlowActivePowerVariable],
    :active_power_hvdc_pst_to_from =>
        [FlowActivePowerToFromVariable, FlowActivePowerVariable],
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
    [:active_power, :active_power_in, :active_power_out]
pf_input_keys(::PFS.PTDFPowerFlowData) =
    [:active_power, :active_power_in, :active_power_out]
pf_input_keys(::PFS.vPTDFPowerFlowData) =
    [:active_power, :active_power_in, :active_power_out]
pf_input_keys(::PFS.ACPowerFlowData) =
    [
        :active_power,
        :active_power_in,
        :active_power_out,
        :reactive_power,
        :voltage_angle_opf,
        :voltage_magnitude_opf,
    ]
pf_input_keys(::PFS.PSSEExporter) =
    [
        :active_power,
        :active_power_in,
        :active_power_out,
        :reactive_power,
        :voltage_angle_export,
        :voltage_magnitude_export,
    ]
# HVDC/PST flows feed every `PowerFlowData` solve. They route by component (see the resolver
# below: HVDC → `bus_hvdc_net_power`, PST → `bus_active_power_injections`); `update_pf_data!`
# clears the HVDC channel before re-population.
pf_input_keys_hvdc_pst(::PFS.PowerFlowData) =
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
        @assert haskey(PF_INPUT_KEY_PRECEDENCES, category) "No source precedence defined for power-flow input category $category"
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
        # A two-terminal element whose from and to resolve to one bus would collapse its
        # from_to/to_from injections onto the same row; guard against that here.
        for comp in PSY.get_available_components(element_type, sys)
            @assert PSY.get_number(PSY.get_from_bus(comp)) !=
                    PSY.get_number(PSY.get_to_bus(comp)) "Two-terminal $(PSY.get_name(comp)) maps from and to to the same bus"
        end
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

# Same for bus aux vars. Loss/voltage-stability factors are registered ONLY when their
# `get_calculate_*` flag is set — the same flag under which `_get_pf_result` returns a non-`nothing`
# matrix (`PFS.get_loss_factors` / `get_voltage_stability_factors` are `nothing` otherwise). Keep
# these two conditions in lockstep so the read-back never indexes a `nothing`.
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

# Time-series parameter types that some power flow evaluators need as input but
# that an optimization formulation may legitimately omit (e.g.,
# `ReactivePowerTimeSeriesParameter` is not added by `AbstractActivePowerModel`
# device constructors). When such a parameter is required by a configured power
# flow evaluator and the user has wired up a time series for it, we add it to
# the container so `_make_pf_input_map!` can route the data to the
# `PowerFlowData`. These parameters are not added to any optimization
# expression, so they don't change the optimization model.
const PF_ONLY_TS_PARAMS_BY_CATEGORY = Dict{Symbol, Type{<:TimeSeriesParameter}}(
    :reactive_power => ReactivePowerTimeSeriesParameter,
)

function _add_pf_only_time_series_parameters!(
    container::OptimizationContainer,
    template::Union{ProblemTemplate, Nothing},
    sys::PSY.System,
    needed_categories::Set{Symbol},
)
    template === nothing && return
    isempty(needed_categories) && return
    for (category, ParamT) in PF_ONLY_TS_PARAMS_BY_CATEGORY
        category in needed_categories || continue
        for device_model in values(template.devices)
            ts_names = get_time_series_names(device_model)
            haskey(ts_names, ParamT) || continue
            D = get_component_type(device_model)
            has_container_key(container, ParamT, D) && continue
            devices = get_available_components(device_model, sys)
            isempty(devices) && continue
            add_parameters!(container, ParamT, devices, device_model)
            has_container_key(container, ParamT, D) || continue
            @debug "Added $(ParamT) for $(D) to support power flow evaluator input :$(category)" _group =
                LOG_GROUP_OPTIMIZATION_CONTAINER
        end
    end
    return
end

function add_power_flow_data!(
    container::OptimizationContainer,
    evaluators::Vector{PFS.PowerFlowEvaluationModel},
    sys::PSY.System,
    template::Union{ProblemTemplate, Nothing} = nothing,
)
    container.power_flow_evaluation_data = Vector{PowerFlowEvaluationData}()
    sizehint!(container.power_flow_evaluation_data, length(evaluators))
    # For each output key, what components are we working with?
    branch_aux_var_components =
        Dict{Type{<:AuxVariableType}, Set{Tuple{<:DataType, String}}}()
    bus_aux_var_components = Dict{Type{<:AuxVariableType}, Set{Tuple{<:DataType, <:Int}}}()
    # Categories of input data needed across all configured PF evaluators.
    needed_categories = Set{Symbol}()
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
        for category in pf_input_keys(pf_data)
            push!(needed_categories, category)
        end
        push!(container.power_flow_evaluation_data, pf_e_data)
    end

    _add_aux_variables!(container, branch_aux_var_components)
    _add_aux_variables!(container, bus_aux_var_components)

    # Add time-series parameters that the PF evaluators need but that the
    # optimization formulation may not have added (e.g., reactive power for
    # AbstractActivePowerModel networks running an AC power flow evaluator).
    _add_pf_only_time_series_parameters!(container, template, sys, needed_categories)

    # Make the input maps after adding aux vars so output of one power flow can be input of another
    for pf_e_data in get_power_flow_evaluation_data(container)
        _make_pf_input_map!(pf_e_data, container, sys)
    end
    return
end

# Injection-sign resolver: the single source of truth mapping an optimization value to its
# power-flow contribution. `sign` equals the multiplier `add_to_expression!` applied to the bus
# balance, so PF reproduces the OPF nodal balance. Two thin writers consume it (below).
#   quantity : :active | :reactive | :angle | :magnitude
#   role     : active/reactive array selector :injection | :withdrawal | :hvdc_net (:none for voltage)
#   sign     : nodal-balance multiplier (+1 / -1)
#   partial  : System writer only — in/out variables accumulate onto a shared active_power field
struct PFContribution
    quantity::Symbol
    role::Symbol
    sign::Float64
    partial::Bool
end

const _PF_FLOW_ENTRY = Union{VariableType, AuxVariableType}
const _PF_PARAM_ENTRY = ParameterType

# ---- variable / aux entries: the input category carries the direction ----
# mirrors `add_to_expression!`: StaticInjection (+1 injection), ElectricLoad (-1 withdrawal).
pf_contribution(
    ::Val{:active_power},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.StaticInjection},
) =
    PFContribution(:active, :injection, 1.0, false)
pf_contribution(
    ::Val{:active_power},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.ElectricLoad},
) =
    PFContribution(:active, :withdrawal, -1.0, false)
# ActivePowerOutVariable: power output (positive injection); ActivePowerInVariable: withdrawal.
pf_contribution(
    ::Val{:active_power_out},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.StaticInjection},
) =
    PFContribution(:active, :injection, 1.0, true)
pf_contribution(
    ::Val{:active_power_in},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.StaticInjection},
) =
    PFContribution(:active, :injection, -1.0, true)
pf_contribution(
    ::Val{:reactive_power},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.StaticInjection},
) =
    PFContribution(:reactive, :injection, 1.0, false)
pf_contribution(
    ::Val{:reactive_power},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.ElectricLoad},
) =
    PFContribution(:reactive, :withdrawal, -1.0, false)
pf_contribution(
    ::Union{Val{:voltage_angle_export}, Val{:voltage_angle_opf}},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.ACBus},
) = PFContribution(:angle, :none, 1.0, false)
pf_contribution(
    ::Union{Val{:voltage_magnitude_export}, Val{:voltage_magnitude_opf}},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.ACBus},
) = PFContribution(:magnitude, :none, 1.0, false)

# ---- HVDC / PST two-terminal (variable entries) ----
# HVDC re-targets to `:hvdc_net` (`bus_hvdc_net_power`); signs follow the injection convention.
# from_to: -1. to_from: `FlowActivePowerToFromVariable` is -tf, signed negative for from→to flow
# (sign -1); a single `FlowActivePowerVariable` (lossless / PowerModels `:p_dc`) is +flow (sign +1).
pf_contribution(
    ::Val{:active_power_hvdc_pst_from_to},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.TwoTerminalHVDC},
) = PFContribution(:active, :hvdc_net, -1.0, false)
pf_contribution(
    ::Val{:active_power_hvdc_pst_to_from},
    ::Type{FlowActivePowerToFromVariable},
    ::Type{<:PSY.TwoTerminalHVDC},
) = PFContribution(:active, :hvdc_net, -1.0, false)
pf_contribution(
    ::Val{:active_power_hvdc_pst_to_from},
    ::Type{FlowActivePowerVariable},
    ::Type{<:PSY.TwoTerminalHVDC},
) = PFContribution(:active, :hvdc_net, 1.0, false)
# PhaseShiftingTransformer stays on the generic injection array: from_to -1, to_from +1.
pf_contribution(
    ::Val{:active_power_hvdc_pst_from_to},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.PhaseShiftingTransformer},
) = PFContribution(:active, :injection, -1.0, false)
pf_contribution(
    ::Val{:active_power_hvdc_pst_to_from},
    ::Type{<:_PF_FLOW_ENTRY},
    ::Type{<:PSY.PhaseShiftingTransformer},
) = PFContribution(:active, :injection, 1.0, false)

# ---- parameter entries: the value already stores the signed nodal contribution ----
# `param_array .* multiplier_array` bakes the direction in, identical to what
# `add_to_expression!` adds to the balance. So these match the variable entries EXCEPT in/out,
# which become +1: re-applying the category sign would double-count (this is the #1631 fix).
pf_contribution(
    ::Val{:active_power},
    ::Type{<:_PF_PARAM_ENTRY},
    ::Type{<:PSY.StaticInjection},
) =
    PFContribution(:active, :injection, 1.0, false)
pf_contribution(
    ::Union{Val{:active_power_in}, Val{:active_power_out}},
    ::Type{<:_PF_PARAM_ENTRY},
    ::Type{<:PSY.StaticInjection},
) = PFContribution(:active, :injection, 1.0, true)
pf_contribution(
    ::Val{:active_power},
    ::Type{<:_PF_PARAM_ENTRY},
    ::Type{<:PSY.ElectricLoad},
) =
    PFContribution(:active, :withdrawal, -1.0, false)
pf_contribution(
    ::Val{:reactive_power},
    ::Type{<:_PF_PARAM_ENTRY},
    ::Type{<:PSY.StaticInjection},
) =
    PFContribution(:reactive, :injection, 1.0, false)
pf_contribution(
    ::Val{:reactive_power},
    ::Type{<:_PF_PARAM_ENTRY},
    ::Type{<:PSY.ElectricLoad},
) =
    PFContribution(:reactive, :withdrawal, -1.0, false)
pf_contribution(
    ::Union{Val{:voltage_angle_export}, Val{:voltage_angle_opf}},
    ::Type{<:_PF_PARAM_ENTRY},
    ::Type{<:PSY.ACBus},
) = PFContribution(:angle, :none, 1.0, false)
pf_contribution(
    ::Union{Val{:voltage_magnitude_export}, Val{:voltage_magnitude_opf}},
    ::Type{<:_PF_PARAM_ENTRY},
    ::Type{<:PSY.ACBus},
) = PFContribution(:magnitude, :none, 1.0, false)

# ---- PowerFlowData writer ----
# Active/reactive quantities accumulate into an injection array chosen by (quantity, role);
# voltage quantities (:angle/:magnitude) are assigned to a bus-state array.
_pf_writes_voltage(q::Symbol) = q === :angle || q === :magnitude

_pf_array(pfd::PFS.PowerFlowData, ::Val{:active}, ::Val{:injection}) =
    pfd.bus_active_power_injections
_pf_array(pfd::PFS.PowerFlowData, ::Val{:active}, ::Val{:withdrawal}) =
    pfd.bus_active_power_withdrawals
_pf_array(pfd::PFS.PowerFlowData, ::Val{:active}, ::Val{:hvdc_net}) = pfd.bus_hvdc_net_power
_pf_array(pfd::PFS.PowerFlowData, ::Val{:reactive}, ::Val{:injection}) =
    pfd.bus_reactive_power_injections
_pf_array(pfd::PFS.PowerFlowData, ::Val{:reactive}, ::Val{:withdrawal}) =
    pfd.bus_reactive_power_withdrawals
_pf_bus_state_array(pfd::PFS.PowerFlowData, ::Val{:angle}) = pfd.bus_angles
_pf_bus_state_array(pfd::PFS.PowerFlowData, ::Val{:magnitude}) = pfd.bus_magnitude

function _write_value_to_pf_data!(
    pf_data::PFS.PowerFlowData,
    category::Symbol,
    container::OptimizationContainer,
    key::OptimizationContainerKey,
    component_map)
    result = lookup_value(container, key)
    c = pf_contribution(Val(category), get_entry_type(key), get_component_type(key))
    # Resolve the target array once per key so the inner write loop runs on a concrete `Matrix`.
    if _pf_writes_voltage(c.quantity)
        _write_pf_array!(_pf_bus_state_array(pf_data, Val(c.quantity)), true, c.sign,
            component_map, container, result)
    else
        _write_pf_array!(_pf_array(pf_data, Val(c.quantity), Val(c.role)), false, c.sign,
            component_map, container, result)
    end
    return
end

# Function barrier: `arr` is a concrete `Matrix{Float64}`, so the per-(device, time) loop is
# monomorphic. `assign` overwrites (voltages); otherwise contributions accumulate.
function _write_pf_array!(
    arr::Matrix{Float64},
    assign::Bool,
    sign::Float64,
    component_map,
    container::OptimizationContainer,
    result,
)
    for (device_name, index) in component_map
        for t in get_time_steps(container)
            value = jump_value(result[device_name, t])
            if assign
                arr[index, t] = value
            else
                arr[index, t] += sign * value
            end
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
    # `clear_injection_data!` does not reset `bus_hvdc_net_power`, which PowerFlows seeds from the
    # system at construction. Zero it before re-writing optimized HVDC flows, else the seed
    # double-counts (AC, #1635) or shadows the optimized DC flow. Co-located with the repopulate
    # below so they can't desync; skipped only if a `PowerFlowData` opts out of hvdc_pst.
    isempty(pf_input_keys_hvdc_pst(pf_data)) || (pf_data.bus_hvdc_net_power .= 0.0)
    input_map = get_input_key_map(pf_e_data)
    for (category, inputs) in input_map
        @debug "Writing $category to $(nameof(typeof(pf_data)))"
        for (key, component_map) in inputs
            _write_value_to_pf_data!(pf_data, category, container, key, component_map)
        end
    end
    return
end

# ---- System writer: same `PFContribution`, sunk into component fields ----
# PERF we use direct dot access here, and implement our own unit conversions, for performance.
# active/reactive convert to the component base; voltages are written raw.
_pf_to_comp(::Union{Val{:active}, Val{:reactive}}, value::Float64, sys_base::Float64,
    comp::PSY.Component) = value * sys_base / PSY.get_base_power(comp)
_pf_to_comp(
    ::Union{Val{:angle}, Val{:magnitude}},
    value::Float64,
    ::Float64,
    ::PSY.Component,
) =
    value

# Set (or accumulate, for `partial` in/out contributions) the signed quantity on the component's
# field. `StandardLoad` (ZIP) has no scalar power field, so it routes to its constant-power
# component (`constant_active_power`/`constant_reactive_power`).
_set_comp_quantity!(comp::PSY.Component, ::Val{:active}, v::Float64, partial::Bool) =
    partial ? (comp.active_power += v) : (comp.active_power = v)
_set_comp_quantity!(comp::PSY.Component, ::Val{:reactive}, v::Float64, ::Bool) =
    (comp.reactive_power = v)
_set_comp_quantity!(comp::PSY.StandardLoad, ::Val{:active}, v::Float64, ::Bool) =
    (comp.constant_active_power = v)
_set_comp_quantity!(comp::PSY.StandardLoad, ::Val{:reactive}, v::Float64, ::Bool) =
    (comp.constant_reactive_power = v)
_set_comp_quantity!(comp::PSY.ACBus, ::Val{:angle}, v::Float64, ::Bool) = (comp.angle = v)
_set_comp_quantity!(comp::PSY.ACBus, ::Val{:magnitude}, v::Float64, ::Bool) =
    (comp.magnitude = v)

function _apply_component_contribution!(
    comp::PSY.Component,
    c::PFContribution,
    value::Float64,
    sys_base::Float64,
)
    v = c.sign * _pf_to_comp(Val(c.quantity), value, sys_base, comp)
    _set_comp_quantity!(comp, Val(c.quantity), v, c.partial)
    return
end

function update_pf_system!(
    sys::PSY.System,
    container::OptimizationContainer,
    input_map::Dict{Symbol, <:Dict{OptimizationContainerKey, <:Any}},
    time_step::Int,
)
    # Reset active_power to zero for components that use separate in/out variables
    # (e.g. storage, import/export sources) before the additive += / -= updates.
    # Collect unique (type, name) pairs to avoid resetting the same component twice.
    reset_components = Set{Tuple{DataType, String}}()
    for category in (:active_power_in, :active_power_out)
        haskey(input_map, category) || continue
        for (key, component_map) in input_map[category]
            for (_, device_name) in component_map
                push!(reset_components, (get_component_type(key), device_name))
            end
        end
    end
    for (comp_type, device_name) in reset_components
        comp = PSY.get_component(comp_type, sys, device_name)
        comp.active_power = 0.0
    end
    for (category, inputs) in input_map
        @debug "Writing $category to (possibly internal) System"
        for (key, component_map) in inputs
            result = lookup_value(container, key)
            c = pf_contribution(Val(category), get_entry_type(key), get_component_type(key))
            for (device_id, device_name) in component_map
                comp = PSY.get_component(get_component_type(key), sys, device_name)
                val = jump_value(result[device_id, time_step])
                _apply_component_contribution!(comp, c, val, get_base_power(container))
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

# ParameterKey → FixedOutput formulation; dispatch is externally determined,
# should not participate in slack.
_accumulate_headroom!(::PFS.PowerFlowData,
    ::OptimizationContainer,
    ::PSY.System,
    ::OptimizationContainerKey{<:ParameterType, <:PSY.Component},
    ::Dict{String, Int},
    ::Int,
    ::Matrix{PSY.ACBusTypes},
    ::Vector{Dict{Tuple{DataType, String}, Float64}},
) = nothing

# Storage uses split In/Out active power variables; its headroom contribution comes
# from `_accumulate_in_out_headroom!` below. These skips guard against any (currently
# unused) `:active_power` mapping for Storage that would otherwise double-count.
_accumulate_headroom!(::PFS.PowerFlowData,
    ::OptimizationContainer,
    ::PSY.System,
    ::OptimizationContainerKey{<:ISOPT.OptimizationKeyType, <:PSY.Storage},
    ::Dict{String, Int},
    ::Int,
    ::Matrix{PSY.ACBusTypes},
    ::Vector{Dict{Tuple{DataType, String}, Float64}},
) = nothing

# needed to fix an ambiguity: ParameterType with Storage.
_accumulate_headroom!(::PFS.PowerFlowData,
    ::OptimizationContainer,
    ::PSY.System,
    ::OptimizationContainerKey{<:ISOPT.ParameterType, <:PSY.Storage},
    ::Dict{String, Int},
    ::Int,
    ::Matrix{PSY.ACBusTypes},
    ::Vector{Dict{Tuple{DataType, String}, Float64}},
) = nothing

"""
Accumulate headroom for a single OptimizationContainerKey into `pf_data` and `computed_gspf`.
The `where {U}` parameter makes the component type a compile-time constant, so
`PSY.get_component(U, ...)`, `has_container_key(..., U)`, and the `(U, device_name)` Dict
key all dispatch concretely. Note that `result` and `ts_param_values` remain abstractly
typed because `OptimizationContainer.variables` and `.parameters` have abstract value types
in their dict signatures — the inner indexing into them still goes through dynamic dispatch.
"""
function _accumulate_headroom!(
    pf_data::PFS.PowerFlowData,
    container::OptimizationContainer,
    sys::PSY.System,
    key::OptimizationContainerKey{<:ISOPT.OptimizationKeyType, U},
    component_map::Dict{String, Int},
    n_time_steps::Int,
    bus_types::Matrix{PSY.ACBusTypes},
    computed_gspf::Vector{Dict{Tuple{DataType, String}, Float64}},
) where {U <: PSY.Component}
    result = lookup_value(container, key)

    # Time-varying active power limits (e.g. renewable availability profiles).
    # Precompute the axis as a Set so the per-(device, t) membership test is O(1).
    ts_param_values, ts_axis =
        if has_container_key(
            container, ActivePowerTimeSeriesParameter, U)
            vals = lookup_value(container, ParameterKey(ActivePowerTimeSeriesParameter, U))
            (vals, Set{String}(axes(vals, 1)))
        else
            (nothing, nothing)
        end

    for (device_name, bus_ix) in component_map
        comp = PSY.get_component(U, sys, device_name)
        PFS.contributes_active_power(comp) || continue
        PFS.active_power_contribution_type(comp) ==
        PFS.PowerContributionType.INJECTION || continue

        # limits.max is already in SYSTEM_BASE because PSI sets units at init
        p_max_static = PFS.get_active_power_limits_for_power_flow(comp).max
        has_ts = !isnothing(ts_axis) && device_name ∈ ts_axis

        for t in 1:n_time_steps
            bus_types[bus_ix, t] ∈ (PSY.ACBusTypes.REF, PSY.ACBusTypes.PV) || continue

            p_setpoint = jump_value(result[device_name, t])
            p_max_t = if has_ts
                min(p_max_static, jump_value(ts_param_values[device_name, t]))
            else
                p_max_static
            end
            headroom = p_max_t - p_setpoint
            headroom <= 0.0 && continue

            computed_gspf[t][(U, device_name)] = headroom
            pf_data.bus_active_power_range[bus_ix, t] += headroom
        end
    end
    return
end

# Maximum discharge active power (system-base PU) for devices that use split
# `ActivePowerInVariable` / `ActivePowerOutVariable`. PFS's
# `get_active_power_limits_for_power_flow(::Source)` returns `(min=-Inf, max=Inf)`,
# which is unusable for headroom math, so we read the device-level limits directly.
_pf_in_out_discharge_max(comp::PSY.Storage) = PSY.get_output_active_power_limits(comp).max
_pf_in_out_discharge_max(comp::PSY.Source) = PSY.get_active_power_limits(comp).max

"""
Accumulate headroom for devices that use split `ActivePowerInVariable` /
`ActivePowerOutVariable` (e.g. Storage `BookKeeping`, Source `ImportExportSourceModel`).

`net = p_out - p_in` is the device's signed contribution at time `t`. With net > 0 the
device is dispatching and its headroom is `p_max_out - net`; with net <= 0 the device is
charging (or idle) and contributes no upward slack.
"""
function _accumulate_in_out_headroom!(
    pf_data::PFS.PowerFlowData,
    container::OptimizationContainer,
    sys::PSY.System,
    in_inputs::Dict{OptimizationContainerKey, Dict{String, Int}},
    out_inputs::Dict{OptimizationContainerKey, Dict{String, Int}},
    n_time_steps::Int,
    bus_types::Matrix{PSY.ACBusTypes},
    computed_gspf::Vector{Dict{Tuple{DataType, String}, Float64}},
)
    for (in_key, in_cmap) in in_inputs
        out_key, out_cmap = _find_paired_out(out_inputs, get_component_type(in_key))
        _accumulate_in_out_headroom_one_type!(
            pf_data, container, sys,
            in_key, in_cmap, out_key, out_cmap,
            n_time_steps, bus_types, computed_gspf,
        )
    end
    return
end

function _find_paired_out(
    out_inputs::Dict{OptimizationContainerKey, Dict{String, Int}},
    comp_type::DataType,
)
    for (key, cmap) in out_inputs
        get_component_type(key) === comp_type && return (key, cmap)
    end
    error(
        "`:active_power_out` map missing for $comp_type — a formulation added " *
        "`ActivePowerInVariable` without a paired `ActivePowerOutVariable`.",
    )
end

# Function barrier: the parametric key types specialize `lookup_value` and `result[...]`
# indexing on the concrete component type `U`.
function _accumulate_in_out_headroom_one_type!(
    pf_data::PFS.PowerFlowData,
    container::OptimizationContainer,
    sys::PSY.System,
    in_key::OptimizationContainerKey{<:ISOPT.OptimizationKeyType, U},
    in_cmap::Dict{String, Int},
    out_key::OptimizationContainerKey{<:ISOPT.OptimizationKeyType, U},
    out_cmap::Dict{String, Int},
    n_time_steps::Int,
    bus_types::Matrix{PSY.ACBusTypes},
    computed_gspf::Vector{Dict{Tuple{DataType, String}, Float64}},
) where {U <: PSY.Component}
    result_in = lookup_value(container, in_key)
    result_out = lookup_value(container, out_key)
    for (device_name, bus_ix) in in_cmap
        comp = PSY.get_component(U, sys, device_name)
        PFS.contributes_active_power(comp) || continue
        PFS.active_power_contribution_type(comp) ==
        PFS.PowerContributionType.INJECTION || continue
        p_max_out = _pf_in_out_discharge_max(comp)
        for t in 1:n_time_steps
            bus_types[bus_ix, t] ∈ (PSY.ACBusTypes.REF, PSY.ACBusTypes.PV) || continue
            net =
                jump_value(result_out[device_name, t]) -
                jump_value(result_in[device_name, t])
            # Net <= 0 means charging or idle — per spec, no upward slack contribution.
            net < 0.0 && continue
            headroom = p_max_out - net
            headroom <= 0.0 && continue
            computed_gspf[t][(U, device_name)] = headroom
            pf_data.bus_active_power_range[bus_ix, t] += headroom
        end
    end
    return
end

"""
Recompute per-time-step headroom-proportional generator slack participation factors using
optimization results. Only runs if headroom proportional slack was enabled during
initialization.

For each generator at a REF or PV bus, headroom is `P_max(t) - P_setpoint(t)`, where
`P_setpoint(t)` comes from the optimization result and `P_max(t)` is the minimum of the
static device limit and any `ActivePowerTimeSeriesParameter` at time `t`. Devices that use
split In/Out active power variables are handled separately via
`_accumulate_in_out_headroom!`. This overwrites the PF-initialized values (which were
computed once from static system data) with time-varying factors.
"""
function _update_headroom_participation_factors!(
    pf_data::PFS.PowerFlowData,
    container::OptimizationContainer,
    sys::PSY.System,
    input_key_map::Dict{Symbol, Dict{OptimizationContainerKey, Dict{String, Int}}},
)
    PFS.get_distribute_slack_proportional_to_headroom(PFS.get_pf(pf_data)) || return
    computed_gspf =
        PFS.get_computed_gspf(pf_data)::Vector{Dict{Tuple{DataType, String}, Float64}}

    n_time_steps = length(get_time_steps(container))
    bus_types = PFS.get_bus_type(pf_data)::Matrix{PSY.ACBusTypes}
    bus_slack_pf =
        PFS.get_bus_slack_participation_factors(
            pf_data,
        )::SparseArrays.SparseMatrixCSC{
            Float64,
            Int,
        }

    # Reset with fresh dicts per time step (init may share references)
    for t in 1:n_time_steps
        computed_gspf[t] = Dict{Tuple{DataType, String}, Float64}()
    end
    pf_data.bus_active_power_range .= 0.0

    # Function barrier so `_accumulate_headroom!` specializes per concrete key type
    # encountered at runtime — the outer Dict iterates abstract `OptimizationContainerKey`s.
    for (key, component_map) in input_key_map[:active_power]
        _accumulate_headroom!(
            pf_data, container, sys, key, component_map,
            n_time_steps, bus_types, computed_gspf)
    end

    # Devices with split `ActivePowerInVariable` / `ActivePowerOutVariable`
    # (e.g. Storage `BookKeeping`, Source `ImportExportSourceModel`) accumulate
    # headroom from the net of out − in.
    _accumulate_in_out_headroom!(
        pf_data, container, sys,
        input_key_map[:active_power_in], input_key_map[:active_power_out],
        n_time_steps, bus_types, computed_gspf)

    # Rebuild bus_slack_pf in one pass. Per-cell writes into the existing CSC matrix
    # would trigger O(nnz) structural inserts whenever runtime headroom appears at
    # (bus, t) pairs outside the t=1-derived sparsity pattern PFS init creates — which
    # is the common case for renewables with intermittent availability. PowerFlowData
    # is immutable, so we mutate the CSC's internal arrays in place to preserve identity.
    n_buses = size(pf_data.bus_active_power_range, 1)
    nnz_hint = count(>(0.0), pf_data.bus_active_power_range)
    I_idx = Int[]
    J_idx = Int[]
    V_val = Float64[]
    sizehint!(I_idx, nnz_hint)
    sizehint!(J_idx, nnz_hint)
    sizehint!(V_val, nnz_hint)
    for t in 1:n_time_steps, bus_ix in 1:n_buses
        R_k = pf_data.bus_active_power_range[bus_ix, t]
        R_k > 0.0 || continue
        push!(I_idx, bus_ix)
        push!(J_idx, t)
        push!(V_val, R_k)
    end
    new_sparse = SparseArrays.sparse(I_idx, J_idx, V_val, n_buses, n_time_steps)
    resize!(bus_slack_pf.nzval, length(new_sparse.nzval))
    copyto!(bus_slack_pf.nzval, new_sparse.nzval)
    resize!(bus_slack_pf.rowval, length(new_sparse.rowval))
    copyto!(bus_slack_pf.rowval, new_sparse.rowval)
    resize!(bus_slack_pf.colptr, length(new_sparse.colptr))
    copyto!(bus_slack_pf.colptr, new_sparse.colptr)
    return
end

"Fetch the most recently solved `PowerFlowEvaluationData`"
function latest_solved_power_flow_evaluation_data(container::OptimizationContainer)
    datas = get_power_flow_evaluation_data(container)
    idx = findlast(x -> x.is_solved, datas)
    isnothing(idx) && error(
        "No power flow evaluation converged; cannot read back power-flow aux variables. " *
        "Check the convergence @error logged during the solve.",
    )
    return datas[idx]
end

function solve_power_flow!(
    pf_e_data::PowerFlowEvaluationData,
    container::OptimizationContainer,
    sys::PSY.System)
    pf_data = get_power_flow_data(pf_e_data)
    if PFS.supports_multi_period(pf_data)
        update_pf_data!(pf_e_data, container)
        _update_headroom_participation_factors!(
            pf_data, container, sys, get_input_key_map(pf_e_data))
        PFS.solve_power_flow!(pf_data)
    else
        for t in get_time_steps(container)
            update_pf_data!(pf_e_data, container, t)
            PFS.solve_power_flow!(pf_data)
        end
    end
    pf_e_data.is_solved = _check_pf_converged(pf_data)
    return
end

# Containers that actually solve a power flow report convergence via `get_converged`;
# pure exporters (PSSEExporter) never solve and are always considered "solved".
function _check_pf_converged(pf_data::PFS.PowerFlowData)
    flags = PFS.get_converged(pf_data)
    converged = all(flags)
    converged || @error(
        "Power flow evaluation $(typeof(pf_data)) failed to converge for $(count(!, flags)) of " *
        "$(length(flags)) time step(s) (indices $(findall(!, flags))); the corresponding " *
        "aux-variable values will be NaN.",
    )
    return converged
end
_check_pf_converged(::PFS.PSSEExporter) = true

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
        sample_line = first(parallel_brs)
        impedance = PSY.get_r(sample_line) + im * PSY.get_x(sample_line)
        first_name = PSY.get_name(sample_line)
        for br in parallel_brs
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
