const DeviceModelForBranches = DeviceModel{<:PSY.Branch, <:AbstractDeviceFormulation}
const BranchModelContainer = Dict{Symbol, DeviceModelForBranches}

function _check_pm_formulation(::Type{T}) where {T <: PM.AbstractPowerModel}
    if !isconcretetype(T)
        throw(
            ArgumentError(
                "The network model must contain only concrete types, $(T) is an Abstract Type",
            ),
        )
    end
end

_maybe_flatten_pfem(pfem::Vector{PFS.PowerFlowEvaluationModel}) = pfem
_maybe_flatten_pfem(pfem::PFS.PowerFlowEvaluationModel) =
    PFS.flatten_power_flow_evaluation_model(pfem)

"""
Establishes the NetworkModel for a given PowerModels formulation type.

# Arguments
- `::Type{T}` where `T <: PM.AbstractPowerModel`: the power-system formulation type.

# Accepted keyword arguments
- `use_slacks::Bool` = false
    Adds slack buses to the network modeling.
- `PTDF_matrix::Union{PNM.PowerNetworkMatrix, Nothing}` = nothing
    PTDF/VirtualPTDF matrix produced by PowerNetworkMatrices (optional).
- `LODF_matrix::Union{PNM.PowerNetworkMatrix, Nothing}` = nothing
    LODF/VirtualLODF matrix produced by PowerNetworkMatrices (optional).
- `reduce_radial_branches::Bool` = false
    Enable radial branch reduction when building network matrices.
- `reduce_degree_two_branches::Bool` = false
    Enable degree-two branch reduction when building network matrices.
- `subnetworks::Dict{Int, Set{Int}}` = Dict()
    Optional mapping of reference bus → set of mapped buses. If not provided,
    subnetworks are inferred from PTDF/VirtualPTDF or discovered from the system.
- `duals::Vector{DataType}` = Vector{DataType}()
    Constraint types for which duals should be recorded.
- `power_flow_evaluation::Union{PFS.PowerFlowEvaluationModel, Vector{PFS.PowerFlowEvaluationModel}}`
    Power-flow evaluation model(s). A single model is flattened to a vector internally.

# Notes
- `modeled_ac_branch_types` and `reduced_branch_tracker` are internal fields managed by the model.
- `subsystem` can be set after construction via `set_subsystem!(model, id)`.
- PTDF/LODF inputs are validated against the requested reduction flags and may raise
  a ConflictingInputsError if they are inconsistent with `reduce_radial_branches`
  or `reduce_degree_two_branches`.

# Examples
ptdf = PNM.VirtualPTDF(system)
nw = NetworkModel(PTDFPowerModel; PTDF_matrix = ptdf, reduce_radial_branches = true,
                  power_flow_evaluation = PFS.PowerFlowEvaluationModel())

nw2 = NetworkModel(CopperPlatePowerModel; subnetworks = Dict(1 => Set([1,2,3])))
"""
mutable struct NetworkModel{T <: PM.AbstractPowerModel}
    use_slacks::Bool
    PTDF_matrix::Union{Nothing, PNM.PowerNetworkMatrix}
    LODF_matrix::Union{Nothing, PNM.PowerNetworkMatrix}
    subnetworks::Dict{Int, Set{Int}}
    bus_area_map::Dict{PSY.ACBus, Int}
    duals::Vector{DataType}
    network_reduction::PNM.NetworkReductionData
    reduce_radial_branches::Bool
    reduce_degree_two_branches::Bool
    power_flow_evaluation::Vector{PFS.PowerFlowEvaluationModel}
    subsystem::Union{Nothing, String}
    hvdc_network_model::Union{Nothing, AbstractHVDCNetworkModel}
    modeled_ac_branch_types::Vector{DataType}
    reduced_branch_tracker::BranchReductionOptimizationTracker

    function NetworkModel(
        ::Type{T};
        use_slacks = false,
        PTDF_matrix = nothing,
        LODF_matrix = nothing,
        reduce_radial_branches = false,
        reduce_degree_two_branches = false,
        subnetworks = Dict{Int, Set{Int}}(),
        duals = Vector{DataType}(),
        power_flow_evaluation::Union{
            PFS.PowerFlowEvaluationModel,
            Vector{PFS.PowerFlowEvaluationModel},
        } = PFS.PowerFlowEvaluationModel[],
        hvdc_network_model = nothing,
    ) where {T <: PM.AbstractPowerModel}
        _check_pm_formulation(T)
        new{T}(
            use_slacks,
            PTDF_matrix,
            LODF_matrix,
            subnetworks,
            Dict{PSY.ACBus, Int}(),
            duals,
            PNM.NetworkReductionData(),
            reduce_radial_branches,
            reduce_degree_two_branches,
            _maybe_flatten_pfem(power_flow_evaluation),
            nothing,
            hvdc_network_model,
            Vector{DataType}(),
            BranchReductionOptimizationTracker(),
        )
    end
end

get_use_slacks(m::NetworkModel) = m.use_slacks
get_PTDF_matrix(m::NetworkModel) = m.PTDF_matrix
get_LODF_matrix(m::NetworkModel) = m.LODF_matrix
get_reduce_radial_branches(m::NetworkModel) = m.reduce_radial_branches
get_network_reduction(m::NetworkModel) = m.network_reduction
get_duals(m::NetworkModel) = m.duals
get_network_formulation(::NetworkModel{T}) where {T} = T
get_reduced_branch_tracker(m::NetworkModel) = m.reduced_branch_tracker
get_reference_buses(m::NetworkModel{T}) where {T <: PM.AbstractPowerModel} =
    collect(keys(m.subnetworks))
get_subnetworks(m::NetworkModel) = m.subnetworks
get_bus_area_map(m::NetworkModel) = m.bus_area_map
get_power_flow_evaluation(m::NetworkModel) = m.power_flow_evaluation
has_subnetworks(m::NetworkModel) = !isempty(m.bus_area_map)
get_subsystem(m::NetworkModel) = m.subsystem
get_hvdc_network_model(m::NetworkModel) = m.hvdc_network_model

set_subsystem!(m::NetworkModel, id::String) = m.subsystem = id
set_hvdc_network_model!(m::NetworkModel, val::Union{Nothing, AbstractHVDCNetworkModel}) =
    m.hvdc_network_model = val

function add_dual!(model::NetworkModel, dual)
    dual in model.duals && error("dual = $dual is already stored")
    push!(model.duals, dual)
    @debug "Added dual" dual _group = LOG_GROUP_NETWORK_CONSTRUCTION
    return
end

function _get_filters(branch_models::BranchModelContainer)
    filters = Dict{DataType, Function}()
    for v in values(branch_models)
        filter_func = get_attribute(v, "filter_function")
        if filter_func !== nothing
            filters[get_component_type(v)] = filter_func
        end
    end
    return filters
end

function _get_irreducible_buses_due_to_dlrs(
    sys::PSY.System,
    network_model::NetworkModel,
    branch_models::BranchModelContainer,
)
    @debug "Identifying buses that are irreducible due to dynamic line ratings"
    irreducible_buses = Set{Int64}()
    for branch_type in network_model.modeled_ac_branch_types
        device_model = branch_models[Symbol(branch_type)]
        if !haskey(
            get_time_series_names(device_model),
            DynamicBranchRatingTimeSeriesParameter,
        )
            continue
        end

        if branch_type == PSY.ThreeWindingTransformer
            @warn "Dynamic branch ratings for ThreeWindingTransformers are not implemented yet. Skipping it."
            continue
        end

        ts_name =
            get_time_series_names(device_model)[DynamicBranchRatingTimeSeriesParameter]
        ts_type = PSY.Deterministic #TODO workaround since we dont have the container

        branches = PSY.get_available_components(branch_type, sys)
        for branch in branches
            if !PSY.has_time_series(branch, ts_type, ts_name)
                continue
            end
            bus_to = PSY.get_number(PSY.get_to(PSY.get_arc(branch)))
            bus_from = PSY.get_number(PSY.get_from(PSY.get_arc(branch)))
            push!(irreducible_buses, bus_to)
            push!(irreducible_buses, bus_from)
        end
    end
    return collect(irreducible_buses)
end

function instantiate_network_model!(
    model::NetworkModel{T},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
) where {T <: PM.AbstractPowerModel}
    if isempty(model.subnetworks)
        model.subnetworks = PNM.find_subnetworks(sys)
    end
    irreducible_buses = _get_irreducible_buses_due_to_dlrs(
        sys,
        model,
        branch_models,
    )
    if model.reduce_radial_branches && model.reduce_degree_two_branches
        @info "Applying both radial and degree two reductions"
        ybus = PNM.Ybus(
            sys;
            network_reductions = PNM.NetworkReduction[
                PNM.RadialReduction(; irreducible_buses = irreducible_buses),
                PNM.DegreeTwoReduction(; irreducible_buses = irreducible_buses),
            ],
        )
    elseif model.reduce_radial_branches
        @info "Applying radial reduction"
        if !isempty(irreducible_buses)
            @warn "Irreducible buses identified due to DLRs. The reduction of any radial branch between 2 irreducible buses wil be ignored"
        end
        ybus =
            PNM.Ybus(
                sys;
                network_reductions = PNM.NetworkReduction[PNM.RadialReduction(;
                    irreducible_buses = irreducible_buses,
                )],
            )
    elseif model.reduce_degree_two_branches
        @info "Applying degree two reduction"
        ybus = PNM.Ybus(
            sys;
            network_reductions = PNM.NetworkReduction[PNM.DegreeTwoReduction(;
                irreducible_buses = irreducible_buses,
            )],
        )
    else
        ybus = PNM.Ybus(sys)
    end
    model.network_reduction = deepcopy(PNM.get_network_reduction_data(ybus))
    #if !isempty(model.network_reductionget_net_reduction_data)
    # TODO: Network reimplement this when it becomes necessary. We don't have any
    # reductions that are incompatible right now.
    # check_network_reduction_compatibility(T)
    #end
    PNM.populate_branch_maps_by_type!(model.network_reduction, _get_filters(branch_models))
    empty!(model.reduced_branch_tracker)
    set_number_of_steps!(model.reduced_branch_tracker, number_of_steps)
    return
end

function instantiate_network_model!(
    model::NetworkModel{AreaBalancePowerModel},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
)
    PNM.populate_branch_maps_by_type!(model.network_reduction)
    empty!(model.reduced_branch_tracker)
    set_number_of_steps!(model.reduced_branch_tracker, number_of_steps)
    return
end

function instantiate_network_model!(
    model::NetworkModel{CopperPlatePowerModel},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
)
    if isempty(model.subnetworks)
        model.subnetworks = PNM.find_subnetworks(sys)
    end
    if length(model.subnetworks) > 1
        @debug "System Contains Multiple Subnetworks. Assigning buses to subnetworks."
        model.network_reduction = deepcopy(PNM.get_network_reduction_data(PNM.Ybus(sys)))
        _assign_subnetworks_to_buses(model, sys)
    end
    empty!(model.reduced_branch_tracker)
    set_number_of_steps!(model.reduced_branch_tracker, number_of_steps)
    return
end

function instantiate_network_model!(
    model::NetworkModel{<:AbstractPTDFModel},
    branch_models::BranchModelContainer,
    number_of_steps::Int,
    sys::PSY.System,
)
    irreducible_buses = _get_irreducible_buses_due_to_dlrs(
        sys,
        model,
        branch_models,
    )
    if get_PTDF_matrix(model) === nothing || !isempty(irreducible_buses)
        if get_PTDF_matrix(model) !== nothing
            @warn "Provided PTDF Matrix is being ignored since irreducible buses were identified because of DLRs. Recalculating PTDF Matrix with PowerNetworkMatrices.VirtualPTDF and the identified irreducible buses."
        else
            @info "No PTDF Matrix provided. Calculating using PowerNetworkMatrices.VirtualPTDF"
        end

        if model.reduce_radial_branches && model.reduce_degree_two_branches
            @info "Applying both radial and degree two reductions"
            ptdf = PNM.VirtualPTDF(
                sys;
                network_reductions = PNM.NetworkReduction[
                    PNM.RadialReduction(; irreducible_buses = irreducible_buses),
                    PNM.DegreeTwoReduction(;
                        irreducible_buses = irreducible_buses,
                    ),
                ],
            )
        elseif model.reduce_radial_branches
            @info "Applying radial reduction"
            if !isempty(irreducible_buses)
                @warn "Irreducible buses identified due to DLRs. The reduction of any radial branch between 2 irreducible buses wil be ignored"
            end
            ptdf = PNM.VirtualPTDF(
                sys;
                network_reductions = PNM.NetworkReduction[PNM.RadialReduction(;
                    irreducible_buses = irreducible_buses,
                )],
            )
        elseif model.reduce_degree_two_branches
            @info "Applying degree two reduction"
            ptdf = PNM.VirtualPTDF(
                sys;
                network_reductions = PNM.NetworkReduction[PNM.DegreeTwoReduction(;
                    irreducible_buses = irreducible_buses,
                )],
            )
        else
            ptdf = PNM.VirtualPTDF(sys)
        end
        model.PTDF_matrix = ptdf
        model.network_reduction = deepcopy(ptdf.network_reduction_data)
    else
        model.network_reduction = deepcopy(model.PTDF_matrix.network_reduction_data)
    end

    if !model.reduce_radial_branches && PNM.has_radial_reduction(
        PNM.get_reductions(model.PTDF_matrix.network_reduction_data),
    )
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has reduced radial branches and mismatches the network \
                model specification reduce_radial_branches = false. Set the keyword argument \
                reduce_radial_branches = true in your network model"),
        )
    end
    if !model.reduce_degree_two_branches && PNM.has_degree_two_reduction(
        PNM.get_reductions(model.PTDF_matrix.network_reduction_data),
    )
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has reduced degree two branches and mismatches the network \
                model specification reduce_degree_two_branches = false. Set the keyword argument \
                reduce_degree_two_branches = true in your network model"),
        )
    end
    if model.reduce_radial_branches &&
       PNM.has_ward_reduction(PNM.get_reductions(model.PTDF_matrix.network_reduction_data))
        throw(
            IS.ConflictingInputsError(
                "The provided PTDF Matrix has  a ward reduction specified and the keyword argument \\
                reduce_radial_branches = true. Set the keyword argument reduce_radial_branches = false \\
                or provide a modified PTDF Matrix without the Ward reduction."),
        )
    end

    if model.reduce_radial_branches
        @assert !isempty(model.PTDF_matrix.network_reduction_data)
    end
    model.subnetworks = _make_subnetworks_from_subnetwork_axes(model.PTDF_matrix)
    if length(model.subnetworks) > 1
        @debug "System Contains Multiple Subnetworks. Assigning buses to subnetworks."
        _assign_subnetworks_to_buses(model, sys)
    end
    PNM.populate_branch_maps_by_type!(model.network_reduction, _get_filters(branch_models))
    empty!(model.reduced_branch_tracker)
    set_number_of_steps!(model.reduced_branch_tracker, number_of_steps)
    return
end

function _make_subnetworks_from_subnetwork_axes(ptdf::PNM.PTDF)
    subnetworks = Dict{Int, Set{Int}}()
    for (ref_bus, ptdf_axes) in ptdf.subnetwork_axes
        subnetworks[ref_bus] = Set(ptdf_axes[1])
    end
    return subnetworks
end

function _make_subnetworks_from_subnetwork_axes(ptdf::PNM.VirtualPTDF)
    subnetworks = Dict{Int, Set{Int}}()
    for (ref_bus, ptdf_axes) in ptdf.subnetwork_axes
        subnetworks[ref_bus] = Set(ptdf_axes[2])
    end
    return subnetworks
end

function _assign_subnetworks_to_buses(
    model::NetworkModel{T},
    sys::PSY.System,
) where {T <: Union{CopperPlatePowerModel, AbstractPTDFModel}}
    subnetworks = model.subnetworks
    temp_bus_map = Dict{Int, Int}()
    network_reduction = PSI.get_network_reduction(model)
    for bus in PSI.get_available_components(model, PSY.ACBus, sys)
        bus_no = PSY.get_number(bus)
        mapped_bus_no = PNM.get_mapped_bus_number(network_reduction, bus)
        mapped_bus_no ∈ network_reduction.removed_buses && continue
        if haskey(temp_bus_map, bus_no)
            model.bus_area_map[bus] = temp_bus_map[bus_no]
            continue
        else
            bus_mapped = false
            for (subnet, bus_set) in subnetworks
                if mapped_bus_no ∈ bus_set
                    temp_bus_map[bus_no] = subnet
                    model.bus_area_map[bus] = subnet
                    bus_mapped = true
                    break
                end
            end
        end
        if !bus_mapped
            error(
                "Bus $(PSY.summary(bus)) not mapped to any reference bus: Mapped bus number: $(mapped_bus_no)",
            )
        end
    end
    return
end

_assign_subnetworks_to_buses(
    ::NetworkModel{T},
    ::PSY.System,
) where {T <: PM.AbstractPowerModel} = nothing

function get_reference_bus(
    model::NetworkModel{T},
    b::PSY.ACBus,
)::Int where {T <: PM.AbstractPowerModel}
    if isempty(model.bus_area_map)
        return first(keys(model.subnetworks))
    else
        return model.bus_area_map[b]
    end
end
