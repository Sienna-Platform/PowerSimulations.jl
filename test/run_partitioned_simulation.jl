using PowerSimulations
using PowerSystems
using PowerSystemCaseBuilder
using InfrastructureSystems
using PowerNetworkMatrices
using Logging
using Test

import PowerModels as PM
using DataFrames
using Dates
using JuMP
using TimeSeries
using CSV
using DataFrames
using DataStructures
import UUIDs
using Random
import Serialization

import PowerSystems as PSY
import PowerSimulations as PSI
import InfrastructureSystems as IS

const PSB = PowerSystemCaseBuilder

const BASE_DIR = string(dirname(dirname(pathof(PowerSimulations))))
const DATA_DIR = joinpath(BASE_DIR, "test/test_data")

include(joinpath(BASE_DIR, "test/test_utils/solver_definitions.jl"))

# avoid redefinition of functions and constants when running on CI
if get(ENV, "CI", nothing) != "true"
    include(joinpath(BASE_DIR, "test/test_utils/common_operation_model.jl"))
    include(joinpath(BASE_DIR, "test/test_utils/model_checks.jl"))
    include(joinpath(BASE_DIR, "test/test_utils/mock_operation_models.jl"))
    include(joinpath(BASE_DIR, "test/test_utils/operations_problem_templates.jl"))
end

function build_simulation(
    output_dir::AbstractString,
    simulation_name::AbstractString,
    partitions::Union{Nothing, SimulationPartitions} = nothing,
    index::Union{Nothing, Integer} = nothing;
    initial_time = nothing,
    num_steps = nothing,
    HiGHS_optimizer = HiGHS_optimizer,
)
    if isnothing(partitions) && isnothing(num_steps)
        error("num_steps must be set if partitions is nothing")
    end
    if !isnothing(partitions) && !isnothing(num_steps)
        error("num_steps and partitions cannot both be set")
    end
    c_sys5_pjm_da = PSB.build_system(PSISystems, "c_sys5_pjm")
    PSY.transform_single_time_series!(c_sys5_pjm_da, Hour(48), Hour(24))
    c_sys5_pjm_rt = PSB.build_system(PSISystems, "c_sys5_pjm_rt")
    PSY.transform_single_time_series!(c_sys5_pjm_rt, Hour(1), Hour(1))

    for sys in [c_sys5_pjm_da, c_sys5_pjm_rt]
        th = get_component(ThermalStandard, sys, "Park City")
        set_active_power_limits!(th, (min = 0.1, max = 1.7))
        set_status!(th, false)
        set_active_power!(th, 0.0)
        c = get_operation_cost(th)
        PSY.set_start_up!(c, 1500.0)
        PSY.set_shut_down!(c, 75.0)
        set_time_at_status!(th, 1)

        th = get_component(ThermalStandard, sys, "Alta")
        set_time_limits!(th, (up = 5, down = 1))
        set_active_power_limits!(th, (min = 0.05, max = 0.4))
        set_active_power!(th, 0.05)
        c = get_operation_cost(th)
        PSY.set_start_up!(c, 400.0)
        PSY.set_shut_down!(c, 200.0)
        set_time_at_status!(th, 2)

        th = get_component(ThermalStandard, sys, "Brighton")
        set_active_power_limits!(th, (min = 2.0, max = 6.0))
        c = get_operation_cost(th)
        set_active_power!(th, 4.88041)
        PSY.set_start_up!(c, 5000.0)
        PSY.set_shut_down!(c, 3000.0)

        th = get_component(ThermalStandard, sys, "Sundance")
        set_active_power_limits!(th, (min = 1.0, max = 2.0))
        set_time_limits!(th, (up = 5, down = 1))
        set_active_power!(th, 2.0)
        c = get_operation_cost(th)
        PSY.set_start_up!(c, 4000.0)
        PSY.set_shut_down!(c, 2000.0)
        set_time_at_status!(th, 1)

        th = get_component(ThermalStandard, sys, "Solitude")
        set_active_power_limits!(th, (min = 1.0, max = 5.2))
        set_ramp_limits!(th, (up = 0.0052, down = 0.0052))
        set_active_power!(th, 2.0)
        c = get_operation_cost(th)
        PSY.set_start_up!(c, 3000.0)
        PSY.set_shut_down!(c, 1500.0)
        PSY.set_must_run!(th, true)
        set_status!(th, true)
    end

    to_json(
        c_sys5_pjm_da,
        joinpath(output_dir, "PSI-5-BUS-UC-ED/c_sys5_pjm_da.json");
        force = true,
    )
    to_json(
        c_sys5_pjm_rt,
        joinpath(output_dir, "PSI-5-BUS-UC-ED/c_sys5_pjm_rt.json");
        force = true,
    )

    template_uc = template_unit_commitment()
    set_network_model!(
        template_uc,
        NetworkModel(
            PTDFPowerModel;
        ),
    )

    set_device_model!(template_uc, ThermalStandard, ThermalStandardUnitCommitment)
    template_ed = deepcopy(template_uc)
    # template_ed.network_model.use_slacks = true
    set_device_model!(template_ed, ThermalStandard, ThermalBasicDispatch)

    models = SimulationModels(;
        decision_models = [
            DecisionModel(
                template_uc,
                c_sys5_pjm_da;
                optimizer = HiGHS_optimizer,
                name = "UC",
                initialize_model = true,
            ),
            DecisionModel(
                template_ed,
                c_sys5_pjm_rt;
                optimizer = HiGHS_optimizer,
                name = "ED",
                calculate_conflict = true,
                initialize_model = true,
            ),
        ],
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
        ),
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = simulation_name,
        steps = isnothing(partitions) ? num_steps : partitions.num_steps,
        models = models,
        sequence = sequence,
        simulation_folder = output_dir,
        initial_time = initial_time,
    )

    status =
        build!(sim; partitions = partitions, index = index)
    if status != PSI.SimulationBuildStatus.BUILT
        error("Failed to build simulation: status=$status")
    end

    return sim
end

function execute_simulation(sim, args...; kwargs...)
    return execute!(sim)
end
