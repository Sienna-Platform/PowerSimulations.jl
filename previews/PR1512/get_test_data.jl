using Cbc
using PowerSimulations
using PowerSystems
using DataStructures
using InfrastructureSystems
import InfrastructureSystems as IS
import PowerSimulations as PSI
import PowerSystems as PSY

include("../../../test/test_utils/get_test_data.jl")

abstract type TestOpProblem <: PSI.DefaultDecisionProblem end

system = build_c_sys5_re(; add_reserves = true)
solver = optimizer_with_attributes(Cbc.Optimizer)

devices = Dict{Symbol, DeviceModel}(
    :Generators => DeviceModel(ThermalStandard, ThermalBasicDispatch),
    :Loads => DeviceModel(PowerLoad, StaticPowerLoad),
)
branches = Dict{Symbol, DeviceModel}(
    :L => DeviceModel(Line, StaticLine),
    :T => DeviceModel(Transformer2W, StaticBranch),
    :TT => DeviceModel(TapTransformer, StaticBranch),
);
services = Dict{Symbol, ServiceModel}();

template = PSI.ProblemTemplate(CopperPlatePowerModel, devices, branches, services);

operation_problem = PSI.DecisionModel(TestOpProblem, template, system; optimizer = solver);

set_services_template!(
    operation_problem,
    Dict(
        :Reserve => ServiceModel(VariableReserve{ReserveUp}, RangeReserve),
        :Down_Reserve => ServiceModel(VariableReserve{ReserveDown}, RangeReserve),
    ),
)

op_results = solve!(operation_problem)
