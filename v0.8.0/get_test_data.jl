using Cbc
using PowerSimulations
using PowerSystems
using DataStructures
using InfrastructureSystems
const IS = InfrastructureSystems
const PSI = PowerSimulations
const PSY = PowerSystems
Cbc_optimizer = optimizer_with_attributes(Cbc.Optimizer)

include("../../../test/test_utils/get_test_data.jl")

abstract type TestOpProblem <: PSI.AbstractOperationsProblem end

system = build_c_sys5_re(; add_reserves = true)
solver = optimizer_with_attributes(Cbc.Optimizer)

devices = Dict{Symbol, DeviceModel}(
    :Generators => DeviceModel(ThermalStandard, ThermalDispatch),
    :Loads => DeviceModel(PowerLoad, StaticPowerLoad),
)
branches = Dict{Symbol, DeviceModel}(
    :L => DeviceModel(Line, StaticLine),
    :T => DeviceModel(Transformer2W, StaticTransformer),
    :TT => DeviceModel(TapTransformer, StaticTransformer),
);
services = Dict{Symbol, ServiceModel}();

template = PSI.OperationsProblemTemplate(CopperPlatePowerModel, devices, branches, services);

operations_problem = PSI.OperationsProblem(
    TestOpProblem,
    template,
    system;
    optimizer = solver,
    use_parameters = true,
);

set_services_template!(
    operations_problem,
    Dict(
        :Reserve => ServiceModel(VariableReserve{ReserveUp}, RangeReserve),
        :Down_Reserve => ServiceModel(VariableReserve{ReserveDown}, RangeReserve),
    ),
)

op_results = solve!(operations_problem)
re_results = PSI.run_economic_dispatch(system; optimizer = solver, use_parameters = true)
