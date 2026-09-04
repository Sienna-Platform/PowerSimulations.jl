#!nb # ```@meta
#!nb # EditURL = "pcm_simulation.jl"
#!nb # ```
#!nb #
# # Running a Multi-Stage Production Cost Simulation
#
# ## Introduction
#
# PowerSimulations.jl supports simulations that consist of sequential optimization problems
# where results from previous problems inform subsequent problems in a variety of ways. This
# example demonstrates some of these capabilities to represent electricity market clearing.
# `PowerSimulations.jl` re-exports the device, service, and network formulations from
# `PowerOperationsModels.jl`, so a single `using PowerSimulations` is enough to build the
# `DecisionModel`s below.
#
# ### Load Packages

using PowerSystems
using PowerSimulations
import PowerSimulations as PSI
using PowerSystemCaseBuilder
import PowerSystemCaseBuilder: PSITestSystems
using Dates
using HiGHS #solver

# ### Optimizer
#
# It's most convenient to define an optimizer instance upfront and pass it into the
# `DecisionModel` constructor. For this example, we can use the free HiGHS solver with a
# relatively relaxed MIP gap setting to improve speed.

solver = optimizer_with_attributes(HiGHS.Optimizer, "mip_rel_gap" => 0.5)

# !!! note
#
#     Defining a solver upfront ensures that only one license is requested when using a license-limited solver, such as Gurobi. We can create a environment variable and pass it to the optimizer constructor for shared license use if using such a solver
#
#     ```julia
#     using Gurobi
#
#     gurobi_env = Gurobi.Env()
#
#     solver = optimizer_with_attributes(() -> Gurobi.Optimizer(gurobi_env),"MIPGap" => 0.01)
#     ```
#
#     Conversely, if a unique optimizer constructor is defined within the SimulationModels for each stage, a separate license will be obtained for each stage.
#
# ### Hourly day-ahead system
#
# First, we'll create a `System` with hourly data to represent day-ahead forecasted wind,
# solar, and load profiles:

sys_DA = build_system(PSITestSystems, "c_sys5_uc")

# ### 5-Minute system
#
# The same test data also includes 5-minute resolution time series data. So, we can create
# another `System` to represent look-ahead forecasted data for a "real-time" market:

sys_RT = build_system(PSITestSystems, "c_sys5_ed")

# ## `PowerOperationsProblemTemplate`s define stages
#
# Sequential simulations in PowerSimulations are created by defining problem templates
# that represent stages, and how information flows between executions of a stage and
# between different stages.
#
# Let's start by defining a two stage simulation that might look like a typical day-Ahead
# and real-time electricity market clearing process.
#
# ### Day-ahead unit commitment stage
#
# First, we define a unit commitment template for the day ahead problem, using
# `ThermalStandardUnitCommitment` for the thermal generators.

template_uc = PowerOperationsProblemTemplate(CopperPlateNetworkModel)
set_device_model!(template_uc, ThermalStandard, ThermalStandardUnitCommitment)
set_device_model!(template_uc, RenewableDispatch, RenewableFullDispatch)
set_device_model!(template_uc, RenewableNonDispatch, FixedOutput)
set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc, InterruptiblePowerLoad, PowerLoadInterruption)
set_device_model!(template_uc, Line, StaticBranch)
set_device_model!(template_uc, TwoWindingTransformer, StaticBranch)
set_device_model!(template_uc, TwoTerminalGenericHVDCLine, HVDCTwoTerminalDispatch)
set_service_model!(template_uc, OnlineReserve{ReserveUp}, RangeReserve)
set_service_model!(template_uc, OnlineReserve{ReserveDown}, RangeReserve)

# ### Define the reference model for the real-time economic dispatch
#
# We define a second template for the real-time problem, using
# `ThermalBasicDispatch` for the thermal generators and a PTDF network model with slacks:

template_ed = PowerOperationsProblemTemplate(
    NetworkModel(PTDFNetworkModel; use_slacks = true),
)
set_device_model!(template_ed, ThermalStandard, ThermalBasicDispatch)
set_device_model!(template_ed, RenewableDispatch, RenewableFullDispatch)
set_device_model!(template_ed, RenewableNonDispatch, FixedOutput)
set_device_model!(template_ed, PowerLoad, StaticPowerLoad)
set_device_model!(template_ed, InterruptiblePowerLoad, PowerLoadInterruption)
set_device_model!(template_ed, Line, StaticBranch)
set_device_model!(template_ed, TwoWindingTransformer, StaticBranch)
set_device_model!(template_ed, TwoTerminalGenericHVDCLine, HVDCTwoTerminalDispatch)
set_service_model!(template_ed, OnlineReserve{ReserveUp}, RangeReserve)
set_service_model!(template_ed, OnlineReserve{ReserveDown}, RangeReserve)

# ### Define the `SimulationModels`
#
# `DecisionModel`s define the problems that are executed in the simulation. The
# actual problem will change as the stage gets updated to represent different time periods,
# but the formulations applied to the components is constant within a stage. In this case, we
# want to define two stages with the `PowerOperationsProblemTemplate`s and the `System`s that
# we've already created.

models = SimulationModels(;
    decision_models = [
        DecisionModel(template_uc, sys_DA; optimizer = solver, name = "UC"),
        DecisionModel(template_ed, sys_RT; optimizer = solver, name = "ED"),
    ],
)

# ### `SimulationSequence`
#
# Similar to a `PowerOperationsProblemTemplate`, the `SimulationSequence` provides a template of
# how to execute a sequential set of operations problems.
#
# Let's review some of the `SimulationSequence` arguments.
#
# ### Chronologies
#
# In PowerSimulations, chronologies define where information is flowing. There are two types
# of chronologies.
#
#   - inter-stage chronologies: Define how information flows between stages. e.g. day-ahead solutions are used to inform economic dispatch problems
#   - intra-stage chronologies: Define how information flows between multiple executions of a single stage. e.g. the dispatch setpoints of the first period of an economic dispatch problem are constrained by the ramping limits from setpoints in the final period of the previous problem.
#
# ### Feedforwards
#
# The definition of exactly what information is passed using the defined chronologies is
# accomplished with feedforwards. Specifically, a feedforward is used
# to define what to do with information being passed with an inter-stage chronology. Let's
# define a `SemiContinuousFeedforward` that affects the semi-continuous range constraints of thermal generators
# in the economic dispatch problems based on the value of the unit-commitment variables.

feedforward = Dict(
    "ED" => [
        SemiContinuousFeedforward(;
            component_type = ThermalStandard,
            source = OnVariable,
            affected_values = [ActivePowerVariable],
        ),
    ],
)

# ### Sequencing
#
# The stage problem length, look-ahead, and other details surrounding the temporal Sequencing
# of stages are controlled using the structure of the time series data in the `System`s. Here,
# `sys_DA` is already set up to run a day-ahead unit commitment, and `sys_RT` an economic
# dispatch with 5-minute resolution look-ahead. Now we can put it all together to define a
# `SimulationSequence`:

DA_RT_sequence = SimulationSequence(;
    models = models,
    ini_cond_chronology = InterProblemChronology(),
    feedforwards = feedforward,
)

# ## `Simulation`
#
# Now, we can build and execute a simulation using the `SimulationSequence` and `Stage`s
# that we've defined.

path = mkdir(joinpath(".", "pcm-store")) #hide
sim = Simulation(;
    name = "pcm-test",
    steps = 2,
    models = models,
    sequence = DA_RT_sequence,
    simulation_folder = joinpath(".", "pcm-store"),
)

# ### Build simulation

build!(sim)

# ### Execute simulation
#
# the following command returns the status of the simulation (`SimulationBuildStatus.BUILT`
# is proper execution) and stores the results in a set of HDF5 files on disk.

execute!(sim; enable_progress_bar = false)

# ## Results
#
# To access the results, we need to load the simulation result metadata and then make
# requests to the specific data of interest. This allows you to efficiently access the
# results of interest without overloading resources.

results = SimulationResults(sim);
uc_results = get_decision_problem_results(results, "UC"); # UC stage result metadata
ed_results = get_decision_problem_results(results, "ED"); # ED stage result metadata

# We can read all the result variables

read_variables(uc_results)

# or all the parameters

read_parameters(uc_results)

# We can just list the variable names contained in `uc_results`:

list_variable_names(uc_results)

# and a number of parameters (this pattern also works for aux_variables, expressions, and duals)

list_parameter_names(uc_results)

# Now we can read the specific results of interest for a specific problem, time window (optional),
# and set of variables, duals, or parameters (optional)

Dict([
    v => read_variable(ed_results, v) for v in [
        "ActivePowerVariable__RenewableDispatch",
        "ActivePowerVariable__ThermalStandard",
    ]
])

# Or if we want the result of just one variable, parameter, or dual (must be defined in the
# problem definition), we can use:

read_parameter(
    ed_results,
    "ActivePowerTimeSeriesParameter__RenewableDispatch";
    count = 2,
)

# !!! info
#
# note that this returns the results of each execution step in a separate dataframe
# If you want the realized results (without lookahead periods), you can call `read_realized_*`:

read_realized_variables(
    uc_results,
    ["ActivePowerVariable__ThermalStandard"],
)
rm(path; force = true, recursive = true) #hide

# ## Plotting
#
# Take a look at the plotting capabilities in [PowerGraphics.jl](https://sienna-platform.github.io/PowerGraphics.jl/stable/)
