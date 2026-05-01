# # [Operations problems with [PowerSimulations.jl](https://github.com/NREL-Sienna/PowerSimulations.jl)](@id op_problem_tutorial)
#
# **Originally Contributed by**: Clayton Barrows
#
# ## Introduction
#
# `PowerSimulations.jl` supports the construction and solution of optimal power system
# scheduling problems (Operations Problems). Operations problems form the fundamental
# building blocks for sequential simulations. This example shows how to specify and customize
# the mathematics that will be applied to the data with a [`ProblemTemplate`](@ref),
# build and execute a [`DecisionModel`](@ref), and access the results.

using PowerSystems
using PowerSimulations
using HydroPowerSimulations
using PowerSystemCaseBuilder
using HiGHS # solver
using Dates

# ## Data
#
# !!! note
#
#     [PowerSystemCaseBuilder.jl](https://github.com/NREL-Sienna/PowerSystemCaseBuilder.jl)
#     is a helper library that makes it easier to reproduce examples in the documentation
#     and tutorials. Normally you would pass your local files to create the system data
#     instead of calling the function `build_system`.
#     For more details visit
#     [PowerSystemCaseBuilder Documentation](https://nrel-sienna.github.io/PowerSystems.jl/stable/how_to/powersystembuilder/)

sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")

# ## Define a problem specification with a `ProblemTemplate`
#
# You can create an empty template with:

template_uc = ProblemTemplate()

# Now, you can add a [`DeviceModel`](@ref) for each device type to create an assignment
# between PowerSystems device types and the subtypes of `AbstractDeviceFormulation`.
# PowerSimulations has a variety of different `AbstractDeviceFormulation` subtypes
# that can be applied to different PowerSystems device types, each dispatching to different
# methods for populating optimization problem objectives, variables, and constraints.
# Documentation on the formulation options for various devices can be found in the
# [formulation library docs](https://nrel-sienna.github.io/PowerSimulations.jl/latest/formulation_library/General/#formulation_library)

# ### Branch Formulations
#
# Here is an example of relatively standard branch formulations. Other formulations allow
# for selective enforcement of transmission limits and greater control on transformer settings.

set_device_model!(template_uc, Line, StaticBranch)
set_device_model!(template_uc, Transformer2W, StaticBranch)
set_device_model!(template_uc, TapTransformer, StaticBranch)

# ### Injection Device Formulations
#
# Here we define template entries for all devices that inject or withdraw power on the
# network. For each device type, we can define a distinct `AbstractDeviceFormulation`. In
# this case, we're defining a basic unit commitment model for thermal generators,
# curtailable renewable generators, and fixed dispatch (net-load reduction) formulations
# for `HydroDispatch` and `RenewableNonDispatch` devices.

set_device_model!(template_uc, ThermalStandard, ThermalStandardUnitCommitment)
set_device_model!(template_uc, RenewableDispatch, RenewableFullDispatch)
set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc, HydroDispatch, HydroDispatchRunOfRiver)
set_device_model!(template_uc, RenewableNonDispatch, FixedOutput)

# ### Service Formulations
#
# We have two `VariableReserve` types, parameterized by their direction. So, similar to
# creating [`DeviceModel`](@ref)s, we can create [`ServiceModel`](@ref)s. The primary difference being
# that [`DeviceModel`](@ref) objects define how constraints get created, while [`ServiceModel`](@ref) objects
# define how constraints get modified.

set_service_model!(template_uc, VariableReserve{ReserveUp}, RangeReserve)
set_service_model!(template_uc, VariableReserve{ReserveDown}, RangeReserve)

# ### Network Formulations
#
# Finally, we can define the transmission network specification that we'd like to model.
# For simplicity, we'll choose a copper plate formulation. But there are dozens of
# specifications available through an integration with
# [PowerModels.jl](https://lanl-ansi.github.io/PowerModels.jl/stable/).
#
# *Note that many formulations will require appropriate data and may be computationally intractable*

set_network_model!(template_uc, NetworkModel(CopperPlatePowerModel))

# ## `DecisionModel`
#
# Now that we have a `System` and a [`ProblemTemplate`](@ref), we can put the two together
# to create a [`DecisionModel`](@ref) that we solve.

# ### Optimizer
#
# It's most convenient to define an optimizer instance upfront and pass it into the
# [`DecisionModel`](@ref) constructor. For this example, we can use the free HiGHS solver
# with a relatively relaxed MIP gap (`ratioGap`) setting to improve speed.

solver = optimizer_with_attributes(HiGHS.Optimizer, "mip_rel_gap" => 0.5)

# ### Build a `DecisionModel`
#
# The construction of a [`DecisionModel`](@ref) essentially applies a [`ProblemTemplate`](@ref)
# to `System` data to create a JuMP model.

problem = DecisionModel(template_uc, sys; optimizer = solver, horizon = Hour(24))
build!(problem; output_dir = mktempdir())

# !!! tip
#
#     The principal component of the [`DecisionModel`](@ref) is the JuMP model.
#     But you can serialize to a file using the following command:
#
#     ```julia
#     serialize_optimization_model(problem, save_path)
#     ```
#
#     Keep in mind that if the setting `"store_variable_names"` is set to `False` then
#     the file won't show the model's names.

# ### Solve a `DecisionModel`

solve!(problem)

# ## Results Inspection
#
# PowerSimulations collects the [`DecisionModel`](@ref) results into a
# `OptimizationProblemResults` struct:

res = OptimizationProblemResults(problem)

# ### Optimizer Stats
#
# The optimizer summary is included

get_optimizer_stats(res)

# ### Objective Function Value

get_objective_value(res)

# ### Variable, Parameter, Auxiliary Variable, Dual, and Expression Values
#
# The solution value data frames for variables, parameters, auxiliary variables, duals, and
# expressions can be accessed using the `read_` methods:

read_variables(res)

# Or, you can read a single parameter value for parameters that exist in the results.

list_parameter_names(res)
read_parameter(res, "ActivePowerTimeSeriesParameter__RenewableDispatch")

# ## Plotting
#
# Take a look at the plotting capabilities in
# [PowerGraphics.jl](https://nrel-sienna.github.io/PowerGraphics.jl/stable/)
