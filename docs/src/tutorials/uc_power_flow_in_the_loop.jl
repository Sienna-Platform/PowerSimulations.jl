#src # EXECUTE = TRUE
# # [Running Power Flow In The Loop with Unit Commitment](@id uc-inloop-pf)
#
# In this tutorial, you'll configure a unit commitment (UC) simulation that
# automatically calls an AC power flow solver from
# [`PowerFlows.jl`](https://sienna-platform.github.io/PowerFlows.jl/stable/) at every dispatch interval.
#
# You'll validate the committed dispatch against the full AC network model at each hour, check for branch overloads, and export the results to PSS/e format if needed for further analysis.
#
# This tutorial builds on the tutorial for
# [Running a Multi-Stage Production Cost Simulation](@ref).

# ## Setup
#
# Load the needed packages and define basic inputs:

using PowerSystemCaseBuilder
using PowerSimulations
using PowerFlows
using PowerSystems
using DataFrames
using HiGHS
using Dates
using Logging

disable_logging(Logging.Warn)

const SCENARIO_NAME = "uc_with_pf"
const INLOOP_SYSTEM = "modified_RTS_GMLC_DA_sys"
const INLOOP_SIM_STEPS = 1

run_dir = joinpath(".", "results")
mkpath(run_dir)
export_dir = joinpath(run_dir, "psse_exports")
mkpath(export_dir)

# Build a test [`PowerSystems.System`](@extref) via
# [`PowerSystemCaseBuilder.build_system`](@extref).
# We use `modified_RTS_GMLC_DA_sys` (24+ DA forecast steps):

sys = build_system(PSISystems, INLOOP_SYSTEM; skip_serialization = true)

# ## Configuring the Power Flow Solver
#
# Create an [`PowerFlows.ACPowerFlow`](@extref) solver. We attach a
# [`PowerFlows.PSSEExportPowerFlow`](@extref) exporter so that
# we can automatically write a PSS/e `.raw` file for each solved interval.

psse_export = PSSEExportPowerFlow(;
    psse_version = :v33,
    export_dir = export_dir,
    overwrite = true,
)

power_flow_model = ACPowerFlow(; exporter = psse_export)

# ## Building the UC Problem Template
#
# Create a [`ProblemTemplate`](@ref) with `power_flow_evaluation` set to the solver we just
# configured. This is the key step that enables power flow in the loop:

template_uc = ProblemTemplate(
    NetworkModel(
        PTDFPowerModel;
        use_slacks = true,
        power_flow_evaluation = power_flow_model,
    ),
)

set_device_model!(
    template_uc,
    ThermalStandard,
    ThermalStandardUnitCommitment,
)
set_device_model!(template_uc, RenewableDispatch, RenewableFullDispatch)
set_device_model!(template_uc, RenewableNonDispatch, FixedOutput)
set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc, Line, StaticBranchUnbounded)
set_device_model!(template_uc, Transformer2W, StaticBranchUnbounded)
set_device_model!(template_uc, TapTransformer, StaticBranchUnbounded)
set_device_model!(template_uc, MonitoredLine, StaticBranch)

# `use_slacks = true` allows the simulation to remain feasible when there is a small
# mismatch between the PTDF-based UC network model and the full AC power flow.

# ## Building and Executing the Simulation
#
# Set up the simulation with a HiGHS optimizer and run for one simulation step.
# The in-step horizon on `modified_RTS_GMLC_DA_sys` provides 24 hourly realized rows.

solver = optimizer_with_attributes(
    HiGHS.Optimizer,
    "log_to_console" => false,
    "mip_rel_gap" => 0.05,
    "time_limit" => 900.0,
)

models = SimulationModels(;
    decision_models = [
        DecisionModel(
            template_uc,
            sys;
            name = "UC",
            optimizer = solver,
            store_variable_names = true,
        ),
    ],
)

sequence = SimulationSequence(;
    models = models,
    ini_cond_chronology = InterProblemChronology(),
)

sim = Simulation(;
    name = SCENARIO_NAME,
    steps = INLOOP_SIM_STEPS,
    models = models,
    sequence = sequence,
    simulation_folder = run_dir,
)

build!(sim; console_level = Logging.Error)
execute!(sim; enable_progress_bar = true)

# ## Loading Simulation Results
#
# Load the simulation results with [`SimulationResults`](@ref)
# and extract the UC problem results with [`get_decision_problem_results`](@ref):

sim_results = SimulationResults(sim)
uc_results = get_decision_problem_results(sim_results, "UC")

# Notice the "UC Problem Auxiliary variables Results" table, which lists the active and
# reactive power flow and bus voltage magnitude and angle results from the AC power flow  (e.g., `PowerFlowBranchReactivePowerFromTo__Line`, `PowerFlowVoltageMagnitude__ACBus`).
# These are not output when a UC problem is run alone.

# ## PTDF UC Flows vs. AC Power Flow In the Loop
#
# Now, we'll compare the PTDF UC flows to the AC power flow results. 
#
# The UC stage optimizes flows using the PTDF network model, and the line flows are not variables; they are recorded in the `PTDFBranchFlow__*` expressions.
#
# First, load in the PTDF branch flows for one line:

ptdf_flows = read_realized_expression(uc_results, "PTDFBranchFlow__Line")
example_line = first(unique(ptdf_flows.name))
ptdf_line = filter(row -> row.name == example_line, ptdf_flows)

# After each in-step hour, the AC power flow writes branch flows into auxiliary variables
# such as `PowerFlowBranchActivePowerFromTo__Line`.
# Compare PTDF expression flows to AC flows for our selected line:

pf_flows_ft = read_realized_aux_variable(
    uc_results,
    "PowerFlowBranchActivePowerFromTo__Line",
)
pf_line = filter(row -> row.name == example_line, pf_flows_ft)

innerjoin(
    rename(select(ptdf_line, :DateTime, :value), :value => :PTDF_MW),
    rename(select(pf_line, :DateTime, :value), :value => :AC_PF_MW);
    on = :DateTime,
)

# With `use_slacks = true` in the template, small differences between PTDF and AC flows
# are expected — the UC model can absorb minor mismatches. The AC auxiliary flows should track
# the PTDF values closely when the network is well conditioned.

# ## Checking for Branch Overloads
#
# Now, we will run a post-hoc check on our UC results to scan every interval for branches whose AC flow exceeds the thermal rating.

# First, build a helper to extract branch flow limits from the [`PowerSystems.System`](@extref) — `name` in the
# auxiliary results matches the branch name on each [`PowerSystems.ACBranch`](@extref).
# AC lines and transformers use [`get_rating`](@extref PowerSystems.get_rating-Tuple{Line}); HVDC branches use active-power limits instead.

function _branch_flow_limit_mw(branch, sys)
    base_power = get_base_power(sys)
    try
        if branch isa TwoTerminalVSCLine
            return get_rating(branch) * base_power
        elseif branch isa TwoTerminalHVDC
            return max(
                get_active_power_limits_from(branch).max,
                get_active_power_limits_to(branch).max,
            )
        elseif branch isa GenericArcImpedance
            return get_max_flow(branch) * base_power
        else
            return get_rating(branch) * base_power
        end
    catch
        return Inf
    end
end

ratings = Dict{String, Float64}()
for b in get_components(ACBranch, sys)
    ratings[get_name(b)] = _branch_flow_limit_mw(b, sys)
end

# Then, scan every interval for branches whose AC flow exceeds the thermal rating:

overloads = DataFrame(;
    DateTime = Dates.DateTime[],
    name = String[],
    P_from_to = Float64[],
    rating = Float64[],
)

for row in eachrow(pf_flows_ft)
    rating = get(ratings, row.name, Inf)
    if abs(row.value) > rating
        push!(overloads, (row.DateTime, row.name, row.value, rating))
    end
end

overloads

# Notice we have multiple overloads in this example.
# Each row identifies a congestion event: the interval (`DateTime`),
# branch name (`name`), actual AC flow in MW (`P_from_to`), and thermal rating (`rating`).
# In-loop AC power flow records realized flows but does not add thermal-limit constraints
# to the UC optimization.
# The UC model's network representation could be too loose if overloads appear — add transmission
# constraints, re-run UC, and repeat the AC power flow check until the table is empty.

# ## PSS/e Export Files
#
# The [`PowerFlows.PSSEExportPowerFlow`](@extref) exporter writes under
# `export_dir` (here, `results/psse_exports`). After running, we have one folder per solved
# interval in the 24-hour in-step horizon, plus the 24-hour lookahead:

psse_export_folders = readdir(export_dir)
length(psse_export_folders)

# Each folder contains the PSS/e `.raw` file for that interval, available for further analysis:

first_subdir = first(sort(psse_export_folders))
readdir(joinpath(export_dir, first_subdir))

# ## Next Steps
#
# - Tighten the UC network model with transmission constraints if needed until overloads are removed — see 
#   the formulation library [Network](@ref network_formulations) and [`PowerSystems.Branch` Formulations](@ref) pages.
# - See the [Solving a Power Flow](@extref Solving-a-Power-Flow)
#   tutorial in PowerFlows.jl for standalone power flow examples.

rm(run_dir; force = true, recursive = true) #hide
