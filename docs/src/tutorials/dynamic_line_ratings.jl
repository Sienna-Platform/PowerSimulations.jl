#!nb # ```@meta
#!nb # EditURL = "dynamic_line_ratings.jl"
#!nb # ```
#!nb #
# # Dynamic Line Ratings (DLR)
#
# ## Introduction
#
# Static branch ratings use a fixed thermal limit ``R^\text{max}`` for each transmission
# line. **Dynamic Line Ratings (DLR)** replace this fixed limit with a time-varying
# parameter ``R^\text{max}_t``, allowing the optimizer to exploit periods when ambient
# conditions (wind, temperature) permit higher line flows. This reduces curtailment and
# can lower total generation cost compared to conservative static limits.
#
# This tutorial demonstrates how to:
#
# 1. Attach a DLR time series to transmission branches in a `PowerSystems.System`.
# 2. Build a `PTDFPowerModel` template that activates the DLR constraints.
# 3. Run a multi-step simulation and read the resulting line flows and DLR parameters.
#
# !!! note
#
#     Dynamic Line Ratings are **only** supported for the `StaticBranch` formulation
#     combined with a `PTDFPowerModel` (or any `AbstractPTDFModel`) network model.
#     Using `DynamicBranchRatingTimeSeriesParameter` with `StaticBranchBounds`,
#     `StaticBranchUnbounded`, or a non-PTDF network model will raise an error during
#     `build!`.

# ### Load packages

using PowerSystems
using PowerSimulations
using HydroPowerSimulations
using PowerNetworkMatrices
using PowerSystemCaseBuilder
using HiGHS
using Dates
using TimeSeries

# ### Optimizer

solver = optimizer_with_attributes(HiGHS.Optimizer, "mip_rel_gap" => 0.01)

# ## Data
#
# !!! note
#
#     [PowerSystemCaseBuilder.jl](https://github.com/NREL-Sienna/PowerSystemCaseBuilder.jl)
#     is a helper library that makes it easier to reproduce examples in the documentation
#     and tutorials. Normally you would pass your local files to create the system data
#     instead of calling `build_system`. For more details visit
#     [PowerSystemCaseBuilder Documentation](https://nrel-sienna.github.io/PowerSystems.jl/stable/how_to/powersystembuilder/)

sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")

# ## Preparing DLR Time Series
#
# DLR is represented in `PowerSystems.jl` as a `SingleTimeSeries` (or `Deterministic`)
# attached directly to each branch component. The time series values are scaling factors
# applied to the branch's static `get_rating`. A value of `1.15` means the line can carry
# 15% more than its rated static capacity during that hour; a value of `0.95` represents a
# de-rating.
#
# The helper function below iterates over a list of branch names, constructs a
# `SingleTimeSeries` from a vector of hourly scaling factors, and attaches it to each
# branch with `scaling_factor_multiplier = get_rating` so PowerSimulations knows how to
# convert the factor to a per-unit limit.

function add_dlr_to_system_branches!(
    sys::System,
    branches_dlr::Vector{String},
    n_steps::Int,
    dlr_factors::Vector{Float64};
    initial_date::String = "2020-01-01",
)
    for branch_name in branches_dlr
        branch = get_component(ACTransmission, sys, branch_name)

        data_ts = collect(
            DateTime("$initial_date 0:00:00", "y-m-d H:M:S"):Hour(1):(
                DateTime("$initial_date 23:00:00", "y-m-d H:M:S") + Day(n_steps - 1)
            ),
        )

        dlr_data = TimeArray(data_ts, dlr_factors)

        PowerSystems.add_time_series!(
            sys,
            branch,
            PowerSystems.SingleTimeSeries(
                "dynamic_line_ratings",
                dlr_data;
                scaling_factor_multiplier = get_rating,
            ),
        )
    end
end

# Define DLR scaling factors. Here we use a daily cycle of four blocks repeated across the
# simulation horizon: the early morning hours are de-rated (0.95), mid-day has higher
# capacity (1.15 and 1.05), and evening hours have intermediate capacity (0.95).

n_steps = 2       # simulation length in days
initial_date = "2020-01-01"

dlr_factors_daily = vcat([fill(x, 6) for x in [1.15, 1.05, 0.95, 0.95]]...)  # 24 values
dlr_factor_ts = repeat(dlr_factors_daily, n_steps)

# Select the branch names that will receive DLR time series. These names must match
# branches present in the system.

branches_dlr = [
    "A2", "AB1", "A24", "B10", "B18", "CA-1", "C22", "C34",
    "A7", "A17", "B14", "B15", "C7", "C17",
]

add_dlr_to_system_branches!(sys, branches_dlr, n_steps, dlr_factor_ts; initial_date)

# Because the simulation uses a rolling horizon of 48 hours (2 days), we transform the
# `SingleTimeSeries` into `Deterministic` forecasts with a 48-hour horizon and a 24-hour
# interval between forecast windows.

transform_single_time_series!(sys, Hour(48), Day(1))

# ## Define the Problem Template
#
# The template must use `PTDFPowerModel` to enable DLR constraints. The key step is
# constructing `DeviceModel` with `time_series_names` that maps
# `DynamicBranchRatingTimeSeriesParameter` to the time series name `"dynamic_line_ratings"`
# attached to the branches above.
#
# !!! tip
#
#     Any branch type that has the `"dynamic_line_ratings"` time series attached and is
#     configured with `DynamicBranchRatingTimeSeriesParameter` in `time_series_names` will
#     have time-varying flow limits. Branches without the time series attached will fall
#     back to static limits automatically.

template_uc = ProblemTemplate(
    NetworkModel(
        PTDFPowerModel;
        reduce_radial_branches = false,
        use_slacks = false,
        PTDF_matrix = PTDF(sys),
    ),
)

# ### Branch models with DLR enabled

line_device_model = DeviceModel(
    Line,
    StaticBranch;
    time_series_names = Dict(
        DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
    ),
)

tap_transformer_device_model = DeviceModel(
    TapTransformer,
    StaticBranch;
    time_series_names = Dict(
        DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings",
    ),
)

set_device_model!(template_uc, line_device_model)
set_device_model!(template_uc, tap_transformer_device_model)

# ### Injection device models

set_device_model!(template_uc, ThermalStandard, ThermalStandardUnitCommitment)
set_device_model!(template_uc, RenewableDispatch, RenewableFullDispatch)
set_device_model!(template_uc, RenewableNonDispatch, FixedOutput)
set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc, HydroDispatch, HydroDispatchRunOfRiver)
set_device_model!(
    template_uc,
    DeviceModel(TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless),
)

# ### Reserve models

set_service_model!(template_uc, ServiceModel(VariableReserve{ReserveUp}, RangeReserve))
set_service_model!(template_uc, ServiceModel(VariableReserve{ReserveDown}, RangeReserve))

# ## Build and Run a Simulation
#
# We wrap the `DecisionModel` in a `Simulation` to run multiple steps. Each step solves a
# 48-hour unit commitment problem and advances the clock by 24 hours.

model = DecisionModel(
    template_uc,
    sys;
    name = "UC",
    optimizer = solver,
    system_to_file = false,
    initialize_model = true,
    store_variable_names = true,
)

models = SimulationModels(; decision_models = [model])

sequence = SimulationSequence(;
    models = models,
    ini_cond_chronology = InterProblemChronology(),
)

sim = Simulation(;
    name = "DLR_example",
    steps = n_steps,
    models = models,
    initial_time = DateTime(initial_date * "T00:00:00"),
    sequence = sequence,
    simulation_folder = mktempdir(; cleanup = true),
)

build!(sim)

execute!(sim)

# ## Inspecting Results
#
# ### Line flows
#
# Retrieve the realized active power flows for `Line` and `TapTransformer` branches. Each
# column corresponds to one branch; each row to one time step.

results = SimulationResults(sim)
uc_results = get_decision_problem_results(results, "UC")

line_flows = read_realized_expression(
    uc_results,
    "PTDFBranchFlow__Line";
    table_format = TableFormat.WIDE,
)

transformer_flows = read_realized_expression(
    uc_results,
    "PTDFBranchFlow__TapTransformer";
    table_format = TableFormat.WIDE,
)

# ### DLR parameter values
#
# The DLR parameters that were applied at each time step can be read back from the results.
# The values are in per-unit (MW if multiplied by base power) and already account for the
# `scaling_factor_multiplier = get_rating` applied when the time series was attached.

dlr_params = read_parameter(
    uc_results,
    "DynamicBranchRatingTimeSeriesParameter__Line";
    table_format = TableFormat.WIDE,
)
first(keys(dlr_params))

# !!! tip
#
#     To verify that DLR constraints are binding, compare the line flows in `line_flows`
#     against the corresponding DLR parameter values in `dlr_params`. When demand is high
#     and the DLR limit is tight, the flow should be at or near the DLR limit rather than
#     the static rating.
