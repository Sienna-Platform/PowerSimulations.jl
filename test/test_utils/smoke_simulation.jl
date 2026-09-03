# test/test_utils/smoke_simulation.jl — the fastest end-to-end check that a simulation builds,
# executes, and reads results back. Not part of the test suite (no @includetests glob picks up
# files under test_utils/); run standalone with:
#   julia --project=test test/test_utils/smoke_simulation.jl
using PowerSimulations
using PowerSystems
using PowerSystemCaseBuilder
using HiGHS
using Dates
using Logging
import PowerSystemCaseBuilder: PSITestSystems
const PSI = PowerSimulations

c_sys5_uc = build_system(PSITestSystems, "c_sys5_uc")
c_sys5_ed = build_system(PSITestSystems, "c_sys5_ed")

solver = optimizer_with_attributes(
    HiGHS.Optimizer,
    "mip_rel_gap" => 0.01,
    "output_flag" => false,
)

include(joinpath(@__DIR__, "operations_problem_templates.jl"))

template_uc = test_template_unit_commitment(CopperPlateNetworkModel)
template_ed = test_template_economic_dispatch(CopperPlateNetworkModel)

models = SimulationModels(;
    decision_models = [
        DecisionModel(template_uc, c_sys5_uc; name = "UC", optimizer = solver),
        DecisionModel(template_ed, c_sys5_ed; name = "ED", optimizer = solver),
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
    name = "smoke",
    steps = 2,
    models = models,
    sequence = sequence,
    simulation_folder = mktempdir(; cleanup = true),
)

build_out = build!(sim; console_level = Logging.Error)
@assert build_out == PSI.SimulationBuildStatus.BUILT
exec_out = execute!(sim; enable_progress_bar = false)
@assert exec_out == PSI.RunStatus.SUCCESSFULLY_FINALIZED

results = SimulationResults(sim)
ed = get_decision_problem_results(results, "ED")
df = read_realized_variable(ed, "ActivePowerVariable__ThermalStandard")
# c_sys5_ed's default load forecast carries 5-minute look-ahead points, so ED dispatches
# (and realizes) every 5 minutes: 2 days * 24h * 12 (5-min steps/hour) = 576 timestamps,
# in PSI's default long table format (one row per component per timestamp) times the 5
# ThermalStandard units in c_sys5_ed = 2880 rows.
@assert size(df, 1) == 2880
println("smoke OK: ", size(df))
