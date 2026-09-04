# PowerSimulations.jl — Claude Guide

Platform-wide Sienna conventions (performance, type stability, formatter, environments, code style) live in `.claude/Sienna.md` and the `sienna-psy6` skill — read them too. This file is repo-specific and does not restate them.

## Scope (psy6 line, post-excision)

PSI is the **simulation orchestration** package of the psy6 line. It runs optimization models in the loop over time, keeps the simulation state, updates parameters and initial conditions between solves, stores results, and reads them back. It does **not** build optimization models. That job belongs to two upstream packages:

- **InfrastructureOptimizationModels (IOM)** — domain-neutral core: `OptimizationContainer`, `DecisionModel{M}`, `EmulationModel{M}`, `ModelInternal`, `Settings`, per-model stores, datasets (`InMemoryDataset`, `HDF5Dataset`, `DatasetContainer`), `OptimizationProblemOutputs`, generic builders, objective functions, status enums.
- **PowerOperationsModels (POM)** — power formulations: every device/service/network/HVDC/storage/hydro/hybrid formulation, `PowerOperationsProblemTemplate`, the `AbstractPowerOperationProblem` problem chain, per-model `build!`/`solve!`/`run!`, `build_problem!`, feedforward types and their constraint construction, parameter types and `add_parameters!`, initial-condition allocation, the initialization sub-problem, and PF-in-the-loop via `ext/PowerFlowsExt`.

Rule of thumb for where a change goes: **if it needs `SimulationState`, a `SimulationStore`, or knowledge of more than one model, it is PSI. If it adds a variable, constraint, parameter, or expression to a container, it is POM (or IOM if domain-neutral).** Never port formulation code back into PSI.

`using PowerSimulations` re-exports the full IOM and POM public API, so user scripts need only one `using`.

PSI was cut down to this scope by the excision plan in `.claude/plans/2026-09-02-pom-excision.md`, with its spec and file classification in `2026-09-02-pom-excision-spec.md`. Commit-by-commit progress and the final counts are in `.claude/plans/excision-progress.md`. These are local, gitignored working notes — not tracked in git, not on other clones.

### Where PSI sits

```
IS  ──▶ IOM ──▶ POM ──▶ PSI ──▶ PowerAnalytics / PowerGraphics
IS  ──▶ PSY ──▶ POM
PSY ──▶ PNM ──▶ POM
PSY ──▶ PF  ──▶ POM
PSY ──▶ PSB
```

- **Upstream deps:** IOM, POM, PSY, IS, PNM (reduced-network branch time-series routing in parameter updates), HDF5, JuMP, DataFrames, Distributed.
- **Not deps anymore:** PowerModels (POM network models are native), PowerFlows (PF-in-the-loop is POM's extension; PSI only needs PF in tests), Distributions (events only, fenced).
- **Downstream:** PowerAnalytics / PowerGraphics consume `SimulationResults`. Results storage, key encoding (`"VariableType__ComponentType"`), and serialization changes have downstream blast radius.
- **Co-dev wiring:** `Project.toml` and `test/Project.toml` `[sources]` carry git rev pins (IS `IS4`, IOM `main`, PSY `psy6`, PNM `psy6`, POM `main`; test env also pins PSB `psy6`, PF `psy6`, PowerFlowFileParser `psy6`, PowerTableDataParser `psy6`) plus the OpenAPI git pins PSY needs — this is what CI (`julia-buildpkg`/`julia-runtest`) resolves from. To co-dev locally against an in-progress sibling checkout without touching these tracked pins, `Pkg.develop(path="../<Sibling>.jl")` from `--project=.` or `--project=test`; that writes only to the gitignored `Manifest.toml`. Switch the tracked pins to `main`/released revs before a PR to `main`. No version bumps: PSI stays `0.38.3` until release.

## What PSI owns

- **`Simulation`** — orchestrates multi-model runs; `build!(sim)`, `execute!(sim)`.
- **`SimulationModels`** — vector of `DecisionModel`s + optional `EmulationModel`; horizon/interval/resolution reconciliation; assigns `SimulationInfo` (an IS type) to each model.
- **`SimulationSequence`** — execution order, feedforward attachment (`attach_feedforward!` is POM's), initial-condition chronologies (`InterProblemChronology`, `IntraProblemChronology`).
- **`SimulationState`** — `decision_states` and `system_states` as IOM `DatasetContainer{InMemoryDataset}`.
- **Parameter update between solves** — `update_parameter_values!`, PSI's methods of IOM's bare extension point `update_container_parameter_values!`, and `update_cost_parameters.jl` (time-varying cost refresh). These calls stay in PSI by decision; POM's standalone emulation does not update between steps.
- **Initial-condition update between solves** — PSI methods of `IOM.update_initial_conditions!` reading from `SimulationState`.
- **Simulation stores** — `HdfSimulationStore`, `InMemorySimulationStore` (subtypes of PSI's `SimulationStore`, distinct from IOM's per-model `AbstractModelStore`), output caches, `SimulationStoreParams`, `SimulationModelStoreRequirements`.
- **Results** — `SimulationResults`, `SimulationProblemResults`, realized reads, export, partitions and partition joins, recorder events.
- **Entry points into a model during a run** — `solve!(step, model, start_time, store::SimulationStore)` defined as methods of `POM.solve!`; `update_model!(model, sim)` which may rebuild via `reset_optimization_model!` + `POM.build_problem!`.

### Execution loop
read state → update feedforward and time-series parameters → update initial conditions → `IOM.solve_model!` → write results to `SimulationState` + `SimulationStore` → advance.

## src/ layout

```
src/
├── PowerSimulations.jl          # exports, explicit IOM imports, IOM/POM re-export loop, include order
├── core/                        # definitions (PSI constants), SimulationStore abstract, cache policy
│                                # event_keys.jl / event_model.jl — EVENTS-EXCISION, not included
├── operation/                   # problem types (GenericOpProblem, UC/ED), simulation solve! entry
│                                # points, update_model!/update_parameters! adapters, templates
├── initial_conditions/          # chronologies, between-solve IC update
├── parameters/                  # update_parameters, update_container_parameter_values,
│                                # update_cost_parameters (highest PSY-psy6 risk file)
├── simulation/                  # Simulation, models, sequence, state, stores, results, partitions
│                                # simulation_events.jl — EVENTS-EXCISION, not included
├── contingency_model/           # EVENTS-EXCISION, not included
└── utils/                       # recorder events, simulation show methods (print_pt_v3.jl),
                                 # store dimensions, CSV/system-filename helpers (file_utils.jl),
                                 # single-time-series resolution consistency checks
```

## Events are fenced, not gone

POM has only the four `EventParameter` types and no-op `add_event_arguments!`/`add_event_constraints!`. Until the event framework moves to POM, every event method in PSI sits inside a `#= EVENTS-EXCISION: … =#` block or behind a commented `include`. `grep -rn EVENTS-EXCISION src test` lists the follow-up. Do not delete fenced code, do not rewrite it, and do not add new event code outside a fence.

## Running tests, docs, formatter (verified commands for THIS repo)

```sh
# Formatter (run after every change; this is the project script)
julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'

# Compile check
julia --project=. -e 'using PowerSimulations'

# Full test suite (test env)
julia --project=test test/runtests.jl

# A single test file by name (runner uses @includetests ARGS)
julia --project=test test/runtests.jl test_simulation_build

# One file in isolation (loads the shared preamble first)
julia --project=test -e 'include("test/includes.jl"); include("test/test_simulation_store.jl")'

# Instantiate test env
julia --project=test -e 'using Pkg; Pkg.instantiate()'

# Build docs (must finish clean; a broken docs build is a task failure)
julia --project=docs docs/make.jl
```
- Test runner is the classic `@includetests ARGS` runner plus Aqua. Test files are `test_*.jl`; deps live in `test/Project.toml` with the same path pins as the package.
- Test fixtures come from PSB `PSITestSystems` (`c_sys5_uc`, `c_sys5_ed`, `c_sys5_hy_uc`, …). Hydro simulation tests use POM's native hydro formulations through `test/test_utils/operations_problem_templates.jl`.
- Test templates use POM names: `PowerOperationsProblemTemplate`, `CopperPlateNetworkModel`, `PTDFNetworkModel`, `DCPNetworkModel`, `ACPNetworkModel`. There is no `ProblemTemplate` alias.

## Conventions, invariants, gotchas

### Extend, never shadow
PSI methods on result and model types must extend the generic they belong to: `function IOM.get_system(res::SimulationProblemResults)`, `function POM.solve!(step::Int, …)`, `function IOM.update_initial_conditions!(…)`. A bare `function get_system(…)` inside PSI creates a second function that shadows the imported one instead of extending it — check with `parentmodule` on the live binding, or grep for a bare `function <name>(` where an `IOM.`/`POM.` qualified definition exists for the same name.

### `get_available_components` is two different functions
`PSY.get_available_components(sys, Type)` and `IOM.get_available_components(device_model, sys)` are unrelated functions that share a name. PSI imports the PSY one bare (for system-level reads) and calls the IOM one qualified as `IOM.get_available_components` everywhere it needs the device-model-aware version (`parameters/update_container_parameter_values.jl`, `parameters/update_cost_parameters.jl`). Never assume the bare name resolves to IOM's version.

### `COST_EPSILON` is defined by both IOM and POM
Both packages `export COST_EPSILON` as their own `const COST_EPSILON = 1e-3`. Re-exporting both via the `names(m)` loop in `src/PowerSimulations.jl` would leave the name unbound (ambiguous) in `PowerSimulations`. PSI resolves this with an explicit `import InfrastructureOptimizationModels: COST_EPSILON` — do not remove it, and do not try to "unify" the two upstream constants into one.

### `populate_units` is gone, not silently ignored
psy6 PowerSystems removed the system-wide unit base (`with_units_base` / `set_units_base_system!` / `get_units_base` no longer exist). `SimulationResults`/`SimulationProblemResults`'s `populate_units` keyword is kept only to error loudly — passing anything but `nothing` raises `IS.InvalidValue` explaining the unit base is gone (`src/simulation/simulation_results.jl`). Never resurrect silent handling for it.

### `SimulationStore` and IOM's `AbstractModelStore` use different verbs, on purpose
PSI's `SimulationStore` (`HdfSimulationStore`, `InMemorySimulationStore`) keeps `write_result!` / `read_result` / `read_results`. IOM's per-model `AbstractModelStore` (`DecisionModelStore`, `EmulationModelStore`) uses `write_output!` / `read_outputs`. These are deliberately different names for different layers — do not rename either side to "unify" them.

### Realized-results export goes through a bridge method
The generic export entry point is `IOM.export_realized_outputs`, which calls `IOM.read_outputs_with_keys` — a method IOM only defines for its own `OptimizationProblemOutputs`. `src/simulation/simulation_problem_results.jl` adds `function IOM.read_outputs_with_keys(res::SimulationProblemResults, ...)` that forwards to PSI's own `read_results_with_keys`, so `SimulationProblemResults` satisfies the same export path without IOM code changes.

### Initial-condition values can legitimately be `Nothing`
A must-run device has no meaningful `InitialTimeDurationOn`, so per-model IC values are dispatched (`_ic_value_is_missing(::Nothing) = true` / `::Any = false`), never `isa`-checked. `_ic_values_reconciled` (`src/simulation/simulation.jl`) treats all-missing as consistent, all-present as a numeric comparison, and a **mix** of missing/present across models as a real mismatch to report. Do not "fix" this by filtering out `nothing` — that would hide genuine cross-model disagreement.

### Unexported IOM surface
Most of IOM's model-lifecycle accessors (`get_store`, `set_status!`, `get_output_dir`, `advance_execution_count!`, dataset functions, `ModelStoreParams`, `LOG_GROUP_SIMULATION_STORE`, `set_interval!`, …) are not exported. PSI imports them explicitly in `src/PowerSimulations.jl`. Add to that block; do not qualify at call sites and do not ask IOM to export them for PSI's sake.

### IOM refuses simulation-owned models
`IOM.OptimizationProblemOutputs(model)` errors with "Model Solved as part of a Simulation" when the model store is empty. That is by design: simulation results come from PSI's `SimulationStore`, never from the per-model store.

### `update_cost_parameters.jl` mirrors POM build-time code
Time-varying cost refresh must call the same IOM/POM objective-function functions the build uses (`market_bid_plumbing.jl`, `value_curve_cost.jl`). psy6 cost curves carry the unit marker as a type parameter and `IS.UnitSystem` is gone. Every `PSY.get_*` on a convertible field passes the unit system explicitly.

### Never modify IOM's `optimization_container.jl`
Same rule as before, one package down. Push fixes into POM's `construct_device!` / `add_parameters!` chain or PSI's update chain.

### No silent absence-sentinel skips
Do not add `isnothing(x) && continue` guards that hide malformed-data bugs; let the next call surface the data error. When triaging bot review comments, mark "add a nothing-skip" suggestions as invalid.

### Parameter multiplier `fill!` optimizations need a manual audit
Whether a parameter type has a uniform multiplier across all `(device, time)` cells is NOT statically derivable. Frame `fill!` proposals as "for uniform-multiplier types" and ask which qualify. Hot path in PSI: `parameters/update_container_parameter_values.jl`.

### Results time-series recovery (downstream coupling)
`populate_system=true` can return a `System` with 0 time series. `TimeSeriesAttributes.component_name_to_ts_uuid` (now IOM `core/parameter_container.jl`) is populated at build time in POM but not serialized — relevant when touching `_serialize_systems_to_json` in `simulation.jl`.

## Follow-ups tracked outside this file
Events framework to POM then unfence; AGC (`AGCReserveDeployment` dropped; POM's `agc.jl` is not compiled); service feedforwards (POM errors on `attach_feedforward!(::ServiceModel, …)`); path pins → git pins before PR; PowerAnalytics/PowerGraphics re-validation; `test/runtests.jl` drops `Aqua.find_persistent_tasks_deps`/`Aqua.test_persistent_tasks` (they resolve a throwaway env from the registry, which can't satisfy PowerSystems' unregistered OpenAPI deps on the psy6 line) — re-enable once those packages are registered, matching the same exclusion already in POM's and IOM's test suites.

### Upstream bugs found during the excision (not PSI's to fix)

- **IOM: frozen PWL breakpoints — fixed.** Was a correctness bug in
  `../InfrastructureOptimizationModels.jl/src/objective_function/objective_function_pwl_delta.jl`:
  `add_pwl_block_offer_constraints!` discarded the width constraint's `ConstraintRef`, so
  time-varying market-bid breakpoints were frozen across a multi-step simulation. Fixed by IOM
  `f127bc5` (stores the width constraint refs) plus PSI `a2f9c7ec5`
  (`_update_pwl_width_constraint!` sets the width RHS between solves).
- **POM: standalone emulation does not update between executions.**
  `../PowerOperationsModels.jl/src/operation/emulation_model.jl` (~line 177) documents that its
  own run loop never calls the update hook, so a standalone `EmulationModel` run outside a
  `Simulation` produces no recorder events and no between-step updates. PSI's simulation path
  drives updates itself (see "Parameter update between solves" above) and is unaffected.

## Default branch
`main` (not `master`). Diff/PR/review against `origin/main`. Excision work is on `jd/pom_excision` with per-task commits (a decision scoped to that effort); elsewhere the global rule holds: never `git commit`, leave changes unstaged.
