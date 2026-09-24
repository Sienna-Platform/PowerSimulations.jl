# PowerSimulations.jl — Claude Guide

**Status (2026-09-23): PSI is the psy6 simulation engine.** Multi-model simulations run end to end
on IOM+POM: build, execute, state, between-solve updates, events, outputs, partitions. The full
suite passes (29,995 tests). The psy6 line lives on branch `psy6`; `main` is the released psy5
line (no IOM/POM dependency).

Platform-wide Sienna conventions (performance, type stability, formatter, environments, code style) live in `.claude/Sienna.md` and the `sienna-psy6` skill — read them too. This file is repo-specific and does not restate them.

## Scope (psy6 line, post-excision)

PSI is the **simulation orchestration** package of the psy6 line. It runs optimization models in the loop over time, keeps the simulation state, updates parameters and initial conditions between solves, stores outputs, and reads them back. It does **not** build optimization models. That job belongs to two upstream packages:

- **InfrastructureOptimizationModels (IOM)** — domain-neutral core: `OptimizationContainer`, `DecisionModel{M}`, `EmulationModel{M}`, `ModelInternal`, `Settings`, per-model stores, datasets (`InMemoryDataset`, `HDF5Dataset`, `DatasetContainer`), `OptimizationProblemOutputs`, generic builders, objective functions, status enums.
- **PowerOperationsModels (POM)** — power formulations: every device/service/network/HVDC/storage/hydro/hybrid formulation, `PowerOperationsProblemTemplate`, the `AbstractPowerOperationProblem` problem chain, per-model `build!`/`solve!`/`run!`, `build_problem!`, feedforward types and their constraint construction, parameter types and `add_parameters!`, initial-condition allocation, the initialization sub-problem, and PF-in-the-loop via `ext/PowerFlowsExt`.

Rule of thumb for where a change goes: **if it needs `SimulationState`, a `SimulationStore`, or knowledge of more than one model, it is PSI. If it adds a variable, constraint, parameter, or expression to a container, it is POM (or IOM if domain-neutral).** Never port formulation code back into PSI.

`using PowerSimulations` re-exports the full IOM and POM public API, so user scripts need only one `using`.

PSI was cut down to this scope by the excision plan in `.claude/plans/2026-09-02-pom-excision.md`, with its spec and file classification in `2026-09-02-pom-excision-spec.md`. Commit-by-commit progress and the final counts are in `.claude/plans/excision-progress.md`. These are local, gitignored working notes — not tracked in git, not on other clones.

### Where PSI sits

```
IS  ──▶ IOM ──▶ POM ──▶ PSI ──▶ HydroPowerSimulations / StorageSystemsSimulations
IS  ──▶ PSY ──▶ POM          (PSI outputs) ··▶ PowerAnalytics ──▶ PowerGraphics
PSY ──▶ PNM ──▶ POM
PSY ──▶ PF  ──▶ POM  (weakdep, ext/PowerFlowsExt)
PSY ──▶ PSB  (test fixtures only)
```

- **Upstream deps:** IOM, POM, PSY, IS, PNM (reduced-network branch time-series routing in parameter updates), HDF5, JuMP, DataFrames, Distributed, plus the PowerOpenAPIModels subpackages PSY needs.
- **Not deps:** PowerModels (POM network models are native), PowerFlows (PF-in-the-loop is POM's extension; PSI only needs PF in tests), InfraStore (reached only through POM's parameter store, below).
- **Downstream:** `.github/workflows/cross-package-test.yml` runs HydroPowerSimulations, StorageSystemsSimulations, and PowerAnalytics against each PR. PowerAnalytics does **not** depend on PSI: it reads outputs through the `IS.Outputs` abstraction, so PSI's `SimulationProblemOutputs` must keep satisfying that interface. Output storage, key encoding (`"VariableType__ComponentType"`), and bundle layout changes have downstream blast radius.
- **Co-dev wiring:** `Project.toml` and `test/Project.toml` `[sources]` carry git rev pins plus the OpenAPI git pins PSY needs — this is what CI (`julia-buildpkg`/`julia-runtest`) resolves from. Base pins are IS `IS4`, IOM `main`, PSY/PNM `psy6`, POM `main`; the test env also pins PSB, PF, PowerFlowFileParser, PowerTableDataParser at `psy6`. A PR that needs unmerged upstream work pins those branches instead (as its own commit, with a comment naming the upstream PRs) and reverts once they merge. `docs/Project.toml` pins the same revs. To co-dev locally against a sibling checkout without touching tracked pins, `Pkg.develop(path="../<Sibling>.jl")` from `--project=test`; that writes only to the gitignored `test/Manifest.toml`. A dev path to a scratch directory goes stale when the scratch is cleared — delete the Manifest and re-instantiate to fall back to `[sources]`. No version bumps: PSI stays `0.38.3` until release.

### Outputs bundle (parameters and the outputs System)

Each model's outputs carry a **bundle** at `problems/<model>/system-<uuid>/`: `system.json` (the
System as an OpenAPI document), `time_series.h5`, and `time_series.h5.sqlite` (an InfraStore
store). Written at build by `_write_system_bundles!` (`simulation.jl`), filled at finalize by
`finalize_parameters!` (`hdf_simulation_store.jl`), and always written — there is no opt-out
keyword.

- **Parameters live in the bundle, not in HDF5.** `HdfSimulationStore` writes no HDF5 dataset for
  parameters. Each execution's multiplied parameter values are buffered and written as rows under
  the synthetic owner `POM.PARAMETER_ROW_OWNER_ID` (features `parameter`, `model`). `read_parameter`
  / `read_realized_parameters` read them back.
- **Model inputs are recast onto their components.** `buffer_parameter_inputs!` keeps the raw
  (un-multiplied) input values per execution; finalize writes them under each component's own id
  with features `{"source" => "parameter"}` — `Deterministic` windows for decision models,
  `SingleTimeSeries` for the emulation model. A System restored from the bundle can rebuild the
  same template and read the same inputs; it covers only the executed windows and need not be
  byte-equal to the input System.
- **Every forecast in a bundle shares the run's window grid** (InfraStore requires it per
  `(resolution, interval)`), so cost time-series copies are re-windowed to the planned grid;
  static series copy verbatim.
- **POM owns the store API.** `PowerOperationsModels.jl/src/operation/parameter_time_series_store.jl`
  is the only file in POM or PSI that touches InfraStore/IS store internals. PSI calls its public
  functions (`open_parameter_store`, `write_parameter_array!`, `write_parameter_windows!`,
  `read_parameter_array`, `read_parameter_windows`, `read_input_time_series`,
  `copy_cost_time_series!`, …) through the `_with_parameter_store` do-block helper. Never reach
  into InfraStore from PSI.
- **Partitions:** `join_simulation` merges each partition's parameter and input rows into the
  main bundle in one pass per partition. A re-join that brings windows an existing destination
  row lacks warns loudly instead of dropping them silently.

## What PSI does

PSI runs built `DecisionModel`s and `EmulationModel`s in a loop over time: it decides execution
order, moves data between models between solves, and produces **outputs** — the design rule this
codebase follows is that modeling tools (IOM, POM, PSI, PowerFlows) produce outputs, and "results"
is `PowerAnalytics.jl`'s term only. Every PSI type and verb below uses "outputs" for that reason
(`SimulationOutputs`, `SimulationProblemOutputs`, `write_output!`, `read_outputs`, `execute!`'s
`outputs_channel` keyword, `SimulationIntermediateOutput`, …).

## What PSI owns

- **`Simulation`** — orchestrates multi-model runs; `build!(sim)`, `execute!(sim)`.
- **`SimulationModels`** — vector of `DecisionModel`s + optional `EmulationModel`; horizon/interval/resolution reconciliation; assigns `SimulationInfo` (an IS type) to each model.
- **`SimulationSequence`** — execution order, feedforward attachment (`attach_feedforward!` is POM's), initial-condition chronologies (`InterProblemChronology`, `IntraProblemChronology`).
- **`SimulationState`** — `decision_states` and `system_states` as IOM `DatasetContainer{InMemoryDataset}`.
- **Parameter update between solves** — `update_parameter_values!`, PSI's methods of IOM's bare extension point `update_container_parameter_values!`, and `update_cost_parameters.jl` (time-varying cost refresh). These calls stay in PSI by decision; POM's standalone emulation does not update between steps.
- **Initial-condition update between solves** — PSI methods of `IOM.update_initial_conditions!` reading from `SimulationState`.
- **Simulation stores** — `HdfSimulationStore`, `InMemorySimulationStore` (subtypes of PSI's `SimulationStore`, distinct from IOM's per-model `AbstractModelStore`), output caches, `SimulationStoreParams`, `SimulationModelStoreRequirements`.
- **Outputs** — `SimulationOutputs`, `SimulationProblemOutputs`, realized reads, export, partitions and partition joins, recorder events. Named to match IOM (`OptimizationProblemOutputs`) and IS (`IS.Outputs`).
- **Entry points into a model during a run** — `solve!(step, model, start_time, store::SimulationStore)` defined as methods of `POM.solve!`; `update_model!(model, sim)` which may rebuild via `reset_optimization_model!` + `POM.build_problem!`.

### Execution loop
read state → update feedforward and time-series parameters → update initial conditions → `IOM.solve_model!` → write outputs to `SimulationState` + `SimulationStore` → advance.

## src/ layout

```
src/
├── PowerSimulations.jl          # exports, explicit IOM imports, IOM/POM re-export loop, include order
├── core/                        # definitions (PSI constants), SimulationStore abstract, cache policy
├── operation/                   # simulation solve! entry points on POM's DecisionModel/EmulationModel,
│                                # update_model!/update_parameters! adapters
├── initial_conditions/          # chronologies, between-solve IC update
├── parameters/                  # update_parameters, update_container_parameter_values,
│                                # update_cost_parameters (highest PSY-psy6 risk file)
├── simulation/                  # Simulation, models, sequence, state, events runtime,
│                                # HDF/in-memory stores + outputs bundle, outputs, partitions
└── utils/                       # recorder events, simulation show methods (print_pt_v3.jl),
                                 # store dimensions, CSV/system-filename helpers (file_utils.jl),
                                 # single-time-series resolution consistency checks
```

## Events: semantics in POM, runtime in PSI

POM owns the event framework (`EventModel`, `set_event_model!`, the event parameters and
constraints, outage countdown semantics). `SimulationSequence(; events = POM.EventModel[...])`
attaches event models to each model's template; `apply_simulation_events!`
(`simulation/simulation_events.jl`) is the runtime half — it reads the previous countdown from
`SimulationState`, evaluates the start condition, and writes values back each step. Events require
an emulation model. New event semantics go to POM; PSI only drives them.

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
PSI methods on output and model types must extend the generic they belong to: `function IOM.get_system(res::SimulationProblemOutputs)`, `function POM.solve!(step::Int, …)`, `function IOM.update_initial_conditions!(…)`. A bare `function get_system(…)` inside PSI creates a second function that shadows the imported one instead of extending it — check with `parentmodule` on the live binding, or grep for a bare `function <name>(` where an `IOM.`/`POM.` qualified definition exists for the same name.

### `get_available_components` is two different functions
`PSY.get_available_components(sys, Type)` and `IOM.get_available_components(device_model, sys)` are unrelated functions that share a name. PSI imports the PSY one bare (for system-level reads) and calls the IOM one qualified as `IOM.get_available_components` everywhere it needs the device-model-aware version (`parameters/update_container_parameter_values.jl`, `parameters/update_cost_parameters.jl`). Never assume the bare name resolves to IOM's version.

### `COST_EPSILON` is defined by both IOM and POM
Both packages `export COST_EPSILON` as their own `const COST_EPSILON = 1e-3`. Re-exporting both via the `names(m)` loop in `src/PowerSimulations.jl` would leave the name unbound (ambiguous) in `PowerSimulations`. PSI resolves this with an explicit `import InfrastructureOptimizationModels: COST_EPSILON` — do not remove it, and do not try to "unify" the two upstream constants into one.

### `populate_units` is gone, not silently ignored
psy6 PowerSystems removed the system-wide unit base (`with_units_base` / `set_units_base_system!` / `get_units_base` no longer exist). `get_decision_problem_outputs`/`get_emulation_problem_outputs`'s `populate_units` keyword is kept only to error loudly — passing anything but `nothing` raises `IS.InvalidValue` explaining the unit base is gone (`src/simulation/simulation_outputs.jl`). Never resurrect silent handling for it.

### `SimulationStore` and IOM's `AbstractModelStore` share the same verbs
PSI's `SimulationStore` (`HdfSimulationStore`, `InMemorySimulationStore`) and IOM's per-model `AbstractModelStore` (`DecisionModelStore`, `EmulationModelStore`) both use `write_output!` / `read_output` / `read_outputs` — PSI's are methods of IOM's generics, dispatched on `SimulationStore`, not a second set of functions (`abstract_simulation_store.jl` documents the required interface). IOM has no singular `read_output` generic, so PSI defines that one itself; it still dispatches through the same name. `simulation_store_common.jl` additionally extends IOM's bulk `write_outputs!` for `SimulationStore`, writing every field (`duals`, `parameters`, `variables`, `aux_variables`, `expressions`) for a model in one call.

### Realized-outputs export dispatches directly on `SimulationProblemOutputs`
The generic export entry point is `IOM.export_realized_outputs`, which calls `IOM.read_outputs_with_keys` — a method IOM only defines for its own `OptimizationProblemOutputs`. `decision_model_simulation_outputs.jl` and `emulation_model_simulation_outputs.jl` each add a `function IOM.read_outputs_with_keys(res::SimulationProblemOutputs{DecisionModelSimulationOutputs}/{EmulationModelSimulationOutputs}, ...)` method, so `SimulationProblemOutputs` satisfies the same export path without IOM code changes.

### Initial-condition values can legitimately be `Nothing`
A must-run device has no meaningful `InitialTimeDurationOn`, so per-model IC values are dispatched (`_ic_value_is_missing(::Nothing) = true` / `::Any = false`), never `isa`-checked. `_ic_values_reconciled` (`src/simulation/simulation.jl`) treats all-missing as consistent, all-present as a numeric comparison, and a **mix** of missing/present across models as a real mismatch to report. Do not "fix" this by filtering out `nothing` — that would hide genuine cross-model disagreement.

### Unexported IOM surface
Most of IOM's model-lifecycle accessors (`get_store`, `set_status!`, `get_output_dir`, `advance_execution_count!`, dataset functions, `ModelStoreParams`, `LOG_GROUP_SIMULATION_STORE`, `set_interval!`, …) are not exported. PSI imports them explicitly in `src/PowerSimulations.jl`. Add to that block; do not qualify at call sites and do not ask IOM to export them for PSI's sake.

### IOM refuses simulation-owned models
`IOM.OptimizationProblemOutputs(model)` errors with "Model Solved as part of a Simulation" when the model store is empty. That is by design: simulation outputs come from PSI's `SimulationStore`, never from the per-model store.

### `update_cost_parameters.jl` mirrors POM build-time code
Time-varying cost refresh must call the same IOM/POM objective-function functions the build uses (`market_bid_plumbing.jl`, `value_curve_cost.jl`). psy6 cost curves carry the unit marker as a type parameter and `IS.UnitSystem` is gone. Every `PSY.get_*` on a convertible field passes the unit system explicitly.

### Never modify IOM's `optimization_container.jl`
Same rule as before, one package down. Push fixes into POM's `construct_device!` / `add_parameters!` chain or PSI's update chain.

### No silent absence-sentinel skips
Do not add `isnothing(x) && continue` guards that hide malformed-data bugs; let the next call surface the data error. When triaging bot review comments, mark "add a nothing-skip" suggestions as invalid.

### Parameter multiplier `fill!` optimizations need a manual audit
Whether a parameter type has a uniform multiplier across all `(device, time)` cells is NOT statically derivable. Frame `fill!` proposals as "for uniform-multiplier types" and ask which qualify. Hot path in PSI: `parameters/update_container_parameter_values.jl`.

### Outputs System recovery (downstream coupling)
`populate_system=true` loads the model's bundle System, which carries the recast input series for
the executed windows (see "Outputs bundle"). A component can hold a static and a forecast input
series of the same name (emulation vs decision model sharing one bundle); read them by type.

## Follow-ups tracked outside this file
AGC (`AGCReserveDeployment` dropped; POM's `agc.jl` is not compiled); service feedforwards (POM errors on `attach_feedforward!(::ServiceModel, …)`); PowerAnalytics/PowerGraphics re-validation against the bundle-backed outputs; partition merge keys input rows by `(owner, name)` without series type when a decision model and the emulator share a bundle; `test/runtests.jl` drops `Aqua.find_persistent_tasks_deps`/`Aqua.test_persistent_tasks` (they resolve a throwaway env from the registry, which can't satisfy PowerSystems' unregistered OpenAPI deps on the psy6 line) — re-enable once those packages are registered, matching the same exclusion already in POM's and IOM's test suites.

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

## Branches
psy6-line work targets **`psy6`**: branch from it, diff/PR/review against `origin/psy6`. `main` is
the released psy5 line and has no IOM/POM dependency — never merge psy6 work into it. `origin` is
`Sienna-Platform/PowerSimulations.jl`. The global rule holds: never `git commit` unless told,
leave changes unstaged.
