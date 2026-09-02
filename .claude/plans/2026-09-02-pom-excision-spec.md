# PSI Excision — Spec and Inventory

Date: 2026-09-02. Repo: `PowerSimulations.jl` (PSI), branch `jd/pom_excision`, base `main` @ `aa474f9e4` (v0.38.3).
Companion plan: `2026-09-02-pom-excision.md`.

This document is the "spec" the plan argues from. It records the scope decision, the
per-file classification of PSI, the symbol rename map into IOM and POM, and the gaps found
in IOM and POM. Executing agents read this first.

## 1. Scope

PSI becomes the **simulation orchestration** package of the psy6 line. It keeps:

- `Simulation`, `SimulationModels`, `SimulationSequence`, `SimulationState`, `SimulationInternal`
- simulation stores (`HdfSimulationStore`, `InMemorySimulationStore`), output caches, store params
- parameter update between solves (`update_parameter_values!`, `update_container_parameter_values!`,
  `update_cost_parameters.jl`) — **these calls stay in PSI** (user decision 2026-09-02)
- initial-condition update between solves and the chronologies
- simulation results, realized results, export, partitions, recorder events
- the `solve!(step, model, start_time, store::SimulationStore)` entry points

PSI **drops** everything that builds a single optimization model. Those live in:

- **IOM** (`InfrastructureOptimizationModels`): `OptimizationContainer`, `DecisionModel{M}`,
  `EmulationModel{M}`, `ModelInternal`, `Settings`, `DecisionModelStore`, `EmulationModelStore`,
  `ModelStoreParams`, datasets (`InMemoryDataset`, `HDF5Dataset`, `DatasetContainer`),
  `OutputsByTime`, `OptimizationProblemOutputs`, `write_outputs!`, generic builders, objective
  functions, status enums.
- **POM** (`PowerOperationsModels`): all device/service/network/HVDC/storage/hydro/hybrid
  formulations, `PowerOperationsProblemTemplate`, the `AbstractPowerOperationProblem` chain,
  `build!`/`solve!`/`run!` per model, `build_problem!`, feedforward types and their
  argument/constraint construction, parameter types and `add_parameters!`, initial-condition
  allocation and the initialization sub-problem, PF-in-the-loop via `ext/PowerFlowsExt`.

**Events are commented out, not deleted.** POM has only the four `EventParameter` types and
no-op `add_event_arguments!`/`add_event_constraints!` stubs. The `EventModel` framework,
`contingency_model/`, `simulation_events.jl`, and every event-dispatched method in KEEP files
are wrapped in `#= EVENTS-EXCISION ... =#` blocks until POM grows the framework.

### Decisions (user, 2026-09-02)

| # | Decision |
|---|---|
| D1 | `using PowerSimulations` re-exports the full POM and IOM public API. |
| D2 | Parameter and initial-condition update calls remain in PSI. No IOM or POM source edits in this effort. PSI reaches unexported IOM symbols by explicit `import InfrastructureOptimizationModels: name` in the main module. |
| D3 | Agents commit per task on `jd/pom_excision`. No co-author or session trailers. |
| D4 | Port **all** simulation tests, including the hydro fixtures, using POM's native hydro formulations through PSI's own `test/test_utils/operations_problem_templates.jl`. |
| D5 | Events: leave files on disk; comment out their `include` lines; wrap event methods in KEEP files with `#= EVENTS-EXCISION` / `=#` block comments. |
| D6 | Drop PowerModels entirely. POM network models are native. |
| D7 | PSI keeps `GenericOpProblem`, `UnitCommitmentProblem`, `EconomicDispatchProblem` as concrete `POM.GenericPowerDecisionProblem` subtypes, and keeps `template_unit_commitment` / `template_economic_dispatch` rewritten to POM names. `AGCReserveDeployment` is dropped (AGC is not compiled in POM; tracked follow-up). |
| D8 | No `ProblemTemplate` alias. PSI code and tests use `PowerOperationsProblemTemplate`. |
| D9 | Co-dev wiring uses `[sources]` **path pins** to sibling checkouts in `psy6/`, marked temporary, mirroring POM's own temporary pins. |
| D10 | Docs: prune to the KEEP pages so `docs/make.jl` builds. Formulation library pages are deleted. |

## 2. Global constraints (from the psy6 line)

- No version or compat bumps in any `Project.toml` beyond what resolution requires. PSI version stays `0.38.3`.
- No shims, no compat aliases, no deprecation layers.
- Explicit units: every `PSY.get_*` on a convertible field passes `PSY.SU` in optimization code.
- Never edit IOM `src/core/optimization_container.jl`.
- No `isa`/`<:` runtime branches; no ternaries; `iszero`; explicit `function ... end` + `return`.
- No `isnothing(x) && continue` guards.
- Formatter after every task: `julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'`.
- Julia 1.12.5. POM requires `julia = "^1.11"`; PSI compat moves to `^1.11`.

## 3. Dependency wiring

Sibling checkouts (all in `/home/jdlara/Sienna_work/psy6/`):

| package | dir | branch |
|---|---|---|
| InfrastructureSystems | `InfrastructureSystems.jl` | `IS4` |
| InfrastructureOptimizationModels | `InfrastructureOptimizationModels.jl` | `main` |
| PowerSystems | `PowerSystems.jl` | `psy6` |
| PowerNetworkMatrices | `PowerNetworkMatrices.jl` | `psy6` |
| PowerOperationsModels | `PowerOperationsModels.jl` | `main` |
| PowerSystemCaseBuilder | `PowerSystemCaseBuilder.jl` | `psy6` |
| PowerFlows | `PowerFlows.jl` | `psy6` (test only, for PF-in-the-loop simulations) |

PSY psy6 depends on seven unregistered OpenAPI packages. `[sources]` of non-root projects are
ignored by Pkg, so PSI's root and test projects must carry those pins and list the packages in
`[deps]`, exactly as POM does.

Target `Project.toml` `[deps]` after excision: CSV, DataFrames, DataFramesMeta, DataStructures,
Dates, Distributed, DocStringExtensions, HDF5, InfrastructureOptimizationModels,
InfrastructureSystems, JSON3, JuMP, Logging, PowerNetworkMatrices, PowerOperationsModels,
PowerSystems, PrettyTables, ProgressMeter, Random, Serialization, TimeSeries, TimerOutputs, plus
the seven OpenAPI packages (pin-only). **Dropped:** Distributions (events only), InteractiveUtils,
LinearAlgebra, MathOptInterface, PowerFlows, PowerModels, SparseArrays.

## 4. File classification (`src/`)

Classes: KEEP (stays, imports rewritten) · REMOVE (`git rm`; counterpart exists) · SPLIT (part
stays) · EVENTS (comment out) · GLUE (rewritten adapter).

### 4a. core/

| file | class | note / counterpart |
|---|---|---|
| `core/definitions.jl` | SPLIT | keep `SIMULATION_LOG_FILENAME`, `REQUIRED_RECORDERS`, `RESULTS_DIR`, `KNOWN_SIMULATION_PATHS`, `NO_SERVICE_NAME_PROVIDED`, `RUN_SIMULATION_TIMER`, `PROBLEM_LOG_FILENAME`; everything else is in IOM `core/definitions.jl` |
| `core/formulations.jl` | REMOVE | POM `core/formulations.jl` |
| `core/network_formulations.jl` | REMOVE | POM `core/network_formulations.jl` |
| `core/abstract_simulation_store.jl` | KEEP | `SimulationStore` is PSI's; IOM's `AbstractModelStore` is per-model |
| `core/operation_model_abstract_types.jl` | REMOVE | IOM |
| `core/abstract_feedforward.jl` | REMOVE | IOM `core/definitions.jl` |
| `core/variables.jl` | REMOVE | POM |
| `core/network_reductions.jl` | REMOVE | POM `network_models/network_reductions.jl` |
| `core/parameters.jl` | REMOVE | IOM `core/parameter_container.jl` + POM `core/parameters.jl` |
| `core/service_model.jl` | REMOVE | IOM |
| `core/event_keys.jl` | EVENTS | include commented |
| `core/event_model.jl` | EVENTS | include commented |
| `core/device_model.jl` | REMOVE | IOM |
| `core/network_model.jl` | REMOVE | IOM + POM `network_models/instantiate_network_model.jl` |
| `core/auxiliary_variables.jl` | REMOVE | POM |
| `core/constraints.jl` | REMOVE | POM |
| `core/expressions.jl` | REMOVE | POM |
| `core/initial_conditions.jl` | REMOVE | IOM + POM |
| `core/settings.jl` | REMOVE | IOM |
| `core/cache_utils.jl` | KEEP | HDF store cache policy |
| `core/dataset.jl` | REMOVE | IOM (verbatim) |
| `core/dataset_container.jl` | REMOVE | IOM; PSI has ~5 extra `get_dataset` overloads (lines 96-134, 140-186) — re-add in PSI only if the shadow load shows a caller |
| `core/results_by_time.jl` | REMOVE | IOM `core/outputs_by_time.jl` (renamed) |
| `core/power_flow_data_wrapper.jl` | REMOVE | POM ext |
| `core/optimization_container.jl` | REMOVE | IOM |
| `core/dual_processing.jl` | REMOVE | IOM |
| `core/store_common.jl` | REMOVE | IOM `operation/store_common.jl` (`write_outputs!`) |
| `core/model_store_params.jl` | REMOVE | IOM (verbatim) |

### 4b. operation/, initial_conditions/, feedforward/, contingency_model/, parameters/

| file | class | note |
|---|---|---|
| `operation/problem_template.jl` | REMOVE | POM `core/problem_template.jl` |
| `operation/operation_model_interface.jl` | REMOVE | IOM `operation/optimization_model_interface.jl` |
| `operation/decision_model_store.jl` | REMOVE | IOM |
| `operation/emulation_model_store.jl` | REMOVE | IOM |
| `operation/initial_conditions_update_in_memory_store.jl` | REMOVE | IOM + POM |
| `operation/operation_model_types.jl` | SPLIT | keep `GenericOpProblem`, `UnitCommitmentProblem`, `EconomicDispatchProblem` re-parented to `POM.GenericPowerDecisionProblem` (D7) |
| `operation/template_validation.jl` | REMOVE | POM |
| `operation/decision_model.jl` | SPLIT | keep only `solve!(step, model, start_time, store::SimulationStore; exports)` (line 549) |
| `operation/emulation_model.jl` | SPLIT | keep `update_parameters!(model, ::EmulationModelStore)` (382), `update_parameters!(model, ::DatasetContainer)` (387), `update_initial_conditions!` (400), `update_model!` (411, 449), `update_parameter_values!` (428), `solve!(step, ..., store::SimulationStore)` (585) |
| `operation/problem_results.jl` | REMOVE | IOM `operation/problem_outputs.jl` |
| `operation/time_series_interface.jl` | REMOVE | IOM |
| `operation/optimization_debugging.jl` | REMOVE | IOM |
| `operation/model_numerical_analysis_utils.jl` | REMOVE | IOM |
| `operation/operation_model_simulation_interface.jl` | GLUE | 3 adapters; rewrite to IOM calls |
| `operation/operation_problem_templates.jl` | SPLIT | keep `template_unit_commitment`, `template_economic_dispatch` with POM names (D7); drop `template_agc_reserve_deployment` |
| `initial_conditions/initial_condition_chronologies.jl` | KEEP | |
| `initial_conditions/add_initial_condition.jl` | REMOVE | POM |
| `initial_conditions/update_initial_conditions.jl` | KEEP | between-solve dispatch + `InitialConditionUpdateEvent` |
| `initial_conditions/calculate_initial_condition.jl` | REMOVE | IOM |
| `initial_conditions/initialization.jl` | REMOVE | POM |
| `feedforward/*.jl` (3 files) | REMOVE | POM |
| `contingency_model/*.jl` (3 files) | EVENTS | includes commented |
| `parameters/add_parameters.jl` | REMOVE | POM `common_models/add_parameters.jl`. Its `lookup_additional_axes` and `_unwrap_for_param` are used by KEEP update files: copy them into PSI `parameters/update_container_parameter_values.jl` if POM does not export equivalents |
| `parameters/update_container_parameter_values.jl` | KEEP + EVENTS | implements IOM's bare `update_container_parameter_values!`; event methods at 578, 635, 696, 837 and the outage branch at 501-531 commented |
| `parameters/update_cost_parameters.jl` | KEEP | stays in PSI (D2). Highest PSY-psy6 risk file: cost API changed (`IS.UnitSystem` gone, `variable_operation_cost` rename). Mirror POM `common_models/market_bid_plumbing.jl` and IOM `objective_function/value_curve_cost.jl` |
| `parameters/update_parameters.jl` | KEEP | driver + `_fix_parameter_value!` |

### 4c. simulation/

| file | class | note |
|---|---|---|
| `simulation/simulation_store_requirements.jl` | GLUE | key types from IOM |
| `simulation/simulation_info.jl` | REMOVE | `IS.Simulation.SimulationInfo` |
| `simulation/optimization_output_cache.jl` | KEEP | |
| `simulation/optimization_output_caches.jl` | KEEP | |
| `simulation/simulation_models.jl` | KEEP | |
| `simulation/simulation_state.jl` | KEEP + EVENTS | comment: `update_decision_state!` event variants (236, 275, 322, 443, 455, 498), `_get_time_to_recover` (403, 423), `_get_outage_occurrence` (842, 865), `update_system_state!` event variants (803, 815, 885, 917, 957, 983, 1009) |
| `simulation/initial_condition_update_simulation.jl` | KEEP | |
| `simulation/simulation_store_params.jl` | KEEP | |
| `simulation/hdf_simulation_store.jl` | KEEP | |
| `simulation/in_memory_simulation_store.jl` | KEEP | |
| `simulation/simulation_problem_results.jl` | KEEP | |
| `simulation/get_components_interface.jl` | GLUE | PSY forwarding for `IS.Results`; keep, verify against PSY psy6 |
| `simulation/decision_model_simulation_results.jl` | KEEP | `ResultsByKeyAndTime` → `IOM.OutputsByKeyAndTime` |
| `simulation/emulation_model_simulation_results.jl` | KEEP | |
| `simulation/realized_meta.jl` | KEEP | |
| `simulation/simulation_partitions.jl` | KEEP | |
| `simulation/simulation_partition_results.jl` | KEEP | |
| `simulation/simulation_sequence.jl` | KEEP + EVENTS | comment: `_add_event_to_model` (201), `_validate_event_timeseries_data` (215), `_add_model_to_event_map!` (251), `_attach_events!` (297), `events` field + ctor kwarg (367, 377), call at 398, `get_events` (423) |
| `simulation/simulation_internal.jl` | KEEP | keep `rng` (Random stays a dep) |
| `simulation/simulation.jl` | KEEP + EVENTS | comment: `_is_event_countdown_parameter_key` (863, 869), countdown-first ordering in `_update_simulation_state_parameters!` (841-861; replace with a plain loop), `apply_simulation_events!` call (1060) |
| `simulation/simulation_events.jl` | EVENTS | include commented |
| `simulation/simulation_results_export.jl` | KEEP | |
| `simulation/simulation_results.jl` | KEEP | |

### 4d. devices_models/, services_models/, network_models/ — all REMOVE

Every file under `src/devices_models/`, `src/services_models/`, `src/network_models/` is
REMOVE. `network_models/power_flow_evaluation.jl` (1498 lines) is fully covered by POM
`ext/PowerFlowsExt/`.

### 4e. utils/

| file | class | note |
|---|---|---|
| `utils/indexing.jl` | REMOVE | IOM (verbatim) |
| `utils/print_pt_v2.jl` | REMOVE | drop the `@static if pkgversion(PrettyTables).major == 2` branch |
| `utils/print_pt_v3.jl` | SPLIT | keep lines 270-499 (`SimulationModels`, `SimulationSequence`, `Simulation`, `SimulationResults` show methods) |
| `utils/file_utils.jl` | SPLIT | keep `read_dataframe` (13) if tests need it; else REMOVE |
| `utils/logging.jl` | REMOVE | IOM; `LOG_GROUP_RESULTS` → `IOM.LOG_GROUP_OUTPUTS` |
| `utils/dataframes_utils.jl` | REMOVE | IOM |
| `utils/jump_utils.jl` | SPLIT | keep `_calc_dimensions` (516, 550) → new `utils/store_dimensions.jl`; `to_results_dataframe` → `IOM.to_outputs_dataframe` |
| `utils/powersystems_utils.jl` | SPLIT | keep `make_system_filename` (204) → `utils/file_utils.jl`; `get_single_time_series_consistency` (517) only if a KEEP caller survives |
| `utils/time_series_utils.jl` | REMOVE | IOM |
| `utils/recorder_events.jl` | KEEP | `ParameterUpdateEvent` `EventParametersAttributes` ctor (153) → EVENTS block |
| `utils/datetime_utils.jl` | REMOVE | IOM |
| `utils/generate_valid_formulations.jl` | REMOVE | POM |
| `PowerSimulations.jl` | SPLIT | rewritten in Phase 1 |

## 5. Symbol rename map (PSI → psy6)

| PSI name | replacement |
|---|---|
| `OperationModel` | `IOM.AbstractOptimizationModel` |
| `DecisionProblem` | `POM.AbstractPowerDecisionProblem` |
| `EmulationProblem` | `POM.AbstractPowerEmulationProblem` |
| `DefaultDecisionProblem` / `DefaultEmulationProblem` | `POM.DefaultPowerDecisionProblem` / `POM.DefaultPowerEmulationProblem` |
| `ProblemTemplate` | `POM.PowerOperationsProblemTemplate` |
| `CopperPlatePowerModel`, `PTDFPowerModel`, `AreaBalancePowerModel`, `AreaPTDFPowerModel` | `CopperPlateNetworkModel`, `PTDFNetworkModel`, `AreaBalanceNetworkModel`, `AreaPTDFNetworkModel` |
| `PM.ACPPowerModel`, `PM.DCPPowerModel`, `PM.NFAPowerModel`, `PM.DCPLLPowerModel`, `PM.ACRPowerModel`, `PM.LPACCPowerModel`, `PM.IVRPowerModel` | `ACPNetworkModel`, `DCPNetworkModel`, `NFANetworkModel`, `DCPLLNetworkModel`, `ACRNetworkModel`, `LPACCNetworkModel`, `IVRNetworkModel` |
| `PM.AbstractPowerModel` | `IOM.AbstractNetworkModel` |
| `OptimizationProblemResults` | `IOM.OptimizationProblemOutputs` |
| `OptimizationProblemResultsExport` | `IOM.OptimizationProblemOutputsExport` |
| `ResultsByTime` / `ResultsByKeyAndTime` | `IOM.OutputsByTime` / `IOM.OutputsByKeyAndTime` |
| `write_results!(store::SimulationStore, model, index, ts; exports)` | **stays PSI-defined** (moves from `core/store_common.jl` to `simulation/simulation_store_common.jl`). IOM's `write_outputs!` dispatches on `AbstractModelStore`, which PSI's `SimulationStore` is not |
| `read_results` / `write_result!` on IOM model stores (`DecisionModelStore`, `EmulationModelStore`) | `IOM.read_outputs` / `IOM.write_output!` — PSI's `SimulationStore` keeps its own `write_result!`/`read_result` names (no clash) |
| `_read_results` | `IOM._read_outputs` |
| `to_results_dataframe` | `IOM.to_outputs_dataframe` |
| `solve_impl!(model)` | `IOM.solve_model!(model)` |
| `solve_impl!(container, sys)` | `IOM.execute_optimizer!(container, sys)` |
| `build_impl!(container, template, sys)` | `POM.build_problem!(container, template, sys)` |
| `build_impl!(model)` | `POM.build_model!(model)` |
| `run_impl!(model)` | `POM.execute_emulation!(model)` (not used by PSI's simulation path) |
| `initialize!(model)` | `POM.solve_and_write_initial_conditions!(model)` |
| `SimulationInfo` | `IS.Simulation.SimulationInfo` (IOM imports it) |
| `LOG_GROUP_RESULTS` | `IOM.LOG_GROUP_OUTPUTS` |
| `RunStatus.SUCCESSFULLY_FINALIZED`, `ModelBuildStatus`, `SimulationBuildStatus` | unchanged (IOM exports) |
| `get_time_series_values!` | `IOM.get_time_series_values!` |
| `IS.Optimization.*` imports in main module | keep the ones PSI still uses; many now come through IOM |

## 6. Unexported IOM symbols PSI needs (explicit `import` list, Phase 1)

Model accessors: `get_store`, `get_status`, `set_status!`, `get_output_dir`, `set_output_dir!`,
`get_simulation_info`, `set_simulation_info!`, `get_run_status`, `set_run_status!`,
`is_synchronized`, `set_synchronized_status!`, `get_store_params`, `set_store_params!`,
`advance_execution_count!`, `get_execution_count`, `get_executions`, `set_executions!`,
`set_execution_count!`, `get_initial_time`, `is_built`, `warm_start_enabled`,
`_pre_solve_model_checks`, `solve_model!`, `get_time_series_cache`, `empty_time_series_cache!`,
`get_log_file`, `get_recorder_dir`, `get_initial_conditions_file`, `add_recorders!`,
`register_recorders!`, `unregister_recorders!`, `configure_logging`, `get_current_timestamp`,
`get_simulation_number`, `set_simulation_number!`, `get_sequence_uuid`, `set_sequence_uuid!`,
`cost_function_unsynch`, `update_objective_function!`, `reset_optimization_model!`.

Store/params: `ModelStoreParams`, `get_num_executions`, `get_horizon_count`, `get_base_power`,
`get_system_uuid`, `DecisionModelStore`, `initialize_storage!`, `write_output!`, `read_outputs`,
`write_optimizer_stats!`, `DecisionModelIndexType`, `EmulationModelIndexType`,
`STORE_CONTAINERS`, `STORE_CONTAINER_*`, `get_data_field`, `list_fields`, `list_keys`.

Datasets: `AbstractDataset`, `InMemoryDataset`, `HDF5Dataset`, `DatasetContainer`,
`make_system_state`, `get_dataset`, `set_dataset!`, `has_dataset`, `get_dataset_keys`,
`get_dataset_values`, `set_dataset_values!`, `get_dataset_value`, `get_last_recorded_row`,
`set_last_recorded_row!`, `get_update_timestamp`, `set_update_timestamp!`,
`get_last_updated_timestamp`, `get_last_recorded_value`, `get_last_update_value`, `get_num_rows`,
`get_data_resolution`, `get_end_of_step_timestamp`, `get_value_timestamp`, `set_value!`,
`get_dataset_size`, `get_column_names`, `OutputsByTime`, `OutputsByKeyAndTime`, `make_dataframes`.

Parameters: `get_parameter_attributes`, `get_parameter_array`, `get_parameter_multiplier_array`,
`get_attribute_key`, `get_time_series_name`, `_get_ts_uuid`, `_set_param_value_at!`,
`set_parameter!`, `ValidDataParamEltypes`, `TimeSeriesAttributes`, `VariableValueAttributes`,
`CostFunctionAttributes`, `EventParametersAttributes`, `NoAttributes`.

Other: `LOG_GROUP_SIMULATION_STORE`, `LOG_GROUP_OUTPUTS`, `LOG_GROUP_MODEL_STORE`,
`LOG_GROUP_BUILD_INITIAL_CONDITIONS`, `set_interval!`, `get_variables`, `get_parameters`,
`get_duals`, `get_expressions`, `get_aux_variables`, `get_time_steps`, `find_timestamp_index`,
`to_matrix`, `get_column_names_from_axis_array`, `should_write_resulting_value`,
`convert_output_to_natural_units`, `deserialize_key`, `get_initial_conditions_data`,
`_deepcopy_template`, `get_deterministic_time_series_type`, `RUN_OPERATION_MODEL_TIMER`.

The list is a starting point. The shadow-load checker (Phase 0) produces the authoritative list.

## 7. Gaps in IOM and POM that PSI must fill locally (no upstream edits, D2)

| gap | PSI response |
|---|---|
| `_calc_dimensions` absent from IOM | keep PSI copy in `utils/store_dimensions.jl` |
| `update_container_parameter_values!` is a bare `function ... end` in IOM | PSI defines all methods (already does) |
| `update_model!` has no POM method and no call in `execute_emulation!` | PSI's simulation loop calls its own `update_model!(model, sim)`; standalone POM emulation stays non-updating |
| `InterProblemChronology` / `IntraProblemChronology` absent | PSI keeps `initial_condition_chronologies.jl` |
| `IOM.OptimizationProblemOutputs(model)` errors on an empty model store ("Model Solved as part of a Simulation") | expected; PSI builds `SimulationProblemResults` from its own store |
| `IOM.HDF5Dataset` lacks `get_value_timestamp`/`get_dataset_value` | PSI's HDF store never uses `HDF5Dataset` for state; system state is `InMemoryDataset`. Verify no KEEP caller |
| `get_run_status`/`set_run_status!` in IOM have no `isnothing` guard on `simulation_info` | PSI always sets `SimulationInfo` in `initialize_simulation_internals!` before use |
| PSI dataset_container extra overloads | re-add locally only if the checker reports a caller |
| `template_unit_commitment`/`template_economic_dispatch` absent | PSI keeps them (D7) |
| POM `feedforward/feedforwards.jl` feedforward types are keyword-only ctors with `component_type`, `source`, `affected_values` | `simulation_sequence.jl` uses only `get_optimization_container_key`, `get_affected_values`, `get_feedforward_meta`, `attach_feedforward!` — all present in POM |
| `attach_feedforward!(::ServiceModel, ff)` errors in POM | `_add_feedforward_to_model(::PSY.Service)` path in `simulation_sequence.jl:150` will error at build for service feedforwards; leave as a loud error, list as follow-up |
| `get_time_series_values!` cache signature | verify against IOM `operation/time_series_interface.jl:1,45` when rewriting `update_container_parameter_values.jl` |
| `PSY` psy6 units and renames in KEEP code | sweep every `PSY.` call in KEEP files; `update_cost_parameters.jl` is the hot spot |

## 8. Events follow-up (out of scope, recorded for the next effort)

Move to POM: `core/event_keys.jl`, `core/event_model.jl` (condition types, `EventModel`),
`contingency_model/*` (event parameter containers, outage constraints, `::EventModel`-dispatched
multipliers). Then in PSI uncomment: `simulation_events.jl`, the EVENTS blocks in
`simulation_state.jl`, `simulation_sequence.jl`, `simulation.jl`,
`update_container_parameter_values.jl`, `recorder_events.jl`; re-enable `test_events.jl`,
`test_postcontingency_mixed_outage_axes.jl`, `test_utils/events_simulation_utils.jl`; restore
`Distributions` dep. Every block carries the marker `EVENTS-EXCISION` so `grep -rn EVENTS-EXCISION`
lists the work.

## 9. Test classification (`test/`)

KEEP: `test_simulation_build.jl`, `test_simulation_execute.jl`, `test_simulation_models.jl`,
`test_simulation_sequence.jl`, `test_simulation_store.jl`, `test_simulation_results.jl`,
`test_simulation_results_export.jl`, `test_simulation_partitions.jl`,
`run_partitioned_simulation.jl`, `test_recorder_events.jl`.

SPLIT (keep the simulation testsets only): `test_print.jl` (Simulation print methods),
`test_utils.jl` (output directory name), `test_market_bid_cost.jl` (testsets that run a
`Simulation` with time-varying MarketBidCost exercise `update_cost_parameters.jl`),
`test_import_export_cost.jl` (testsets using `run_iec_sim`).

EVENTS (disable via `DISABLED_TEST_FILES`, comment `include` of utils): `test_events.jl`,
`test_postcontingency_mixed_outage_axes.jl`, `test_utils/events_simulation_utils.jl`.

REMOVE: every other `test_*.jl` (single-model formulation tests now in POM), `test/performance/`.

`test_utils/` KEEP and rewrite: `common_operation_model.jl`, `mock_operation_models.jl`
(re-parent to `POM.GenericPowerDecisionProblem` / `GenericPowerEmulationProblem`),
`solver_definitions.jl`, `operations_problem_templates.jl` (POM names, hydro via POM
formulations, D4), `run_simulation.jl`, `model_checks.jl` (trim to helpers simulation tests call),
`mbc_simulation_utils.jl`, `mbc_system_utils.jl`, `add_market_bid_cost.jl`,
`iec_simulation_utils.jl`, `add_components_to_system.jl` (only if a KEEP test calls it).
REMOVE: `add_branch_rating_time_series.jl`.

`test/Project.toml`: drop `HydroPowerSimulations`, `StorageSystemsSimulations`, `PowerModels`,
`Ipopt`, `SCS`, `LinearAlgebra`; add `InfrastructureOptimizationModels`, `PowerOperationsModels`,
`PowerFlows` (PF-in-the-loop simulation tests), the OpenAPI pins; `[sources]` path pins to
siblings and `PowerSimulations = {path = ".."}`.

## 10. Docs classification

KEEP pages: `index.md` (rewrite scope), `tutorials/pcm_simulation.jl`, `how_to/read_results.md`,
`how_to/logging.md`, `how_to/simulation_recorder.md`, `how_to/parallel_simulations.md`,
`explanation/psi_structure.md` (rewrite for the split), `explanation/feedforward.md` (sequence
wiring only), `explanation/chronologies.md`, `explanation/sequencing.md`, `api/*`.

REMOVE: `tutorials/decision_problem.jl`, `dynamic_line_ratings.jl`, `uc_power_flow_in_the_loop.jl`,
`security_constrained_reserves.jl`, `how_to/register_variable.md`, `problem_templates.md`,
`debugging_infeasible_models.md`, `security_constrained_models.md`, `adding_new_problem_model.md`,
`explanation/branch_rating_limits.md`, all of `formulation_library/`,
`code_base_developer_guide/extending_powersimulations.md`.
