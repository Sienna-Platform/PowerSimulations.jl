# PSI Excision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut PowerSimulations.jl down to simulation orchestration and wire it onto InfrastructureOptimizationModels (IOM) and PowerOperationsModels (POM) so a multi-model simulation builds, runs, and produces results again on the psy6 line.

**Architecture:** Big-bang removal of all single-model building code (it cannot compile against psy6 PowerSystems anyway), followed by a load gate, a parallel per-file body rewrite verified by a static symbol checker, an integration compile pass, then tests, docs, and guides. Events are fenced with `EVENTS-EXCISION` block comments, not deleted.

**Tech Stack:** Julia 1.12.5, IOM `main`, POM `main`, PSY `psy6`, IS `IS4`, PNM `psy6`, PSB `psy6`, HDF5, JuMP, HiGHS (tests).

**Spec:** `.claude/plans/2026-09-02-pom-excision-spec.md` — read sections 1, 2, 5, 6, 7 before any task; section 4 for the file classification; section 9 for tests.

## Global Constraints

- Work only on branch `jd/pom_excision` in `/home/jdlara/Sienna_work/psy6/PowerSimulations.jl`.
- Never edit files in the IOM, POM, PSY, IS, PNM, or PSB checkouts (spec D2).
- No version or compat bumps beyond what resolution requires (spec §2). PSI stays `0.38.3`.
- No shims or aliases for renamed types (spec D8). No `isa`/`<:` branches, no ternaries, explicit `function … end` with `return`, no `isnothing(x) && continue`.
- Every event-related removal is a `#= EVENTS-EXCISION: <what> =#` … `=#` fence or a commented `include` carrying the same marker. Nothing event-related is deleted.
- Commit per task on `jd/pom_excision` (spec D3). Commit only the files the task touched: `git add <paths>` then `git commit -m "excision(<phase>): <summary>"`. No trailers of any kind. If `.git/index.lock` exists, wait two seconds and retry.
- Formatter before every commit that touches `.jl`: `julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'`.
- All Julia invocations use `--project=<env>`; the package env is `--project=.`, tests `--project=test`.
- Phase 2 and Phase 4 tasks run in parallel; every other phase is serial. Do not start a phase before the previous phase's gate is green.

---

## Phase 0 — Foundations (serial)

### Task 0.1: Dependency wiring

**Files:**
- Modify: `Project.toml`
- Create: `.claude/plans/excision-progress.md`

**Interfaces:**
- Produces: a PSI package env in which `using PowerOperationsModels` and `using InfrastructureOptimizationModels` succeed.

- [ ] **Step 1: Rewrite `Project.toml`**

Replace the `[deps]`, `[compat]` sections and add `[sources]`. Keep `name`, `uuid`, `authors`, `version = "0.38.3"`.

```toml
[deps]
CSV = "336ed68f-0bac-5ca0-87d4-7b16caf5d00b"
DataFrames = "a93c6f00-e57d-5684-b7b6-d8193f3e46c0"
DataFramesMeta = "1313f7d8-7da2-5740-9ea0-a2ca25f37964"
DataStructures = "864edb3b-99cc-5e75-8d2d-829cb0a9cfe8"
Dates = "ade2ca70-3891-5945-98fb-dc099432e06a"
Distributed = "8ba89e20-285c-5b6f-9357-94700520ee1b"
DocStringExtensions = "ffbed154-4ef7-542d-bbb7-c09d3a79fcae"
HDF5 = "f67ccb44-e63f-5c2f-98bd-6dc0ccc4ba2f"
InfrastructureOptimizationModels = "bed98974-b02a-5e2f-9ee0-a103f5c45069"
InfrastructureSystems = "2cd47ed4-ca9b-11e9-27f2-ab636a7671f1"
JSON3 = "0f8b85d8-7281-11e9-16c2-39a750bddbf1"
JuMP = "4076af6c-e467-56ae-b986-b466b2749572"
Logging = "56ddb016-857b-54e1-b83d-db4d58db5568"
PowerNetworkMatrices = "bed98974-b02a-5e2f-9fe0-a103f5c450dd"
PowerOperationsModels = "c9281074-0ec2-4761-850d-1a24107ea5e8"
PowerSystems = "bcd98974-b02a-5e2f-9ee0-a103f5c450dd"
PrettyTables = "08abe8d2-0d0c-5749-adfa-8a2ac140af0d"
ProgressMeter = "92933f4c-e287-5a05-a399-4b506db050ca"
Random = "9a3f8284-a2c9-5f02-9a11-845980a1fd5c"
Serialization = "9e88b42a-f829-5b0c-bbe9-9e923198166b"
TimeSeries = "9e3dc215-6440-5c97-bce1-76c03772f85e"
TimerOutputs = "a759f4b9-e2f1-59dc-863e-4aeb61b1ea8f"
# PSY psy6 depends on these unregistered packages; [sources] of non-root projects are ignored
# by Pkg, so this project must pin them itself. PSI imports none of them.
PowerCoreOpenAPIModels = "b7b40286-e793-417d-a9a0-b1583e4da1cb"
InfrastructureTimeSeriesOpenAPIModels = "37a216c8-a490-47cf-89d7-db2cb9618199"
InfrastructureCoreOpenAPIModels = "1f5e1c8d-e0cc-4dbf-8c6d-f2a3d2ae70a8"
PowerDynamicsOpenAPIModels = "044a0b22-31f8-4ef6-8282-c9a61b3013f6"
PowerInvestmentsOpenAPIModels = "33cb4396-f4d9-4f59-8585-787ebb56cb1b"
PowerOpenAPIModels = "0730f07c-cff6-4c3b-a9df-c546153be50a"
PowerOperationsOpenAPIModels = "a372b6d7-45a2-44c2-8199-6a724b72e8ff"

[sources]
# LOCAL PATH CO-DEV PINS (temporary): PSI is being rewired onto IOM and POM against the sibling
# psy6 checkouts. Switch to git rev pins (IS -> IS4, PSY/PNM -> psy6, IOM/POM -> main) once the
# excision lands.
InfrastructureSystems = {path = "../InfrastructureSystems.jl"}
InfrastructureOptimizationModels = {path = "../InfrastructureOptimizationModels.jl"}
PowerSystems = {path = "../PowerSystems.jl"}
PowerNetworkMatrices = {path = "../PowerNetworkMatrices.jl"}
PowerOperationsModels = {path = "../PowerOperationsModels.jl"}
PowerCoreOpenAPIModels = {url = "https://github.com/Sienna-Platform/PowerOpenAPIModels.git", rev = "main", subdir = "PowerCoreOpenAPIModels.jl"}
InfrastructureTimeSeriesOpenAPIModels = {url = "https://github.com/Sienna-Platform/PowerOpenAPIModels.git", rev = "main", subdir = "InfrastructureTimeSeriesOpenAPIModels.jl"}
InfrastructureCoreOpenAPIModels = {url = "https://github.com/Sienna-Platform/PowerOpenAPIModels.git", rev = "main", subdir = "InfrastructureCoreOpenAPIModels.jl"}
PowerDynamicsOpenAPIModels = {url = "https://github.com/Sienna-Platform/PowerOpenAPIModels.git", rev = "main", subdir = "PowerDynamicsOpenAPIModels.jl"}
PowerInvestmentsOpenAPIModels = {url = "https://github.com/Sienna-Platform/PowerOpenAPIModels.git", rev = "main", subdir = "PowerInvestmentsOpenAPIModels.jl"}
PowerOpenAPIModels = {url = "https://github.com/Sienna-Platform/PowerOpenAPIModels.git", rev = "main", subdir = "PowerOpenAPIModels.jl"}
PowerOperationsOpenAPIModels = {url = "https://github.com/Sienna-Platform/PowerOpenAPIModels.git", rev = "main", subdir = "PowerOperationsOpenAPIModels.jl"}

[compat]
CSV = "~0.10"
DataFrames = "1"
DataFramesMeta = "~0.15"
DataStructures = "~0.18, ~0.19"
Dates = "1"
Distributed = "1"
DocStringExtensions = "~v0.9"
HDF5 = "~0.17"
InfrastructureOptimizationModels = "0.1"
InfrastructureSystems = "3"
JSON3 = "1"
JuMP = "^1.28"
Logging = "1"
PowerNetworkMatrices = "^0.24"
PowerOperationsModels = "0.1"
PowerSystems = "^5.10"
PrettyTables = "3"
ProgressMeter = "^1.5"
Random = "^1.10"
Serialization = "1"
TimeSeries = "~0.25"
TimerOutputs = "~0.5, 1"
julia = "^1.11"
```

If any OpenAPI package pin in POM's `Project.toml` differs from the above (rev or subdir), copy POM's. If PSY's `Project.toml` on disk pins additional packages (for example `InfraStore`), copy those too.

- [ ] **Step 2: Resolve and verify the deps load**

Run:
```sh
cd /home/jdlara/Sienna_work/psy6/PowerSimulations.jl
rm -f Manifest.toml
julia --project=. -e 'using Pkg; Pkg.resolve(); Pkg.instantiate(); using InfrastructureOptimizationModels, PowerOperationsModels; println("deps OK")'
```
Expected: `deps OK`. Do not `using PowerSimulations` yet; it will not compile until Phase 1.

If resolution fails on an IS or OpenAPI mismatch, compare against POM's working `Project.toml` and `test/Project.toml` and copy its pins verbatim.

- [ ] **Step 3: Create the progress ledger**

Write `.claude/plans/excision-progress.md`:
```markdown
# Excision progress
| task | status | commit | notes |
|---|---|---|---|
| 0.1 | done | <sha> | deps resolve |
```
Every later task appends one row.

- [ ] **Step 4: Commit**

```sh
git add Project.toml .claude/plans/excision-progress.md .claude/plans/2026-09-02-pom-excision.md .claude/plans/2026-09-02-pom-excision-spec.md
git commit -m "excision(0.1): rewire Project.toml onto IOM and POM"
```

### Task 0.2: Static symbol checker

**Files:**
- Create: `scripts/excision/check_undefined.jl`
- Create: `scripts/excision/README.md`

**Interfaces:**
- Produces: `julia --project=. scripts/excision/check_undefined.jl <file>...` prints, per file, every identifier that is not bound in `Main.PowerSimulations`, `Base`, or `Core` and is not bound locally in the file; and every PSI-defined function whose name is also exported by IOM or POM but is not the same object. Exit code 1 when anything is reported.

Why: Julia only errors on undefined globals when a method runs. `using PowerSimulations` succeeding proves definitions load, not that bodies reference existing names. Phase 2 agents need a per-file check that does not require running a simulation.

- [ ] **Step 1: Write the checker**

```julia
# scripts/excision/check_undefined.jl
# Usage: julia --project=. scripts/excision/check_undefined.jl src/simulation/simulation.jl [more...]
# Loads src/PowerSimulations.jl into Main (definitions only), then walks each file's AST and
# reports identifiers not resolvable in the module. Heuristic: locally bound names are excluded.

const ROOT = normpath(joinpath(@__DIR__, "..", ".."))
Base.include(Main, joinpath(ROOT, "src", "PowerSimulations.jl"))
const MOD = Main.PowerSimulations
import InfrastructureOptimizationModels as IOM
import PowerOperationsModels as POM

const SKIP_HEADS = Set([:quote, :macrocall, :meta, :line, :inert])

function bindings!(acc::Set{Symbol}, ex)
    ex isa Symbol && (push!(acc, ex); return)
    ex isa Expr || return
    if ex.head == :(::)
        bindings!(acc, ex.args[1])
    elseif ex.head in (:tuple, :parameters, :kw, :(=), :curly, :(<:), :(...))
        ex.head == :kw && return bindings!(acc, ex.args[1])
        ex.head == :(=) && return bindings!(acc, ex.args[1])
        ex.head == :curly && return bindings!(acc, ex.args[1])
        ex.head == :(<:) && return bindings!(acc, ex.args[1])
        foreach(a -> bindings!(acc, a), ex.args)
    elseif ex.head == :call
        foreach(a -> bindings!(acc, a), ex.args[2:end])
    elseif ex.head == :where
        bindings!(acc, ex.args[1])
        foreach(a -> bindings!(acc, a), ex.args[2:end])
    end
    return
end

function collect_locals!(acc::Set{Symbol}, ex)
    ex isa Expr || return
    if ex.head in (:function, :(->), :macro) && length(ex.args) >= 1
        bindings!(acc, ex.args[1])
    elseif ex.head == :(=) && length(ex.args) == 2
        bindings!(acc, ex.args[1])
    elseif ex.head == :for
        bindings!(acc, ex.args[1])
    elseif ex.head == :where
        foreach(a -> bindings!(acc, a), ex.args[2:end])
    elseif ex.head == :do
        bindings!(acc, ex.args[2].args[1])
    elseif ex.head == :struct
        ex.args[2] isa Expr && bindings!(acc, ex.args[2])
        for f in ex.args[3].args
            f isa Expr && f.head == :(::) && bindings!(acc, f.args[1])
            f isa Symbol && push!(acc, f)
        end
    elseif ex.head == :try && length(ex.args) >= 2 && ex.args[2] isa Symbol
        push!(acc, ex.args[2])
    elseif ex.head == :let
        bindings!(acc, ex.args[1])
    end
    foreach(a -> collect_locals!(acc, a), ex.args)
    return
end

function collect_refs!(acc::Vector{Tuple{Symbol, Int}}, ex, line::Int)
    if ex isa Symbol
        push!(acc, (ex, line))
        return
    end
    ex isa Expr || return
    ex.head in SKIP_HEADS && return
    if ex.head == :(.) && length(ex.args) == 2
        collect_refs!(acc, ex.args[1], line)
        return
    end
    if ex.head == :kw
        collect_refs!(acc, ex.args[2], line)
        return
    end
    if ex.head == :parameters
        for a in ex.args
            a isa Expr && a.head == :kw ? collect_refs!(acc, a.args[2], line) :
            collect_refs!(acc, a, line)
        end
        return
    end
    for a in ex.args
        a isa LineNumberNode && (line = a.line; continue)
        collect_refs!(acc, a, line)
    end
    return
end

function check_file(path::String)
    src = read(path, String)
    ex = Meta.parseall(src; filename = path)
    locals = Set{Symbol}()
    collect_locals!(locals, ex)
    refs = Tuple{Symbol, Int}[]
    collect_refs!(refs, ex, 0)
    reported = Set{Symbol}()
    bad = Tuple{Symbol, Int}[]
    for (s, line) in refs
        s in locals && continue
        s in reported && continue
        str = string(s)
        (startswith(str, "@") || startswith(str, "#")) && continue
        isdefined(MOD, s) && continue
        isdefined(Base, s) && continue
        isdefined(Core, s) && continue
        push!(reported, s)
        push!(bad, (s, line))
    end
    return bad
end

function check_shadowing()
    clashes = Symbol[]
    for n in names(MOD; all = true)
        isdefined(MOD, n) || continue
        v = getfield(MOD, n)
        v isa Function || continue
        parentmodule(v) === MOD || continue
        if (isdefined(IOM, n) && getfield(IOM, n) !== v) ||
           (isdefined(POM, n) && getfield(POM, n) !== v)
            push!(clashes, n)
        end
    end
    return clashes
end

function main(files)
    status = 0
    for f in files
        bad = check_file(f)
        isempty(bad) && (println("OK   ", f); continue)
        status = 1
        println("FAIL ", f)
        for (s, line) in sort(bad; by = last)
            println("  ", f, ":", line, "  ", s)
        end
    end
    clashes = check_shadowing()
    if !isempty(clashes)
        status = 1
        println("SHADOWED IOM/POM names (define as IOM.f / POM.f methods instead):")
        foreach(c -> println("  ", c), sort(clashes))
    end
    exit(status)
end

main(ARGS)
```

- [ ] **Step 2: Write the README**

`scripts/excision/README.md`:
```markdown
# Excision tooling

`check_undefined.jl` loads `src/PowerSimulations.jl` into `Main` and reports, per file,
identifiers that resolve nowhere. Run it on every file you edit in Phase 2:

    julia --project=. scripts/excision/check_undefined.jl src/simulation/simulation.jl

False positives happen for names bound by macros or by destructuring inside `do` blocks.
Triage each line; do not silence the tool. Temporary directory: delete after Phase 6.
```

- [ ] **Step 3: Verify the checker runs on a known-good file**

The module does not load yet (Phase 1 not done), so verify the parser half only:
```sh
julia --project=. -e 'include("scripts/excision/check_undefined.jl")' 2>&1 | head -5
```
Expected: an error that originates in `Base.include(Main, .../PowerSimulations.jl)` — the load, not the checker's own code. Full verification happens in Task 1.4 Step 4.

- [ ] **Step 4: Commit**

```sh
git add -N scripts/excision/check_undefined.jl scripts/excision/README.md
git add scripts/excision .claude/plans/excision-progress.md
git commit -m "excision(0.2): add static undefined-symbol checker"
```

---

## Phase 1 — Mechanical excision (serial)

### Task 1.1: Remove the moved files

**Files:**
- Delete: every REMOVE file in spec §4a-4e (the list below is exhaustive).
- Keep on disk, untouched: EVENTS files (`src/core/event_keys.jl`, `src/core/event_model.jl`, `src/contingency_model/*.jl`, `src/simulation/simulation_events.jl`).

- [ ] **Step 1: Delete REMOVE files**

```sh
cd /home/jdlara/Sienna_work/psy6/PowerSimulations.jl
git rm -q \
  src/core/formulations.jl src/core/network_formulations.jl \
  src/core/operation_model_abstract_types.jl src/core/abstract_feedforward.jl \
  src/core/variables.jl src/core/network_reductions.jl src/core/parameters.jl \
  src/core/service_model.jl src/core/device_model.jl src/core/network_model.jl \
  src/core/auxiliary_variables.jl src/core/constraints.jl src/core/expressions.jl \
  src/core/initial_conditions.jl src/core/settings.jl src/core/dataset.jl \
  src/core/dataset_container.jl src/core/results_by_time.jl \
  src/core/power_flow_data_wrapper.jl src/core/optimization_container.jl \
  src/core/dual_processing.jl src/core/store_common.jl src/core/model_store_params.jl \
  src/operation/problem_template.jl src/operation/operation_model_interface.jl \
  src/operation/decision_model_store.jl src/operation/emulation_model_store.jl \
  src/operation/initial_conditions_update_in_memory_store.jl \
  src/operation/template_validation.jl src/operation/problem_results.jl \
  src/operation/time_series_interface.jl src/operation/optimization_debugging.jl \
  src/operation/model_numerical_analysis_utils.jl \
  src/simulation/simulation_info.jl \
  src/initial_conditions/add_initial_condition.jl \
  src/initial_conditions/calculate_initial_condition.jl \
  src/initial_conditions/initialization.jl \
  src/feedforward/feedforwards.jl src/feedforward/feedforward_arguments.jl \
  src/feedforward/feedforward_constraints.jl \
  src/parameters/add_parameters.jl \
  src/utils/indexing.jl src/utils/print_pt_v2.jl src/utils/logging.jl \
  src/utils/dataframes_utils.jl src/utils/time_series_utils.jl \
  src/utils/datetime_utils.jl src/utils/generate_valid_formulations.jl
git rm -rq src/devices_models src/services_models src/network_models
```

- [ ] **Step 2: Salvage the three PSI-only helpers before their files go**

Before deleting `src/utils/jump_utils.jl`, `src/utils/powersystems_utils.jl`, `src/utils/file_utils.jl`, copy out:

- `_calc_dimensions` (both methods, `jump_utils.jl:516-580`) into a new `src/utils/store_dimensions.jl`.
- `make_system_filename` (`powersystems_utils.jl:204-205`) and `read_dataframe` (`file_utils.jl:13`) into a new `src/utils/file_utils.jl` that contains only those two functions.

Then:
```sh
git rm -q src/utils/jump_utils.jl src/utils/powersystems_utils.jl
git add -N src/utils/store_dimensions.jl
```
(`src/utils/file_utils.jl` is modified in place, so it stays tracked.)

- [ ] **Step 3: Verify the tree**

```sh
git status --short | grep -c '^D'   # expect ~100
ls src
```
Expected `src` contents: `PowerSimulations.jl core/ operation/ initial_conditions/ contingency_model/ parameters/ simulation/ utils/`.

- [ ] **Step 4: Commit**

```sh
git add -A src
git commit -m "excision(1.1): remove model-building code moved to IOM and POM"
```

### Task 1.2: Rewrite the main module

**Files:**
- Modify: `src/PowerSimulations.jl` (full rewrite)

**Interfaces:**
- Produces: the include order and import surface every later task relies on. Names in spec §6 are imported here so file bodies stay unqualified.

- [ ] **Step 1: Write the new `src/PowerSimulations.jl`**

```julia
isdefined(Base, :__precompile__) && __precompile__()
module PowerSimulations

#################################################################################
# Exports — simulation orchestration (PSI's own API)
export Simulation
export SimulationModels
export SimulationSequence
export SimulationResults
export SimulationPartitions
export SimulationPartitionResults
export SimulationResultsExport
export SimulationProblemResults
export InterProblemChronology
export IntraProblemChronology
export GenericOpProblem
export UnitCommitmentProblem
export EconomicDispatchProblem
export template_economic_dispatch
export template_unit_commitment
export build!
export execute!
export solve!
export run!
export get_simulation_model
export run_parallel_simulation
export process_simulation_partition_cli_args
export join_simulation
export export_results
export export_realized_results
export export_optimizer_stats
export get_decision_problem_results
export get_emulation_problem_results
export get_system!
export set_system!
export list_decision_problems
export list_supported_formats
export load_results!
export read_realized_variable
export read_realized_dual
export read_realized_parameter
export read_realized_aux_variable
export read_realized_expression
export read_realized_variables
export read_realized_duals
export read_realized_parameters
export read_realized_aux_variables
export read_realized_expressions
export get_realized_timestamps
export list_simulation_events
export show_simulation_events
export show_recorder_events
export get_num_partitions

#################################################################################
# Imports
import DataStructures: OrderedDict, Deque, SortedDict
import Logging
import Serialization
import JuMP
import JuMP: optimizer_with_attributes
import JuMP.Containers: DenseAxisArray, SparseAxisArray
import JSON3
import PowerSystems as PSY
import InfrastructureSystems as IS
import InfrastructureSystems: @assert_op, TableFormat, list_recorder_events, get_name
import InfrastructureSystems.Simulation: SimulationInfo
import PowerNetworkMatrices as PNM
import PowerSystems:
    get_components, get_component, get_available_components, get_available_component,
    get_groups, get_available_groups

using InfrastructureOptimizationModels
using PowerOperationsModels
import InfrastructureOptimizationModels as IOM
import PowerOperationsModels as POM

# Unexported IOM surface PSI orchestrates with. The authoritative list is produced by
# scripts/excision/check_undefined.jl; extend this block, never qualify at call sites.
import InfrastructureOptimizationModels:
    get_store, get_status, set_status!, get_output_dir, set_output_dir!,
    get_simulation_info, set_simulation_info!, get_run_status, set_run_status!,
    is_synchronized, set_synchronized_status!, get_store_params, set_store_params!,
    advance_execution_count!, get_execution_count, get_executions, set_executions!,
    set_execution_count!, get_initial_time, is_built, warm_start_enabled,
    _pre_solve_model_checks, solve_model!, get_time_series_cache, empty_time_series_cache!,
    get_log_file, get_recorder_dir, get_initial_conditions_file, add_recorders!,
    register_recorders!, unregister_recorders!, configure_logging, get_current_timestamp,
    get_simulation_number, set_simulation_number!, get_sequence_uuid, set_sequence_uuid!,
    cost_function_unsynch, update_objective_function!, reset_optimization_model!
import InfrastructureOptimizationModels:
    ModelStoreParams, get_num_executions, get_horizon_count, get_base_power, get_system_uuid,
    DecisionModelStore, initialize_storage!, write_output!, read_outputs,
    write_optimizer_stats!, DecisionModelIndexType, EmulationModelIndexType,
    STORE_CONTAINERS, STORE_CONTAINER_DUALS, STORE_CONTAINER_PARAMETERS,
    STORE_CONTAINER_VARIABLES, STORE_CONTAINER_AUX_VARIABLES, STORE_CONTAINER_EXPRESSIONS,
    get_data_field, list_fields, list_keys
import InfrastructureOptimizationModels:
    AbstractDataset, InMemoryDataset, HDF5Dataset, DatasetContainer, make_system_state,
    get_dataset, set_dataset!, has_dataset, get_dataset_keys, get_dataset_values,
    set_dataset_values!, get_dataset_value, get_last_recorded_row, set_last_recorded_row!,
    get_update_timestamp, set_update_timestamp!, get_last_updated_timestamp,
    get_last_recorded_value, get_last_update_value, get_num_rows, get_data_resolution,
    get_end_of_step_timestamp, get_value_timestamp, set_value!, get_dataset_size,
    get_column_names, OutputsByTime, OutputsByKeyAndTime, make_dataframes
import InfrastructureOptimizationModels:
    get_parameter_attributes, get_parameter_array, get_parameter_multiplier_array,
    get_attribute_key, get_time_series_name, _get_ts_uuid, _set_param_value_at!,
    set_parameter!, ValidDataParamEltypes, TimeSeriesAttributes, VariableValueAttributes,
    CostFunctionAttributes, EventParametersAttributes, NoAttributes
import InfrastructureOptimizationModels:
    LOG_GROUP_SIMULATION_STORE, LOG_GROUP_OUTPUTS, LOG_GROUP_MODEL_STORE,
    LOG_GROUP_BUILD_INITIAL_CONDITIONS, set_interval!, get_variables, get_parameters,
    get_duals, get_expressions, get_aux_variables, get_time_steps, find_timestamp_index,
    to_matrix, get_column_names_from_axis_array, should_write_resulting_value,
    convert_output_to_natural_units, deserialize_key, get_initial_conditions_data,
    _deepcopy_template, get_deterministic_time_series_type, RUN_OPERATION_MODEL_TIMER,
    to_outputs_dataframe, _read_outputs, get_time_series_values!

import TimerOutputs
import ProgressMeter
import Distributed
import Random
import Random: AbstractRNG
import Dates
import TimeSeries
import DataFrames
import DataFrames: DataFrame, DataFrameRow, Not, innerjoin
import DataFramesMeta: @chain, @orderby, @rename, @select, @subset, @transform
import CSV
import HDF5
import PrettyTables

# Re-export the IOM and POM public API so `using PowerSimulations` is sufficient (spec D1).
for m in (InfrastructureOptimizationModels, PowerOperationsModels)
    for n in names(m)
        n === nameof(m) && continue
        @eval export $n
    end
end

################################################################################
const PSI = PowerSimulations
const ISOPT = IS.Optimization
const TS = TimeSeries

function progress_meter_enabled()
    return isa(stderr, Base.TTY) &&
           (get(ENV, "CI", nothing) != "true") &&
           (get(ENV, "RUNNING_PSI_TESTS", nothing) != "true")
end

using DocStringExtensions

@template DEFAULT = """
                    $(TYPEDSIGNATURES)
                    $(DOCSTRING)
                    """

# Includes — order matters. Constants and types before their users.
include("core/definitions.jl")
include("core/abstract_simulation_store.jl")
include("core/cache_utils.jl")
# EVENTS-EXCISION: event framework not yet in POM.
# include("core/event_keys.jl")
# include("core/event_model.jl")

include("initial_conditions/initial_condition_chronologies.jl")
include("simulation/simulation_store_requirements.jl")
include("operation/operation_model_types.jl")
include("operation/decision_model.jl")
include("operation/emulation_model.jl")

include("initial_conditions/update_initial_conditions.jl")

# EVENTS-EXCISION: contingency model not yet in POM.
# include("contingency_model/contingency.jl")
# include("contingency_model/contingency_arguments.jl")
# include("contingency_model/contingency_constraints.jl")

include("simulation/optimization_output_cache.jl")
include("simulation/optimization_output_caches.jl")
include("simulation/simulation_models.jl")
include("simulation/simulation_state.jl")
include("simulation/initial_condition_update_simulation.jl")
include("simulation/simulation_store_params.jl")
include("simulation/hdf_simulation_store.jl")
include("simulation/in_memory_simulation_store.jl")
include("simulation/simulation_store_common.jl")
include("simulation/simulation_problem_results.jl")
include("simulation/get_components_interface.jl")
include("simulation/decision_model_simulation_results.jl")
include("simulation/emulation_model_simulation_results.jl")
include("simulation/realized_meta.jl")
include("simulation/simulation_partitions.jl")
include("simulation/simulation_partition_results.jl")
include("simulation/simulation_sequence.jl")
include("simulation/simulation_internal.jl")
include("simulation/simulation.jl")
# EVENTS-EXCISION: include("simulation/simulation_events.jl")
include("simulation/simulation_results_export.jl")
include("simulation/simulation_results.jl")
include("operation/operation_model_simulation_interface.jl")
include("parameters/update_container_parameter_values.jl")
include("parameters/update_cost_parameters.jl")
include("parameters/update_parameters.jl")

include("operation/operation_problem_templates.jl")

include("utils/store_dimensions.jl")
include("utils/print_pt_v3.jl")
include("utils/file_utils.jl")
include("utils/recorder_events.jl")

end
```

Notes for the implementer:
- The `import InfrastructureOptimizationModels: …` blocks will fail at load for any name IOM does not define. Task 1.4 fixes the list against the real IOM; do not guess — run the load and remove or fix each reported name.
- `solve!`, `run!`, `build!` are POM functions. PSI's `solve!(step, model, start_time, store)` must be defined as `function POM.solve!(step::Int, …)` (Task 2.2). Exporting them from PSI is then a re-export of the same binding.
- `get_system!`, `set_system!`, `read_realized_*`, `export_results` and friends are PSI-defined on PSI result types and do not exist in IOM; if the checker in Task 1.4 reports a shadowing clash for one of them, define it as a method of the IOM function instead.

- [ ] **Step 2: Trim `src/core/definitions.jl`**

Keep only:
```julia
const SIMULATION_LOG_FILENAME = "simulation.log"
const PROBLEM_LOG_FILENAME = "problem_build.log"
const REQUIRED_RECORDERS = (:simulation_status, :simulation)
const RESULTS_DIR = "results"
const KNOWN_SIMULATION_PATHS = [
    "data_store",
    "logs",
    "models_json",
    "recorder",
    "results",
    "simulation_files",
    "simulation_partitions",
]
const IGNORABLE_FILES = ["tar.gz", ".DS_Store"]
const NO_SERVICE_NAME_PROVIDED = ""
const RUN_SIMULATION_TIMER = TimerOutputs.TimerOutput()
```
Copy the exact current values for `REQUIRED_RECORDERS`, `KNOWN_SIMULATION_PATHS`, `IGNORABLE_FILES` from the existing file before deleting the rest. Delete every alias, tolerance, enum, and `ENUM_MAPPINGS` (they are IOM's). Keep the `JuMPFloatArray`/`JuMPVariableArray` style aliases **only** if a KEEP file uses them and IOM does not export them (check with `grep -rn` across `src/` and `isdefined(IOM, :name)`).

- [ ] **Step 3: Trim `src/operation/operation_model_types.jl`**

```julia
struct GenericOpProblem <: POM.GenericPowerDecisionProblem end
struct UnitCommitmentProblem <: POM.GenericPowerDecisionProblem end
struct EconomicDispatchProblem <: POM.GenericPowerDecisionProblem end
```
Drop `AGCReserveDeployment` (spec D7). If POM's `DecisionModel{M}` constructors need `M <: GenericPowerDecisionProblem` they are satisfied by these.

- [ ] **Step 4: Commit**

```sh
git add src/PowerSimulations.jl src/core/definitions.jl src/operation/operation_model_types.jl
git commit -m "excision(1.2): rewrite main module onto IOM and POM"
```

### Task 1.3: Fence events and cut the SPLIT files to their KEEP fragments

**Files:**
- Modify: `src/operation/decision_model.jl`, `src/operation/emulation_model.jl`, `src/operation/operation_problem_templates.jl`, `src/utils/print_pt_v3.jl`, `src/simulation/simulation_sequence.jl`, `src/simulation/simulation_state.jl`, `src/simulation/simulation.jl`, `src/parameters/update_container_parameter_values.jl`, `src/utils/recorder_events.jl`

This task cuts and fences only; it does not fix names. Use the line numbers in spec §4b-4e (they refer to the pre-excision files; re-locate by function name, never by line alone).

- [ ] **Step 1: `decision_model.jl`** — delete everything except `solve!(step::Int, model::DecisionModel{<:DecisionProblem}, start_time, store::SimulationStore; exports)` and its docstring. Change the signature to `function POM.solve!(step::Int, model::DecisionModel{<:POM.AbstractPowerDecisionProblem}, start_time::Dates.DateTime, store::SimulationStore; exports = nothing)`. Inside: `solve_impl!(model)` → `solve_model!(model)`. Keep the `write_results!(store, model, start_time, start_time; exports)` call: it is PSI's fan-out into its own `SimulationStore` (IOM's `write_outputs!` dispatches on `AbstractModelStore`, which `SimulationStore` is not). Re-create that fan-out in the new file `src/simulation/simulation_store_common.jl` (already in the Task 1.2 include list): copy `write_results!` and the five `write_model_*_results!` functions verbatim from the pre-excision `src/core/store_common.jl` (`git show main:src/core/store_common.jl`). Their bodies read the container (`get_duals`, `get_parameters`, …) and call `write_result!(store, …)` on the `SimulationStore`; leave the names unchanged.

- [ ] **Step 2: `emulation_model.jl`** — keep only: `update_parameters!(model::EmulationModel, ::EmulationModelStore)`, `update_parameters!(model::EmulationModel, ::DatasetContainer{InMemoryDataset})`, `update_initial_conditions!(model::EmulationModel, ::EmulationModelStore)`, `update_model!(model::EmulationModel, ::EmulationModelStore)`, `update_model!(model::EmulationModel, ::DatasetContainer)`, `update_parameter_values!(model::EmulationModel, key, ::EmulationModelStore)`, and `solve!(step, model::EmulationModel, start_time, store::SimulationStore; exports)`. Signature: `function POM.solve!(step::Int, model::EmulationModel{<:POM.AbstractPowerEmulationProblem}, …)`. `solve_impl!` → `solve_model!`.

- [ ] **Step 3: `operation_problem_templates.jl`** — keep `template_unit_commitment` and `template_economic_dispatch`; delete `template_agc_reserve_deployment`. Rename: `ProblemTemplate(NetworkModel(network))` → `PowerOperationsProblemTemplate(NetworkModel(network))`; default `network = CopperPlatePowerModel` → `CopperPlateNetworkModel`. Device/service formulation names are unchanged in POM.

- [ ] **Step 4: `print_pt_v3.jl`** — delete lines 1-269 (container/model/results show methods; IOM and POM own them). Keep the `SimulationModels`, `SimulationSequence`, `Simulation`, `SimulationResults` show methods and `_get_*_for_show`. Replace `ProblemResultsTypes` union at the end with `Union{SimulationProblemResults}` only if a caller remains; else delete it.

- [ ] **Step 5: Fence events in KEEP files.** Fence each block with:
```julia
#= EVENTS-EXCISION: <one-line what>
<original code, untouched>
=#
```
Blocks: `simulation_sequence.jl` (`_add_event_to_model`, `_validate_event_timeseries_data`, `_add_model_to_event_map!`, `_attach_events!`, `get_events`; the `events` field and its kwarg default in the `SimulationSequence` struct and constructor — remove the field and the kwarg, fence their lines; the `_attach_events!(…)` call in the constructor); `simulation_state.jl` (all methods listed in spec §4c); `simulation.jl` (`_is_event_countdown_parameter_key` ×2; the `apply_simulation_events!` call; in `_update_simulation_state_parameters!` fence the countdown-first split and replace with a single `for key in keys(get_parameters(container))` loop calling `update_decision_state!` exactly as the second half of the current function does); `update_container_parameter_values.jl` (the two `EventParametersAttributes` methods, the `EventParameter` dispatchers for `PSY.Component` and `PSY.Service`, and the `has_outage`/`AvailableStatusParameter` branch inside the `OnVariable` emulation method — fence the branch, keep the non-outage path); `recorder_events.jl` (the `ParameterUpdateEvent` constructor taking `EventParametersAttributes`).

Fencing means the fenced code must be **exactly** the pre-excision code so the follow-up is a pure unfence.

- [ ] **Step 6: Verify the fences are complete**

```sh
grep -rn "EventModel\|EventKey\|apply_simulation_events\|AvailableStatus\|ActivePowerOffsetParameter\|ReactivePowerOffsetParameter\|_get_outage_occurrence\|_get_time_to_recover" src --include=*.jl \
  | grep -v "EVENTS-EXCISION" | grep -v "^src/core/event_\|^src/contingency_model\|^src/simulation/simulation_events.jl"
```
Expected: only lines that sit **inside** a fence (open the reported file to confirm) or zero output. Anything outside a fence is a miss.

- [ ] **Step 7: Format and commit**

```sh
julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'
git add -N src/simulation/simulation_store_common.jl
git add src
git commit -m "excision(1.3): cut SPLIT files to simulation fragments and fence events"
```

### Task 1.4: Load gate — `using PowerSimulations` succeeds

**Files:**
- Modify: any `src/` file, limited to definition-level fixes (signatures, struct field types, supertypes, `const` values, import lists). Do not fix function bodies here; that is Phase 2.

- [ ] **Step 1: Load loop**

```sh
julia --project=. -e 'using PowerSimulations' 2>&1 | head -40
```
Fix the first error, re-run, repeat. Typical fixes:
- `UndefVarError` in an `import InfrastructureOptimizationModels: name` line → check `isdefined(IOM, :name)` in a REPL; remove the name if IOM lacks it and note it in `excision-progress.md` under "gaps".
- `UndefVarError: OperationModel` in a signature → `IOM.AbstractOptimizationModel` (spec §5).
- `UndefVarError: DecisionProblem` → `POM.AbstractPowerDecisionProblem`.
- `UndefVarError: ResultsByKeyAndTime` in a struct field → `OutputsByKeyAndTime`.
- `UndefVarError: EventModel` in `SimulationSequence` → Task 1.3 missed a fence; fix there.
- "cannot assign a value to imported variable `IOM.get_system`" or a method-extension warning → define as `function IOM.get_system(res::SimulationProblemResults)`; same for `get_system!`? (no — that one is PSI-only), `read_variable`, `list_variable_keys`, `get_timestamps`, `get_resolution`, `read_optimizer_stats`, `export_results`, `serialize_results`… For each, check `isdefined(IOM, name)`; if true, extend, else define.

- [ ] **Step 2: Run the checker on the whole tree to size Phase 2**

```sh
julia --project=. scripts/excision/check_undefined.jl $(git ls-files 'src/**/*.jl' | grep -v 'event_\|contingency_model\|simulation_events') > .claude/plans/excision-checker-baseline.txt 2>&1; tail -3 .claude/plans/excision-checker-baseline.txt
```
Expected: the module loads (no load error at the top of the file); many `FAIL` lines. This file is the work list for Phase 2.

- [ ] **Step 3: Aqua sanity (exports)**

```sh
julia --project=. -e 'using PowerSimulations, Aqua; Aqua.test_undefined_exports(PowerSimulations)' 2>&1 | tail -5
```
Expected: pass. If `Aqua` is not in the package env, run it from `--project=test` after Task 4.1 instead and note that here.

- [ ] **Step 4: Format and commit**

```sh
julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'
git add -N .claude/plans/excision-checker-baseline.txt
git add src .claude/plans
git commit -m "excision(1.4): module loads against IOM and POM"
```

**Phase 1 gate:** `julia --project=. -e 'using PowerSimulations'` exits 0. The user reviews `git log main..HEAD` before Phase 2 starts.

---

## Phase 2 — Per-file body rewrite (parallel, one agent per task)

Every Phase 2 task follows the same recipe. Task text below lists only the file set, the known hot spots, and the checks. Agents touch **only** their listed files. If a fix genuinely belongs in another file, record it in `excision-progress.md` under "cross-file" and stop; the Phase 3 integrator handles it.

**Recipe (each task):**
1. Read spec §5 (rename map), §6 (import list), §7 (gaps).
2. Read the checker baseline for your files: `grep -A50 "^FAIL <file>" .claude/plans/excision-checker-baseline.txt`.
3. For each reported symbol: rename per §5; or confirm it is a false positive (locally bound, macro-generated); or, if it is a PSI-only helper that lived in a REMOVE file, re-create it in your file **only if** IOM and POM have no equivalent (check `isdefined(IOM, :name)`, `isdefined(POM, :name)`, and `grep -rn` in both `src/` trees).
4. Sweep every `PSY.` call in your files against psy6 PSY: open the getter in `../PowerSystems.jl/src` and pass `PSY.SU` where the field is convertible. Sweep every `IS.` call similarly against `../InfrastructureSystems.jl/src`.
5. Re-run: `julia --project=. scripts/excision/check_undefined.jl <your files>` until `OK` for each, and `julia --project=. -e 'using PowerSimulations'` still exits 0.
6. Format, `git add <your files>`, commit `excision(2.x): <file group> bodies onto IOM/POM`. Append a row to `excision-progress.md` (that file is shared; edit and commit it with your task's commit, retrying on lock).

### Task 2.1: Core and store scaffolding

**Files:** `src/core/abstract_simulation_store.jl`, `src/core/cache_utils.jl`, `src/simulation/simulation_store_requirements.jl`, `src/simulation/simulation_store_params.jl`, `src/initial_conditions/initial_condition_chronologies.jl`, `src/simulation/get_components_interface.jl`, `src/utils/file_utils.jl`, `src/utils/store_dimensions.jl`.

Hot spots: `simulation_store_requirements.jl` uses `ConstraintKey`, `ParameterKey`, `VariableKey`, `AuxVarKey`, `ExpressionKey` — IOM exports them. `get_components_interface.jl` forwards `PSY.get_components(::Type, ::IS.Results)`; check each forwarded function still exists in psy6 PSY. `store_dimensions.jl` (`_calc_dimensions`) uses `get_column_names` — IOM's is imported.

### Task 2.2: Operation glue

**Files:** `src/operation/decision_model.jl`, `src/operation/emulation_model.jl`, `src/operation/operation_model_simulation_interface.jl`, `src/simulation/simulation_store_common.jl`, `src/operation/operation_problem_templates.jl`, `src/operation/operation_model_types.jl`.

Hot spots: `update_parameters!(model::DecisionModel, state::SimulationState)` calls `cost_function_unsynch(container)`, iterates `get_parameters(container)`, calls `update_parameter_values!(model, key, state)`, then `update_objective_function!` and `set_synchronized_status!` — all IOM names in the import list. `update_model!(model, state, chronology)` wraps the two updates in `TimerOutputs.@timeit RUN_SIMULATION_TIMER`. The store fan-out in `simulation_store_common.jl` reads `get_duals(container)`, `get_parameters`, `get_variables`, `get_aux_variables`, `get_expressions`, applies `should_write_resulting_value` and the `exports` predicates, then calls `write_result!(store, model_name, key, index, update_timestamp, array)`. In `emulation_model.jl`, `update_parameters!(model, ::EmulationModelStore)` calls `update_parameter_values!(model, key, store)` — confirm PSI's `update_parameters.jl` still defines the `EmulationModelStore` variant or the `DatasetContainer` variant it relies on.

### Task 2.3: Parameter updates

**Files:** `src/parameters/update_parameters.jl`, `src/parameters/update_container_parameter_values.jl`.

Hot spots: `lookup_additional_axes` and `_unwrap_for_param` lived in the removed `add_parameters.jl`; check POM `common_models/add_parameters.jl` for equivalents (`grep -n "lookup_additional_axes\|_unwrap_for_param" ../PowerOperationsModels.jl/src -r`); if absent, re-create them in `update_container_parameter_values.jl` from `git show main:src/parameters/add_parameters.jl` (lines ~723-753). The `PNM.get_all_branch_maps_by_type` / `get_name_to_arc_map` block (lines ~132-150) reads the network reduction from the model's `NetworkModel` — the accessor is now `IOM.get_network_reduction(network_model)`. `get_time_series_values!(model, component, name, initial_time, horizon)` signatures are in IOM `operation/time_series_interface.jl:1,45`. `_set_param_value!` helpers may exist in IOM as `_set_param_value_at!` (imported) — prefer IOM's. The `ParameterUpdateEvent` constructors are in `utils/recorder_events.jl` (Task 2.6 owns that file; use the existing constructor signatures).

### Task 2.4: Parameter cost updates (highest PSY-psy6 risk)

**Files:** `src/parameters/update_cost_parameters.jl`.

Hot spots: this file rebuilds `ProductionCostExpression`/`FuelCostExpression`/`StartUpCostExpression`/`ShutDownCostExpression` from time-varying `PSY.MarketBidCost`, `PSY.OfferCurveCost`, `PSY.ReserveDemandCurve`, `PSY.ThermalGenerationCost` — every one of these changed in psy6 (cost curve unit marker is a type parameter; `variable` → `variable_operation_cost`; `IS.UnitSystem` gone). Do not port line by line. For each `handle_variable_cost_parameter` / `update_variable_cost!` method, find the **build-time** equivalent in POM `common_models/market_bid_plumbing.jl`, `market_bid_overrides.jl`, `common_models/objective_function.jl`, and IOM `objective_function/value_curve_cost.jl`, `offer_curve_types.jl`, and reuse those functions for the per-step recomputation (mirror structure exactly; divergence only where "recompute at time t" is the point). Every `PSY.get_*` cost accessor passes the unit system explicitly. If an IOM/POM function the update needs is `_`-prefixed, use `IOM._name` with a comment naming the build-time caller it mirrors; record it under "IOM private reaches" in `excision-progress.md`. Test coverage for this file arrives in Task 4.4 (MarketBidCost simulation testsets).

### Task 2.5: Initial conditions

**Files:** `src/initial_conditions/update_initial_conditions.jl`, `src/simulation/initial_condition_update_simulation.jl`.

Hot spots: IC types `DevicePower`, `DeviceStatus`, `DeviceAboveMinPower`, `InitialTimeDurationOn/Off`, `InitialEnergyLevel` are POM exports; `InitialCondition`, `get_condition`, `set_ic_quantity!`, `get_component`, `get_value` are IOM's. The per-type `update_initial_conditions!(ics::Vector{InitialCondition{T}}, state::SimulationState, resolution)` methods stay PSI-defined but must be **methods of `IOM.update_initial_conditions!`** (`function IOM.update_initial_conditions!(…)`) to avoid shadowing. Same for the `(model, key, source)` forwarders. `requires_reconciliation` is POM's.

### Task 2.6: Simulation core

**Files:** `src/simulation/simulation_models.jl`, `src/simulation/simulation_sequence.jl`, `src/simulation/simulation_state.jl`, `src/simulation/simulation_internal.jl`, `src/simulation/simulation.jl`, `src/utils/recorder_events.jl`.

Hot spots: `simulation.jl` `update_model!(model, sim)` rebuild path: `reset_optimization_model!(container)` then `POM.build_problem!(container, get_template(model), get_system(model))`. `_build_single_model_for_simulation` calls POM `build!(model; output_dir, recorders, console_level, file_level, disable_timer_outputs, store_system_in_results)`; POM's `build!` kwargs are listed in POM `operation/decision_model.jl:77` — match them exactly. `initialize_simulation_internals!` sets `SimulationInfo` via `set_simulation_info!(model, SimulationInfo())` — IOM's `DecisionModel` constructor already creates one; use `set_simulation_number!`/`set_sequence_uuid!`. `_get_model_store_requirements!` uses `_calc_dimensions` (Task 2.1 file). `simulation_sequence.jl` `_add_feedforward_to_model` calls `attach_feedforward!(model, ff)` on the device model — POM's; `get_feedforward_meta` — POM's; `NO_SERVICE_NAME_PROVIDED` — PSI's. `simulation_state.jl` `STATE_TIME_PARAMS` and `_get_state_params` read `get_horizon(model)`, `get_interval(model)`, `get_resolution(model)` (IOM). `simulation_internal.jl` keeps `Random.Xoshiro(IS.get_random_seed())` — check `IS.get_random_seed` exists in IS4.

### Task 2.7: Stores

**Files:** `src/simulation/hdf_simulation_store.jl`, `src/simulation/in_memory_simulation_store.jl`, `src/simulation/optimization_output_cache.jl`, `src/simulation/optimization_output_caches.jl`.

Hot spots: `in_memory_simulation_store.jl` holds `DecisionModelStore`/`EmulationModelStore` per model and calls `write_output!`/`read_outputs`/`initialize_storage!` on them (IOM renames of `write_result!`/`read_results`). `hdf_simulation_store.jl` uses `DatasetContainer{HDF5Dataset}`, `HDF5Dataset{1}`/`{2}` constructors (IOM `core/dataset.jl:227,246`) — compare field lists; `to_matrix`; `get_column_names`; `ModelStoreParams` getters; `deserialize_key`/`get_container_key` from `OptimizationContainerMetadata`; `serialize_system!` uses `PSY.to_json(sys, path)` — confirm psy6 PSY signature.

### Task 2.8: Results

**Files:** `src/simulation/simulation_problem_results.jl`, `src/simulation/decision_model_simulation_results.jl`, `src/simulation/emulation_model_simulation_results.jl`, `src/simulation/realized_meta.jl`, `src/simulation/simulation_results.jl`, `src/simulation/simulation_results_export.jl`, `src/simulation/simulation_partitions.jl`, `src/simulation/simulation_partition_results.jl`, `src/utils/print_pt_v3.jl`.

Hot spots: `ResultsByKeyAndTime` → `OutputsByKeyAndTime`, `ResultsByTime` → `OutputsByTime`, `make_dataframes`, `to_results_dataframe` → `to_outputs_dataframe`. `SimulationProblemResults{T} <: IS.Results` extends `IS.Optimization`/IOM functions (`read_variable`, `list_variable_keys`, `get_timestamps`, `get_resolution`, `read_optimizer_stats`, `get_system`, `export_results`, `serialize_results`) — every one must be `function IOM.name(res::SimulationProblemResults, …)` or `function ISOPT.name(…)` depending on where the generic lives (`parentmodule(IOM.read_variable)` tells you). `OptimizationProblemResultsExport` → `IOM.OptimizationProblemOutputsExport` in `simulation_results_export.jl`. `_deserialize_system` uses `PSY.System(path)` — confirm psy6.

**Phase 2 gate:** `julia --project=. scripts/excision/check_undefined.jl $(git ls-files 'src/**/*.jl' | grep -v 'event_\|contingency_model\|simulation_events')` prints only `OK` lines and no `SHADOWED` section. The user reviews before Phase 3.

---

## Phase 3 — Integration (serial)

### Task 3.1: Cross-file fixes, Aqua, smoke simulation

**Files:** any `src/` file; Create: `scripts/excision/smoke_simulation.jl`.

- [ ] **Step 1: Resolve the "cross-file" ledger entries** from `excision-progress.md`, one at a time, re-running the checker after each.

- [ ] **Step 2: Write the smoke script** (this is also the Phase 6 end-to-end check):

```julia
# scripts/excision/smoke_simulation.jl — run with: julia --project=test scripts/excision/smoke_simulation.jl
using PowerSimulations, PowerSystems, PowerSystemCaseBuilder, HiGHS, Dates, Logging
import PowerSystemCaseBuilder: PSITestSystems
const PSI = PowerSimulations
c_sys5_uc = build_system(PSITestSystems, "c_sys5_uc")
c_sys5_ed = build_system(PSITestSystems, "c_sys5_ed")
solver = optimizer_with_attributes(HiGHS.Optimizer, "mip_rel_gap" => 0.01, "output_flag" => false)
template_uc = template_unit_commitment(; network = CopperPlateNetworkModel)
template_ed = template_economic_dispatch(; network = CopperPlateNetworkModel)
models = SimulationModels(;
    decision_models = [
        DecisionModel(template_uc, c_sys5_uc; name = "UC", optimizer = solver),
        DecisionModel(template_ed, c_sys5_ed; name = "ED", optimizer = solver),
    ],
)
sequence = SimulationSequence(;
    models = models,
    feedforwards = Dict(
        "ED" => [SemiContinuousFeedforward(;
            component_type = ThermalStandard,
            source = OnVariable,
            affected_values = [ActivePowerVariable],
        )],
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
@assert size(df, 1) == 48
println("smoke OK: ", size(df))
```
`template_unit_commitment`'s keyword name for the network must match Task 1.3 Step 3. If `build_system` needs `PSB` argument changes on psy6, copy the call form from POM `test/test_utils/`.

- [ ] **Step 3: Run the smoke script**

The test env does not exist until Task 4.1, so temporarily run it from the package env with `PowerSystemCaseBuilder` and `HiGHS` added by `Pkg.add`-free means: `julia --project=. -e 'using Pkg; Pkg.develop(path="../PowerSystemCaseBuilder.jl"); Pkg.add("HiGHS")'`, run, then `git checkout Project.toml Manifest.toml` to restore. Or defer this step to Task 4.1 Step 6 and note it. Fix every runtime error surfaced here; these are the body bugs the static checker cannot see (wrong argument order, changed kwargs, units).

- [ ] **Step 4: Aqua**

```sh
julia --project=. -e 'using PowerSimulations, Aqua; Aqua.test_undefined_exports(PowerSimulations); Aqua.test_ambiguities(PowerSimulations); Aqua.test_unbound_args(PowerSimulations)'
```
Expected: all pass (run from `--project=test` if Aqua is not in the package env).

- [ ] **Step 5: Format and commit**

```sh
julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'
git add -N scripts/excision/smoke_simulation.jl
git add src scripts/excision .claude/plans/excision-progress.md
git commit -m "excision(3.1): integration fixes; smoke simulation runs"
```

**Phase 3 gate:** smoke simulation prints `smoke OK`. User review.

---

## Phase 4 — Tests

### Task 4.1: Test infrastructure (serial, before 4.2-4.x)

**Files:**
- Modify: `test/Project.toml`, `test/includes.jl`, `test/runtests.jl`
- Modify: `test/test_utils/mock_operation_models.jl`, `test/test_utils/operations_problem_templates.jl`, `test/test_utils/model_checks.jl`, `test/test_utils/common_operation_model.jl`, `test/test_utils/run_simulation.jl`
- Delete: REMOVE test files (spec §9), `test/test_utils/add_branch_rating_time_series.jl`, `test/performance/`

- [ ] **Step 1: `test/Project.toml`** — `[deps]`: Aqua, CSV, DataFrames, DataFramesMeta, DataStructures, Dates, HiGHS, InfrastructureOptimizationModels, InfrastructureSystems, JSON3, JuMP, Logging, Pkg, PowerFlows, PowerNetworkMatrices, PowerOperationsModels, PowerSimulations, PowerSystemCaseBuilder, PowerSystems, Random, Serialization, Test, TestSetExtensions, TimeSeries, TimerOutputs, UUIDs, plus the seven OpenAPI packages and `InfraStore` if PSB/PSY need it (copy from POM `test/Project.toml`). `[sources]`: `PowerSimulations = {path = ".."}`, path pins for IS, IOM, PSY, PNM, POM, PSB (`../../PowerSystemCaseBuilder.jl`), PowerFlows (`../../PowerFlows.jl`), and the OpenAPI git pins. `[compat]`: `HiGHS = "1"`, `julia = "^1.11"`.

- [ ] **Step 2: `test/includes.jl`** — replace the `using`/`import` block:
```julia
using PowerSimulations
using PowerOperationsModels
using InfrastructureOptimizationModels
using PowerSystems
using PowerSystemCaseBuilder
using InfrastructureSystems
using PowerNetworkMatrices
using PowerFlows
import PowerSystemCaseBuilder: PSITestSystems
using DataFramesMeta
using Test
using Logging
using DataFrames
using Dates
using JuMP
import JuMP.Containers: DenseAxisArray, SparseAxisArray
using TimeSeries
using CSV
import JSON3
using DataStructures
import UUIDs
using Random
import Serialization
import PowerSystems as PSY
import PowerSimulations as PSI
import PowerOperationsModels as POM
import InfrastructureOptimizationModels as IOM
import PowerNetworkMatrices as PNM
import InfrastructureSystems as IS
const PFS = PowerFlows
const PSB = PowerSystemCaseBuilder
const ISOPT = IS.Optimization
const BASE_DIR = string(dirname(dirname(pathof(PowerSimulations))))
const DATA_DIR = joinpath(BASE_DIR, "test/test_data")
```
Includes: keep `common_operation_model.jl`, `model_checks.jl`, `mock_operation_models.jl`, `solver_definitions.jl`, `operations_problem_templates.jl`, `run_simulation.jl`, `add_components_to_system.jl`, `add_market_bid_cost.jl`, `mbc_system_utils.jl`, `mbc_simulation_utils.jl`, `iec_simulation_utils.jl`. Comment `# EVENTS-EXCISION: include("test_utils/events_simulation_utils.jl")`. Drop `add_branch_rating_time_series.jl`.

- [ ] **Step 3: `test/runtests.jl`** — `DISABLED_TEST_FILES = ["test_events.jl", "test_postcontingency_mixed_outage_axes.jl"]  # EVENTS-EXCISION`. Keep the Aqua block.

- [ ] **Step 4: Delete REMOVE tests**

```sh
git rm -q test/test_model_decision.jl test/test_model_emulation.jl test/test_multi_interval.jl \
  test/test_initialization_problem.jl test/test_problem_template.jl test/test_basic_model_structs.jl \
  test/test_formulation_combinations.jl test/test_jump_utils.jl test/test_network_constructors.jl \
  test/test_network_constructors_with_branch_rating_time_series.jl test/test_device_branch_constructors.jl \
  test/test_device_thermal_generation_constructors.jl test/test_device_load_constructors.jl \
  test/test_device_renewable_generation_constructors.jl test/test_device_source_constructors.jl \
  test/test_device_synchronous_condenser_constructors.jl test/test_device_hvdc.jl test/test_device_lcc.jl \
  test/test_services_constructor.jl test/test_ac_transmission_security_constrained_models.jl \
  test/test_static_injection_security_constrained_models.jl test/test_mbc_sanity_check.jl \
  test/test_power_flow_in_the_loop.jl test/test_pf_injection_resolver.jl \
  test/test_parallel_branch_parameter_multipliers.jl test/test_utils/add_branch_rating_time_series.jl
git rm -rq test/performance
```
`test_market_bid_cost.jl` and `test_import_export_cost.jl` are SPLIT (Task 4.4), not deleted here.

- [ ] **Step 5: Rewrite test utils**
- `mock_operation_models.jl`: `struct MockOperationProblem <: POM.GenericPowerDecisionProblem end`, `struct MockEmulationProblem <: POM.GenericPowerEmulationProblem end`; constructors call `IOM.DecisionModel(::Type{MockOperationProblem}, ::Type{T}, sys; …)`; copy the constructor bodies from POM `test/test_utils/mock_operation_models.jl:9-112` (they already target IOM). Keep `setup_ic_model_container!`.
- `operations_problem_templates.jl`: every `ProblemTemplate` → `PowerOperationsProblemTemplate`; `*PowerModel` → `*NetworkModel`; `NETWORKS_FOR_TESTING` drops `PM.*`; **hydro stays** (spec D4) using POM's `HydroDispatchRunOfRiver`, `HydroCommitmentRunOfRiver`, `HydroEnergyModelReservoir`, etc. — map each old HydroPowerSimulations formulation name to POM's by opening POM `src/core/formulations.jl` hydro section and choosing the same-meaning type; if a formulation has no POM equivalent, drop that device model and record it.
- `model_checks.jl`: keep only helpers a KEEP/SPLIT test calls (`grep -ho "[a-z_]*(" test/test_simulation*.jl test/test_recorder_events.jl test/test_print.jl test/test_utils.jl test/run_partitioned_simulation.jl | sort -u`), rewrite `PSI.get_optimization_container` → `IOM.get_optimization_container`, etc.
- `common_operation_model.jl`: `PSI.OperationModel` → `IOM.AbstractOptimizationModel`.
- `run_simulation.jl`: template and network renames; PF-in-the-loop helpers use `POM.power_flow_evaluations(PFS.DCPowerFlow())` (POM's exported builder).

- [ ] **Step 6: Instantiate the test env and run the smoke script from it**

```sh
julia --project=test -e 'using Pkg; Pkg.resolve(); Pkg.instantiate()'
julia --project=test -e 'include("test/includes.jl"); println("includes OK")'
julia --project=test scripts/excision/smoke_simulation.jl
```
Expected: `includes OK`, then `smoke OK`.

- [ ] **Step 7: Format and commit**

```sh
julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'
git add -A test
git commit -m "excision(4.1): test infrastructure onto IOM and POM"
```

### Tasks 4.2 – 4.8: One agent per test file (parallel)

Recipe per task: run the file in isolation, fix the **test** and, when the failure is a PSI bug, the PSI source (list every `src/` edit in `excision-progress.md`); re-run until green; format; commit only the touched files.

Run command: `julia --project=test -e 'include("test/includes.jl"); include("test/<file>")'`.

| task | file(s) | notes |
|---|---|---|
| 4.2 | `test_simulation_models.jl`, `test_simulation_sequence.jl` | renames only; `SimulationSequence(; events = …)` kwarg no longer exists — fence those testsets with `EVENTS-EXCISION` |
| 4.3 | `test_simulation_build.jl` | `NetworkModel(ACPPowerModel; use_slacks=true)` → `ACPNetworkModel`; hydro systems `c_sys5_hy_uc`/`c_sys5_hy_ed` stay (D4) |
| 4.4 | `test_market_bid_cost.jl`, `test_import_export_cost.jl` (SPLIT) | keep only testsets that construct a `Simulation` (they exercise `update_cost_parameters.jl`); delete single-model testsets (POM covers them); `test_utils/mbc_simulation_utils.jl`, `iec_simulation_utils.jl` rename sweep |
| 4.5 | `test_simulation_execute.jl` | PF-in-the-loop simulation testsets use POM's evaluator API |
| 4.6 | `test_simulation_store.jl`, `test_simulation_results.jl`, `test_simulation_results_export.jl` | `PSI.read_dataframe` kept in `utils/file_utils.jl`; result key strings unchanged (`"ActivePowerVariable__ThermalStandard"`) |
| 4.7 | `test_simulation_partitions.jl`, `run_partitioned_simulation.jl` | Distributed workers need the same `--project=test`; check `process_simulation_partition_cli_args` still receives the template builders it expects |
| 4.8 | `test_recorder_events.jl`, `test_print.jl` (Simulation testsets only), `test_utils.jl` (output-dir testset only) | delete the model print testsets (POM/IOM own them) |

### Task 4.9: Full suite (serial)

- [ ] **Step 1:** `julia --project=test test/runtests.jl 2>&1 | tee .claude/plans/excision-full-suite.log; tail -20 .claude/plans/excision-full-suite.log`
  Expected: Aqua passes; every enabled file passes; summary shows 0 failed, 0 errored. Report the exact pass/fail counts in `excision-progress.md`.
- [ ] **Step 2:** Formatter; `git add -A test src .claude/plans; git commit -m "excision(4.9): full simulation test suite green"`.

**Phase 4 gate:** full suite green. User review.

---

## Phase 5 — Docs and guides (serial)

### Task 5.1: Docs prune and build

**Files:** `docs/make.jl`, `docs/make_tutorials.jl`, `docs/Project.toml`, `docs/src/**`

- [ ] Delete REMOVE pages and tutorials from spec §10 (`git rm`), including all of `docs/src/formulation_library/`.
- [ ] `docs/make.jl`: remove their entries from `pages`; drop the `StorageSystemsSimulations`, `HydroPowerSimulations`, `PowerFlows` InterLinks entries; add `InfrastructureOptimizationModels` and `PowerOperationsModels` entries only if their docs are published (else omit). `docs/Project.toml`: add IOM/POM path sources, drop PowerModels/Hydro/Storage.
- [ ] Rewrite `docs/src/index.md` and `docs/src/explanation/psi_structure.md` to describe PSI as the orchestration layer over IOM and POM (use the stack diagram from `.claude/CLAUDE.md`). Trim `explanation/feedforward.md` to sequence wiring; point to POM docs for constraint math. Trim `how_to/logging.md` log-group table to PSI groups.
- [ ] `tutorials/pcm_simulation.jl`: `ProblemTemplate` → `PowerOperationsProblemTemplate`, network renames, drop `using HydroPowerSimulations`/`StorageSystemsSimulations`.
- [ ] `api/PowerSimulations.md`, `api/internal.md`: `@autodocs` modules stay `[PowerSimulations]`; re-exported IOM/POM names are documented upstream — if Documenter reports `missing_docs` for re-exports, restrict the public page with `Filter = t -> parentmodule(t) === PowerSimulations` rather than `warnonly`.
- [ ] Build: `julia --project=docs -e 'using Pkg; Pkg.develop(PackageSpec(path=pwd())); Pkg.instantiate()'` then `julia --project=docs docs/make.jl`. Expected: clean finish.
- [ ] Commit: `git add -A docs; git commit -m "excision(5.1): prune docs to simulation scope; docs build"`.

### Task 5.2: Guides and memory

**Files:** `.claude/CLAUDE.md` (already rewritten on 2026-09-02 — verify against the final code and fix drift), `/home/jdlara/Sienna_work/psy6/.claude/CLAUDE.md` (stack table and glossary already updated — verify), `/home/jdlara/Sienna_work/psy6/.claude/sienna-psy6/SKILL.md` glossary line "PSI — … Does not exist here", `references/psy5-migration.md` first table row and "Simulation orchestration is out of scope — never port it" (now: "lives in PSI"), POM `.claude/CLAUDE.md:15` ("PSI (the old PowerSimulations.jl)").

- [ ] Update each statement so it reads: PSI is the psy6 simulation orchestration package depending on IOM and POM; formulation code still goes to POM, orchestration to PSI.
- [ ] Delete `scripts/excision/` (checker and smoke script) **or** move `smoke_simulation.jl` to `test/test_utils/` if Task 4.x adopted it; remove `.claude/plans/excision-checker-baseline.txt`. Keep `excision-progress.md` as the record.
- [ ] Commit in PSI: `git add -A .claude scripts; git commit -m "excision(5.2): guides reflect PSI scope"`. The workspace-level and POM guide edits are outside PSI's repo: leave them unstaged for the user (POM) or as plain file edits (workspace root is not a git repo).

---

## Phase 6 — End-to-end validation (serial)

### Task 6.1: Fresh-environment run

- [ ] Fresh clone check: `git stash -u` any leftovers; `rm -rf Manifest.toml test/Manifest.toml`; `julia --project=test -e 'using Pkg; Pkg.instantiate()'`; `julia --project=test scripts/excision/smoke_simulation.jl` (or its adopted location). Expected: `smoke OK`.
- [ ] HDF5 store variant: edit the smoke script call to `build!(sim; …)` followed by `execute!(sim; in_memory = false)` if the kwarg exists, else confirm the default is HDF5 and run once with `InMemorySimulationStore` via the documented kwarg. Both must produce identical `read_realized_variable` frames (`isapprox` on the numeric columns).
- [ ] Report: append final counts (files deleted, lines removed, tests passing, `grep -c EVENTS-EXCISION` fence count) to `excision-progress.md`; commit.

**Done when:** Phase 6 report is committed and the user has reviewed `git log main..HEAD`.

---

## Follow-ups recorded, not in scope

1. Events framework → POM, then unfence PSI (spec §8).
2. AGC / `AGCReserveDeployment` (POM `services_models/agc.jl` is not compiled; needs a state-reading `_get_ace_error` that belongs in PSI).
3. Service feedforwards (`attach_feedforward!(::ServiceModel, …)` errors in POM).
4. Switch `[sources]` path pins to git rev pins before opening the PR against `main`.
5. PowerAnalytics / PowerGraphics downstream re-validation against the new results API names.
