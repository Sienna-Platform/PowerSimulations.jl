# [Branch Rating Limits](@id branch_rating_limits)

This page explains how PowerSimulations derives a flow **rating** for an AC
transmission element and how that rating is turned into optimization
constraints. It covers every reduction type produced by
`PowerNetworkMatrices` (PNM), the formulations that consume the rating, the
difference between linear and nonlinear network models, the parallel-branch
aggregation attribute and its defaults, and the branch-rating time series.

For the per-formulation constraint algebra (variable names, slacks, objective
terms) see the [`PowerSystems.Branch` Formulations](@ref) page in the
Formulation Library. This page is the conceptual companion: *where the number
on the right-hand side comes from*.

## The single source of truth

All branch flow ratings flow through one type-aware entry point.
`branch_rating(entry, device_model)` returns the scalar rating for a reduction
`entry`, and `min_max_flow_limits(entry, device_model)` wraps it into the
symmetric pair `(min = -rating, max = rating)` used by linear constraints. Every
constraint builder — linear or nonlinear, static or time series — resolves its
rating through this path so the same physical element gets the same limit
regardless of the network model.

`branch_rating` is *type-aware*: it dispatches on the kind of reduction entry
PNM produced for the element and delegates the aggregation policy to PNM. It
does **not** reimplement per-type rating math; extend
`PowerNetworkMatrices.get_equivalent_rating` instead if a new element type needs
support.

## PNM rating aggregation by reduction type

Network reduction can collapse several physical branches into one equivalent
element. The rating of that equivalent element depends on its topology:

| Reduction entry                                                          | Rating policy                                                                                                                                                      |
|:------------------------------------------------------------------------ |:------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Single line / two-winding transformer (`PSY.ACTransmission`)             | `PSY.get_rating`                                                                                                                                                   |
| `PSY.GenericArcImpedance`                                                | `PSY.get_max_flow` (used as a rating proxy)                                                                                                                        |
| Three-winding transformer winding (`PNM.ThreeWindingTransformerWinding`) | The winding-specific rating (primary/secondary/tertiary); falls back to the parent transformer rating when the winding rating is `0.0`                             |
| Series chain (`PNM.BranchesSeries`)                                      | **Weakest link**: the minimum equivalent rating across the chain members. A parallel block embedded in the chain contributes its single-element-contingency rating |
| Parallel group (`PNM.BranchesParallel`)                                  | Attribute-driven — see [Parallel-branch aggregation](@ref parallel_branch_aggregation)                                                                             |
| Mixed parallel group (`PNM.MixedBranchesParallel`)                       | Always the plain sum of member ratings (`sum_of_max`); the attribute is ignored                                                                                    |

A **post-contingency** (emergency) rating uses the parallel structure with
`PSY.rating_b` in place of `PSY.rating`:
`PowerNetworkMatrices.get_equivalent_emergency_rating` mirrors the normal
aggregation type-for-type, and for a single `ACTransmission` it returns
`PSY.get_rating_b` and falls back to `PSY.get_rating` when no `rating_b` is
defined. Three-winding windings have no `rating_b` in PowerSystems and fall
back to their normal winding rating.

## [Parallel-branch aggregation](@id parallel_branch_aggregation)

When PNM produces a homogeneous `PNM.BranchesParallel` group, the way the
individual circuit ratings combine into one limit is a modeling choice carried
on the `DeviceModel` as an attribute:

  - **Attribute key:** `"parallel_branch_max_rating_method"` (the constant
    `PARALLEL_BRANCH_MAX_RATING_KEY`).
  - **Default:** `"single_element_contingency"`.

Valid values:

| Value                                    | Aggregated rating                            | Meaning                                                                                                                                           |
|:---------------------------------------- |:-------------------------------------------- |:------------------------------------------------------------------------------------------------------------------------------------------------- |
| `"single_element_contingency"` (default) | ``\sum_i S_i - \max_i S_i``                  | N-1 surviving capacity after the largest-rated circuit trips. A single-circuit group has zero capacity under this policy                          |
| `"sum_of_max"`                           | ``\sum_i S_i``                               | Each circuit independently loadable to its own thermal limit (least conservative)                                                                 |
| `"impedance_averaged"`                   | ``\sum_i f_i S_i,\; f_i = b_i / \sum_k b_k`` | Susceptance-weighted average, reflecting how DC flow physically splits across the group. Errors if total series susceptance is zero or non-finite |

`PNM.MixedBranchesParallel` (a parallel group whose members carry different
`DeviceModel` preferences) always uses `sum_of_max`, because there is no
defensible way to pick one member's attribute for the whole group.

Security-constrained branch formulations carry one additional default
attribute, `"include_planned_outages" => false`, alongside the same
`"single_element_contingency"` parallel default.

## How the rating enters the optimization

### Linear network models

This covers the PTDF family (`PTDFPowerModel`, `AreaPTDFPowerModel`, and other
`AbstractPTDFModel`s) and the DC power model (`DCPPowerModel` and other
`PM.AbstractActivePowerModel`s). `CopperPlatePowerModel` and
`AreaBalancePowerModel` enforce no per-branch flow limits.

The active-power flow is bounded symmetrically by the aggregated rating:

```math
-R \;\le\; f_{b,t} \;\le\; R
```

applied through `FlowRateConstraint` (PTDF and DC) or, for
`StaticBranchBounds`, directly as variable bounds on the flow variable.
`MonitoredLine` is the exception — it carries explicit, possibly asymmetric
`flow_limits` (see [MonitoredLine](@ref monitored_line_rating)).
`PhaseShiftingTransformer` under `PhaseAngleControl` additionally bounds the
phase-shifter angle to ``[-\pi/2,\, \pi/2]``.

### Nonlinear network models

For full AC models (any `PM.AbstractPowerModel` that is not an active-power-only
model) the apparent-power limit is enforced per flow direction:

```math
(P^{ft}_{b,t})^2 + (Q^{ft}_{b,t})^2 \;\le\; R^2
\qquad
(P^{tf}_{b,t})^2 + (Q^{tf}_{b,t})^2 \;\le\; R^2
```

through `FlowRateConstraintFromTo` and `FlowRateConstraintToFrom`. The rating
``R`` is the **same** type-aware `branch_rating` value used by the linear path;
only its application differs (squared, on apparent power).

### Formulation behavior summary

| Formulation                                          | Rating limit behavior                                        |
|:---------------------------------------------------- |:------------------------------------------------------------ |
| `StaticBranch`                                       | Flow rate limits enforced (and apparent-power limits for AC) |
| `StaticBranchBounds`                                 | Rating applied as bounds on the flow variable                |
| `StaticBranchUnbounded`                              | No flow limits enforced                                      |
| `AbstractSecurityConstrainedStaticBranch` (SCUC/N-1) | Flow rate limits enforced; pre- and post-contingency ratings |
| `PhaseAngleControl`                                  | Flow rate limits plus phase-angle limits                     |

## Branch rating time series

A branch may carry a time series that makes its rating vary per time step (for
example dynamic line ratings). The time series is **normalized**: its values
are a per-unit multiplier of a static rating, and the static rating is supplied
as the parameter *multiplier*. The right-hand side that the solver sees is
therefore:

  - Linear models: `parameter[t] * multiplier`
  - Nonlinear models: `(parameter[t] * multiplier)^2`

so the effective limit at time `t` is `ts(t) · R` (and `(ts(t) · R)^2` for
apparent power). The simulation update policy refreshes only the normalized
parameter value each window; the multiplier is fixed at build time.

The multiplier is resolved with the **same type-aware aggregation as the static
path**, so series chains, three-winding windings and single elements all scale
the correct equivalent rating. There is one deliberate exception:

  - **Parallel groups:** a rating time series attached to one member of a
    parallel group cannot be distributed across the group's other circuits.
    The multiplier is the group's combined rating (`sum_of_max`) **regardless
    of `"parallel_branch_max_rating_method"`**, and a warning is emitted.
  - **Post-contingency** (`PostContingencyBranchRatingTimeSeriesParameter`):
    the multiplier uses the emergency (`rating_b`) aggregation — the summed
    emergency rating for parallel groups, and the type-aware emergency rating
    otherwise.

Support and validation:

  - Branch rating time series are only supported with `StaticBranch` and
    `AbstractSecurityConstrainedStaticBranch`. Any other formulation —
    including `StaticBranchUnbounded`, which enforces no flow limits — raises a
    `ConflictingInputsError` rather than silently ignoring the time series.
  - At build a sanity check warns if the normalized series scaled by the
    multiplier ever falls below the device's static rating (i.e. the series is
    expected to represent at-or-above-nominal capacity).

## [MonitoredLine](@id monitored_line_rating)

`PSY.MonitoredLine` does not use the symmetric `branch_rating` path because it
carries explicit `flow_limits` that can be tighter than, and asymmetric to, its
rating:

  - General rate limit: ``\min(\text{rating},\ \text{flow\_limits.from\_to},\ \text{flow\_limits.to\_from})``,
    applied symmetrically. A warning is emitted when the from-to and to-from
    limits differ (the minimum is then used).
  - Directional limits (`FlowLimitFromToConstraint` / `FlowLimitToFromConstraint`)
    use `flow_limits.from_to` and `flow_limits.to_from` respectively.

## Defaults at a glance

| Setting                                                             | Default                                                   |
|:------------------------------------------------------------------- |:--------------------------------------------------------- |
| `"parallel_branch_max_rating_method"` (`AbstractBranchFormulation`) | `"single_element_contingency"`                            |
| Security-constrained extra attribute                                | `"include_planned_outages" => false`                      |
| `MixedBranchesParallel` aggregation                                 | `sum_of_max` (attribute ignored)                          |
| Single branch / transformer rating                                  | `PSY.get_rating`                                          |
| Series chain rating                                                 | weakest link (minimum member rating)                      |
| Three-winding winding rating                                        | winding rating, falling back to parent transformer rating |
| Post-contingency single-branch rating                               | `PSY.get_rating_b`, falling back to `PSY.get_rating`      |
| Time-series multiplier (parallel)                                   | `sum_of_max` regardless of the attribute (with warning)   |
| Branch rating time series formulation support                       | `StaticBranch`, `AbstractSecurityConstrainedStaticBranch` |
