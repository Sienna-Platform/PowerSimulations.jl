# Run security-constrained (N-1) branch models

This guide shows how to build and solve security-constrained operation models in
`PowerSimulations.jl`, and documents the non-obvious pitfalls around network
reductions, PTDF/MODF consistency, and reported branch flows.

## What a security-constrained model does

A normal `PTDFPowerModel` enforces branch flow limits for the *base* topology.
A **security-constrained** model additionally enforces post-contingency flow
limits: for each registered contingency (an outaged branch), the flow on every
*monitored* branch after that outage must stay within its (post-contingency)
rating.

Two pieces make this work:

  - **PTDF** (`PNM.VirtualPTDF`) — maps nodal injections to base branch flows.
    Drives the in-model `PTDFBranchFlow` expression and the base
    `FlowRateConstraint`.
  - **MODF** (`PNM.VirtualMODF`) — line-outage distribution factors. For an
    outage `o` and monitored arc `a`, `modf[a, o]` gives the post-contingency
    sensitivity used to build `PostContingencyBranchFlow` and
    `PostContingencyFlowRateConstraint`.

Contingencies come from `PSY.Outage` supplemental attributes
(e.g. `GeometricDistributionForcedOutage`) whose `monitored_components` list the
branches watched after that outage. The behavior is engaged solely by choosing
the `SecurityConstrainedStaticBranch` formulation for a branch `DeviceModel` —
the MODF and outages are only consumed by that formulation.

## Minimal working example

```julia
using PowerSimulations
using PowerSystems
using PowerSystemCaseBuilder
import PowerNetworkMatrices: VirtualPTDF, VirtualMODF
const PSI = PowerSimulations

sys = build_system(PSITestSystems, "c_sys5")

# 1. Attach N-1 contingencies BEFORE building the MODF (see Gotcha 4).
all_branches = collect(get_components(ACTransmission, sys))
for line_name in ("1", "2", "3")
    line = get_component(ACTransmission, sys, line_name)
    add_supplemental_attribute!(
        sys,
        line,
        GeometricDistributionForcedOutage(;
            mean_time_to_recovery = 10,
            outage_transition_probability = 0.5,
            monitored_components = all_branches,
        ),
    )
end

# 2. Build PTDF and MODF with the SAME network reductions (see Gotcha 1).
ptdf = VirtualPTDF(sys)
modf = VirtualMODF(sys)

# 3. Network + template. The SC behavior is selected by the Line formulation.
template = ProblemTemplate(
    NetworkModel(PTDFPowerModel; PTDF_matrix = ptdf, MODF_matrix = modf),
)
set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template, PowerLoad, StaticPowerLoad)
set_device_model!(template, Line, SecurityConstrainedStaticBranch)
set_device_model!(template, Transformer2W, SecurityConstrainedStaticBranch)

model = DecisionModel(template, sys; optimizer = your_optimizer)
build!(model; output_dir = mktempdir())
solve!(model)
```

You can omit `MODF_matrix`; if the template uses
`SecurityConstrainedStaticBranch` and no MODF is supplied, PSI constructs one
from the system with `PNM.VirtualMODF`.

## Network reductions with security constraints

Large transmission systems are typically reduced before building PTDF/MODF:

```julia
import PowerNetworkMatrices: RadialReduction, DegreeTwoReduction

reductions = [RadialReduction(), DegreeTwoReduction()]
ptdf = VirtualPTDF(sys; network_reductions = reductions)
modf = VirtualMODF(sys; network_reductions = reductions)

template = ProblemTemplate(
    NetworkModel(
        PTDFPowerModel;
        PTDF_matrix = ptdf,
        MODF_matrix = modf,
        reduce_radial_branches = true,
        reduce_degree_two_branches = true,
    ),
)
```

A bus that something *monitored* is pinned to cannot be reduced away (PSI must
be able to express that monitored flow), so PSI computes a set of **irreducible
buses** from each SC `DeviceModel`'s outage `monitored_components` /
`associated_components` (and from branches carrying time-series ratings) and
forces them to survive the reduction. **You do not need to compute or pre-bake
these yourself** — PSI owns the contingency/monitored definitions and therefore
owns the consistency (see Gotcha 1).

!!! tip
    
    Monitoring more components keeps more buses irreducible, which means *less*
    reduction and a larger optimization problem. Monitor the set you actually
    need post-contingency coverage for, not "everything", unless you have a
    reason to.

## Gotchas

### Gotcha 1 — PTDF, MODF and the nodal balance must share one reduction

The optimization container's `ActivePowerBalance` (nodal balance) is
dimensioned on the **PTDF** reduction, while post-contingency expressions index
**MODF** columns. PowerNetworkMatrices' `VirtualPTDF` and `VirtualMODF` can
reduce the same system to *slightly different* bus sets even when given the
identical `network_reductions` argument. If they disagree, the model is
dimensionally inconsistent.

PSI reconciles this automatically: after building both matrices it rebuilds
both onto the union of buses either retains and re-derives the nodal-balance
reduction so all three agree. The `@warn` ("PTDF and MODF reduced to different
bus sets … Reconciling …") is informational. If a single reconciliation pass
cannot make them agree, the build is aborted with a `ConflictingInputsError`.

### Gotcha 2 — "Flow-expression dimension mismatch" is a real error, not noise

If you see:

```
Flow-expression dimension mismatch for branch/arc '…': the PTDF/MODF column
has N entries but the nodal-balance expression has M buses. The PTDF and MODF
matrices must be built with the same network reduction …
```

a PTDF/MODF column is being indexed against a nodal-balance vector of a
different bus dimension (formerly an out-of-bounds read under `@inbounds`, now
a catchable error raised before the unsafe access). Gotcha 1's reconciliation
prevents this for PSI-constructed matrices; with hand-built matrices, rebuild
the MODF with the same reduction as the PTDF (and the monitored-component
irreducible buses).

### Gotcha 3 — Reported `PTDFBranchFlow` orientation under reductions

When degree-two/radial reductions merge several branches into one equivalent,
each member branch's `PTDFBranchFlow` is assembled from the *representative*
arc's PTDF column. For a member whose native `from → to` arc is opposite the
merged-path direction, the recorded flow was historically **sign-flipped**
relative to its own orientation (and relative to
`PowerFlowBranchActivePowerFromTo`).

This is now corrected: each member's reported `PTDFBranchFlow` matches its
native `from → to` convention. The same per-member sign is applied to the
`InterfaceTotalFlow` and `AreaInterchange` direction multipliers, so interface
and interchange totals are unchanged — only the per-branch reported value is
fixed.

Practical check: with `power_flow_evaluation = DCPowerFlow()`, scatter
`PTDFBranchFlow` (x) vs `PowerFlowBranchActivePowerFromTo` (y). They should
collapse onto `y = x`; an anti-diagonal (`y ≈ −x`) cluster is the symptom of
this bug on an unpatched PSI.

### Gotcha 4 — Attach outages before building the MODF

`VirtualMODF` registers contingencies from the system's outage supplemental
attributes at construction time. Outages added *after* the MODF is built are
not registered, and PSI drops any SC-`DeviceModel` outage that is not on the
MODF (with a warning) — it contributes no post-contingency constraints. Always
`add_supplemental_attribute!` the outages, then build the MODF (or let PSI
build it).

### Gotcha 5 — `monitored_components` only pins buses under an SC formulation

A `PSY.Outage` attached to a device whose branch formulation is *not*
security-constrained is ignored: it neither produces post-contingency
constraints nor pins irreducible buses. The scoping is per-`DeviceModel` — only
outages on a model whose formulation is `<: AbstractSecurityConstrainedStaticBranch`
are consumed. If your contingencies "do nothing", check that the relevant
branch type's formulation is `SecurityConstrainedStaticBranch`.

### Gotcha 6 — Providing your own PTDF/MODF

If you pass `PTDF_matrix` / `MODF_matrix`, build **both** with the same
`network_reductions`. PSI may still recalculate them when monitored/outage
components force irreducible buses (you will see a "Provided … Matrix is being
ignored … Recalculating" warning) and will reconcile them per Gotcha 1. This is
expected and correct; it is not an error.

## Verifying a security-constrained run

  - `build!` returns `ModelBuildStatus.BUILT` (no `ConflictingInputsError`, no
    dimension-mismatch error, no crash).
  - `PostContingencyFlowRateConstraint` exists for the monitored branches.
  - With `DCPowerFlow()` power-flow evaluation, `PTDFBranchFlow` and
    `PowerFlowBranchActivePowerFromTo` agree (`y = x`, no anti-diagonal).
  - PTDF, MODF and the nodal-balance bus dimensions are equal after `build!`
    (the invariant Gotcha 1/2 enforce).
