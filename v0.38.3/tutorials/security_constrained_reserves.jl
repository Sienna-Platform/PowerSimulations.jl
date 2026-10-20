#src # EXECUTE = TRUE
# # [G-1 Security-Constrained Reserves](@id sc_reserves)
#
# ## Introduction
#
# A conventional reserve model (`RangeReserve`, `RampReserve`) procures a *quantity* of
# reserve. It never checks that the reserve can be delivered: nothing in the model says the
# network can carry the response from the units holding the reserve to the bus that lost its
# generation. The security-constrained reserve formulations close that gap. They co-optimize
# the reserve award with the **deliverability** of that reserve after a generator outage
# (G-1), on a user-chosen subset of **monitored** components.
#
# For each contingency the model adds a post-contingency world: every contributing unit gets
# a deployment variable, the deployment must exactly replace the lost generation, and the
# resulting flow on each monitored component must respect its emergency rating. Because both
# worlds share the pre-contingency dispatch variables, the post-contingency limits push back
# on the base-case schedule.
#
# !!! note
#
#     This tutorial covers the *service* formulations [`SecurityConstrainedContingencyReserve`](@ref)
#     and [`SecurityConstrainedRampReserve`](@ref), which model **generator** outages and use
#     the base-topology PTDF to redistribute the response. They are distinct from the
#     `SecurityConstrainedStaticBranch` *device* formulation, which models **branch** outages
#     with line-outage distribution factors (MODF) — see
#     [Run security-constrained (N-1) branch models](@ref).

# ## Worked example: `PTDFPowerModel` with three monitored lines
#
# ### Load packages

using PowerSimulations
using PowerSystems
using PowerSystemCaseBuilder
using PowerNetworkMatrices
using HydroPowerSimulations
using DataFrames
using HiGHS
using JuMP
using Dates
import InfrastructureSystems as IS

# ### Data
#
# !!! note
#
#     [PowerSystemCaseBuilder.jl](https://github.com/Sienna-Platform/PowerSystemCaseBuilder.jl)
#     is a helper library that makes it easier to reproduce examples in the documentation and
#     tutorials. Normally you would pass your local files to create the system data instead
#     of calling `build_system`. For more details visit
#     [PowerSystemCaseBuilder Documentation](https://sienna-platform.github.io/PowerSystems.jl/stable/how_to/powersystembuilder/)
#
# The only data preparation this page needs is a system and a forecast window. The
# `transform_single_time_series!` call sets the horizon of the problem to four hours.

sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
transform_single_time_series!(sys, Hour(4), Hour(1))

# ### Declare the contingency
#
# A contingency is a `PSY.Outage` supplemental attribute, and it carries two pieces of
# information:
#
#   - **what is lost** — the attribute is attached to the generator that goes out;
#   - **what is watched** — the attribute's `monitored_components` list the components whose
#     post-contingency flow is constrained.
#
# The same attribute is then attached to the reserve `Service`. That second attachment is the
# *only* mechanism that opts a service into responding to the contingency: a service responds
# to exactly the outages attached to it.

outaged_unit = get_component(ThermalStandard, sys, "216_STEAM_1")
reserve = get_component(VariableReserve{ReserveUp}, sys, "Reg_Up")
monitored_lines = [get_component(Line, sys, name) for name in ("AB2", "AB3", "CB-1")]

outage = FixedForcedOutage(;
    outage_status = 1.0,
    monitored_components = monitored_lines,
)

add_supplemental_attribute!(sys, outaged_unit, outage)   # what is lost
add_supplemental_attribute!(sys, reserve, outage)        # who responds

# !!! tip
#
#     PowerSimulations keys off the *presence* of the attribute on the generator and on the
#     service, not off the numeric `outage_status`. A `FixedForcedOutage` with
#     `outage_status = 1.0` is the least ceremony needed to declare that a contingency exists;
#     use `GeometricDistributionForcedOutage` only when you actually have the stochastic
#     parameters and something else consumes them.

# ### Problem template
#
# Two entries do the work. To constrain monitored *branches* the network model must be an
# `AbstractPTDFModel` (`PTDFPowerModel` or `AreaPTDFPowerModel`), because the
# post-contingency flow expression is built from PTDF columns. And every monitored component
# type must have its own `DeviceModel` in the template — a monitored `Line` with no `Line`
# device model is dropped with a warning, and no post-contingency flow constraint is built
# for it.

template = ProblemTemplate(NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)))

set_device_model!(template, ThermalStandard, ThermalDispatchNoMin)
set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
set_device_model!(template, RenewableNonDispatch, FixedOutput)
set_device_model!(template, PowerLoad, StaticPowerLoad)
set_device_model!(template, HydroDispatch, HydroDispatchRunOfRiver)
set_device_model!(template, Line, StaticBranch)
set_device_model!(template, Transformer2W, StaticBranch)
set_device_model!(template, TapTransformer, StaticBranch)

# The service model names the service explicitly, so only `Reg_Up` is modeled as
# security-constrained. `use_slacks = true` adds priced slacks on the post-contingency flow
# limits (and on the pre-contingency reserve requirement), which keeps an over-constrained
# case solvable and reports the violation instead of hiding it in an infeasibility.

set_service_model!(
    template,
    ServiceModel(
        VariableReserve{ReserveUp},
        SecurityConstrainedContingencyReserve,
        "Reg_Up";
        use_slacks = true,
    ),
)

# ### Build and solve one `DecisionModel`

solver = optimizer_with_attributes(HiGHS.Optimizer, "log_to_console" => false)

model = DecisionModel(
    template,
    sys;
    resolution = Hour(1),
    optimizer = solver,
    store_variable_names = true,
)

build_time = @elapsed build!(model; output_dir = mktempdir(; cleanup = true))

solve_time = @elapsed solve!(model)

(build_time_s = build_time, solve_time_s = solve_time)

# Adding [`SecurityConstrainedContingencyReserve`](@ref) on `Reg_Up`, with one outage and
# three monitored lines, is what grows the PTDF problem to the size below. `get_jump_model`
# reaches the underlying `JuMP.Model`, so the counts come straight off the model that was just
# built and solved.

jump_model = PowerSimulations.get_jump_model(model)

(
    variables = JuMP.num_variables(jump_model),
    constraints = JuMP.num_constraints(
        jump_model;
        count_variable_in_set_constraints = true,
    ),
    objective = JuMP.objective_value(jump_model),
)

# ## What the model builds
#
# Everything below is indexed by contingency first. Throughout, ``c`` is a contingency
# (identified by the outage's UUID), ``g`` a contributing generator, ``g^*`` the generator
# outaged by ``c``, ``\ell`` a monitored component, ``a`` an area, ``b`` a bus and ``t`` a
# time step.
#
# ### Variables
#
# | Type | Meaning | Indexed by |
# |---|---|---|
# | `PostContingencyActivePowerReserveDeploymentVariable` (``\Delta rsv_{c,g,t}``) | Reserve the unit actually deploys after the outage. Non-negative, upper bound `get_max_active_power(device)`, and pinned to zero for the outaged unit itself | (outage, contributing device, ``t``) |
# | `PostContingencyFlowActivePowerSlackUpperBound` / `...LowerBound` (``s^{f,\text{ub}}``, ``s^{f,\text{lb}}``) | Non-negative relaxations of the two post-contingency flow inequalities, priced at `CONSTRAINT_VIOLATION_SLACK_COST`. Only with `use_slacks = true` | (outage, monitored component, ``t``) — sparse |
# | `PostContingencyAreaInterchangeFlowDeviationVariable` (``\Delta f_{c,\ell,t}``) | Change in an `AreaInterchange` flow that carries the response between areas. Free (either direction). `AreaBalancePowerModel` only | (outage, **modeled** tie, ``t``) |
#
# The pre-contingency `ActivePowerReserveVariable` (``rsv_{g,t}``) is the ordinary reserve
# award. It is built when the service model maps a `RequirementTimeSeriesParameter` to a
# requirement series the service actually carries — the default mapping is the series named
# `"requirement"`.
#
# ### Expressions
#
# | Type | Meaning | Indexed by | Network model |
# |---|---|---|---|
# | `PostContingencyActivePowerBalance` | Deployed reserve minus the outaged unit's output, system-wide | (outage, ``t``) | PTDF, CopperPlate |
# | `PostContingencyNodalActivePowerDeployment` | The same net injection change, per bus. Restricted to buses hosting a contributing device or an outaged unit; every other bus's entry is identically zero | (outage, bus, ``t``) | PTDF |
# | `PostContingencyAreaActivePowerDeployment` | The same net injection change, per area | (outage, area, ``t``) | AreaBalance |
# | `PostContingencyBranchFlow` | Post-contingency flow on a monitored branch | (outage, branch, ``t``) — sparse | PTDF |
# | `PostContingencyAreaInterchangeFlow` | Post-contingency flow on a monitored tie | (outage, tie, ``t``) — sparse | AreaBalance |
# | `PostContingencyActivePowerGeneration` | ``p_{g,t} + \Delta rsv_{c,g,t}``, used only when the service has no requirement series | (outage, device, ``t``) | PTDF, CopperPlate, AreaBalance |
#
# The nodal deployment expression is the net injection change the contingency causes:
#
# ```math
# \text{dep}_{c,b,t} = \sum_{g \in b,\, g \neq g^*} \Delta rsv_{c,g,t} - p_{g^*,t} \mathbb{1}_{b = b(g^*)}
# ```
#
# and the monitored flow is the pre-contingency flow displaced by that injection change,
# through the base-topology PTDF:
#
# ```math
# f^\text{post}_{c,\ell,t} = f_{\ell,t} + \sum_{b} \text{PTDF}_{\ell,b}\, \text{dep}_{c,b,t}
# ```
#
# ### Constraints
#
# | Type | Meaning | Indexed by |
# |---|---|---|
# | `PostContingencyGenerationBalanceConstraint` | The post-contingency balance closes: deployment exactly replaces the lost generation | (outage, ``t``) |
# | `PostContingencyActivePowerReserveDeploymentVariableLimitsConstraint` | ``\Delta rsv_{c,g,t} \le rsv_{g,t}``: a unit can only deploy what it was awarded | (outage, device, ``t``) — sparse |
# | `PostContingencyActivePowerGenerationLimitsConstraint` | ``p_{g,t} + \Delta rsv_{c,g,t} \le P^\text{max}_g`` (and ``= 0`` for the outaged unit). Used instead of the row above when there is no requirement series | (outage, device, ``t``) |
# | `PostContingencyFlowRateConstraint` | Monitored flow inside the component's emergency rating; two containers, meta `"<service>_ub"` and `"<service>_lb"` | (outage, monitored component, ``t``) — sparse |
# | `PostContingencyCopperPlateBalanceConstraint` | Per-area post-contingency balance, closed through ``\Delta f`` on every modeled tie touching the area. `AreaBalancePowerModel` only | (outage, area, ``t``) |
#
# The balance is what makes the requirement bite: the deployment must sum to the outaged
# unit's pre-contingency output.
#
# ```math
# \sum_{g \neq g^*} \Delta rsv_{c,g,t} - p_{g^*,t} = 0, \quad \forall c,\ \forall t
# ```
#
# The monitored-flow limits use the **emergency** rating of the monitored component,
# symmetric around zero, with the optional slacks:
#
# ```math
# \begin{align*}
# & f^\text{post}_{c,\ell,t} - s^{f,\text{ub}}_{c,\ell,t} \le R^\text{emg}_\ell \\
# & f^\text{post}_{c,\ell,t} + s^{f,\text{lb}}_{c,\ell,t} \ge -R^\text{emg}_\ell
# \end{align*}
# ```
#
# ### The two formulations
#
# Both build the identical post-contingency stack above. They differ only in the
# pre-contingency reserve model they wrap, and in whether the reserve requirement time series
# is optional:
#
# | | [`SecurityConstrainedContingencyReserve`](@ref) | [`SecurityConstrainedRampReserve`](@ref) |
# |---|---|---|
# | Pre-contingency constraints | `RequirementConstraint`, `ParticipationFractionConstraint`, reserve objective term — the `RangeReserve` stack | The same, plus `RampConstraint` — the `RampReserve` stack |
# | Requirement time series | Optional. Without one, no pre-contingency reserve variable is built and deployment is bounded by generator capacity through `PostContingencyActivePowerGeneration` | Always required |
#
# See [Service Formulations](@ref service_formulations) for the pre-contingency math of
# `RangeReserve` and `RampReserve`.

# ## [Reading the results](@id sc_reserves_results)
#
# The deployment variable is three-dimensional, so its long-format table has a `name` column
# holding the **outage UUID as a string** and a `name2` column holding the contributing
# device.

results = OptimizationProblemResults(model)
variables = read_variables(results)
expressions = read_expressions(results)

deployment =
    variables["PostContingencyActivePowerReserveDeploymentVariable__VariableReserve__ReserveUp__Reg_Up"]

outage_id = string(IS.get_uuid(outage))

# The defining identity of a G-1 requirement is that the total deployment equals the outaged
# unit's pre-contingency output, time step by time step. Summing the deployment over
# contributing devices and comparing against the outaged unit's `ActivePowerVariable` is the
# first thing to check on any new case.

function total_deployment(deployment::DataFrame, outage_id::String)
    rows = filter(row -> row["name"] == outage_id, deployment)
    totals = combine(groupby(rows, :DateTime), :value => sum => :total)
    return sort!(totals, :DateTime)
end

function device_series(df::DataFrame, device_name::String)
    rows = filter(row -> row["name"] == device_name, df)
    return sort(rows, :DateTime)[!, "value"]
end

outaged_output =
    device_series(variables["ActivePowerVariable__ThermalStandard"], "216_STEAM_1")

deployment_sum = total_deployment(deployment, outage_id)[!, :total]

(pre_contingency_output = outaged_output, total_deployment = deployment_sum)

# `216_STEAM_1` sits in area 2. The block above lists what it actually dispatches over the
# horizon next to the reserve deployment summed over every contributing device. The balance
# constraint forces the two to agree, so the gap between them should be zero up to solver
# tolerance:

maximum(abs.(deployment_sum - outaged_output))

# !!! note
#
#     `read_variables` converts to natural units (MW). Values read directly off the JuMP
#     model or the optimization container are in the system per-unit base. Do not mix the two
#     in one comparison.
#
# The monitored flows and the slacks are the second thing to check. Both are sparse
# containers keyed by `(outage, monitored component, t)`; in the results tables the outage
# and the component are flattened into one column name, `"<outage_uuid>__<component>"`. Use
# the list helpers to discover the exact keys rather than guessing them:

list_expression_names(results)

list_variable_names(results)

# Reading the flow and the two slacks off those keys, for every one of the 12 monitored rows
# (1 outage × 3 lines × 4 steps), is the mechanical version of that check.

flows = expressions["PostContingencyBranchFlow__VariableReserve__ReserveUp__Reg_Up"]

slack_ub =
    variables["PostContingencyFlowActivePowerSlackUpperBound__VariableReserve__ReserveUp__Reg_Up"]
slack_lb =
    variables["PostContingencyFlowActivePowerSlackLowerBound__VariableReserve__ReserveUp__Reg_Up"]

max(maximum(abs, slack_ub[!, :value]), maximum(abs, slack_lb[!, :value]))

# Zero on both means every monitored row respected its post-contingency limit without help
# from the priced relaxation — a nonzero value would mean the limit was violated and the
# violation was *priced* rather than enforced, and should be read before trusting a solved
# model. The tightest margin over the horizon, by line, uses the same emergency-vs-normal
# rating fallback `PowerNetworkMatrices.get_equivalent_emergency_rating` applies:

function tightest_margin(flows::DataFrame, outage_id::String, line::Line, sys::System)
    key = string(outage_id, "__", get_name(line))
    rows = filter(row -> row["name"] == key, flows)
    rating_mw = something(get_rating_b(line), get_rating(line)) * get_base_power(sys)
    return minimum(rating_mw .- abs.(rows[!, :value]))
end

DataFrame(;
    line = get_name.(monitored_lines),
    tightest_margin_mw = [
        tightest_margin(flows, outage_id, line, sys) for line in monitored_lines
    ],
)

# ## The zonal variant: `AreaBalancePowerModel` and ``\Delta f``
#
# Under `AreaBalancePowerModel` there is no nodal network, so post-contingency deliverability
# is expressed between **areas** instead of across branches. The formulation:
#
#  1. builds a free flow-deviation variable ``\Delta f_{c,\ell,t}``
#     (`PostContingencyAreaInterchangeFlowDeviationVariable`) on every tie in the template's
#     `AreaInterchange` `DeviceModel` set — not only the monitored ones, because any modeled
#     tie could carry part of the response. That set is exactly the one the pre-contingency
#     `FlowActivePowerVariable` is built over, so a tie the `DeviceModel` excludes (through
#     its `filter_function`, its subsystem, or availability) has no pre-contingency flow for
#     ``\Delta f`` to deviate from and gets no ``\Delta f`` term at all;
#  2. closes the balance per area rather than system-wide
#     (`PostContingencyCopperPlateBalanceConstraint`):
#
# ```math
# \sum_{g \in a,\, g \neq g^*} \Delta rsv_{c,g,t} - p_{g^*,t}\mathbb{1}_{g^* \in a}
# - \sum_{\ell:\, \text{from}(\ell) = a} \Delta f_{c,\ell,t}
# + \sum_{\ell:\, \text{to}(\ell) = a} \Delta f_{c,\ell,t} = 0 ;
# ```
#
#     where both sums run over the modeled ties only;
#  3. bounds the monitored ties' post-contingency flow, ``f^\text{post}_{c,\ell,t} =
#     f_{\ell,t} + \Delta f_{c,\ell,t}``, against `get_flow_limits(tie)`.
#
# If the template registers no `AreaInterchange` `DeviceModel` at all, no ``\Delta f`` is
# created, a warning fires once per service, and the balance above loses its tie terms
# entirely: each area must then cover its own outages from its own contributing devices. That
# is the correct physics for a template that models no inter-area transfer, but it also means
# **no cross-area deliverability is being tested** — if that is not what you intended, add
# `set_device_model!(template, AreaInterchange, StaticBranch)`.
#
# The consequence is the useful part: reserve held in one area can serve an outage in
# another, but only as far as the tie limits allow. Take a reserve product whose contributing
# devices all sit in area 1, responding to an outage in area 2: as long as the outaged unit's
# pre-contingency output is genuinely nonzero (see [Limitations](@ref sc_reserves_limitations)),
# the balance in area 2 has no local contributing device to draw on, so it must close entirely
# through ``\Delta f`` on the ties from area 1 — the net transfer into area 2 equals the
# outaged unit's output, and whichever monitored ties carry that transfer are exactly the ones
# whose post-contingency limits are worth checking for a binding case.
#
# !!! warning
#
#     This path requires `AreaInterchange` components in the system, plus an
#     `AreaInterchange` `DeviceModel` in the template. RTS-GMLC ships none, which is why the
#     worked example above uses the PTDF path. A **monitored** tie must be available, inside
#     the network model's scope, *and* inside that `DeviceModel`'s set; a monitored tie that
#     fails any of the three is rejected at template validation with a
#     `ConflictingInputsError` (reported as `ModelBuildStatus.FAILED` by `build!`), naming
#     which condition it failed.

# ## Use cases
#
#   - **G-1 reserve deliverability screening.** Confirm that the reserve a market or study
#     procures can actually reach the loss, given the network. The base-case schedule adjusts
#     to leave the needed headroom on the monitored paths.
#   - **Locational reserve questions.** Ask whether reserve held in a particular zone or area
#     is deliverable to an outage elsewhere. Under `AreaBalancePowerModel` the ``\Delta f``
#     terms report how much of the response crossed each tie.
#   - **Tractability control.** The monitored set is the tuning knob. Start from the handful
#     of paths that historically bind post-contingency, and grow it only where the flow
#     margins or the slacks say you need to.

# ## [Limitations](@id sc_reserves_limitations)
#
#   - **Up-reserves only.** Template validation rejects a security-constrained
#     `ServiceModel` for any `Reserve` direction other than `ReserveUp` with a
#     `ConflictingInputsError`; `build!` reports it as `ModelBuildStatus.FAILED`. Deployment
#     is modeled as a strictly non-negative response to a generation shortfall.
#   - **Generator outages only.** The lost component set is derived from `PSY.Generator`
#     components carrying the outage attribute, and the post-contingency flow uses the
#     base-topology PTDF. No line-outage sensitivity is involved; branch contingencies are
#     the separate `SecurityConstrainedStaticBranch` device formulation.
#   - **This is targeted screening, not full N-1.** The contingency set is exactly the
#     outages you attach to the service, and the monitored set is exactly each outage's
#     `monitored_components`. There is no implicit "monitor everything": an outage with an
#     empty `monitored_components` is skipped entirely, with a warning.
#   - **Monitored types must be modeled.** For a service outage only `ACTransmission` and
#     `AreaInterchange` components are admissible monitors, and the concrete type needs a
#     `DeviceModel` in the template. Anything else is warned about and skipped.
#   - **A monitored `AreaInterchange` needs `AreaBalancePowerModel`.** Under a PTDF network
#     model it is dropped with a warning, because no post-contingency flow expression exists
#     for it there. Under `AreaBalancePowerModel` it must also be in the template's
#     `AreaInterchange` `DeviceModel` set, or template validation rejects it with a
#     `ConflictingInputsError`.
#   - **``\Delta f`` follows the `AreaInterchange` `DeviceModel`, not the network model.** A
#     tie excluded by that `DeviceModel` (`filter_function`, subsystem, or availability) has no
#     pre-contingency flow variable, so it gets no ``\Delta f`` term and cannot carry any
#     post-contingency transfer. With no `AreaInterchange` `DeviceModel` at all, no
#     ``\Delta f`` is built (one warning per service) and the per-area balance reduces to
#     in-area coverage — correct for a template with no inter-area transfer, but it tests no
#     cross-area deliverability whatsoever.
#   - **Under `CopperPlatePowerModel` the monitors are inert.** Only the per-outage balance
#     and the deployment limits are built; there is no flow representation to constrain, so
#     the model checks that the response *exists*, never that it is deliverable.
#   - **Slacks price violations, they do not forbid them.** With `use_slacks = true` a
#     violated post-contingency limit costs `CONSTRAINT_VIOLATION_SLACK_COST` and the model
#     still solves. Read the slack variables.
#   - **`SecurityConstrainedRampReserve` always needs a requirement time series.** Only the
#     contingency variant tolerates a service without one.
#   - **The outaged unit must actually be dispatched.** The G-1 requirement is
#     ``\sum \Delta rsv = p_{g^*,t}``, so if the optimizer leaves the outaged unit at zero the
#     requirement is zero and every post-contingency constraint holds trivially. This is easy
#     to hit with `ThermalDispatchNoMin`, where same-area units can absorb the unit's output
#     for free. Before reading anything into a passing case, check that the outaged unit's
#     pre-contingency output is nonzero. In a test, you can force this by setting a lower
#     bound on the unit's `ActivePowerVariable` between `build!` and `solve!` — a diagnostic
#     lever for making a test case bite, not a modeling pattern. In a study, get the nonzero
#     pre-contingency output from the data and the formulation (for example a commitment
#     formulation with a minimum power level), not from a hand-set bound.
#   - **Size scales multiplicatively.** Deployment variables scale as
#     outages × contributing devices × time steps (and ``\Delta f`` as
#     outages × `DeviceModel`-set ties × time steps), while the monitored flow rows scale as
#     outages × monitored components × time steps, each contributing two constraints plus two
#     slacks. Reserve products with many contributing devices, and long horizons, dominate.
