# `PowerSystems.RenewableGen` Formulations

Renewable generation formulations define the optimization models that describe renewable units mathematical model in different operational settings, such as economic dispatch and unit commitment.

!!! note
    
    The use of reactive power variables and constraints will depend on the network model used, i.e., whether it uses (or does not use) reactive power. If the network model is purely active power-based, reactive power variables and related constraints are not created.

!!! note
    
    Reserve variables for services are not included in the formulation, albeit their inclusion change the variables, expressions, constraints and objective functions created. A detailed description of the implications in the optimization models is described in the [Service formulation](@ref service_formulations) section.

### Table of contents

 1. [`RenewableFullDispatch`](#RenewableFullDispatch)
 2. [`RenewableConstantPowerFactor`](#RenewableConstantPowerFactor)
 3. [Valid configurations](#Valid-configurations)

* * *

## `RenewableFullDispatch`

```@docs
RenewableFullDispatch
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``p^\text{re}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{re}``

**Static Parameters:**

  - ``P^\text{re,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``Q^\text{re,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{re,max}`` = `PowerSystems.get_reactive_power_limits(device).max`

**Time Series Parameters:**

Uses the `max_active_power` timeseries parameter to limit the available active power at each time-step.

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.get_default_time_series_names(RenewableGen, RenewableFullDispatch)
combo_table = DataFrame(
    "Parameter" => map(x -> "[`$x`](@ref)", collect(keys(combos))),
    "Default Time Series Name" => map(x -> "`$x`", collect(values(combos))),
)
mdtable(combo_table; latex = false)
```

**Objective:**

Adds two terms to the objective function based on the [`FunctionData` Options](@ref):

  - ``+ C^\text{re,var} \cdot p_t^\text{re}`` from the `variable` field of `RenewableGenerationCost` (the VOM cost of dispatching the unit).
  - ``- C^\text{re,curt} \cdot p_t^\text{re}`` from the `curtailment_cost` field of `RenewableGenerationCost` (an incentive to dispatch up to ``\text{ActivePowerTimeSeriesParameter}_t``; defaults to ``0``).

The net contribution is ``(C^\text{re,var} - C^\text{re,curt}) \cdot p_t^\text{re}``: when ``C^\text{re,curt} > C^\text{re,var}`` the cost coefficient is negative, creating a marginal incentive to dispatch up to the time-series limit; when ``C^\text{re,curt} < C^\text{re,var}`` the incentive favors curtailment, though system constraints (load balance, reserves) may still require some dispatch. `CurtailmentCostExpression` reports the per-device dollar value of curtailed energy ``C^\text{re,curt} \cdot (p^\text{re,max}_t - p_t^\text{re})`` and is not propagated to `ProductionCostExpression`.

**Expressions:**

Adds ``p^\text{re}`` and ``q^\text{re}`` terms to the respective active and reactive power balance expressions created by the selected [Network Formulations](@ref network_formulations).

**Constraints:**

```math
\begin{aligned}
&  P^\text{re,min} \le p_t^\text{re} \le \text{ActivePowerTimeSeriesParameter}_t, \quad \forall t \in \{1,\dots, T\} \\
&  Q^\text{re,min} \le q_t^\text{re} \le Q^\text{re,max}, \quad \forall t \in \{1,\dots, T\}
\end{aligned}
```

* * *

## `RenewableConstantPowerFactor`

```@docs
RenewableConstantPowerFactor
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: `PowerSystems.get_active_power(device)`
      + Symbol: ``p^\text{re}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: `PowerSystems.get_reactive_power(device)`
      + Symbol: ``q^\text{re}``

**Static Parameters:**

  - ``P^\text{re,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``Q^\text{re,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{re,max}`` = `PowerSystems.get_reactive_power_limits(device).max`
  - ``\text{pf}`` = `PowerSystems.get_power_factor(device)`

**Time Series Parameters:**

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.get_default_time_series_names(
    RenewableGen,
    RenewableConstantPowerFactor,
)
combo_table = DataFrame(
    "Parameter" => map(x -> "[`$x`](@ref)", collect(keys(combos))),
    "Default Time Series Name" => map(x -> "`$x`", collect(values(combos))),
)
mdtable(combo_table; latex = false)
```

**Objective:**

Adds two terms to the objective function based on the [`FunctionData` Options](@ref):

  - ``+ C^\text{re,var} \cdot p_t^\text{re}`` from the `variable` field of `RenewableGenerationCost` (the VOM cost of dispatching the unit).
  - ``- C^\text{re,curt} \cdot p_t^\text{re}`` from the `curtailment_cost` field of `RenewableGenerationCost` (an incentive to dispatch up to ``\text{ActivePowerTimeSeriesParameter}_t``; defaults to ``0``).

The net contribution is ``(C^\text{re,var} - C^\text{re,curt}) \cdot p_t^\text{re}``: when ``C^\text{re,curt} > C^\text{re,var}`` the cost coefficient is negative, creating a marginal incentive to dispatch up to the time-series limit; when ``C^\text{re,curt} < C^\text{re,var}`` the incentive favors curtailment, though system constraints (load balance, reserves) may still require some dispatch. `CurtailmentCostExpression` reports the per-device dollar value of curtailed energy ``C^\text{re,curt} \cdot (p^\text{re,max}_t - p_t^\text{re})`` and is not propagated to `ProductionCostExpression`.

**Expressions:**

Adds ``p^\text{re}`` and ``q^\text{re}`` terms to the respective active and reactive power balance expressions created by the selected [Network Formulations](@ref network_formulations)

**Constraints:**

```math
\begin{aligned}
&  P^\text{re,min} \le p_t^\text{re} \le \text{ActivePowerTimeSeriesParameter}_t, \quad \forall t \in \{1,\dots, T\} \\
&  q_t^\text{re} = \text{pf} \cdot p_t^\text{re}, \quad \forall t \in \{1,\dots, T\}
\end{aligned}
```

* * *

## Valid configurations

Valid [`DeviceModel`](@ref)s for subtypes of `RenewableGen` include the following:

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.generate_device_formulation_combinations()
filter!(x -> x["device_type"] <: RenewableGen, combos)
combo_table = DataFrame(
    "Valid DeviceModel" =>
        ["`DeviceModel($(c["device_type"]), $(c["formulation"]))`" for c in combos],
    "Device Type" => [
        "[$(c["device_type"])](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_$(c["device_type"])/)"
        for c in combos
    ],
    "Formulation" => ["[$(c["formulation"])](@ref)" for c in combos],
)
mdtable(combo_table; latex = false)
```
