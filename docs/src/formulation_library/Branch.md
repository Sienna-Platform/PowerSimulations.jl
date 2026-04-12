# `PowerSystems.Branch` Formulations

!!! note
    
    The use of reactive power variables and constraints will depend on the network model used, i.e., whether it uses (or does not use) reactive power. If the network model is purely active power-based, reactive power variables and related constraints are not created.

### Table of contents

 1. [`StaticBranch`](#StaticBranch)
 2. [`StaticBranchBounds`](#StaticBranchBounds)
 3. [`StaticBranchUnbounded`](#StaticBranchUnbounded)
 4. [`HVDCTwoTerminalUnbounded`](#HVDCTwoTerminalUnbounded)
 5. [`HVDCTwoTerminalLossless`](#HVDCTwoTerminalLossless)
 6. [`HVDCTwoTerminalDispatch`](#HVDCTwoTerminalDispatch)
 7. [`PhaseAngleControl`](#PhaseAngleControl)
 8. [`TwoTerminalLCCLine`](#TwoTerminalLCCLine)
 9. [Valid configurations](#Valid-configurations)

## `StaticBranch`

Formulation valid for `PTDFPowerModel` Network model

```@docs
StaticBranch
```

!!! note "Dynamic Line Ratings (DLR)"
    
    `StaticBranch` supports time-varying line rating constraints via `DynamicBranchRatingTimeSeriesParameter`.
    When a `Deterministic` or `SingleTimeSeries` time series named `"dynamic_line_ratings"` is attached to
    a branch and the `DeviceModel` is configured with `time_series_names = Dict(DynamicBranchRatingTimeSeriesParameter => "dynamic_line_ratings")`,
    the static ``R^\text{max}`` bound is replaced by a time-varying parameter ``R^\text{max}_t = R^\text{max} \cdot \text{DLR}_t``.
    This feature requires the `PTDFPowerModel` network model and is **only** available for `StaticBranch`.
    See the [Dynamic Line Ratings (DLR)](@ref) for a complete example.

**Variables:**

  - [`FlowActivePowerVariable`](@ref):
    
      + Bounds: ``(-\infty,\infty)``
      + Symbol: ``f``
        If Slack variables are enabled:

  - [`FlowActivePowerSlackUpperBound`](@ref):
    
      + Bounds: [0.0, ]
      + Default proportional cost: 2e5
      + Symbol: ``f^\text{sl,up}``
  - [`FlowActivePowerSlackLowerBound`](@ref):
    
      + Bounds: [0.0, ]
      + Default proportional cost: 2e5
      + Symbol: ``f^\text{sl,lo}``

**Static Parameters**

  - ``R^\text{max}`` = `PowerSystems.get_rating(branch)`

**Objective:**

Add a large proportional cost to the objective function if rate constraint slack variables are used ``+ (f^\text{sl,up} + f^\text{sl,lo}) \cdot 2 \cdot 10^5``

**Expressions:**

No expressions are used.

**Constraints:**

For each branch ``b \in \{1,\dots, B\}`` (in a system with ``N`` buses) the constraints are given by:

```math
\begin{aligned}
&  f_t = \sum_{i=1}^N \text{PTDF}_{i,b} \cdot \text{Bal}_{i,t}, \quad \forall t \in \{1,\dots, T\}\\
&  f_t - f_t^\text{sl,up} \le R^\text{max},\quad \forall t \in \{1,\dots, T\} \\
&  f_t + f_t^\text{sl,lo} \ge -R^\text{max},\quad \forall t \in \{1,\dots, T\} 
\end{aligned}
```

on which ``\text{PTDF}`` is the ``N \times B`` system Power Transfer Distribution Factors (PTDF) matrix, and ``\text{Bal}_{i,t}`` is the active power bus balance expression (i.e. ``\text{Generation}_{i,t} - \text{Demand}_{i,t}``) at bus ``i`` at time-step ``t``.

* * *

## `StaticBranchBounds`

Formulation valid for `PTDFPowerModel` Network model

!!! warning
    
    `DynamicBranchRatingTimeSeriesParameter` (Dynamic Line Ratings) is **not** supported for
    `StaticBranchBounds`. Use `StaticBranch` if time-varying line rating constraints are needed.

```@docs
StaticBranchBounds
```

**Variables:**

  - [`FlowActivePowerVariable`](@ref):
    
      + Bounds: ``\left[-R^\text{max},R^\text{max}\right]``
      + Symbol: ``f``

**Static Parameters**

  - ``R^\text{max}`` = `PowerSystems.get_rating(branch)`

**Objective:**

No cost is added to the objective function.

**Expressions:**

No expressions are used.

**Constraints:**

For each branch ``b \in \{1,\dots, B\}`` (in a system with ``N`` buses) the constraints are given by:

```math
\begin{aligned}
&  f_t = \sum_{i=1}^N \text{PTDF}_{i,b} \cdot \text{Bal}_{i,t}, \quad \forall t \in \{1,\dots, T\}
\end{aligned}
```

on which ``\text{PTDF}`` is the ``N \times B`` system Power Transfer Distribution Factors (PTDF) matrix, and ``\text{Bal}_{i,t}`` is the active power bus balance expression (i.e. ``\text{Generation}_{i,t} - \text{Demand}_{i,t}``) at bus ``i`` at time-step ``t``.

* * *

## `StaticBranchUnbounded`

Formulation valid for `PTDFPowerModel` Network model

!!! warning
    
    `DynamicBranchRatingTimeSeriesParameter` (Dynamic Line Ratings) is **not** supported for
    `StaticBranchUnbounded`. Use `StaticBranch` if time-varying line rating constraints are needed.

```@docs
StaticBranchUnbounded
```

  - [`FlowActivePowerVariable`](@ref):
    
      + Bounds: ``(-\infty,\infty)``
      + Symbol: ``f``

**Objective:**

No cost is added to the objective function.

**Expressions:**

No expressions are used.

**Constraints:**

For each branch ``b \in \{1,\dots, B\}`` (in a system with ``N`` buses) the constraints are given by:

```math
\begin{aligned}
&  f_t = \sum_{i=1}^N \text{PTDF}_{i,b} \cdot \text{Bal}_{i,t}, \quad \forall t \in \{1,\dots, T\}
\end{aligned}
```

on which ``\text{PTDF}`` is the ``N \times B`` system Power Transfer Distribution Factors (PTDF) matrix, and ``\text{Bal}_{i,t}`` is the active power bus balance expression (i.e. ``\text{Generation}_{i,t} - \text{Demand}_{i,t}``) at bus ``i`` at time-step ``t``.

* * *

## `HVDCTwoTerminalUnbounded`

Formulation valid for `PTDFPowerModel` Network model

```@docs
HVDCTwoTerminalUnbounded
```

This model assumes that it can transfer power from two AC buses without losses and no limits.

**Variables:**

  - [`FlowActivePowerVariable`](@ref):
    
      + Bounds: ``\left(-\infty,\infty\right)``
      + Symbol: ``f``

**Objective:**

No cost is added to the objective function.

**Expressions:**

The variable `FlowActivePowerVariable` ``f`` is added to the nodal balance expression `ActivePowerBalance`, by adding the flow ``f`` in the receiving bus and subtracting it from the sending bus. This is used then to compute the AC flows using the PTDF equation.

**Constraints:**

No constraints are added.

* * *

## `HVDCTwoTerminalLossless`

Formulation valid for `PTDFPowerModel` Network model

```@docs
HVDCTwoTerminalLossless
```

This model assumes that it can transfer power from two AC buses without losses.

**Variables:**

  - [`FlowActivePowerVariable`](@ref):
    
      + Bounds: ``\left(-\infty,\infty\right)``
      + Symbol: ``f``

**Static Parameters**

  - ``R^\text{from,min}`` = `PowerSystems.get_active_power_limits_from(branch).min`
  - ``R^\text{from,max}`` = `PowerSystems.get_active_power_limits_from(branch).max`
  - ``R^\text{to,min}`` = `PowerSystems.get_active_power_limits_to(branch).min`
  - ``R^\text{to,max}`` = `PowerSystems.get_active_power_limits_to(branch).max`

**Objective:**

No cost is added to the objective function.

**Expressions:**

The variable `FlowActivePowerVariable` ``f`` is added to the nodal balance expression `ActivePowerBalance`, by adding the flow ``f`` in the receiving bus and subtracting it from the sending bus. This is used then to compute the AC flows using the PTDF equation.

**Constraints:**

```math
\begin{align*}
&  R^\text{min} \le f_t  \le R^\text{max},\quad \forall t \in \{1,\dots, T\} \\
\end{align*}
```

where:

```math
\begin{align*}
&  R^\text{min} = \begin{cases}
			\min\left(R^\text{from,min}, R^\text{to,min}\right), & \text{if } R^\text{from,min} \ge 0 \text{ and } R^\text{to,min} \ge 0 \\
      \max\left(R^\text{from,min}, R^\text{to,min}\right), & \text{if } R^\text{from,min} \le 0 \text{ and } R^\text{to,min} \le 0 \\
      R^\text{from,min},& \text{if } R^\text{from,min} \le 0 \text{ and } R^\text{to,min} \ge 0 \\
      R^\text{to,min},& \text{if } R^\text{from,min} \ge 0 \text{ and } R^\text{to,min} \le 0
		 \end{cases}
\end{align*}
```

and

```math
\begin{align*}
&  R^\text{max} = \begin{cases}
			\min\left(R^\text{from,max}, R^\text{to,max}\right), & \text{if } R^\text{from,max} \ge 0 \text{ and } R^\text{to,max} \ge 0 \\
      \max\left(R^\text{from,max}, R^\text{to,max}\right), & \text{if } R^\text{from,max} \le 0 \text{ and } R^\text{to,max} \le 0 \\
      R^\text{from,max},& \text{if } R^\text{from,max} \le 0 \text{ and } R^\text{to,max} \ge 0 \\
      R^\text{to,max},& \text{if } R^\text{from,max} \ge 0 \text{ and } R^\text{to,max} \le 0
		 \end{cases}
\end{align*}
```

* * *

## `HVDCTwoTerminalDispatch`

Formulation valid for `PTDFPowerModel` Network model

```@docs
HVDCTwoTerminalDispatch
```

**Variables**

  - [`FlowActivePowerToFromVariable`](@ref):
    
      + Symbol: ``f^\text{to-from}``

  - [`FlowActivePowerFromToVariable`](@ref):
    
      + Symbol: ``f^\text{from-to}``
  - [`HVDCLosses`](@ref):
    
      + Symbol: ``\ell``
  - [`HVDCFlowDirectionVariable`](@ref)
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``u^\text{dir}``

**Static Parameters**

  - ``R^\text{from,min}`` = `PowerSystems.get_active_power_limits_from(branch).min`
  - ``R^\text{from,max}`` = `PowerSystems.get_active_power_limits_from(branch).max`
  - ``R^\text{to,min}`` = `PowerSystems.get_active_power_limits_to(branch).min`
  - ``R^\text{to,max}`` = `PowerSystems.get_active_power_limits_to(branch).max`
  - ``L_0`` = `PowerSystems.get_loss(branch).l0`
  - ``L_1`` = `PowerSystems.get_loss(branch).l1`

**Objective:**

No cost is added to the objective function.

**Expressions:**

Each `FlowActivePowerToFromVariable` ``f^\text{to-from}`` and `FlowActivePowerFromToVariable` ``f^\text{from-to}``  is added to the nodal balance expression `ActivePowerBalance`, by adding the respective flow in the receiving bus and subtracting it from the sending bus. That is,  ``f^\text{to-from}`` adds the flow to the `from` bus, and subtracts the flow from the `to` bus, while ``f^\text{from-to}`` adds the flow to the `to` bus, and subtracts the flow from the `from` bus  This is used then to compute the AC flows using the PTDF equation.

In addition, the `HVDCLosses` are subtracted to the `from` bus in the `ActivePowerBalance` expression.

**Constraints:**

```math
\begin{align*}
&  R^\text{from,min} \le f_t^\text{from-to}  \le R^\text{from,max}, \forall t \in \{1,\dots, T\} \\
&  R^\text{to,min} \le f_t^\text{to-from}  \le R^\text{to,max},\quad \forall t \in \{1,\dots, T\} \\
& f_t^\text{to-from} - f_t^\text{from-to} \le L_1 \cdot f_t^\text{to-from} - L_0,\quad \forall t \in \{1,\dots, T\} \\
& f_t^\text{from-to} - f_t^\text{to-from} \ge L_1 \cdot f_t^\text{from-to} + L_0,\quad \forall t \in \{1,\dots, T\} \\
& f_t^\text{from-to} - f_t^\text{to-from} \ge - M^\text{big} (1 - u^\text{dir}_t),\quad \forall t \in \{1,\dots, T\} \\
& f_t^\text{to-from} - f_t^\text{from-to} \ge - M^\text{big} u^\text{dir}_t,\quad \forall t \in \{1,\dots, T\} \\
& f_t^\text{to-from} - f_t^\text{from-to} \le \ell_t,\quad \forall t \in \{1,\dots, T\} \\
& f_t^\text{from-to} - f_t^\text{to-from} \le \ell_t,\quad \forall t \in \{1,\dots, T\} 
\end{align*}
```

* * *

## `PhaseAngleControl`

Formulation valid for `PTDFPowerModel` Network model

```@docs
PhaseAngleControl
```

**Variables:**

  - [`FlowActivePowerVariable`](@ref):
    
      + Bounds: ``(-\infty,\infty)``
      + Symbol: ``f``

  - [`PhaseShifterAngle`](@ref):
    
      + Symbol: ``\theta^\text{shift}``

**Static Parameters**

  - ``R^\text{max}`` = `PowerSystems.get_rating(branch)`
  - ``\Theta^\text{min}`` = `PowerSystems.get_phase_angle_limits(branch).min`
  - ``\Theta^\text{max}`` = `PowerSystems.get_phase_angle_limits(branch).max`
  - ``X`` = `PowerSystems.get_x(branch)` (series reactance)

**Objective:**

No changes to objective function

**Expressions:**

Adds to the `ActivePowerBalance` expression the term ``-\theta^\text{shift} /X`` to the `from` bus and ``+\theta^\text{shift} /X`` to the `to` bus, that the `PhaseShiftingTransformer` is connected.

**Constraints:**

For each branch ``b \in \{1,\dots, B\}`` (in a system with ``N`` buses) the constraints are given by:

```math
\begin{aligned}
&  f_t = \sum_{i=1}^N \text{PTDF}_{i,b} \cdot \text{Bal}_{i,t} + \frac{\theta^\text{shift}_t}{X}, \quad \forall t \in \{1,\dots, T\}\\
&  -R^\text{max} \le f_t  \le R^\text{max},\quad \forall t \in \{1,\dots, T\} 
\end{aligned}
```

on which ``\text{PTDF}`` is the ``N \times B`` system Power Transfer Distribution Factors (PTDF) matrix, and ``\text{Bal}_{i,t}`` is the active power bus balance expression (i.e. ``\text{Generation}_{i,t} - \text{Demand}_{i,t}``) at bus ``i`` at time-step ``t``.

* * *

## `TwoTerminalLCCLine`

Formulation valid for `ACPPowerModel` Network model

**Variables:**

  - [`HVDCRectifierDelayAngleVariable`]:
    
        + Bounds: ``(-\alpha_{r,t}^\text{min},\alpha_{r,t}^\text{max})``
        + Symbol: ``\alpha_{r,t}``

  - [`HVDCInverterExtinctionAngleVariable`]:
    
        + Bounds: ``(-\gamma_{i,t}^\text{min},\gamma_{i,t}^\text{max})``
        + Symbol: ``\gamma_{i,t}``
  - [`HVDCRectifierPowerFactorAngleVariable`]:
    
        + Bounds: ``\{0,1\}``
        + Symbol: ``\phi_{r,t}``
  - [`HVDCInverterPowerFactorAngleVariable`]:
    
        + Bounds: ``\{0,1\}``
        + Symbol: ``\phi_{i,t}``
  - [`HVDCRectifierOverlapAngleVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``\mu_{r,t}``
  - [`HVDCInverterOverlapAngleVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``\mu_{i,t}``
  - [`HVDCRectifierTapSettingVariable`]:
    
        + Bounds: ``(t_{r,t}^\text{min},t_{r,t}^\text{max})``
        + Symbol: ``t_{r,t}``
  - [`HVDCInverterTapSettingVariable`]:
    
        + Bounds: ``(t_{i,t}^\text{min},t_{i,t}^\text{max})``
        + Symbol: ``t_{i,t}``
  - [`HVDCRectifierDCVoltageVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``v_{r,t}^\text{dc}``
  - [`HVDCInverterDCVoltageVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``v_{i,t}^\text{dc}``
  - [`HVDCRectifierACCurrentVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``I_{r,t}^\text{ac}``
  - [`HVDCInverterACCurrentVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``I_{i,t}^\text{ac}``
  - [`DCLineCurrentFlowVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``I^\text{dc}``
  - [`HVDCActivePowerReceivedFromVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``p_{r,t}^\text{ac}``
  - [`HVDCActivePowerReceivedToVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``p_{i,t}^\text{ac}``
  - [`HVDCReactivePowerReceivedFromVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``q_{r,t}^\text{ac}``
  - [`HVDCReactivePowerReceivedToVariable`]:
    
        + Bounds: [0.0, ]
        + Symbol: ``q_{i,t}^\text{ac}``

**Static Parameters**

  - ``R^\text{dc}`` = `PowerSystems.get_r(lcc)`
  - ``N_r`` = `PowerSystems.get_rectifier_bridges(lcc)`
  - ``N_i`` = `PowerSystems.get_inverter_bridges(lcc)`
  - ``X_r`` = `PowerSystems.get_rectifier_xc(lcc)`
  - ``X_i`` = `PowerSystems.get_inverter_xc(lcc)`
  - ``a_r`` = `PowerSystems.get_rectifier_transformer_ratio(lcc)`
  - ``a_i`` = `PowerSystems.get_inverter_transformer_ratio(lcc)`
  - ``t_r`` = `PowerSystems.get_rectifier_tap_setting(lcc)`
  - ``t_i`` = `PowerSystems.get_inverter_tap_setting(lcc)`
  - ``t^\text{min}_r`` = `PowerSystems.get_rectifier_tap_setting(lcc).min`
  - ``t^\text{max}_r`` = `PowerSystems.get_rectifier_tap_setting(lcc).max`
  - ``t^\text{min}_i`` = `PowerSystems.get_inverter_tap_setting(lcc).min`
  - ``t^\text{max}_i`` = `PowerSystems.get_inverter_tap_setting(lcc).max`

**Objective:**

No changes to objective function

**Expressions:**

The variable `HVDCActivePowerReceivedFromVariable` ``p_{r,t}^\text{ac}`` is added to the nodal balance expression `ActivePowerBalance` as a negative load, since the rectifier takes power from the AC system and to injects it into the DC system. On the other hand, the variable `HVDCActivePowerReceivedToVariable` ``p_{i,t}^\text{ac}`` is added to the nodal balance expression `ActivePowerBalance` as a positive load, since it takes the power from the DC system and injects it  back into the AC system.

The variables `HVDCReactivePowerReceivedFromVariable` ``q_{r,t}^\text{ac}`` and `HVDCReactivePowerReceivedToVariable` ``q_{i,t}^\text{ac}`are added to the nodal balance expression`ActivePowerBalance` as positive loads, since they consume reactive power from the AC system to allow current transfer in converters during commutation.

**Constraints:**

  - **Rectifier:**

```math
\begin{aligned}
&  v^\text{dc}_{r,t} = \frac{3}{\pi}N_r \left( \sqrt{2}\frac{a_r v^\text{ac}_{r,t}}{t_{r,t}}\\cos{\alpha_{r,t}}-X_r I^\text{dc}_t \right)\\
& \mu_{r,t} = \arccos \left( \cos\alpha_{r,t} - \frac{\sqrt{2} I^\text{dc}_t X_r t_{r,t}}{a_r v^\text{ac}_{r,t}} \right) - \alpha_{r,t}\\
& \phi_{r,t} = \arctan \left( \frac{2\mu_{r,t} + \sin(2\alpha_{r,t}) - \sin(2\mu_{r,t} + 2\alpha_{r,t})}{\cos(2\alpha_{r,t}) - \cos(2\mu_{r,t} + 2\alpha_{r,t})} \right)\\
\end{aligned}
```

Which can be approximated as:

```math
\begin{aligned}
& \phi_{r,t} = arccos(\frac{1}{2}\cos\alpha_{r,t} + \frac{1}{2}\cos(\alpha_{r,t} + \mu_{r,t}))
\end{aligned}
```

```math
\begin{aligned}
& I^\text{ac}_{r,t} = \sqrt{6} \frac{N_r}{\pi} I^\text{dc}_t\\
& p^\text{ac}_{r,t} = \sqrt{3} I^\text{ac}_{r,t} \frac{a_r v^\text{ac}_{r,t}}{t_{r,t}}\cos{\phi_{r,t}} \\
& q^\text{ac}_{r,t} = \sqrt{3} I^\text{ac}_{r,t} \frac{a_r v^\text{ac}_{r,t}}{t_{r,t}}\sin{\phi_{r,t}} \\
\end{aligned}
```

  - **Inverter:**

```math
\begin{aligned}
&  v^\text{dc}_{i,t} = \frac{3}{\pi}N_i \left( \sqrt{2}\frac{a_i v^\text{ac}_{i,t}}{t_{i,t}}\\cos{\gamma_{i,t}}-X_i I^\text{dc}_t \right)\\
& \mu_{i,t} = \arccos \left( \cos\gamma_{i,t} - \frac{\sqrt{2} I^\text{dc}_t X_i t_{i,t}}{a_i v^\text{ac}_{i,t}} \right) - \gamma_{i,t}\\
& \phi_{i,t} = \arctan \left( \frac{2\mu_{i,t} + \sin(2\gamma_{i,t}) - \sin(2\mu_{i,t} + 2\gamma_{i,t})}{\cos(2\gamma_{i,t}) - \cos(2\mu_{r,t} + 2\gamma_{i,t})} \right)\\
\end{aligned}
```

Which can be approximated as:

```math
\begin{aligned}
& \phi_{i,t} = arccos(\frac{1}{2}\cos\gamma_{i,t} + \frac{1}{2}\cos(\gamma_{i,t} + \mu_{i,t}))
\end{aligned}
```

```math
\begin{aligned}
& I^\text{ac}_{i,t} = \sqrt{6} \frac{N_i}{\pi} I^\text{dc}_t\\
& p^\text{ac}_{i,t} = \sqrt{3} I^\text{ac}_{i,t} \frac{a_i v^\text{ac}_{i,t}}{t_{i,t}}\cos{\phi_{i,t}} \\
& q^\text{ac}_{i,t} = \sqrt{3} I^\text{ac}_{i,t} \frac{a_i v^\text{ac}_{i,t}}{t_{i,t}}\sin{\phi_{i,t}} \\
\end{aligned}
```

  - **DC Transmission Line:**

```math
\begin{aligned}
&  v^\text{dc}_{i,t} = v^\text{dc}_{r,t} - R_\text{dc}I^\text{dc}_t
\end{aligned}
```

* * *

## Valid configurations

Valid [`DeviceModel`](@ref)s for subtypes of `Branch` include the following:

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.generate_device_formulation_combinations()
filter!(x -> (x["device_type"] <: Branch) && (x["device_type"] != TModelHVDCLine), combos)
combo_table = DataFrame(
    "Valid DeviceModel" =>
        ["`DeviceModel($(c["device_type"]), $(c["formulation"]))`" for c in combos],
    "Device Type" => [
        "[$(c["device_type"])](https://nrel-Sienna.github.io/PowerSystems.jl/stable/model_library/generated_$(c["device_type"])/)"
        for c in combos
    ],
    "Formulation" => ["[$(c["formulation"])](@ref)" for c in combos],
)
mdtable(combo_table; latex = false)
```
