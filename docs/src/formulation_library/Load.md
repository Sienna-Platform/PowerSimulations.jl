# `PowerSystems.ElectricLoad` Formulations

Electric load formulations define the optimization models that describe load units (demand) mathematical model in different operational settings, such as economic dispatch and unit commitment.

!!! note
    
    The use of reactive power variables and constraints will depend on the network model used, i.e., whether it uses (or does not use) reactive power. If the network model is purely active power-based, reactive power variables and related constraints are not created.

### Table of contents

 1. [`StaticPowerLoad`](#StaticPowerLoad)
 2. [`PowerLoadInterruption`](#PowerLoadInterruption)
 3. [`PowerLoadDispatch`](#PowerLoadDispatch)
 4. [`PowerLoadShift`](#PowerLoadShift)
 5. [Valid configurations](#Valid-configurations)

* * *

## `StaticPowerLoad`

```@docs
StaticPowerLoad
```

**Variables:**

No variables are created

**Time Series Parameters:**

Uses the `max_active_power`  timeseries parameter to determine the demand value at each time-step

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.get_default_time_series_names(ElectricLoad, StaticPowerLoad)
combo_table = DataFrame(
    "Parameter" => map(x -> "[`$x`](@ref)", collect(keys(combos))),
    "Default Time Series Name" => map(x -> "`$x`", collect(values(combos))),
)
mdtable(combo_table; latex = false)
```

**Expressions:**

Subtracts the parameters listed above from the respective active and reactive power balance expressions created by the selected [Network Formulations](@ref network_formulations).

**Constraints:**

No constraints are created

* * *

## `PowerLoadInterruption`

```@docs
PowerLoadInterruption
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: 0.0
      + Symbol: ``p^\text{ld}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: 0.0
      + Symbol: ``q^\text{ld}``
  - [`OnVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Default initial value: 1
      + Symbol: ``u^\text{ld}``

**Static Parameters:**

  - ``P^\text{ld,max}`` = `PowerSystems.get_max_active_power(device)`
  - ``Q^\text{ld,max}`` = `PowerSystems.get_max_reactive_power(device)`

**Time Series Parameters:**

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.get_default_time_series_names(ElectricLoad, PowerLoadInterruption)
combo_table = DataFrame(
    "Parameter" => map(x -> "[`$x`](@ref)", collect(keys(combos))),
    "Default Time Series Name" => map(x -> "`$x`", collect(values(combos))),
)
mdtable(combo_table; latex = false)
```

**Objective:**

Creates an objective function term based on the [`FunctionData` Options](@ref) where the quantity term is defined as ``p^\text{ld}``.

**Expressions:**

  - Subtract``p^\text{ld}`` and ``q^\text{ld}`` terms and to the respective active and reactive power balance expressions created by the selected [Network Formulations](@ref network_formulations)

**Constraints:**

```math
\begin{aligned}
&  p_t^\text{ld} \le u_t^\text{ld} \cdot \text{ActivePowerTimeSeriesParameter}_t, \quad \forall t \in \{1,\dots, T\} \\
&  q_t^\text{re} = \text{pf} \cdot p_t^\text{re}, \quad \forall t \in \{1,\dots, T\}
\end{aligned}
```

on which ``\text{pf} = \sin(\arctan(Q^\text{ld,max}/P^\text{ld,max}))``.

* * *

## `PowerLoadDispatch`

```@docs
PowerLoadDispatch
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: `PowerSystems.get_active_power(device)`
      + Symbol: ``p^\text{ld}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: `PowerSystems.get_reactive_power(device)`
      + Symbol: ``q^\text{ld}``

**Static Parameters:**

  - ``P^\text{ld,max}`` = `PowerSystems.get_max_active_power(device)`
  - ``Q^\text{ld,max}`` = `PowerSystems.get_max_reactive_power(device)`

**Time Series Parameters:**

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.get_default_time_series_names(ElectricLoad, PowerLoadDispatch)
combo_table = DataFrame(
    "Parameter" => map(x -> "[`$x`](@ref)", collect(keys(combos))),
    "Default Time Series Name" => map(x -> "`$x`", collect(values(combos))),
)
mdtable(combo_table; latex = false)
```

**Objective:**

Creates an objective function term based on the [`FunctionData` Options](@ref) where the quantity term is defined as ``p^\text{ld}``.

**Expressions:**

  - Subtract``p^\text{ld}`` and ``q^\text{ld}`` terms and to the respective active and reactive power balance expressions created by the selected [Network Formulations](@ref network_formulations)

**Constraints:**

```math
\begin{aligned}
&  p_t^\text{ld} \le \text{ActivePowerTimeSeriesParameter}_t, \quad \forall t \in \{1,\dots, T\}\\
&  q_t^\text{ld} = \text{pf} \cdot p_t^\text{ld}, \quad \forall t \in \{1,\dots, T\}\\
\end{aligned}
```

* * *

## `PowerLoadShift`

```@docs
PowerLoadShift
```

**Variables:**

  - [`ShiftUpActivePowerVariable`](@ref):
    
      + Bounds: [0.0, `ShiftUpActivePowerTimeSeriesParameter`]
      + Default initial value: 0.0
      + Symbol: ``p^\text{shift,up}``

  - [`ShiftDownActivePowerVariable`](@ref):
    
      + Bounds: [0.0, `ShiftDownActivePowerTimeSeriesParameter`]
      + Default initial value: 0.0
      + Symbol: ``p^\text{shift,dn}``
  - [`ReactivePowerVariable`](@ref) *(AC network models only)*:
    
      + Bounds: [0.0, ]
      + Default initial value: `PowerSystems.get_reactive_power(device)`
      + Symbol: ``q^\text{ld}``

**Static Parameters:**

  - ``P^\text{ld,max}`` = `PowerSystems.get_max_active_power(device)`
  - ``Q^\text{ld,max}`` = `PowerSystems.get_max_reactive_power(device)`

**Time Series Parameters:**

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.get_default_time_series_names(ShiftablePowerLoad, PowerLoadShift)
combo_table = DataFrame(
    "Parameter" => map(x -> "[`$x`](@ref)", collect(keys(combos))),
    "Default Time Series Name" => map(x -> "`$x`", collect(values(combos))),
)
mdtable(combo_table; latex = false)
```

**Expressions:**

  - Defines the `RealizedShiftedLoad` expression per device per time step:
    
    ```math
    p_t^\text{realized} = \text{ActivePowerTimeSeriesParameter}_t + p_t^\text{shift,up} - p_t^\text{shift,dn}, \quad \forall t \in \{1,\dots,T\}
    ```
  - Subtracts ``p_t^\text{realized}`` from the active power balance expression of the selected [Network Formulations](@ref network_formulations).

**Objective:**

Creates objective function terms based on the [`FunctionData` Options](@ref) for both shift variables:

  - A cost term on ``p^\text{shift,up}`` (typically zero or negative, rewarding shifting up)
  - A cost term on ``p^\text{shift,dn}`` (typically positive, penalizing shifting down)

**Constraints:**

```math
\begin{aligned}
& \sum_{t=1}^{T} \left( p_t^\text{shift,up} - p_t^\text{shift,dn} \right) = 0 \\
& \sum_{t=1}^{T_\text{sub}} \left( p_t^\text{shift,up} - p_t^\text{shift,dn} \right) = 0 \quad \text{(if \texttt{additional\_balance\_interval} is set)}
\end{aligned}
```

```math
p_t^\text{realized} \ge 0, \quad \forall t \in \{1,\dots,T\}
```

```math
p_t^\text{shift,up} \le \text{ShiftUpActivePowerTimeSeriesParameter}_t, \quad \forall t \in \{1,\dots,T\}
```

```math
p_t^\text{shift,dn} \le \text{ShiftDownActivePowerTimeSeriesParameter}_t, \quad \forall t \in \{1,\dots,T\}
```

```math
\sum_{\tau=1}^{t} \left( p_\tau^\text{shift,dn} - p_\tau^\text{shift,up} \right) \ge 0, \quad \forall t \in \{1,\dots,T\}
```

```math
q_t^\text{ld} = \text{pf} \cdot p_t^\text{realized}, \quad \forall t \in \{1,\dots,T\}
```

## Valid configurations

Valid [`DeviceModel`](@ref)s for subtypes of `ElectricLoad` include the following:

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.generate_device_formulation_combinations()
filter!(x -> x["device_type"] <: ElectricLoad, combos)
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
