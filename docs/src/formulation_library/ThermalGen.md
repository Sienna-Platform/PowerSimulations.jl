# `ThermalGen` Formulations

Thermal generation formulations define the optimization models that describe thermal units mathematical model in different operational settings, such as economic dispatch and unit commitment.

!!! note
    
    Thermal units can include multiple terms added to the objective function, such as no-load cost, turn-on/off cost, fixed cost and variable cost. In addition, variable costs can be linear, quadratic or piecewise-linear formulations. These methods are properly described in the [cost function page](@ref pwl_cost).

!!! note
    
    The use of reactive power variables and constraints will depend on the network model used, i.e., whether it uses (or does not use) reactive power. If the network model is purely active power-based,  reactive power variables and related constraints are not created.

!!! note
    
    Reserve variables for services are not included in the formulation, albeit their inclusion change the variables, expressions, constraints and objective functions created. A detailed description of the implications in the optimization models is described in the [Service formulation](@ref service_formulations) section.

### Table of Contents

 1. [`ThermalBasicDispatch`](#ThermalBasicDispatch)
 2. [`ThermalDispatchNoMin`](#ThermalDispatchNoMin)
 3. [`ThermalCompactDispatch`](#ThermalCompactDispatch)
 4. [`ThermalStandardDispatch`](#ThermalStandardDispatch)
 5. [`ThermalBasicUnitCommitment`](#ThermalBasicUnitCommitment)
 6. [`ThermalBasicCompactUnitCommitment`](#ThermalBasicCompactUnitCommitment)
 7. [`ThermalStandardUnitCommitment`](#ThermalStandardUnitCommitment)
 8. [`ThermalMultiStartUnitCommitment`](#ThermalMultiStartUnitCommitment)
 9. [Valid configurations](#Valid-configurations)

* * *

## `ThermalBasicDispatch`

```@docs
ThermalBasicDispatch
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``

**Static Parameters:**

  - ``P^\text{th,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters.

```math
\begin{align*}
&  P^\text{th,min} \le p^\text{th}_t \le P^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
&  Q^\text{th,min} \le q^\text{th}_t \le Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\} 
\end{align*}
```

* * *

## `ThermalDispatchNoMin`

```@docs
ThermalDispatchNoMin
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``

**Static Parameters:**

  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters.

```math
\begin{align}
&  0 \le p^\text{th}_t \le P^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
&  Q^\text{th,min} \le q^\text{th}_t \le Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\} 
\end{align}
```

* * *

## `ThermalCompactDispatch`

```@docs
ThermalCompactDispatch
```

**Variables:**

  - [`PowerAboveMinimumVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``\Delta p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``

**Auxiliary Variables:**

  - [`PowerOutput`](@ref):
    
      + Symbol: ``P^\text{th}``
      + Definition: ``P^\text{th} = \text{on}^\text{th}P^\text{min} + \Delta p^\text{th}``

**Static Parameters:**

  - ``P^\text{th,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`
  - ``R^\text{th,up}`` = `PowerSystems.get_ramp_limits(device).up`
  - ``R^\text{th,dn}`` = `PowerSystems.get_ramp_limits(device).down`

**Variable Value Parameters:**

  - ``\text{on}^\text{th}``: Used in feedforwards to define if the unit is on/off at each time-step from another problem. If no feedforward is used, the parameter takes a {0,1} value if the unit is available or not.

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``\text{on}^\text{th}P^\text{th,min} + \Delta p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters. It also implements ramp constraints for the active power variable.

```math
\begin{align*}
&  0 \le \Delta p^\text{th}_t \le \text{on}^\text{th}_t\left(P^\text{th,max} - P^\text{th,min}\right), \quad \forall t\in \{1, \dots, T\} \\
&  \text{on}^\text{th}_t Q^\text{th,min} \le q^\text{th}_t \le \text{on}^\text{th}_t Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\}  \\
& -R^\text{th,dn} \le \Delta p_1^\text{th} - \Delta p^\text{th, init} \le R^\text{th,up} \\
& -R^\text{th,dn} \le \Delta p_t^\text{th} - \Delta p_{t-1}^\text{th} \le R^\text{th,up}, \quad \forall  t\in \{2, \dots, T\}
\end{align*}
```

* * *

## `ThermalStandardDispatch`

```@docs
ThermalStandardDispatch
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``

If Slack variables are enabled (`use_slacks = true`):

  - [`RateofChangeConstraintSlackUp`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: 0.0
      + Default proportional cost: 2e5
      + Symbol: ``p^\text{sl,up}``

  - [`RateofChangeConstraintSlackDown`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: 0.0
      + Default proportional cost: 2e5
      + Symbol: ``p^\text{sl,dn}``

**Static Parameters:**

  - ``P^\text{th,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`
  - ``R^\text{th,up}`` = `PowerSystems.get_ramp_limits(device).up`
  - ``R^\text{th,dn}`` = `PowerSystems.get_ramp_limits(device).down`

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters.

```math
\begin{align*}
&  P^\text{th,min} \le p^\text{th}_t \le P^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
&  Q^\text{th,min} \le q^\text{th}_t \le Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
& p_1^\text{th} - p^\text{th, init} - p_1^\text{sl,up} \le R^\text{th,up} \\
& p_t^\text{th} - p_{t-1}^\text{th} - p_t^\text{sl,up}\le R^\text{th,up}, \quad \forall  t\in \{2, \dots, T\} \\
& -R^\text{th,dn} \le  p_1^\text{th} - p^\text{th, init} + p_1^\text{sl,dn} \\
&  -R^\text{th,dn} \le p_t^\text{th} - p_{t-1}^\text{th} + p_t^\text{sl,dn}, \quad \forall  t\in \{2, \dots, T\} \\
\end{align*}
```

* * *

## `ThermalBasicUnitCommitment`

```@docs
ThermalBasicUnitCommitment
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``
  - [`OnVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``u_t^\text{th}``
  - [`StartVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``v_t^\text{th}``
  - [`StopVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``w_t^\text{th}``

**Static Parameters:**

  - ``P^\text{th,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters. In addition, it creates the commitment constraint to turn on/off the device.

```math
\begin{align*}
&  u_t^\text{th} P^\text{th,min} \le p^\text{th}_t \le u_t^\text{th} P^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
&  u_t^\text{th} Q^\text{th,min} \le q^\text{th}_t \le u_t^\text{th} Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
& u_1^\text{th} = u^\text{th,init} + v_1^\text{th} - w_1^\text{th} \\
& u_t^\text{th} = u_{t-1}^\text{th} + v_t^\text{th} - w_t^\text{th}, \quad \forall t \in \{2,\dots,T\} \\
& v_t^\text{th} + w_t^\text{th} \le 1, \quad \forall t \in \{1,\dots,T\}
\end{align*}
```

* * *

## `ThermalBasicCompactUnitCommitment`

```@docs
ThermalBasicCompactUnitCommitment
```

**Variables:**

  - [`PowerAboveMinimumVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``\Delta p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``
  - [`OnVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``u_t^\text{th}``
  - [`StartVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``v_t^\text{th}``
  - [`StopVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``w_t^\text{th}``

**Auxiliary Variables:**

  - [`PowerOutput`](@ref):
    
      + Symbol: ``P^\text{th}``
      + Definition: ``P^\text{th} = u^\text{th}P^\text{min} + \Delta p^\text{th}``

**Static Parameters:**

  - ``P^\text{th,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``u^\text{th}P^\text{th,min} + \Delta p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters. In addition, it creates the commitment constraint to turn on/off the device.

```math
\begin{align*}
&  0 \le \Delta p^\text{th}_t \le u^\text{th}_t\left(P^\text{th,max} - P^\text{th,min}\right), \quad \forall t\in \{1, \dots, T\} \\
&  u_t^\text{th} Q^\text{th,min} \le q^\text{th}_t \le u_t^\text{th} Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
& u_1^\text{th} = u^\text{th,init} + v_1^\text{th} - w_1^\text{th} \\
& u_t^\text{th} = u_{t-1}^\text{th} + v_t^\text{th} - w_t^\text{th}, \quad \forall t \in \{2,\dots,T\} \\
& v_t^\text{th} + w_t^\text{th} \le 1, \quad \forall t \in \{1,\dots,T\}
\end{align*}
```

* * *

## `ThermalCompactUnitCommitment`

```@docs
ThermalCompactUnitCommitment
```

**Variables:**

  - [`PowerAboveMinimumVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``\Delta p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``
  - [`OnVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``u_t^\text{th}``
  - [`StartVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``v_t^\text{th}``
  - [`StopVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``w_t^\text{th}``

**Auxiliary Variables:**

  - [`PowerOutput`](@ref):
    
      + Symbol: ``P^\text{th}``
      + Definition: ``P^\text{th} = u^\text{th}P^\text{min} + \Delta p^\text{th}``

  - [`TimeDurationOn`](@ref):
    
      + Symbol: ``V_t^\text{th}``
      + Definition: Computed post optimization by adding consecutive turned on variable ``u_t^\text{th}``
  - [`TimeDurationOff`](@ref):
    
      + Symbol: ``W_t^\text{th}``
      + Definition: Computed post optimization by adding consecutive turned off variable ``1 - u_t^\text{th}``

**Static Parameters:**

  - ``P^\text{th,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`
  - ``R^\text{th,up}`` = `PowerSystems.get_ramp_limits(device).up`
  - ``R^\text{th,dn}`` = `PowerSystems.get_ramp_limits(device).down`
  - ``D^\text{min,up}`` = `PowerSystems.get_time_limits(device).up`
  - ``D^\text{min,dn}`` = `PowerSystems.get_time_limits(device).down`

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``u^\text{th}P^\text{th,min} + \Delta p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters. It also creates the commitment constraint to turn on/off the device.

```math
\begin{align*}
&  0 \le \Delta p^\text{th}_t \le u^\text{th}_t\left(P^\text{th,max} - P^\text{th,min}\right), \quad \forall t\in \{1, \dots, T\} \\
&  u_t^\text{th} Q^\text{th,min} \le q^\text{th}_t \le u_t^\text{th} Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
& -R^\text{th,dn} \le \Delta p_1^\text{th} - \Delta p^\text{th, init} \le R^\text{th,up} \\
& -R^\text{th,dn} \le \Delta p_t^\text{th} - \Delta p_{t-1}^\text{th} \le R^\text{th,up}, \quad \forall  t\in \{2, \dots, T\} \\
& u_1^\text{th} = u^\text{th,init} + v_1^\text{th} - w_1^\text{th} \\
& u_t^\text{th} = u_{t-1}^\text{th} + v_t^\text{th} - w_t^\text{th}, \quad \forall t \in \{2,\dots,T\} \\
& v_t^\text{th} + w_t^\text{th} \le 1, \quad \forall t \in \{1,\dots,T\} 
\end{align*}
```

In addition, this formulation adds duration constraints, i.e. minimum-up time and minimum-down time constraints.  The duration constraints are added over the start times looking backwards.

The duration times ``D^\text{min,up}`` and ``D^\text{min,dn}`` are processed to be used in multiple of the time-steps, given the resolution of the specific problem. In addition, parameters ``D^\text{init,up}`` and ``D^\text{init,dn}`` are used to identify how long the unit was on or off, respectively, before the simulation started.

Minimum up-time constraint for ``t \in \{1,\dots T\}``:

```math
\begin{align*}
&  \text{If } t \leq D^\text{min,up} - D^\text{init,up} \text{ and } D^\text{init,up} > 0: \\
& 1 + \sum_{i=t-D^\text{min,up} + 1}^t v_i^\text{th}  \leq u_t^\text{th} \quad \text{(for } i \text{ in the set of time steps).} \\
& \text{Otherwise:} \\ 
& \sum_{i=t-D^\text{min,up} + 1}^t v_i^\text{th} \leq  u_t^\text{th} 
\end{align*}
```

Minimum down-time constraint for ``t \in \{1,\dots T\}``:

```math
\begin{align*}
&  \text{If } t \leq D^\text{min,dn} - D^\text{init,dn} \text{ and } D^\text{init,up} > 0: \\
& 1 + \sum_{i=t-D^\text{min,dn} + 1}^t w_i^\text{th} \leq 1 -  u_t^\text{th}  \quad \text{(for } i \text{ in the set of time steps).} \\
& \text{Otherwise:} \\ 
& \sum_{i=t-D^\text{min,dn} + 1}^t w_i^\text{th}  \leq 1 - u_t^\text{th}
\end{align*}
```

* * *

## `ThermalStandardUnitCommitment`

```@docs
ThermalStandardUnitCommitment
```

**Variables:**

  - [`ActivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``
  - [`OnVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``u_t^\text{th}``
  - [`StartVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``v_t^\text{th}``
  - [`StopVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``w_t^\text{th}``

If Slack variables are enabled (`use_slacks = true`):

  - [`RateofChangeConstraintSlackUp`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: 0.0
      + Default proportional cost: 2e5
      + Symbol: ``p^\text{sl,up}``

  - [`RateofChangeConstraintSlackDown`](@ref):
    
      + Bounds: [0.0, ]
      + Default initial value: 0.0
      + Default proportional cost: 2e5
      + Symbol: ``p^\text{sl,dn}``

**Auxiliary Variables:**

  - [`TimeDurationOn`](@ref):
    
      + Symbol: ``V_t^\text{th}``
      + Definition: Computed post optimization by adding consecutive turned on variable ``u_t^\text{th}``

  - [`TimeDurationOff`](@ref):
    
      + Symbol: ``W_t^\text{th}``
      + Definition: Computed post optimization by adding consecutive turned off variable ``1 - u_t^\text{th}``

**Static Parameters:**

  - ``P^\text{th,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`
  - ``R^\text{th,up}`` = `PowerSystems.get_ramp_limits(device).up`
  - ``R^\text{th,dn}`` = `PowerSystems.get_ramp_limits(device).down`
  - ``D^\text{min,up}`` = `PowerSystems.get_time_limits(device).up`
  - ``D^\text{min,dn}`` = `PowerSystems.get_time_limits(device).down`

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters. It also creates the commitment constraint to turn on/off the device.

```math
\begin{align*}
&  u^\text{th}_t P^\text{th,min} \le  p^\text{th}_t \le u^\text{th}_t P^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
&  u_t^\text{th} Q^\text{th,min} \le q^\text{th}_t \le u_t^\text{th} Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
& p_1^\text{th} -  p^\text{th, init} - p_1^\text{sl,up} \le R^\text{th,up} \\
& p_t^\text{th} -  p_{t-1}^\text{th} - p_t^\text{sl,up} \le R^\text{th,up}, \quad \forall  t\in \{2, \dots, T\} \\
& -R^\text{th,dn} \le p_1^\text{th} -  p^\text{th, init} + p_1^\text{sl,dn} \\
& -R^\text{th,dn} \le  p_t^\text{th} -  p_{t-1}^\text{th} + p_t^\text{sl,dn}, \quad \forall  t\in \{2, \dots, T\} \\
& u_1^\text{th} = u^\text{th,init} + v_1^\text{th} - w_1^\text{th} \\
& u_t^\text{th} = u_{t-1}^\text{th} + v_t^\text{th} - w_t^\text{th}, \quad \forall t \in \{2,\dots,T\} \\
& v_t^\text{th} + w_t^\text{th} \le 1, \quad \forall t \in \{1,\dots,T\} 
\end{align*}
```

In addition, this formulation adds duration constraints, i.e. minimum-up time and minimum-down time constraints.  The duration constraints are added over the start times looking backwards.

The duration times ``D^\text{min,up}`` and ``D^\text{min,dn}`` are processed to be used in multiple of the time-steps, given the resolution of the specific problem. In addition, parameters ``D^\text{init,up}`` and ``D^\text{init,dn}`` are used to identify how long the unit was on or off, respectively, before the simulation started.

Minimum up-time constraint for ``t \in \{1,\dots T\}``:

```math
\begin{align*}
&  \text{If } t \leq D^\text{min,up} - D^\text{init,up} \text{ and } D^\text{init,up} > 0: \\
& 1 + \sum_{i=t-D^\text{min,up} + 1}^t v_i^\text{th}  \leq u_t^\text{th} \quad \text{(for } i \text{ in the set of time steps).} \\
& \text{Otherwise:} \\ 
& \sum_{i=t-D^\text{min,up} + 1}^t v_i^\text{th} \leq  u_t^\text{th} 
\end{align*}
```

Minimum down-time constraint for ``t \in \{1,\dots T\}``:

```math
\begin{align*}
&  \text{If } t \leq D^\text{min,dn} - D^\text{init,dn} \text{ and } D^\text{init,up} > 0: \\
& 1 + \sum_{i=t-D^\text{min,dn} + 1}^t w_i^\text{th} \leq 1 -  u_t^\text{th}  \quad \text{(for } i \text{ in the set of time steps).} \\
& \text{Otherwise:} \\ 
& \sum_{i=t-D^\text{min,dn} + 1}^t w_i^\text{th}  \leq 1 - u_t^\text{th}
\end{align*}
```

* * *

## `ThermalMultiStartUnitCommitment`

```@docs
ThermalMultiStartUnitCommitment
```

**Variables:**

  - [`PowerAboveMinimumVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``\Delta p^\text{th}``

  - [`ReactivePowerVariable`](@ref):
    
      + Bounds: [0.0, ]
      + Symbol: ``q^\text{th}``
  - [`OnVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``u_t^\text{th}``
  - [`StartVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``v_t^\text{th}``
  - [`StopVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``w_t^\text{th}``
  - [`ColdStartVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``x_t^\text{th}``
  - [`WarmStartVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``y_t^\text{th}``
  - [`HotStartVariable`](@ref):
    
      + Bounds: ``\{0,1\}``
      + Symbol: ``z_t^\text{th}``

**Auxiliary Variables:**

  - [`PowerOutput`](@ref):
    
      + Symbol: ``P^\text{th}``
      + Definition: ``P^\text{th} = u^\text{th}P^\text{min} + \Delta p^\text{th}``

  - [`TimeDurationOn`](@ref):
    
      + Symbol: ``V_t^\text{th}``
      + Definition: Computed post optimization by adding consecutive turned on variable ``u_t^\text{th}``
  - [`TimeDurationOff`](@ref):
    
      + Symbol: ``W_t^\text{th}``
      + Definition: Computed post optimization by adding consecutive turned off variable ``1 - u_t^\text{th}``

**Static Parameters:**

  - ``P^\text{th,min}`` = `PowerSystems.get_active_power_limits(device).min`
  - ``P^\text{th,max}`` = `PowerSystems.get_active_power_limits(device).max`
  - ``Q^\text{th,min}`` = `PowerSystems.get_reactive_power_limits(device).min`
  - ``Q^\text{th,max}`` = `PowerSystems.get_reactive_power_limits(device).max`
  - ``R^\text{th,up}`` = `PowerSystems.get_ramp_limits(device).up`
  - ``R^\text{th,dn}`` = `PowerSystems.get_ramp_limits(device).down`
  - ``D^\text{min,up}`` = `PowerSystems.get_time_limits(device).up`
  - ``D^\text{min,dn}`` = `PowerSystems.get_time_limits(device).down`
  - ``D^\text{cold}`` = `PowerSystems.get_start_time_limits(device).cold`
  - ``D^\text{warm}`` = `PowerSystems.get_start_time_limits(device).warm`
  - ``D^\text{hot}`` = `PowerSystems.get_start_time_limits(device).hot`
  - ``P^\text{th,startup}`` = `PowerSystems.get_power_trajectory(device).startup`
  - ``P^\text{th, shdown}`` = `PowerSystems.get_power_trajectory(device).shutdown`

**Objective:**

Add a cost to the objective function depending on the defined cost structure of the thermal unit by adding it to its `ProductionCostExpression`.

**Expressions:**

Adds ``u^\text{th}P^\text{th,min} + \Delta p^\text{th}`` to the `ActivePowerBalance` expression and ``q^\text{th}`` to the `ReactivePowerBalance`, to be used in the supply-balance constraint depending on the network model used.

**Constraints:**

For each thermal unit creates the range constraints for its active and reactive power depending on its static parameters. It also creates the commitment constraint to turn on/off the device.

```math
\begin{align*}
&  0 \le \Delta p^\text{th}_t \le u^\text{th}_t\left(P^\text{th,max} - P^\text{th,min}\right), \quad \forall t\in \{1, \dots, T\} \\
&  u_t^\text{th} Q^\text{th,min} \le q^\text{th}_t \le u_t^\text{th} Q^\text{th,max}, \quad \forall t\in \{1, \dots, T\} \\
& -R^\text{th,dn} \le \Delta p_1^\text{th} - \Delta p^\text{th, init} \le R^\text{th,up} \\
& -R^\text{th,dn} \le \Delta p_t^\text{th} - \Delta p_{t-1}^\text{th} \le R^\text{th,up}, \quad \forall  t\in \{2, \dots, T\} \\
& u_1^\text{th} = u^\text{th,init} + v_1^\text{th} - w_1^\text{th} \\
& u_t^\text{th} = u_{t-1}^\text{th} + v_t^\text{th} - w_t^\text{th}, \quad \forall t \in \{2,\dots,T\} \\
& v_t^\text{th} + w_t^\text{th} \le 1, \quad \forall t \in \{1,\dots,T\} \\
& \max\{P^\text{th,max} - P^\text{th,shdown}, 0\} \cdot w_1^\text{th} \le u^\text{th,init} (P^\text{th,max} - P^\text{th,min}) - P^\text{th,init}
\end{align*}
```

In addition, this formulation adds duration constraints, i.e. minimum-up time and minimum-down time constraints.  The duration constraints are added over the start times looking backwards.

The duration times ``D^\text{min,up}`` and ``D^\text{min,dn}`` are processed to be used in multiple of the time-steps, given the resolution of the specific problem. In addition, parameters ``D^\text{init,up}`` and ``D^\text{init,dn}`` are used to identify how long the unit was on or off, respectively, before the simulation started.

Minimum up-time constraint for ``t \in \{1,\dots T\}``:

```math
\begin{align*}
&  \text{If } t \leq D^\text{min,up} - D^\text{init,up} \text{ and } D^\text{init,up} > 0: \\
& 1 + \sum_{i=t-D^\text{min,up} + 1}^t v_i^\text{th}  \leq u_t^\text{th} \quad \text{(for } i \text{ in the set of time steps).} \\
& \text{Otherwise:} \\ 
& \sum_{i=t-D^\text{min,up} + 1}^t v_i^\text{th} \leq  u_t^\text{th} 
\end{align*}
```

Minimum down-time constraint for ``t \in \{1,\dots T\}``:

```math
\begin{align*}
&  \text{If } t \leq D^\text{min,dn} - D^\text{init,dn} \text{ and } D^\text{init,up} > 0: \\
& 1 + \sum_{i=t-D^\text{min,dn} + 1}^t w_i^\text{th} \leq 1 -  u_t^\text{th}  \quad \text{(for } i \text{ in the set of time steps).} \\
& \text{Otherwise:} \\ 
& \sum_{i=t-D^\text{min,dn} + 1}^t w_i^\text{th}  \leq 1 - u_t^\text{th}
\end{align*}
```

Finally, multi temperature start/stop constraints are implemented using the following constraints:

```math
\begin{align*}
& v_t^\text{th} = x_t^\text{th} + y_t^\text{th} + z_t^\text{th}, \quad \forall t \in \{1, \dots, T\} \\
& z_t^\text{th} \le \sum_{i \in [D^\text{hot}, D^\text{warm})}w_{t-i}^\text{th}, \quad \forall t \in \{D^\text{warm}, \dots, T\} \\
& y_t^\text{th} \le \sum_{i \in [D^\text{warm}, D^\text{cold})}w_{t-i}^\text{th}, \quad \forall t \in \{D^\text{cold}, \dots, T\} \\
& (D^\text{warm} - 1) z_t^\text{th} + (1 - z_t^\text{th}) M^\text{big} \ge \sum_{i=1}^t (1 - u_i^\text{th}) + D^\text{init,hot}, \quad \forall t \in \{1, \dots, T\}  \\
& D^\text{hot} z_t^\text{th} \le \sum_{i=1}^t (1 - u_i^\text{th}) +  D^\text{init,hot}, \quad \forall t \in \{1, \dots, T\} \\
& (D^\text{cold} - 1) y_t^\text{th} + (1 - y_t^\text{th}) M^\text{big} \ge \sum_{i=1}^t (1 - u_i^\text{th}) + D^\text{init,warm}, \quad \forall t \in \{1, \dots, T\}  \\
& D^\text{warm} y_t^\text{th} \le \sum_{i=1}^t (1 - u_i^\text{th}) +  D^\text{init,warm}, \quad \forall t \in \{1, \dots, T\} \\
\end{align*}
```

* * *

## Valid configurations

Valid `DeviceModel`s for subtypes of `ThermalGen` include the following:

```@eval
using PowerSimulations
using PowerSystems
using DataFrames
using Latexify
combos = PowerSimulations.generate_device_formulation_combinations()
filter!(x -> x["device_type"] <: ThermalGen, combos)
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
