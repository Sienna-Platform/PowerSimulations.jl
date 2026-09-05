# [Read results](@id read_results)

Once a `DecisionModel` is solved via `solve!(model)` or a Simulation is executed (and solved) via `execute!(simulation)`, the results are stored and can be accessed directly in the REPL for result exploration and plotting.

## Read results of a Decision Problem

Once a `DecisionModel` is solved, results are accessed using `OptimizationProblemOutputs(model)` as follows:

```julia
# The DecisionModel is already constructed
build!(model; output_dir = mktempdir())
solve!(model)

results = OptimizationProblemOutputs(model)
```

The output will showcase the available expressions, parameters and variables to read. For example it will look like:

```raw
Start: 2020-01-01T00:00:00
End: 2020-01-03T23:00:00
Resolution: 60 minutes

PowerSimulations Problem Auxiliary variables Results
┌────────────────────────────────┐
│ TimeDurationOn__ThermalStandard │
│ TimeDurationOff__ThermalStandard│
└────────────────────────────────┘

PowerSimulations Problem Expressions Results
┌─────────────────────────────────────────────┐
│ ProductionCostExpression__RenewableDispatch │
│ ProductionCostExpression__ThermalStandard   │
└─────────────────────────────────────────────┘

PowerSimulations Problem Duals Results
┌──────────────────────────────────────┐
│ CopperPlateBalanceConstraint__System │
└──────────────────────────────────────┘

PowerSimulations Problem Parameters Results
┌────────────────────────────────────────────────────────────────────┐
│ ActivePowerTimeSeriesParameter__RenewableNonDispatch                │
│ RequirementTimeSeriesParameter__OnlineReserve__ReserveUp__Spin_Up_R3│
│ RequirementTimeSeriesParameter__OnlineReserve__ReserveUp__Reg_Up    │
│ ActivePowerTimeSeriesParameter__PowerLoad                           │
│ ActivePowerTimeSeriesParameter__RenewableDispatch                   │
│ RequirementTimeSeriesParameter__OnlineReserve__ReserveDown__Reg_Down│
│ RequirementTimeSeriesParameter__OnlineReserve__ReserveUp__Spin_Up_R1│
│ RequirementTimeSeriesParameter__OnlineReserve__ReserveUp__Spin_Up_R2│
└────────────────────────────────────────────────────────────────────┘

PowerSimulations Problem Variables Results
┌───────────────────────────────────────────────────────────────────┐
│ ActivePowerReserveVariable__OnlineReserve__ReserveUp__Spin_Up_R1   │
│ SystemBalanceSlackUp__System                                      │
│ ActivePowerReserveVariable__OnlineReserve__ReserveUp__Reg_Up       │
│ StopVariable__ThermalStandard                                      │
│ ActivePowerVariable__RenewableDispatch                             │
│ ActivePowerReserveVariable__OnlineReserve__ReserveDown__Reg_Down   │
│ SystemBalanceSlackDown__System                                     │
│ ActivePowerReserveVariable__OnlineReserve__ReserveUp__Spin_Up_R2   │
│ ActivePowerVariable__ThermalStandard                               │
│ StartVariable__ThermalStandard                                     │
│ ActivePowerReserveVariable__OnlineReserve__ReserveUp__Spin_Up_R3   │
│ OnVariable__ThermalStandard                                        │
└───────────────────────────────────────────────────────────────────┘
```

Then the following code can be used to read results:

```julia
# Read active power of Thermal Standard
thermal_active_power = read_variable(results, "ActivePowerVariable__ThermalStandard")

# Read max active power parameter of RenewableDispatch
renewable_param =
    read_parameter(results, "ActivePowerTimeSeriesParameter__RenewableDispatch")

# Read cost expressions of ThermalStandard units
cost_thermal = read_expression(results, "ProductionCostExpression__ThermalStandard")

# Read dual variables
dual_balance_constraint = read_dual(results, "CopperPlateBalanceConstraint__System")

# Read auxiliary variables
aux_var_result = read_aux_variable(results, "TimeDurationOn__ThermalStandard")
```

Results will be in the form of DataFrames that can be easily explored.

## Read results of a Simulation

```julia
# The Simulation is already constructed
build!(sim)
execute!(sim; enable_progress_bar = true)

results_sim = SimulationResults(sim)
```

As an example, the `SimulationResults` printing will look like:

```raw
Decision Problem Results
┌──────────────┬─────────────────────┬──────────────┬─────────────────────────┐
│ Problem Name │ Initial Time        │ Resolution   │ Last Solution Timestamp │
├──────────────┼─────────────────────┼──────────────┼─────────────────────────┤
│ ED           │ 2020-10-02T00:00:00 │ 60 minutes   │ 2020-10-09T23:00:00     │
│ UC           │ 2020-10-02T00:00:00 │ 1440 minutes │ 2020-10-09T00:00:00     │
└──────────────┴─────────────────────┴──────────────┴─────────────────────────┘

Emulator Results
┌─────────────────┬───────────┐
│ Name            │ Emulator  │
│ Resolution      │ 5 minutes │
│ Number of steps │ 2304      │
└─────────────────┴───────────┘
```

With this, it is possible to obtain results of each `DecisionModel` and `EmulationModel` as follows:

```julia
# Use the Problem Name for Decision Problems
results_uc = get_decision_problem_results(results_sim, "UC")
results_ed = get_decision_problem_results(results_sim, "ED")
results_emulator = get_emulation_problem_results(results_sim)
```

Once we have each decision (or emulation) problem results, we can explore directly using the approach for Decision Models, mentioned in the previous section.

### Reading solutions for all simulation steps

In this case, using `read_variable` (or read expression, parameter or dual), will return a dictionary of all steps (of that Decision Problem). For example, the following code:

```julia
thermal_active_power = read_variable(results_uc, "ActivePowerVariable__ThermalStandard")
```

will return:

```
DataStructures.SortedDict{Any, Any, Base.Order.ForwardOrdering} with 8 entries:
  DateTime("2020-10-02T00:00:00") => 72×54 DataFrame…
  DateTime("2020-10-03T00:00:00") => 72×54 DataFrame…
  DateTime("2020-10-04T00:00:00") => 72×54 DataFrame…
  DateTime("2020-10-05T00:00:00") => 72×54 DataFrame…
  DateTime("2020-10-06T00:00:00") => 72×54 DataFrame…
  DateTime("2020-10-07T00:00:00") => 72×54 DataFrame…
  DateTime("2020-10-08T00:00:00") => 72×54 DataFrame…
  DateTime("2020-10-09T00:00:00") => 72×54 DataFrame…
```

That is, a sorted dictionary for each simulation step, using as a key the initial timestamp for that specific simulation step.

Note that in this case, each DataFrame, has a dimension of ``72 \times 54``, since the horizon is 72 hours (number of rows), but the interval is only 24 hours. Indeed, note the initial timestamp of each simulation step is the beginning of each day, i.e. 24 hours. Finally, there 54 columns, since this example system has 53 `ThermalStandard` units (plus 1 column for the timestamps). The user is free to explore the solution of any simulation step as needed.

### Reading the "realized" solution (i.e. the interval)

Using `read_realized_variable` (or read realized expression, parameter or dual), will return the DataFrame of the realized solution of any specific variable. That is, it will concatenate the corresponding simulation step with the specified interval of that step, to construct a single DataFrame with the "realized solution" of the entire simulation.

For example, the code:

```julia
th_realized_power =
    read_realized_variable(results_uc, "ActivePowerVariable__ThermalStandard")
```

will return:

```raw
92×54 DataFrame
 Row │ DateTime             322_CT_6      321_CC_1  202_STEAM_3   223_CT_4  123_STEAM_2    213_CT_1  223_CT_6  313_CC_1  101_STEAM_3  123_C ⋯
     │ DateTime             Float64       Float64   Float64       Float64   Float64        Float64   Float64   Float64   Float64      Float ⋯
─────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
   1 │ 2020-10-02T00:00:00   0.0           293.333   0.0               0.0    0.0               0.0       0.0   231.667      76.0     0.0   ⋯
   2 │ 2020-10-02T01:00:00   0.0           267.552   0.0               0.0    0.0               0.0       0.0   231.667      76.0     0.0
   3 │ 2020-10-02T02:00:00   0.0           234.255   0.0               0.0   -4.97544e-11       0.0       0.0   231.667      76.0     0.0
   4 │ 2020-10-02T03:00:00   0.0           249.099   0.0               0.0   -4.97544e-11       0.0       0.0   231.667      76.0     0.0
   5 │ 2020-10-02T04:00:00   0.0           293.333   0.0               0.0   -4.97544e-11       0.0       0.0   231.667      76.0     0.0   ⋯
   6 │ 2020-10-02T05:00:00   0.0           293.333   1.27578e-11       0.0   -4.97544e-11       0.0       0.0   293.333      76.0     0.0
  ⋮  │          ⋮                ⋮           ⋮           ⋮           ⋮            ⋮           ⋮         ⋮         ⋮           ⋮             ⋱
 187 │ 2020-10-09T18:00:00   0.0           293.333  76.0               0.0  155.0               0.0       0.0   318.843      76.0     0.0
 188 │ 2020-10-09T19:00:00   0.0           293.333  76.0               0.0  124.0               0.0       0.0   293.333      76.0     0.0
 189 │ 2020-10-09T20:00:00   0.0           293.333  60.6667            0.0  124.0               0.0       0.0     0.0        76.0     0.0   ⋯
 190 │ 2020-10-09T21:00:00  -7.65965e-12   293.333  60.6667            0.0  124.0               0.0       0.0     0.0        76.0     0.0
 191 │ 2020-10-09T22:00:00   0.0             0.0    60.6667            0.0  124.0               0.0       0.0     0.0        76.0     7.156
 192 │ 2020-10-09T23:00:00   0.0             0.0    60.6667            0.0  117.81              0.0       0.0     0.0        76.0     0.0
                                                                                                              44 columns and 180 rows omitted
```

In this case, the 8 simulation steps of 24 hours (192 hours), in a single DataFrame, to enable easy exploration of the realized results for the user.
