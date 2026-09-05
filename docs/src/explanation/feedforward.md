# [Feedforward](@id feedforward)

Chronologies define where information flows between models in a `Simulation`; feedforwards
define what to do with it. A feedforward is used to define what to do with information being
passed with an inter-stage chronology. The most common feedforward is the
`SemiContinuousFeedforward` that affects the semi-continuous range constraints of thermal
generators in an economic dispatch problem based on the value of the (already solved)
unit-commitment variables.

Creating a feedforward requires at least the `component_type` it applies to. The `source`
specifies which variable is read from the model that already solved (for example, the
commitment variable from the unit commitment problem). The `affected_values` specify which
variables are affected in the model still to be solved (for example, the next economic
dispatch problem).

```julia
SemiContinuousFeedforward(;
    component_type = ThermalStandard,
    source = OnVariable,
    affected_values = [ActivePowerVariable],
)
```

A `Simulation`'s `SimulationSequence` attaches feedforwards to the models they affect, keyed
by model name:

```julia
sequence = SimulationSequence(;
    models = models,
    feedforwards = Dict(
        "ED" => [
            SemiContinuousFeedforward(;
                component_type = ThermalStandard,
                source = OnVariable,
                affected_values = [ActivePowerVariable],
            ),
        ],
    ),
    ini_cond_chronology = InterProblemChronology(),
)
```

PSI owns this wiring. The feedforward types themselves, and the constraints they add to a
model, are `PowerOperationsModels.jl`'s — see its
[documentation](https://github.com/Sienna-Platform/PowerOperationsModels.jl) for the
constraint math.
