# PowerSimulations.jl

```@meta
CurrentModule = PowerSimulations
```

## Overview

`PowerSimulations.jl` is a power system operations simulation tool developed as a flexible and open source software for quasi-static power systems simulations including Production Cost Models. `PowerSimulations.jl` tackles the issues of developing a simulation model in a modular way providing tools for the formulation of decision models and emulation models that can be solved independently or in an interconnected fashion.

`PowerSimulations.jl` supports the workflows to develop simulations by separating the development
of operations models and simulation models.

  - **Operation Models**: Optimization model used to find the solution of an operation problem.
  - **Simulations Models**: Defined the requirements to find solutions to a sequence of operation problems in a way that resembles the procedures followed by operators.

The most common Simulation Model is the solution of a Unit Commitment and Economic Dispatch sequence of problems. This model is used in commercial Production Cost Modeling tools, but it has a limited scope of analysis.

`PowerSimulations.jl` is an active project under development, and we welcome your feedback, suggestions, and bug reports.

## About Sienna

`PowerSimulations.jl` is part of the National Renewable Energy Laboratory's
[Sienna ecosystem](https://sienna-platform.github.io/Sienna/), an open source framework for
power system modeling, simulation, and optimization. The Sienna ecosystem can be
[found on Github](https://github.com/Sienna-Platform/Sienna). It contains three applications:

  - [Sienna\Data](https://sienna-platform.github.io/Sienna/pages/applications/sienna_data.html) enables
    efficient data input, analysis, and transformation
  - [Sienna\Ops](https://sienna-platform.github.io/Sienna/pages/applications/sienna_ops.html) enables
    enables system scheduling simulations by formulating and solving optimization problems
  - [Sienna\Dyn](https://sienna-platform.github.io/Sienna/pages/applications/sienna_dyn.html) enables
    system transient analysis including small signal stability and full system dynamic
    simulations

Each application uses multiple packages in the [`Julia`](http://www.julialang.org)
programming language.

## Installation and Quick Links

  - [Sienna installation page](https://sienna-platform.github.io/Sienna/SiennaDocs/docs/build/how-to/install/):
    Instructions to install `PowerSimulations.jl` and other Sienna\Ops packages
  - [`JuMP.jl` solver's page](https://jump.dev/JuMP.jl/stable/installation/#Install-a-solver): An appropriate optimization solver is required for running `PowerSimulations.jl` models. Refer to this page to select and install a solver for your application.
  - [Sienna Documentation Hub](https://sienna-platform.github.io/Sienna/SiennaDocs/docs/build/index.html):
    Links to other Sienna packages' documentation

## How To Use This Documentation

There are five main sections containing different information:

  - **Tutorials** - Detailed walk-throughs to help you *learn* how to use
    `PowerSimulations.jl`
  - **How to...** - Directions to help *guide* your work for a particular task
  - **Explanation** - Additional details and background information to help you *understand*
    `PowerSimulations.jl`, its structure, and how it works behind the scenes
  - **Reference** - Technical references and API for a quick *look-up* during your work
  - **Formulation Library** - Technical reference for the variables, parameters, and
  equations that PowerSimulations.jl uses to define device behavior

`PowerSimulations.jl` strives to follow the [Diataxis](https://diataxis.fr/) documentation
framework.
