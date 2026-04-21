# PowerSimulations.jl

[![Main - CI](https://github.com/Sienna-Platform/PowerSimulations.jl/actions/workflows/main-tests.yml/badge.svg)](https://github.com/Sienna-Platform/PowerSimulations.jl/actions/workflows/main-tests.yml)
[![codecov](https://codecov.io/gh/Sienna-Platform/PowerSimulations.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/Sienna-Platform/PowerSimulations.jl)
[![Documentation](https://github.com/Sienna-Platform/PowerSimulations.jl/workflows/Documentation/badge.svg)](https://sienna-platform.github.io/PowerSimulations.jl/latest)
[![DOI](https://zenodo.org/badge/109443246.svg)](https://zenodo.org/badge/latestdoi/109443246)
[<img src="https://img.shields.io/badge/slack-@Sienna/PSI-sienna.svg?logo=slack">](https://join.slack.com/t/sienna-platform/shared_invite/zt-glam9vdu-o8A9TwZTZqqNTKHa7q3BpQ)
[![PowerSimulations.jl Downloads](https://img.shields.io/badge/dynamic/json?url=http%3A%2F%2Fjuliapkgstats.com%2Fapi%2Fv1%2Ftotal_downloads%2FPowerSimulations&query=total_requests&label=Downloads)](http://juliapkgstats.com/pkg/PowerSimulations)

`PowerSimulations.jl` is a Julia package for power system modeling and simulation of Power Systems operations. The objectives of the package are:

  - Provide a flexible modeling framework that can accommodate problems of different complexity and at different time-scales.

  - Streamline the construction of large scale optimization problems to avoid repetition of work when adding/modifying model details.
  - Exploit Julia's capabilities to improve computational performance of large scale power system quasi-static simulations.

The flexible modeling framework is enabled through a modular set of capabilities that enable scalable power system analysis and exploration of new analysis methods. The modularity of PowerSimulations results from the structure of the simulations enabled by the package:

  - _Simulations_ define a set of problems that can be solved using numerical techniques.

For example, an annual production cost modeling simulation can be created by formulating a unit commitment model against system data to assemble a set of 365 daily time-coupled scheduling problems.

## Simulations enabled by PowerSimulations

  - Integrated Resource Planning
  - Production Cost Modeling
  - Market Simulations

## Installation

```julia
julia> ]
(v1.9) pkg> add PowerSystems
(v1.9) pkg> add PowerSimulations
```

## Usage

`PowerSimulations.jl` uses [PowerSystems.jl](https://github.com/Sienna-Platform/PowerSystems.jl) to handle the data used in the simulations.

```julia
using PowerSimulations
using PowerSystems
```

For information on using the package, see the [stable documentation](https://sienna-platform.github.io/PowerSimulations.jl/stable/). Use the [in-development documentation](https://sienna-platform.github.io/PowerSimulations.jl/dev/) for the version of the documentation which contains the unreleased features.

## Development

Contributions to the development and enhancement of PowerSimulations is welcome. Please see [CONTRIBUTING.md](https://github.com/Sienna-Platform/PowerSimulations.jl/blob/main/CONTRIBUTING.md) for code contribution guidelines.

## License

PowerSimulations is released under a BSD [license](https://github.com/Sienna-Platform/PowerSimulations.jl/blob/main/LICENSE). PowerSimulations has been developed as part of the Scalable Integrated Infrastructure Planning (SIIP) initiative at the U.S. Department of Energy's National Renewable Energy Laboratory ([NREL](https://www.nrel.gov/)) Software Record SWR-23-104.
