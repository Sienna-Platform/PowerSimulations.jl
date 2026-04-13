# SIIP Packages
using PowerSimulations
using PowerSystems
using PowerSystemCaseBuilder
using InfrastructureSystems
using PowerNetworkMatrices
using HydroPowerSimulations
import PowerSystemCaseBuilder: PSITestSystems
using PowerNetworkMatrices
using StorageSystemsSimulations
using PowerFlows
using DataFramesMeta

# Test Packages
using Test
using Logging

# Dependencies for testing
import PowerModels as PM
using DataFrames
using DataFramesMeta
using Dates
using JuMP
import JuMP.Containers: DenseAxisArray, SparseAxisArray
using TimeSeries
using CSV
import JSON3
using DataStructures
import UUIDs
using Random
import Serialization
import LinearAlgebra

import PowerSystems as PSY
import PowerSimulations as PSI
import PowerNetworkMatrices as PNM
import InfrastructureSystems as IS

const PFS = PowerFlows
const PSB = PowerSystemCaseBuilder
const ISOPT = IS.Optimization

const BASE_DIR = string(dirname(dirname(pathof(PowerSimulations))))
const DATA_DIR = joinpath(BASE_DIR, "test/test_data")

include("test_utils/common_operation_model.jl")
include("test_utils/model_checks.jl")
include("test_utils/mock_operation_models.jl")
include("test_utils/solver_definitions.jl")
include("test_utils/operations_problem_templates.jl")
include("test_utils/run_simulation.jl")
include("test_utils/add_components_to_system.jl")
include("test_utils/add_dlr_ts.jl")
include("test_utils/add_market_bid_cost.jl")
include("test_utils/mbc_system_utils.jl")
include("test_utils/mbc_simulation_utils.jl")
include("test_utils/events_simulation_utils.jl")
include("test_utils/iec_simulation_utils.jl")

ENV["RUNNING_PSI_TESTS"] = "true"
ENV["SIENNA_RANDOM_SEED"] = 1234  # Set a fixed seed for reproducibility in tests
