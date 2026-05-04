############################## Network Model Formulations ##################################
# These formulations are taken directly from PowerModels

abstract type AbstractPTDFModel <: PM.AbstractDCPModel end
"""
Linear active power approximation using the power transfer distribution factor [PTDF](https://sienna-platform.github.io/PowerNetworkMatrices.jl/stable/tutorials/tutorial_PTDF_matrix/) matrix.
"""
struct PTDFPowerModel <: AbstractPTDFModel end
"""
Infinite capacity approximation of network flow to represent entire system with a single node.
"""
struct CopperPlatePowerModel <: PM.AbstractActivePowerModel end
"""
Approximation to represent inter-area flow with each area represented as a single node.
"""
struct AreaBalancePowerModel <: PM.AbstractActivePowerModel end
"""
Linear active power approximation using the power transfer distribution factor [PTDF](https://sienna-platform.github.io/PowerNetworkMatrices.jl/stable/tutorials/tutorial_PTDF_matrix/) matrix. Balancing areas as well as synchronous regions.
"""
struct AreaPTDFPowerModel <: AbstractPTDFModel end

#================================================
    # exact non-convex models
    ACPPowerModel, ACRPowerModel, ACTPowerModel

    # linear approximations
    DCPPowerModel, NFAPowerModel

    # quadratic approximations
    DCPLLPowerModel, LPACCPowerModel

    # quadratic relaxations
    SOCWRPowerModel, SOCWRConicPowerModel,
    SOCBFPowerModel, SOCBFConicPowerModel,
    QCRMPowerModel, QCLSPowerModel,

    # sdp relaxations
    SDPWRMPowerModel
================================================#

##### Exact Non-Convex Models #####
import PowerModels: ACPPowerModel

import PowerModels: ACRPowerModel

import PowerModels: ACTPowerModel

##### Linear Approximations #####
import PowerModels: DCPPowerModel

import PowerModels: NFAPowerModel

##### Quadratic Approximations #####
import PowerModels: DCPLLPowerModel

import PowerModels: LPACCPowerModel

##### Quadratic Relaxations #####
import PowerModels: SOCWRPowerModel

import PowerModels: SOCWRConicPowerModel

import PowerModels: QCRMPowerModel

import PowerModels: QCLSPowerModel

supports_branch_filtering(::Type{<:PM.AbstractPowerModel}) = false
supports_branch_filtering(::Type{<:AbstractPTDFModel}) = true

ignores_branch_filtering(::Type{<:PM.AbstractPowerModel}) = false
ignores_branch_filtering(::Type{CopperPlatePowerModel}) = true
ignores_branch_filtering(::Type{AreaBalancePowerModel}) = true

requires_all_branch_models(::Type{<:PM.AbstractPowerModel}) = true
requires_all_branch_models(::Type{<:AbstractPTDFModel}) = false
requires_all_branch_models(::Type{CopperPlatePowerModel}) = false
requires_all_branch_models(::Type{AreaBalancePowerModel}) = false
