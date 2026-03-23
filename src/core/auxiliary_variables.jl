"""
Auxiliary Variable for Thermal Generation Models to keep track of time elapsed on
"""
struct TimeDurationOn <: AuxVariableType end

"""
Auxiliary Variable for Thermal Generation Models to keep track of time elapsed off
"""
struct TimeDurationOff <: AuxVariableType end

"""
Auxiliary Variable for Thermal Generation Models that solve for power above min
"""
struct PowerOutput <: AuxVariableType end

"""
Auxiliary Variable for [``PowerLoadShift``](@ref) for the realized load after shifting
"""
struct RealizedLoad <: AuxVariableType end

"""
Auxiliary Variable of DC Current Variables for DC Lines formulations
Docs abbreviation: ``p_l^{loss}``
"""
struct DCLineLosses <: AuxVariableType end

"""
Auxiliary Variables that are calculated using a `PowerFlowEvaluationModel`
"""
abstract type PowerFlowAuxVariableType <: AuxVariableType end

"""
Auxiliary Variable for the bus angle results from power flow evaluation
"""
struct PowerFlowVoltageAngle <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the bus voltage magnitued results from power flow evaluation
"""
struct PowerFlowVoltageMagnitude <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for line power flow results from power flow evaluation
"""
abstract type BranchFlowAuxVariableType <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the line reactive flow in the from -> to direction from power flow evaluation
"""
struct PowerFlowBranchReactivePowerFromTo <: BranchFlowAuxVariableType end

"""
Auxiliary Variable for the line reactive flow in the to -> from direction from power flow evaluation
"""
struct PowerFlowBranchReactivePowerToFrom <: BranchFlowAuxVariableType end

"""
Auxiliary Variable for the line active flow in the from -> to direction from power flow evaluation
"""
struct PowerFlowBranchActivePowerFromTo <: BranchFlowAuxVariableType end

"""
Auxiliary Variable for the line active flow in the to -> from direction from power flow evaluation
"""
struct PowerFlowBranchActivePowerToFrom <: BranchFlowAuxVariableType end

"""
Auxiliary Variable for the loss factors from AC power flow evaluation that are calculated using the Jacobian matrix
"""
struct PowerFlowLossFactors <: PowerFlowAuxVariableType end

"""
Auxiliary Variable for the voltage stability factors from AC power flow evaluation that are calculated using the Jacobian matrix
"""
struct PowerFlowVoltageStabilityFactors <: PowerFlowAuxVariableType end

# should this be a subtype of BranchFlowAuxVariableType? It's line-related but has no flow direction.
"""
Auxiliary Variable for the active power loss on a line from AC power flow evaluation.
"""
struct PowerFlowBranchActivePowerLoss <: PowerFlowAuxVariableType end

# TODO reactive loss?

convert_result_to_natural_units(::Type{PowerOutput}) = true
convert_result_to_natural_units(
    ::Type{
        <:Union{
            PowerFlowBranchReactivePowerFromTo, PowerFlowBranchReactivePowerToFrom,
            PowerFlowBranchActivePowerFromTo, PowerFlowBranchActivePowerToFrom,
            PowerFlowBranchActivePowerLoss,
        },
    },
) = true

"Whether the auxiliary variable is calculated using a `PowerFlowEvaluationModel`"
is_from_power_flow(::Type{<:AuxVariableType}) = false
is_from_power_flow(::Type{<:PowerFlowAuxVariableType}) = true
