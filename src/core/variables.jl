abstract type AbstractContingencyVariableType <: VariableType end
abstract type SparseVariableType <: VariableType end
abstract type InterpolationVariableType <: SparseVariableType end
abstract type BinaryInterpolationVariableType <: SparseVariableType end

"""
Struct to dispatch the creation of Active Power Variables

Docs abbreviation: ``p``
"""
struct ActivePowerVariable <: VariableType end

"""
Struct to dispatch the creation of Post-Contingency Active Power Change Variables.

Docs abbreviation: ``\\Delta p_{g,c}``
"""
struct PostContingencyActivePowerChangeVariable <: AbstractContingencyVariableType end

"""
Struct to dispatch the creation of Post-Contingency Active Power Deployment Variable for mapping reserves deployment under contingencies.

Docs abbreviation: ``\\Delta rsv_{r,g,c}``
"""
struct PostContingencyActivePowerReserveDeploymentVariable <:
       AbstractContingencyVariableType end

"""
Struct to dispatch the creation of Active Power Variables above minimum power for Thermal Compact formulations

Docs abbreviation: ``\\Delta p``
"""
struct PowerAboveMinimumVariable <: VariableType end

"""
Struct to dispatch the creation of Active Power Input Variables for 2-directional devices. For instance storage or pump-hydro

Docs abbreviation: ``p^\\text{in}``
"""
struct ActivePowerInVariable <: VariableType end

"""
Struct to dispatch the creation of Active Power Output Variables for 2-directional devices. For instance storage or pump-hydro

Docs abbreviation: ``p^\\text{out}``
"""
struct ActivePowerOutVariable <: VariableType end

"Multi-start startup variables"
abstract type MultiStartVariable <: VariableType end

"""
Struct to dispatch the creation of Hot Start Variable for Thermal units with temperature considerations

Docs abbreviation: ``z^\\text{th}``
"""
struct HotStartVariable <: MultiStartVariable end

"""
Struct to dispatch the creation of Warm Start Variable for Thermal units with temperature considerations

Docs abbreviation: ``y^\\text{th}``
"""
struct WarmStartVariable <: MultiStartVariable end

"""
Struct to dispatch the creation of Cold Start Variable for Thermal units with temperature considerations

Docs abbreviation: ``x^\\text{th}``
"""
struct ColdStartVariable <: MultiStartVariable end

"""
Struct to dispatch the creation of a variable for energy storage level (state of charge)

Docs abbreviation: ``e``
"""
struct EnergyVariable <: VariableType end

struct LiftVariable <: VariableType end

"""
Struct to dispatch the creation of a binary commitment status variable

Docs abbreviation: ``u``
"""
struct OnVariable <: VariableType end

"""
Struct to dispatch the creation of Reactive Power Variables

Docs abbreviation: ``q``
"""
struct ReactivePowerVariable <: VariableType end

"""
Struct to dispatch the creation of binary storage charge reservation variable

Docs abbreviation: ``u^\\text{st}``
"""
struct ReservationVariable <: VariableType end

"""
Struct to dispatch the creation of Active Power Reserve Variables

Docs abbreviation: ``r``
"""
struct ActivePowerReserveVariable <: VariableType end

"""
Struct to dispatch the creation of Service Requirement Variables

Docs abbreviation: ``\\text{req}``
"""
struct ServiceRequirementVariable <: VariableType end

"""
Struct to dispatch the creation of Binary Start Variables

Docs abbreviation: ``v``
"""
struct StartVariable <: VariableType end

"""
Struct to dispatch the creation of Binary Stop Variables

Docs abbreviation: ``w``
"""
struct StopVariable <: VariableType end

struct SteadyStateFrequencyDeviation <: VariableType end

struct AreaMismatchVariable <: VariableType end

struct DeltaActivePowerUpVariable <: VariableType end

struct DeltaActivePowerDownVariable <: VariableType end

struct AdditionalDeltaActivePowerUpVariable <: VariableType end

struct AdditionalDeltaActivePowerDownVariable <: VariableType end

struct SmoothACE <: VariableType end

"""
Struct to dispatch the creation of System-wide slack up variables. Used when there is not enough generation.

Docs abbreviation: ``p^\\text{sl,up}``
"""
struct SystemBalanceSlackUp <: VariableType end

"""
Struct to dispatch the creation of System-wide slack down variables. Used when there is not enough load curtailment.

Docs abbreviation: ``p^\\text{sl,dn}``
"""
struct SystemBalanceSlackDown <: VariableType end

"""
Struct to dispatch the creation of Reserve requirement slack variables. Used when there is not reserves in the system to satisfy the requirement.

Docs abbreviation: ``r^\\text{sl}``
"""
struct ReserveRequirementSlack <: VariableType end

"""
Struct to dispatch the creation of Voltage Magnitude Variables for AC formulations

Docs abbreviation: ``v``
"""
struct VoltageMagnitude <: VariableType end

"""
Struct to dispatch the creation of Voltage Angle Variables for AC/DC formulations

Docs abbreviation: ``\\theta``
"""
struct VoltageAngle <: VariableType end

#########################################
###### Power Load Shift Variables #######
#########################################

"""
Struct to dispatch the creation of Shifted Active Power Variables
Docs abbreviation: ``p^\\text{shift,up}``
"""
struct ShiftUpActivePowerVariable <: VariableType end

"""
Struct to dispatch the creation of Shifted Down Active Power Variables
Docs abbreviation: ``p^\\text{shift,dn}``
"""
struct ShiftDownActivePowerVariable <: VariableType end

#########################################
####### DC Converter Variables ##########
#########################################

"""
Struct to dispatch the variable of DC Current Variables for DC Lines formulations
Docs abbreviation: ``i_l^{dc}``
"""
struct DCLineCurrent <: VariableType end

"""
Struct to dispatch the creation of Voltage Variables for DC formulations
Docs abbreviation: ``v^{dc}``
"""
struct DCVoltage <: VariableType end

"""
Struct to dispatch the creation of Squared Voltage Variables for DC formulations
Docs abbreviation: ``v^{sq,dc}``
"""
struct SquaredDCVoltage <: VariableType end

"""
Struct to dispatch the creation of DC Converter Current Variables for DC formulations
Docs abbreviation: ``i_c^{dc}``
"""
struct ConverterCurrent <: VariableType end

"""
Struct to dispatch the creation of DC Converter Power Variables for DC formulations
Docs abbreviation: ``p_c^{dc}``
"""
struct ConverterDCPower <: VariableType end

"""
Struct to dispatch the creation of Squared DC Converter Current Variables for DC formulations
Docs abbreviation: ``i_c^{sq,dc}``
"""
struct SquaredConverterCurrent <: VariableType end

"""
Struct to dispatch the creation of DC Converter Positive Term Current Variables for DC formulations
Docs abbreviation: ``i_c^{+,dc}``
"""
struct ConverterPositiveCurrent <: VariableType end

"""
Struct to dispatch the creation of DC Converter Negative Term Current Variables for DC formulations
Docs abbreviation: ``i_c^{-,dc}``
"""
struct ConverterNegativeCurrent <: VariableType end

"""
Struct to dispatch the creation of DC Converter Binary for Absolute Value Current Variables for DC formulations
Docs abbreviation: `\\nu_c``
"""
struct ConverterCurrentDirection <: VariableType end

"""
Struct to dispatch the creation of Binary Variable for Converter Power Direction
Docs abbreviation: ``\\kappa_c^{dc}``
"""
struct ConverterPowerDirection <: VariableType end

"""
Struct to dispatch the creation of Auxiliary Variable for Converter Bilinear term: v * i
Docs abbreviation: ``\\gamma_c^{dc}``
"""
struct AuxBilinearConverterVariable <: VariableType end

"""
Struct to dispatch the creation of Auxiliary Variable for Squared Converter Bilinear term: v * i
    
Docs abbreviation: ``\\gamma_c^{sq,dc}``
"""
struct AuxBilinearSquaredConverterVariable <: VariableType end

"""
Struct to dispatch the creation of Continuous Interpolation Variable for Squared Converter Voltage
    
Docs abbreviation: ``\\delta_c^{v}``
"""
struct InterpolationSquaredVoltageVariable <: InterpolationVariableType end

"""
Struct to dispatch the creation of Binary Interpolation Variable for Squared Converter Voltage
    
Docs abbreviation: ``z_c^{v}``
"""
struct InterpolationBinarySquaredVoltageVariable <: BinaryInterpolationVariableType end

"""
Struct to dispatch the creation of Continuous Interpolation Variable for Squared Converter Current
    
Docs abbreviation: ``\\delta_c^{i}``
"""
struct InterpolationSquaredCurrentVariable <: InterpolationVariableType end

"""
Struct to dispatch the creation of Binary Interpolation Variable for Squared Converter Current
    
Docs abbreviation: ``z_c^{i}``
"""
struct InterpolationBinarySquaredCurrentVariable <: BinaryInterpolationVariableType end

"""
Struct to dispatch the creation of Continuous Interpolation Variable for Squared Converter AuxVar
    
Docs abbreviation: ``\\delta_c^{\\gamma}``
"""
struct InterpolationSquaredBilinearVariable <: InterpolationVariableType end

"""
Struct to dispatch the creation of Binary Interpolation Variable for Squared Converter AuxVar
    
Docs abbreviation: ``z_c^{\\gamma}``
"""
struct InterpolationBinarySquaredBilinearVariable <: BinaryInterpolationVariableType end

#########################################################
#########################################################

abstract type AbstractACActivePowerFlow <: VariableType end

abstract type AbstractACReactivePowerFlow <: VariableType end

"""
Struct to dispatch the creation of bidirectional Active Power Flow Variables

Docs abbreviation: ``f``
"""
struct FlowActivePowerVariable <: AbstractACActivePowerFlow end

# This Variable Type doesn't make sense since there are no lossless NetworkModels with ReactivePower.
# struct FlowReactivePowerVariable <: VariableType end

"""
Struct to dispatch the creation of unidirectional Active Power Flow Variables

Docs abbreviation: ``f^\\text{from-to}``
"""
struct FlowActivePowerFromToVariable <: AbstractACActivePowerFlow end

"""
Struct to dispatch the creation of unidirectional Active Power Flow Variables

Docs abbreviation: ``f^\\text{to-from}``
"""
struct FlowActivePowerToFromVariable <: AbstractACActivePowerFlow end

"""
Struct to dispatch the creation of unidirectional Reactive Power Flow Variables

Docs abbreviation: ``f^\\text{q,from-to}``
"""
struct FlowReactivePowerFromToVariable <: AbstractACReactivePowerFlow end

"""
Struct to dispatch the creation of unidirectional Reactive Power Flow Variables

Docs abbreviation: ``f^\\text{q,to-from}``
"""
struct FlowReactivePowerToFromVariable <: AbstractACReactivePowerFlow end

"""
Struct to dispatch the creation of active power flow upper bound slack variables. Used when there is not enough flow through the branch in the forward direction.

Docs abbreviation: ``f^\\text{sl,up}``
"""
struct FlowActivePowerSlackUpperBound <: AbstractACActivePowerFlow end

"""
Struct to dispatch the creation of active power flow lower bound slack variables. Used when there is not enough flow through the branch in the reverse direction.

Docs abbreviation: ``f^\\text{sl,lo}``
"""
struct FlowActivePowerSlackLowerBound <: AbstractACActivePowerFlow end

"""
Struct to dispatch the creation of Phase Shifters Variables

Docs abbreviation: ``\\theta^\\text{shift}``
"""
struct PhaseShifterAngle <: VariableType end

# Necessary as a work around for HVDCTwoTerminal models with losses
"""
Struct to dispatch the creation of HVDC Losses Auxiliary Variables

Docs abbreviation: ``\\ell``
"""
struct HVDCLosses <: VariableType end

"""
Struct to dispatch the creation of HVDC Flow Direction Auxiliary Variables

Docs abbreviation: ``u^\\text{dir}``
"""
struct HVDCFlowDirectionVariable <: VariableType end

"""
Struct to dispatch the creation of HVDC Received Flow at From Bus Variables for PWL formulations

Docs abbreviation: ``x``
"""
struct HVDCActivePowerReceivedFromVariable <: VariableType end

"""
Struct to dispatch the creation of HVDC Received Flow at To Bus Variables for PWL formulations

Docs abbreviation: ``y``
"""
struct HVDCActivePowerReceivedToVariable <: VariableType end

"""
Struct to dispatch the creation of HVDC Received Reactive Flow From Bus Variables

Docs abbreviation: ``x^r``
"""
struct HVDCReactivePowerReceivedFromVariable <: VariableType end

"""
Struct to dispatch the creation of HVDC Received Reactive Flow To Bus Variables

Docs abbreviation: ``y^i``
"""
struct HVDCReactivePowerReceivedToVariable <: VariableType end

"""
Struct to define the creation of HVDC Rectifier Delay Angle Variable

Docs abbreviation: ``\\alpha^r``
"""
struct HVDCRectifierDelayAngleVariable <: VariableType end

"""
Struct to define the creation of HVDC Inverter Extinction Angle Variable

Docs abbreviation: ``\\gamma^i``
"""
struct HVDCInverterExtinctionAngleVariable <: VariableType end

"""
Struct to define the creation of HVDC Rectifier Power Factor Angle Variable

Docs abbreviation: ``\\phi^r``
"""
struct HVDCRectifierPowerFactorAngleVariable <: VariableType end

"""
Struct to define the creation of HVDC Inverter Power Factor Angle Variable

Docs abbreviation: ``\\phi^i``
"""
struct HVDCInverterPowerFactorAngleVariable <: VariableType end

"""
Struct to define the creation of HVDC Rectifier Overlap Angle Variable

Docs abbreviation: ``\\mu^r``
"""
struct HVDCRectifierOverlapAngleVariable <: VariableType end

"""
Struct to define the creation of HVDC Inverter Overlap Angle Variable

Docs abbreviation: ``\\mu^i``
"""
struct HVDCInverterOverlapAngleVariable <: VariableType end

"""
Struct to define the creation of HVDC DC Line Voltage at Rectifier Side

Docs abbreviation: ``\\v_{d}^r``
"""
struct HVDCRectifierDCVoltageVariable <: VariableType end

"""
Struct to define the creation of HVDC DC Line Voltage at Inverter Side

Docs abbreviation: ``\\v_{d}^i``
"""
struct HVDCInverterDCVoltageVariable <: VariableType end

"""
Struct to define the creation of HVDC AC Line Current flowing into the AC side of Rectifier

Docs abbreviation: ``\\i_{ac}^r``
"""
struct HVDCRectifierACCurrentVariable <: VariableType end

"""
Struct to define the creation of HVDC AC Line Current flowing into the AC side of Inverter

Docs abbreviation: ``\\i_{ac}^i``
"""
struct HVDCInverterACCurrentVariable <: VariableType end

"""
Struct to define the creation of HVDC DC Line Current Flow

Docs abbreviation: ``\\i_{d}``
"""
struct DCLineCurrentFlowVariable <: VariableType end

"""
Struct to define the creation of HVDC Tap Setting at Rectifier Transformer

Docs abbreviation: ``\\t^r``
"""
struct HVDCRectifierTapSettingVariable <: VariableType end

"""
Struct to define the creation of HVDC Tap Setting at Inverter Transformer

Docs abbreviation: ``\\t^i``
"""
struct HVDCInverterTapSettingVariable <: VariableType end

"""
Struct to dispatch the creation of HVDC Piecewise Loss Variables

Docs abbreviation: ``h`` or ``w``
"""
struct HVDCPiecewiseLossVariable <: SparseVariableType end

"""
Struct to dispatch the creation of HVDC Piecewise Binary Loss Variables

Docs abbreviation: ``z``
"""
struct HVDCPiecewiseBinaryLossVariable <: SparseVariableType end

"""
Struct to dispatch the creation of piecewise linear cost variables for objective function

Docs abbreviation: ``\\delta``
"""
struct PiecewiseLinearCostVariable <: SparseVariableType end

abstract type AbstractPiecewiseLinearBlockOffer <: SparseVariableType end

"""
Struct to dispatch the creation of piecewise linear block incremental offer variables for objective function

Docs abbreviation: ``\\delta``
"""
struct PiecewiseLinearBlockIncrementalOffer <: AbstractPiecewiseLinearBlockOffer end

"""
Struct to dispatch the creation of piecewise linear block decremental offer variables for objective function

Docs abbreviation: ``\\delta_d``
"""
struct PiecewiseLinearBlockDecrementalOffer <: AbstractPiecewiseLinearBlockOffer end

"""
Struct to dispatch the creation of Interface Flow Slack Up variables

Docs abbreviation: ``f^\\text{sl,up}``
"""
struct InterfaceFlowSlackUp <: VariableType end
"""
Struct to dispatch the creation of Interface Flow Slack Down variables

Docs abbreviation: ``f^\\text{sl,dn}``
"""
struct InterfaceFlowSlackDown <: VariableType end

"""
Struct to dispatch the creation of Slack variables for UpperBoundFeedforward

Docs abbreviation: ``p^\\text{ff,ubsl}``
"""
struct UpperBoundFeedForwardSlack <: VariableType end
"""
Struct to dispatch the creation of Slack variables for LowerBoundFeedforward

Docs abbreviation: ``p^\\text{ff,lbsl}``
"""
struct LowerBoundFeedForwardSlack <: VariableType end

"""
Struct to dispatch the creation of Slack variables for rate of change constraints up limits

Docs abbreviation: ``p^\\text{sl,up}``
"""
struct RateofChangeConstraintSlackUp <: VariableType end
"""
Struct to dispatch the creation of Slack variables for rate of change constraints down limits

Docs abbreviation: ``p^\\text{sl,dn}``
"""
struct RateofChangeConstraintSlackDown <: VariableType end

const MULTI_START_VARIABLES = Tuple(IS.get_all_concrete_subtypes(PSI.MultiStartVariable))

should_write_resulting_value(::Type{PiecewiseLinearCostVariable}) = false
should_write_resulting_value(::Type{PiecewiseLinearBlockIncrementalOffer}) = false
should_write_resulting_value(::Type{PiecewiseLinearBlockDecrementalOffer}) = false
should_write_resulting_value(::Type{HVDCPiecewiseLossVariable}) = false
should_write_resulting_value(::Type{HVDCPiecewiseBinaryLossVariable}) = false
function should_write_resulting_value(
    ::Type{T},
) where {T <: Union{InterpolationVariableType, BinaryInterpolationVariableType}}
    return false
end
convert_result_to_natural_units(::Type{ActivePowerVariable}) = true
convert_result_to_natural_units(::Type{PostContingencyActivePowerChangeVariable}) = true
convert_result_to_natural_units(::Type{PowerAboveMinimumVariable}) = true
convert_result_to_natural_units(::Type{ActivePowerInVariable}) = true
convert_result_to_natural_units(::Type{ActivePowerOutVariable}) = true
convert_result_to_natural_units(::Type{EnergyVariable}) = true
convert_result_to_natural_units(::Type{ReactivePowerVariable}) = true
convert_result_to_natural_units(::Type{ActivePowerReserveVariable}) = true
convert_result_to_natural_units(
    ::Type{PostContingencyActivePowerReserveDeploymentVariable},
) = true
convert_result_to_natural_units(::Type{ServiceRequirementVariable}) = true
convert_result_to_natural_units(::Type{RateofChangeConstraintSlackUp}) = true
convert_result_to_natural_units(::Type{RateofChangeConstraintSlackDown}) = true
convert_result_to_natural_units(::Type{AreaMismatchVariable}) = true
convert_result_to_natural_units(::Type{DeltaActivePowerUpVariable}) = true
convert_result_to_natural_units(::Type{DeltaActivePowerDownVariable}) = true
convert_result_to_natural_units(::Type{AdditionalDeltaActivePowerUpVariable}) = true
convert_result_to_natural_units(::Type{AdditionalDeltaActivePowerDownVariable}) = true
convert_result_to_natural_units(::Type{SmoothACE}) = true
convert_result_to_natural_units(::Type{SystemBalanceSlackUp}) = true
convert_result_to_natural_units(::Type{SystemBalanceSlackDown}) = true
convert_result_to_natural_units(::Type{ReserveRequirementSlack}) = true
convert_result_to_natural_units(::Type{FlowActivePowerVariable}) = true
convert_result_to_natural_units(::Type{FlowActivePowerFromToVariable}) = true
convert_result_to_natural_units(::Type{FlowActivePowerToFromVariable}) = true
convert_result_to_natural_units(::Type{FlowReactivePowerFromToVariable}) = true
convert_result_to_natural_units(::Type{FlowReactivePowerToFromVariable}) = true
convert_result_to_natural_units(::Type{HVDCLosses}) = true
convert_result_to_natural_units(::Type{InterfaceFlowSlackUp}) = true
convert_result_to_natural_units(::Type{InterfaceFlowSlackDown}) = true
