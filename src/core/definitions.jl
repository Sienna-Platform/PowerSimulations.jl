#################################################################################
# Type Alias for long type signatures
const MinMax = NamedTuple{(:min, :max), NTuple{2, Float64}}
const NamedMinMax = Tuple{String, MinMax}
const UpDown = NamedTuple{(:up, :down), NTuple{2, Float64}}
const InOut = NamedTuple{(:in, :out), NTuple{2, Float64}}

const BUILD_PROBLEMS_TIMER = TimerOutputs.TimerOutput()
const RUN_OPERATION_MODEL_TIMER = TimerOutputs.TimerOutput()
const RUN_SIMULATION_TIMER = TimerOutputs.TimerOutput()

# Type Alias for JuMP containers
const JuMPOrFloat = Union{JuMP.AbstractJuMPScalar, Float64}
const GAE = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}
const JuMPAffineExpressionArray = Matrix{GAE}
const JuMPAffineExpressionVector = Vector{GAE}
const JuMPConstraintArray = DenseAxisArray{JuMP.ConstraintRef}
const JuMPAffineExpressionDArrayStringInt = JuMP.Containers.DenseAxisArray{
    JuMP.AffExpr,
    2,
    Tuple{Vector{String}, UnitRange{Int64}},
    Tuple{
        JuMP.Containers._AxisLookup{Dict{String, Int64}},
        JuMP.Containers._AxisLookup{Tuple{Int64, Int64}},
    },
}
const JuMPAffineExpressionDArrayIntInt = JuMP.Containers.DenseAxisArray{
    JuMP.AffExpr,
    2,
    Tuple{Vector{Int64}, UnitRange{Int64}},
    Tuple{
        JuMP.Containers._AxisLookup{Dict{Int64, Int64}},
        JuMP.Containers._AxisLookup{Tuple{Int64, Int64}},
    },
}

const JuMPVariableTensor{N} = DenseAxisArray{
    JuMP.VariableRef,
    N,
    <:Tuple{Vector{String}, Vararg{Any}},  # last element is always UnitRange{Int64} but we cannot specify anything after the Vararg
    <:Tuple{
        JuMP.Containers._AxisLookup{Dict{String, Int64}},
        Vararg{JuMP.Containers._AxisLookup{<:Any}},
        # last element is always JuMP.Containers._AxisLookup{Tuple{Int64, Int64}} but we cannot specify anything after the Vararg
    },
}

const JuMPFloatMatrix = DenseAxisArray{Float64, 2}
const JuMPFloatArray = DenseAxisArray{Float64}
const JuMPVariableArray = DenseAxisArray{JuMP.VariableRef}
const JumpSupportedLiterals =
    Union{Number, Vector{<:Tuple{Number, Number}}, Tuple{Vararg{Number}}}

# Settings constants
const UNSET_HORIZON = Dates.Millisecond(0)
const UNSET_RESOLUTION = Dates.Millisecond(0)
const UNSET_INI_TIME = Dates.DateTime(0)

# Tolerance of comparisons
# MIP gap tolerances in most solvers are set to 1e-4
const ABSOLUTE_TOLERANCE = 1.0e-3
const BALANCE_SLACK_COST = 1e6
const CONSTRAINT_VIOLATION_SLACK_COST = 2e5
const SERVICES_SLACK_COST = 1e5
const COST_EPSILON = 1e-3
const MISSING_INITIAL_CONDITIONS_TIME_COUNT = 999.0
const SECONDS_IN_MINUTE = 60.0
const MINUTES_IN_HOUR = 60.0
const SECONDS_IN_HOUR = 3600.0
const MILLISECONDS_IN_HOUR = 3600000.0
const HOURS_IN_WEEK = 168.0
const MAX_START_STAGES = 3
const OBJECTIVE_FUNCTION_POSITIVE = 1.0
const OBJECTIVE_FUNCTION_NEGATIVE = -1.0
const INITIALIZATION_PROBLEM_HORIZON_COUNT = 3
# The DEFAULT_RESERVE_COST value is used to avoid degeneracy of the solutions, reserve cost isn't provided.
const DEFAULT_RESERVE_COST = 1.0
const DEFAULT_INTERPOLATION_LENGTH = 4
const KiB = 1024
const MiB = KiB * KiB
const GiB = MiB * KiB

const PSI_NAME_DELIMITER = "__"

const M_VALUE = 1e6

const NO_SERVICE_NAME_PROVIDED = ""
const UPPER_BOUND = "ub"
const LOWER_BOUND = "lb"
const MAX_OPTIMIZE_TRIES = 2

# File Names definitions
const PROBLEM_LOG_FILENAME = "operation_problem.log"
const SIMULATION_LOG_FILENAME = "simulation.log"
const REQUIRED_RECORDERS = (:simulation_status, :execution)
const KNOWN_SIMULATION_PATHS = [
    "data_store",
    "logs",
    "models_json",
    "problems",
    "recorder",
    "results",
    "simulation_files",
    "simulation_partitions",
]
"If the name of an extraneous file that appears in simulation results matches one of these regexes, it is safe to ignore"
const IGNORABLE_FILES = [
    r"^\.DS_Store$",
    r"^\.Trashes$",
    r"^\.Trash-.*$",
    r"^\.nfs.*$",
    r"^[Dd]esktop.ini$",
]
const RESULTS_DIR = "results"

# Enums
ModelBuildStatus = ISOPT.ModelBuildStatus
SimulationBuildStatus = IS.Simulation.SimulationBuildStatus

RunStatus = IS.Simulation.RunStatus

IS.@scoped_enum(SOSStatusVariable, NO_VARIABLE = 1, PARAMETER = 2, VARIABLE = 3,)

IS.@scoped_enum(COMPACT_PWL_STATUS, VALID = 1, INVALID = 2, UNDETERMINED = 3)

const ENUMS = (ModelBuildStatus, SimulationBuildStatus, RunStatus, SOSStatusVariable)

const ENUM_MAPPINGS = Dict()

for enum in ENUMS
    ENUM_MAPPINGS[enum] = Dict()
    for value in instances(enum)
        ENUM_MAPPINGS[enum][lowercase(string(value))] = value
    end
end

# Special cases for backwards compatibility
ENUM_MAPPINGS[RunStatus]["ready"] = RunStatus.INITIALIZED
ENUM_MAPPINGS[RunStatus]["successful"] = RunStatus.SUCCESSFULLY_FINALIZED

"""
Get the enum value for the string. Case insensitive.
"""
function get_enum_value(enum, value::String)
    if !haskey(ENUM_MAPPINGS, enum)
        throw(ArgumentError("enum=$enum is not valid"))
    end

    val = lowercase(value)
    if !haskey(ENUM_MAPPINGS[enum], val)
        throw(ArgumentError("enum=$enum does not have value=$val"))
    end

    return ENUM_MAPPINGS[enum][val]
end

Base.convert(::Type{SimulationBuildStatus}, val::String) =
    get_enum_value(SimulationBuildStatus, val)
Base.convert(::Type{ModelBuildStatus}, val::String) = get_enum_value(ModelBuildStatus, val)
Base.convert(::Type{RunStatus}, val::String) = get_enum_value(RunStatus, val)
Base.convert(::Type{SOSStatusVariable}, x::String) = get_enum_value(SOSStatusVariable, x)
