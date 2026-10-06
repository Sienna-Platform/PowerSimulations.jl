""" Supertype for initial condition chronologies """
abstract type InitialConditionChronology end

"""
    InterProblemChronology()

Type struct to select an information sharing model between stages that uses outputs from the most recent stage executed to calculate the initial conditions. This model takes into account solutions from stages defined with finer temporal resolutions

See also: [`IntraProblemChronology`](@ref)
"""
struct InterProblemChronology <: InitialConditionChronology end

"""
    IntraProblemChronology()

Type struct to select an information sharing model between stages that uses outputs from the same recent stage to calculate the initial conditions. This model ignores solutions from stages defined with finer temporal resolutions.

See also: [`InterProblemChronology`](@ref)
"""
struct IntraProblemChronology <: InitialConditionChronology end
