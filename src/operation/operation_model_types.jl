"""
Abstract type for models than employ PowerSimulations methods. For custom decision problems
    use DecisionProblem as the super type.
"""
abstract type DefaultDecisionProblem <: DecisionProblem end

"""
Generic PowerSimulations Operation Problem Type for unspecified models
"""
struct GenericOpProblem <: DefaultDecisionProblem end

"""
Abstract type for models than employ PowerSimulations methods. For custom emulation problems
    use EmulationProblem as the super type.
"""
abstract type DefaultEmulationProblem <: EmulationProblem end

"""
Default PowerSimulations Emulation Problem Type for unspecified problems
"""
struct GenericEmulationProblem <: DefaultEmulationProblem end
