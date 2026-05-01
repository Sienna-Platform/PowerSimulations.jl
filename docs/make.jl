using Documenter
using PowerSystems
using PowerSimulations
using DataStructures
using DocumenterInterLinks
using Literate

links = InterLinks(
    "Julia" => "https://docs.julialang.org/en/v1/",
    "InfrastructureSystems" => "https://sienna-platform.github.io/InfrastructureSystems.jl/stable/",
    "PowerSystems" => "https://sienna-platform.github.io/PowerSystems.jl/stable/",
    "PowerSimulations" => "https://sienna-platform.github.io/PowerSimulations.jl/stable/",
    "StorageSystemsSimulations" => "https://sienna-platform.github.io/StorageSystemsSimulations.jl/stable/",
    "HydroPowerSimulations" => "https://sienna-platform.github.io/HydroPowerSimulations.jl/dev/",
)

include(joinpath(@__DIR__, "make_tutorials.jl"))
make_tutorials()

pages = OrderedDict(
    "Welcome Page" => "index.md",
    "Tutorials" => Any[
        "Single-step Problem" => "tutorials/generated_decision_problem.md",
        "Multi-stage Production Cost Simulation" => "tutorials/generated_pcm_simulation.md",
        "Dynamic Line Ratings" => "tutorials/generated_dynamic_line_ratings.md",
    ],
    "How to..." => Any[
        "...register a variable in a custom operation model" => "how_to/register_variable.md",
        "...create a problem template" => "how_to/problem_templates.md",
        "...read the simulation results" => "how_to/read_results.md",
        "...debug an infeasible model" => "how_to/debugging_infeasible_models.md",
        "...configure logging" => "how_to/logging.md",
        "...inspect simulation events using the recorder" => "how_to/simulation_recorder.md",
        "...run a parallel simulation" => "how_to/parallel_simulations.md",
    ],
    "Explanation" => Any[
        "explanation/psi_structure.md",
        "explanation/feedforward.md",
        "explanation/chronologies.md",
        "explanation/sequencing.md",
    ],
    "Reference" => Any[
        "Glossary and Acronyms" => "api/glossary.md",
        "Public API" => "api/PowerSimulations.md",
        "Developers" => ["Developer Guidelines" => "api/developer.md",
            "Internals" => "api/internal.md"],
    ],
    "Formulation Library" => Any[
        "Introduction" => "formulation_library/Introduction.md",
        "General" => "formulation_library/General.md",
        "Network" => "formulation_library/Network.md",
        "Thermal Generation" => "formulation_library/ThermalGen.md",
        "Renewable Generation" => "formulation_library/RenewableGen.md",
        "Load" => "formulation_library/Load.md",
        "Branch" => "formulation_library/Branch.md",
        "Source" => "formulation_library/Source.md",
        "Services" => "formulation_library/Service.md",
        "Feedforwards" => "formulation_library/Feedforward.md",
        "Piecewise Linear Cost" => "formulation_library/Piecewise.md",
    ],
)

makedocs(;
    modules = [PowerSimulations],
    format = Documenter.HTML(;
        prettyurls = haskey(ENV, "GITHUB_ACTIONS"),
        size_threshold = nothing),
    sitename = "PowerSimulations.jl",
    authors = "Jose Daniel Lara, Daniel Thom, Kate Doubleday, Rodrigo Henriquez-Auba, and Clayton Barrows",
    pages = Any[p for p in pages],
    plugins = [links],
)

deploydocs(;
    repo = "github.com/Sienna-Platform/PowerSimulations.jl.git",
    target = "build",
    branch = "gh-pages",
    devbranch = "main",
    devurl = "dev",
    push_preview = true,
    versions = ["stable" => "v^", "v#.#"],
)
