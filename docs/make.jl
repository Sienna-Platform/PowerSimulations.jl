using Documenter
using PowerSystems
using PowerSimulations
using DataStructures
using DocumenterInterLinks
using Downloads: download
using Literate

# DocumenterInterLinks fetches each remote `objects.inv` through DocInventories,
# which defaults to a 1-second timeout — CI runners occasionally miss that on a
# cold connection to GitHub Pages, failing the whole doc build. Pre-fetch each
# inventory to a local file with a generous timeout and hand InterLinks that as
# the primary source (as a Tuple, so InterLinks still applies its normal
# method-to-function aliasing); fall back to the live URL if the pre-fetch fails.
const inventory_cache_dir = mktempdir()

function inventory_source(root_url; inventory_file = "objects.inv")
    url = root_url * inventory_file
    local_path = joinpath(inventory_cache_dir, replace(root_url, r"[^A-Za-z0-9]" => "_"))
    try
        download(url, local_path; timeout = 30.0)
        return (root_url, local_path, url)
    catch exception
        @warn "Failed to pre-fetch inventory" root_url exception
        return (root_url, url)
    end
end

links = InterLinks(
    "Julia" => "https://docs.julialang.org/en/v1/",
    "InfrastructureSystems" =>
        inventory_source(
            "https://sienna-platform.github.io/InfrastructureSystems.jl/stable/",
        ),
    "PowerSystems" =>
        inventory_source("https://sienna-platform.github.io/PowerSystems.jl/stable/"),
    "PowerSimulations" =>
        inventory_source("https://sienna-platform.github.io/PowerSimulations.jl/stable/"),
    "PowerSystemCaseBuilder" =>
        inventory_source(
            "https://sienna-platform.github.io/PowerSystemCaseBuilder.jl/stable/",
        ),
    # InfrastructureOptimizationModels has no published doc inventory yet (stable and dev
    # both 404 as of this excision); omit rather than break the build. PowerOperationsModels
    # publishes only a dev inventory so far.
    "PowerOperationsModels" =>
        inventory_source("https://sienna-platform.github.io/PowerOperationsModels.jl/dev/"),
)

include(joinpath(@__DIR__, "make_tutorials.jl"))
make_tutorials()

pages = OrderedDict(
    "Welcome Page" => "index.md",
    "Tutorials" => Any[
        "Multi-stage Production Cost Simulation" => "tutorials/generated_pcm_simulation.md",
    ],
    "How to..." => Any[
        "...read the simulation results" => "how_to/read_results.md",
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
