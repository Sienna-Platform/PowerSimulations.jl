# Excision tooling

`check_undefined.jl` loads `src/PowerSimulations.jl` into `Main` and reports, per file,
identifiers that resolve nowhere. Run it on every file you edit in Phase 2:

    julia --project=. scripts/excision/check_undefined.jl src/simulation/simulation.jl

False positives happen for names bound by macros or by destructuring inside `do` blocks.
Triage each line; do not silence the tool. Temporary directory: delete after Phase 6.
