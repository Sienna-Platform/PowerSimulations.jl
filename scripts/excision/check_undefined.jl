# scripts/excision/check_undefined.jl
# Usage: julia --project=. scripts/excision/check_undefined.jl src/simulation/simulation.jl [more...]
# Loads src/PowerSimulations.jl into Main (definitions only), then walks each file's AST and
# reports identifiers not resolvable in the module. Heuristic: locally bound names are excluded.

const ROOT = normpath(joinpath(@__DIR__, "..", ".."))
Base.include(Main, joinpath(ROOT, "src", "PowerSimulations.jl"))
const MOD = Main.PowerSimulations
import InfrastructureOptimizationModels as IOM
import PowerOperationsModels as POM

const SKIP_HEADS = Set([:quote, :macrocall, :meta, :line, :inert])

function bindings!(acc::Set{Symbol}, ex)
    ex isa Symbol && (push!(acc, ex); return)
    ex isa Expr || return
    if ex.head == :(::)
        bindings!(acc, ex.args[1])
    elseif ex.head in (:tuple, :parameters, :kw, :(=), :curly, :(<:), :(...))
        ex.head == :kw && return bindings!(acc, ex.args[1])
        ex.head == :(=) && return bindings!(acc, ex.args[1])
        ex.head == :curly && return bindings!(acc, ex.args[1])
        ex.head == :(<:) && return bindings!(acc, ex.args[1])
        foreach(a -> bindings!(acc, a), ex.args)
    elseif ex.head == :call
        foreach(a -> bindings!(acc, a), ex.args[2:end])
    elseif ex.head == :where
        bindings!(acc, ex.args[1])
        foreach(a -> bindings!(acc, a), ex.args[2:end])
    end
    return
end

function function_name!(acc::Set{Symbol}, sig)
    sig isa Expr || return
    if sig.head == :where
        function_name!(acc, sig.args[1])
    elseif sig.head == :call
        sig.args[1] isa Symbol && push!(acc, sig.args[1])
    elseif sig.head == :(::) && length(sig.args) == 2
        function_name!(acc, sig.args[1])
    end
    return
end

function collect_locals!(acc::Set{Symbol}, ex)
    ex isa Expr || return
    if ex.head in (:function, :(->), :macro) && length(ex.args) >= 1
        bindings!(acc, ex.args[1])
        # A nested (non-top-level) function/macro definition binds its own name in the
        # enclosing scope; the module-level `isdefined(MOD, ...)` check never sees it.
        ex.head in (:function, :macro) && function_name!(acc, ex.args[1])
    elseif ex.head == :(=) && length(ex.args) == 2
        bindings!(acc, ex.args[1])
    elseif ex.head == :for
        bindings!(acc, ex.args[1])
    elseif ex.head == :where
        foreach(a -> bindings!(acc, a), ex.args[2:end])
    elseif ex.head == :do
        bindings!(acc, ex.args[2].args[1])
    elseif ex.head == :struct
        ex.args[2] isa Expr && bindings!(acc, ex.args[2])
        for f in ex.args[3].args
            f isa Expr && f.head == :(::) && bindings!(acc, f.args[1])
            f isa Symbol && push!(acc, f)
        end
    elseif ex.head == :try && length(ex.args) >= 2 && ex.args[2] isa Symbol
        push!(acc, ex.args[2])
    elseif ex.head == :let
        bindings!(acc, ex.args[1])
    end
    foreach(a -> collect_locals!(acc, a), ex.args)
    return
end

function collect_refs!(acc::Vector{Tuple{Symbol, Int}}, ex, line::Int)
    if ex isa Symbol
        push!(acc, (ex, line))
        return
    end
    ex isa Expr || return
    ex.head in SKIP_HEADS && return
    if ex.head == :(.) && length(ex.args) == 2
        collect_refs!(acc, ex.args[1], line)
        return
    end
    if ex.head == :kw
        collect_refs!(acc, ex.args[2], line)
        return
    end
    if ex.head == :parameters
        for a in ex.args
            a isa Expr && a.head == :kw ? collect_refs!(acc, a.args[2], line) :
            collect_refs!(acc, a, line)
        end
        return
    end
    for a in ex.args
        a isa LineNumberNode && (line = a.line; continue)
        collect_refs!(acc, a, line)
    end
    return
end

function check_file(path::String)
    src = read(path, String)
    ex = Meta.parseall(src; filename = path)
    locals = Set{Symbol}()
    collect_locals!(locals, ex)
    refs = Tuple{Symbol, Int}[]
    collect_refs!(refs, ex, 0)
    reported = Set{Symbol}()
    bad = Tuple{Symbol, Int}[]
    for (s, line) in refs
        s in locals && continue
        s in reported && continue
        s === :end && continue     # `a[end]` indexing placeholder, not a real binding
        s === :new && continue     # inner-constructor pseudo-function
        str = string(s)
        (startswith(str, "@") || startswith(str, "#") || startswith(str, ".")) && continue
        isdefined(MOD, s) && continue
        isdefined(Base, s) && continue
        isdefined(Core, s) && continue
        push!(reported, s)
        push!(bad, (s, line))
    end
    return bad
end

function check_shadowing()
    clashes = Symbol[]
    for n in names(MOD; all = true)
        isdefined(MOD, n) || continue
        v = getfield(MOD, n)
        v isa Function || continue
        parentmodule(v) === MOD || continue
        if (isdefined(IOM, n) && getfield(IOM, n) !== v) ||
           (isdefined(POM, n) && getfield(POM, n) !== v)
            push!(clashes, n)
        end
    end
    return clashes
end

function main(files)
    status = 0
    for f in files
        bad = check_file(f)
        isempty(bad) && (println("OK   ", f); continue)
        status = 1
        println("FAIL ", f)
        for (s, line) in sort(bad; by = last)
            println("  ", f, ":", line, "  ", s)
        end
    end
    clashes = check_shadowing()
    if !isempty(clashes)
        status = 1
        println("SHADOWED IOM/POM names (define as IOM.f / POM.f methods instead):")
        foreach(c -> println("  ", c), sort(clashes))
    end
    exit(status)
end

main(ARGS)
