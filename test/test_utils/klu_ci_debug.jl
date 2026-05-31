# TEMPORARY CI DIAGNOSTIC — remove before merge.
#
# PR #1619 fails on Linux/KLU with `ArgumentError: KLU klu_solve failed: invalid
# argument` (KLU_INVALID) followed by SIGSEGV, in the security-constrained MODF
# Woodbury build. The crash does not reproduce on macOS (heap-layout luck), so we
# capture the deterministic Linux signal directly in CI.
#
# CI builds PSI against the *registered* PowerNetworkMatrices 0.22.0, so we cannot
# edit PNM source for CI. Instead we redefine `KLUWrapper.solve!` at runtime to:
#   - print the full klu_solve precondition snapshot when the ccall returns FALSE
#     (which {Numeric, Symbolic, B} is NULL? what is ldim / nrhs / status?), then
#     throw exactly as the original does, and
#   - optionally trace every solve's dimensions (gated by PNM_KLU_CI_TRACE[]) so
#     the LAST line before a SIGSEGV pinpoints the crashing call.
#
# All output goes to stderr with an explicit flush so it survives a segfault.

@eval PowerNetworkMatrices.KLUWrapper begin
    # Toggled true only around the crashing testset (see
    # test_ac_transmission_security_constrained_models.jl) to bound trace volume.
    const PNM_KLU_CI_TRACE = Base.Ref(false)

    function solve!(
        cache::KLULinSolveCache{Tv, Ti},
        B::StridedVecOrMat{Tv},
    ) where {Tv, Ti}
        is_factored(cache) || error("KLULinSolveCache: not factored yet.")
        n = _dim(cache)
        size(B, 1) == Int(n) || throw(DimensionMismatch(
            "size(B, 1) = $(size(B, 1)), cache n = $(Int(n))",
        ))
        stride(B, 1) == 1 || throw(ArgumentError(
            "B must have unit stride in the first dimension.",
        ))
        nrhs = size(B, 2)
        nrhs == 0 && return B
        if PNM_KLU_CI_TRACE[]
            Base.println(
                Base.stderr,
                "[KLU-CI-TRACE] pre-solve n(ldim)=$(Int(n)) nrhs=$(Int(nrhs)) ",
                "sym_null=$(cache.symbolic == C_NULL) ",
                "num_null=$(cache.numeric == C_NULL) ",
                "cache_id=$(objectid(cache)) ",
                "sym=$(UInt(cache.symbolic)) num=$(UInt(cache.numeric)) ",
                "B=$(UInt(pointer(B))) size(B)=$(size(B))",
            )
            Base.flush(Base.stderr)
        end
        ok = _solve_call(
            Tv, Ti, cache.symbolic, cache.numeric, n, nrhs, pointer(B), cache.common,
        )
        if ok == 0
            Base.println(
                Base.stderr,
                "[KLU-CI-FAIL] klu_solve returned FALSE: ",
                "status=$(Int(cache.common[].status)) ",
                "n(ldim)=$(Int(n)) nrhs=$(Int(nrhs)) ",
                "sym_null=$(cache.symbolic == C_NULL) ",
                "num_null=$(cache.numeric == C_NULL) ",
                "B_null=$(pointer(B) == C_NULL) ",
                "cache_id=$(objectid(cache)) ",
                "sym=$(UInt(cache.symbolic)) num=$(UInt(cache.numeric)) ",
                "size(B)=$(size(B)) tid=$(Threads.threadid())",
            )
            Base.flush(Base.stderr)
            klu_throw(cache.common[], "klu_solve")
        end
        return B
    end
end

@info "[KLU-CI-DEBUG] KLUWrapper.solve! instrumented for CI diagnostics."
