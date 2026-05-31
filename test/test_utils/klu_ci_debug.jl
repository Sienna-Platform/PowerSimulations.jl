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
            # Read the handles' internal dimension directly to detect a corrupted
            # or mismatched factorization before the (possibly fatal) ccall.
            #   klu_l_symbolic: n is at byte offset 40 (4 doubles + 1 ptr).
            #   klu_l_numeric:  n is at byte offset 0.
            sym_n = cache.symbolic == C_NULL ? -1 :
                    Base.unsafe_load(Base.reinterpret(Base.Ptr{Int64}, cache.symbolic), 6)
            num_n = cache.numeric == C_NULL ? -1 :
                    Base.unsafe_load(Base.reinterpret(Base.Ptr{Int64}, cache.numeric), 1)
            Base.println(
                Base.stderr,
                "[KLU-CI-TRACE] pre-solve Ti=$(Ti) n(ldim)=$(Int(n)) nrhs=$(Int(nrhs)) ",
                "Symbolic_n=$(sym_n) Numeric_n=$(num_n) ",
                "sizeof_common=$(Base.sizeof(eltype(typeof(cache.common)))) ",
                "status=$(Int(cache.common[].status)) ",
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

# --- Parallel AA (Apple Accelerate) solve-path instrumentation ---------------
# The AA backend crashes in the same Woodbury/PTDF solve path on macOS. AA is
# macOS-only, so this block only redefines on Apple. It dumps the opaque factor's
# internal state (status, dimensions, the numeric/symbolic factor pointers, and
# the required-vs-allocated solve-workspace bytes) right before the libSparse
# ccall — so a run under `MallocPreScribble` pinpoints which field is corrupt.
# Shares the same trace toggle as the KLU path.
if Sys.isapple()
    @eval PowerNetworkMatrices.AccelerateWrapper begin
        function solve!(cache::AAFactorCache, b::StridedVector{Cdouble})
            is_factored(cache) || error("AAFactorCache: not factored yet.")
            n = cache.n
            length(b) == n || throw(DimensionMismatch(
                "length(b) = $(length(b)), cache n = $(n)",
            ))
            stride(b, 1) == 1 || throw(ArgumentError("b must have unit stride."))
            ws = _ensure_solve_workspace!(cache, 1)
            if Main.PowerNetworkMatrices.KLUWrapper.PNM_KLU_CI_TRACE[]
                f = cache.numeric
                sf = f.symbolicFactorization
                req = _solve_workspace_bytes(f, 1)
                Base.println(
                    Base.stderr,
                    "[AA-CI-TRACE] pre-solve n=$(n) len_b=$(length(b)) ",
                    "fac_status=$(Int(f.status)) sym_status=$(Int(sf.status)) ",
                    "sym_rows=$(sf.rowCount) sym_cols=$(sf.columnCount) ",
                    "numFac=$(UInt(f.numericFactorization)) ",
                    "symFac=$(UInt(sf.factorization)) ",
                    "userFactorStorage=$(f.userFactorStorage) ",
                    "wsStatic=$(f.solveWorkspaceRequiredStatic) ",
                    "wsPerRHS=$(f.solveWorkspaceRequiredPerRHS) ",
                    "ws_bytes_req=$(req) ws_have_bytes=$(length(cache.solve_workspace) * 8) ",
                    "ws_ptr=$(UInt(ws)) nnz=$(cache.nnz) ",
                    "colStarts_end=$(cache.columnStarts[end]) ",
                    "n_rowIdx=$(length(cache.rowIndices))",
                )
                Base.flush(Base.stderr)
            end
            GC.@preserve cache _sparse_solve_vector_ws!(cache.numeric, _dense_vector(b), ws)
            return b
        end
    end
    @info "[AA-CI-DEBUG] AccelerateWrapper.solve! instrumented for AA diagnostics."
end
