# Pad `ixs` with `:` for any unindexed middle dimensions of `dest` (Python `...`-style).
# Fast path for AbstractArrays where `N` is known at compile time → allocation-free `Val(N - K)`.
@inline function expand_ixs(
    ixs::NTuple{K, Any},
    dest::AbstractArray{<:Any, N},
) where {K, N}
    K <= N || throw(ArgumentError("`ixs` must not index more dimensions than `dest` has"))
    K == N && return ixs
    # Single-element ixs is the leading axis; multi-element preserves first..last with `:` filling the middle.
    K == 1 && return (only(ixs), ntuple(_ -> Colon(), Val(N - 1))...)
    return (Base.front(ixs)..., ntuple(_ -> Colon(), Val(N - K))..., last(ixs))
end

# Fallback for non-AbstractArray containers (e.g. `HDF5.Dataset`) — `ndims` resolved at runtime.
@inline function expand_ixs(ixs::NTuple{K, Any}, dest) where {K}
    N = ndims(dest)
    K <= N || throw(ArgumentError("`ixs` must not index more dimensions than `dest` has"))
    K == N && return ixs
    K == 1 && return (only(ixs), ntuple(_ -> Colon(), N - 1)...)
    return (Base.front(ixs)..., ntuple(_ -> Colon(), N - K)..., last(ixs))
end

# Concrete fast path: scalar `src` with a fully-specified index tuple goes through `setindex!`.
@inline function assign_maybe_broadcast!(
    dest::DenseAxisArray{T, N},
    src::T,
    ixs::NTuple{N, Any},
) where {T, N}
    dest[ixs...] = src
    return
end

# Array `src`: assign a slice of `dest` from `src` (standard assignment handles non-1:n axes).
# `dest` is left untyped so non-`AbstractArray` containers (e.g. `HDF5.Dataset`) are also accepted.
@inline function assign_maybe_broadcast!(dest, src::AbstractArray, ixs::Tuple)
    dest[expand_ixs(ixs, dest)...] = src
    return
end

# Scalar/tuple `src`: broadcast the value across the indexed slice of `dest`.
@inline function assign_maybe_broadcast!(dest, src, ixs::Tuple)
    expanded = expand_ixs(ixs, dest)
    @views dest[expanded...] .= src
    return
end

# Similar to assign_maybe_broadcast! but for fixing JuMP VariableRefs
@inline function fix_maybe_broadcast!(
    dest::DenseAxisArray{JuMP.VariableRef, N},
    src::Float64,
    ixs::NTuple{N, Any},
) where {N}
    JuMP.fix(dest[ixs...], src; force = true)
    return
end

fix_expand(dest, src, ixs::Tuple) =
    fix_parameter_value.(dest[expand_ixs(ixs, dest)...], src)
fix_maybe_broadcast!(dest, src::AbstractArray, ixs::Tuple) =
    fix_expand(dest, src, ixs)
fix_maybe_broadcast!(dest, src, ixs::Tuple) =
    fix_expand(dest, Ref(src), ixs)
