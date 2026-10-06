"""
Cache for a single parameter/variable/dual.
Stores arrays chronologically by simulation timestamp.
"""
mutable struct OptimizationOutputCache
    key::OptimizationOutputCacheKey
    "Contains both clean and dirty entries. Any key in data that is earlier than the first
    dirty timestamp must be clean."
    data::OrderedDict{Dates.DateTime, Array}
    "Oldest entry is first"
    dirty_timestamps::Deque{Dates.DateTime}
    stats::CacheStats
    size_per_entry::Int
    flush_rule::CacheFlushRule
end

function OptimizationOutputCache(key, flush_rule)
    return OptimizationOutputCache(
        key,
        OrderedDict{Dates.DateTime, Array}(),
        Deque{Dates.DateTime}(),
        CacheStats(),
        0,
        flush_rule,
    )
end

Base.length(x::OptimizationOutputCache) = length(x.data)
get_cache_hit_percentage(x::OptimizationOutputCache) = get_cache_hit_percentage(x.stats)
get_size(x::OptimizationOutputCache) = length(x) * x.size_per_entry
has_clean(x::OptimizationOutputCache) =
    !isempty(x.data) && !is_dirty(x, first(keys(x.data)))
has_dirty(x::OptimizationOutputCache) = !isempty(x.dirty_timestamps)
should_keep_in_cache(x::OptimizationOutputCache) = x.flush_rule.keep_in_cache

function get_dirty_size(cache::OptimizationOutputCache)
    return length(cache.dirty_timestamps) * cache.size_per_entry
end

function is_dirty(cache::OptimizationOutputCache, timestamp)
    isempty(cache.dirty_timestamps) && return false
    return timestamp >= first(cache.dirty_timestamps)
end

"""
    Base.empty!(cache::OptimizationOutputCache)

Empty the [`OptimizationOutputCache`](@ref)
"""
function Base.empty!(cache::OptimizationOutputCache)
    @assert isempty(cache.dirty_timestamps) "dirty cache was still present $(cache.key) $(cache.dirty_timestamps)"
    empty!(cache.data)
    cache.size_per_entry = 0
    return
end

"""
Add output to the cache.
"""
function add_output!(cache::OptimizationOutputCache,
    timestamp::Dates.DateTime,
    array::Array{Float64},
    system_cache_is_full::Bool)
    if cache.size_per_entry == 0
        cache.size_per_entry = length(array) * sizeof(first(array))
    end

    @debug "add_output!" cache.key timestamp get_size(cache)
    if haskey(cache.data, timestamp)
        throw(IS.InvalidValue("$timestamp is already stored in $(cache.key)"))
    end

    # Note that we buffer all writes in cache until we reach the flush size.
    # The entries using "should_keep_in_cache" can grow quite large for read caching.
    if system_cache_is_full && should_keep_in_cache(cache)
        if has_clean(cache)
            popfirst!(cache.data)
            @debug "replaced cache entry" LOG_GROUP_SIMULATION_STORE cache.key length(
                cache.data,
            )
        end
    end

    _add_output!(cache, timestamp, array)
    return cache.size_per_entry
end

function _add_output!(
    cache::OptimizationOutputCache,
    timestamp::Dates.DateTime,
    data::Array{Float64},
)
    cache.data[timestamp] = data
    push!(cache.dirty_timestamps, timestamp)
    return
end

function discard_outputs!(cache::OptimizationOutputCache, timestamps)
    for timestamp in timestamps
        pop!(cache.data, timestamp)
    end

    @debug "Removed $(first(timestamps)) - $(last(timestamps)) from cache" cache.key
    return
end

"""
Return all dirty data from the cache. Mark the timestamps as clean.
"""
function get_dirty_data_to_flush!(cache::OptimizationOutputCache)
    timestamps = [x for x in cache.dirty_timestamps]
    empty!(cache.dirty_timestamps)
    # Uncomment for performance testing of CacheFlush
    temp = cache.data[first(timestamps)]
    sd = collect(size(temp))
    push!(sd, length(timestamps))
    arrays = Array{Float64}(undef, sd...)
    for (ix, x) in enumerate(timestamps)
        temp_data = cache.data[x]
        selectdim(arrays, ndims(arrays), ix) .= temp_data
    end

    return timestamps, arrays
end

function has_timestamp(cache::OptimizationOutputCache, timestamp)
    present = haskey(cache.data, timestamp)
    if present
        cache.stats.hits += 1
    else
        cache.stats.misses += 1
    end

    return present
end
