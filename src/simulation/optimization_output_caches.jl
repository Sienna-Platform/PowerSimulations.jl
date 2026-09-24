"""
Cache for all model outputs
"""
struct OptimizationOutputCaches
    data::Dict{OptimizationOutputCacheKey, OptimizationOutputCache}
    max_size::Int
    min_flush_size::Int
end

function OptimizationOutputCaches()
    return OptimizationOutputCaches(
        Dict{OptimizationOutputCacheKey, OptimizationOutputCache}(),
        0,
        0,
    )
end

function OptimizationOutputCaches(rules::CacheFlushRules)
    return OptimizationOutputCaches(
        Dict{OptimizationOutputCacheKey, OptimizationOutputCache}(),
        rules.max_size,
        rules.min_flush_size,
    )
end

"""
    Base.empty!(cache::OptimizationOutputCaches)

Empty the [`OptimizationOutputCaches`](@ref)
"""
function Base.empty!(cache::OptimizationOutputCaches)
    for output_cache in values(cache.data)
        empty!(output_cache)
    end
end

# PERF: incremental improvement if we manually keep track of the current size.
# Could be prone to bugs if we miss a change.
# The number of containers is not expected to be more than 100.

#decrement_size!(cache::OptimizationOutputCaches, size) = cache.size -= size
#increment_size!(cache::OptimizationOutputCaches, size) = cache.size += size

get_max_size(cache::OptimizationOutputCaches) = cache.max_size
get_min_flush_size(cache::OptimizationOutputCaches) = cache.min_flush_size
get_size(cache::OptimizationOutputCaches) =
    reduce(+, (get_size(x) for x in values(cache.data)))

# Leave some buffer because we may slightly exceed the limit.
is_full(cache::OptimizationOutputCaches, cur_size) = cur_size >= cache.max_size * 0.95

function add_output_cache!(cache::OptimizationOutputCaches, model_name, key, flush_rule)
    cache_key = OptimizationOutputCacheKey(model_name, key)
    cache.data[cache_key] = OptimizationOutputCache(cache_key, flush_rule)
    @debug "Added cache container for" LOG_GROUP_SIMULATION_STORE model_name key flush_rule
    return
end

"""
Return true if the cache has data that has not been flushed to storage.
"""
function has_dirty(cache::OptimizationOutputCaches)
    for output_cache in values(cache.data)
        if has_dirty(output_cache)
            return true
        end
    end

    return false
end

get_output_cache(cache::OptimizationOutputCaches, key::OptimizationOutputCacheKey) =
    cache.data[key]

function get_output_cache(
    cache::OptimizationOutputCaches,
    model_name,
    key::OptimizationContainerKey,
)
    cache_key = OptimizationOutputCacheKey(model_name, key)
    return get_output_cache(cache, cache_key)
end

"""
Return true if the data for `timestamp` is stored in cache.
"""
function is_cached(cache::OptimizationOutputCaches, model_name, key, index)
    cache_key = OptimizationOutputCacheKey(model_name, key)
    return is_cached(cache, cache_key, index)
end

is_cached(cache::OptimizationOutputCaches, key, timestamp::Dates.DateTime) =
    has_timestamp(cache.data[key], timestamp::Dates.DateTime)

is_cached(cache::OptimizationOutputCaches, key, ::Int) = false

"""
Log the cache hit percentages for all caches.
"""
function log_cache_hit_percentages(cache::OptimizationOutputCaches)
    for key in keys(cache.data)
        output_cache = cache.data[key]
        cache_hit_pecentage = get_cache_hit_percentage(output_cache)
        @debug "Cache stats" LOG_GROUP_SIMULATION_STORE key cache_hit_pecentage
    end
    return
end

"""
Read the output from cache. Callers must first call [`is_cached`](@ref) to check if the
timestamp is present.
"""
function read_output(cache::OptimizationOutputCaches, model_name, key, timestamp)
    cache_key = OptimizationOutputCacheKey(model_name, key)
    return read_output(cache, cache_key, timestamp)
end

read_output(cache::OptimizationOutputCaches, key, timestamp) =
    cache.data[key].data[timestamp]
