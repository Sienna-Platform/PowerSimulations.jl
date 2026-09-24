
struct OptimizationOutputCacheKey
    model::Symbol
    key::OptimizationContainerKey
end

struct CacheFlushRule
    keep_in_cache::Bool
end

CacheFlushRule() = CacheFlushRule(false)

const DEFAULT_SIMULATION_STORE_CACHE_SIZE_MiB = 1024
const DEFAULT_SIMULATION_STORE_CACHE_SIZE = DEFAULT_SIMULATION_STORE_CACHE_SIZE_MiB * MiB
const MIN_CACHE_FLUSH_SIZE_MiB = 1
const MIN_CACHE_FLUSH_SIZE = MIN_CACHE_FLUSH_SIZE_MiB * MiB

"""
Informs the flusher on what data to keep in cache.
"""
struct CacheFlushRules
    data::Dict{OptimizationOutputCacheKey, CacheFlushRule}
    min_flush_size::Int
    max_size::Int
end

function CacheFlushRules(;
    max_size = DEFAULT_SIMULATION_STORE_CACHE_SIZE,
    min_flush_size = MIN_CACHE_FLUSH_SIZE,
)
    return CacheFlushRules(
        Dict{OptimizationOutputCacheKey, CacheFlushRule}(),
        min_flush_size,
        max_size,
    )
end

function add_rule!(
    rules::CacheFlushRules,
    model_name,
    op_container_key,
    keep_in_cache::Bool,
)
    key = OptimizationOutputCacheKey(model_name, op_container_key)
    rules.data[key] = CacheFlushRule(keep_in_cache)
    return
end

function get_rule(x::CacheFlushRules, model, op_container_key)
    return get_rule(x, OptimizationOutputCacheKey(model, op_container_key))
end

get_rule(x::CacheFlushRules, key::OptimizationOutputCacheKey) = x.data[key]

mutable struct CacheStats
    hits::Int
    misses::Int
end

CacheStats() = CacheStats(0, 0)

function get_cache_hit_percentage(x::CacheStats)
    total = x.hits + x.misses
    total == 0 && return 0.0
    return x.hits / (total) * 100
end
