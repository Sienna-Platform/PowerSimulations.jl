struct Settings
    horizon::Base.RefValue{Dates.Millisecond}
    resolution::Base.RefValue{Dates.Millisecond}
    time_series_cache_size::Int
    warm_start::Base.RefValue{Bool}
    initial_time::Base.RefValue{Dates.DateTime}
    optimizer::Union{Nothing, MOI.OptimizerWithAttributes}
    direct_mode_optimizer::Bool
    optimizer_solve_log_print::Bool
    detailed_optimizer_stats::Bool
    calculate_conflict::Bool
    check_components::Bool
    initialize_model::Bool
    initialization_file::String
    deserialize_initial_conditions::Bool
    export_pwl_vars::Bool
    allow_fails::Bool
    rebuild_model::Bool
    export_optimization_model::Bool
    store_variable_names::Bool
    check_numerical_bounds::Bool
    ext::Dict{String, Any}
end

function Settings(
    sys;
    initial_time::Dates.DateTime = UNSET_INI_TIME,
    time_series_cache_size::Int = IS.TIME_SERIES_CACHE_SIZE_BYTES,
    warm_start::Bool = true,
    horizon::Dates.Period = UNSET_HORIZON,
    resolution::Dates.Period = UNSET_RESOLUTION,
    optimizer = nothing,
    direct_mode_optimizer::Bool = false,
    optimizer_solve_log_print::Bool = false,
    detailed_optimizer_stats::Bool = false,
    calculate_conflict::Bool = false,
    check_components::Bool = true,
    initialize_model::Bool = true,
    initialization_file = "",
    deserialize_initial_conditions::Bool = false,
    export_pwl_vars::Bool = false,
    allow_fails::Bool = false,
    check_numerical_bounds = true,
    rebuild_model = false,
    export_optimization_model = false,
    store_variable_names = false,
    ext = Dict{String, Any}(),
)
    if time_series_cache_size > 0 && PSY.stores_time_series_in_memory(sys)
        @info "Overriding time_series_cache_size because time series is stored in memory"
        time_series_cache_size = 0
    end

    if isa(optimizer, MOI.OptimizerWithAttributes) || optimizer === nothing
        optimizer_ = optimizer
    elseif isa(optimizer, DataType)
        optimizer_ = MOI.OptimizerWithAttributes(optimizer)
    else
        error(
            "The provided input for optimizer is invalid. Provide a JuMP.OptimizerWithAttributes object or a valid Optimizer constructor (e.g. HiGHS.Optimizer).",
        )
    end

    return Settings(
        Ref(IS.time_period_conversion(horizon)),
        Ref(IS.time_period_conversion(resolution)),
        time_series_cache_size,
        Ref(warm_start),
        Ref(initial_time),
        optimizer_,
        direct_mode_optimizer,
        optimizer_solve_log_print,
        detailed_optimizer_stats,
        calculate_conflict,
        check_components,
        initialize_model,
        initialization_file,
        deserialize_initial_conditions,
        export_pwl_vars,
        allow_fails,
        rebuild_model,
        export_optimization_model,
        store_variable_names,
        check_numerical_bounds,
        ext,
    )
end

function log_values(settings::Settings)
    text = Vector{String}()
    for (name, type) in zip(fieldnames(Settings), fieldtypes(Settings))
        val = getfield(settings, name)
        if type <: Base.RefValue
            val = val[]
        end
        push!(text, "$name = $val")
    end

    @debug "Settings: $(join(text, ", "))" _group = LOG_GROUP_OPTIMIZATION_CONTAINER
end

get_horizon(settings::Settings) = settings.horizon[]
get_resolution(settings::Settings) = settings.resolution[]
get_initial_time(settings::Settings)::Dates.DateTime = settings.initial_time[]
get_optimizer(settings::Settings) = settings.optimizer
get_ext(settings::Settings) = settings.ext
get_warm_start(settings::Settings) = settings.warm_start[]
get_check_components(settings::Settings) = settings.check_components
get_initialize_model(settings::Settings) = settings.initialize_model
get_initialization_file(settings::Settings) = settings.initialization_file
get_deserialize_initial_conditions(settings::Settings) =
    settings.deserialize_initial_conditions
get_export_pwl_vars(settings::Settings) = settings.export_pwl_vars
get_check_numerical_bounds(settings::Settings) = settings.check_numerical_bounds
get_allow_fails(settings::Settings) = settings.allow_fails
get_optimizer_solve_log_print(settings::Settings) = settings.optimizer_solve_log_print
get_calculate_conflict(settings::Settings) = settings.calculate_conflict
get_detailed_optimizer_stats(settings::Settings) = settings.detailed_optimizer_stats
get_direct_mode_optimizer(settings::Settings) = settings.direct_mode_optimizer
get_store_variable_names(settings::Settings) = settings.store_variable_names
get_rebuild_model(settings::Settings) = settings.rebuild_model
get_export_optimization_model(settings::Settings) = settings.export_optimization_model
use_time_series_cache(settings::Settings) = settings.time_series_cache_size > 0

function set_horizon!(settings::Settings, horizon::Dates.TimePeriod)
    settings.horizon[] = IS.time_period_conversion(horizon)
    return
end

function set_resolution!(settings::Settings, resolution::Dates.TimePeriod)
    settings.resolution[] = IS.time_period_conversion(resolution)
    return
end

function set_initial_time!(settings::Settings, initial_time::Dates.DateTime)
    settings.initial_time[] = initial_time
    return
end

function set_warm_start!(settings::Settings, warm_start::Bool)
    settings.warm_start[] = warm_start
    return
end
