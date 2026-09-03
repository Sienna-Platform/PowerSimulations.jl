# NOTE: None of the models and function in this file are functional. All of these are used for testing purposes and do not represent valid examples either to develop custom
# models. Please refer to the documentation.

struct MockOperationProblem <: POM.GenericPowerDecisionProblem end
struct MockEmulationProblem <: POM.GenericPowerEmulationProblem end

function IOM.DecisionModel(
    ::Type{MockOperationProblem},
    ::Type{T},
    sys::PSY.System;
    name = nothing,
    kwargs...,
) where {T <: POM.AbstractNetworkModel}
    settings = IOM.Settings(sys; kwargs...)
    available_resolutions = PSY.get_time_series_resolutions(sys)
    if length(available_resolutions) == 1
        IOM.set_resolution!(settings, first(available_resolutions))
    else
        error("System has multiple resolutions MockOperationProblem won't work")
    end
    return DecisionModel{MockOperationProblem}(
        PowerOperationsProblemTemplate(T),
        sys,
        settings,
        nothing;
        name = name,
    )
end

function make_mock_forecast(
    horizon::Dates.TimePeriod,
    resolution::Dates.TimePeriod,
    interval::Dates.TimePeriod,
    steps,
)
    init_time = DateTime("2024-01-01")
    timeseries_data = Dict{Dates.DateTime, Vector{Float64}}()
    horizon_count = horizon ÷ resolution
    for i in 1:steps
        forecast_timestamps = init_time + interval * i
        timeseries_data[forecast_timestamps] = rand(horizon_count)
    end
    return Deterministic(;
        name = "mock_forecast",
        data = timeseries_data,
        resolution = resolution,
    )
end

function make_mock_singletimeseries(horizon, resolution)
    init_time = DateTime("2024-01-01")
    horizon_count = horizon ÷ resolution
    tstamps = collect(range(init_time; length = horizon_count, step = resolution))
    timeseries_data = TimeArray(tstamps, rand(horizon_count))
    return SingleTimeSeries(; name = "mock_timeseries", data = timeseries_data)
end

function IOM.DecisionModel(::Type{MockOperationProblem}; name = nothing, kwargs...)
    sys = System(100.0)
    add_component!(sys, ACBus(nothing))
    l = PowerLoad(nothing)
    gen = ThermalStandard(nothing)
    set_bus!(l, get_component(Bus, sys, "init"))
    set_bus!(gen, get_component(Bus, sys, "init"))
    add_component!(sys, l)
    add_component!(sys, gen)
    forecast = make_mock_forecast(
        get(kwargs, :horizon, Hour(24)),
        get(kwargs, :resolution, Hour(1)),
        get(kwargs, :interval, Hour(1)),
        get(kwargs, :steps, 2),
    )
    add_time_series!(sys, l, forecast)
    settings = IOM.Settings(sys;
        horizon = get(kwargs, :horizon, Hour(24)),
        resolution = get(kwargs, :resolution, Hour(1)))
    return DecisionModel{MockOperationProblem}(
        PowerOperationsProblemTemplate(CopperPlateNetworkModel),
        sys,
        settings,
        nothing;
        name = name,
    )
end

function IOM.EmulationModel(::Type{MockEmulationProblem}; name = nothing, kwargs...)
    sys = System(100.0)
    add_component!(sys, ACBus(nothing))
    l = PowerLoad(nothing)
    gen = ThermalStandard(nothing)
    set_bus!(l, get_component(Bus, sys, "init"))
    set_bus!(gen, get_component(Bus, sys, "init"))
    add_component!(sys, l)
    add_component!(sys, gen)
    single_ts = make_mock_singletimeseries(
        get(kwargs, :horizon, Hour(24)),
        get(kwargs, :resolution, Hour(1)),
    )
    add_time_series!(sys, l, single_ts)

    settings = IOM.Settings(sys;
        horizon = get(kwargs, :resolution, Hour(1)),
        resolution = get(kwargs, :resolution, Hour(1)))
    return EmulationModel{MockEmulationProblem}(
        PowerOperationsProblemTemplate(CopperPlateNetworkModel),
        sys,
        settings,
        nothing;
        name = name,
    )
end

function create_simulation_build_test_problems(
    template_uc = get_template_standard_uc_simulation(),
    template_ed = get_template_nomin_ed_simulation(),
    sys_uc = PSB.build_system(PSITestSystems, "c_sys5_uc"),
    sys_ed = PSB.build_system(PSITestSystems, "c_sys5_ed"),
)
    return SimulationModels(;
        decision_models = [
            DecisionModel(template_uc, sys_uc; name = "UC", optimizer = HiGHS_optimizer),
            DecisionModel(template_ed, sys_ed; name = "ED", optimizer = HiGHS_optimizer),
        ],
    )
end

function setup_ic_model_container!(model::DecisionModel)
    # This function is only for testing purposes.
    if !IOM.isempty(model)
        POM.reset!(model)
    end

    IOM.init_optimization_container!(
        IOM.get_optimization_container(model),
        IOM.get_network_model(IOM.get_template(model)),
        IOM.get_system(model),
    )

    IOM.init_model_store_params!(model)

    @info "Make Initial Conditions Model"
    IOM.set_output_dir!(model, mktempdir(; cleanup = true))
    POM.build_initial_conditions!(model)
    POM.solve_and_write_initial_conditions!(model)
    return
end
