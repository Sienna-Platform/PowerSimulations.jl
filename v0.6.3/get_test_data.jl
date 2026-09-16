time_steps = 1:24
using Cbc
using PowerSimulations
using PowerSystems
const PSI = PowerSimulations
const PSY = PowerSystems
Cbc_optimizer = optimizer_with_attributes(Cbc.Optimizer)
abstract type TestOpProblem <: PSI.AbstractOperationsProblem end
base_dir = string(dirname(dirname(pathof(PowerSimulations))))
DATA_DIR = joinpath(base_dir, "test/test_data")
include(joinpath(DATA_DIR, "data_5bus_pu.jl"))
#include(joinpath(DATA_DIR, "data_14bus_pu.jl"))
file_path = joinpath(pwd(), "Documentation_folder")
if !isdir(file_path)
    mkdir(file_path)
end

# Test Systems

# The code below provides a mechanism to optimally construct test systems. The first time a
# test builds a particular system name, the code will construct the system from raw files
# and then serialize it to storage.
# When future tests ask for the same system the code will deserialize it from storage.
#
# If you add a new system then you need to add an entry to TEST_SYSTEMS.
# The build function should accept `kwargs...` instead of specific named keyword arguments.
# This will allow easy addition of new parameters in the future.

struct TestSystemLabel
    name::String
    add_forecasts::Bool
    add_reserves::Bool
end

mutable struct SystemBuildStats
    count::Int
    initial_construct_time::Float64
    serialize_time::Float64
    min_deserialize_time::Float64
    max_deserialize_time::Float64
    total_deserialize_time::Float64
end

function SystemBuildStats(initial_construct_time::Float64, serialize_time::Float64)
    return SystemBuildStats(1, initial_construct_time, serialize_time, 0.0, 0.0, 0.0)
end

function update_stats!(stats::SystemBuildStats, deserialize_time::Float64)
    stats.count += 1
    if stats.min_deserialize_time == 0 || deserialize_time < stats.min_deserialize_time
        stats.min_deserialize_time = deserialize_time
    end
    if deserialize_time > stats.max_deserialize_time
        stats.max_deserialize_time = deserialize_time
    end
    stats.total_deserialize_time += deserialize_time
end

avg_deserialize_time(stats::SystemBuildStats) = stats.total_deserialize_time / stats.count

g_system_serialized_files = Dict{TestSystemLabel, String}()
g_system_build_stats = Dict{TestSystemLabel, SystemBuildStats}()

function initialize_system_serialized_files()
    empty!(g_system_serialized_files)
    empty!(g_system_build_stats)
end

function summarize_system_build_stats()
    @info "System Build Stats"
    labels = sort!(collect(keys(g_system_build_stats)), by = x -> x.name)
    for label in labels
        x = g_system_build_stats[label]
        system = "$(label.name) add_forecasts=$(label.add_forecasts) add_reserves=$(label.add_reserves)"
        @info system x.count x.initial_construct_time x.serialize_time x.min_deserialize_time x.max_deserialize_time avg_deserialize_time(
            x,
        )
    end
end

function build_system(name::String; add_forecasts = true, add_reserves = false)
    !haskey(TEST_SYSTEMS, name) && error("invalid system name: $name")
    label = TestSystemLabel(name, add_forecasts, add_reserves)
    sys_params = TEST_SYSTEMS[name]
    if !haskey(g_system_serialized_files, label)
        @debug "Build new system" label sys_params.description
        build_func = sys_params.build
        start = time()
        sys = build_func(;
            add_forecasts = add_forecasts,
            add_reserves = add_reserves,
            time_series_in_memory = sys_params.time_series_in_memory,
        )
        construct_time = time() - start
        serialized_file = joinpath(mktempdir(), "sys.json")
        start = time()
        PSY.to_json(sys, serialized_file)
        serialize_time = time() - start
        g_system_build_stats[label] = SystemBuildStats(construct_time, serialize_time)
        g_system_serialized_files[label] = serialized_file
    else
        @debug "Deserialize system from file" label
        start = time()
        sys = System(
            g_system_serialized_files[label];
            time_series_in_memory = sys_params.time_series_in_memory,
        )
        update_stats!(g_system_build_stats[label], time() - start)
    end

    return sys
end

function build_c_sys5(; kwargs...)
    nodes = nodes5()
    c_sys5 = System(
        nodes,
        thermal_generators5(nodes),
        loads5(nodes),
        branches5(nodes),
        nothing,
        100.0,
        nothing,
        nothing,
    )

    if get(kwargs, :add_forecasts, true)
        for t in 1:2
            for (ix, l) in enumerate(get_components(PowerLoad, c_sys5))
                add_forecast!(
                    c_sys5,
                    l,
                    Deterministic("get_maxactivepower", load_timeseries_DA[t][ix]),
                )
            end
        end
    end

    return c_sys5
end

function build_c_sys5_ml(; kwargs...)
    nodes = nodes5()
    c_sys5_ml = System(
        nodes,
        thermal_generators5(nodes),
        loads5(nodes),
        branches5(nodes),
        nothing,
        100.0,
        nothing,
        nothing;
        time_series_in_memory = get(kwargs, :time_series_in_memory, true),
    )

    if get(kwargs, :add_forecasts, true)
        for t in 1:2
            for (ix, l) in enumerate(get_components(PowerLoad, c_sys5_ml))
                add_forecast!(
                    c_sys5_ml,
                    l,
                    Deterministic("get_maxactivepower", load_timeseries_DA[t][ix]),
                )
            end
        end
    end

    return c_sys5_ml
end

function build_c_sys5_re(; kwargs...)
    nodes = nodes5()
    c_sys5_re = System(
        nodes,
        vcat(thermal_generators5(nodes), renewable_generators5(nodes)),
        loads5(nodes),
        branches5(nodes),
        nothing,
        100.0,
        nothing,
        nothing;
        time_series_in_memory = get(kwargs, :time_series_in_memory, true),
    )

    if get(kwargs, :add_forecasts, true)
        for t in 1:2
            for (ix, l) in enumerate(get_components(PowerLoad, c_sys5_re))
                add_forecast!(
                    c_sys5_re,
                    l,
                    Deterministic("get_max_active_power", load_timeseries_DA[t][ix]),
                )
            end
            for (ix, r) in enumerate(get_components(RenewableGen, c_sys5_re))
                add_forecast!(
                    c_sys5_re,
                    r,
                    Deterministic("get_max_active_power", ren_timeseries_DA[t][ix]),
                )
            end
        end
    end

    if get(kwargs, :add_reserves, false)
        reserve_re = reserve5_re(get_components(RenewableDispatch, c_sys5_re))
        add_service!(c_sys5_re, reserve_re[1], get_components(RenewableDispatch, c_sys5_re))
        add_service!(
            c_sys5_re,
            reserve_re[2],
            [collect(get_components(RenewableDispatch, c_sys5_re))[end]],
        )
        add_service!(c_sys5_re, reserve_re[3], get_components(RenewableDispatch, c_sys5_re))
        for t in 1:2, (ix, serv) in enumerate(get_components(VariableReserve, c_sys5_re))
            add_forecast!(c_sys5_re, serv, Deterministic("get_requirement", Reserve_ts[t]))
        end
        for t in 1:2, serv in get_components(ReserveDemandCurve, c_sys5_re)
            add_forecast!(
                c_sys5_re,
                serv,
                PiecewiseFunction("get_variable", 5, ORDC_cost_ts[t]),
            )
        end
    end

    return c_sys5_re
end

c_sys5_re = build_c_sys5_re(; add_reserves = true)
system = c_sys5_re
solver = optimizer_with_attributes(Cbc.Optimizer)

devices = Dict{Symbol, DeviceModel}(
    :Generators => DeviceModel(ThermalStandard, ThermalDispatch),
    :Loads => DeviceModel(PowerLoad, StaticPowerLoad),
)
branches = Dict{Symbol, DeviceModel}(
    :L => DeviceModel(Line, StaticLine),
    :T => DeviceModel(Transformer2W, StaticTransformer),
    :TT => DeviceModel(TapTransformer, StaticTransformer),
);
services = Dict{Symbol, ServiceModel}();

template = PSI.OperationsProblemTemplate(CopperPlatePowerModel, devices, branches, services);

operations_problem = PSI.OperationsProblem(
    TestOpProblem,
    template,
    system;
    optimizer = solver,
    use_parameters = true,
);

set_services_template!(
    operations_problem,
    Dict(
        :Reserve => ServiceModel(VariableReserve{ReserveUp}, RangeReserve),
        :Down_Reserve => ServiceModel(VariableReserve{ReserveDown}, RangeReserve),
    ),
)

op_results = solve!(operations_problem)
re_results = PSI.run_economic_dispatch(c_sys5_re; optimizer = solver, use_parameters = true)
