import PowerGraphics
using PowerSystems
const PG = PowerGraphics
include("../../src/get_test_data.jl")

# 3.0

path = mkdir(joinpath(pwd(), "plots-01"));
PG.stack_plot(re_results; save = path, display = false, title = "Example GR Plot");

# ## To make an interactive PlotlyJS plot, reset the backend
#Plots.plotlyjs()
path = mkdir(joinpath(pwd(), "plots-02"));
PG.stack_plot(re_results; save = path, display = false, title = "Example PlotlyJS Plot");

# 3.1

path = mkdir(joinpath(pwd(), "plots-10"));
PG.stack_plot(re_results; save = path, title = "Example Stack Plot");
path = mkdir(joinpath(pwd(), "plots-11"));
PG.stack_plot(re_results; save = path, title = "Example saved Stack Plot");
path = mkdir(joinpath(pwd(), "plots-12"));
PG.stack_plot(
    re_results;
    reserves = true,
    save = path,
    format = "png",
    title = "Example Stack Plot with Reserves",
);

path = mkdir(joinpath(pwd(), "plots-13"));
colors = [:pink :green :blue :magenta :black]
PG.stack_plot(
    re_results;
    seriescolor = colors,
    save = path,
    format = "png",
    title = "Example Stack Plot with Other Colors",
);

path = mkdir(joinpath(pwd(), "plots-14"));
PG.stack_plot(re_results; stair = true, save = path, title = "Stair Plot");
path = mkdir(joinpath(pwd(), "plots-15"));
title = "Example of a Title"
PG.stack_plot(re_results; save = path, title = title);
# 3.2
path = mkdir(joinpath(pwd(), "plots-20"));
PG.bar_plot(re_results; save = path, title = "Example Bar Plot");

path = mkdir(joinpath(pwd(), "plots-21"));
PG.bar_plot(re_results; save = path, title = "Example saved Bar Plot");

path = mkdir(joinpath(pwd(), "plots-22"))
PG.bar_plot(
    re_results;
    reserves = true,
    save = path,
    title = "Example Bar Plot with Reserves",
);

path = mkdir(joinpath(pwd(), "plots-23"))
PG.bar_plot(
    re_results;
    seriescolor = colors,
    save = path,
    title = "Example Bar Plot with Other Colors",
);

title = "Example of a Title"

path = mkdir(joinpath(pwd(), "plots-24"));
PG.bar_plot(re_results; save = path, title = title);

# 3.3

path = mkdir(joinpath(pwd(), "plots-3"));
PG.fuel_plot(re_results, c_sys5_re; save = path, title = "Example Fuel Plot");
PG.fuel_plot(re_results, c_sys5_re; save = path, load = true, title = "Fuel Plot with Load");
PG.fuel_plot(
    re_results,
    c_sys5_re;
    save = path,
    curtailment = true,
    title = "Fuel Plot with Curtailment",
);
PG.fuel_plot(
    op_results,
    c_sys5_re;
    reserves = true,
    save = path,
    title = "Example Fuel Plot with Reserves",
);

colors = [:pink :green :blue :magenta :black]
PG.fuel_plot(
    re_results,
    c_sys5_re;
    seriescolor = colors,
    save = path,
    title = "Example Fuel Plot with Other Colors",
);

title = "Example of a Title";
PG.fuel_plot(re_results, c_sys5_re; save = path, title = title);
PG.fuel_plot(re_results, c_sys5_re; save = path, stair = true);

# 3.4 FORECAST PLOTS
path = mkdir(joinpath(pwd(), "plots-4"));
PG.plot_reserves(op_results; save = path);
PG.plot_demand(re_results; save = path, title = "Example Demand Plot");
PG.plot_demand(system; save = path, title = "Example Demand Plot From System");
initial_time = Dates.DateTime(2024, 01, 01, 02, 0, 0);
horizon = 6;

PG.plot_demand(
    system;
    horizon = horizon,
    initial_time = initial_time,
    save = path,
    title = "Example Demand Plot Subsection",
);

PG.plot_demand(
    system;
    aggregate = PSY.System,
    save = path,
    title = "Example Demand Plot by Type",
);

PG.plot_demand(system; stair = true, title = "Example Stair Demand Plot", save = path);
#
colors = [:orange :pink :blue :red :grey]
PG.plot_demand(
    system;
    seriescolor = colors,
    save = path,
    title = "Example Demand Plot with Different Colors",
);

PG.plot_demand(system; title = "Example Demand Plot with Title", save = path);

# 3.5

path = mkdir(joinpath(pwd(), "plots-50"));

PG.plot_variable(op_results, "P__ThermalStandard"; save = path);
path = mkdir(joinpath(pwd(), "plots-53"));

p = PG.plot_variable(re_results, "P__ThermalStandard");
PG.plot_variable(p, re_results, "P__RenewableDispatch"; title = "overlay", save = path);
path = mkdir(joinpath(pwd(), "plots-54"));
PG.plot_dataframe(
    re_results.variable_values[:P__ThermalStandard],
    re_results.time_stamp;
    save = path,
);
path = mkdir(joinpath(pwd(), "plots-55"));

p2 = PG.plot_dataframe(
    re_results.variable_values[:P__ThermalStandard],
    re_results.time_stamp,
);
PG.plot_dataframe(
    p2,
    re_results.variable_values[:P__RenewableDispatch],
    re_results.time_stamp;
    title = "overlay",
    save = path,
    format = "png",
);

variables = [Symbol("P__ThermalStandard")]
path = mkdir(joinpath(pwd(), "plots-51"));

PG.stack_plot(op_results, variables; save = path, title = "Plot with Fewer Variables");
#
path = mkdir(joinpath(pwd(), "plots-52"));

selected_variables = Dict(Symbol("P__ThermalStandard") => [:Brighton, :Solitude]);
results_subset = PG.sort_data(op_results; Variables = selected_variables);

PG.stack_plot(results_subset; save = path, title = "Selected Variables Plot");

# 3.6
results_one =
    PSI.run_economic_dispatch(c_sys5_re; optimizer = solver, use_parameters = true);
results_two =
    PSI.run_economic_dispatch(c_sys5_re; optimizer = solver, use_parameters = true);

path = mkdir(joinpath(pwd(), "plots-6"));
PG.stack_plot([results_one, results_two]; save = path, title = "Comparison");
#
PG.fuel_plot([results_one, results_two], c_sys5_re; save = path, title = "Comparison");
#
variables = [Symbol("P__ThermalStandard")]
PG.stack_plot(
    [results_one, results_two],
    variables;
    save = path,
    title = "Comparison with fewer variables",
);
