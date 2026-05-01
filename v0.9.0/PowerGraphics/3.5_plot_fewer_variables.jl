# # How to plot only specific variables from the results

# See [How to set up plots](3.0_set_up_plots.md) to get started

# ## How to plot a single dataframe

# ```julia
# plots = plot_dataframe(dataframe, time_range)
# ```
# ![this one](plots-54/Generation.png)

# ## How to plot a single dataframe overlayed on an existing plot

# ```julia
# plot = plot_variable(dataframe_1, time_range)
# plot_2 = plot_variable(plot, dataframe_2, time_range)
# ```
# ![this one](plots-55/overlay.png)

# ## How to plot a single variable

# ```julia
# plots = plot_variable(results, "P__ThermalStandard")
# ```
# ![this one](plots-50/P__ThermalStandard.png)

# ## How to plot a single variable overlayed on an existing plot

# ```julia
# plot = plot_variable(results, "P__ThermalStandard")
# plot_2 = plot_variable(plot, results, "P__RenewableDispatch")
# ```
# ![this one](plots-53/overlay.png)


# ## To plot only a couple of variables from the collected results:
# ### Define the variables to be plotted

# ```julia
# variables = [Symbol("P__ThermalStandard")]
# plots = stack_plot(results, variables)
# ```

# ![this one](plots-51/P__ThermalStandard_Stack.png)
# ![this one](plots-51/Plot_with_Fewer_Variables.png)

# This will plot all of the generators for the listed variables.

# ## To collect only a subset of variables and generators:

# ```julia
# selected_variables = Dict(
#   Symbol("P__ThermalStandard") => [:Brighton, :Solitude],
# )
# results_subset = sort_data(results; Variables = selected_variables)
# plots = stack_plot(results_subset)
# ```

# ![this one](plots-52/P__ThermalStandard_Stack.png)
# ![this one](plots-52/Selected_Variables_Plot.png)

# `sort_data()` creates a new results object only containing the subset of variables and generators listed in the dictionary.
# If the Variables key word argument is not called, the default for `sort_data()` is to alphebatize the generators per variable.
