# # How to plot multiple results in subplots for comparisons

# See [How to set up plots](3.0_set_up_plots.md) to get started

# ### Run the simulation and get various results

# ### Plot the results as an array

# ```julia
# plots = stack_plot([results_one, results_two])
# ```

# ![this one](plots-6/P__ThermalStandard_Stack.png)
# ![this one](plots-6/Comparison.png)

# ### Plot by Fuel Type

# ```julia
# plots = fuel_plot([results_one, results_two], c_sys5_re)
# ```

# ![this one](plots-6/Comparison_Stack.png)
# ![this one](plots-6/Comparison_Bar.png)

# ## Multiple results can be compared while also plotting fewer variables

# ```julia
# variables = [Symbol("P__ThermalStandard")]
# plots = stack_plot([results_one, results_two], variables)
# ```

# ![this one](plots-6/P__ThermalStandard_Stack.png)
# ![this one](plots-6/Comparison_with_fewer_variables.png)
