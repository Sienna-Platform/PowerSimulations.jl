# # How to Make Forecast Plots

# See [How to set up plots](3.0_set_up_plots.md) to get started

# ```julia
# using PowerGraphics
# ```

# # Make a plot of the reserves

# ```julia
# plots = plot_reserves(results)
# ```

# ![this one](plots-4/Stack_Up_Reserves.png)
# ![this one](plots-4/Stack_Down_Reserves.png)

# # Make demand plots from results

# ```julia
# plots = plot_demand(results)
# ```

# ![this one](plots-4/Example_Demand_Plot.png)

# ### Make demand plots from a system

# ```julia
# plots = plot_demand(system)
# ```

# ![this one](plots-4/Example_Demand_Plot_From_System.png)

# ### Make demand plots from a Subsection of Time

# ```julia
# initial_time = Dates.Date(2024, 01, 01, 02, 0, 0)
# horizon = 6
# plots = plot_demand(system; horizon = horizon, initial_time = initial_time)
# ```

# ![this one](plots-4/Example_Demand_Plot_Subsection.png)

# ## Make Demand Plots by Power Systems DataType

# ### sort by PowerLoad, Bus, System

# ```julia
# plots = plot_demand(system; aggregate = System)
# ```

# ![this one](plots-4/Example_Demand_Plot_by_Type.png)

# ### Make demand plots without interpolation between points (stair plot)

# ```julia
# plots = plot_demand(system; stair = true)
# ```

# ![this one](plots-4/Example_Stair_Demand_Plot.png)

# ### Set different colors for the plots

# ```julia
# colors = [:orange :pink :blue :red :grey]
# plot_demand(system; seriescolor = colors)
# ```

# ![this one](plots-4/Example_Demand_Plot_with_Different_Colors.png)

# ### Set a title

# ```julia
# title = "Example Demand Plot with Title"
# plot_demand(system; title = title)
# ```

# ![this one](plots-4/Example_Demand_Plot_with_Title.png)

# ### For saving the plot with the PlotlyJS backend, you can set a different format for saving
# ```julia
# plot_demand(system; save = path, format = "html")
# ```
# Default format for saving is png.
# Optional formats for saving include png, html, and svg.
