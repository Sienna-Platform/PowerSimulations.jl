# # How to Make a stack plot

# See [How to set up plots](3.0_set_up_plots.md) to get started

# ```julia
# using PowerGraphics
# const PG = PowerGraphics
# ```

# ### Make stack plots of results

# ```julia
# stack_plot(results)
# ```

# ![this one](plots-10/P__ThermalStandard_Stack.png)
# ![this one](plots-10/Example_Stack_Plot.png)

# ### Save the stack plots to a folder

# ```julia
# folder_path = joinpath(file_path, "plots_1")
# if !isdir(folder_path)
#    mkdir(folder_path)
# end
# stack_plot(results; save = folder_path)
# ```

# ![this one](plots-11/P__ThermalStandard_Stack.png)
# ![this one](plots-11/Example_saved_Stack_Plot.png)

# ### Show reserves in the stack plots

# ```julia
# stack_plot(results; reserves = true)
# ```
# ![this one](plots-12/P__ThermalStandard_Stack.png)
# ![this one](plots-12/Example_Stack_Plot_with_Reserves.png)

# ### Set different colors for the plots

# ```julia
# colors = [:pink :green :blue :magenta :black]
# stack_plot(results; seriescolor = colors)
# ```
# ![this one](plots-13/P__ThermalStandard_Stack.png)
# ![this one](plots-13/Example_Stack_Plot_with_Other_Colors.png)

# ### Create a Stair Plot, instead of interpolating between values

# ```julia
# stair_plot(results)
# ```
# or
# ```julia
# stack_plot(results; stair = true)
# ```
# ![this one](plots-14/P__ThermalStandard_Stair.png)
# ![this one](plots-14/Stair_Plot.png)

# ### Set a title

# ```julia
# title = "Example of a Title"
# stack_plot(results; title = title)
# ```
# ![this one](plots-15/P__ThermalStandard_Stack.png)
# ![this one](plots-15/Example_of_a_Title.png)

# ### For saving the plot with the PlotlyJS backend, you can set a different format for saving
# ```julia
# stack_plot(results; save = path, format = "html")
# ```
# Default format for saving is png.
# Optional formats for saving include png, html, and svg.
