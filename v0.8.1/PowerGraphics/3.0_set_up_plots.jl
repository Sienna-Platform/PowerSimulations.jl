# # Getting Started with Power Graphics

# This uses the plotting package for SIIP [PowerGraphics](http://github.com/nrel-siip/PowerGraphics.jl.git)

# Start by creating a results object using [PowerSimulations](http://github.com/nrel-siip/PowerSimulations.jl.git)

# Set up Power Graphics:
# ```julia
# import PowerGraphics
# const PG = PowerGraphics
# ```

# ## To make a simple GR() backend plot (static plot), simply call

# ```julia
# stack_plot(results)
# ```

# ![this one](plots-01/P__ThermalStandard_Stack.png)
# ![this one](plots-01/Example_GR_Plot.png)

# ## To make an interactive PlotlyJS plot, reset the backend

# ```julia
# Plots.plotlyjs()

# stack_plot(results)
# ```
# ![this one](plots-02/P__ThermalStandard_Stack.png)
# ![this one](plots-02/Example_PlotlyJS_Plot.png)
