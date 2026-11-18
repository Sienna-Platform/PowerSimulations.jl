# # How to Make a report

# See [How to set up plots](3.0_set_up_plots.md) to get started

# ### Set up a report template
# In [report_templates](http://github.com/nrel-siip/PowerGraphics.jl/tree/master/report_templates) there are templates that can be copied and re-named to make a report.

# ```julia
# design_template = "generic_report_template.jmd"
# ```

# ### Generate report

# The report can be generated as a LaTeX PDF (default) or an html file.
# To set the document type as HTML, use the key word argument `doctype = "md2html"`

# ```julia
# out_path = joinpath(pwd(), "Generated_Reports/")
# if !isdir(out_path)
#     mkdir(out_path)
# end
# report(results, out_path, design_template; doctype = "md2html")
# ```

# creates an HTML

# ```julia
# report(results, out_path, design_template)
# ```

# creates a PDF
