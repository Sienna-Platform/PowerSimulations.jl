"""
Return a DataFrame from a CSV file.
"""
function read_dataframe(filename::AbstractString)
    return CSV.read(filename, DataFrames.DataFrame)
end
