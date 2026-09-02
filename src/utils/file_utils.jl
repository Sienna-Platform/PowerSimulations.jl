"""
Return a DataFrame from a CSV file.
"""
function read_dataframe(filename::AbstractString)
    return CSV.read(filename, DataFrames.DataFrame)
end

make_system_filename(sys::PSY.System) = make_system_filename(IS.get_uuid(sys))
make_system_filename(sys_uuid::Union{Base.UUID, AbstractString}) = "system-$(sys_uuid).json"
