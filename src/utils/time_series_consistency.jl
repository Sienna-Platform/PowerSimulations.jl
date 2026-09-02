"""
Return `(initial_timestamp, length)` for the `SingleTimeSeries` in `sys` whose
resolution matches `resolution`. Throws `IS.InvalidValue` when no match exists
or when matching series disagree on either field. Used to validate emulation
model inputs when a system carries SingleTimeSeries at multiple resolutions.
"""
function get_single_time_series_consistency(
    sys::PSY.System,
    resolution::Dates.Period,
)
    table = PSY.get_static_time_series_summary_table(sys)
    target = Dates.canonicalize(Dates.Millisecond(resolution))
    filtered = [row for row in DataFrames.eachrow(table) if row.resolution == target]
    if isempty(filtered)
        throw(
            IS.InvalidValue(
                "No SingleTimeSeries found at resolution $(target)",
            ),
        )
    end
    unique_pairs =
        unique((row.initial_timestamp, row.time_step_count) for row in filtered)
    if length(unique_pairs) > 1
        throw(
            IS.InvalidValue(
                "SingleTimeSeries at resolution $(target) have inconsistent " *
                "initial times and lengths: $(collect(unique_pairs))",
            ),
        )
    end
    ini_time, ts_length = first(unique_pairs)
    return (Dates.DateTime(ini_time), ts_length)
end
