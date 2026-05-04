function add_dlr_to_system_branches!(
    sys::System,
    branches_dlr::Vector{String},
    n_steps::Int,
    dlr_factors::Vector{Float64};
    initial_date::String = "2020-01-01",
)
    # Add dynamic line ratings to the system
    for branch_name in branches_dlr
        branch = get_component(ACTransmission, sys, branch_name)
        dlr_data = SortedDict{Dates.DateTime, TimeSeries.TimeArray}()
        data_ts = collect(
            DateTime("$initial_date 0:00:00", "y-m-d H:M:S"):Hour(1):(
                DateTime("$initial_date 23:00:00", "y-m-d H:M:S")
            ),
        )
        for t in 1:n_steps
            ini_time = data_ts[1] + Day(t - 1)
            dlr_data[ini_time] =
                TimeArray(
                    data_ts + Day(t - 1),
                    dlr_factors,
                )
        end

        PowerSystems.add_time_series!(
            sys,
            branch,
            PowerSystems.Deterministic(
                "dynamic_line_ratings",
                dlr_data;
                scaling_factor_multiplier = get_rating,
            ),
        )
    end
end
