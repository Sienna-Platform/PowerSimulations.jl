# Solvers
using HiGHS

HiGHS_optimizer = JuMP.optimizer_with_attributes(
    HiGHS.Optimizer,
    "time_limit" => 100.0,
    "random_seed" => 12345,
    "log_to_console" => false,
)

HiGHS_optimizer_small_gap = JuMP.optimizer_with_attributes(
    HiGHS.Optimizer,
    "time_limit" => 100.0,
    "random_seed" => 12345,
    "mip_rel_gap" => 0.001,
    "log_to_console" => false,
)
