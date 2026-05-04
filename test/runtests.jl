include("includes.jl")

# Code Quality Tests
import Aqua
Aqua.test_undefined_exports(PowerSimulations)
Aqua.test_ambiguities(PowerSimulations)
Aqua.test_stale_deps(PowerSimulations)
Aqua.find_persistent_tasks_deps(PowerSimulations)
Aqua.test_persistent_tasks(PowerSimulations)
Aqua.test_unbound_args(PowerSimulations)

const LOG_FILE = "power-simulations-test.log"

const DISABLED_TEST_FILES = [  # Can generate with ls -1 test | grep "test_.*.jl"
# "test_basic_model_structs.jl",
# "test_device_branch_constructors.jl",
# "test_device_hvdc.jl",
# "test_device_lcc.jl",
# "test_device_load_constructors.jl",
# "test_device_renewable_generation_constructors.jl",
# "test_device_source_constructors.jl",
# "test_device_thermal_generation_constructors.jl",
# "test_events.jl",
# "test_formulation_combinations.jl",
# "test_import_export_cost.jl",
# "test_initialization_problem.jl",
# "test_jump_utils.jl",
# "test_market_bid_cost.jl",
# "test_mbc_sanity_check.jl",
# "test_model_decision.jl",
# "test_model_emulation.jl",
# "test_network_constructors.jl",
# "test_network_constructors_with_dlr.jl",
# "test_power_flow_in_the_loop.jl",
# "test_print.jl",
# "test_problem_template.jl",
# "test_recorder_events.jl",
# "test_services_constructor.jl",
# "test_simulation_build.jl",
# "test_simulation_execute.jl",
# "test_simulation_models.jl",
# "test_simulation_partitions.jl",
# "test_simulation_results_export.jl",
# "test_simulation_results.jl",
# "test_simulation_sequence.jl",
# "test_simulation_store.jl",
# "test_utils.jl",
]

LOG_LEVELS = Dict(
    "Debug" => Logging.Debug,
    "Info" => Logging.Info,
    "Warn" => Logging.Warn,
    "Error" => Logging.Error,
)

function get_logging_level(env_name::String, default)
    level = get(ENV, env_name, default)
    log_level = get(LOG_LEVELS, level, nothing)
    if log_level === nothing
        error("Invalid log level $level: Supported levels: $(values(LOG_LEVELS))")
    end

    return log_level
end

"""
Includes the given test files, given as a list without their ".jl" extensions.
If none are given it will scan the directory of the calling file and include all
the julia files.
"""
macro includetests(testarg...)
    if length(testarg) == 0
        tests = []
    elseif length(testarg) == 1
        tests = testarg[1]
    else
        error("@includetests takes zero or one argument")
    end

    quote
        tests = $tests
        rootfile = @__FILE__
        if length(tests) == 0
            tests = readdir(dirname(rootfile))
            tests = filter(
                f ->
                    startswith(f, "test_") && endswith(f, ".jl") && f != basename(rootfile),
                tests,
            )
        else
            tests = map(f -> string(f, ".jl"), tests)
        end
        println()
        if !isempty(DISABLED_TEST_FILES)
            @warn("Some tests are disabled $DISABLED_TEST_FILES")
        end
        for test in tests
            test ∈ DISABLED_TEST_FILES && continue
            print(splitext(test)[1], ": ")
            include(test)
            println()
        end
    end
end

function get_logging_level_from_env(env_name::String, default)
    level = get(ENV, env_name, default)
    return IS.get_logging_level(level)
end

function run_tests()
    logging_config_filename = get(ENV, "SIIP_LOGGING_CONFIG", nothing)
    if logging_config_filename !== nothing
        config = IS.LoggingConfiguration(logging_config_filename)
    else
        config = IS.LoggingConfiguration(;
            filename = LOG_FILE,
            file_level = Logging.Info,
            console_level = Logging.Error,
        )
    end
    console_logger = ConsoleLogger(config.console_stream, config.console_level)

    IS.open_file_logger(LOG_FILE, config.file_level) do file_logger
        levels = (Logging.Info, Logging.Warn, Logging.Error)
        multi_logger =
            IS.MultiLogger([console_logger, file_logger], IS.LogEventTracker(levels))
        global_logger(multi_logger)

        if !isempty(config.group_levels)
            IS.set_group_levels!(multi_logger, config.group_levels)
        end

        @time @testset "Begin PowerSimulations tests" begin
            @includetests ARGS
        end

        @test length(IS.get_log_events(multi_logger.tracker, Logging.Error)) == 0

        @info IS.report_log_summary(multi_logger)
    end
end

logger = global_logger()

try
    run_tests()
finally
    # Guarantee that the global logger is reset.
    global_logger(logger)
    nothing
end
