# File Names definitions
const PROBLEM_LOG_FILENAME = "operation_problem.log"
const SIMULATION_LOG_FILENAME = "simulation.log"
const REQUIRED_RECORDERS = (:simulation_status, :execution)
const KNOWN_SIMULATION_PATHS = [
    "data_store",
    "logs",
    "models_json",
    "problems",
    "recorder",
    "results",
    "simulation_files",
    "simulation_partitions",
]
"If the name of an extraneous file that appears in simulation results matches one of these regexes, it is safe to ignore"
const IGNORABLE_FILES = [
    r"^\.DS_Store$",
    r"^\.Trashes$",
    r"^\.Trash-.*$",
    r"^\.nfs.*$",
    r"^[Dd]esktop.ini$",
]
const RESULTS_DIR = "results"
const NO_SERVICE_NAME_PROVIDED = ""

const RUN_SIMULATION_TIMER = TimerOutputs.TimerOutput()
