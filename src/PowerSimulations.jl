isdefined(Base, :__precompile__) && __precompile__()
module PowerSimulations

#################################################################################
# Exports — simulation orchestration (PSI's own API)
export Simulation
export SimulationModels
export SimulationSequence
export SimulationResults
export SimulationPartitions
export SimulationPartitionResults
export SimulationResultsExport
export SimulationProblemResults
export InterProblemChronology
export IntraProblemChronology
export GenericOpProblem
export UnitCommitmentProblem
export EconomicDispatchProblem
export template_economic_dispatch
export template_unit_commitment
export build!
export execute!
export solve!
export run!
export get_simulation_model
export run_parallel_simulation
export process_simulation_partition_cli_args
export join_simulation
export export_results
export export_optimizer_stats
export export_realized_outputs
export get_decision_problem_results
export get_emulation_problem_results
export get_system!
export set_system!
export list_decision_problems
export list_supported_formats
export load_results!
export read_realized_variable
export read_realized_dual
export read_realized_parameter
export read_realized_aux_variable
export read_realized_expression
export read_realized_variables
export read_realized_duals
export read_realized_parameters
export read_realized_aux_variables
export read_realized_expressions
export get_realized_timestamps
export list_simulation_events
export show_simulation_events
export show_recorder_events
export get_num_partitions

#################################################################################
# Imports
import DataStructures: OrderedDict, Deque, SortedDict
import Logging
import Serialization
import JuMP
import JuMP: optimizer_with_attributes
import JuMP.Containers: DenseAxisArray, SparseAxisArray
import JSON3
import PowerSystems as PSY
import InfrastructureSystems as IS
import InfrastructureSystems: @assert_op, TableFormat, list_recorder_events, get_name
import InfrastructureSystems.Simulation: SimulationInfo
import PowerNetworkMatrices as PNM
import PowerSystems:
    get_components, get_component, get_available_components, get_available_component,
    get_groups, get_available_groups

using InfrastructureOptimizationModels
using PowerOperationsModels
import InfrastructureOptimizationModels as IOM
import PowerOperationsModels as POM

# IOM and POM each define their own `const COST_EPSILON = 1e-3`; re-exporting both via the
# `names(m)` loop below leaves the name ambiguous (undefined) in PowerSimulations. Importing it
# explicitly from one side resolves the ambiguity with a real binding.
import InfrastructureOptimizationModels: COST_EPSILON

# Unexported IOM surface PSI orchestrates with. The authoritative list is produced by
# scripts/excision/check_undefined.jl; extend this block, never qualify at call sites.
import InfrastructureOptimizationModels:
    get_store, get_status, set_status!, get_output_dir, set_output_dir!,
    get_simulation_info, set_simulation_info!, get_run_status, set_run_status!,
    is_synchronized, set_synchronized_status!, get_store_params, set_store_params!,
    advance_execution_count!, get_execution_count, get_executions, set_executions!,
    set_execution_count!, get_initial_time, is_built, warm_start_enabled,
    _pre_solve_model_checks, solve_model!, get_time_series_cache, empty_time_series_cache!,
    get_log_file, get_recorder_dir, get_initial_conditions_file, add_recorders!,
    register_recorders!, unregister_recorders!, configure_logging, get_current_timestamp,
    get_simulation_number, set_simulation_number!, get_sequence_uuid, set_sequence_uuid!,
    cost_function_unsynch, update_objective_function!, reset_optimization_model!
import InfrastructureOptimizationModels:
    ModelStoreParams, OptimizationContainerMetadata, get_num_executions, get_horizon_count,
    get_base_power,
    get_system_uuid, get_source_data_uuid, make_key,
    DecisionModelStore, initialize_storage!, write_output!, read_outputs,
    write_optimizer_stats!, DecisionModelIndexType, EmulationModelIndexType,
    STORE_CONTAINERS, STORE_CONTAINER_DUALS, STORE_CONTAINER_PARAMETERS,
    STORE_CONTAINER_VARIABLES, STORE_CONTAINER_AUX_VARIABLES, STORE_CONTAINER_EXPRESSIONS,
    get_data_field, list_fields, list_keys
import InfrastructureOptimizationModels:
    AbstractDataset, InMemoryDataset, HDF5Dataset, DatasetContainer, make_system_state,
    get_dataset, set_dataset!, has_dataset, get_dataset_keys, get_dataset_values,
    set_dataset_values!, get_dataset_value, get_last_recorded_row, set_last_recorded_row!,
    get_update_timestamp, set_update_timestamp!, get_last_updated_timestamp,
    get_last_recorded_value, get_last_update_value, get_num_rows, get_data_resolution,
    get_end_of_step_timestamp, get_value_timestamp, set_value!, get_dataset_size,
    get_column_names, OutputsByTime, OutputsByKeyAndTime, make_dataframes
import InfrastructureOptimizationModels:
    get_parameter_attributes, get_parameter_array, get_parameter_multiplier_array,
    get_attribute_key, get_time_series_name, _get_ts_uuid, _set_param_value_at!,
    set_parameter!, ValidDataParamEltypes, TimeSeriesAttributes, VariableValueAttributes,
    CostFunctionAttributes, EventParametersAttributes, NoAttributes
import InfrastructureOptimizationModels:
    LOG_GROUP_SIMULATION_STORE, LOG_GROUP_OUTPUTS, LOG_GROUP_MODEL_STORE,
    LOG_GROUP_BUILD_INITIAL_CONDITIONS, set_interval!, get_variables, get_parameters,
    get_duals, get_expressions, get_aux_variables, get_time_steps, find_timestamp_index,
    to_matrix, get_column_names_from_axis_array, should_write_resulting_value,
    convert_output_to_natural_units, deserialize_key, get_initial_conditions_data,
    _deepcopy_template, get_deterministic_time_series_type, RUN_OPERATION_MODEL_TIMER,
    to_outputs_dataframe, _read_outputs, get_time_series_values!, export_optimizer_stats,
    export_output, export_realized_outputs
import InfrastructureOptimizationModels:
    ABSOLUTE_TOLERANCE, AbstractAffectFeedforward, calculate_parameter_values,
    encode_key_as_string, get_aux_variables_values, get_container_keys, get_duals_values,
    get_enum_value, get_ic_type, get_parameters_values, get_piecewise_curve_per_system_unit,
    get_store_container_type, get_variables_values, get_variable_types,
    InitialConditionKey, is_time_variant, JuMPFloatArray, JuMPVariableTensor, KiB,
    lookup_additional_axes, MiB, MILLISECONDS_IN_HOUR, ObjectiveFunctionParameter,
    read_json, start_up_cost, TimeSeriesParameter, to_dataframe, UNSET_HORIZON,
    UNSET_INI_TIME, UNSET_INTERVAL, UNSET_RESOLUTION, unwrap_for_param
import InfrastructureOptimizationModels:
    add_to_objective_variant_expression!, assign_maybe_broadcast!, fix_maybe_broadcast!,
    set_expression!
import InfrastructureOptimizationModels:
    _deserialize_key, _get_parameter_field, _process_timestamps, _should_export,
    get_current_time, get_forecast_horizon, should_export_aux_variable,
    should_export_dual, should_export_expression, should_export_parameter,
    should_export_variable, check_file_integrity
import PowerOperationsModels:
    DeviceAboveMinPower, get_feedforward_meta, get_input_offer_curves,
    get_output_offer_curves, MultiStartVariable, requires_reconciliation,
    BUILD_PROBLEMS_TIMER

import TimerOutputs
import ProgressMeter
import Distributed
import Random
import Random: AbstractRNG
import Dates
import TimeSeries
import DataFrames
import DataFrames: DataFrame, DataFrameRow, Not, innerjoin
import DataFramesMeta: @chain, @orderby, @rename, @select, @subset, @transform
import CSV
import HDF5
import PrettyTables

# Re-export the IOM and POM public API so `using PowerSimulations` is sufficient (spec D1).
for m in (InfrastructureOptimizationModels, PowerOperationsModels)
    for n in names(m)
        n === nameof(m) && continue
        @eval export $n
    end
end

################################################################################
const PSI = PowerSimulations
const ISOPT = IS.Optimization
const TS = TimeSeries

function progress_meter_enabled()
    return isa(stderr, Base.TTY) &&
           (get(ENV, "CI", nothing) != "true") &&
           (get(ENV, "RUNNING_PSI_TESTS", nothing) != "true")
end

using DocStringExtensions

@template DEFAULT = """
                    $(TYPEDSIGNATURES)
                    $(DOCSTRING)
                    """

# Includes — order matters. Constants and types before their users.
include("core/definitions.jl")
include("core/abstract_simulation_store.jl")
include("core/cache_utils.jl")
# EVENTS-EXCISION: event framework not yet in POM.
# include("core/event_keys.jl")
# include("core/event_model.jl")

include("initial_conditions/initial_condition_chronologies.jl")
include("simulation/simulation_store_requirements.jl")
include("operation/operation_model_types.jl")
include("operation/decision_model.jl")
include("operation/emulation_model.jl")

include("initial_conditions/update_initial_conditions.jl")

# EVENTS-EXCISION: contingency model not yet in POM.
# include("contingency_model/contingency.jl")
# include("contingency_model/contingency_arguments.jl")
# include("contingency_model/contingency_constraints.jl")

include("simulation/optimization_output_cache.jl")
include("simulation/optimization_output_caches.jl")
include("simulation/simulation_models.jl")
include("simulation/simulation_state.jl")
include("simulation/initial_condition_update_simulation.jl")
include("simulation/simulation_store_params.jl")
include("simulation/hdf_simulation_store.jl")
include("simulation/in_memory_simulation_store.jl")
include("simulation/simulation_store_common.jl")
include("simulation/simulation_problem_results.jl")
include("simulation/get_components_interface.jl")
include("simulation/decision_model_simulation_results.jl")
include("simulation/emulation_model_simulation_results.jl")
include("simulation/realized_meta.jl")
include("simulation/simulation_partitions.jl")
include("simulation/simulation_partition_results.jl")
include("simulation/simulation_sequence.jl")
include("simulation/simulation_internal.jl")
include("simulation/simulation.jl")
# EVENTS-EXCISION: include("simulation/simulation_events.jl")
include("simulation/simulation_results_export.jl")
include("simulation/simulation_results.jl")
include("operation/operation_model_simulation_interface.jl")
include("parameters/update_container_parameter_values.jl")
include("parameters/update_cost_parameters.jl")
include("parameters/update_parameters.jl")

include("operation/operation_problem_templates.jl")

include("utils/store_dimensions.jl")
include("utils/print_pt_v3.jl")
include("utils/file_utils.jl")
include("utils/time_series_consistency.jl")
include("utils/recorder_events.jl")

end
