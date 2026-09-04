function _calc_dimensions(
    array::DenseAxisArray,
    key::OptimizationContainerKey,
    num_rows::Int,
    horizon::Int,
)
    ax = axes(array)
    columns = get_column_names_from_axis_array(key, array)
    # Stored time-major as (horizon, columns..., num_executions) in every branch. HDF5's
    # fastest-varying dimension is Julia's first, so keeping the execution index last makes
    # one execution contiguous on disk: reading one device is `[:, col, row]` and all
    # devices is `[:, :, row]`. Time is the container's LAST axis, hence the rotation.
    if length(ax) == 1
        if length(ax[1]) != horizon
            @debug "$(encode_key_as_string(key)) has length $(length(ax[1])). Different than horizon $horizon."
        end
        dims = (length(ax[1]), 1, num_rows)
    elseif length(ax) == 2
        if length(ax[2]) != horizon
            @debug "$(encode_key_as_string(key)) has length $(length(ax[2])). Different than horizon $horizon."
        end
        dims = (length(ax[2]), length(columns[1]), num_rows)
    elseif length(ax) == 3
        if length(ax[3]) != horizon
            @debug "$(encode_key_as_string(key)) has length $(length(ax[3])). Different than horizon $horizon."
        end
        dims = (length(ax[3]), length(columns[1]), length(columns[2]), num_rows)
    else
        error("unsupported data size $(length(ax))")
    end

    return Dict("columns" => columns, "dims" => dims)
end

function _calc_dimensions(
    array::SparseAxisArray,
    key::OptimizationContainerKey,
    num_rows::Int,
    horizon::Int,
)
    columns = get_column_names_from_axis_array(key, array)
    dims = (horizon, length.(columns)..., num_rows)
    return Dict("columns" => columns, "dims" => dims)
end
