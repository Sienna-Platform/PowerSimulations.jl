# See also test_device_source_constructors.jl
for reservation in (false, true)
    local label
    if reservation
        label = "on"
    else
        label = "off"
    end
    @testset "ImportExportCost incremental+decremental Source, no time series versus constant time series, reservation $label" begin
        sys_no_ts = make_5_bus_with_import_export(; name = "sys_no_ts")
        sys_constant_ts =
            make_5_bus_with_ie_ts(false, false, false, false; name = "sys_constant_ts")
        test_generic_mbc_equivalence(sys_no_ts, sys_constant_ts;
            device_to_formulation = FormulationDict(
                Source => DeviceModel(
                    Source,
                    ImportExportSourceModel;
                    attributes = Dict("reservation" => reservation),
                ),
            ),
        )
    end
end

@testset "ImportExportCost constant time series, reservation sanity checks" begin
    sys_constant_ts =
        make_5_bus_with_ie_ts(false, false, false, false; name = "sys_constant_ts")

    for use_simulation in (false, true),
        in_memory_store in (use_simulation ? (false, true) : (false,)),
        reservation in (false, true)

        run_iec_sim(sys_constant_ts,
            IEC_COMPONENT_NAME,
            IECComponentType;
            simulation = use_simulation,
            in_memory_store = in_memory_store,
            reservation = true,
        )
    end
end

# import_scalar/export_scalar ultimately multiply the ActivePowerOutVariable/ActivePowerInVariable
# objective function coefficients; the "breakpoints" cases pick a scalar that maxes out the
# corresponding variable.
const _IEC_VARYING_CASES = (
    (desc = "import slopes", varying = (false, true, false, false),
        import_scalar = 0.5, export_scalar = 2.0),
    (desc = "import breakpoints", varying = (true, false, false, false),
        import_scalar = 0.2, export_scalar = 2.0),
    (desc = "export slopes", varying = (false, false, false, true),
        import_scalar = 0.5, export_scalar = 2.0),
    (desc = "export breakpoints", varying = (false, false, true, false),
        import_scalar = 1.0, export_scalar = 50.0),
    (desc = "everything", varying = (true, true, true, true),
        import_scalar = 0.2, export_scalar = 40.0),
)

for case in _IEC_VARYING_CASES
    @testset "ImportExportCost with time varying $(case.desc), reservation off" begin
        sys_constant = make_5_bus_with_ie_ts(false, false, false, false;
            import_scalar = case.import_scalar, export_scalar = case.export_scalar,
            name = "sys_constant")
        sys_varying = make_5_bus_with_ie_ts(case.varying...;
            import_scalar = case.import_scalar, export_scalar = case.export_scalar,
            name = "sys_varying_$(replace(case.desc, ' ' => '_'))")
        iec_obj_fun_test_wrapper(sys_constant, sys_varying)
    end
end
