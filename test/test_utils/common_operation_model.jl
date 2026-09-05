const _DESERIALIZE_MESSAGE = "Deserialized initial_conditions_data"
const _MAKE_IC_MESSAGE = "Make Initial Conditions Model"
const _SKIP_IC_MESSAGE = "Skip build of initial conditions"

function test_ic_serialization_outputs(
    model::IOM.AbstractOptimizationModel;
    ic_file_exists,
    message,
)
    ic_file = IOM.get_initial_conditions_file(model)
    log_file = IOM.get_log_file(model)

    @test isfile(ic_file) == ic_file_exists
    if ic_file_exists
        @test Serialization.deserialize(ic_file) isa IOM.InitialConditionsData
    end

    make = false
    deserialize = false
    skip = false
    if message == "make"
        make = true
    elseif message == "deserialize"
        deserialize = true
    elseif message == "skip"
        skip = true
    else
        error("invalid: $message")
    end

    text = read(log_file, String)
    @test make == occursin(_MAKE_IC_MESSAGE, text)
    @test deserialize == occursin(_DESERIALIZE_MESSAGE, text)
    @test skip == occursin(_SKIP_IC_MESSAGE, text)
end
