@testset "Test simulation output directory name" begin
    tmpdir = mktempdir()
    name = "simulation"
    dir1 = mkdir(joinpath(tmpdir, name))
    @test PSI._get_output_dir_name(tmpdir, name) == joinpath(tmpdir, name * "-2")
    dir2 = mkdir(joinpath(tmpdir, name * "-2"))
    @test PSI._get_output_dir_name(tmpdir, name) == joinpath(tmpdir, name * "-3")
end
