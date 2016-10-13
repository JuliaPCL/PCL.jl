using Base.Test

# Tutorials
for f in [
        "passthrough",
        "global_hypothesis_verification",
        "voxel_grid",
        "correspondence_grouping",
        "planar_segmentation",
        "statistical_removal",
        "region_growing_rgb_segmentation",
        "extract_indices",
        "tilt_compensation",
        ]
    @testset "$f" begin
        include(joinpath(string(f, ".jl")))
    end
end

if get(ENV, "PCLJL_RUN_VISUALIZATION_TESTS", true)
    @testset "offscreen_rendering" begin
        include("offscreen_rendering.jl")
    end
end
