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
        "offscreen_rendering",
        ]
    @testset "$f" begin
        include(joinpath(string(f, ".jl")))
    end
end
