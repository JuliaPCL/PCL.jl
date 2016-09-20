using Documenter
using PCL

makedocs(
    modules = [PCL],
    clean   = false,
    format   = Documenter.Formats.HTML,
    sitename = "PCL.jl",
    pages = Any[
        "Home" => "index.md"
        "API" => Any[
            "Common" => "common.md"
            "Features" => "features.md"
            "Filters" => "filters.md"
            "IO" => "io.md"
            "KDTree" => "kdtree.md"
            "KeyPoints" => "keypoints.md"
            "Octree" => "octree.md"
            "Recognition" => "recognition.md"
            "Registration" => "registration.md"
            "SampleConsensus" => "sampleconsensus.md"
            "Search" => "seearch.md"
            "Segmentation" => "segmentation.md"
            "Surface" => "surface.md"
            "Tracking" => "tracking.md"
            "Visualization" => "visualization.md"
            ]
        ],
)

#deploydocs(
#    target = "build",
#    deps = nothing,
#    make = nothing,
#    repo = "github.com/r9y9/WORLD.jl.git",
#)
