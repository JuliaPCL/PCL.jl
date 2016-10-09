using Documenter
using PCL

makedocs(
    modules = [PCL],
    clean   = false,
    format   = Documenter.Formats.HTML,
    sitename = "PCL.jl",
    pages = Any[
        "Home" => "index.md",
        "Installation" => "installation.md",
        "Getting started" => "getting-started.md",
        "Modules" => Any[
            "Common" => "modules/common.md"
            "Features" => "modules/features.md"
            "Filters" => "modules/filters.md"
            "IO" => "modules/io.md"
            "KDTree" => "modules/kdtree.md"
            "KeyPoints" => "modules/keypoints.md"
            "Octree" => "modules/octree.md"
            "Recognition" => "modules/recognition.md"
            "Registration" => "modules/registration.md"
            "SampleConsensus" => "modules/sampleconsensus.md"
            "Search" => "modules/search.md"
            "Segmentation" => "modules/segmentation.md"
            "Surface" => "modules/surface.md"
            "Tracking" => "modules/tracking.md"
            "Visualization" => "modules/visualization.md"
            "LibPCL" => "modules/libpcl.md"
            ]
        ],
)

#deploydocs(
#    target = "build",
#    deps = nothing,
#    make = nothing,
#    repo = "github.com/r9y9/WORLD.jl.git",
#)
