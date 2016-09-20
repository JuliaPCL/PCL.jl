debug = false

using Documenter

@static if debug
    using WORLD
    const PCL = WORLD
else
    using PCL
end

makedocs(
    modules = [PCL],
    clean   = false,
    format   = Documenter.Formats.HTML,
    sitename = "PCL.jl",
    pages = Any[
        "Home" => "index.md",
        "Installation" => "installation.md",
        "Getting started" => "getting-started.md",
        "API" => Any[
            "Common" => "api/common.md"
            "Features" => "api/features.md"
            "Filters" => "api/filters.md"
            "IO" => "api/io.md"
            "KDTree" => "api/kdtree.md"
            "KeyPoints" => "api/keypoints.md"
            "Octree" => "api/octree.md"
            "Recognition" => "api/recognition.md"
            "Registration" => "api/registration.md"
            "SampleConsensus" => "api/sampleconsensus.md"
            "Search" => "api/search.md"
            "Segmentation" => "api/segmentation.md"
            "Surface" => "api/surface.md"
            "Tracking" => "api/tracking.md"
            "Visualization" => "api/visualization.md"
            ]
        ],
)

#deploydocs(
#    target = "build",
#    deps = nothing,
#    make = nothing,
#    repo = "github.com/r9y9/WORLD.jl.git",
#)
