__precompile__(false)

module PCL

export pcl
module pcl

const VERBOSE = true

searchdir(path, key) = filter(x->contains(x, key), readdir(path))

# Search GUI backend
vtk_dirs = searchdir("/usr/local/include", "vtk-")
if !isempty(vtk_dirs)
    global const has_vtk_backend = true
    global const vtk_version = vtk_dirs[1][5:end]
    VERBOSE && info("vtk backend enabled. version: $vtk_version")
end

using BinDeps

# Load dependency
deps = joinpath(Pkg.dir("PCL"), "deps", "deps.jl")
if isfile(deps)
    include(deps)
else
    error("PCL not properly installed. Please run Pkg.build(\"PCL\")")
end

VERBOSE && info("Loading Cxx.jl...")
using Cxx

VERBOSE && info("dlopen...")
for lib in [
        libpcl_common,
        libpcl_io,
        libpcl_features,
        libpcl_filters,
        libpcl_segmentation,
        libpcl_visualization,
        libpcl_registration,
        ]
    p = Libdl.dlopen_e(lib, Libdl.RTLD_GLOBAL)
    p == C_NULL && warn("Failed to load: $lib")
end

function include_headers(top)
    # This is necesarry, but not sure why...
    cxx"""#include <iostream>"""

    # Add top of the system directory that contains whole boost library
    # so that #include <boost/xxx.hpp> works
    boost_parent_dir = "/usr/local/include"
    addHeaderDir(boost_parent_dir, kind=C_System)

    # Eigen (required)
    addHeaderDir("/usr/local/include/eigen3", kind=C_System)

    # FLANN (required)
    addHeaderDir("/usr/local/include/flann", kind=C_System)

    # PCL top directory
    addHeaderDir(top, kind=C_System)
    addHeaderDir(joinpath(top, "pcl"), kind=C_System)

    cxxinclude(joinpath(top, "pcl/pcl_base.h"))
    cxxinclude(joinpath(top, "pcl/common/common_headers.h"))
    cxxinclude(joinpath(top, "pcl/common/transforms.h"))
    cxxinclude(joinpath(top, "pcl/io/pcd_io.h"))

    if has_vtk_backend
        VERBOSE && info("adding vtk and visualization module headers")
        addHeaderDir("/usr/local/include/vtk-$vtk_version/", kind=C_System)
        cxxinclude(joinpath(top, "pcl/visualization/pcl_visualizer.h"))
    end

    # recognition
    for name in ["hough_3d.h", "geometric_consistency.h"]
        cxxinclude(joinpath(top, "pcl", "recognition", "cg", name))
    end

    # features
    for name in ["normal_3d.h", "normal_3d_omp.h", "shot_omp.h", "board.h"]
        cxxinclude(joinpath(top, "pcl", "features", name))
    end

    # filters
    for name in ["uniform_sampling.h"]
        cxxinclude(joinpath(top, "pcl", "filters", name))
    end

    # kdtree
    for name in ["kdtree_flann.h", "impl/kdtree_flann.hpp"]
        cxxinclude(joinpath(top, "pcl", "kdtree", name))
    end
end


const system_include_top = "/usr/local/include"
const local_include_top = joinpath(Pkg.dir("PCL", "deps", "usr", "include"))

function get_pcl_version(top)
    dirs = searchdir(top, "pcl-")
    isempty(dirs) && error("could not find pcl directory")
    pcl_dir = dirs[1]
    return pcl_dir[5:end]
end

topdir_to_be_included = local_include_top

if isdir(local_include_top)
    VERBOSE && info("Including headers from local path: $local_include_top")
    pcl_version = get_pcl_version(local_include_top)
elseif isdir(joinpath(system_include_top, "pcl"))
    VERBOSE && info("Including headers from system path: $system_include_top")
    pcl_version = get_pcl_version(system_include_top)
else
    error("Cannot find PCL headers")
end

VERBOSE && info("pcl_version: $pcl_version")
include_headers(joinpath(topdir_to_be_included, "pcl-$pcl_version"))

for name in [
    "std",
    "common",
    "io",
    "filters",
    "features",
    "kdtree",
    "visualization"
    ]
    include(string(name, ".jl"))
end


end # module pcl

end # module PCL
