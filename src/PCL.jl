__precompile__(false)

module PCL

export pcl
module pcl

const VERBOSE = Bool(parse(Int, get(ENV, "PCLJL_VERBOSE", "1")))

const BOOST_INCLUDE_DIR = get(ENV, "BOOST_INCLUDE_DIR", "/usr/local/include")
const FLANN_INCLUDE_DIR = get(ENV, "FLANN_INCLUDE_DIR", "/usr/local/include")
const EIGEN_INCLUDE_DIR = get(ENV, "EIGEN_INCLUDE_DIR",
    "/usr/local/include/eigen3")

searchdir(path, key) = filter(x->contains(x, key), readdir(path))

const VTK_INCLUDE_DIR_PARENT = get(ENV, "VTK_INCLUDE_DIR_PARENT",
    "/usr/local/include")

# Search GUI backend
vtk_dirs = searchdir(VTK_INCLUDE_DIR_PARENT, "vtk-")
const VTK_INCLUDE_DIR = get(ENV, "VTK_INCLUDE_DIR",
    isempty(vtk_dirs) ? "" : joinpath(VTK_INCLUDE_DIR_PARENT, vtk_dirs[1]))

const has_vtk_backend = isfile(joinpath(VTK_INCLUDE_DIR, "vtkVersion.h"))
if has_vtk_backend
    VERBOSE && info("vtk include directory found: $VTK_INCLUDE_DIR")
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
using CxxStd

macro timevb(expr)
    if VERBOSE
        return Expr(:macrocall, symbol("@time"), expr)
    else
        return expr
    end
end

VERBOSE && info("dlopen...")
for lib in [
        libpcl_common,
        libpcl_io,
        libpcl_features,
        libpcl_filters,
        libpcl_kdtree,
        libpcl_keypoints,
        libpcl_segmentation,
        libpcl_visualization,
        libpcl_recognition,
        libpcl_registration,
        libpcl_octree,
        libpcl_surface,
        libpcl_people,
        libpcl_search,
        libpcl_tracking,
        ]
    p = Libdl.dlopen_e(lib, Libdl.RTLD_GLOBAL)
    p == C_NULL && warn("Failed to load: $lib")
end

# Make sure vtk libraries are loaded before calling @cxx vtkVersion::xxx()
if has_vtk_backend
    cxxinclude(joinpath(VTK_INCLUDE_DIR, "vtkVersion.h"))
    global const vtk_version = bytestring(@cxx vtkVersion::GetVTKVersion())
    VERBOSE && info("vtk version: $vtk_version")
end

function include_headers(top)
    # This is necesarry, but not sure why...
    cxx"""#include <iostream>"""

    # Boost (required)
    addHeaderDir(BOOST_INCLUDE_DIR, kind=C_System)

    # Eigen (required)
    addHeaderDir(EIGEN_INCLUDE_DIR, kind=C_System)

    # FLANN (required)
    addHeaderDir(FLANN_INCLUDE_DIR, kind=C_System)

    # PCL top directory
    addHeaderDir(top, kind=C_System)
    addHeaderDir(joinpath(top, "pcl"), kind=C_System)

    # top level
    VERBOSE && info("Include pcl top-level headers")
    @timevb for name in ["pcl_base.h", "point_cloud.h", "point_types.h",
            "correspondence.h", "ModelCoefficients.h"]
        cxxinclude(joinpath(top, "pcl", name))
    end

    # common
    VERBOSE && info("Include pcl::common headers")
    @timevb for name in ["common_headers.h", "transforms.h", "centroid.h"]
        cxxinclude(joinpath(top, "pcl", "common", name))
    end

    # visualization
    if has_vtk_backend
        VERBOSE && info("adding vtk and visualization module headers")
        addHeaderDir(VTK_INCLUDE_DIR, kind=C_System)
        cxx"""
        #define protected public  // to access PCLVisualizer::interactor_
        #include <pcl/visualization/pcl_visualizer.h>
        #undef protected
        """
        # cxxinclude(joinpath(top, "pcl", "visualization", "pcl_visualizer.h"))
    end

    # io
    VERBOSE && info("Include pcl::io headers")
    @timevb for name in ["pcd_io.h"]
        cxxinclude(joinpath(top, "pcl", "io", name))
    end

    # registration
    VERBOSE && info("Include pcl::registration headers")
    @timevb for name in ["icp.h"]
        cxxinclude(joinpath(top, "pcl", "registration", name))
    end

    # recognition
    VERBOSE && info("Include pcl::recognition headers")
    @timevb for name in [
        "cg/hough_3d.h",
        "cg/geometric_consistency.h",
        "hv/hv_go.h"]
        cxxinclude(joinpath(top, "pcl", "recognition", name))
    end

    # features
    VERBOSE && info("Include pcl::features headers")
    @timevb for name in ["normal_3d.h", "normal_3d_omp.h", "shot_omp.h", "board.h"]
        cxxinclude(joinpath(top, "pcl", "features", name))
    end

    # filters
    VERBOSE && info("Include pcl::filters headers")
    @timevb for name in ["uniform_sampling.h", "passthrough.h", "voxel_grid.h",
        "approximate_voxel_grid.h", "statistical_outlier_removal.h",
        "radius_outlier_removal.h", "extract_indices.h"]
        cxxinclude(joinpath(top, "pcl", "filters", name))
    end

    # kdtree
    VERBOSE && info("Include pcl::kdtree headers")
    @timevb for name in ["kdtree_flann.h", "impl/kdtree_flann.hpp"]
        cxxinclude(joinpath(top, "pcl", "kdtree", name))
    end

    # sample_consensus
    VERBOSE && info("Include pcl::sample_consensus headers")
    @timevb for name in ["model_types.h", "method_types.h"]
        cxxinclude(joinpath(top, "pcl", "sample_consensus", name))
    end

    # segmentation
    VERBOSE && info("Include pcl::segmentation headers")
    @timevb for name in ["sac_segmentation.h", "region_growing_rgb.h"]
        cxxinclude(joinpath(top, "pcl", "segmentation", name))
    end

    # search
    VERBOSE && info("Include pcl::search headers")
    @timevb for name in ["search.h", "kdtree.h"]
        cxxinclude(joinpath(top, "pcl", "search", name))
    end

    # tracking
    VERBOSE && info("Include pcl::tracking headers")
    @timevb for name in [
            "approx_nearest_pair_point_cloud_coherence.h",
            "coherence.h",
            "distance_coherence.h",
            "hsv_color_coherence.h",
            "kld_adaptive_particle_filter_omp.h",
            "nearest_pair_point_cloud_coherence.h",
            "particle_filter.h",
            "particle_filter_omp.h",
            "tracking.h",
            ]
        cxxinclude(joinpath(top, "pcl", "tracking", name))
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
elseif !isempty(searchdir(joinpath(system_include_top), "pcl-"))
    VERBOSE && info("Including headers from system path: $system_include_top")
    pcl_version = get_pcl_version(system_include_top)
    topdir_to_be_included = system_include_top
else
    error("Cannot find PCL headers")
end

VERBOSE && info("pcl_version: $pcl_version")
include_headers(joinpath(topdir_to_be_included, "pcl-$pcl_version"))

# Check boost version
const _BOOST_VERSION = icxx"BOOST_VERSION;"
const BOOST_VERSION_MAJOR = trunc(Int, _BOOST_VERSION / 100000)
const BOOST_VERSION_MINOR = trunc(Int, _BOOST_VERSION / 100 % 1000)
const BOOST_VERSION_PATCH = trunc(Int, _BOOST_VERSION % 100)
const BOOST_VERSION = join([BOOST_VERSION_MAJOR, BOOST_VERSION_MINOR, BOOST_VERSION_PATCH], ".")
VERBOSE && info("boost version: $BOOST_VERSION")

# Check FLANN vesion
# make sure FLANN_INCLUDE_DIR is addded as kind=C_System
cxxinclude(joinpath(FLANN_INCLUDE_DIR, "flann/flann.h"))
cxx"""
std::string getFLANNVersion() { return FLANN_VERSION_; }
"""
getFLANNVersion() = bytestring(@cxx getFLANNVersion())
VERBOSE && info("FLANN version: $(getFLANNVersion())")


for name in [
    "macros",
    "std",
    "constants",
    "common",
    "io",
    "filters",
    "features",
    "kdtree",
    "search",
    "octree",
    "recognition",
    "registration",
    "segmentation",
    "tracking",
    ]
    include(string(name, ".jl"))
end

has_vtk_backend && include("visualization.jl")

end # module pcl

end # module PCL
