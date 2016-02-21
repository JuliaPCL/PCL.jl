__precompile__(false)

module PCL

export pcl
module pcl

const VERBOSE = Bool(parse(Int, get(ENV, "PCLJL_VERBOSE", "1")))

const BOOST_INCLUDE_DIR = get(ENV, "BOOST_INCLUDE_DIR",
    @osx? "/usr/local/include" : "/usr/include")
const FLANN_INCLUDE_DIR = get(ENV, "FLANN_INCLUDE_DIR",
    @osx? "/usr/local/include" : "/usr/include")
const EIGEN_INCLUDE_DIR = get(ENV, "EIGEN_INCLUDE_DIR",
    @osx? "/usr/local/include/eigen3" : "/usr/include/eigen3")

searchdir(path, key) = filter(x->contains(x, key), readdir(path))

const VTK_INCLUDE_DIR_PARENT = get(ENV, "VTK_INCLUDE_DIR_PARENT",
    @osx? "/usr/local/include" : "/usr/include")

# Search GUI backend
vtk_dirs = searchdir(VTK_INCLUDE_DIR_PARENT, "vtk-")
const VTK_INCLUDE_DIR = get(ENV, "VTK_INCLUDE_DIR",
    isempty(vtk_dirs) ? "" : joinpath(VTK_INCLUDE_DIR_PARENT, vtk_dirs[1]))

const has_vtk_backend = isfile(joinpath(VTK_INCLUDE_DIR, "vtkVersion.h"))
if has_vtk_backend
    VERBOSE && info("vtk include directory found: $VTK_INCLUDE_DIR")
end

using BinDeps

# Load required dependency
deps = joinpath(Pkg.dir("PCL"), "deps", "deps.jl")
if isfile(deps)
    include(deps)
else
    error("PCL not properly installed. Please run Pkg.build(\"PCL\")")
end
Libdl.dlopen(libpcl_common, Libdl.RTLD_GLOBAL)

# Map that represents which modules to be enabled
modules = Dict{Symbol,Bool}(:common => true)

# Load optional dependency
VERBOSE && info("Loading optional dependencies")
libdir = dirname(libpcl_common)
ext = splitext(libpcl_common)[2]
for mod in [
        :filters,
        :search,
        :surface,
        :sample_consensus,
        :kdtree,
        :people,
        :keypoints,
        :recognition,
        :features,
        :registration,
        :octree,
        :io,
        :tracking,
        :segmentation,
        :visualization,
        ]
    libpath = joinpath(libdir, string("libpcl_", mod, ext))
    if !Bool(parse(Int, get(ENV, "PCLJL_LOAD_$mod", "1")))
        VERBOSE && println("disabled: module $(string(mod))")
        modules[mod] = false
        continue
    end
    if isfile(libpath)
        modules[mod] = true
        libname = symbol(:libpcl_, mod)
        @eval begin
            global const $libname = $libpath
            Libdl.dlopen($libname, Libdl.RTLD_GLOBAL)
        end
        VERBOSE && println("enabled: module $(string(mod))")
    else
        VERBOSE && warn("not found: module $(string(mod))")
        modules[mod] = false
    end
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

# Make sure vtk libraries are loaded before calling @cxx vtkVersion::xxx()
if has_vtk_backend && modules[:visualization]
    cxxinclude(joinpath(VTK_INCLUDE_DIR, "vtkVersion.h"))
    global const vtk_version = bytestring(@cxx vtkVersion::GetVTKVersion())
    VERBOSE && info("vtk version: $vtk_version")
end

function add_header_dirs(top)
    # Boost (required)
    addHeaderDir(BOOST_INCLUDE_DIR, kind=C_System)

    # Eigen (required)
    addHeaderDir(EIGEN_INCLUDE_DIR, kind=C_System)

    # FLANN (required)
    addHeaderDir(FLANN_INCLUDE_DIR, kind=C_System)

    # VTK (optional)
    has_vtk_backend && addHeaderDir(VTK_INCLUDE_DIR, kind=C_System)

    # PCL top directory
    addHeaderDir(top, kind=C_System)
    addHeaderDir(joinpath(top, "pcl"), kind=C_System)
end

function include_toplevel_headers()
    # This is necesarry, but not sure why...
    cxx"""#include <iostream>"""

    # top level
    VERBOSE && info("Include pcl top-level headers")
    @timevb cxx"""
    #include <pcl/pcl_base.h>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl/correspondence.h>
    #include <pcl/ModelCoefficients.h>
    """
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
const base_include_path = joinpath(topdir_to_be_included, "pcl-$pcl_version")

# Add header search paths
add_header_dirs(base_include_path)

# Once we add proper header search paths, include necessary headers
include_toplevel_headers()

# Check boost version
cxxinclude(joinpath(BOOST_INCLUDE_DIR, "boost/version.hpp"))
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


for name in ["macros", "std"]
    include(string(name, ".jl"))
end

modules[:common] && include("common.jl")
modules[:visualization] && include("visualization.jl")
modules[:io] && include("io.jl")
modules[:filters] && include("filters.jl")
modules[:features] && include("features.jl")
modules[:kdtree] && include("kdtree.jl")
modules[:search] && include("search.jl")
modules[:octree] && include("octree.jl")
modules[:surface] && include("surface.jl")
modules[:sample_consensus] && include("sample_consensus.jl")
modules[:keypoints] && include("keypoints.jl")
modules[:people] && include("people.jl")
modules[:recognition] && include("recognition.jl")
modules[:registration] && include("registration.jl")
modules[:segmentation] && include("segmentation.jl")
modules[:tracking] && include("tracking.jl")

end # module pcl

end # module PCL
