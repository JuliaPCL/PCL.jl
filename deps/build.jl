using BinDeps
using Compat

@BinDeps.setup

# specify the version of pcl we use
pcl_git_version = "master"

ignore_paths = split(strip(get(ENV, "PCLJL_LIBRARY_IGNORE_PATH", "")), ':')

# default validate function
default_validate = function(libpath, handle)
    for path in ignore_paths
        isempty(path) && continue
        ismatch(Regex("^$(path)"), libpath) && return false
    end
    return true
end

function pcl_library_dependency(s, validate=default_validate)
    library_dependency(s, validate=validate)
end

pcl_common = pcl_library_dependency("libpcl_common")
pcl_io = pcl_library_dependency("libpcl_io")
pcl_features = pcl_library_dependency("libpcl_features")
pcl_filters = pcl_library_dependency("libpcl_filters")
pcl_kdtree = pcl_library_dependency("libpcl_kdtree")
pcl_keypoints = pcl_library_dependency("libpcl_keypoints")
pcl_segmentation = pcl_library_dependency("libpcl_segmentation")
pcl_visualization = pcl_library_dependency("libpcl_visualization")
pcl_recognition = pcl_library_dependency("libpcl_recognition")
pcl_registration = pcl_library_dependency("libpcl_registration")
pcl_octree = pcl_library_dependency("libpcl_octree")
pcl_surface = pcl_library_dependency("libpcl_surface")
pcl_people = pcl_library_dependency("libpcl_people")
pcl_search = pcl_library_dependency("libpcl_search")
pcl_tracking = pcl_library_dependency("libpcl_tracking")

pcl_libs = [
    pcl_common,
    pcl_io,
    pcl_features,
    pcl_filters,
    pcl_kdtree,
    pcl_keypoints,
    pcl_segmentation,
    pcl_visualization,
    pcl_recognition,
    pcl_registration,
    pcl_octree,
    pcl_surface,
    pcl_people,
    pcl_search,
    pcl_tracking,
    ]

github_root = "https://github.com/PointCloudLibrary/pcl"
provides(Sources,
         URI("$(github_root)/archive/$(pcl_git_version).tar.gz"),
         pcl_libs,
         unpacked_dir="pcl-$(pcl_git_version)")

prefix = joinpath(BinDeps.depsdir(pcl_common), "usr")
srcdir = joinpath(BinDeps.depsdir(pcl_common), "src", "pcl-$(pcl_git_version)")

cmake_options = [
    "-DCMAKE_INSTALL_PREFIX=$prefix",
    "-DPCL_SHARED_LIBS=ON",
    "-DCMAKE_BUILD_TYPE=Release",
]

provides(SimpleBuild,
          (@build_steps begin
              GetSources(pcl_common)
              @build_steps begin
                  ChangeDirectory(srcdir)
                  `mkdir -p build`
                  @build_steps begin
                      ChangeDirectory(joinpath(srcdir, "build"))
                      `rm -f CMakeCache.txt`
                      `cmake $cmake_options ..`
                      `make -j4`
                      `make install`
                  end
                end
          end), pcl_libs, os = :Unix)

@BinDeps.install Dict(
    :libpcl_common => :libpcl_common,
    :libpcl_io => :libpcl_io,
    :libpcl_features => :libpcl_features,
    :libpcl_filters => :libpcl_filters,
    :libpcl_kdtree => :libpcl_kdtree,
    :libpcl_keypoints => :libpcl_keypoints,
    :libpcl_segmentation => :libpcl_segmentation,
    :libpcl_visualization => :libpcl_visualization,
    :libpcl_recognition => :libpcl_recognition,
    :libpcl_registration => :libpcl_registration,
    :libpcl_octree => :libpcl_octree,
    :libpcl_surface => :libpcl_surface,
    :libpcl_people => :libpcl_people,
    :libpcl_search => :libpcl_search,
    :libpcl_tracking => :libpcl_tracking,
    )
