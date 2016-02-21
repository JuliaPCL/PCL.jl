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

pcl_common = library_dependency("libpcl_common", validate=default_validate)


github_root = "https://github.com/PointCloudLibrary/pcl"
provides(Sources,
         URI("$(github_root)/archive/$(pcl_git_version).tar.gz"),
         pcl_common,
         unpacked_dir="pcl-$(pcl_git_version)")

prefix = joinpath(BinDeps.depsdir(pcl_common), "usr")
srcdir = joinpath(BinDeps.depsdir(pcl_common), "src", "pcl-$(pcl_git_version)")

cmake_options = [
    "-DCMAKE_INSTALL_PREFIX=$prefix",
    "-DPCL_SHARED_LIBS=ON",
    "-DBUILD_tools=OFF",
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
          end), pcl_common, os = :Unix)

@BinDeps.install Dict(:libpcl_common => :libpcl_common)
