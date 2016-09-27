# Installation

PCL Julia packages have fairly complex dependencies, so please take a careful
look at the installation guide for each dependency.

## Requirements

### Platform

Currently only tested on OSX.

### PCL

You need to install [PCL](https://github.com/PointCloudLibrary/pcl) 1.8 or later
with shared library option ON. Since PCL have a fair amount of dependencies,
you might suffer from build issues. For macOSX, the following command will help
to install PCL dependencies:

```
brew install cmake openni openni2 qhull boost glew flann eigen libusb vtk
export OPENNI2_INCLUDE=/usr/local/include/ni2
export OPENNI2_REDIST=/usr/local/lib/ni2
```

If you have PCL dependencies installed, then recommended installation steps are as
follows:

```
git clone https://github.com/PointCloudLibrary/pcl && cd pcl
mkdir build && cd build
cmake -DPCL_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_global_tests=OFF -DBUILD_tools=OFF ..
make -j4
make -j4 install
```

### Julia

[Julia](https://github.com/JuliaLang/julia) 0.5 or later is required.
Install Julia v0.5 from binary distributions or build it from source.

### Cxx.jl

You need to install [r9y9/Cxx.jl](https://github.com/r9y9/Cxx.jl) [^1]. Building
Cxx.jl is a bit complex, as it builds its own llvm and clang by default. Please
take a careful look at the installation guide of Cxx.jl. Ideally, the
installation can be done as follows:

```julia
Pkg.clone("https://github.com/r9y9/Cxx.jl")
Pkg.build("Cxx")
```

[^1]: Forked from [Keno/Cxx.jl](https://github.com/Keno/Cxx.jl) for RTTI support. [Keno/Cxx.jl](https://github.com/Keno/Cxx.jl) doesn't provides a way to enable RTTI without code modification for now.

## Install PCL Julia packages

### Set environmental variables property

There are a few environmental variables to be set property before installing
Julia packages, otherwise it throws errors during package compilation time. You must
tell locations of PCL dependencies (`boost`, `FLANN`, `Eigen` and `VTK`) to Julia via
the following environmental variables:

|Key|Default|Description|
|---|---|---|
|`BOOST_INCLUDE_DIR`| `/usr/local/include` (`/usr/include/` for linux)|Boost include directory|
|`FLANN_INCLUDE_DIR`| `/usr/local/include` (`/usr/include/` for linux)|Flann include directory|
|`EIGEN_INCLUDE_DIR`| `/usr/local/include/eigen3` (`/usr/include/eigen3` for linux)|Eigen include directory|
|`VTK_INCLUDE_DIR_PARENT`| `/usr/local/include` (`/usr/include/` for linux)|Parent directory for VTK includes|
|`VTK_INCLUDE_DIR`| `${VTK_INCLUDE_DIR_PARENT}/vtk-${major}.${minor}`|VTK include directory (`${major}` and `${minor}` will be automatically detected)|

### Clone and build

You are almost there! Clone the packages:

```julia
Pkg.clone("https://github.com/JuliaPCL/PCLCommon.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLIO.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLFeatures.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLFilters.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLVisualization.jl")
Pkg.clone("https://github.com/JuliaPCL/LibPCL.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLKDTree.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLOctree.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLRecognition.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLSampleConsensus.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLSearch.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLSegmentation.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLTracking.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLRegistration.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLKeyPoints.jl")
Pkg.clone("https://github.com/JuliaPCL/PCL.jl")
```

and then:

```julia
Pkg.build("LibPCL")
```

which searches installed PCL shared libraries. If it fails, please make sure again
that you have set correct environmental variables. If you don't have PCL installed,
`Pkg.build("LibPCL")` will try to build and install them into the LibPCL package
directory but not recommended, unless if you have perfect requirements to build PCL.

## Test if the installation succeeded

```julia
Pkg.test("PCL")
```

If it succeeded, installation is done. If you encounter errors even though all
the previous steps succeeded, please file a bug report.
