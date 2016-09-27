__precompile__(false)

# ls -l ~/.julia/v0.6  | grep PCL | awk '{print $9}' | sort | grep -v "^PCL$"|\
#   awk '{print "-", "["$1"]""(https://github.com/JuliaPCL/"$1".jl)"}'
"""
The Julia wrapper for Point Cloud Library (PCL)

With the packages, e.g., you can visualize a point cloud in 5 lines:

```julia
using PCLCommon, PCLIO, PCLVisualization
cloud = PointCloud{PointXYZRGB}("your_pcd_file.pcd")
viewer = PCLVisualizer("pcl visualizer")
addPointCloud(viewer, cloud, id="input")
spin(viewer) # you will see the PCLVisualizer
```

![](assets/pcl_visualizer_5lines.png)

## Why?

PCL.jl was stated to give us **fast prototyping** using PCL in interactive
environment, without much loss of computational efficiency and flexibility.

You might find that there is already a python binding
([strawlab/python-pcl](https://github.com/strawlab/python-pcl)) for the purpose,
however, it lacks flexibility in particular; it only supports `PointXYZ` for
point clouds at the moment. I guess the reason why it doesn't support arbitary
point types for point clouds is that there is not elegant and simple way to
expose C++ templated classes in python
([actually I tried](https://github.com/r9y9/pypcl), but couldn't figure it out).
Since I wanted to use arbiraty point types for point clouds, I decided to create
a new one.

The reasons why I started writing the [Julia](http://julialang.org/) (not python)
binding are:

- **The Julia C++ interface [Keno/Cxx.jl](https://github.com/Keno/Cxx.jl) is quite powerful**: It enables us to call C++ functions/methods directly from Julia *without any glue code* (unlike cython and swig), embed C++ into Julia and vise versa.
- **Julia's types can be parameterized and exposing C++ templated classes is quite straightfood**: e.g. C++ `pcl::PointCloud<PointT>` can be represented as `PointCloud{PointT}` in Julia.

### Comparison to python-pcl

Pros:

- Support *arbitaty* point types for point clouds, whereas python-pcl only supports `PointXYZ`
- Support PCL 1.8 or later
- More consistent APIs
- Can write mixed C++/Julia
- PCLVisualizer: Jupyter integration using off-screen rendering

Cons:

- Only works on osx for now
- Hard to build entire dependencies
- Sometime segfaults in Cxx.jl and Julia when doing wrong

### Dependencies (in short)

- [pcl](https://github.com/PointCloudLibrary/pcl)
- [Julia](https://github.com/JuliaLang/julia)
- [Cxx.jl](https://github.com/Keno/Cxx.jl)

## API guidelines

- Function names should be exactly same between Julia and C++.
- C++ template classes are available in Julia as templated types
- C++ dereferences which sometimes needed in C++, are hidden in implementation in Julia

e.g.

In C++:

```cpp
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud (cloud);
pass.setFilterFieldName ("z");
pass.setFilterLimits (0.0, 1.0);
pass.filter (*cloud_filtered);
```

In Julia:

```julia
pass = PassThrough{PointXYZ}()
setInputCloud(pass, cloud)
setFilterFieldName(pass, "z")
setFilterLimits(pass, 0.0, 1.0)
filter(pass, cloud_filtered)
```

## Package structure

To simplify development and minimize dependencies, the Julia wrapper consists of
the packages below:

- [LibPCL.jl](@ref)
- [PCLCommon.jl](@ref)
- [PCLFeatures.jl](@ref)
- [PCLFilters.jl](@ref)
- [PCLIO.jl](@ref)
- [PCLKDTree.jl](@ref)
- [PCLKeyPoints.jl](@ref)
- [PCLOctree.jl](@ref)
- [PCLRecognition.jl](@ref)
- [PCLRegistration.jl](@ref)
- [PCLSampleConsensus.jl](@ref)
- [PCLSearch.jl](@ref)
- [PCLSegmentation.jl](@ref)
- [PCLSurface.jl](@ref)
- [PCLTracking.jl](@ref)
- [PCLVisualization.jl](@ref)

following the [PCL module structure](http://docs.pointclouds.org/trunk/modules.html)
except for the `LibPCL.jl` which manages binary dependencies (i.e. search
installed PCL shared libraries or build and install them if not found).
"""
module PCL

using Reexport
using DocStringExtensions

@reexport using PCLCommon
@reexport using PCLIO
@reexport using PCLFeatures
@reexport using PCLFilters
@reexport using PCLVisualization
@reexport using PCLKDTree
@reexport using PCLOctree
@reexport using PCLRecognition
@reexport using PCLSampleConsensus
@reexport using PCLSearch
@reexport using PCLSegmentation
@reexport using PCLTracking
@reexport using PCLRegistration
@reexport using PCLKeyPoints
@reexport using PCLSurface

end # module PCL
