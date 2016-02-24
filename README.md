# PCL

A Julia interface for the [Point Cloud Library (PCL)](http://www.pointclouds.org/)

Note that PCL.jl was started as an experimental project and is very much work in progress. Currently only tested on osx 10.10.4.

## Why?

PCL.jl was stated to give us **fast prototyping** using PCL in interactive environment, without much loss of computational efficiency and flexibility. 

You might find that there is already a python binding ([strawlab/python-pcl](https://github.com/strawlab/python-pcl)) for the purpose, however, it lacks flexibility in particular; it only supports `PointXYZ` for point clouds at the moment. I guess the reason why it doesn't support arbitary point types for point clouds is that there is not elegant and simple way to expose C++ templated classes in python ([actually I tried](https://github.com/r9y9/pypcl), but couldn't figure it out). Since I wanted to use arbiraty point types for point clouds, I decided to create a new one.

The reasons why I started writing the [Julia](http://julialang.org/) (not python) binding are:

- The Julia C++ interface [Keno/Cxx.jl](https://github.com/Keno/Cxx.jl) is quite powerful.
 - It enables us to call C++ functions/methods directly from Julia *without any glue code* (unlike cython and swig), embed C++ into Julia and vise versa.
- Julia's types can be parameterized and exposing C++ templated classes is quite straightfood.
 - e.g. C++ `pcl::PointCloud<T>` can be represented as `PointCloud{T}` in Julia
 - I thought it's suitable to wrap PCL's heavily templated APIs. 

I couldn't imagine that I'd create a binding with another language. There is a blog post that explains why I created PCL.jl (in Japanese): [Trying-to-use-pcl-in-dynamic-language | LESS IS MORE](http://r9y9.github.io/blog/2016/01/18/trying-to-use-pcl-in-dynamic-language/).

### Comparison to python-pcl

Pros:

- Support *arbitaty* point types for point clouds, whereas python-pcl only supports `PointXYZ`
- Support PCL trunk
- More consistent APIs
- Can write mixed C++/Julia (see [examples/libfreenect2_grabbar.jl](https://github.com/r9y9/PCL.jl/blob/master/examples/libfreenect2_grabbar.jl) for example)
- PCLVisuzlier: Jupyter integration using off-screen rendering

Cons:

- Only works on osx for now
- Hard to build entire dependencies
- Sometime segfaults in Cxx.jl and Julia...
 
## Dependencies

- [Julia](https://github.com/JuliaLang/julia) (master)
- [Cxx.jl](https://github.com/Keno/Cxx.jl) (master)
- [pcl](https://github.com/PointCloudLibrary/pcl) (master)

## Installation

You first need to install [Keno/Cxx.jl](https://github.com/Keno/Cxx.jl). And then, you can install PCL.jl by:

```jl
Pkg.clone("https://github.com/r9y9/PCL.jl.git")
Pkg.build("PCL")
```

This should install PCL.jl and resolve its binary dependency property. Note that building PCL library (C++) is relatively complex (needs boost, flann, vtk, etc.), so you are recommended to check if PCL is able to build successfully first, and then build PCL.jl.

By default, `Pkg.build("PCL")` will try to find system-installed PCL libraries. If you have already PCL libraries installed, you don't have to re-build PCL for PCL.jl, however, this might cause incompatibility to PCL.jl depends on your PCL version. If you have any problem with system-installed ones, please try to ignore them and rebuild PCL.jl as follows:

In your terminal:
```
% export DYLD_LIBRARY_PATH=$HOME/.julia/v0.5/PCL/deps/usr/lib
```

and then in Julia REPL:

```jl
julia> ENV["PCLJL_LIBRARY_IGNORE_PATH"] = "/usr/lib:/usr/local/lib" # depends on your environment
julia> Pkg.build("PCL")
```

The PCL library paths will be stored in `PCL.pcl.libpcl_xxx` after loading PCL.jl. The library path should look like for example:

```jl
julia> using PCL
julia> PCL.pcl.libpcl_common
"$your_home_dir_path/.julia/v0.5/PCL/deps/usr/lib/libpcl_common.dylib"
```

### Notes for osx

```
brew install cmake openni openni2 qhull boost glew flann eigen libusb vtk
```

and 

```
export OPENNI2_INCLUDE=/usr/local/include/ni2
export OPENNI2_REDIST=/usr/local/lib/ni2
```

may help to build PCL.

## Documentation

Please check the http://www.pointclouds.org/documentation/.

## API

- Function names should be exactly same between Julia and C++.
- C++ template classes are available in Julia as templated types
- Single namespace `pcl` in Julia (**inconsistent between Julia and C++. Might change**)
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

```jl
pass = pcl.PassThrough{pcl.PointXYZ}()
pcl.setInputCloud(pass, cloud)
pcl.setFilterFieldName(pass, "z")
pcl.setFilterLimits(pass, 0.0, 1.0)
pcl.filter(pass, cloud_filtered)
```

## ENV

|Key|Default|Description|
|---|---|---|
|`PCLJL_VERBOSE`|1|Verbose output|
|`BOOST_INCLUDE_DIR`| `/usr/local/include` (`/usr/include/` for linux)|Boost include directory|
|`FLANN_INCLUDE_DIR`| `/usr/local/include` (`/usr/include/` for linux)|Flann include directory|
|`EIGEN_INCLUDE_DIR`| `/usr/local/include/eigen3` (`/usr/include/eigen3` for linux)|Eigen include directory|
|`VTK_INCLUDE_DIR_PARENT`| `/usr/local/include` (`/usr/include/` for linux)|Parent directory for VTK includes|
|`VTK_INCLUDE_DIR`| `${VTK_INCLUDE_DIR_PARENT}/vtk-${major}.${minor}`|VTK include directory (`${major}` and `${minor}` will be automatically detected)|
|`PCLJL_LOAD_${module}`|1|Controls which modules to be loaded (e.g. to diable visualization module, set `PCLJL_LOAD_visualization=0`)|




## How it works

### PCLVisualizer [[code]](examples/pcl_visualizer.jl)

<div align="center"><img src="examples/images/milk_cartoon_all_small_clorox.gif" /></div>

### Real-time Kinect v2 grabber [[code]](examples/libfreenect2_grabbar.jl)

<div align="center"><img src="examples/images/libfreenect2_integration.gif" /></div>

Requires [Libfreenect2.jl](https://github.com/r9y9/Libfreenect2.jl) and Kinect v2.

### 3D Object Recognition based on Correspondence Grouping [[code]](examples/correspondence_grouping.jl)

<div align="center"><img src="examples/images/correspondence_grouping.png" /></div>

### Hypothesis Verification for 3D Object Recognition [[code]](examples/global_hypothesis_verification.jl)

<div align="center"><img src="examples/images/global_hypothesis_verification.png" /></div>

### Extracting indices from a PointCloud [[code]](examples/extract_indices.jl)

<div align="center"><img src="examples/images/extract_indices.png" /></div>

### Color-based region growing segmentation [[code]](examples/region_growing_rgb_segmentation.jl)

<div align="center"><img src="examples/images/region_growing_rgb_segmentation.png" /></div>

### Jupyter integration

http://nbviewer.jupyter.org/gist/r9y9/6ed9a1d0b46993d374f5


You can find more examples in [examples directory](examples/).

## Something missing?

You have two solutions:

1. Write C++ using Cxx.jl. No glue code is required, but cannot get full benefit of Julia.
2. Write Julia glue code (e.g. Julia type for C++ class, with appropriate functions), need certain work to maintain though.

Currently PCL.jl takes the second approach. I'm still thinking what is the best way to make PCL.jl more usable.
