# Getting stareted

PCL Julia packages follows the
[PCL module structure](http://docs.pointclouds.org/trunk/modules.html).
If you need a particular module (e.g. [PCLCommon.jl](@ref)), then you can use the
specific module(s) as follows:

```julia
using PCLCommon, PCLIO;
cloud = PointCloud{PointXYZ}("your_pcd_data.pcd")
```

If you want to import all the PCL packages, then:

```julia
using PCL
```

Note that it will take a few times to load (30 seconds~) since it compiles all
the PCL packages.

## PointCloud{PointT}

The most frequently used type would be
[`PointCloud`](@ref)(defined in [PCLCommon.jl](@ref)), which represents
arbitary point type of point cloud (`pcl::PointCloud<PointT>::Ptr` in C++).
In this section we will show the basic usage of PointCloud type quickly.

### Create an empty point cloud

```julia
using PCLCommon
cloud_xyz = PointCloud{PointXYZ}()
```

For different point types, just change the type parameter as follows:

```julia
using PCLCommon
cloud_rgba = PointCloud{PointXYZRGBA}()
```

### Create a point cloud with specified size

```julia
using PCLCommon
cloud_xyz = PointCloud{PointXYZ}(100, 200) # width=100, height=200
```

## IO

### Load a PCD file

```julia
using PCLCommon, PCLIO
cloud_xyz = PointCloud{PointXYZ}("your_pcd_data.pcd")
```

needs [PCLIO.jl](@ref) in addition to [PCLCommon.jl](@ref).

## Filtering

### PassThrough filter

```julia
using PCLCommon, PCLIO, PCLFilters;
cloud = PointCloud{PointXYZRGB}("your_pcd_file.pcd")
cloud_filtered = PointCloud{PointXYZRGB}()

pass = PassThrough{PointXYZRGB}()
setInputCloud(pass, cloud)
setFilterFieldName(pass, "z")
setFilterLimits(pass, 0.0, 1.0)
filter(pass, cloud_filtered)
```

needs [PCLFilters.jl](@ref).

## Visualization

```julia
using PCLCommon, PCLIO, PCLVisualization;
cloud = PointCloud{PointXYZRGB}("your_pcd_file.pcd")
viewer = PCLVisualizer("pcl visualizer")
addPointCloud(viewer, cloud, id="input")
spin(viewer) # you will see the PCLVisualizer
```

needs [PCLVisualization.jl](@ref).

## Examples and tutorials

See [JuliaPCL/PCL/test](https://github.com/JuliaPCL/PCL.jl/tree/master/test)
directory for more examples. It includes more complex filtering, feature extraction,
recognition, tracking and visualization examples and also some PCL
tutorial translations as well.
