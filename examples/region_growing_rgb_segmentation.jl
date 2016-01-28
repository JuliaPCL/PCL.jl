# Color-based region growing segmentation
# http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php

using PCL
using Cxx

vis = false

pcd_file = Pkg.dir("PCL", "test", "data", "region_growing_rgb_tutorial.pcd")

tree = pcl.KdTree{pcl.PointXYZRGB}()

cloud = pcl.PointCloud{pcl.PointXYZRGB}(pcd_file)
indices = icxx"boost::shared_ptr<std::vector<int>>(new std::vector<int>);"

@show length(cloud)
pass = pcl.PassThrough{pcl.PointXYZRGB}()
pcl.setInputCloud(pass, cloud)
pcl.setFilterFieldName(pass, "z")
pcl.setFilterLimits(pass, 0.0, 3.5)
filter(pass, indices)

len = convert(Int, icxx"$(indices)->size();")
@assert len > 0

reg = pcl.RegionGrowingRGB{pcl.PointXYZRGB}()
pcl.setInputCloud(reg, cloud)
pcl.setIndices(reg, indices)
pcl.setSearchMethod(reg, tree.handle)
pcl.setDistanceThreshold(reg, 5)
pcl.setPointColorThreshold(reg, 6)
pcl.setRegionColorThreshold(reg, 10)
pcl.setMinClusterSize(reg, 600)

clusters = icxx"std::vector<pcl::PointIndices>();"
pcl.extract(reg, clusters)
@show convert(Int, length(clusters))

colored_cloud = pcl.getColoredCloud(reg)

if vis
    viewer = pcl.PCLVisualizer("pcl visualizer")
    pcl.addPointCloud(viewer, colored_cloud, id="colored_cloud")
    while !pcl.wasStopped(viewer)
        pcl.spinOnce(viewer)
    end
end
