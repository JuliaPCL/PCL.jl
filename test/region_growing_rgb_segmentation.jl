# Color-based region growing segmentation
# http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php

using PCLCommon
using PCLFilters
using PCLFeatures
using PCLSearch
using PCLSegmentation
using PCLSampleConsensus
using PCLVisualization
using Cxx

pcd_file = Pkg.dir("PCLIO", "test", "data", "region_growing_rgb_tutorial.pcd")

tree = PCLSearch.KdTree{PointXYZRGB}()

cloud = PointCloud{PointXYZRGB}(pcd_file)
indices = icxx"boost::shared_ptr<std::vector<int>>(new std::vector<int>);"

@show length(cloud)
pass = PassThrough{PointXYZRGB}()
setInputCloud(pass, cloud)
setFilterFieldName(pass, "z")
setFilterLimits(pass, 0.0, 3.5)
filter(pass, indices)

len = convert(Int, icxx"$(indices)->size();")
@assert len > 0

reg = RegionGrowingRGB{PointXYZRGB}()
setInputCloud(reg, cloud)
setIndices(reg, indices)
PCLSegmentation.setSearchMethod(reg, tree.handle)
setDistanceThreshold(reg, 5)
setPointColorThreshold(reg, 6)
setRegionColorThreshold(reg, 10)
setMinClusterSize(reg, 600)

clusters = icxx"std::vector<pcl::PointIndices>();"
extract(reg, clusters)
@show convert(Int, length(clusters))

colored_cloud = getColoredCloud(reg)

if isdefined(:vis) && vis
    viewer = PCLVisualizer("pcl visualizer")
    addPointCloud(viewer, colored_cloud, id="colored_cloud")
    spin(viewer)
    close(viewer)
end
