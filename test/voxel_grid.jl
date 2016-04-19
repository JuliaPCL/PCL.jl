# Downsampling a PointCloud using a VoxelGrid filter
# http://www.pointclouds.org/documentation/tutorials/voxel_grid.php

using PCLCommon
using PCLIO
using PCLFilters
using PCLVisualization

pcd_file = Pkg.dir("PCLIO", "test", "data", "table_scene_lms400.pcd")
cloud = PointCloud{PointXYZI}(pcd_file)
cloud_filtered = PointCloud{PointXYZI}()

sor = VoxelGrid{PointXYZI}()
setInputCloud(sor, cloud)
setLeafSize(sor, 0.01, 0.01, 0.01)
filter(sor, cloud_filtered)

println("PointCloud before filtering: $(length(cloud)) data points")
println("PointCloud before filtering: $(length(cloud_filtered)) data points")

if isdefined(:vis) && vis
    viewer = PCLVisualizer("pcl visualizer")
    red_handler = PointCloudColorHandlerCustom(cloud, 255, 0, 0)
    green_handler = PointCloudColorHandlerCustom(cloud, 0, 255, 0)
    addPointCloud(viewer, cloud, red_handler, id="cloud")
    addPointCloud(viewer, cloud_filtered, green_handler, id="cloud_filtered")
    spin(viewer)
    close(viewer)
end
