# Downsampling a PointCloud using a VoxelGrid filter
# http://www.pointclouds.org/documentation/tutorials/voxel_grid.php

using PCL
using Cxx

vis = false

pcd_file = Pkg.dir("PCL", "test", "data", "table_scene_lms400.pcd")
cloud = pcl.PointCloud{pcl.PointXYZI}(pcd_file)
cloud_filtered = pcl.PointCloud{pcl.PointXYZI}()

sor = pcl.VoxelGrid{pcl.PointXYZI}()
pcl.setInputCloud(sor, cloud)
pcl.setLeafSize(sor, 0.01, 0.01, 0.01)
pcl.filter(sor, cloud_filtered)

println("PointCloud before filtering: $(length(cloud)) data points")
println("PointCloud before filtering: $(length(cloud_filtered)) data points")

if vis
    viewer = pcl.PCLVisualizer("pcl visualizer")
    pcl.addPointCloud(viewer, cloud_filtered)
    while !pcl.wasStopped(viewer)
        pcl.spinOnce(viewer)
    end
end
