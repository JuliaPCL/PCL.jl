# Removing outliers using a StatisticalOutlierRemoval filter
# http://www.pointclouds.org/documentation/tutorials/statistical_outlier.php

using PCL
using Cxx

vis = false

pcd_file = Pkg.dir("PCL", "test", "data", "table_scene_lms400.pcd")
cloud = pcl.PointCloud{pcl.PointXYZ}(pcd_file)
cloud_filtered = pcl.PointCloud{pcl.PointXYZ}()

sor = pcl.StatisticalOutlierRemoval{pcl.PointXYZ}()
pcl.setInputCloud(sor, cloud)
pcl.setMeanK(sor, 50)
pcl.setStddevMulThresh(sor, 1.0)
@time pcl.filter(sor, cloud_filtered)

println("PointCloud before filtering: $(length(cloud)) data points")
println("PointCloud before filtering: $(length(cloud_filtered)) data points")

if vis
    viewer = pcl.PCLVisualizer("pcl visualizeer")
    red_handler = pcl.PointCloudColorHandlerCustom(cloud, 255, 0, 0)
    green_handler = pcl.PointCloudColorHandlerCustom(cloud, 0, 255, 0)
    pcl.addPointCloud(viewer, cloud, red_handler, id="cloud")
    pcl.addPointCloud(viewer, cloud_filtered, green_handler, id="cloud_filtered")
    while !pcl.wasStopped(viewer)
        pcl.spinOnce(viewer)
    end
end
