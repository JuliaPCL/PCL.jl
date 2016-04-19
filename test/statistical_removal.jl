# Removing outliers using a StatisticalOutlierRemoval filter
# http://www.pointclouds.org/documentation/tutorials/statistical_outlier.php

using PCLCommon
using PCLIO
using PCLFilters
using PCLVisualization

pcd_file = Pkg.dir("PCLIO", "test", "data", "table_scene_lms400.pcd")
cloud = PointCloud{PointXYZ}(pcd_file)
cloud_filtered = PointCloud{PointXYZ}()

sor = StatisticalOutlierRemoval{PointXYZ}()
setInputCloud(sor, cloud)
setMeanK(sor, 50)
setStddevMulThresh(sor, 1.0)
@time filter(sor, cloud_filtered)

println("PointCloud before filtering: $(length(cloud)) data points")
println("PointCloud before filtering: $(length(cloud_filtered)) data points")

if isdefined(:vis) && vis
    viewer = PCLVisualizer("pcl visualizeer")
    red_handler = PointCloudColorHandlerCustom(cloud, 255, 0, 0)
    green_handler = PointCloudColorHandlerCustom(cloud, 0, 255, 0)
    addPointCloud(viewer, cloud, red_handler, id="cloud")
    addPointCloud(viewer, cloud_filtered, green_handler, id="cloud_filtered")
    spin(viewer)
    close(viewer)
end
