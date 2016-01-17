# Extracting indices from a PointCloud
# http://www.pointclouds.org/documentation/tutorials/extract_indices.php

using PCL
using Cxx

vis = false

pcd_file = Pkg.dir("PCL", "test", "data", "table_scene_lms400.pcd")

cloud = pcl.PointCloud{pcl.PointXYZ}(pcd_file)
cloud_filtered = pcl.PointCloud{pcl.PointXYZ}()

vgf = pcl.VoxelGrid{pcl.PointXYZ}()
pcl.setInputCloud(vgf, cloud)
pcl.setLeafSize(vgf, 0.01, 0.01, 0.01)
filter(vgf, cloud_filtered)

println("PointCloud before filtering: ", length(cloud), " data points")
println("PointCloud after filtering: ", length(cloud_filtered), " data points")

coefficients = pcl.ModelCoefficients()
inliers = pcl.PointIndices()

seg = pcl.SACSegmentation{pcl.PointXYZ}()
pcl.setOptimizeCoefficients(seg, true)
pcl.setModelType(seg, pcl.SACMODEL_PLANE)
pcl.setMethodType(seg, pcl.SAC_RANSAC)
pcl.setMaxIterations(seg, 1000)
pcl.setDistanceThreshold(seg, 0.01)

extract = pcl.ExtractIndices{pcl.PointXYZ}()
nr_points = length(cloud_filtered)

planes = []
while length(cloud_filtered) > 0.3 * nr_points
    pcl.setInputCloud(seg, cloud_filtered)
    pcl.segment(seg, inliers, coefficients)

    coefvalues = icxx"$(coefficients.handle)->values;"
    indices = icxx"$(inliers.handle)->indices;"

    if length(indices) == 0
        error("Could not estimate a planar model for the given dataset.")
    end

    cloud_p = pcl.PointCloud{pcl.PointXYZ}()
    cloud_f = pcl.PointCloud{pcl.PointXYZ}()

    pcl.setInputCloud(extract, cloud_filtered)
    pcl.setIndices(extract, inliers)
    pcl.setNegative(extract, false)
    filter(extract, cloud_p)
    println("PointCloud representing the planar component:",
        length(cloud_p), " data points")

    pcl.setNegative(extract, true)
    filter(extract, cloud_f)
    cloud_filtered = cloud_f

    push!(planes, cloud_p)
end

neg_plane = cloud_filtered

if vis
    viewer = pcl.PCLVisualizer("pcl visualizer")
    @show length(planes)
    for i in 1:length(planes)
        color = i == 1 ? (255,0,0) : i == 2 ? (0,255,0) : tuple(rand(UInt, 3))
        @show color
        handler = pcl.PointCloudColorHandlerCustom(planes[i], color...)
        pcl.addPointCloud(viewer, planes[i], handler, id="plane $i")
    end
    white_hanlder = pcl.PointCloudColorHandlerCustom(neg_plane, 255,255,255)
    pcl.addPointCloud(viewer, neg_plane, white_hanlder, id="neg plane")
    while !pcl.wasStopped(viewer)
        pcl.spinOnce(viewer)
    end
end
