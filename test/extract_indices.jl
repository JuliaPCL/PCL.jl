# exing indices from a PointCloud
# http://www.pointclouds.org/documentation/tutorials/extract_indices.php

using PCLCommon
using PCLIO
using PCLFilters
using PCLSegmentation
using PCLSampleConsensus
using PCLVisualization
using Cxx

PT = PointXYZ

pcd_file = Pkg.dir("PCLIO", "test", "data", "table_scene_lms400.pcd")

cloud = PointCloud{PT}(pcd_file)
cloud_filtered = PointCloud{PT}()

vgf = VoxelGrid{PT}()
setInputCloud(vgf, cloud)
setLeafSize(vgf, 0.01, 0.01, 0.01)
filter(vgf, cloud_filtered)

println("PointCloud before filtering: ", length(cloud), " data points")
println("PointCloud after filtering: ", length(cloud_filtered), " data points")

coefficients = ModelCoefficients()
inliers = PointIndices()

seg = SACSegmentation{PT}()
setOptimizeCoefficients(seg, true)
setModelType(seg, SACMODEL_PLANE)
setMethodType(seg, SAC_RANSAC)
setMaxIterations(seg, 1000)
setDistanceThreshold(seg, 0.01)

ex = ExtractIndices{PT}()
nr_points = length(cloud_filtered)

planes = []
coefs = []
while length(cloud_filtered) > 0.3 * nr_points
    setInputCloud(seg, cloud_filtered)
    segment(seg, inliers, coefficients)

    coefvalues = icxx"$(coefficients.handle)->values;"
    indices = icxx"$(inliers.handle)->indices;"

    push!(coefs, coefvalues)

    if length(indices) == 0
        error("Could not estimate a planar model for the given dataset.")
    end

    cloud_p = PointCloud{PT}()
    cloud_f = PointCloud{PT}()

    setInputCloud(ex, cloud_filtered)
    setIndices(ex, inliers)
    setNegative(ex, false)
    filter(ex, cloud_p)
    println("PointCloud representing the planar component:",
        length(cloud_p), " data points")

    setNegative(ex, true)
    filter(ex, cloud_f)
    cloud_filtered = cloud_f

    push!(planes, cloud_p)
end

neg_plane = cloud_filtered

if isdefined(:vis) && vis
    viewer = PCLVisualizer("pcl visualizer")
    addCoordinateSystem(viewer, 1.2, 0,0,0)
    @show length(planes)
    for i in 1:length(planes)
        color = i == 1 ? (255,0,0) : i == 2 ? (0,255,0) : tuple(rand(UInt, 3)...)
        handler = PointCloudColorHandlerCustom(planes[i], color...)
        addPointCloud(viewer, planes[i], handler, id="plane $i")
    end
    white_hanlder = PointCloudColorHandlerCustom(neg_plane, 255,255,255)
    addPointCloud(viewer, neg_plane, white_hanlder, id="neg plane")
    spin(viewer)
    close(viewer)
    viewer = 0; gc(); gc();
end
