# Plane model segmentation
# http://pointclouds.org/documentation/tutorials/planar_segmentation.php

using PCL
using Cxx

function showxyz(c, i)
    x,y,z = c[i,:x],c[i,:y],c[i,:z]
    println("x,y,z: $x,$y,$z")
end

cloud = pcl.PointCloud{pcl.PointXYZ}(15, 1)
for i = 0:length(cloud)-1
    cloud[i,:x] = randn()
    cloud[i,:y] = randn()
    cloud[i,:z] = 1.0
end

# Set a few outliers
cloud[0,:z] = 2.0
cloud[3,:z] = -2.0
cloud[6,:z] = 4.0

println("Point cloud data: $(length(cloud)) points")
for i = 0:length(cloud)-1
    showxyz(cloud, i)
end

coefficients = pcl.ModelCoefficients()
inliers = pcl.PointIndices()

seg = pcl.SACSegmentation{pcl.PointXYZ}()
pcl.setOptimizeCoefficients(seg, true)
pcl.setModelType(seg, pcl.SACMODEL_PLANE)
pcl.setMethodType(seg, pcl.SAC_RANSAC)
pcl.setDistanceThreshold(seg, 0.01)

pcl.setInputCloud(seg, cloud)
pcl.segment(seg, inliers, coefficients)

coefvalues = icxx"$(coefficients.handle)->values;"
indices = icxx"$(inliers.handle)->indices;"

if length(inliers) == 0
    error("Could not estimate a planar model for the given dataset.")
end

print("Model Coefficients: ")
for i in 0:length(coefvalues)-1
    print(Float32(coefvalues[i]), " ")
end
println("")

println("Model inliers: $(length(indices))")
for i in 0:length(indices)-1
    print(i, "  ")
    showxyz(cloud, Int32(indices[i]))
end
