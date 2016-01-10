# Filtering a PointCloud using a PassThrough filter
# http://www.pointclouds.org/documentation/tutorials/passthrough.php#passthrough

using PCL
using Cxx

cloud = pcl.PointCloud{pcl.PointXYZ}(5, 1)
cloud_filtered = pcl.PointCloud{pcl.PointXYZ}()

for i in 0:length(cloud)-1
    cloud[i,:x] = randn()
    cloud[i,:y] = randn()
    cloud[i,:z] = randn()
end

println("Cloud before filtering")
for i in 0:length(cloud)-1
    println("    ", cloud[i,:x], " ", cloud[i,:y], " ", cloud[i,:z])
end

pass = pcl.PassThrough{pcl.PointXYZ}()
pcl.setInputCloud(pass, cloud)
pcl.setFilterFieldName(pass, "z")
pcl.setFilterLimits(pass, 0.0, 1.0)
# pcl.setFilterLimitsNegative(pass, true)
pcl.filter(pass, cloud_filtered)

println("Cloud after filtering")
for i in 0:length(cloud_filtered)-1
    println("    ", cloud_filtered[i,:x], " ", cloud_filtered[i,:y],
        " ", cloud_filtered[i,:z])
end
