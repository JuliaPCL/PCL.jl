# Clustering example following the tutorial:
# http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
# essentially same but implementation details are different.
# We use downsampled and plane removed cloud here for simplicity

using PCLCommon,PCLVisualization,PCLIO,PCLFilters,PCLSegmentation,Cxx

T = PointXYZ

pcd_file =joinpath(dirname(@__FILE__), "pcd", "2016-12-15-23-45-38-884_concat.pcd")
cloud = PointCloud{T}(pcd_file)

# Remove noise
sor = StatisticalOutlierRemoval{T}()
setMeanK(sor, 50)
setStddevMulThresh(sor, 1.0)
setInputCloud(sor, cloud)
filter(sor, cloud)

tree = PCLSearch.KdTree{T}()
setInputCloud(tree, cloud)

cluster_indices = icxx"std::vector<pcl::PointIndices>();"

ec = EuclideanClusterExtraction{T}()
PCLSegmentation.setClusterTolerance(ec, 0.02)
PCLSegmentation.setMinClusterSize(ec, 100)
PCLSegmentation.setMaxClusterSize(ec, 25000)
PCLSegmentation.setSearchMethod(ec, tree)
setInputCloud(ec, cloud)
@time extract(ec, cluster_indices)

clusters = []
for i in 0:Int(length(cluster_indices))-1
    c = PointCloud{T}()
    icxx"""
    auto cloudp = $(c.handle);
    for (auto idx : $(cluster_indices)[$i].indices) {
        cloudp->push_back($(cloud.handle)->points[idx]);
    }
    cloudp->width = cloudp->points.size();
    cloudp->height = 1;
    cloudp->is_dense = true;
    """
    push!(clusters, c)
end

@show length(clusters)
@assert length(clusters) > 0

if isdefined(:vis) && vis
    viewer = PCLVisualizer("pcl visualizer")
    addCoordinateSystem(viewer, 1.2, 0,0,0)
    addPointCloud(viewer, cloud, PointCloudColorHandlerCustom(cloud, 255, 0, 0), id="cloud")
    for (idx, cluster) in enumerate(clusters)
        addPointCloud(viewer, cluster,
            PointCloudColorHandlerCustom(cluster, rand(0:255),rand(0:255),rand(0:255)),
            id="cluster #$idx")
    end
    spin(viewer)
    close(viewer); viewer = 0; gc();gc()
end
