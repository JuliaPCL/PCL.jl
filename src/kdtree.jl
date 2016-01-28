import Base: call

abstract AbstractKdTree

@defpcltype KdTreeFLANN{T} <: AbstractKdTree "pcl::KdTreeFLANN"
@defptrconstructor KdTreeFLANN{T}() "pcl::KdTreeFLANN"
@defconstructor KdTreeFLANNVal{T}() "pcl::KdTreeFLANN"

setInputCloud(kd::AbstractKdTree, cloud::PointCloud) =
    icxx"$(handle(kd))->setInputCloud($(handle(cloud)));"

function nearestKSearch(flann::KdTreeFLANN, point, k::Integer,
    k_indices::StdVector, k_sqr_distances::StdVector)
    k = Cint(k)
    found_neighs = icxx"$(handle(flann))->nearestKSearch($point,
        $k, $k_indices, $k_sqr_distances);"
    Int(found_neighs)
end
