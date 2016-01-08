import Base: call

abstract AbstractKdTree

handle(kd::AbstractKdTree) = kd.handle

type KdTreeFLANN{T} <: AbstractKdTree
    handle::SharedPtr
end

function call{T}(::Type{KdTreeFLANN{T}})
    handle = icxx"""
        boost::shared_ptr<pcl::KdTreeFLANN<$T>>(
        new pcl::KdTreeFLANN<$T>);"""
    KdTreeFLANN{T}(handle)
end

setInputCloud(kd::AbstractKdTree, cloud::PointCloud) =
    icxx"$(handle(kd)).get()->setInputCloud($(handle(cloud)));"

function nearestKSearch(flann::KdTreeFLANN, point, k::Integer,
    k_indices::StdVector, k_sqr_distances::StdVector)
    k = Cint(k)
    found_neighs = icxx"""
        $(flann.handle).get()->nearestKSearch($point, $k, $k_indices,
            $k_sqr_distances);
    """
    Int(found_neighs)
end
