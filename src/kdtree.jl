import Base: call

abstract AbstractKdTree

@inline handle(kd::AbstractKdTree) = kd.handle

type KdTreeFLANN{T} <: AbstractKdTree
    handle::cxxt"boost::shared_ptr<pcl::KdTreeFLANN<$T>>"
end
@defconstructor KdTreeFLANN{T}()

setInputCloud(kd::AbstractKdTree, cloud::PointCloud) =
    @cxx cxxpointer(handle(kd))->setInputCloud(handle(cloud))

function nearestKSearch(flann::KdTreeFLANN, point, k::Integer,
    k_indices::StdVector, k_sqr_distances::StdVector)
    k = Cint(k)
    found_neighs = @cxx cxxpointer(handle(flann))->nearestKSearch(
        point, k, k_indices, k_sqr_distances)
    Int(found_neighs)
end
