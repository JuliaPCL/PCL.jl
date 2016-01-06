import Base: call

typealias CxxKdTreeFLANN{T} cxxt"boost::shared_ptr<pcl::KdTreeFLANN<$T>>"

type KdTreeFLANN{T}
    handle::CxxKdTreeFLANN
end

function call{T}(::Type{KdTreeFLANN{T}})
    handle = icxx"""
        boost::shared_ptr<pcl::KdTreeFLANN<$T>>(
        new pcl::KdTreeFLANN<$T>);"""
    KdTreeFLANN{T}(handle)
end

setInputCloud(us::KdTreeFLANN, cloud::PointCloud) =
    icxx"$(us.handle).get()->setInputCloud($(cloud.handle));"

function nearestKSearch(flann::KdTreeFLANN, point, k::Integer,
    k_indices::StdVector, k_sqr_distances::StdVector)
    k = Cint(k)
    found_neighs = icxx"""
        $(flann.handle).get()->nearestKSearch($point, $k, $k_indices,
            $k_sqr_distances);
    """
    Int(found_neighs)
end
