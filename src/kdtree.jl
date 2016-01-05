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
