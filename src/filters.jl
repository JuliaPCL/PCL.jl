import Base: call, filter

typealias CxxUniformSampling{T} cxxt"boost::shared_ptr<pcl::UniformSampling<$T>>"

type UniformSampling{T}
    handle::CxxUniformSampling
end

function call{T}(::Type{UniformSampling{T}})
    handle = icxx"""
        boost::shared_ptr<pcl::UniformSampling<$T>>(
        new pcl::UniformSampling<$T>);"""
    UniformSampling{T}(handle)
end

setInputCloud(us::UniformSampling, cloud::PointCloud) =
    icxx"$(us.handle).get()->setInputCloud($(cloud.handle));"

setRadiusSearch(us::UniformSampling, ss) =
    icxx"$(us.handle).get()->setRadiusSearch($ss);"

filter(us::UniformSampling, cloud::PointCloud) =
    icxx"$(us.handle).get()->filter(*$(cloud.handle));"
