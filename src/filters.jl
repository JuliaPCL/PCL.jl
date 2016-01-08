import Base: call, filter

abstract AbstractFilter

@inline handle(f::AbstractFilter) = f.handle

typealias CxxUniformSampling{T} cxxt"boost::shared_ptr<pcl::UniformSampling<$T>>"

type UniformSampling{T} <: AbstractFilter
    handle::CxxUniformSampling
end

function call{T}(::Type{UniformSampling{T}})
    handle = icxx"""
        boost::shared_ptr<pcl::UniformSampling<$T>>(
        new pcl::UniformSampling<$T>);"""
    UniformSampling{T}(handle)
end

setInputCloud(us::UniformSampling, cloud::PointCloud) =
    @cxx cxxpointer(handle(us))->setInputCloud(handle(cloud))
setRadiusSearch(us::UniformSampling, ss) =
    @cxx cxxpointer(handle(us))->setRadiusSearch(ss)
filter(us::UniformSampling, cloud::PointCloud) =
    @cxx cxxpointer(handle(us))->filter(cxxderef(handle(cloud)))
