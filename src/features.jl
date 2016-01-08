import Base: call

"""
Feature estimators that implemtents the following methods:

- setRadiusSearch
- setInputCloud
- setInputNormals
- setSearchSurface
- compute
"""
abstract AbstractFeatureEstimator

handle(fe::AbstractFeatureEstimator) = fe.handle

setRadiusSearch(n::AbstractFeatureEstimator, rad) =
    icxx"$(handle(n)).get()->setRadiusSearch($rad);"

setInputCloud(n::AbstractFeatureEstimator, cloud::PointCloud) =
    icxx"$(handle(n)).get()->setInputCloud($(handle(cloud)));"

setInputNormals(n::AbstractFeatureEstimator, normals::PointCloud) =
    icxx"$(handle(n)).get()->setInputNormals($(handle(normals)));"

setSearchSurface(n::AbstractFeatureEstimator, surface::PointCloud) =
    icxx"$(handle(n)).get()->setSearchSurface($(handle(surface)));"

compute(n::AbstractFeatureEstimator, descriptors::PointCloud) =
    icxx"$(handle(n)).get()->compute(*$(handle(descriptors)));"


type NormalEstimationOMP{PT,NT} <: AbstractFeatureEstimator
    handle::SharedPtr # TODO: typed
end

function call{PT,NT}(::Type{NormalEstimationOMP{PT,NT}})
    handle = icxx"""
        boost::shared_ptr<pcl::NormalEstimationOMP<$PT,$NT>>(
        new pcl::NormalEstimationOMP<$PT,$NT>);"""
    NormalEstimationOMP{PT,NT}(handle)
end

setKSearch(n::NormalEstimationOMP, k) = icxx"$(handle(n)).get()->setKSearch($k);"

type SHOTEstimationOMP{PT,NT,OT} <: AbstractFeatureEstimator
    handle::SharedPtr # TODO: typed
end

function call{PT,NT,OT}(::Type{SHOTEstimationOMP{PT,NT,OT}})
    handle = icxx"""
        boost::shared_ptr<pcl::SHOTEstimationOMP<$PT,$NT,$OT>>(
        new pcl::SHOTEstimationOMP<$PT,$NT,$OT>);"""
    SHOTEstimationOMP{PT,NT,OT}(handle)
end

type BOARDLocalReferenceFrameEstimation{T,N,F} <: AbstractFeatureEstimator
    handle::SharedPtr # typed
end

function call{T,N,F}(::Type{BOARDLocalReferenceFrameEstimation{T,N,F}})
    handle = icxx"""
        boost::shared_ptr<pcl::BOARDLocalReferenceFrameEstimation<$T,$N,$F>>(
        new pcl::BOARDLocalReferenceFrameEstimation<$T,$N,$F>);"""
    BOARDLocalReferenceFrameEstimation{T,N,F}(handle)
end

setFindHoles(rfe::BOARDLocalReferenceFrameEstimation, v::Bool) =
    icxx"$(handle(rfe)).get()->setFindHoles($v);"
