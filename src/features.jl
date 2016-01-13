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

@inline handle(fe::AbstractFeatureEstimator) = fe.handle

setRadiusSearch(n::AbstractFeatureEstimator, rad) =
    @cxx cxxpointer(handle(n))->setRadiusSearch(rad)

setInputCloud(n::AbstractFeatureEstimator, cloud::PointCloud) =
    @cxx cxxpointer(handle(n))->setInputCloud(handle(cloud))

setInputNormals(n::AbstractFeatureEstimator, normals::PointCloud) =
    @cxx cxxpointer(handle(n))->setInputNormals(handle(normals))

setSearchSurface(n::AbstractFeatureEstimator, surface::PointCloud) =
    @cxx cxxpointer(handle(n))->setSearchSurface(handle(surface))

compute(n::AbstractFeatureEstimator, descriptors::PointCloud) =
    @cxx cxxpointer(handle(n))->compute(cxxderef(handle(descriptors)))

abstract AbstractNormalEstimator <: AbstractFeatureEstimator

type NormalEstimationOMP{PT,NT} <: AbstractNormalEstimator
    handle::SharedPtr # TODO: typed
end

function call{PT,NT}(::Type{NormalEstimationOMP{PT,NT}})
    handle = @sharedptr "pcl::NormalEstimationOMP<\$PT,\$NT>"
    NormalEstimationOMP{PT,NT}(handle)
end

type NormalEstimation{PT,NT} <: AbstractNormalEstimator
    handle::SharedPtr # TODO: typed
end

function call{PT,NT}(::Type{NormalEstimation{PT,NT}})
    handle = @sharedptr "pcl::NormalEstimation<\$PT,\$NT>"
    NormalEstimation{PT,NT}(handle)
end

setKSearch(n::AbstractNormalEstimator, k) =
    @cxx cxxpointer(handle(n))->setKSearch(k)

abstract AbstractSHOTEstimator <: AbstractFeatureEstimator

type SHOTEstimationOMP{PT,NT,OT} <: AbstractSHOTEstimator
    handle::SharedPtr # TODO: typed
end

function call{PT,NT,OT}(::Type{SHOTEstimationOMP{PT,NT,OT}})
    handle = @sharedptr "pcl::SHOTEstimationOMP<\$PT,\$NT,\$OT>"
    SHOTEstimationOMP{PT,NT,OT}(handle)
end

type SHOTEstimation{PT,NT,OT} <: AbstractSHOTEstimator
    handle::SharedPtr # TODO: typed
end

function call{PT,NT,OT}(::Type{SHOTEstimation{PT,NT,OT}})
    handle = @sharedptr "pcl::SHOTEstimation<\$PT,\$NT,\$OT>"
    SHOTEstimation{PT,NT,OT}(handle)
end

type BOARDLocalReferenceFrameEstimation{T,N,F} <: AbstractFeatureEstimator
    handle::SharedPtr # typed
end

function call{T,N,F}(::Type{BOARDLocalReferenceFrameEstimation{T,N,F}})
    handle = @sharedptr "pcl::BOARDLocalReferenceFrameEstimation<\$T,\$N,\$F>"
    BOARDLocalReferenceFrameEstimation{T,N,F}(handle)
end

setFindHoles(rfe::BOARDLocalReferenceFrameEstimation, v::Bool) =
    @cxx cxxpointer(handle(rfe))->setFindHoles(v)
