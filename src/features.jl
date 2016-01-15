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
    handle::cxxt"boost::shared_ptr<pcl::NormalEstimationOMP<$PT,$NT>>"
end
@defconstructor NormalEstimationOMP{PT,NT}()

type NormalEstimation{PT,NT} <: AbstractNormalEstimator
    handle::cxxt"boost::shared_ptr<pcl::NormalEstimation<$PT,$NT>>"
end
@defconstructor NormalEstimation{PT,NT}()

setKSearch(n::AbstractNormalEstimator, k) =
    @cxx cxxpointer(handle(n))->setKSearch(k)

abstract AbstractSHOTEstimator <: AbstractFeatureEstimator

type SHOTEstimationOMP{PT,NT,OT} <: AbstractSHOTEstimator
    handle::cxxt"boost::shared_ptr<pcl::SHOTEstimationOMP<$PT,$NT,$OT>>"
end
@defconstructor SHOTEstimationOMP{PT,NT,OT}()

type SHOTEstimation{PT,NT,OT} <: AbstractSHOTEstimator
    handle::cxxt"boost::shared_ptr<pcl::SHOTEstimation<$PT,$NT,$OT>>"
end
@defconstructor SHOTEstimation{PT,NT,OT}()

type BOARDLocalReferenceFrameEstimation{T,N,F} <: AbstractFeatureEstimator
    handle::cxxt"boost::shared_ptr<pcl::BOARDLocalReferenceFrameEstimation<$T,$N,$F>>"
end
@defconstructor BOARDLocalReferenceFrameEstimation{T,N,F}()

setFindHoles(rfe::BOARDLocalReferenceFrameEstimation, v::Bool) =
    @cxx cxxpointer(handle(rfe))->setFindHoles(v)
