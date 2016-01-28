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
abstract AbstractNormalEstimator <: AbstractFeatureEstimator
abstract AbstractSHOTEstimator <: AbstractFeatureEstimator

@inline handle(fe::AbstractFeatureEstimator) = fe.handle

setRadiusSearch(n::AbstractFeatureEstimator, rad) =
    icxx"$(handle(n))->setRadiusSearch($rad);"

setInputCloud(n::AbstractFeatureEstimator, cloud::PointCloud) =
    icxx"$(handle(n))->setInputCloud($(handle(cloud)));"

setInputNormals(n::AbstractFeatureEstimator, normals::PointCloud) =
    icxx"$(handle(n))->setInputNormals($(handle(normals)));"

setSearchSurface(n::AbstractFeatureEstimator, surface::PointCloud) =
    icxx"$(handle(n))->setSearchSurface($(handle(surface)));"

compute(n::AbstractFeatureEstimator, descriptors::PointCloud) =
    icxx"$(handle(n))->compute(*$(handle(descriptors)));"

for (name, type_params, supername) in [
    (:NormalEstimation, (:PT,:NT), AbstractNormalEstimator),
    (:NormalEstimationOMP, (:PT,:NT), AbstractNormalEstimator),
    (:SHOTEstimation, (:PT,:NT,:OT), AbstractSHOTEstimator),
    (:SHOTEstimationOMP, (:PT,:NT,:OT), AbstractSHOTEstimator),
    (:BOARDLocalReferenceFrameEstimation, (:T,:N,:F),  AbstractFeatureEstimator),
    ]
    cxxname = "pcl::$name"
    name_with_params = Expr(:curly, name, type_params...)
    valname_with_params = Expr(:curly, symbol(name, "Val"), type_params...)
    @eval begin
        @defpcltype $name_with_params <: $supername $cxxname
        @defptrconstructor $name_with_params() $cxxname
        @defconstructor $valname_with_params() $cxxname
    end
end

setKSearch(n::AbstractNormalEstimator, k) = icxx"$(handle(n))->setKSearch($k);"

setFindHoles(rfe::BOARDLocalReferenceFrameEstimation, v::Bool) =
    icxx"$(handle(rfe))->setFindHoles($v);"
