### tracking ###

VERBOSE && info("Include pcl::tracking headers")
@timevb cxx"""
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/normal_coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/tracking.h>
"""

abstract AbstractTracker
abstract AbstractCloudCoherence
abstract AbstractCoherence

abstract AbstractKLDAdaptiveParticleFilterTracker <: AbstractTracker

compute(t::AbstractTracker) = icxx"$(handle(t))->compute();"

for name in [
    :ParticleXYZRPY,
    :ParticleXYZR,
    :ParticleXYRPY,
    :ParticleXYRP,
    :ParticleXYR,
    ]
    refname = symbol(name, :Ref)
    valorref = symbol(name, :ValOrRef)
    cppname = string("pcl::tracking::", name)
    cxxtdef = Expr(:macrocall, symbol("@cxxt_str"), cppname);
    rcppdef = Expr(:macrocall, symbol("@rcpp_str"), cppname);

    @eval begin
        global const $name = $cxxtdef
        global const $refname = $rcppdef
        global const $valorref = Union{$name, $refname}
    end

    # no args constructor
    body = Expr(:macrocall, symbol("@icxx_str"), string(cppname, "();"))
    @eval (::Type{$name})() = $body
end

for (name, type_params, supername) in [
    (:KLDAdaptiveParticleFilterTracker, (:RT,:PT), AbstractKLDAdaptiveParticleFilterTracker),
    (:KLDAdaptiveParticleFilterOMPTracker, (:RT,:PT), AbstractKLDAdaptiveParticleFilterTracker),
    (:ApproxNearestPairPointCloudCoherence, (:T,), AbstractCloudCoherence),
    (:NearestPairPointCloudCoherence, (:T,), AbstractCloudCoherence),
    (:DistanceCoherence, (:T,), AbstractCoherence),
    (:HSVColorCoherence, (:T,), AbstractCoherence),
    (:NormalCoherence, (:T,), AbstractCoherence),
    ]
    cxxname = "pcl::tracking::$name"
    name_with_params = Expr(:curly, name, type_params...)
    valname_with_params = Expr(:curly, symbol(name, "Val"), type_params...)
    @eval begin
        @defpcltype $name_with_params <: $supername $cxxname
        @defptrconstructor $name_with_params() $cxxname
        @defconstructor $valname_with_params() $cxxname
    end
end

@defptrconstructor(KLDAdaptiveParticleFilterTracker{RT,PT}(n::Integer),
    "pcl::tracking::KLDAdaptiveParticleFilterOMPTracker")
@defptrconstructor(KLDAdaptiveParticleFilterOMPTracker{RT,PT}(n::Integer),
    "pcl::tracking::KLDAdaptiveParticleFilterOMPTracker")

for f in [
        :setInputCloud,
        :setReferenceCloud,
        :setTrans,
        :setStepNoiseCovariance,
        :setInitialNoiseCovariance,
        :setInitialNoiseMean,
        :setIterationNum,
        :setParticleNum,
        :setResampleLikelihoodThr,
        :setUseNormal,
        :setCloudCoherence,
        :setMaximumParticleNum,
        :setDelta,
        :setEpsilon,
        :setBinSize,
        :toEigenMatrix,
        ]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(t))->$f(\$v);")
    @eval $f(t::AbstractKLDAdaptiveParticleFilterTracker, v) = $body
end

for f in [
        :getResult,
        ]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(t))->$f();")
    @eval $f(t::AbstractKLDAdaptiveParticleFilterTracker) = $body
end

function getReferenceCloud(t::AbstractKLDAdaptiveParticleFilterTracker)
    cloud = icxx"$(handle(t))->getReferenceCloud();"
    PointCloud(cloud)
end

function setInputCloud(t::AbstractKLDAdaptiveParticleFilterTracker,
        cloud::PointCloud)
    setInputCloud(t, handle(cloud))
end

function setReferenceCloud(t::AbstractKLDAdaptiveParticleFilterTracker,
        cloud::PointCloud)
    setReferenceCloud(t, handle(cloud))
end

function setCloudCoherence(t::AbstractKLDAdaptiveParticleFilterTracker,
        c::ApproxNearestPairPointCloudCoherence)
    setCloudCoherence(t, handle(c))
end

for f in [
        :addPointCoherence,
        :setSearchMethod,
        :setMaximumDistance,
        ]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(c))->$f(\$v);")
    @eval $f(c::AbstractCloudCoherence, v) = $body
end

function addPointCoherence(c::AbstractCloudCoherence,
        coherence::AbstractCoherence)
    addPointCoherence(c, handle(coherence))
end

setSearchMethod(c::AbstractCloudCoherence, t::AbstractOctree) =
    setSearchMethod(c, handle(t))

for f in [
        :setWeight,
        ]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(t))->$f(\$v);")
    @eval $f(t::AbstractCoherence, v) = $body
end
