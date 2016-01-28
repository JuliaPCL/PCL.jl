abstract AbstractTracker
abstract AbstractCloudCoherence
abstract AbstractCoherence

abstract AbstractKLDAdaptiveParticleFilterTracker <: AbstractTracker

compute(t::AbstractTracker) = @cxx cxxpointer(handle(t))->compute()

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
    @eval call(::Type{$name}) = $body
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
    @eval begin
        $f(t::AbstractKLDAdaptiveParticleFilterTracker, v) =
            @cxx cxxpointer(handle(t))->$f(v)
    end
end

for f in [
        :getResult,
        ]
    @eval begin
        $f(t::AbstractKLDAdaptiveParticleFilterTracker) =
            @cxx cxxpointer(handle(t))->$f()
    end
end

function getReferenceCloud(t::AbstractKLDAdaptiveParticleFilterTracker)
    PointCloud(@cxx cxxpointer(handle(t))->getReferenceCloud())
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
    @eval $f(c::AbstractCloudCoherence, v) = @cxx cxxpointer(handle(c))->$f(v)
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
    @eval $f(t::AbstractCoherence, v) = @cxx cxxpointer(handle(t))->$f(v)
end
