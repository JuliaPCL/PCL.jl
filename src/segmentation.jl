abstract AbstractSegmentation

setInputCloud(s::AbstractSegmentation, cloud::PointCloud) =
    @cxx cxxpointer(handle(s))->setInputCloud(handle(cloud))
function segment(s::AbstractSegmentation, inliers::PointIndices,
        coefficients::ModelCoefficients)
    @cxx cxxpointer(handle(s))->segment(
        cxxderef(handle(inliers)), cxxderef(handle(coefficients)))
end

for (name, supername) in [
    (:SACSegmentation, AbstractSegmentation),
    ]
    cxxname = "pcl::$name"
    valname = symbol(name, "Val")
    @eval begin
        @defpcltype $name{T} <: $supername $cxxname
        @defptrconstructor $name{T}() $cxxname
        @defconstructor $valname{T}() $cxxname
    end
end

for f in [
    :setOptimizeCoefficients,
    :setModelType,
    :setMethodType,
    :setMaxIterations,
    :setDistanceThreshold,
    ]
    @eval $f(s::SACSegmentation, arg) = @cxx cxxpointer(handle(s))->$f(arg)
end
