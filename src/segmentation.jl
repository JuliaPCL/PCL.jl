abstract AbstractSegmentation

setInputCloud(s::AbstractSegmentation, cloud::PointCloud) =
    @cxx cxxpointer(handle(s))->setInputCloud(handle(cloud))
setIndices(s::AbstractSegmentation, indices::SharedPtr) =
    @cxx cxxpointer(handle(s))->setIndices(indices)
function segment(s::AbstractSegmentation, inliers::PointIndices,
        coefficients::ModelCoefficients)
    @cxx cxxpointer(handle(s))->segment(
        cxxderef(handle(inliers)), cxxderef(handle(coefficients)))
end

for (name, supername) in [
    (:SACSegmentation, AbstractSegmentation),
    (:RegionGrowingRGB, AbstractSegmentation),
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

for f in [
    :setSearchMethod,
    :setDistanceThreshold,
    :setPointColorThreshold,
    :setRegionColorThreshold,
    :setMinClusterSize,
    :setMaxClusterSize,
    :setSmoothnessThreshold,
    :setCurvatureThreshold,
    ]
    @eval $f(s::RegionGrowingRGB, arg) = @cxx cxxpointer(handle(s))->$f(arg)
end

setSearchMethod(s::RegionGrowingRGB, tree::KdTree) =
    setSearchMethod(s, handle(tree))

extract(s::RegionGrowingRGB, clusters::CxxStd.StdVector) =
    @cxx cxxpointer(handle(s))->extract(clusters)

getColoredCloud(s::RegionGrowingRGB) =
    PointCloud(@cxx cxxpointer(handle(s))->getColoredCloud())
