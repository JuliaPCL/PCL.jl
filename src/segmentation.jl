### segmentation ###

VERBOSE && info("Include pcl::segmentation headers")
@timevb cxx"""
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
"""

abstract AbstractSegmentation

setInputCloud(s::AbstractSegmentation, cloud::PointCloud) =
    icxx"$(handle(s))->setInputCloud($(handle(cloud)));"
setIndices(s::AbstractSegmentation, indices::SharedPtr) =
    icxx"$(handle(s))->setIndices($indices);"
function segment(s::AbstractSegmentation, inliers::PointIndices,
        coefficients::ModelCoefficients)
    icxx"$(handle(s))->segment(*$(handle(inliers)), *$(handle(coefficients)));"
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
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(s))->$f(\$arg);")
    @eval $f(s::SACSegmentation, arg) = $body
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
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(s))->$f(\$arg);")
    @eval $f(s::RegionGrowingRGB, arg) = $body
end

setSearchMethod(s::RegionGrowingRGB, tree::KdTree) =
    setSearchMethod(s, handle(tree))

extract(s::RegionGrowingRGB, clusters::CxxStd.StdVector) =
    icxx"$(handle(s))->extract($clusters);"

function getColoredCloud(s::RegionGrowingRGB)
    c = icxx"$(handle(s))->getColoredCloud();"
    PointCloud(c)
end
