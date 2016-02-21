### recognition ###

VERBOSE && info("Include pcl::recognition headers")
@timevb cxx"""
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
"""

abstract AbstractRecognizer
abstract AbstractVerifier

for f in [:setInputCloud, :setInputRf, :setSceneCloud, :setSceneRf]
    body = Expr(:macrocall, symbol("@icxx_str"),
        "\$(handle(recognizer))->$f(\$(handle(cloud)));")
    @eval $f(recognizer::AbstractRecognizer, cloud::PointCloud) = $body
end

function setModelSceneCorrespondences(recognizer::AbstractRecognizer,
    corr::Correspondences)
    icxx"$(handle(recognizer))->setModelSceneCorrespondences(
        $(handle(corr)));"
end

recognize(recognizer::AbstractRecognizer, rototranslations, clustered_corrs) =
    icxx"$(handle(recognizer))->recognize($rototranslations, $clustered_corrs);"

for (name, type_params, supername) in [
    (:GeometricConsistencyGrouping, (:MT,:ST), AbstractRecognizer),
    (:Hough3DGrouping, (:T1,:T2,:R1,:R2), AbstractRecognizer),
    (:GlobalHypothesesVerification, (:MT,:ST), AbstractVerifier),
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

for f in [:setGCSize, :setGCThreshold]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(g))->$f(\$s);")
    @eval $f(g::GeometricConsistencyGrouping, s) = $body
end

for f in [
        :setHoughBinSize,
        :setHoughThreshold,
        :setUseInterpolation,
        :setUseDistanceWeight,
        ]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(h))->$f(\$s);")
    @eval $f(h::Hough3DGrouping, s) = $body
end

for f in [:setSceneCloud, :setOcclusionCloud]
    body = Expr(:macrocall, symbol("@icxx_str"),
        "\$(handle(ver))->$f(\$(handle(cloud)));")
    @eval $f(ver::GlobalHypothesesVerification, cloud::PointCloud) = $body
end

function addModels(ver::GlobalHypothesesVerification, models::CxxStd.StdVector,
    occlusion_reasoning=false)
    icxx"$(handle(ver))->addModels($models, $occlusion_reasoning);"
end

for f in [
        :setInlierThreshold,
        :setOcclusionThreshold,
        :setRegularizer,
        :setRadiusClutter,
        :setClutterRegularizer,
        :setRadiusNormals,
        :setDetectClutter,
        ]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(ver))->$f(\$v);")
    @eval $f(ver::GlobalHypothesesVerification, v) = $body
end

verify(ver::GlobalHypothesesVerification) = icxx"$(handle(ver))->verify();"
getMask(ver::GlobalHypothesesVerification, mask::CxxStd.StdVector) =
    icxx"$(handle(ver))->getMask($mask);"
