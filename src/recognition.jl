"""
Abstract recognizers that implemnts the following methods:

- setInputCloud
- setInputRf
- setSceneCloud
- setSceneRf
- setModelSceneCorrespondences
- recognize
"""
abstract AbstractRecognizer

@inline handle(r::AbstractRecognizer) = r.handle

for f in [:setInputCloud, :setInputRf, :setSceneCloud, :setSceneRf]
    @eval begin
        $f(recognizer::AbstractRecognizer, cloud::PointCloud) =
            @cxx cxxpointer(handle(recognizer))->$f(handle(cloud))
    end
end

function setModelSceneCorrespondences(recognizer::AbstractRecognizer,
    corr::Correspondences)
    @cxx cxxpointer(handle(recognizer))->setModelSceneCorrespondences(handle(corr))
end

recognize(recognizer::AbstractRecognizer, rototranslations, clustered_corrs) =
    @cxx cxxpointer(handle(recognizer))->recognize(rototranslations, clustered_corrs)


type GeometricConsistencyGrouping{MT,ST} <: AbstractRecognizer
    handle::cxxt"boost::shared_ptr<pcl::GeometricConsistencyGrouping<$MT,$ST>>"
end
@defconstructor GeometricConsistencyGrouping{MT,ST}()

for f in [:setGCSize, :setGCThreshold]
    @eval begin
        $f(g::GeometricConsistencyGrouping, s) = @cxx cxxpointer(handle(g))->$f(s)
    end
end

type Hough3DGrouping{T1,T2,R1,R2} <: AbstractRecognizer
    handle::cxxt"boost::shared_ptr<pcl::Hough3DGrouping<$T1,$T2,$R1,$R2>>"
end
@defconstructor Hough3DGrouping{T1,T2,R1,R2}()

for f in [:setHoughBinSize, :setHoughThreshold]
    @eval begin
        $f(h::Hough3DGrouping, s) = @cxx cxxpointer(handle(h))->$f(s)
    end
end

for f in [:setUseInterpolation, :setUseDistanceWeight]
    @eval begin
        $f(h::Hough3DGrouping, v::Bool) = @cxx cxxpointer(handle(h))->$f(v)
    end
end

abstract AbstractVerifier

@inline handle(v::AbstractVerifier) = v.handle

type GlobalHypothesesVerification{MT,ST} <: AbstractVerifier
    handle::cxxt"boost::shared_ptr<pcl::GlobalHypothesesVerification<$MT,$ST>>"
end
@defconstructor GlobalHypothesesVerification{MT,ST}()

for f in [:setSceneCloud, :setOcclusionCloud]
    @eval begin
        $f(ver::GlobalHypothesesVerification, cloud::PointCloud) =
            @cxx cxxpointer(handle(ver))->$f(handle(cloud))
    end
end

function addModels(ver::GlobalHypothesesVerification, models::CxxStd.StdVector,
    occlusion_reasoning=false)
    @cxx cxxpointer(handle(ver))->addModels(models, occlusion_reasoning)
end

for f in [:setInlierThreshold, :setOcclusionThreshold, :setRegularizer,
    :setRadiusClutter, :setClutterRegularizer, :setRadiusNormals]
    @eval begin
        $f(ver::GlobalHypothesesVerification, v::AbstractFloat) =
            @cxx cxxpointer(handle(ver))->$f(v)
    end
end

for f in [:setDetectClutter]
    @eval begin
        $f(ver::GlobalHypothesesVerification, v::Bool) =
            @cxx cxxpointer(handle(ver))->$f(v)
    end
end

verify(ver::GlobalHypothesesVerification) = @cxx cxxpointer(handle(ver))->verify()
getMask(ver::GlobalHypothesesVerification, mask::CxxStd.StdVector) =
    @cxx cxxpointer(handle(ver))->getMask(mask)
