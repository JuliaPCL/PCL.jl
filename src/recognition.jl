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
    handle::SharedPtr # TODO: typed
end

function call{MT,ST}(::Type{GeometricConsistencyGrouping{MT,ST}})
    handle = icxx"""
        boost::shared_ptr<pcl::GeometricConsistencyGrouping<$MT,$ST>>(
        new pcl::GeometricConsistencyGrouping<$MT,$ST>);"""
    GeometricConsistencyGrouping{MT,ST}(handle)
end

for f in [:setGCSize, :setGCThreshold]
    @eval begin
        $f(g::GeometricConsistencyGrouping, s) = @cxx cxxpointer(handle(g))->$f(s)
    end
end

type Hough3DGrouping{T1,T2,R1,R2} <: AbstractRecognizer
    handle::SharedPtr # TODO: typed
end

function call{T1,T2,R1,R2}(::Type{Hough3DGrouping{T1,T2,R1,R2}})
    handle = icxx"""
        boost::shared_ptr<pcl::Hough3DGrouping<$T1,$T2,$R1,$R2>>(
        new pcl::Hough3DGrouping<$T1,$T2,$R1,$R2>);"""
    Hough3DGrouping{T1,T2,R1,R2}(handle)
end

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
