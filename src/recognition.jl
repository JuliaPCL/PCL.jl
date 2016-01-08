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

handle(r::AbstractRecognizer) = r.handle

setInputCloud(recognizer::AbstractRecognizer, cloud::PointCloud) =
    icxx"$(handle(recognizer)).get()->setInputCloud($(handle(cloud)));"

setInputRf(recognizer::AbstractRecognizer, cloud::PointCloud) =
    icxx"$(handle(recognizer)).get()->setInputRf($(handle(cloud)));"

setSceneCloud(recognizer::AbstractRecognizer, cloud::PointCloud) =
    icxx"$(handle(recognizer)).get()->setSceneCloud($(handle(cloud)));"

setSceneRf(recognizer::AbstractRecognizer, cloud::PointCloud) =
    icxx"$(handle(recognizer)).get()->setSceneRf($(handle(cloud)));"

setModelSceneCorrespondences(recognizer::AbstractRecognizer, corr::Correspondences) =
    icxx"$(handle(recognizer)).get()->setModelSceneCorrespondences($(corr.handle));"

recognize(recognizer::AbstractRecognizer, rototranslations, clustered_corrs) =
    icxx"$(handle(recognizer)).get()->recognize($rototranslations, $clustered_corrs);"


type GeometricConsistencyGrouping{MT,ST} <: AbstractRecognizer
    handle::SharedPtr # TODO: typed
end

function call{MT,ST}(::Type{GeometricConsistencyGrouping{MT,ST}})
    handle = icxx"""
        boost::shared_ptr<pcl::GeometricConsistencyGrouping<$MT,$ST>>(
        new pcl::GeometricConsistencyGrouping<$MT,$ST>);"""
    GeometricConsistencyGrouping{MT,ST}(handle)
end

setGCSize(g::GeometricConsistencyGrouping, s) =
    icxx"$(handle(g)).get()->setGCSize($s);"
setGCThreshold(g::GeometricConsistencyGrouping, t) =
    icxx"$(handle(g)).get()->setGCThreshold($t);"


type Hough3DGrouping{T1,T2,R1,R2} <: AbstractRecognizer
    handle::SharedPtr # TODO: typed
end

function call{T1,T2,R1,R2}(::Type{Hough3DGrouping{T1,T2,R1,R2}})
    handle = icxx"""
        boost::shared_ptr<pcl::Hough3DGrouping<$T1,$T2,$R1,$R2>>(
        new pcl::Hough3DGrouping<$T1,$T2,$R1,$R2>);"""
    Hough3DGrouping{T1,T2,R1,R2}(handle)
end

setHoughBinSize(h::Hough3DGrouping, s) =
    icxx"$(handle(h)).get()->setHoughBinSize($s);"
setHoughThreshold(h::Hough3DGrouping, s) =
    icxx"$(handle(h)).get()->setHoughThreshold($s);"
setUseInterpolation(h::Hough3DGrouping, s::Bool) =
    icxx"$(handle(h)).get()->setUseInterpolation($s);"
setUseDistanceWeight(h::Hough3DGrouping, s::Bool) =
    icxx"$(handle(h)).get()->setUseDistanceWeight($s);"
