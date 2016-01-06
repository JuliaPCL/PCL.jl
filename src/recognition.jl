typealias CxxGeometricConsistencyGrouping{MT,ST} cxxt"boost::shared_ptr<pcl::GeometricConsistencyGrouping<$MT,$ST>>"

type GeometricConsistencyGrouping{MT,ST}
    handle::CxxGeometricConsistencyGrouping # TODO typed
end

function call{MT,ST}(::Type{GeometricConsistencyGrouping{MT,ST}})
    handle = icxx"""
        boost::shared_ptr<pcl::GeometricConsistencyGrouping<$MT,$ST>>(
        new pcl::GeometricConsistencyGrouping<$MT,$ST>);"""
    GeometricConsistencyGrouping{MT,ST}(handle)
end

setGCSize(g::GeometricConsistencyGrouping, s) =
    icxx"$(g.handle).get()->setGCSize($s);"

setGCThreshold(g::GeometricConsistencyGrouping, t) =
    icxx"$(g.handle).get()->setGCThreshold($t);"

setInputCloud(g::GeometricConsistencyGrouping, cloud::PointCloud) =
    icxx"$(g.handle).get()->setInputCloud($(cloud.handle));"

setSceneCloud(g::GeometricConsistencyGrouping, cloud::PointCloud) =
    icxx"$(g.handle).get()->setSceneCloud($(cloud.handle));"

setModelSceneCorrespondences(g::GeometricConsistencyGrouping, corr::Correspondences) =
    icxx"$(g.handle).get()->setModelSceneCorrespondences($(corr.handle));"

recognize(g::GeometricConsistencyGrouping, rototranslations, clustered_corrs) =
    icxx"$(g.handle).get()->recognize($rototranslations, $clustered_corrs);"
