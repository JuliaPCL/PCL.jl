import Base: call

typealias CxxNormalEstimationOMP{PT,NT} cxxt"boost::shared_ptr<pcl::NormalEstimationOMP<$PT,$NT>>"

type NormalEstimationOMP{PT,NT}
    handle::CxxNormalEstimationOMP # TODO: typed
end

function call{PT,NT}(::Type{NormalEstimationOMP{PT,NT}})
    handle = icxx"""
        boost::shared_ptr<pcl::NormalEstimationOMP<$PT,$NT>>(
        new pcl::NormalEstimationOMP<$PT,$NT>);"""
    NormalEstimationOMP{PT,NT}(handle)
end

setKSearch(n::NormalEstimationOMP, k) = icxx"$(n.handle).get()->setKSearch($k);"

setInputCloud(n::NormalEstimationOMP, cloud::PointCloud) =
    icxx"$(n.handle).get()->setInputCloud($(cloud.handle));"

compute(n::NormalEstimationOMP, normals::PointCloud) =
    icxx"$(n.handle).get()->compute(*$(normals.handle));"


typealias CxxSHOTEstimationOMP{PT,NT,OT} cxxt"boost::shared_ptr<pcl::SHOTEstimationOMP<$PT,$NT,$OT>>"

type SHOTEstimationOMP{PT,NT,OT}
    handle::CxxSHOTEstimationOMP # TODO: typed
end

function call{PT,NT,OT}(::Type{SHOTEstimationOMP{PT,NT,OT}})
    handle = icxx"""
        boost::shared_ptr<pcl::SHOTEstimationOMP<$PT,$NT,$OT>>(
        new pcl::SHOTEstimationOMP<$PT,$NT,$OT>);"""
    SHOTEstimationOMP{PT,NT,OT}(handle)
end

setRadiusSearch(n::SHOTEstimationOMP, rad) =
    icxx"$(n.handle).get()->setRadiusSearch($rad);"

setInputCloud(n::SHOTEstimationOMP, cloud::PointCloud) =
    icxx"$(n.handle).get()->setInputCloud($(cloud.handle));"

setInputNormals(n::SHOTEstimationOMP, normals::PointCloud) =
    icxx"$(n.handle).get()->setInputNormals($(normals.handle));"

setSearchSurface(n::SHOTEstimationOMP, surface::PointCloud) =
    icxx"$(n.handle).get()->setSearchSurface($(surface.handle));"

compute(n::SHOTEstimationOMP, descriptors::PointCloud) =
    icxx"$(n.handle).get()->compute(*$(descriptors.handle));"
