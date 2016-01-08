
abstract AbstractRegistration

@inline handle(r::AbstractRegistration) = r.handle

type IterativeClosestPoint{T1,T2} <: AbstractRegistration
    handle::SharedPtr
end

function call{T1,T2}(::Type{IterativeClosestPoint{T1,T2}})
    handle = icxx"""
        boost::shared_ptr<pcl::IterativeClosestPoint<$T1,$T2>>(
        new pcl::IterativeClosestPoint<$T1,$T2>);"""
    IterativeClosestPoint{T1,T2}(handle)
end

for f in [:setMaximumIterations, :setMaxCorrespondenceDistance]
    @eval begin
        $f(icp::IterativeClosestPoint, s) = @cxx cxxpointer(handle(icp))->$f(s)
    end
end

for f in [:setInputTarget, :setInputSource]
    @eval begin
        $f(icp::IterativeClosestPoint, cloud::PointCloud) =
            @cxx cxxpointer(handle(icp))->$f(handle(cloud))
    end
    @eval begin
        $f(icp::IterativeClosestPoint, cloud) =
            @cxx cxxpointer(handle(icp))->$f(cloud)
    end
end

hasConverged(icp::IterativeClosestPoint) =
    @cxx cxxpointer(handle(icp))->hasConverged()
align(icp::IterativeClosestPoint, cloud::PointCloud) =
    @cxx cxxpointer(handle(icp))->align(cxxderef(handle(cloud)))
