abstract AbstractRegistration

@defpcltype IterativeClosestPoint{T1,T2} "pcl::IterativeClosestPoint"
@defptrconstructor IterativeClosestPoint{T1,T2}() "pcl::IterativeClosestPoint"

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
