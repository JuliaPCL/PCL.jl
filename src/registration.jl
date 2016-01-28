abstract AbstractRegistration

@defpcltype IterativeClosestPoint{T1,T2} "pcl::IterativeClosestPoint"
@defptrconstructor IterativeClosestPoint{T1,T2}() "pcl::IterativeClosestPoint"
@defconstructor IterativeClosestPointVal{T1,T2}() "pcl::IterativeClosestPoint"

for f in [:setMaximumIterations, :setMaxCorrespondenceDistance]
    @eval begin
        $f(icp::IterativeClosestPoint, s) = @cxx cxxpointer(handle(icp))->$f(s)
    end
end

for f in [:setInputTarget, :setInputSource]
    body1 = Expr(:macrocall, symbol("@icxx_str"),
        "\$(handle(icp))->$f(\$(handle(cloud)));")
    @eval $f(icp::IterativeClosestPoint, cloud::PointCloud) = $body1

    body2 = Expr(:macrocall, symbol("@icxx_str"),
        "\$(handle(icp))->$f(\$(cloud));")
    @eval $f(icp::IterativeClosestPoint, cloud) = $body2
end

hasConverged(icp::IterativeClosestPoint) =
    @cxx cxxpointer(handle(icp))->hasConverged()
align(icp::IterativeClosestPoint, cloud::PointCloud) =
    icxx"$(handle(icp))->align(*$(handle(cloud)));"
