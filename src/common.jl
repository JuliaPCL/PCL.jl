### common ###

import Base: call, eltype


# TODO: wrap it into jl type
const PointXYZ = cxxt"pcl::PointXYZ"
const PointXYZI = cxxt"pcl::PointXYZI"

type PointCloud{T}
    handle::cxxt"boost::shared_ptr<pcl::PointCloud<$T>>"
end

function call{T}(::Type{PointCloud{T}})
    handle = icxx"boost::shared_ptr<pcl::PointCloud<$T>>(new pcl::PointCloud<$T>);"
    PointCloud(handle)
end
