### common ###

import Base: call, eltype

### PointType definitions ###

# TODO: wrap it into jl type
const PointXYZ = cxxt"pcl::PointXYZ"
const PointXYZI = cxxt"pcl::PointXYZI"
const PointXYZRGBA = cxxt"pcl::PointXYZRGBA"
const PointXYZRGB = cxxt"pcl::PointXYZRGB"
const PointXY = cxxt"pcl::PointXY"
const PointUV = cxxt"pcl::PointUV"
const InterestPoint = cxxt"pcl::InterestPoint"
const Normal = cxxt"pcl::Normal"
const Axis = cxxt"pcl::Axis"
const PointNormal = cxxt"pcl::PointNormal"
const PointXYZRGBNormal = cxxt"pcl::PointXYZRGBNormal"
const PointXYZINormal = cxxt"pcl::PointXYZINormal"
const PointXYZLNormal = cxxt"pcl::PointXYZLNormal"

type PointCloud{T}
    handle::cxxt"boost::shared_ptr<pcl::PointCloud<$T>>"
end

"""Create empty PointCloud instance"""
function call{T}(::Type{PointCloud{T}})
    handle = icxx"boost::shared_ptr<pcl::PointCloud<$T>>(new pcl::PointCloud<$T>);"
    PointCloud(handle)
end

"""Create PointCloud instance and then load PCD data."""
function call{T}(::Type{PointCloud{T}}, pcd_file::AbstractString)
    handle = icxx"boost::shared_ptr<pcl::PointCloud<$T>>(new pcl::PointCloud<$T>);"
    cloud = PointCloud(handle)
    pcl.loadPCDFile(pcd_file, cloud)
    return cloud
end
