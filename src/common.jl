### common ###

import Base: call, eltype, length, size, getindex, push!

typealias SharedPtr{T} cxxt"boost::shared_ptr<$T>"

cxxpointer(p) = p
cxxpointer(p::SharedPtr) = @cxx p->get()
cxxderef(x) = icxx"*$x;"

### PointType definitions ###

for name in [
    :PointXYZ,
    :PointXYZI,
    :PointXYZRGBA,
    :PointXYZRGB,
    :PointXY,
    :PointUV,
    :InterestPoint,
    :Normal,
    :Axis,
    :PointNormal,
    :PointXYZRGBNormal,
    :PointXYZRGBNormal,
    :PointXYZINormal,
    :PointXYZLNormal,
    :ReferenceFrame,
    :SHOT352,
    ]
    refname = symbol(name, :Ref)
    valorref = symbol(name, :ValOrRef)
    cppname = string("pcl::", name)
    cxxtdef = Expr(:macrocall, symbol("@cxxt_str"), cppname);
    rcppdef = Expr(:macrocall, symbol("@rcpp_str"), cppname);

    @eval begin
        global const $name = $cxxtdef
        global const $refname = $rcppdef
        global const $valorref = Union{$name, $refname}
    end

    # no args constructor
    body = Expr(:macrocall, symbol("@icxx_str"), string(cppname, "();"))
    @eval call(::Type{$name}) = $body
end

call(::Type{PointXYZ}, x, y, z) = icxx"pcl::PointXYZ($x, $y, $z);"


type PointCloud{T}
    handle::cxxt"boost::shared_ptr<pcl::PointCloud<$T>>"
end

@inline handle(cloud::PointCloud) = cloud.handle

getindex(cloud::PointCloud, i::Integer) = icxx"$(handle(cloud)).get()->at($i);"

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

length(cloud::PointCloud) = convert(Int, icxx"$(handle(cloud))->size();")
width(cloud::PointCloud) = convert(Int, icxx"$(handle(cloud))->width;")
height(cloud::PointCloud) = convert(Int, icxx"$(handle(cloud))->height;")
is_dense(cloud::PointCloud) = icxx"$(handle(cloud))->is_dense;"
points(cloud::PointCloud) = icxx"$(handle(cloud))->points;"

function transformPointCloud(cloud_in::PointCloud, cloud_out::PointCloud,
    transform)
    icxx"""
        pcl::transformPointCloud(*$(cloud_in.handle), *$(cloud_out.handle),
            $transform);
    """
end

type Correspondence
    handle::cxxt"pcl::Correspondence"
end

call(::Type{Correspondence}) = Correspondence(
    icxx"pcl::Correspondence();")
function call(::Type{Correspondence}, index_query, index_match, distance)
    handle = icxx"pcl::Correspondence($index_query, $index_match, $distance);"
    Correspondence(handle)
end

type Correspondences
    handle::cxxt"boost::shared_ptr<std::vector<pcl::Correspondence,
        Eigen::aligned_allocator<pcl::Correspondence>>>"
end

@inline handle(c::Union{Correspondences,Correspondence}) = c.handle

function call(::Type{Correspondences})
    handle = icxx"""
        boost::shared_ptr<pcl::Correspondences>(
            new pcl::Correspondences());"""
    Correspondences(handle)
end

length(cs::Correspondences) = convert(Int, @cxx cxxpointer(handle(cs))->size())
push!(cs::Correspondences, c::Correspondence) =
    @cxx cxxpointer(handle(cs))->push_back(handle(c))
