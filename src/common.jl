### common ###

import Base: call, eltype, length, size, getindex, setindex!, push!

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

import Base: show

function show(io::IO, p::PointXYZRGBValOrRef)
    x = icxx"$p.x;"
    y = icxx"$p.y;"
    z = icxx"$p.z;"
    r = icxx"$p.r;"
    g = icxx"$p.g;"
    b = icxx"$p.b;"
    print(io, string(typeof(p)));
    print(io, "\n");
    print(io, "(x,y,z,r,g,b): ");
    print(io, (x,y,z,r,g,b))
end

function show(io::IO, p::PointXYZValOrRef)
    x = icxx"$p.x;"
    y = icxx"$p.y;"
    z = icxx"$p.z;"
    print(io, string(typeof(p)));
    print(io, "\n");
    print(io, "(x,y,z): ");
    print(io, (x,y,z))
end

@defpcltype PointCloud{T} "pcl::PointCloud"
@defptrconstructor PointCloud{T}() "pcl::PointCloud"
@defptrconstructor PointCloud{T}(w::Integer, h::Integer) "pcl::PointCloud"
@defconstructor PointCloudVal{T}() "pcl::PointCloud"
@defconstructor PointCloudVal{T}(w::Integer, h::Integer) "pcl::PointCloud"

eltype{T}(cloud::PointCloud{T}) = T
eltype{T}(cloud::PointCloudVal{T}) = T

import Base: similar

similar{T}(cloud::PointCloud{T}) = PointCloud{T}(width(cloud), height(cloud))

import Base: copy, deepcopy

function deepcopy{T}(cloud::PointCloud{T})
    cloud_out = PointCloud{T}()
    @cxx pcl::copyPointCloud(cxxderef(handle(cloud)),
        cxxderef(handle(cloud_out)))
    cloud_out
end

copy(cloud::PointCloud) = cloud

getindex(cloud::PointCloud, i::Integer) = icxx"$(handle(cloud))->at($i);"

function getindex(cloud::PointCloud, i::Integer, name::Symbol)
    p = icxx"&$(handle(cloud))->points[$i];"
    @eval @cxx $p->$name
end
function setindex!(cloud::PointCloud, v, i::Integer, name::Symbol)
    p = icxx"&$(handle(cloud))->points[$i];"
    vp = @eval @cxx &($p->$name)
    unsafe_store!(vp, v, 1)
end

"""Create PointCloud instance and then load PCD data."""
function call{T}(::Type{PointCloud{T}}, pcd_file::AbstractString)
    handle = @sharedptr "pcl::PointCloud<\$T>"
    cloud = PointCloud(handle)
    pcl.loadPCDFile(pcd_file, cloud)
    return cloud
end

length(cloud::PointCloud) = convert(Int, icxx"$(handle(cloud))->size();")
width(cloud::PointCloud) = convert(Int, icxx"$(handle(cloud))->width;")
height(cloud::PointCloud) = convert(Int, icxx"$(handle(cloud))->height;")
is_dense(cloud::PointCloud) = icxx"$(handle(cloud))->is_dense;"
points(cloud::PointCloud) = icxx"$(handle(cloud))->points;"

length(cloud::PointCloudVal) = convert(Int, icxx"$(handle(cloud)).size();")
width(cloud::PointCloudVal) = convert(Int, icxx"$(handle(cloud)).width;")
height(cloud::PointCloudVal) = convert(Int, icxx"$(handle(cloud)).height;")
is_dense(cloud::PointCloudVal) = icxx"$(handle(cloud)).is_dense;"
points(cloud::PointCloudVal) = icxx"$(handle(cloud)).points;"

function transformPointCloud(cloud_in::PointCloud, cloud_out::PointCloud,
    transform)
    @cxx pcl::transformPointCloud(cxxderef(handle(cloud_in)),
        cxxderef(handle(cloud_out)), transform)
end

function removeNaNFromPointCloud(cloud_in::PointCloud,
    indices::CxxStd.StdVector{Cint})
    @cxx pcl::removeNaNFromPointCloud(cxxderef(handle(cloud_in)), indices)
end

function removeNaNFromPointCloud(cloud_in::PointCloud, cloud_out::PointCloud,
    indices::CxxStd.StdVector{Cint})
    @cxx pcl::removeNaNFromPointCloud(cxxderef(handle(cloud_in)),
        cxxderef(handle(cloud_out)), indices)
end

@defpcltype PCLPointCloud2 "pcl::PCLPointCloud2"
@defptrconstructor PCLPointCloud2() "pcl::PCLPointCloud2"
@defconstructor PCLPointCloud2Val() "pcl::PCLPointCloud2"

@defpcltype Correspondence "pcl::Correspondence"
@defconstructor CorrespondenceVal() "pcl::Correspondence"
@defconstructor(CorrespondenceVal(index_query, index_match, distance),
    "pcl::Correspondence")

@defpcltype Correspondences "pcl::Correspondences"
@defptrconstructor Correspondences() "pcl::Correspondences"

length(cs::Correspondences) = convert(Int, @cxx cxxpointer(handle(cs))->size())
push!(cs::Correspondences, c::CorrespondenceVal) =
    @cxx cxxpointer(handle(cs))->push_back(handle(c))

@defpcltype ModelCoefficients "pcl::ModelCoefficients"
@defptrconstructor ModelCoefficients() "pcl::ModelCoefficients"
@defconstructor ModelCoefficientsVal() "pcl::ModelCoefficients"

length(coef::ModelCoefficients) =
    convert(Int, icxx"$(handle(coef))->values.size();")

@defpcltype PointIndices "pcl::PointIndices"
@defptrconstructor PointIndices() "pcl::PointIndices"
@defconstructor PointIndicesVal() "pcl::PointIndices"

length(indices::PointIndices) =
    convert(Int, icxx"$(handle(indices))->indices.size();")
