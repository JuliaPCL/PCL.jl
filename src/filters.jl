import Base: call, filter

abstract AbstractFilter
abstract AbstractVoxelGridFilter <: AbstractFilter

setInputCloud(f::AbstractFilter, cloud::PointCloud) =
    icxx"$(handle(f))->setInputCloud($(handle(cloud)));"
filter(f::AbstractFilter, cloud::cxxt"boost::shared_ptr<std::vector<int>>") =
    icxx"$(handle(f))->filter(*$(cloud));"
filter(f::AbstractFilter, cloud::PointCloud) =
    icxx"$(handle(f))->filter(*$(handle(cloud)));"

for (name, supername) in [
    (:UniformSampling, AbstractFilter),
    (:PassThrough, AbstractFilter),
    (:VoxelGrid, AbstractVoxelGridFilter),
    (:ApproximateVoxelGrid, AbstractVoxelGridFilter),
    (:StatisticalOutlierRemoval, AbstractFilter),
    (:RadiusOutlierRemoval, AbstractFilter),
    (:ExtractIndices, AbstractFilter),
    ]
    cxxname = "pcl::$name"
    valname = symbol(name, "Val")
    @eval begin
        @defpcltype $name{T} <: $supername $cxxname
        @defptrconstructor $name{T}() $cxxname
        @defconstructor $valname{T}() $cxxname
    end
end

setRadiusSearch(us::UniformSampling, ss) =
    icxx"$(handle(us))->setRadiusSearch($ss);"

setFilterFieldName(pass::PassThrough, name::AbstractString) =
    icxx"$(handle(pass))->setFilterFieldName($(pointer(name)));"
setFilterLimits(pass::PassThrough, lo, hi) =
    icxx"$(handle(pass))->setFilterLimits($lo, $hi);"

for f in [:setKeepOrganized, :setFilterLimitsNegative]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(pass))->$f(\$v);")
    @eval $f(pass::PassThrough, v::Bool) = $body
end

setLeafSize(v::AbstractVoxelGridFilter, lx, ly, lz) =
    icxx"$(handle(v))->setLeafSize($lx, $ly, $lz);"


for f in [:setMeanK, :setStddevMulThresh]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(s))->$f(\$v);")
    @eval $f(s::StatisticalOutlierRemoval, v) = $body
end

for f in [:setRadiusSearch, :setMinNeighborsInRadius]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(r))->$f(\$v);")
    @eval $f(r::RadiusOutlierRemoval, v) = $body
end

for f in [:setNegative, :setIndices]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(ex))->$f(\$v);")
    @eval $f(ex::ExtractIndices, v) = $body
end

setIndices(ex::ExtractIndices, indices::PointIndices) =
    setIndices(ex, handle(indices))
