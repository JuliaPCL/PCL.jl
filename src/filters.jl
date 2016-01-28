import Base: call, filter

abstract AbstractFilter
abstract AbstractVoxelGridFilter <: AbstractFilter

@inline handle(f::AbstractFilter) = f.handle

setInputCloud(f::AbstractFilter, cloud::PointCloud) =
    icxx"$(handle(f))->setInputCloud($(handle(cloud)));"
filter(f::AbstractFilter, cloud::PointCloud) =
    icxx"$(handle(f))->filter(*$(handle(cloud)));"
filter(f::AbstractFilter, cloud::SharedPtr) =
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
    @cxx cxxpointer(handle(us))->setRadiusSearch(ss)

setFilterFieldName(pass::PassThrough, name::AbstractString) =
    @cxx cxxpointer(handle(pass))->setFilterFieldName(pointer(name))
setFilterLimits(pass::PassThrough, lo, hi) =
    @cxx cxxpointer(handle(pass))->setFilterLimits(lo, hi)

for f in [:setKeepOrganized, :setFilterLimitsNegative]
    @eval begin
        $f(pass::PassThrough, v::Bool) = @cxx cxxpointer(handle(pass))->$f(v)
    end
end

setLeafSize(v::AbstractVoxelGridFilter, lx, ly, lz) =
    @cxx cxxpointer(handle(v))->setLeafSize(lx, ly, lz)


for f in [:setMeanK, :setStddevMulThresh]
    @eval begin
        $f(s::StatisticalOutlierRemoval, v) = @cxx cxxpointer(handle(s))->$f(v)
    end
end

for f in [:setRadiusSearch, :setMinNeighborsInRadius]
    @eval begin
        $f(r::RadiusOutlierRemoval, v) = @cxx cxxpointer(handle(r))->$f(v)
    end
end

for f in [:setNegative]
    @eval begin
        $f(ex::ExtractIndices, v::Bool) = @cxx cxxpointer(handle(ex))->$f(v)
    end
end

setIndices(e::ExtractIndices, indices::PointIndices) =
    icxx"$(handle(e))->setIndices($(handle(indices)));"
