import Base: call, filter

abstract AbstractFilter

@inline handle(f::AbstractFilter) = f.handle

setInputCloud(f::AbstractFilter, cloud::PointCloud) =
    @cxx cxxpointer(handle(f))->setInputCloud(handle(cloud))
filter(f::AbstractFilter, cloud::PointCloud) =
    @cxx cxxpointer(handle(f))->filter(cxxderef(handle(cloud)))

type UniformSampling{T} <: AbstractFilter
    handle::cxxt"boost::shared_ptr<pcl::UniformSampling<$T>>"
end
@defconstructor UniformSampling{T}()

setRadiusSearch(us::UniformSampling, ss) =
    @cxx cxxpointer(handle(us))->setRadiusSearch(ss)

type PassThrough{T} <: AbstractFilter
    handle::cxxt"boost::shared_ptr<pcl::PassThrough<$T>>"
end
@defconstructor PassThrough{T}()

setFilterFieldName(pass::PassThrough, name::AbstractString) =
    @cxx cxxpointer(handle(pass))->setFilterFieldName(pointer(name))
setFilterLimits(pass::PassThrough, lo, hi) =
    @cxx cxxpointer(handle(pass))->setFilterLimits(lo, hi)
setFilterLimitsNegative(pass::PassThrough, v::Bool) =
    @cxx cxxpointer(handle(pass))->setFilterLimitsNegative(v)

abstract AbstractVoxelGridFilter <: AbstractFilter

setLeafSize(v::AbstractVoxelGridFilter, lx, ly, lz) =
    @cxx cxxpointer(handle(v))->setLeafSize(lx, ly, lz)

type VoxelGrid{T} <: AbstractVoxelGridFilter
    handle::cxxt"boost::shared_ptr<pcl::VoxelGrid<$T>>"
end
@defconstructor VoxelGrid{T}()

type ApproximateVoxelGrid{T} <: AbstractVoxelGridFilter
    handle::cxxt"boost::shared_ptr<pcl::ApproximateVoxelGrid<$T>>"
end
@defconstructor ApproximateVoxelGrid{T}()


type StatisticalOutlierRemoval{T} <: AbstractFilter
    handle::cxxt"boost::shared_ptr<pcl::StatisticalOutlierRemoval<$T>>"
end
@defconstructor StatisticalOutlierRemoval{T}()


for f in [:setMeanK, :setStddevMulThresh]
    @eval begin
        $f(s::StatisticalOutlierRemoval, v) = @cxx cxxpointer(handle(s))->$f(v)
    end
end

type RadiusOutlierRemoval{T} <:  AbstractFilter
    handle::cxxt"boost::shared_ptr<pcl::RadiusOutlierRemoval<$T>>"
end
@defconstructor RadiusOutlierRemoval{T}()


for f in [:setRadiusSearch, :setMinNeighborsInRadius]
    @eval begin
        $f(r::RadiusOutlierRemoval, v) = @cxx cxxpointer(handle(r))->$f(v)
    end
end
