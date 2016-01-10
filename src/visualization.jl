### visualization ###

abstract PointCloudColorHandler

@inline handle(p::PointCloudColorHandler) = p.handle

type PointCloudColorHandlerRGBField{T} <: PointCloudColorHandler
    handle::SharedPtr
end

function call{T}(::Type{PointCloudColorHandlerRGBField{T}}, cloud::PointCloud)
    handle = @sharedptr(
        "pcl::visualization::PointCloudColorHandlerRGBField<\$T>",
        "\$(cloud.handle)")
    PointCloudColorHandlerRGBField{T}(handle)
end

call{T}(::Type{PointCloudColorHandlerRGBField}, cloud::PointCloud{T}) =
    PointCloudColorHandlerRGBField{T}(cloud)


type PointCloudColorHandlerCustom{T} <: PointCloudColorHandler
    handle::SharedPtr
end

function call{T}(::Type{PointCloudColorHandlerCustom{T}}, cloud::PointCloud,
    r, g, b)
    handle = @sharedptr(
        "pcl::visualization::PointCloudColorHandlerCustom<\$T>",
        "\$(cloud.handle), \$r, \$g, \$b")
    PointCloudColorHandlerCustom{T}(handle)
end

call{T}(::Type{PointCloudColorHandlerCustom}, cloud::PointCloud{T}, r, g, b) =
    PointCloudColorHandlerCustom{T}(cloud, r, g, b)


const CxxPCLVisualizerPtr =
    cxxt"boost::shared_ptr<pcl::visualization::PCLVisualizer>"

type PCLVisualizer
    handle::CxxPCLVisualizerPtr
    PCLVisualizer(handle::CxxPCLVisualizerPtr) = new(handle)
end

@inline handle(viewer::PCLVisualizer) = viewer.handle

function PCLVisualizer(name::AbstractString="", create_interactor::Bool=true)
    handle = @sharedptr("pcl::visualization::PCLVisualizer",
        "\$(pointer(name)), \$create_interactor")
    PCLVisualizer(handle)
end

setBackgroundColor(viewer::PCLVisualizer, x, y, z) =
    @cxx cxxpointer(handle(viewer))->setBackgroundColor(x, y, z)
addCoordinateSystem(viewer::PCLVisualizer, n) =
    @cxx cxxpointer(handle(viewer))->addCoordinateSystem($n)
spinOnce(viewer::PCLVisualizer, spin=1) =
    @cxx cxxpointer(handle(viewer))->spinOnce(spin)

import Base: close

for f in [
        :initCameraParameters,
        :wasStopped,
        :removeAllPointClouds,
        :removeAllShapes,
        :removeAllCoordinateSystems,
        :resetStoppedFlag,
        :close,
        :updateCamera,
        :resetCamera,
        ]
    @eval begin
        $f(viewer::PCLVisualizer) = @cxx cxxpointer(handle(viewer))->$f()
    end
end

function addPointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T};
    id::AbstractString="cloud", viewport::Int=0)
    icxx"""
        $(viewer.handle).get()->addPointCloud<$T>(
            $(cloud.handle), $(pointer(id)), $viewport);"""
end

function addPointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T},
    color_handler::PointCloudColorHandler;
    id::AbstractString="cloud", viewport::Int=0)
    icxx"""
        $(viewer.handle).get()->addPointCloud<$T>(
            $(cloud.handle), *$(color_handler.handle), $(pointer(id)),
            $viewport);"""
end

function updatePointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T};
    id::AbstractString="cloud")
    icxx"""
        $(viewer.handle).get()->updatePointCloud<$T>(
            $(cloud.handle), $(pointer(id)));"""
end

function updatePointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T},
    color_handler::PointCloudColorHandler; id::AbstractString="cloud")
    icxx"""
        $(viewer.handle).get()->updatePointCloud<$T>(
            $(cloud.handle), *$(color_handler.handle), $(pointer(id)));"""
end

function run(viewer::PCLVisualizer; spin::Int=1, sleep::Int=100000)
    icxx"""
    while (!$(viewer.handle)->wasStopped()) {
        $(viewer.handle)->spinOnce($spin);
        boost::this_thread::sleep(boost::posix_time::microseconds($sleep));
    }
    """
end
