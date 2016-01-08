### visualization ###

abstract PointCloudColorHandler

@inline handle(p::PointCloudColorHandler) = p.handle

typealias CxxPointCloudColorHandlerRGBField{T} cxxt"boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<$T>>"
typealias CxxPointCloudColorHandlerCustom{T} cxxt"boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<$T>>"

type PointCloudColorHandlerRGBField{T} <: PointCloudColorHandler
    handle::CxxPointCloudColorHandlerRGBField
end

function call{T}(::Type{PointCloudColorHandlerRGBField{T}}, cloud::PointCloud)
    handle = icxx"""
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<$T>>(
            new pcl::visualization::PointCloudColorHandlerRGBField<$T>(
                $(cloud.handle)));"""
    PointCloudColorHandlerRGBField{T}(handle)
end

call{T}(::Type{PointCloudColorHandlerRGBField}, cloud::PointCloud{T}) =
    PointCloudColorHandlerRGBField{T}(cloud)


type PointCloudColorHandlerCustom{T} <: PointCloudColorHandler
    handle::CxxPointCloudColorHandlerCustom
end

function call{T}(::Type{PointCloudColorHandlerCustom{T}}, cloud::PointCloud,
    r, g, b)
    handle = icxx"""
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<$T>>(
            new pcl::visualization::PointCloudColorHandlerCustom<$T>(
                $(cloud.handle), $r, $g, $b));"""
    PointCloudColorHandlerCustom{T}(handle)
end

call{T}(::Type{PointCloudColorHandlerCustom}, cloud::PointCloud{T}, r, g, b) =
    PointCloudColorHandlerCustom{T}(cloud, r, g, b)


const CxxPCLVisualizerPtr = cxxt"boost::shared_ptr<pcl::visualization::PCLVisualizer>"

type PCLVisualizer
    handle::CxxPCLVisualizerPtr
    PCLVisualizer(handle::CxxPCLVisualizerPtr) = new(handle)
end

@inline handle(viewer::PCLVisualizer) = viewer.handle

function PCLVisualizer(name::AbstractString="", create_interactor::Bool=true)
    handle = icxx"""
        boost::shared_ptr<pcl::visualization::PCLVisualizer>(
            new pcl::visualization::PCLVisualizer(
                $(pointer(name)), $create_interactor)
            );"""
    PCLVisualizer(handle)
end

setBackgroundColor(viewer::PCLVisualizer, x, y, z) =
    @cxx cxxpointer(handle(viewer))->setBackgroundColor(x, y, z)
addCoordinateSystem(viewer::PCLVisualizer, n) =
    @cxx cxxpointer(handle(viewer))->addCoordinateSystem($n)
initCameraParameters(viewer::PCLVisualizer) =
    @cxx cxxpointer(handle(viewer))->initCameraParameters()
wasStopped(viewer::PCLVisualizer) =
    @cxx cxxpointer(handle(viewer))->wasStopped()
spinOnce(viewer::PCLVisualizer, spin=100) =
    @cxx cxxpointer(handle(viewer))->spinOnce(spin)

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

function run(viewer::PCLVisualizer; spin::Int=100, sleep::Int=100000)
    icxx"""
    while (!$(viewer.handle)->wasStopped()) {
        $(viewer.handle)->spinOnce($spin);
        boost::this_thread::sleep(boost::posix_time::microseconds($sleep));
    }
    """
end
