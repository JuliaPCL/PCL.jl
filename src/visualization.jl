### visualization ###

typealias CxxPointCloudColorHandlerRGBField{T} cxxt"boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<$T>>"

abstract PointCloudColorHandler

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


const CxxPCLVisualizerPtr = cxxt"boost::shared_ptr<pcl::visualization::PCLVisualizer>"

type PCLVisualizer
    handle::CxxPCLVisualizerPtr
    PCLVisualizer(handle::CxxPCLVisualizerPtr) = new(handle)
end

function PCLVisualizer(name::AbstractString="", create_interactor::Bool=true)
    handle = icxx"""
        boost::shared_ptr<pcl::visualization::PCLVisualizer>(
            new pcl::visualization::PCLVisualizer(
                $(pointer(name)), $create_interactor)
            );"""
    PCLVisualizer(handle)
end

function setBackgroundColor(viewer::PCLVisualizer, x, y, z)
    icxx"$(viewer.handle).get()->setBackgroundColor($x, $y, $z);"
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

function addCoordinateSystem(viewer::PCLVisualizer, n)
    icxx"$(viewer.handle).get()->addCoordinateSystem($n);"
end

function initCameraParameters(viewer::PCLVisualizer)
    icxx"$(viewer.handle).get()->initCameraParameters();"
end

function run(viewer::PCLVisualizer; spin::Int=100, sleep::Int=100000)
    icxx"""
    while (!$(viewer.handle)->wasStopped()) {
        $(viewer.handle)->spinOnce($spin);
        boost::this_thread::sleep(boost::posix_time::microseconds($sleep));
    }
    """
end
