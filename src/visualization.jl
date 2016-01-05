### visualization ###

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
