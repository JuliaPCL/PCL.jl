using PCL
using Libfreenect2
using Cxx

const w = 512
const h = 424
save_pcd = false

genfilename(ext=".pcd") =
    joinpath(dirname(@__FILE__), string(now(), "_", time_ns(), ext))

"""Get point cloud from undistored depth and reigstered color"""
function getPointCloudXYZRGB(registration, undistorted, registered)
    w = width(undistorted)
    h = height(undistorted)
    cloud = pcl.PointCloud{pcl.PointXYZRGB}(w, h)
    icxx"$(cloud.handle)->is_dense = false;"
    pointsptr = icxx"&$(cloud.handle)->points[0];"
    # NOTE: function call inside loop below is somewhat expensive for real-time
    # point cloud processing. Here we wrap it into a single icxx for efficiency.
    icxx"""
    for (size_t ri = 0; ri < $h; ++ri) {
        for (size_t ci = 0; ci < $w; ++ci) {
            auto p = $(pointsptr) + $w * ri + ci;
            $(registration)->getPointXYZRGB($(undistorted.handle),
                $(registered.handle), ri, ci, p->x, p->y, p->z, p->rgb);
        }
    }
    """
    cloud
end

f = Freenect2()
device = openDefaultDevice(f, OpenGLPacketPipeline())
listener = SyncMultiFrameListener()
setIrAndDepthFrameListener(device, listener)
setColorFrameListener(device, listener)

start(device)

# NOTE: must be called after start(device)
registration = Registration(getIrCameraParams(device),
    getColorCameraParams(device))
undistorted = FrameContainer(w, h, 4, key=Libfreenect2.FRAME_DEPTH)
registered = FrameContainer(w, h, 4, key=Libfreenect2.FRAME_COLOR)

info("Prepare PCL visualizer...")
viewer = pcl.PCLVisualizer("pcl visualizer")

# Add empty poind
cloud = pcl.PointCloud{pcl.PointXYZRGB}(w, h)
color_handler = pcl.PointCloudColorHandlerRGBField(cloud)
pcl.addPointCloud(viewer, cloud, color_handler, id="libfreenect2")

while !pcl.wasStopped(viewer)
    frames = waitForNewFrame(listener)
    color = frames[FrameType.COLOR]
    ir = frames[FrameType.IR]
    depth = frames[FrameType.DEPTH]

    # Depth and color registration
    apply(registration, color, depth, undistorted, registered)

    # Get point cloud from registered color and undistored depth
    @time cloud = getPointCloudXYZRGB(registration, undistorted, registered)

    save_pcd && pcl.savePCDFile(genfilename(), cloud; binary_mode=true)

    color_handler = pcl.PointCloudColorHandlerRGBField(cloud)
    pcl.updatePointCloud(viewer, cloud, color_handler, id="libfreenect2")
    pcl.spinOnce(viewer)

    map(release, [color, ir, depth])
end

stop(device)
close(device)
close(viewer)
