using PCLCommon
using PCLIO
using PCLVisualization
using Libfreenect2
using Cxx

const w = 512
const h = 424
save_pcd = false

genfilename(ext=".pcd") =
    joinpath(dirname(@__FILE__), string(now(), "_", time_ns(), ext))

"""Get point cloud from undistored depth and reigstered color"""
function getPointCloudXYZRGB(registration::Registration, undistorted, registered)
    w = Libfreenect2.width(undistorted)
    h = Libfreenect2.height(undistorted)
    cloud = PointCloud{PointXYZRGB}(w, h)
    icxx"$(cloud.handle)->is_dense = false;"
    pointsptr = icxx"&$(cloud.handle)->points[0];"
    icxx"""
    for (size_t ri = 0; ri < $h; ++ri) {
        for (size_t ci = 0; ci < $w; ++ci) {
            auto p = $(pointsptr) + $w * ri + ci;
            $(registration.handle)->getPointXYZRGB($(undistorted.handle),
                $(registered.handle), ri, ci, p->x, p->y, p->z, p->rgb);
        }
    }
    """
    cloud
end

f = Freenect2()
device = openDefaultDevice(f, OpenGLPacketPipeline())
listener = SyncMultiFrameListenerPtr()
setIrAndDepthFrameListener(device, listener)
setColorFrameListener(device, listener)

start(device)

# NOTE: must be called after start(device)
registration = Registration(getIrCameraParams(device),
    getColorCameraParams(device))
undistorted = FramePtr(w, h, 4, key=Libfreenect2.FRAME_DEPTH)
registered = FramePtr(w, h, 4, key=Libfreenect2.FRAME_COLOR)

info("Prepare PCL visualizer...")
global viewer = PCLVisualizer("pcl visualizer")

# Add empty poind
cloud = PointCloud{PointXYZRGB}(w, h)
color_handler = PointCloudColorHandlerRGBField(cloud)
addPointCloud(viewer, cloud, color_handler, id="libfreenect2")

global should_save = false
if !isdefined(:viewer_cb_defined)
cxx"""
void viewer_cb(const pcl::visualization::KeyboardEvent &event) {
    std::cout << "key event:" << event.getKeyCode() << std::endl;
    if (event.getKeyCode() == 's') {
        $:(global should_save = true; nothing);
    }
}
"""
const viewer_cb_defined = true
end

icxx"$(viewer.handle)->registerKeyboardCallback(viewer_cb);"

while !wasStopped(viewer)
    frames = waitForNewFrame(listener)
    color = frames[FrameType.COLOR]
    ir = frames[FrameType.IR]
    depth = frames[FrameType.DEPTH]

    # Depth and color registration
    t = @elapsed begin
        apply(registration, color, depth, undistorted, registered)
        cloud = getPointCloudXYZRGB(registration, undistorted, registered)
    end
    println("Registration and getPointCloudXYZRGB: $(1/t) Hz")

    if should_save
        info("save pcd file...")
        savePCDFile(genfilename(), cloud; binary_mode=true)
        should_save = false
    end

    color_handler = PointCloudColorHandlerRGBField(cloud)
    updatePointCloud(viewer, cloud, color_handler, id="libfreenect2")
    spinOnce(viewer, 1)

    release(listener, frames)

    rand() > 0.95 && gc(false)
end

stop(device)
close(device)
close(viewer)
