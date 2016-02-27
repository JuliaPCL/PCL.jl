using Cxx
using OpenNI2
using PCL

const ni2 = OpenNI2

const w = 640
const h = 480
const pxfmt = ni2.PIXEL_FORMAT_DEPTH_1_MM

function getPointXYZ(stream, deptharr)
    w, h = size(deptharr)
    depthvec = vec(deptharr)
    depthp = pointer(depthvec)
    cloud = pcl.PointCloud{pcl.PointXYZ}(w, h)
    icxx"$(cloud.handle)->is_dense = false;"
    pointsptr = icxx"&$(cloud.handle)->points[0];"
    icxx"""
    const float millimeterToMeter = 0.001f;
    for (int ri = 0; ri < $h; ++ri) {
        for (int ci = 0; ci < $w; ++ci) {
            auto p = $(pointsptr) + $w * ri + ci;
            auto z = $(depthp) + $w * ri + ci;
            openni::CoordinateConverter::convertDepthToWorld(
                *$(stream.handle), ci, ri, *z, &p->x, &p->y, &p->z);
            p->x *= millimeterToMeter;
            p->y *= millimeterToMeter;
            p->z *= millimeterToMeter;
        }
    }
    """
    cloud
end

ni2.initialize()
device = ni2.DevicePtr()
ni2.open(device)
ni2.setDepthColorSyncEnabled(device, false)

di = ni2.getDeviceInfo(device)
@show ni2.getName(di)
@show ni2.getVendor(di)

depth = ni2.VideoStreamPtr()
ni2.create(depth, device, ni2.SENSOR_DEPTH)
ni2.setMirroringEnabled(depth, true)

modes = ni2.getSupportedVideoModes(ni2.getSensorInfo(device, ni2.SENSOR_DEPTH))
for mode in modes
    if Int(ni2.getResolutionX(mode)) == w &&
            Int(ni2.getResolutionY(mode)) == h &&
            ni2.getPixelFormat(mode) == pxfmt
        println("$w x $h, PIXEL_FORMAT_DEPTH_1_MM: video mode found")
        ni2.setVideoMode(depth, mode)
        break
    end
end

ni2.start(depth)

frame = ni2.VideoFrameRef()

info("Prepare PCL visualizer...")
global viewer = pcl.PCLVisualizer("pcl visualizer")

while true
    ni2.waitForAnyStream([depth])
    ni2.readFrame(depth, frame)
    arr = convert(Array{Cushort,2}, frame)
    @assert size(arr) == (w, h)

    cloud = getPointXYZ(depth, arr)
    if !pcl.updatePointCloud(viewer, cloud, id="depth")
        pcl.addPointCloud(viewer, cloud, id="depth")
    end
    pcl.spinOnce(viewer, 1)

    rand() > 0.95 && gc(false)
end

ni2.stop(depth)
ni2.close(device)
ni2.shutdown()

close(viewer)
