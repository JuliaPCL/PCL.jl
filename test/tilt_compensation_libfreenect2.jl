# x0y plane

using PCLCommon
using PCLIO
using PCLFilters
using PCLSegmentation
using PCLSampleConsensus
using PCLVisualization
using Cxx

function Base.convert{T}(Array, v::CxxStd.StdVector{T})
    r = T[]
    for e in v
        push!(r,T(e))
    end
    r
end

# IR camera parameters of Kinect v2
fx = 365.1177
fy = 365.1177
cx = 256.0793
cy = 204.4635

function depth2World(x, y, z, cx, fx, cy, fy)
    X = (Float64(x) - cx) * z /  fx
    Y = (Float64(y) - cy) * z / fy
    Z = z
    (X, Y, Z)
end

function world2Depth(X, Y, Z, cx, fx, cy, fy)
    x = fx * X / Z + cx
    y = fy * Y / Z + cy
    z = Z
    (x, y, z)
end

PT = PointXYZRGB

fname = joinpath(dirname(@__FILE__), "pcd",
    "2016-09-07T19:51:46.802_41015197239218.pcd")
cloud = PointCloud{PT}(joinpath(dirname(@__FILE__), fname))
rotated_model = PointCloud{PT}();

### Extract ground plane
cloud_filtered = deepcopy(cloud)
coefficients = ModelCoefficients()
inliers = PointIndices()

seg = SACSegmentation{PT}()
setOptimizeCoefficients(seg, true)
setModelType(seg, SACMODEL_PLANE)
setMethodType(seg, SAC_RANSAC)
setMaxIterations(seg, 1000)
setDistanceThreshold(seg, 0.01)

ex = ExtractIndices{PT}()
nr_points = length(cloud_filtered)

planes = []
coefs = []
while length(cloud_filtered) > 0.3 * nr_points
    setInputCloud(seg, cloud_filtered)
    segment(seg, inliers, coefficients)

    coefvalues = icxx"$(coefficients.handle)->values;"
    indices = icxx"$(inliers.handle)->indices;"

    push!(coefs, coefvalues)

    if length(indices) == 0
        error("Could not estimate a planar model for the given dataset.")
    end

    cloud_p = PointCloud{PT}()
    cloud_f = PointCloud{PT}()

    setInputCloud(ex, cloud_filtered)
    setIndices(ex, inliers)
    setNegative(ex, false)
    filter(ex, cloud_p)
    println("PointCloud representing the planar component:",
        length(cloud_p), " data points")

    setNegative(ex, true)
    filter(ex, cloud_f)
    cloud_filtered = cloud_f

    push!(planes, cloud_p)
    break
end

neg_plane = cloud_filtered


### Compute trasnsform matrix so that ground plane is parallel with x0y axis

xy_plane_normal = Float32[0.0,0.0,1.0]
ground_normal = convert(Array, coefs[1])[1:end-1]
rotation = cross(xy_plane_normal, ground_normal)
theta = -atan2(norm(rotation), dot(xy_plane_normal, ground_normal))
@show ground_normal
@show rotation
@show theta
@show rad2deg(theta)

# Make rotation a unit vector
rotation = normalize(rotation)
# @assert norm(rotation) == 1.0

#=
transform = icxx"""
              Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
                transform_2.translation() << 0, 0, 0;
                transform_2.rotate (Eigen::AngleAxisf ($theta, Eigen::Vector3f::UnitY()));
           return transform_2;
       """
=#

transform = icxx"""
             Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
               transform_2.translation() << 0, 0, 0;
               transform_2.rotate (Eigen::AngleAxisf ($theta, Eigen::Vector3f($(rotation[1]),$(rotation[2]),$(rotation[3]))));
          return transform_2;
      """
icxx"std::cout << $(transform).matrix() << std::endl;"

transformPointCloud(cloud, rotated_model, transform)

if isdefined(:vis) && vis
    viewer = PCLVisualizer("test");
    addCoordinateSystem(viewer, 1.2, 0,0,0)
    addPointCloud(viewer, cloud,
        PointCloudColorHandlerCustom(cloud, 0,255,0), id="input")
    addPointCloud(viewer, first(planes),
        PointCloudColorHandlerCustom(first(planes), 0,0,255), id="ground")
    addPointCloud(viewer, rotated_model,
        PointCloudColorHandlerRGBField(rotated_model), id="rotated_model")
    try
        spin(viewer)
    finally
        close(viewer);viewer=0;gc();gc()
    end
    error()
end


function getDepthFromPointCloud!(depth, cloud)
    # @assert size(depth) == (512,424)
    fill!(depth, 0.0)
    w, h = size(depth)
    depthp = pointer(depth)
    icxx"""
        const float fx = $fx;
        const float fy = $fy;
        const float cx = $cx;
        const float cy = $cy;
        for (size_t i = 0; i < $(cloud.handle)->points.size(); ++i) {
            const auto p = $(cloud.handle)->points[i];
            if (std::isnan(p.z) || p.z <= 0) {
                continue;
            }
            int r, c;
            float depth;
            c = fx * p.x / p.z + cx;
            r = fy * p.y / p.z + cy;
            depth = p.z * 1000.0;
            if (r >= 0 && r < $h && c >= 0 && c < $w) {
                $(depthp)[$w * r + c] = depth;
            }
        }
    """
    depth
end

using Libfreenect2
using Cxx
using CVCore
using CVHighGUI
using CVImgProc
using CVImgCodecs


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


isesc(key) = key == 27

f = Freenect2()
num_devices = enumerateDevices(f)
if num_devices <= 0
    error("No device!")
end
device = openDefaultDevice(f, OpenCLPacketPipeline())
listener = SyncMultiFrameListenerPtr()
setIrAndDepthFrameListener(device, listener)
setColorFrameListener(device, listener)

start(device)

# NOTE: must be called after start(device)
registration = Registration(getIrCameraParams(device),
    getColorCameraParams(device))
undistorted = FramePtr(512, 424, 4, key=Libfreenect2.FRAME_DEPTH)
registered = FramePtr(512, 424, 4, key=Libfreenect2.FRAME_COLOR)

tilt_img_scale_w = 2
tilt_img_scale_h = 1
tilt_compensatated_depth = zeros(Float64,
    Int(512*tilt_img_scale_w), Int(424*tilt_img_scale_h))

pass = PassThrough{PT}()
sor = VoxelGrid{PT}()

try
    viewer = PCLVisualizer("pcl visualizer")
    addCoordinateSystem(viewer, 1.2, 0,0,0)
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

        # Point cloud density reduction
        cloud_filtered = PointCloud{PT}();

        setInputCloud(sor, cloud)
        setLeafSize(sor, 0.01, 0.01, 0.01)
        filter(sor, cloud_filtered)
        cloud = cloud_filtered

        setInputCloud(pass, cloud)
        setFilterFieldName(pass, "z")
        setFilterLimits(pass, 1.2, 3.0)
        filter(pass, cloud_filtered)
        cloud = cloud_filtered


        # tilt compensation
        rotated_model = PointCloud{PT}();
        t = @elapsed begin
            transformPointCloud(cloud, rotated_model, transform)
        end
        println("transformPointCloud: $(1/t) Hz")

        # Get tilt compensatd depth
        t = @elapsed begin
            getDepthFromPointCloud!(tilt_compensatated_depth, rotated_model);
        end
        println("getDepthFromPointCloud: $(1/t) Hz")

        scale!(tilt_compensatated_depth, 1/4500)

        # Get depth array for comparison
        deptharr = Array(depth)
        scale!(deptharr, 1/4500)

        # Update point cloud viewer
        cloud_color_handler = PointCloudColorHandlerCustom(cloud, 0,255,0)
        if !updatePointCloud(viewer, cloud, cloud_color_handler, id="input_cloud")
            addPointCloud(viewer, cloud, cloud_color_handler, id="input_cloud")
        end
        color_handler = PointCloudColorHandlerRGBField(cloud)
        if !updatePointCloud(viewer, rotated_model, color_handler, id="rotated_model")
            addPointCloud(viewer, rotated_model, color_handler, id="rotated_model")
        end
        spinOnce(viewer, 1)

        # Show depth
        imshow("original depth", deptharr)
        imshow("tilt compensated depth", tilt_compensatated_depth)
        key = waitKey(1)
        if key == Int('s')
            scale!(deptharr, 255)
            scale!(tilt_compensatated_depth, 255)
            depthmat = Mat(deptharr)
            flip!(depthmat, 1)
            tilt_compensatated_depth_mat = Mat(tilt_compensatated_depth)
            flip!(tilt_compensatated_depth_mat, 1)
            imwrite("original_depth.png", depthmat)
            imwrite("tilt_compensated_depth.png", tilt_compensatated_depth_mat)
        end
        isesc(key) && break

        Libfreenect2.release(listener, frames)

        rand() > 0.95 && gc(false)
    end
finally
    stop(device)
    close(device)
    close(viewer)
    destroyAllWindows()
    device=0;viewer=0;gc()
end
