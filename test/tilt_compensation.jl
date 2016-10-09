# Find largest plane from a point cloud and then transform point cloud so that
# the plane is parallel to x0y plane of camera.

using PCLCommon
using PCLIO
using PCLFilters
using PCLSegmentation
using PCLSampleConsensus
using PCLVisualization
using Cxx

# IR camera parameters of Kinect v2
fx = 365.1177
fy = 365.1177
cx = 256.0793
cy = 204.4635

function Base.convert{T}(Array, v::CxxStd.StdVector{T})
    r = T[]
    for e in v
        push!(r,T(e))
    end
    r
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

PT = PointXYZRGB

fname = joinpath(dirname(@__FILE__), "pcd",
    "2016-09-07T19:51:46.802_41015197239218.pcd")
cloud = PointCloud{PT}(joinpath(dirname(@__FILE__), fname))
rotated_model = PointCloud{PT}();
rotated_plane = PointCloud{PT}();

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


### Compute trasnsform matrix so that ground plane is parallel to x0y axis

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

transform = icxx"""
             Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
               transform_2.translation() << 0, 0, 0;
               transform_2.rotate (Eigen::AngleAxisf ($theta, Eigen::Vector3f($(rotation[1]),$(rotation[2]),$(rotation[3]))));
          return transform_2;
      """
icxx"std::cout << $(transform).matrix() << std::endl;"

transformPointCloud(cloud, rotated_model, transform)
transformPointCloud(first(planes), rotated_plane, transform)

# Estimate distance
tilt_compensatated_plane_depth = zeros(
    Float64, PCLCommon.height(cloud), trunc(Int,PCLCommon.width(cloud)*1.2))
getDepthFromPointCloud!(tilt_compensatated_plane_depth, rotated_plane)
estimated_distance = mean(tilt_compensatated_plane_depth[tilt_compensatated_plane_depth .> 0])
@show estimated_distance

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
end
