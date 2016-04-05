# Tracking object in real time
# similar: http://pointclouds.org/documentation/tutorials/tracking.php

using PCL
using Libfreenect2
using Cxx

RT = pcl.PointXYZRGB
PT = pcl.ParticleXYZRPY

# Tracking target
target_cloud = pcl.PointCloud{RT}(Pkg.dir("PCL", "test", "data",
    "kumamon_head.pcd"))

downsampling_grid_size =  0.010;


function filterPassThrough{T}(cloud_in::pcl.PointCloud{T},
        cloud_out::pcl.PointCloud{T})
    pass = pcl.PassThrough{T}()
    pcl.setFilterFieldName(pass, "z")
    pcl.setFilterLimits(pass, 0.0, 1.4)
    pcl.setKeepOrganized(pass, false)
    pcl.setInputCloud(pass, cloud_in)
    filter(pass, cloud_out)
end

function gridSampleApprox{T}(cloud_in::pcl.PointCloud{T},
        cloud_out::pcl.PointCloud{T}, leaf_size)
    grid = pcl.ApproximateVoxelGrid{T}()
    pcl.setLeafSize(grid, leaf_size, leaf_size, leaf_size)
    pcl.setInputCloud(grid, cloud_in)
    filter(grid, cloud_out)
end

default_step_covariance = icxx"std::vector<double>(6, 0.015 * 0.015);"
icxx"$(default_step_covariance)[3] *= 40.0;"
icxx"$(default_step_covariance)[4] *= 40.0;"
icxx"$(default_step_covariance)[5] *= 40.0;"

initial_noise_covariance = icxx"std::vector<double>(6, 0.00001);"
default_initial_mean = icxx"std::vector<double>(6, 0.0);"

# Initialize tracker
global tracker = pcl.KLDAdaptiveParticleFilterOMPTracker{RT,PT}(8)

bin_size = PT()
icxx"$bin_size.x = 0.1f;"
icxx"$bin_size.y = 0.1f;"
icxx"$bin_size.z = 0.1f;"
icxx"$bin_size.roll = 0.1f;"
icxx"$bin_size.pitch = 0.1f;"
icxx"$bin_size.yaw = 0.1f;"

pcl.setMaximumParticleNum(tracker, 500)
pcl.setDelta(tracker, 0.99)
pcl.setEpsilon(tracker, 0.2)
pcl.setBinSize(tracker, bin_size)

pcl.setTrans(tracker, icxx"Eigen::Affine3f::Identity();")
pcl.setStepNoiseCovariance(tracker, default_step_covariance)
pcl.setInitialNoiseCovariance(tracker, initial_noise_covariance)
pcl.setInitialNoiseMean(tracker, default_initial_mean);
pcl.setIterationNum(tracker, 1);
pcl.setParticleNum(tracker, 400);
pcl.setResampleLikelihoodThr(tracker, 0.00);
pcl.setUseNormal(tracker, false);

# Setup coherence object for tracking
coherence = pcl.ApproxNearestPairPointCloudCoherence{RT}()

distance_coherence = pcl.DistanceCoherence{RT}()
color_coherence = pcl.HSVColorCoherence{RT}()
pcl.setWeight(color_coherence, 0.1)

pcl.addPointCoherence(coherence, distance_coherence)
pcl.addPointCoherence(coherence, color_coherence)

search = pcl.Octree{RT}(0.01)
pcl.setSearchMethod(coherence, search)
pcl.setMaximumDistance(coherence, 0.01)

pcl.setCloudCoherence(tracker, coherence)

# prepare the model of tracker's target
c = icxx"Eigen::Vector4f();"
trans = icxx"Eigen::Affine3f trans = Eigen::Affine3f::Identity(); return trans;"
transed_ref = pcl.PointCloud{RT}()
transed_ref_downsampled = pcl.PointCloud{RT}()

pcl.compute3DCentroid(target_cloud, c)
vec = icxx"Eigen::Vector3f($(c)[0], $(c)[1], $(c)[2]);"
icxx"$(trans).translation().matrix() = $vec;"
pcl.transformPointCloud(target_cloud, transed_ref, icxx"$trans.inverse();")
gridSampleApprox(transed_ref, transed_ref_downsampled, downsampling_grid_size)
@show length(target_cloud)
@show length(transed_ref)
@show length(transed_ref_downsampled)

pcl.setReferenceCloud(tracker, transed_ref_downsampled)
pcl.setTrans(tracker, trans)


global counter = 0

function update(tracker, cloud)
    cloud_pass = pcl.PointCloud{RT}()
    cloud_pass_downsampled = pcl.PointCloud{RT}()
    filterPassThrough(cloud, cloud_pass)
    gridSampleApprox(cloud_pass, cloud_pass_downsampled, downsampling_grid_size)
    if counter > 10
        pcl.setInputCloud(tracker, cloud_pass_downsampled)
        t = @elapsed pcl.compute(tracker)
        println("compute: $(1/t) Hz")
    end
    return cloud_pass_downsampled
end

function draw(tracker, viewer)
    particles = icxx"$(tracker.handle)->getParticles();"
    icxx"$particles == nullptr;" && return

    particle_cloud = pcl.PointCloud{pcl.PointXYZ}()
    icxx"""
    for (size_t i = 0; i < $particles->points.size(); i++) {
      pcl::PointXYZ point;
      point.x = $particles->points[i].x;
      point.y = $particles->points[i].y;
      point.z = $particles->points[i].z;
      $(particle_cloud.handle)->points.push_back(point);
    }
    """
    red_color = pcl.PointCloudColorHandlerCustom{pcl.PointXYZ}(particle_cloud,
        255, 0, 0)
    if !pcl.updatePointCloud(viewer, particle_cloud, red_color, id="particle cloud")
        pcl.addPointCloud(viewer, particle_cloud, red_color, id="particle cloud")
    end

    result = pcl.getResult(tracker)
    transformation = pcl.toEigenMatrix(tracker, result)
    icxx"$transformation.translation() += Eigen::Vector3f(0.0f, 0.0f, -0.005f);"
    result_cloud = pcl.PointCloud{RT}()
    icxx"""
        pcl::transformPointCloud<$RT>(*($(tracker.handle)->getReferenceCloud()),
            *$(result_cloud.handle), $transformation);
    """
    blue_color = pcl.PointCloudColorHandlerCustom{RT}(result_cloud, 0, 0, 255)
    if !pcl.updatePointCloud(viewer, result_cloud, blue_color, id="result cloud")
        pcl.addPointCloud(viewer, result_cloud, blue_color, id="result cloud")
    end
end


const w = 512
const h = 424
save_pcd = false

genfilename(ext=".pcd") =
    joinpath(dirname(@__FILE__), string(now(), "_", time_ns(), ext))

"""Get point cloud from undistored depth and reigstered color"""
function getPointCloudXYZRGB(registration::Registration, undistorted, registered)
    w = width(undistorted)
    h = height(undistorted)
    cloud = pcl.PointCloud{RT}(w, h)
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
global viewer = pcl.PCLVisualizer("pcl visualizer")

global should_save = false
if !isdefined(:viewer_cb_defined) && VERSION < v"0.5.0-dev+2396"
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
if VERSION < v"0.5.0-dev+2396"
    icxx"$(pcl.handle(viewer))->registerKeyboardCallback(viewer_cb);"
end

while !pcl.wasStopped(viewer)
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

    # tracking
    t = @elapsed begin
        cloud_pass_downsampled = update(tracker, cloud)
    end
    println("Update tracker: $(1/t) Hz")

    # draw particles
    draw(tracker, viewer)

    if should_save
        info("save pcd file...")
        pcl.savePCDFile(genfilename(), cloud; binary_mode=true)
        should_save = false
    end

    color_handler = pcl.PointCloudColorHandlerRGBField(cloud_pass_downsampled)
    updated = pcl.updatePointCloud(viewer, cloud_pass_downsampled, color_handler,
        id="libfreenect2")
    if !updated
        pcl.addPointCloud(viewer, cloud_pass_downsampled, color_handler, id="libfreenect2")
    end
    pcl.spinOnce(viewer, 1)

    release(listener, frames)

    rand() > 0.95 && gc(false)
    global counter += 1
end

stop(device)
close(device)
close(viewer)
