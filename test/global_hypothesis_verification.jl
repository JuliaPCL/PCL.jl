# Hypothesis Verification for 3D Object Recognition
# http://www.pointclouds.org/documentation/tutorials/global_hypothesis_verification.php#global-hypothesis-verification

using PCLCommon
using PCLIO
using PCLFilters
using PCLFeatures
using PCLKDTree
using PCLRecognition
using PCLRegistration
using PCLVisualization
using Cxx

import PCLCommon: CorrespondenceVal

model_pcd = Pkg.dir("PCLIO", "test", "data", "milk.pcd")
scene_pcd = Pkg.dir("PCLIO", "test", "data", "milk_cartoon_all_small_clorox.pcd")

T = PointXYZRGBA
NT = Normal
DT = SHOT352
RT = ReferenceFrame

model_ss = 0.02
scene_ss = 0.02
descr_rad = 0.02
rf_rad = 0.015
cg_size = 0.035
cg_thresh = 5.0
use_hough = true

# Hypothesis verification
hv_clutter_reg = 5.0
hv_inlier_th = 0.005
hv_occlusion_th = 0.01
hv_rad_clutter = 0.03
hv_regularizer = 3.0
hv_rad_normals = 0.05
hv_detect_clutter = false

# ICP
icp_max_iter = 5
icp_corr_distance = 0.005

model = PointCloud{T}(model_pcd)
scene = PointCloud{T}(scene_pcd)
model_keypoints = PointCloud{T}()
scene_keypoints = PointCloud{T}()
model_normals = PointCloud{NT}()
scene_normals = PointCloud{NT}()
model_descriptors = PointCloud{DT}()
scene_descriptors = PointCloud{DT}()

info("Normal estimation")
norm_est = NormalEstimationOMP{T,NT}()
setKSearch(norm_est, 10)
setInputCloud(norm_est, model)
compute(norm_est, model_normals)

setInputCloud(norm_est, scene)
compute(norm_est, scene_normals)

# Downsample
info("Uniform sampling")
uniform_sampling = UniformSampling{T}()
setInputCloud(uniform_sampling, model)
PCLFilters.setRadiusSearch(uniform_sampling, model_ss)
filter(uniform_sampling, model_keypoints)
println("Model total points: $(length(model)); Selectd Key points: $(length(model_keypoints))")

setInputCloud(uniform_sampling, scene)
PCLFilters.setRadiusSearch(uniform_sampling, scene_ss)
filter(uniform_sampling, scene_keypoints)
println("Scene total points: $(length(scene)); Selectd Key points: $(length(scene_keypoints))")

info("Associating a 3D descriptor to each model and scene keypoint")
descr_est = SHOTEstimationOMP{T,NT,DT}()
PCLFeatures.setRadiusSearch(descr_est, descr_rad)
setInputCloud(descr_est, model_keypoints)
setInputNormals(descr_est, model_normals)
setSearchSurface(descr_est, model)
compute(descr_est, model_descriptors)

setInputCloud(descr_est, scene_keypoints)
setInputNormals(descr_est, scene_normals)
setSearchSurface(descr_est, scene)
compute(descr_est, scene_descriptors)

info("Find Model-Scene Correspondences with KdTree")
match_search = KdTreeFLANN{DT}()
setInputCloud(match_search, model_descriptors)

model_scene_corrs = Correspondences()
for i in 0:length(scene_descriptors)-1
    neigh_indices = icxx"std::vector<int>(1);"
    neigh_sqr_dists = icxx"std::vector<float>(1);"
    p = icxx"$(scene_descriptors[i]).descriptor[0];"
    !isfinite(Cfloat(p)) && continue

    found_neighs = nearestKSearch(match_search, scene_descriptors[i],
        1, neigh_indices, neigh_sqr_dists)
    if found_neighs == 1 && (Float32(neigh_sqr_dists[0]) < Float32(0.25))
        corr = CorrespondenceVal(neigh_indices[0], i, neigh_sqr_dists[0])
        push!(model_scene_corrs, corr)
    end
end
@show length(model_scene_corrs)

info("Clustering")
# TODO: this is apperently cheating.. Re-allocations causes segfault, so reserve enough
# memory in advance
rototranslations = icxx"""
    auto v = std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>();
    v.reserve(10);
    return v;
"""
clustered_corrs = icxx"std::vector<pcl::Correspondences>();"

clusterer = use_hough ? Hough3DGrouping{T,T,RT,RT}() :
    GeometricConsistencyGrouping{T,T}()

if use_hough
    model_rf = PointCloud{RT}()
    scene_rf = PointCloud{RT}()
    rf_est = BOARDLocalReferenceFrameEstimation{T,NT,RT}()
    setFindHoles(rf_est, true)
    PCLFeatures.setRadiusSearch(rf_est, rf_rad)

    setInputCloud(rf_est, model_keypoints)
    setInputNormals(rf_est, model_normals)
    setSearchSurface(rf_est, model)
    compute(rf_est, model_rf)

    setInputCloud(rf_est, scene_keypoints)
    setInputNormals(rf_est, scene_normals)
    setSearchSurface(rf_est, scene)
    compute(rf_est, scene_rf)

    setHoughBinSize(clusterer, cg_size)
    setHoughThreshold(clusterer, cg_thresh)
    setUseInterpolation(clusterer, true)
    setUseDistanceWeight(clusterer, false)
else
    setGCSize(clusterer, cg_size)
    setGCThreshold(clusterer, cg_thresh)
end

setInputCloud(clusterer, model_keypoints)
use_hough && setInputRf(clusterer, model_rf)
setSceneCloud(clusterer, scene_keypoints)
use_hough && setSceneRf(clusterer, scene_rf)
setModelSceneCorrespondences(clusterer, model_scene_corrs)
recognize(clusterer, rototranslations, clustered_corrs)

@show length(rototranslations)

# error("")
# instances = CxxStd.StdVector{cxxt"pcl::PointCloud<$T>::ConstPtr"}()
instances = icxx"std::vector<pcl::PointCloud<$T>::ConstPtr>();"
for i in 0:length(rototranslations)-1
    rotated_model = PointCloud{T}()
    # transformPointCloud(model, rotated_model, rototranslations[i])
    @show i
    hoge = icxx"$(rototranslations)[$i];"
    transformPointCloud(model, rotated_model, hoge)
    push!(instances, rotated_model.handle)
end

registered_instances = icxx"std::vector<pcl::PointCloud<$T>::ConstPtr>();"

info("ICP")
for i in 0:length(rototranslations)-1
    icp = IterativeClosestPoint{T,T}()
    setMaximumIterations(icp, icp_max_iter)
    setMaxCorrespondenceDistance(icp, icp_corr_distance)
    setInputTarget(icp, scene)
    setInputSource(icp, instances[i])
    registered = PointCloud{T}()
    align(icp, registered)

    print("Instance $i ")
    if hasConverged(icp)
        println("Aligned!")
    else
        println("Not aligned!")
    end
    push!(registered_instances, registered.handle)
end

hypothesis_mask = icxx"std::vector<bool>();"

GoHv = GlobalHypothesesVerification{T,T}()
setSceneCloud(GoHv, scene)
addModels(GoHv, registered_instances, true)
setInlierThreshold(GoHv, hv_inlier_th)
setRegularizer(GoHv, hv_regularizer)
setRadiusClutter(GoHv, hv_rad_clutter)
setClutterRegularizer(GoHv, hv_clutter_reg)
setDetectClutter(GoHv, hv_detect_clutter)
setRadiusNormals(GoHv, hv_rad_normals)

verify(GoHv)
getMask(GoHv, hypothesis_mask)
for i in 0:length(hypothesis_mask)-1
    if icxx"$hypothesis_mask[$i] == true;"
        println("Instance $i is good!")
    else
        println("Instance $i is bad!")
    end
end

if isdefined(:vis) && vis
    info("Prepare PCL visualizer...")
    viewer = PCLVisualizer("pcl visualizer")
    color_handler = PointCloudColorHandlerRGBField(scene)
    addPointCloud(viewer, scene, color_handler, id="scene")

    for i in 0:length(rototranslations)-1
        rotated_model = PointCloud{T}()
        transformPointCloud(model, rotated_model, rototranslations[i])

        r,g,b = icxx"$hypothesis_mask[$i]==true;" ? (255,0,0) : (0,255,0)
        color_handler = PointCloudColorHandlerCustom(scene, r, g, b)
        addPointCloud(viewer, rotated_model, color_handler, id="instance $i")
    end
    spin(viewer)
    close(viewer)
end
