# 3D Object Recognition based on Correspondence Grouping
# http://www.pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping

using PCL
using Cxx

model_pcd = Pkg.dir("PCL", "test", "data", "milk.pcd")
scene_pcd = Pkg.dir("PCL", "test", "data", "milk_cartoon_all_small_clorox.pcd")

T = pcl.PointXYZRGBA
NT = pcl.Normal
DT = pcl.SHOT352
RT = pcl.ReferenceFrame

model_ss = 0.01
scene_ss = 0.03
descr_rad = 0.02
rf_rad = 0.015
cg_size = 0.01
cg_thresh = 5.0
use_hough = true

model = pcl.PointCloud{T}(model_pcd)
scene = pcl.PointCloud{T}(scene_pcd)
model_keypoints = pcl.PointCloud{T}()
scene_keypoints = pcl.PointCloud{T}()
model_normals = pcl.PointCloud{NT}()
scene_normals = pcl.PointCloud{NT}()
model_descriptors = pcl.PointCloud{DT}()
scene_descriptors = pcl.PointCloud{DT}()

info("Normal estimation")
norm_est = pcl.NormalEstimationOMP{T,NT}()
pcl.setKSearch(norm_est, 10)
pcl.setInputCloud(norm_est, model)
pcl.compute(norm_est, model_normals)

pcl.setInputCloud(norm_est, scene)
pcl.compute(norm_est, scene_normals)

# Downsample
info("Uniform sampling")
uniform_sampling = pcl.UniformSampling{T}()
pcl.setInputCloud(uniform_sampling, model)
pcl.setRadiusSearch(uniform_sampling, model_ss)
pcl.filter(uniform_sampling, model_keypoints)
println("Model total points: $(length(model)); Selectd Key points: $(length(model_keypoints))")

pcl.setInputCloud(uniform_sampling, scene)
pcl.setRadiusSearch(uniform_sampling, scene_ss)
pcl.filter(uniform_sampling, scene_keypoints)
println("Scene total points: $(length(scene)); Selectd Key points: $(length(scene_keypoints))")

info("Associating a 3D descriptor to each model and scene keypoint")
descr_est = pcl.SHOTEstimationOMP{T,NT,DT}()
pcl.setRadiusSearch(descr_est, descr_rad)
pcl.setInputCloud(descr_est, model_keypoints)
pcl.setInputNormals(descr_est, model_normals)
pcl.setSearchSurface(descr_est, model)
pcl.compute(descr_est, model_descriptors)

pcl.setInputCloud(descr_est, scene_keypoints)
pcl.setInputNormals(descr_est, scene_normals)
pcl.setSearchSurface(descr_est, scene)
pcl.compute(descr_est, scene_descriptors)

info("Find Model-Scene Correspondences with KdTree")
match_search = pcl.KdTreeFLANN{DT}()
pcl.setInputCloud(match_search, model_descriptors)

model_scene_corrs = pcl.Correspondences()
for i in 0:length(scene_descriptors)-1
    neigh_indices = pcl.StdVector{Cint}(1)
    neigh_sqr_dists = pcl.StdVector{Cfloat}(1)
    p = icxx"$(scene_descriptors[i]).descriptor[0];"
    !isfinite(Cfloat(p)) && continue

    found_neighs = pcl.nearestKSearch(match_search, scene_descriptors[i],
        1, neigh_indices, neigh_sqr_dists)
    if found_neighs == 1 && (Float32(neigh_sqr_dists[0]) < Float32(0.25))
        corr = pcl.Correspondence(neigh_indices[0], i, neigh_sqr_dists[0])
        push!(model_scene_corrs, corr)
    end
end
@show length(model_scene_corrs)

info("Clustering")
rototranslations = icxx"""
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>();
"""
clustered_corrs = icxx"std::vector<pcl::Correspondences>();"

clusterer = use_hough ? pcl.Hough3DGrouping{T,T,RT,RT}() :
    pcl.GeometricConsistencyGrouping{T,T}()

if use_hough
    model_rf = pcl.PointCloud{RT}()
    scene_rf = pcl.PointCloud{RT}()
    rf_est = pcl.BOARDLocalReferenceFrameEstimation{T,NT,RT}()
    pcl.setFindHoles(rf_est, true)
    pcl.setRadiusSearch(rf_est, rf_rad)

    pcl.setInputCloud(rf_est, model_keypoints)
    pcl.setInputNormals(rf_est, model_normals)
    pcl.setSearchSurface(rf_est, model)
    pcl.compute(rf_est, model_rf)

    pcl.setInputCloud(rf_est, scene_keypoints)
    pcl.setInputNormals(rf_est, scene_normals)
    pcl.setSearchSurface(rf_est, scene)
    pcl.compute(rf_est, scene_rf)

    pcl.setHoughBinSize(clusterer, cg_size)
    pcl.setHoughThreshold(clusterer, cg_thresh)
    pcl.setUseInterpolation(clusterer, true)
    pcl.setUseDistanceWeight(clusterer, false)
else
    pcl.setGCSize(clusterer, cg_size)
    pcl.setGCThreshold(clusterer, cg_thresh)
end

pcl.setInputCloud(clusterer, model_keypoints)
use_hough && pcl.setInputRf(clusterer, model_rf)
pcl.setSceneCloud(clusterer, scene_keypoints)
use_hough && pcl.setSceneRf(clusterer, scene_rf)
pcl.setModelSceneCorrespondences(clusterer, model_scene_corrs)
pcl.recognize(clusterer, rototranslations, clustered_corrs)

@show length(rototranslations)

info("Prepare PCL visualizer...")
viewer = pcl.PCLVisualizer("pcl visualizer")
color_handler = pcl.PointCloudColorHandlerRGBField(scene)
pcl.addPointCloud(viewer, scene, color_handler, id="scene")

for i in 0:length(rototranslations)-1
    rotated_model = pcl.PointCloud{T}()
    pcl.transformPointCloud(model, rotated_model, rototranslations[i])

    rotated_model_color_handler = pcl.PointCloudColorHandlerCustom(scene,
        255, 0, 0)
    pcl.addPointCloud(viewer, rotated_model, rotated_model_color_handler,
        id="instance $i")
end

while !pcl.wasStopped(viewer)
    pcl.spinOnce(viewer)
end
