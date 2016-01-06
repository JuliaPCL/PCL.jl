doc = """3D Object Recognition based on Correspondence Grouping

Usage:
  correspondence_grouping.jl <model_pcd> <scene_pcd> [options]
  correspondence_grouping.jl -h | --help
  correspondence_grouping.jl --version

Options:
  -h --help         Show this screen.
  --version         Show version.
  --point_type=T    Point type [default: PointXYZRGBA]
  --normal_type=NT  Normal point type [default: Normal]
  --descr_type=DT   Description point type [default: SHOT352]
  --model_ss=ms     Model uniform sampling radius [default: 0.01]
  --scene_ss=ss     Scene uniform sampling radius [default: 0.03]
  --descr_rad=rad   Descriptor radius [default: 0.02]
  --cg_size=cg      Cluster size [default: 0.01]
  --cg_thresh=th    Clustering threshold [default: 5.0]
"""

using DocOpt
using PCL
using Cxx

let
    args = docopt(doc, version=v"0.0.1")
    @show args

    model_pcd = args["<model_pcd>"]
    scene_pcd = args["<scene_pcd>"]
    T = eval(Expr(:., :pcl, QuoteNode(symbol(args["--point_type"]))))
    NT = eval(Expr(:., :pcl, QuoteNode(symbol(args["--normal_type"]))))
    DT = eval(Expr(:., :pcl, QuoteNode(symbol(args["--descr_type"]))))
    model_ss = parse(Float64, args["--model_ss"])
    scene_ss = parse(Float64, args["--scene_ss"])
    descr_rad = parse(Float64, args["--descr_rad"])
    cg_size = parse(Float64, args["--cg_size"])
    cg_thresh = parse(Float64, args["--cg_thresh"])


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

    info("Clustering...")
    rototranslations = icxx"""
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>();
    """
    clustered_corrs = icxx"std::vector<pcl::Correspondences>();"

    gc_clusterer = pcl.GeometricConsistencyGrouping{T,T}()

    pcl.setGCSize(gc_clusterer, Cfloat(cg_size))
    pcl.setGCThreshold(gc_clusterer, Cfloat(cg_thresh))
    pcl.setInputCloud(gc_clusterer, model_keypoints)
    pcl.setSceneCloud(gc_clusterer, scene_keypoints)
    pcl.setModelSceneCorrespondences(gc_clusterer, model_scene_corrs)
    pcl.recognize(gc_clusterer, rototranslations, clustered_corrs)

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

    pcl.run(viewer)
end
