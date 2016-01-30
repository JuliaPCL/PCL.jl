using PCL
using Cxx

pcd_file = Pkg.dir("PCL", "test", "data", "table_scene_lms400.pcd")
cloud = pcl.PointCloud{pcl.PointXYZ}(pcd_file)
cloud_filtered = pcl.PointCloud{pcl.PointXYZ}()

sor = pcl.StatisticalOutlierRemoval{pcl.PointXYZ}()
pcl.setInputCloud(sor, cloud)
pcl.setMeanK(sor, 50)
pcl.setStddevMulThresh(sor, 1.0)
pcl.filter(sor, cloud_filtered)

viewer = pcl.PCLVisualizer("pcl visualizeer", create_interactor=false)
@assert !pcl.hasInteractor(viewer)
pcl.setOffScreenRendering(viewer, true)

red_handler = pcl.PointCloudColorHandlerCustom(cloud, 255, 0, 0)
green_handler = pcl.PointCloudColorHandlerCustom(cloud, 0, 255, 0)
pcl.addPointCloud(viewer, cloud, red_handler, id="cloud")
pcl.addPointCloud(viewer, cloud_filtered, green_handler, id="cloud_filtered")

dst_filename = string(tempname() |> x -> split(x, '/')[end], ".png")
pcl.renderToPng(viewer, dst_filename)
@assert isfile(dst_filename)

println("renderToPng: $dst_filename was created")
rm(dst_filename)
println("$dst_filename was sucessfully removed")
