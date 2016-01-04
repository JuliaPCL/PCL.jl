using PCL
using Cxx
using Base.Test

milk_cartoon_path = joinpath(dirname(@__FILE__), "data",
    "milk_cartoon_all_small_clorox.pcd")

@testset "pcl::common" begin
    cloudxyz = pcl.PointCloud{pcl.PointXYZ}()
    @test (@cxx (cloudxyz.handle)->get()) != C_NULL
    cloudxyzi = pcl.PointCloud{pcl.PointXYZI}()
    @test (@cxx (cloudxyzi.handle)->get()) != C_NULL

    milk_cloud = pcl.PointCloud{pcl.PointXYZ}(milk_cartoon_path)
    @test (@cxx (milk_cloud.handle)->get()) != C_NULL
end

@testset "pcl::io" begin
    cloudxyz = pcl.PointCloud{pcl.PointXYZ}()
    @test pcl.loadPCDFile(milk_cartoon_path, cloudxyz) == 0
    cloud_xyzrgb = pcl.PointCloud{pcl.PointXYZRGB}()
    @test pcl.loadPCDFile(milk_cartoon_path, cloud_xyzrgb) == 0
end
