using PCL
using Cxx

PT = pcl.PointXYZRGB
kumamon = pcl.PointCloud{PT}("kumamon.pcd")
@show length(kumamon)

kumamon_zfiltered = pcl.PointCloud{PT}()
kumamon_xfiltered = pcl.PointCloud{PT}()
kumamon_yfiltered = pcl.PointCloud{PT}()

zpass = pcl.PassThrough{pcl.PointXYZRGB}()
pcl.setInputCloud(zpass, kumamon)
pcl.setFilterFieldName(zpass, "z")
pcl.setFilterLimits(zpass, 0.0, 1.5)
filter(zpass, kumamon_zfiltered)
@show length(kumamon_zfiltered)

xpass = pcl.PassThrough{pcl.PointXYZRGB}()
pcl.setInputCloud(xpass, kumamon_zfiltered)
pcl.setFilterFieldName(xpass, "x")
pcl.setFilterLimits(xpass, -0.4, 0.4)
filter(xpass, kumamon_xfiltered)
@show length(kumamon_xfiltered)

ypass = pcl.PassThrough{pcl.PointXYZRGB}()
pcl.setInputCloud(ypass, kumamon_xfiltered)
pcl.setFilterFieldName(ypass, "y")
pcl.setFilterLimits(ypass, 0, 0.23)
filter(ypass, kumamon_yfiltered)
@show length(kumamon_yfiltered)
