using PCL
using Cxx

addHeaderDir("/usr/local/include/ni2/", kind=C_System)

cxx"""
#include <iostream>

#include <mutex>
#include <pcl/io/openni2_grabber.h>
"""

if !isdefined(:has_SimpleOpenNIViewer)
cxx"""
typedef pcl::PointXYZ T;

class SimpleOpenNIViewer {
public:
  SimpleOpenNIViewer()
      : viewer(new pcl::visualization::PCLVisualizer("PCL OpenNI Viewer")) {}

  void cloud_cb_(const pcl::PointCloud<T>::ConstPtr &cloud) {
    if (!viewer->wasStopped()) {
      mtx.lock();
      viewer->updatePointCloud<T>(cloud, "openni");
      mtx.unlock();
    }
  }

  void run() {
    pcl::Grabber *interface = new pcl::io::OpenNI2Grabber(
        "", pcl::io::OpenNI2Grabber::OpenNI_QVGA_30Hz,
        pcl::io::OpenNI2Grabber::OpenNI_QVGA_30Hz);

    pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
    viewer->addPointCloud<T>(cloud, "openni", 0);

    boost::function<void(const pcl::PointCloud<T>::ConstPtr &)> f =
        boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

    interface->registerCallback(f);

    interface->start();

    while (!viewer->wasStopped()) {
      boost::this_thread::sleep(boost::posix_time::millisec(50));
      mtx.lock();
      viewer->spinOnce();
      mtx.unlock();
    }

    interface->stop();
  }

  std::mutex mtx;
  pcl::visualization::PCLVisualizer::Ptr viewer;
};
"""
end

const has_SimpleOpenNIViewer = true

# viewer = @cxxnew SimpleOpenNIViewer()
# @cxx viewer->run()
