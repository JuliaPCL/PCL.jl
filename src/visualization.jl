### visualization ###

abstract PointCloudColorHandler

@defpcltype(PointCloudColorHandlerRGBField{T} <: PointCloudColorHandler,
    "pcl::visualization::PointCloudColorHandlerRGBField")

function call{T}(::Type{PointCloudColorHandlerRGBField{T}}, cloud::PointCloud)
    handle = @sharedptr(
        "pcl::visualization::PointCloudColorHandlerRGBField<\$T>",
        "\$(cloud.handle)")
    PointCloudColorHandlerRGBField(handle)
end

call{T}(::Type{PointCloudColorHandlerRGBField}, cloud::PointCloud{T}) =
    PointCloudColorHandlerRGBField{T}(cloud)

@defpcltype(PointCloudColorHandlerCustom{T} <: PointCloudColorHandler,
    "pcl::visualization::PointCloudColorHandlerCustom")

function call{T}(::Type{PointCloudColorHandlerCustom{T}}, cloud::PointCloud,
    r, g, b)
    handle = @sharedptr(
        "pcl::visualization::PointCloudColorHandlerCustom<\$T>",
        "\$(cloud.handle), \$r, \$g, \$b")
    PointCloudColorHandlerCustom(handle)
end

call{T}(::Type{PointCloudColorHandlerCustom}, cloud::PointCloud{T}, r, g, b) =
    PointCloudColorHandlerCustom{T}(cloud, r, g, b)

const CxxPCLVisualizerPtr =
    cxxt"boost::shared_ptr<pcl::visualization::PCLVisualizer>"

type PCLVisualizer
    handle::CxxPCLVisualizerPtr
    PCLVisualizer(handle::CxxPCLVisualizerPtr) = new(handle)
end

@inline handle(viewer::PCLVisualizer) = viewer.handle

function PCLVisualizer(name::AbstractString=""; create_interactor::Bool=true)
    handle = @sharedptr("pcl::visualization::PCLVisualizer",
        "\$(pointer(name)), \$create_interactor")
    PCLVisualizer(handle)
end

setBackgroundColor(viewer::PCLVisualizer, x, y, z) =
    icxx"$(handle(viewer))->setBackgroundColor($x, $y, $z);"
addCoordinateSystem(viewer::PCLVisualizer, scale) =
    icxx"$(handle(viewer))->addCoordinateSystem($scale);"
addCoordinateSystem(viewer::PCLVisualizer, scale, x, y, z) =
    icxx"$(handle(viewer))->addCoordinateSystem($scale, $x, $y, $z);"
spinOnce(viewer::PCLVisualizer, spin=1) =
    icxx"$(handle(viewer))->spinOnce($spin);"

# Generally, you don't have to chagne camera parameters manually. This would be
# useful for off-screen rendering in paricular.
function setCameraPosition(viewer::PCLVisualizer,
        pos_x, pos_y, pos_z,
        up_x, up_y, up_z; viewport::Integer=0)
    icxx"$(handle(viewer))->setCameraPosition(
        $pos_x, $pos_y, $pos_z, $up_x, $up_y, $up_z, $viewport);"
end
function setCameraPosition(viewer::PCLVisualizer,
        pos_x, pos_y, pos_z,
        view_x, view_y, view_z,
        up_x, up_y, up_z; viewport::Integer=0)
    icxx"$(handle(viewer))->setCameraPosition(
        $pos_x, $pos_y, $pos_z, $view_x, $view_y, $view_z,
        $up_x, $up_y, $up_z, $viewport);"
end
function setCameraClipDistances(viewer::PCLVisualizer, near, far;
    viewport::Integer=0)
    icxx"$(handle(viewer))->setCameraClipDistances($near, $far, $viewport);"
end

import Base: close

for f in [
        :initCameraParameters,
        :wasStopped,
        :removeAllPointClouds,
        :removeAllShapes,
        :removeAllCoordinateSystems,
        :resetStoppedFlag,
        :close,
        :updateCamera,
        :resetCamera,
        :spin,
        ]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(handle(viewer))->$f();")
    @eval $f(viewer::PCLVisualizer) = $body
end
setShowFPS(viewer::PCLVisualizer, v::Bool) =
    @cxx cxxpointer(handle(viewer))->setShowFPS(v)

function addPointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T};
    id::AbstractString="cloud", viewport::Int=0)
    icxx"$(handle(viewer))->addPointCloud($(handle(cloud)), $(pointer(id)),
        $viewport);"
end

function addPointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T},
    color_handler::PointCloudColorHandler;
    id::AbstractString="cloud", viewport::Int=0)
    icxx"$(handle(viewer))->addPointCloud($(handle(cloud)),
        *$(handle(color_handler)), $(pointer(id)), $viewport);"
end

function updatePointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T};
    id::AbstractString="cloud")
    icxx"$(handle(viewer))->updatePointCloud($(handle(cloud)),
        $(pointer(id)));"
end

function updatePointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T},
    color_handler::PointCloudColorHandler; id::AbstractString="cloud")
    icxx"$(handle(viewer))->updatePointCloud($(handle(cloud)),
        *$(handle(color_handler)), $(pointer(id)));"
end

function removePointCloud(viewer::PCLVisualizer;
        id::AbstractString="cloud", viewport::Int=0)
    icxx"$(handle(viewer))->removePointCloud($(pointer(id)), $viewport);"
end

function run(viewer::PCLVisualizer; spin::Int=1, sleep::Int=100000)
    icxx"""
    while (!$(viewer.handle)->wasStopped()) {
        $(viewer.handle)->spinOnce($spin);
        boost::this_thread::sleep(boost::posix_time::microseconds($sleep));
    }
    """
end

### For off-screen rendering ###

cxx"""
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
"""

cxx"""
namespace vis {

int renderToPng(vtkSmartPointer<vtkRenderWindow> &renderWindow,
                const char *filename) {
  renderWindow->Render();

  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
      vtkSmartPointer<vtkWindowToImageFilter>::New();
  windowToImageFilter->SetInput(renderWindow);
  windowToImageFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(filename);
  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
  writer->Write();
  return 0;
}

} // end namespace vis
"""

getRenderWindow(viewer::PCLVisualizer) =
    icxx"$(viewer.handle)->getRenderWindow();"
# NOTE: I had to access a protected  member of PCLVisualizer  `interactor_`
hasInteractor(viewer::PCLVisualizer) =
    icxx"$(viewer.handle)->interactor_ != NULL;"

function setOffScreenRendering(viewer::PCLVisualizer, v::Bool)
    if hasInteractor(viewer) && v
        error("Shouldn't have interactor for off screeen rendering")
    end
    icxx"$(getRenderWindow(viewer))->SetOffScreenRendering($v);"
end

renderToPng(viewer::PCLVisualizer, s::AbstractString) =
    @cxx vis::renderToPng(getRenderWindow(viewer), pointer(s))


cxx"""
namespace vis {

std::vector<unsigned char>
renderToVec(vtkSmartPointer<vtkRenderWindow> &renderWindow) {
  renderWindow->Render();

  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
      vtkSmartPointer<vtkWindowToImageFilter>::New();
  windowToImageFilter->SetInput(renderWindow);
  windowToImageFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetWriteToMemory(1);
  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
  writer->Write();

  auto rawPngBuffer = writer->GetResult();
  auto rawPointer = rawPngBuffer->GetPointer(0);
  auto total_size =
      rawPngBuffer->GetDataSize() * rawPngBuffer->GetDataTypeSize();
  std::vector<unsigned char> buffer(rawPointer, rawPointer + total_size);

  return buffer;
}

} // end namespace vis
"""

# > v = renderedData(viewer)
# > display("text/html", "<img src=data:image/png;base64,$(base64encode(v))>")
# should display image in a jupyter notebook
function renderedData(viewer::PCLVisualizer)
    vec = @cxx vis::renderToVec(getRenderWindow(viewer))
    p = icxx"&$(vec[0]);"
    pointer_to_array(p, length(vec))
end

# just for debugging: to be removed
cxx"""
namespace vis {

void dumpCameraParameters(pcl::visualization::PCLVisualizer::Ptr &vis) {
  std::vector<pcl::visualization::Camera> cameras;
  vis->getCameras(cameras);
  for (size_t i = 0; i < cameras.size(); ++i) {
    auto &c = cameras[i];
    std::cout << "[Camera #" << i << "]" << std::endl;
    std::cout << "focal : " << c.focal[0] << " " << c.focal[1] << " "
              << c.focal[2] << std::endl;
    std::cout << "pos : " << c.pos[0] << " " << c.pos[1] << " " << c.pos[2]
              << std::endl;
    std::cout << "view : " << c.view[0] << " " << c.view[1] << " " << c.view[2]
              << std::endl;
    std::cout << "clip : " << c.clip[0] << " " << c.clip[1] << std::endl;
    std::cout << "fovy : " << c.fovy << std::endl;
    std::cout << "window_size : " << c.window_size[0] << " " << c.window_size[1]
              << std::endl;
    std::cout << "window_pos : " << c.window_pos[0] << " " << c.window_pos[1]
              << std::endl;
  }
}

} // end namespace vis
"""
dumpCameraParameters(viewer::PCLVisualizer) =
    @cxx vis::dumpCameraParameters(viewer)
