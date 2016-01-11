### visualization ###

abstract PointCloudColorHandler

@inline handle(p::PointCloudColorHandler) = p.handle

type PointCloudColorHandlerRGBField{T} <: PointCloudColorHandler
    handle::SharedPtr
end

function call{T}(::Type{PointCloudColorHandlerRGBField{T}}, cloud::PointCloud)
    handle = @sharedptr(
        "pcl::visualization::PointCloudColorHandlerRGBField<\$T>",
        "\$(cloud.handle)")
    PointCloudColorHandlerRGBField{T}(handle)
end

call{T}(::Type{PointCloudColorHandlerRGBField}, cloud::PointCloud{T}) =
    PointCloudColorHandlerRGBField{T}(cloud)


type PointCloudColorHandlerCustom{T} <: PointCloudColorHandler
    handle::SharedPtr
end

function call{T}(::Type{PointCloudColorHandlerCustom{T}}, cloud::PointCloud,
    r, g, b)
    handle = @sharedptr(
        "pcl::visualization::PointCloudColorHandlerCustom<\$T>",
        "\$(cloud.handle), \$r, \$g, \$b")
    PointCloudColorHandlerCustom{T}(handle)
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

function PCLVisualizer(name::AbstractString="", create_interactor::Bool=true)
    handle = @sharedptr("pcl::visualization::PCLVisualizer",
        "\$(pointer(name)), \$create_interactor")
    PCLVisualizer(handle)
end

setBackgroundColor(viewer::PCLVisualizer, x, y, z) =
    @cxx cxxpointer(handle(viewer))->setBackgroundColor(x, y, z)
addCoordinateSystem(viewer::PCLVisualizer, n) =
    @cxx cxxpointer(handle(viewer))->addCoordinateSystem($n)
spinOnce(viewer::PCLVisualizer, spin=1) =
    @cxx cxxpointer(handle(viewer))->spinOnce(spin)
spin(viewer::PCLVisualizer) = @cxx cxxpointer(handle(viewer))->spin()

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
        ]
    @eval begin
        $f(viewer::PCLVisualizer) = @cxx cxxpointer(handle(viewer))->$f()
    end
end

function addPointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T};
    id::AbstractString="cloud", viewport::Int=0)
    icxx"""
        $(viewer.handle).get()->addPointCloud<$T>(
            $(cloud.handle), $(pointer(id)), $viewport);"""
end

function addPointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T},
    color_handler::PointCloudColorHandler;
    id::AbstractString="cloud", viewport::Int=0)
    icxx"""
        $(viewer.handle).get()->addPointCloud<$T>(
            $(cloud.handle), *$(color_handler.handle), $(pointer(id)),
            $viewport);"""
end

function updatePointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T};
    id::AbstractString="cloud")
    icxx"""
        $(viewer.handle).get()->updatePointCloud<$T>(
            $(cloud.handle), $(pointer(id)));"""
end

function updatePointCloud{T}(viewer::PCLVisualizer, cloud::PointCloud{T},
    color_handler::PointCloudColorHandler; id::AbstractString="cloud")
    icxx"""
        $(viewer.handle).get()->updatePointCloud<$T>(
            $(cloud.handle), *$(color_handler.handle), $(pointer(id)));"""
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
render_to_vec(vtkSmartPointer<vtkRenderWindow> &renderWindow) {
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
    vec = @cxx vis::render_to_vec(getRenderWindow(viewer))
    p = icxx"&$(vec[0]);"
    pointer_to_array(p, length(vec))
end
