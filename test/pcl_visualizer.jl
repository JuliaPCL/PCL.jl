doc = """PCL visualizer

Usage:
  pcl_visualizer.jl <pcd_file> [options]
  pcl_visualizer.jl -h | --help
  pcl_visualizer.jl --version

Options:
  -h --help       Show this screen.
  --version       Show version.
  --point_type=T  Point type [default: PointXYZ]
"""

using DocOpt
using PCL
using Cxx

let
    args = docopt(doc, version=v"0.0.1")
    pcd_file = args["<pcd_file>"]

    # e.g. pcl.PointXYZ
    T = eval(Expr(:., :pcl, QuoteNode(Symbol(args["--point_type"]))))

    cloud = pcl.PointCloud{T}(pcd_file)

    viewer = pcl.PCLVisualizer("pcl visualizer")
    pcl.addPointCloud(viewer, cloud)

    pcl.run(viewer)
end
