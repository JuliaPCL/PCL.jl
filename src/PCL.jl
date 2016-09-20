__precompile__(false)

# ls -l ~/.julia/v0.6  | grep PCL | awk '{print $9}' | sort | grep -v "^PCL$"|\
#   awk '{print "-", "["$1"]""(https://github.com/JuliaPCL/"$1".jl)"}'
"""
The Julia wrapper for Point Cloud Library (PCL)

To simplify development and minimize dependencies, the Julia wrapper consists of
the packages below:

- [LibPCL](https://github.com/JuliaPCL/LibPCL.jl)
- [PCLCommon](https://github.com/JuliaPCL/PCLCommon.jl)
- [PCLFeatures](https://github.com/JuliaPCL/PCLFeatures.jl)
- [PCLFilters](https://github.com/JuliaPCL/PCLFilters.jl)
- [PCLIO](https://github.com/JuliaPCL/PCLIO.jl)
- [PCLKDTree](https://github.com/JuliaPCL/PCLKDTree.jl)
- [PCLKeyPoints](https://github.com/JuliaPCL/PCLKeyPoints.jl)
- [PCLOctree](https://github.com/JuliaPCL/PCLOctree.jl)
- [PCLRecognition](https://github.com/JuliaPCL/PCLRecognition.jl)
- [PCLRegistration](https://github.com/JuliaPCL/PCLRegistration.jl)
- [PCLSampleConsensus](https://github.com/JuliaPCL/PCLSampleConsensus.jl)
- [PCLSearch](https://github.com/JuliaPCL/PCLSearch.jl)
- [PCLSegmentation](https://github.com/JuliaPCL/PCLSegmentation.jl)
- [PCLSurface](https://github.com/JuliaPCL/PCLSurface.jl)
- [PCLTracking](https://github.com/JuliaPCL/PCLTracking.jl)
- [PCLVisualization](https://github.com/JuliaPCL/PCLVisualization.jl)

following the [PCL module structure](http://docs.pointclouds.org/trunk/modules.html)
except for the `LibPCL.jl` which manages binary dependencies (i.e. search
installed PCL shared libraries or build and install them if not found).
"""
module PCL

using Reexport
using DocStringExtensions

@reexport using PCLCommon
@reexport using PCLIO
@reexport using PCLFeatures
@reexport using PCLFilters
@reexport using PCLVisualization
@reexport using PCLKDTree
@reexport using PCLOctree
@reexport using PCLRecognition
@reexport using PCLSampleConsensus
@reexport using PCLSearch
@reexport using PCLSegmentation
@reexport using PCLTracking
@reexport using PCLRegistration
@reexport using PCLKeyPoints
@reexport using PCLSurface

end # module PCL
