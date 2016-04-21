# PCL

The package is re-organized into https://github.com/JuliaPCL to simplify development and minimize dependencies. Note that PCL packages are **much work in progress** and there's almost no docs. Please file an issue if you have any trouble or request for docs, etc. Currently only tested on OSX.

## Package organization

- JuliaPCL Package organization follows PCL module organization (see http://docs.pointclouds.org/trunk/modules.html). You can use any of them: e.g. `using PCLCommon, PCLIO, PCLVisualization` instead of `using PCL` (this is quite slow since it compiles all the PCL packages).
- A special package [LibPCL.jl](https://github.com/JuliaPCL/LibPCL.jl) handles binary dependency management and used by *every* PCL packages.


## Requirement

- Julia (master) with [Keno/Cxx.jl](https://github.com/Keno/Cxx.jl)
- PCL 1.8 (built as shared libraries)

## Installation

```jl
Pkg.clone("https://github.com/JuliaPCL/PCLCommon.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLIO.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLFeatures.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLFilters.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLVisualization.jl")
Pkg.clone("https://github.com/JuliaPCL/LibPCL.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLKDTree.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLOctree.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLRecognition.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLSampleConsensus.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLSearch.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLSegmentation.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLTracking.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLRegistration.jl")
Pkg.clone("https://github.com/JuliaPCL/PCLKeyPoints.jl")
Pkg.clone("https://github.com/JuliaPCL/PCL.jl")
```

```jl
Pkg.build("LibPCL")
```

## Why

[README.org.md](README.org.md)
