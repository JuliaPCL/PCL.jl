### octree ###

VERBOSE && info("Include pcl::octree headers")
@timevb cxx"""
#include <pcl/search/octree.h>
"""

abstract AbstractOctree

@defpcltype Octree{T} <: AbstractOctree "pcl::search::Octree"
@defptrconstructor Octree{T}(v::AbstractFloat) "pcl::search::Octree"
@defconstructor OctreeVal{T}(v::AbstractFloat) "pcl::search::Octree"
