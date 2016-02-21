### pcl::search ###

VERBOSE && info("Include pcl::search headers")
@timevb cxx"""
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
"""

abstract AbstractSearch

@defpcltype KdTree{T} <: AbstractSearch "pcl::search::KdTree"
@defptrconstructor KdTree{T}() "pcl::search::KdTree"
@defconstructor KdTreeVal{T}() "pcl::search::KdTree"
