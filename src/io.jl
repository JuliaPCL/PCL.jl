### io ###

function loadPCDFile{T}(s::AbstractString, cloud::PointCloud{T})
    ret = @cxx pcl::io::loadPCDFile(pointer(s), cxxderef(handle(cloud)))
    if ret != 0
        error("failed to load PCD file: code $ret")
    end
    ret
end

function savePCDFile{T}(s::AbstractString, cloud::PointCloud{T};
        binary_mode=false)
    ret = @cxx pcl::io::savePCDFile(pointer(s), cxxderef(handle(cloud)))
    if ret != 0
        error("failed to save PCD file: code $ret")
    end
    ret
end
