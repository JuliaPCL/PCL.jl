### io ###

function loadPCDFile{T}(s::AbstractString, cloud::PointCloud{T})
    ps = pointer(s)
    ret = icxx"pcl::io::loadPCDFile<$T>($ps, *$(cloud.handle));";
    if ret != 0
        error("failed to load PCD file: code $ret")
    end
    ret
end

function savePCDFile{T}(s::AbstractString, cloud::PointCloud{T};
        binary_mode=false)
    ps = pointer(s)
    ret = icxx"pcl::io::savePCDFile($ps, *$(cloud.handle), $binary_mode);";
    if ret != 0
        error("failed to save PCD file: code $ret")
    end
    ret
end
