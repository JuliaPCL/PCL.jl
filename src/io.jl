### io ###

for f in [
        :loadPCDFile,
        :loadOBJFile,
        :loadPLYFile,
        :load,
        ]
    ex = Expr(:macrocall, symbol("@icxx_str"),
        "pcl::io::$f(\$(pointer(s)), *\$(handle(cloud)));")
    @eval begin
        function $f{T}(s::AbstractString, cloud::PointCloud{T})
            ret = $ex
            if ret != 0
                error("failed to $f: code $ret")
            end
            ret
        end
    end
end

for f in [
        :savePCDFile,
        :saveOBJFile,
        :savePLYFile,
        ]
    ex = Expr(:macrocall, symbol("@icxx_str"),
        "pcl::io::$f(\$(pointer(s)), *\$(handle(cloud)), \$binary_mode);")
    @eval begin
        function $f{T}(s::AbstractString, cloud::PointCloud{T};
                binary_mode::Bool=true)
            ret = $ex
            if ret != 0
                error("failed to $f: code $ret")
            end
            ret
        end
    end
end
