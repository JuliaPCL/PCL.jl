for enumname in [
    :SACMODEL_PLANE,
    :SACMODEL_LINE,
    :SACMODEL_CIRCLE2D,
    :SACMODEL_CIRCLE3D,
    :SACMODEL_SPHERE,
    :SACMODEL_CYLINDER,
    :SACMODEL_CONE,
    :SACMODEL_TORUS,
    :SACMODEL_PARALLEL_LINE,
    :SACMODEL_PERPENDICULAR_PLANE,
    :SACMODEL_PARALLEL_LINES,
    :SACMODEL_NORMAL_PLANE,
    :SACMODEL_NORMAL_SPHERE,
    :SACMODEL_REGISTRATION,
    :SACMODEL_REGISTRATION_2D,
    :SACMODEL_PARALLEL_PLANE,
    :SACMODEL_NORMAL_PARALLEL_PLANE,
    :SACMODEL_STICK
    ]
    # build icxx expr
    ex = Expr(:macrocall, symbol("@icxx_str"), string("pcl::", enumname, ";"))
    @eval begin
        enumtyp = $ex
        isa(enumtyp, Cxx.CppEnum)
        global $enumname = enumtyp.val
    end
end

for intname in [
    :SAC_RANSAC,
    :SAC_LMEDS,
    :SAC_MSAC,
    :SAC_RRANSAC,
    :SAC_RMSAC,
    :SAC_MLESAC,
    :SAC_PROSAC,
    ]
    ex = Expr(:macrocall, symbol("@icxx_str"), string("pcl::", intname, ";"))
    @eval begin
        global $intname = $ex
        @assert isa($intname, Cint)
    end
end
