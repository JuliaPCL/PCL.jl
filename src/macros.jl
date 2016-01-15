function remove_type_annotation(e)
    if isa(e, Symbol)
        return e
    elseif isa(e, Expr)
        return remove_type_annotation(e.args[1])
    else
        @assert false
    end
end

function remove_type_annotation(args::Vector)
    ret_args = Any[]
    for e in args
        push!(ret_args, remove_type_annotation(e))
    end
    ret_args
end

macro sharedptr(name, args...)
    @assert length(args) <= 1
    if length(args) > 0 && args[1] != nothing
        Expr(:macrocall, symbol("@icxx_str"), """
        boost::shared_ptr<$name>(new $name($(args...)));""")
    else
        Expr(:macrocall, symbol("@icxx_str"), """
        boost::shared_ptr<$name>(new $name());""")
    end
end

"""
A macro to define PCL convenient types

e.g. `@defpcltype PointCloud{T}` defines:

- PointCloudPtr{T}: boost shared pointer of pcl::PointCloud (i.e.
  pcl::PointCloud::Ptr) wrapper
- PointCloudVal{T}: pcl::PointCloud value wrapper
- PointCloud{T}: type aliased to PointCloudPtr
- pclPointCloudPtr: pcl::PointCloud::Ptr
- pclPointCloudVal: pcl::PointCloud
"""
macro defpcltype(expr, cxxname)
    if isa(expr, Expr) && expr.head == :comparison
        has_supertype = true
        jlname = expr.args[1]
        @assert expr.args[2] == :(<:)
        super_name = expr.args[3]
    else
        has_supertype = false
        jlname = expr
    end

    # build names
    if isa(jlname, Expr) && jlname.head == :curly
        jlname_noparams = jlname.args[1]
        jlname_noparams_ptr = symbol(jlname_noparams, :Ptr)
        jlname_ptr = copy(jlname)
        jlname_noparams_val = symbol(jlname_noparams, :Val)
        jlname_ptr.args[1] = jlname_noparams_ptr
        jlname_val = copy(jlname)
        jlname_val.args[1] = jlname_noparams_val
        pclname_ptr = copy(jlname_ptr)
        pclname_ptr.args[1] = symbol(:pcl, pclname_ptr.args[1])
        pclname_val = copy(jlname_val)
        pclname_val.args[1] = symbol(:pcl, pclname_val.args[1])
    else
        jlname_noparams = jlname
        jlname_noparams_ptr = symbol(jlname_noparams, :Ptr)
        jlname_ptr = jlname_noparams_ptr
        jlname_noparams_val = symbol(jlname_noparams, :Val)
        jlname_val = jlname_noparams_val
        pclname_ptr = symbol(:pcl, jlname_ptr)
        pclname_val = symbol(:pcl, jlname_val)
    end

    # build cxxt str
    if isa(jlname, Expr) && jlname.head == :curly
        type_params = jlname.args[2:end]
        esc_type_params = map(x -> string("\$", x), type_params)
        cxxtstr_body = string(cxxname, "<", join(esc_type_params, ','), ">")
    else
        cxxtstr_body = string(cxxname)
    end
    cxxtstr_ptr_body = string("boost::shared_ptr<", cxxtstr_body, ">")
    cxxptrtype = Expr(:macrocall, symbol("@cxxt_str"), cxxtstr_ptr_body)
    cxxvaltype = Expr(:macrocall, symbol("@cxxt_str"), cxxtstr_body)

    # type body
    ptrtype_body = Expr(:(::), :handle, cxxptrtype)
    valtype_body = Expr(:(::), :handle, cxxvaltype)

    if has_supertype
        ptrtype_def = Expr(:comparison, jlname_ptr, :(:<), super_name)
        valtype_def = Expr(:comparison, jlname_val, :(:<), super_name)
    else
        ptrtype_def = jlname_ptr
        valtype_def = jlname_val
    end

    typdef = has_supertype ? quote
        type $jlname_ptr <: $super_name
            $ptrtype_body
        end

        type $jlname_val <: $super_name
            $valtype_body
        end
    end : quote
        type $jlname_ptr
            $ptrtype_body
        end

        type $jlname_val
            $valtype_body
        end
    end

    typaliases = quote
        typealias $jlname $jlname_ptr
        typealias $pclname_ptr $cxxptrtype
        typealias $pclname_val $cxxvaltype
    end

    def = esc(quote
        $typdef
        $typaliases
        has_sharedptr(x::$jlname_noparams_ptr) = true
        has_sharedptr(x::$jlname_noparams_val) = false
        handle(x::$jlname_noparams_ptr) = x.handle
        handle(x::$jlname_noparams_val) = x.handle
    end)

    # @show def
    return def
end

macro defptrconstructor(expr, cxxname)
    _defconstructor_impl(expr, cxxname, true)
end

macro defconstructor(expr, cxxname)
    _defconstructor_impl(expr, cxxname, false)
end

function _defconstructor_impl(expr::Expr, cxxname, is_sharedptr::Bool)
    @assert expr.head == :call
    typname = expr.args[1]

    if isa(typname, Expr) && typname.head == :curly
        type_params = typname.args[2:end]
        esc_type_params = map(x -> string("\$", x), type_params)
        cxxconstructor_def = string(cxxname, "<",
            join(esc_type_params, ','), ">")
    else
        type_params = nothing
        cxxconstructor_def = string(cxxname)
    end

    cxxconstructor_args = ""
    if length(expr.args) > 1
        simplified_args = remove_type_annotation(expr.args[2:end])
        esc_args = map(x -> string("\$", x), simplified_args)
        cxxconstructor_args = join(esc_args, ',')
    end

    # Function args
    fargs = length(expr.args) > 1 ? expr.args[2:end] : nothing
    fargs = Any[Expr(:(::), Expr(:curly, :Type, typname))]
    if length(expr.args) > 1
        for e in expr.args[2:end]
            push!(fargs, e)
        end
    end

    # build shared pointer or value instantiation expr
    cxxvalnew = Expr(:macrocall, symbol("@icxx_str"),
        string(cxxconstructor_def, "(", cxxconstructor_args, ");"))
    handledef = is_sharedptr ? quote
        @sharedptr $cxxconstructor_def $cxxconstructor_args
    end : cxxvalnew

    typname_no_params = isa(typname, Symbol) ? typname : typname.args[1]

    # Function body
    body = quote
        handle = $handledef
        $(typname_no_params)(handle)
        # $(typname)(handle)
    end

    if type_params != nothing
        type_params = map(esc, type_params)
        callexpr = Expr(:call,
            Expr(:curly, Expr(:., :Base, QuoteNode(:call)), type_params...),
            fargs...)
    else
        callexpr = Expr(:call, Expr(:., :Base, QuoteNode(:call)), fargs...)
    end

    # Function definition
    def = Expr(:function, callexpr, Expr(:block, body))
    # @show def

    return def
end
