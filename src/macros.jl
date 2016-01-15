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

macro defconstructor(expr)
    @assert expr.head == :call
    typname = expr.args[1]
    # @show typname

    if typname.head == :curly
        params = typname.args[2:end]
        esc_params = map(x -> string("\$", x), params)
        sharedptr_body = "pcl::$(typname.args[1])<$(join(esc_params, ','))>"
        sharedptr_args = nothing
        if length(expr.args) > 1
            simplified_args = remove_type_annotation(expr.args[2:end])
            esc_args = map(x -> string("\$", x), simplified_args)
            sharedptr_args = join(esc_args, ',')
        end

        # Function args
        fargs = length(expr.args) > 1 ? expr.args[2:end] : nothing
        fargs = Any[Expr(symbol("::"), Expr(:curly, :Type, typname))]
        if length(expr.args) > 1
            for e in expr.args[2:end]
                push!(fargs, e)
            end
        end

        # Function body
        body = quote
            handle = @sharedptr $sharedptr_body $sharedptr_args
            $(typname.args[1])(handle)
            # $(typname)(handle)
        end
        # @show body

        params = map(esc, params)

        # Function definition
        def = Expr(:function, Expr(:call,
            Expr(:curly, Expr(:., :Base, QuoteNode(:call)), params...),
            fargs...), Expr(:block, body))
        # @show def
        return def
    else
        quote error("not implemented") end
    end
end
