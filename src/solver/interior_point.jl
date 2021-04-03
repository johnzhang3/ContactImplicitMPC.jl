abstract type LinearSolver end
abstract type Cache end

# check that inequality constraints are satisfied
function inequality_check(x, idx_ineq)
    for i in idx_ineq
        if x[i] <= 0.0
            return true
        end
    end
    return false
end

struct NoCache <: Cache end
# abstract type rCache <: Cache end
# abstract type rzCache <: Cache end
# abstract type rθCache <: Cache end

# residual
function r!(r, z, θ, κ, cache)
    @warn "residual not defined"
    nothing
end

# residual Jacobian wrt z
function rz!(rz, z, θ, cache)
    @warn "residual Jacobian wrt z not defined"
    nothing
end

# residual Jacobian wrt θ
function rθ!(rθ, z, θ, cache)
    @warn "residual Jacobian wrt θ not defined"
    nothing
end

# interior-point solver options
@with_kw mutable struct InteriorPointOptions{T}
    r_tol::T = 1.0e-5
    κ_tol::T = 1.0e-5
    κ_init::T = 1.0
    κ_scale::T = 0.1
    ls_scale::T = 0.5
    max_iter_inner::Int = 100
    max_iter_outer::Int = 10
    max_ls::Int = 50
    max_time::T = 60.0
    diff_sol::Bool = false
    res_norm::Real = Inf
    reg::Bool = false
    reg_pr_init = 0.0
    reg_du_init = 0.0
    solver::Symbol = :lu_solver
end

mutable struct ResidualMethods
    r!
    rz!
    rθ!
end

# regularize Jacobian / Hessian
function regularize!(v_pr, v_du, reg_pr, reg_du)
    v_pr .+= reg_pr
    v_du .-= reg_du
end

mutable struct InteriorPoint{T}
    methods::ResidualMethods
    z::Vector{T}               # current point
    z̄::Vector{T}               # candidate point
    r#::Vector{T}               # residual
    r_norm::T                  # residual norm
    r̄#::Vector{T}               # candidate residual
    r̄_norm::T                  # candidate residual norm
    rz#::SparseMatrixCSC{T,Int} # residual Jacobian wrt z
    rθ#::SparseMatrixCSC{T,Int} # residual Jacobian wrt θ
    Δ::Vector{T}               # search direction
    idx_ineq::Vector{Int}      # indices for inequality constraints
    idx_pr::Vector{Int}        # indices for primal variables
    idx_du::Vector{Int}        # indices for dual variables
    δz::Matrix{T}              # solution gradients (this is always dense)
    θ::Vector{T}               # problem data
    κ::Vector{T}               # barrier parameter
    num_var::Int
    num_data::Int
    solver::LinearSolver
    v_pr
    v_du
    reg_pr
    reg_du
    r_cache::Cache
    r̄_cache::Cache
    rz_cache::Cache
    rθ_cache::Cache
    opts::InteriorPointOptions
end

function interior_point(x, θ;
        num_var = length(x),
        num_data = length(θ),
        idx_ineq = collect(1:0),
        idx_pr = collect(1:num_var),
        idx_du = collect(1:0),
        r! = r!, rz! = rz!, rθ! = rθ!,#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        # r_! = r!, rz_! = rz!, rθ_! = rθ!,#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        r  = zeros(num_var), #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        rz = spzeros(num_var, num_var),
        rθ = spzeros(num_var, num_data),
        reg_pr = [0.0], reg_du = [0.0],
        r_cache = NoCache(),
        r̄_cache = NoCache(),
        rz_cache = NoCache(),
        rθ_cache = NoCache(),
        v_pr = view(rz, CartesianIndex.(idx_pr, idx_pr)),
        v_du = view(rz, CartesianIndex.(idx_du, idx_du)),
        opts = InteriorPointOptions()) where T

    rz!(rz, x, θ, rz_cache) # compute Jacobian for pre-factorization

    InteriorPoint(
        ResidualMethods(r!, rz!, rθ!),
        # ResidualMethods(r_!, rz_!, rθ_!),#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        x,
        zeros(num_var),
        r,
        0.0,
        deepcopy(r),
        0.0,
        rz,
        rθ,
        zeros(num_var),
        idx_ineq,
        idx_pr,
        idx_du,
        zeros(num_var, num_data),
        θ,
        zeros(1),
        num_var,
        num_data,
        eval(opts.solver)(rz),
        v_pr,
        v_du,
        reg_pr, reg_du,
        r_cache, r̄_cache, rz_cache, rθ_cache,
        opts)
end

# interior point solver
function interior_point!(ip::InteriorPoint{T}) where T

    # methods
    r! = ip.methods.r!
    rz! = ip.methods.rz!
    rθ! = ip.methods.rθ!

    # options
    opts = ip.opts
    r_tol = opts.r_tol
    κ_tol = opts.κ_tol
    κ_init = opts.κ_init
    κ_scale = opts.κ_scale
    ls_scale = opts.ls_scale
    max_iter_inner = opts.max_iter_inner
    max_iter_outer = opts.max_iter_outer
    max_time = opts.max_time
    max_ls = opts.max_ls
    diff_sol = opts.diff_sol
    res_norm = opts.res_norm
    reg = opts.reg

    # unpack pre-allocated data
    z = ip.z
    z̄ = ip.z̄
    r = ip.r
    r_norm = ip.r_norm
    r̄ = ip.r̄
    r̄_norm = ip.r̄_norm
    rz = ip.rz
    Δ = ip.Δ
    idx_ineq = ip.idx_ineq
    θ = ip.θ
    κ = ip.κ
    v_pr = ip.v_pr
    v_du = ip.v_du
    reg_pr = ip.reg_pr
    reg_du = ip.reg_du
    r_cache = ip.r_cache
    r̄_cache = ip.r̄_cache
    rz_cache = ip.rz_cache
    solver = ip.solver

    # initialize barrier parameter
    κ[1] = κ_init

    # initialize regularization
    reg_pr[1] = opts.reg_pr_init
    reg_du[1] = opts.reg_du_init

    # compute residual, residual Jacobian
    r!(r, z, θ, κ[1], r_cache)
    r_norm = norm(r, res_norm)

    elapsed_time = 0.0

    for i = 1:max_iter_outer
        elapsed_time >= max_time && break
        for j = 1:max_iter_inner
            elapsed_time >= max_time && break
            elapsed_time += @elapsed begin
                # check for converged residual
                if r_norm < r_tol
                    break
                end

                # compute residual Jacobian
                rz!(rz, z, θ, rz_cache)

                # regularize (fixed, TODO: adaptive)
                reg && regularize!(v_pr, v_du, reg_pr[1], reg_du[1])

                # compute step
                linear_solve!(solver, Δ, rz, r)

                # initialize step length
                α = 1.0

                # candidate point
                z̄ .= z - α * Δ

                # check inequality constraints
                iter = 0
                while inequality_check(z̄, idx_ineq)
                    α *= ls_scale
                    z̄ .= z - α * Δ
                    iter += 1
                    if iter > max_ls
                        @error "backtracking line search fail"
                        return false
                    end
                end

                # reduce norm of residual
                r!(r̄, z̄, θ, κ[1], r̄_cache)
                r̄_norm = norm(r̄, res_norm)

                while r̄_norm >= (1.0 - 0.001 * α) * r_norm
                    α *= ls_scale
                    z̄ .= z - α * Δ

                    r!(r̄, z̄, θ, κ[1], r̄_cache)
                    r̄_norm = norm(r̄, Inf)

                    iter += 1
                    if iter > max_ls
                        @error "line search fail"
                        return false
                    end
                end

                # update
                z .= z̄
                # r .= r̄ #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
                r!(r, z, θ, κ[1], r_cache)
                # v_pr .= 0.0
                # v_du .= 0.0
                r_norm = r̄_norm
            end
        end

        if κ[1] <= κ_tol
            # differentiate solution
            diff_sol && differentiate_solution!(ip)
            return true
        else
            # update barrier parameter
            κ[1] *= κ_scale

            # update residual
            r!(r, z, θ, κ[1], r_cache)
            r_norm = norm(r, res_norm)
        end
    end
end

function interior_point!(ip::InteriorPoint{T}, z::AbstractVector{T}, θ::AbstractVector{T}) where T
    ip.z .= z
    ip.θ .= θ
    interior_point!(ip)
end

function differentiate_solution!(ip::InteriorPoint)
    z = ip.z
    θ = ip.θ
    rz = ip.rz
    rθ = ip.rθ
    δz = ip.δz
    rz_cache = ip.rz_cache
    rθ_cache = ip.rθ_cache
    κ = ip.κ

    ip.methods.rz!(rz, z, θ, rz_cache) # maybe not needed
    ip.methods.rθ!(rθ, z, θ, rθ_cache)

    linear_solve!(ip.solver, δz, rz, rθ)
    @inbounds @views @. ip.δz .*= -1.0
    nothing
end

linear_solve!(solver::LinearSolver, x::Vector{T}, A::Array{T, 2}, b::Vector{T}) where T = linear_solve!(solver, x, sparse(A), b)
