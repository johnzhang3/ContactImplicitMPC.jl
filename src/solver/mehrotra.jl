# interior-point solver options
@with_kw mutable struct MehrotraOptions{T} <: AbstractIPOptions
    r_tol::T = 1.0e-5
    κ_tol::T = 1.0e-5
    κ_init::T = 1.0                   # useless
    # κ_scale::T = 0.1                  # useless
    # ls_scale::T = 0.5                 # useless
    max_iter_inner::Int = 100
    # max_iter_outer::Int = 1           # useless
    # max_ls::Int = 50                  # useless
    max_time::T = 60.0
    diff_sol::Bool = false
    res_norm::Real = Inf
    reg::Bool = false
    reg_pr_init = 0.0
    reg_du_init = 0.0
    ϵ_min = 0.05
    solver::Symbol = :lu_solver
    verbose::Bool = false
end

mutable struct Mehrotra{T} <: AbstractIPSolver
    s::Space
    methods::ResidualMethods
    z::Vector{T}                 # current point
    # z̄::Vector{T}                 # candidate point
    Δaff::Vector{T}              # affine search direction
    Δ::Vector{T}                 # corrector search direction
    r                            # residual
    rm                           # corrector residual
    rbil                         # corrector residual
    r_merit::T                   # residual norm
    r̄ #useless                            # candidate residual
    # r̄_merit::T                   # candidate residual norm
    rz                           # residual Jacobian wrt z
    rθ                           # residual Jacobian wrt θ
    idx_ineq::Vector{Int}        # indices for inequality constraints
    idx_soc::Vector{Vector{Int}} # indices for second-order cone constraints
    idx_pr::Vector{Int}          # indices for primal variables
    idx_du::Vector{Int}          # indices for dual variables
    δz::Matrix{T}                # solution gradients (this is always dense)
    δzs::Matrix{T}               # solution gradients (in optimization space; δz = δzs for Euclidean)
    θ::Vector{T}                 # problem data
    κ::Vector{T}                 # barrier parameter
    num_var::Int
    num_data::Int
    nbil::Int
    solver::LinearSolver
    v_pr # view
    v_du # view
    z_y1 # view into z corresponding to the first set of variables in the bilinear constraints y1 .* y2 = 0 (/!\this is z space)
    z_y2 # view into z corresponding to the second set of variables in the bilinear constraints y1 .* y2 = 0 (/!\this is z space)
    Δaff_y1 # view into Δaff corresponding to the first set of variables in the bilinear constraints y1 .* y2 = 0 (/!\this is Δ space)
    Δaff_y2 # view into Δaff corresponding to the second set of variables in the bilinear constraints y1 .* y2 = 0 (/!\this is Δ space)
    Δ_y1 # view into Δ corresponding to the first set of variables in the bilinear constraints y1 .* y2 = 0 (/!\this is Δ space)
    Δ_y2 # view into Δ corresponding to the second set of variables in the bilinear constraints y1 .* y2 = 0 (/!\this is Δ space)
    iy1
    iy2
    ibil
    reg_pr
    reg_du
    iterations::Int
    opts::MehrotraOptions
end

function mehrotra(z, θ;
        s = Euclidean(length(z)),
        num_var = length(z),
        num_data = length(θ),
        idx_ineq = collect(1:0),
        idx_soc = Vector{Int}[],
        idx_pr = collect(1:s.n),
        idx_du = collect(1:0),
        iy1 = collect(1:0),
        iy2 = collect(1:0),
        ibil = collect(1:0),
        r! = r!, rm! = rm!, rz! = rz!, rθ! = rθ!,
        r  = zeros(s.n),
        rm = deepcopy(r),
        rz = spzeros(s.n, s.n),
        rθ = spzeros(s.n, num_data),
        reg_pr = [0.0], reg_du = [0.0],
        v_pr = view(rz, CartesianIndex.(idx_pr, idx_pr)),
        v_du = view(rz, CartesianIndex.(idx_du, idx_du)),
        opts::MehrotraOptions = MehrotraOptions()) where T

    rz!(rz, z, θ) # compute Jacobian for pre-factorization

    # Search direction
    Δaff = zeros(s.n)
    Δ = zeros(s.n)

    # Indices
    nbil = length(iy1)
    iy1 = SVector{nbil, Int}(iy1)
    iy2 = SVector{nbil, Int}(iy2)
    ibil = SVector{nbil, Int}(ibil)
    nbil == 0 && @warn "nbil == 0, we will get NaNs during the Mehrotra solve."

    # Views
    z_y1 = view(z, iy1)
    z_y2 = view(z, iy2)
    Δaff_y1 = view(Δaff, iy1) # TODO this should be in Δ space
    Δaff_y2 = view(Δaff, iy2) # TODO this should be in Δ space
    Δ_y1 = view(Δ, iy1) # TODO this should be in Δ space
    Δ_y2 = view(Δ, iy2) # TODO this should be in Δ space
    rbil = bilinear_res(rm, ibil)

    Mehrotra(
        s,
        ResidualMethods(r!, rm!, rz!, rθ!),
        z,
        # zeros(length(z)),
        Δaff,
        Δ,
        r,
        rm, # rm
        rbil,
        0.0,
        deepcopy(r), #useless
        # 0.0,
        rz,
        rθ,
        idx_ineq,
        idx_soc,
        idx_pr,
        idx_du,
        zeros(length(z), num_data),
        zeros(s.n, num_data),
        θ,
        zeros(1),
        num_var,
        num_data,
        nbil,
        eval(opts.solver)(rz),
        v_pr,
        v_du,
        z_y1,
        z_y2,
        Δaff_y1,
        Δaff_y2,
        Δ_y1,
        Δ_y2,
        iy1,
        iy2,
        ibil,
        reg_pr, reg_du,
        0,
        opts)
end

function bilinear_res(r::AbstractVector, ibil)
    view(r, ibil)
end

# interior point solver
function interior_point_solve!(ip::Mehrotra{T}) where T

    # space
    s = ip.s

    # methods
    r! = ip.methods.r!
    rm! = ip.methods.rm!
    rz! = ip.methods.rz!
    rθ! = ip.methods.rθ!

    # options
    opts = ip.opts
    r_tol = opts.r_tol
    κ_tol = opts.κ_tol
    max_iter_inner = opts.max_iter_inner
    max_time = opts.max_time
    diff_sol = opts.diff_sol
    res_norm = opts.res_norm
    reg = opts.reg
    ϵ_min = opts.ϵ_min
    verbose = opts.verbose

    # unpack pre-allocated data
    z = ip.z
    # z̄ = ip.z̄
    Δaff = ip.Δaff
    Δ = ip.Δ
    r = ip.r
    rm = ip.rm
    rbil = ip.rbil
    nbil = ip.nbil
    r_merit = ip.r_merit
    # r̄ = ip.r̄
    # r̄_merit = ip.r̄_merit
    rz = ip.rz
    idx_ineq = ip.idx_ineq
    idx_soc = ip.idx_soc
    θ = ip.θ
    κ = ip.κ
    v_pr = ip.v_pr
    v_du = ip.v_du
    z_y1 = ip.z_y1
    z_y2 = ip.z_y2
    Δaff_y1 = ip.Δaff_y1
    Δaff_y2 = ip.Δaff_y2
    Δ_y1 = ip.Δ_y1
    Δ_y2 = ip.Δ_y2
    iy1 = ip.iy1
    iy2 = ip.iy2
    reg_pr = ip.reg_pr
    reg_du = ip.reg_du
    solver = ip.solver

    # initialize regularization
    reg_pr[1] = opts.reg_pr_init
    reg_du[1] = opts.reg_du_init

    # compute residual, residual Jacobian
    ip.methods.rm!(r, z, 0.0 .* Δaff, θ, 0.0) # here we set κ = 0, Δ = 0
    r_merit = norm(r, res_norm)

    elapsed_time = 0.0

    for j = 1:max_iter_inner
        elapsed_time >= max_time && break
        elapsed_time += @elapsed begin

            # @show j
            # @show norm(z)
            # @show norm(θ)
            # @show minimum(abs.(z))
            # plt = plot()
            # plot!(z)
            # display(plt)

            # check for converged residual
            if r_merit < r_tol
                break
            end
            ip.iterations += 1
            # compute residual Jacobian
            rz!(rz, z, θ)
            # @show norm(rz)

            # regularize (fixed, TODO: adaptive)
            reg && regularize!(v_pr, v_du, reg_pr[1], reg_du[1])

            # compute affine search direction
            linear_solve!(solver, Δaff, rz, r)
            @show norm(Δaff)
            # plt = plot()
            # plot!(Δaff)
            # display(plt)
            αaff = step_length(z_y1, z_y2, Δaff_y1, Δaff_y2, τ=1.0)
            μaff = (z_y1 - αaff * Δaff[iy1])' * (z_y2 - αaff * Δaff[iy2]) / nbil
            # @show nbil
            # @show norm(αaff)
            # @show norm(μaff)

            μ = z_y1'*z_y2 / length(z_y1)
            σ = (μaff / μ)^3

            # Compute corrector residual
            rm!(rm, z, Δaff, θ, σ*μ) # here we set κ = σ*μ, Δ = Δaff

            # Compute corrector search direction
            linear_solve!(solver, Δ, rz, rm)
            τ = progress(r_merit, ϵ_min=ϵ_min)
            α = step_length(z_y1, z_y2, Δ_y1, Δ_y2, τ=τ)

            # candidate point
            candidate_point!(z, s, z, Δ, α)

            # update
            r!(r, z, θ, 0.0) # we set κ= 0.0 to measure the bilinear constraint violation
            r_merit = norm(r, res_norm)
            verbose && println("iter: ", j, "   res∞: ", scn(r_merit))
        end
    end

    if r_merit > r_tol
        @error "Mehrotra solver failed to reduce residual below r_tol."
        return false
    end

    # differentiate solution
    diff_sol && differentiate_solution!(ip)
    return true
end

# TODO maybe we will need to implement this merit function to use κ_tol > b and r_tol > a
# function merit(rlin::AbstractVector, rbil::AbstractVector, t::Real)
# 	a = norm(rlin, t)
# 	b = norm(rbil, t)
# 	return a, b
# end

function progress(merit; ϵ_min=0.05)
    ϵ = min(ϵ_min, merit^2)
    τ = 1 - ϵ
    return τ
end

function step_length(w2::S, w3::S, Δw2::S, Δw3::S; τ::Real=0.9995) where {S}
    ατ_p = 1.0
    ατ_d = 1.0
    for i in eachindex(w2)
        if Δw2[i] > 0.0
            ατ_p = min(ατ_p, τ * w2[i] / Δw2[i])
        end
        if Δw3[i] > 0.0
            ατ_d = min(ατ_d, τ * w3[i] / Δw3[i])
        end
    end
    α = min(ατ_p, ατ_d)
    return α
end

function interior_point_solve!(ip::Mehrotra{T}, z::AbstractVector{T}, θ::AbstractVector{T}) where T
    ip.z .= z
    ip.θ .= θ
    interior_point_solve!(ip)
end

function differentiate_solution!(ip::Mehrotra)
    s = ip.s
    z = ip.z
    θ = ip.θ
    rz = ip.rz
    rθ = ip.rθ
    δz = ip.δz
    δzs = ip.δzs

    κ = ip.κ

    ip.methods.rz!(rz, z, θ)
    ip.methods.rθ!(rθ, z, θ)

    linear_solve!(ip.solver, δzs, rz, rθ)
    @inbounds @views @. δzs .*= -1.0
    mapping!(δz, s, δzs, z)

    nothing
end
