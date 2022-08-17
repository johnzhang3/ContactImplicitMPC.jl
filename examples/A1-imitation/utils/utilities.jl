
function save_to_jld2(model, x_sol, u_sol, gait, tol)
    q_opt = [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...]
    v_opt = [(x[model.nq .+ (1:model.nq)] - x[0 .+ (1:model.nq)]) ./ h for x in x_sol]
    u_opt = [u[1:model.nu] for u in u_sol]
    γ_opt = [u[model.nu .+ (1:4)] for u in u_sol]
    b_opt = [u[model.nu + 4 .+ (1:16)] for u in u_sol]
    ψ_opt = [u[model.nu + 4 + 16 .+ (1:4)] for u in u_sol]
    η_opt = [u[model.nu + 4 + 16 + 4 .+ (1:16)] for u in u_sol]

    qm = copy(q_opt)
    um = copy(u_opt)
    γm = copy(γ_opt)
    bm = copy(b_opt)
    ψm = copy(ψ_opt)
    ηm = copy(η_opt)
    μm = model.μ_world

    hm = h

    @save joinpath(@__DIR__, "..", "results/optimal_trajectories/", "$gait", "$(gait)_tol$(tol).jld2") qm um γm bm ψm ηm μm hm

end