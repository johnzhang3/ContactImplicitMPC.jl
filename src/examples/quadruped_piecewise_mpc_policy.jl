include(joinpath(@__DIR__, "..", "dynamics", "quadruped", "visuals.jl"))
T = Float64
vis = Visualizer()
open(vis)
render(vis)

### quadruped piecewise surface
model_sim = deepcopy(quadruped)
dir = joinpath(pwd(), "src/dynamics/quadruped")
include(joinpath(pwd(), "src/simulator/environment/piecewise.jl"))
model_sim.env = Environment{R2}(terrain_sym, d_terrain_sym)

t = range(-5.0, stop = 5.0, length = 100)
plot(t, model_sim.env.surf.(t))
path_base = joinpath(dir, "dynamics/base.jld2")
path_dyn = joinpath(dir, "dynamics/dynamics.jld2")
path_res = joinpath(dir, "piecewise/residual.jld2")
path_jac = joinpath(dir, "piecewise/sparse_jacobians.jld2")
path_linearized = joinpath(dir, "piecewise/linearized.jld2")

instantiate_base!(model_sim, path_base)

expr_dyn = generate_dynamics_expressions(model_sim, derivs = true)
save_expressions(expr_dyn, path_dyn, overwrite=true)
instantiate_dynamics!(model_sim, path_dyn, derivs = true)

expr_res, rz_sp, rθ_sp = generate_residual_expressions(model_sim, jacobians = :approx)
save_expressions(expr_res, path_res, overwrite=true)
@save path_jac rz_sp rθ_sp
instantiate_residual!(model_sim, path_res, jacobians = :approx)
model_sim.spa.rz_sp = copy(rz_sp)
model_sim.spa.rθ_sp = copy(rθ_sp)

###

model = get_model("quadruped", surf="flat")
nq = model.dim.q
nu = model.dim.u
nc = model.dim.c
nb = model.dim.b
nd = nq + nc + nb
nr = nq + nu + nc + nb + nd

# get trajectory
ref_traj = get_trajectory("quadruped", "gait0", load_type=:split_traj, model=model)
ref_traj_copy = deepcopy(ref_traj)

# time
H = ref_traj.H
h = ref_traj.h
N_sample = 5
H_mpc = 10
h_sim = h / N_sample
H_sim = 1000

# barrier parameter
κ_mpc = 1.0e-4

cost = CostFunction(H_mpc, model.dim,
    q = [Diagonal(1e-2 * [0.02; 0.02; 1.0; 0.25 * ones(nq-3)]) for t = 1:H_mpc],
    u = [Diagonal(3e-2 * ones(model.dim.u)) for t = 1:H_mpc],
    γ = [Diagonal(1.0e-100 * ones(model.dim.c)) for t = 1:H_mpc],
    b = [Diagonal(1.0e-100 * ones(model.dim.b)) for t = 1:H_mpc])

p = linearized_mpc_policy(ref_traj, model, cost,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
    n_opts = NewtonOptions(
        r_tol = 3e-4,
        max_iter = 5),
    mpc_opts = LinearizedMPCOptions(
        # live_plotting=true,
        altitude_update = true,
        altitude_impact_threshold = 0.05,
        # altitude_verbose = true,
        )
    )

q1_ref = copy(ref_traj.q[2])
q0_ref = copy(ref_traj.q[1])
q1_sim = SVector{model.dim.q}(q1_ref)
q0_sim = SVector{model.dim.q}(copy(q1_sim - (q1_ref - q0_ref) / N_sample))

ϕ_func(model_sim, q1_ref)
sim = simulator(model2, q0_sim, q1_sim, h_sim, H_sim,
    p = p,
    # d = open_loop_disturbances([rand(model.dim.w) .* w_amp for i=1:H_sim]),
    ip_opts = InteriorPointOptions(
        r_tol = 1.0e-8,
        κ_init = 1.0e-6,
        κ_tol = 2.0e-6),
    sim_opts = SimulatorOptions(warmstart = true)
    )

@time status = simulate!(sim)

# plt = plot(layout=(3,1), legend=false)
# plot!(plt[1,1], hcat(Vector.(vcat([fill(ref_traj.q[i], N_sample) for i=1:H]...))...)',
#     color=:red, linewidth=3.0)
# plot!(plt[1,1], hcat(Vector.(sim.traj.q)...)', color=:blue, linewidth=1.0)
# plot!(plt[2,1], hcat(Vector.(vcat([fill(ref_traj.u[i][1:nu], N_sample) for i=1:H]...))...)',
#     color=:red, linewidth=3.0)
# plot!(plt[2,1], hcat(Vector.([u[1:nu] for u in sim.traj.u]*N_sample)...)', color=:blue, linewidth=1.0)
# plot!(plt[3,1], hcat(Vector.([γ[1:nc] for γ in sim.traj.γ]*N_sample)...)', color=:blue, linewidth=1.0)

# visualize!(vis, model, sim.traj.q[1:N_sample:end], Δt=10*h/N_sample, name=:mpc)
plot_lines!(vis, model, sim.traj.q[1:N_sample:end])
plot_surface!(vis, model_sim.env)
anim = visualize_robot!(vis, model_sim, sim.traj)
anim = visualize_force!(vis, model_sim, sim.traj, anim=anim, h=h_sim)

settransform!(vis["/Cameras/default"],
	    compose(Translation(0.0, 0.0, -1.0), LinearMap(RotZ(-pi / 2.0))))

# filename = "quadruped_forces"
# MeshCat.convert_frames_to_video(
#     "/home/simon/Downloads/$filename.tar",
#     "/home/simon/Documents/$filename.mp4", overwrite=true)

# convert_video_to_gif(
#     "/home/simon/Documents/$filename.mp4",
#     "/home/simon/Documents/$filename.gif", overwrite=true)

# const ContactControl = Main
