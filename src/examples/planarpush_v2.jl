const ContactControl = Main
vis = Visualizer()
open(vis)
# render(vis)



include(joinpath(@__DIR__, "..", "dynamics", "planarpush", "visuals.jl"))
model = get_model("planarpush")
nq = model.dim.q
nu = model.dim.u
nw = model.dim.w
nc = model.dim.c
nb = model.dim.b
nf = 4

# time
h = 0.01
H = 550
N_sample = 1

# initial conditions
q0 = @SVector [0.00, 0.00, 0.0, 0.0, -0.25, 5e-3, 0.00]
q1 = @SVector [0.00, 0.00, 0.0, 0.0, -0.25, 5e-3, 0.00]

# p = open_loop_policy(fill(SVector{nu}([10.0*h, -0.0]), H*2), N_sample=N_sample)
p = open_loop_policy(fill(SVector{nu}([40*h, -0.0]), H), N_sample=N_sample)

# simulator
sim0 = ContactControl.simulator(model, q0, q1, h, H,
	p = p,
	ip_opts = ContactControl.InteriorPointOptions(
		r_tol = 1.0e-8, κ_init=1e-6, κ_tol = 2.0e-6),
	sim_opts = ContactControl.SimulatorOptions(warmstart = true))

# simulate
status = ContactControl.simulate!(sim0)
@test status

visualize_robot!(vis, model, sim0.traj)

plot(hcat([x[1:nq] for x in sim0.traj.q]...)')
plot(hcat([x[1:nu] for x in sim0.traj.u]...)')
plot(hcat([x[1:nw] for x in sim0.traj.w]...)')
plot(hcat([x[1:nc] for x in sim0.traj.γ]...)')
plot(hcat([x[1:nb] for x in sim0.traj.b]...)')
# plot(hcat([x[1:8] for x in sim0.traj.b]...)')


ref_traj = deepcopy(sim0.traj)
ref_traj.H

# MPC
N_sample = 1
H_mpc = 40
h_sim = h / N_sample
H_sim = 500

# barrier parameter
κ_mpc = 1.0e-4

# Aggressive
obj = TrackingVelocityObjective(H_mpc, model.dim,
	q = [Diagonal(1.0e-4 * [1e2, 1e2, 1e-1, 1e-1, 1e-0, 1e-0, 1e-0,]) for t = 1:H_mpc],
	v = [Diagonal(1.0e-3 * ones(model.dim.q)) for t = 1:H_mpc],
	u = [Diagonal(1.0e-6 * ones(model.dim.u)) for t = 1:H_mpc],
	γ = [Diagonal(1.0e-100 * ones(model.dim.c)) for t = 1:H_mpc],
	b = [Diagonal(1.0e-100 * ones(model.dim.b)) for t = 1:H_mpc])

#
obj = TrackingVelocityObjective(H_mpc, model.dim,
	q = [Diagonal(1.0e-2 * [1e2, 1e2, 1e-1, 1e-1, 1e-0, 1e-0, 1e-0,]) for t = 1:H_mpc],
	v = [Diagonal(1.0e-1 * ones(model.dim.q)) for t = 1:H_mpc],
	u = [Diagonal(1.0e-4 * ones(model.dim.u)) for t = 1:H_mpc],
	γ = [Diagonal(1.0e-100 * ones(model.dim.c)) for t = 1:H_mpc],
	b = [Diagonal(1.0e-100 * ones(model.dim.b)) for t = 1:H_mpc])

p = linearized_mpc_policy(ref_traj, model, obj,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
    n_opts = NewtonOptions(
        r_tol = 3e-4,
        solver = :ldl_solver,
		# max_iter = 5),
		max_iter = 5),
    mpc_opts = LinearizedMPCOptions(
		# live_plotting=true
		))

# p = open_loop_policy(fill(SVector{nu}([40*h, -0.0]), H*2), N_sample=N_sample)
using Random
Random.seed!(100)
d = open_loop_disturbances([[0.05*rand(), 0.0, 0.4*rand()] for t=1:H_sim])

q0_sim = @SVector [0.00, 0.00, 0.0, 0.0, -0.25, 5e-3, 0.00]
q1_sim = @SVector [0.00, 0.00, 0.0, 0.0, -0.25, 5e-3, 0.00]

sim = ContactControl.simulator(model, q0_sim, q1_sim, h_sim, H_sim,
	uL=-0.9*ones(model.dim.u),
	uU=+2.0*ones(model.dim.u),
    p = p,
	d = d,
    ip_opts = ContactControl.InteriorPointOptions(
        r_tol = 1.0e-8,
        κ_init = 1.0e-6,
        κ_tol = 2.0e-6,
		diff_sol=true),
    sim_opts = ContactControl.SimulatorOptions(warmstart = false))

sim.traj.u

@time status = ContactControl.simulate!(sim)

anim = visualize_robot!(vis, model, sim.traj, name=:sim, sample = N_sample, α=1.0)
anim = visualize_robot!(vis, model, ref_traj, name=:ref, anim=anim, sample = 1, α=0.5)

plot(hcat([x[1:2] ./ N_sample for x in ref_traj.u]...)')
scatter!(hcat([x[1:2] for x in sim.traj.u]...)')
sim.traj.u
# filename = "planarpush_precise"
# MeshCat.convert_frames_to_video(
#     "/home/simon/Downloads/$filename.tar",
#     "/home/simon/Documents/$filename.mp4", overwrite=true)
#
# convert_video_to_gif(
#     "/home/simon/Documents/$filename.mp4",
#     "/home/simon/Documents/$filename.gif", overwrite=true)
