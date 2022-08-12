# PREAMBLE

# PKG_SETUP

# ## Setup

using ContactImplicitMPC
using LinearAlgebra
using Quaternions
using BenchmarkTools

# ## Visualizer
vis = ContactImplicitMPC.Visualizer()
ContactImplicitMPC.render(vis)

@show Threads.nthreads()

# include("continuous_policy.jl")
include("continuous_policy_v3.jl")

# ## Simulation
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env


# ## Reference Trajectory
ref_traj = deepcopy(get_trajectory(s.model, s.env,
	joinpath(@__DIR__, "reference/inplace_trot_v7.jld2"),
    load_type = :split_traj_alt));

H = ref_traj.H
h = ref_traj.h

# ## MPC setup
N_sample = 5
H_mpc = 50
h_sim = h / N_sample
H_sim = 2000
κ_mpc = 1.0e-3

v0 = -0.0
obj = TrackingVelocityObjective(model, env, H_mpc,
    v = [Diagonal(1e-3 * [[1,1,1]; 1e+3*[1,1,1]; fill([1,1,1], 4)...]) for t = 1:H_mpc],
	q = [relative_state_cost(1e-0*[0.0,1,1], 3e-1*[1,1,1], 1e-0*[0.2,0.2,1]) for t = 1:H_mpc],
	u = [Diagonal(3e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
	v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# obj = TrackingVelocityObjective(model, env, H_mpc,
#     v = h / H_mpc * [Diagonal([[1,1,1]; [1,1,1]; fill([1,1,1], 4)...]) for t = 1:H_mpc],
# 	q = h / H_mpc * [relative_state_cost([1,1,1], [1,1,1], [1,1,1]) for t = 1:H_mpc],
# 	u = h / H_mpc * [Diagonal(vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# 	v_target = h / H_mpc * [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

p = ci_mpc_policy(ref_traj, s, obj,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
	mode = :configuration,
	ip_opts = InteriorPointOptions(
					undercut = 5.0,
					κ_tol = κ_mpc,
					r_tol = 1.0e-4, # TODO maybe relax this parameter
					diff_sol = true,
					solver = :empty_solver,
					max_time = 1e5),
    n_opts = NewtonOptions(
        r_tol = 3e-5,
        max_time=1.0e-1,
		solver=:ldl_solver,
        threads=false,
        verbose=false,
        max_iter = 5),
    mpc_opts = CIMPCOptions(
		# live_plotting=true
		# gains=true,
		));

# ## Disturbances
w = [1.0 * [0.0, i == 100 ? 5.0 : 0.0, i == 10 ? 1.0 : 0.0] for i=1:H_sim/N_sample]
d = open_loop_disturbances(w, N_sample)

# ## Initial conditions
q1_sim, v1_sim = initial_conditions(ref_traj);

# ## Simulator
sim = simulator(s, H_sim, h=h_sim, policy=p, dist=d);

# ## Simulate
q1_sim0 = deepcopy(q1_sim)
RoboDojo.simulate!(sim, q1_sim0, v1_sim)

# ## Visualize
#set_light!(vis)
set_floor!(vis, grid=false)
set_background!(vis)
anim = visualize!(vis, model, sim.traj.q; Δt=h_sim)

# # ## Timing result
# # Julia is [JIT-ed](https://en.wikipedia.org/wiki/Just-in-time_compilation) so re-run the MPC setup through Simulate for correct timing results.
process!(sim.stats, N_sample) # Time budget
H_sim * h_sim / sum(sim.stats.policy_time) # Speed ratio
plot(sim.stats.policy_time, xlabel="timestep", ylabel="mpc time (s)",
	ylims=[-0.001, 0.1],
	label="", linetype=:steppost)


plt = plot()
plot!(plt, hcat(Vector.([(sim.traj.q[i+1][1:1] - sim.traj.q[i][1:1]) / sim.h for i=1:H_sim])...)')
plot!(plt, hcat(Vector.([(sim.traj.q[i+1][7:7] - sim.traj.q[i][7:7]) / sim.h for i=1:H_sim])...)')
plot!(plt, hcat(Vector.([(sim.traj.q[i+1][10:10] - sim.traj.q[i][10:10]) / sim.h for i=1:H_sim])...)')
plot!(plt, hcat(Vector.([(sim.traj.q[i+1][13:13] - sim.traj.q[i][13:13]) / sim.h for i=1:H_sim])...)')
plot!(plt, hcat(Vector.([(sim.traj.q[i+1][16:16] - sim.traj.q[i][16:16]) / sim.h for i=1:H_sim])...)')
