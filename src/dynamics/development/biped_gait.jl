# Reference trajectory
model = get_model("biped", surf = "flat")
ref_traj = get_trajectory("biped", "gait1")
T = ref_traj.H
h = ref_traj.h

for t = 1:T
	# r = residual(model, ref_traj.z[t], ref_traj.θ[t], 0.0)
	r = zero(ref_traj.z[t])
	r_fast!(r, model, ref_traj.z[t], ref_traj.θ[t], 0.0)
	@show norm(r)#[nq + 2nc + nb + 2nc .+ (1:nb)])
end

maximum([norm(ContactControl.dynamics(model,
	ref_traj.h, ref_traj.q[t], ref_traj.q[t+1], ref_traj.u[t],
	zeros(model.dim.w), ref_traj.γ[t], ref_traj.b[t], ref_traj.q[t+2]), Inf) for t = 1:T])

# initial conditions
q0 = SVector{model.dim.q}(ref_traj.q[1])
q1 = SVector{model.dim.q}(ref_traj.q[2])

# simulator
sim = ContactControl.simulator(model, q0, q1, h, T,
    p = open_loop_policy([SVector{model.dim.u}(ut) for ut in ref_traj.u]),
    ip_opts = ContactControl.InteriorPointOptions(
		r_tol = 1.0e-8, κ_tol = 1.0e-8, κ_init = 1.0e-6, solver = :mgs_solver),
    sim_opts = ContactControl.SimulatorOptions(warmstart = true))

# simulate
@time status = ContactControl.simulate!(sim, verbose = false)

include(joinpath(module_dir(), "src/dynamics/biped/visuals.jl"))
vis = Visualizer()
# open(vis)
render(vis)
# visualize!(vis, model, q, Δt = h)
visualize!(vis, model, sim.traj.q, Δt = h)
