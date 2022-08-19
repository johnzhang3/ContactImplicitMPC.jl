using ContactImplicitMPC
using LinearAlgebra

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat");
model = s.model;
env = s.env;

pace_forward_opt_path = joinpath(@__DIR__, "..", "results", "pace_forward", "run21", "pace_forward_tol0.001.jld2")
get_trajectory(s.model, s.env, pace_forward_opt_path, load_type = :split_traj_alt)
ref_traj = deepcopy(get_trajectory(s.model, s.env, pace_forward_opt_path, load_type = :split_traj_alt))

update_friction_coefficient!(ref_traj, model, env);
H = ref_traj.H
h = ref_traj.h


# ## MPC setup
N_sample = 5
H_mpc = 10
h_sim = h / N_sample
H_sim = 1000
κ_mpc = 2.0e-4

obj = TrackingObjective(model, env, H_mpc,
    q = [Diagonal(1e-2 * [1.0; 0.02; 0.25; 0.25 * ones(model.nq-3)]) for t = 1:H_mpc],
    u = [Diagonal(3e-2 * ones(model.nu)) for t = 1:H_mpc],
    γ = [Diagonal(1.0e-100 * ones(model.nc)) for t = 1:H_mpc],
    b = [Diagonal(1.0e-100 * ones(model.nc * friction_dim(env))) for t = 1:H_mpc]);

p = ci_mpc_policy(ref_traj, s, obj,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
	mode = :configuration,
    n_opts = NewtonOptions(
		solver = :lu_solver,
		r_tol = 3e-4,
		max_iter = 5,
		),
    mpc_opts = CIMPCOptions(),
	ip_opts = InteriorPointOptions(
					undercut = 5.0,
					γ_reg = 0.1,
					κ_tol = κ_mpc,
					r_tol = 1.0e-8,
					diff_sol = true,
					solver = :empty_solver,
					# max_time = 1000.0,
					),
    )

# ## Initial conditions
q1_sim, v1_sim = initial_conditions(ref_traj); 

# ## Simulator
sim = simulator(s, H_sim, h=h_sim, policy=p);

# ## Simulate
simulate!(sim, q1_sim, v1_sim);

# ## Visualizer
vis = ContactImplicitMPC.Visualizer()
ContactImplicitMPC.render(vis)

# ## Visualize
anim = visualize_meshrobot!(vis, model, sim.traj, h=h_sim * 5, sample=5);

# ## Timing result
# Julia is [JIT-ed](https://en.wikipedia.org/wiki/Just-in-time_compilation) so re-run the MPC setup through Simulate for correct timing results.
process!(sim.stats, N_sample) # Time budget
H_sim * h_sim / sum(sim.stats.policy_time) # Speed ratio