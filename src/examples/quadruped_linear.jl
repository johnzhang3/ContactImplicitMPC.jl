include(joinpath(@__DIR__, "..", "dynamics", "quadrupedlinear", "visuals.jl"))
vis = Visualizer()
open(vis)
# render(vis)

# get model
model = get_model("quadrupedlinear")

# get trajectory
ref_traj = get_trajectory("quadrupedlinear", "gait1", load_type = :split_traj_alt)
ref_traj_copy = deepcopy(ref_traj)


function check_traj(model::ContactDynamicsModel, traj::ContactTraj)
    nq = model.dim.q
    nz = num_var(model)
    d = [zeros(nq) for t=1:traj.H]
    r = [zeros(nz) for t=1:traj.H]
    for t = 1:ref_traj.H
        k = kinematics(model, traj.q[t+2])
        λ = contact_forces(model, traj.γ[t], traj.b[t], traj.q[t+2], k)
        d[t] = dynamics(model, traj.h, traj.q[t], traj.q[t+1], traj.u[t], traj.w[t], λ, traj.q[t+2])
        r[t] = residual(model, traj.z[t], traj.θ[t], [1e-8])
    end
    return d, r
end
ref_traj
nz = num_var(model)
dvio, rvio = check_traj(model, ref_traj)
plot(hcat([v[1:end] for v in dvio]...)')
plot(hcat([v[1:1] for v in dvio]...)')
plot(hcat([v[1:66] for v in rvio]...)')
plot(hcat([v[1:18] for v in rvio]...)')

# time
H = ref_traj.H
h = ref_traj.h
N_sample = 5
H_mpc = 10
h_sim = h / N_sample
H_sim = 30 #4000 #3000

# barrier parameter
κ_mpc = 1.0e-4

obj = TrackingObjective(H_mpc, model.dim,
    q = [Diagonal(1e-2 * [1.0; 0.02; 0.25; 0.25 * ones(model.dim.q-3)]) for t = 1:H_mpc],
    u = [Diagonal(3e-2 * ones(model.dim.u)) for t = 1:H_mpc],
    γ = [Diagonal(1.0e-100 * ones(model.dim.c)) for t = 1:H_mpc],
    b = [Diagonal(1.0e-100 * ones(model.dim.b)) for t = 1:H_mpc])

nz = num_var(model)
nθ = num_data(model)

rz0 = zeros(nz,nz)
rθ0 = zeros(nz,nθ)
model.res.rz!(rz0, ref_traj.z[1], ref_traj.θ[1])
model.res.rθ!(rθ0, ref_traj.z[1], ref_traj.θ[1])

ix, iy1, iy2 = linearization_var_index(model)
idyn, irst, ibil, ialt = linearization_term_index(model)
plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [ix; iy1; iy2]])))
plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [ix;  ]])))
plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [iy1; ]])))
plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [iy2; ]])))
plot(Gray.(1e10*abs.(rz0[[idyn;], [ix; iy1; iy2]])))
plot(Gray.(1e10*abs.(rz0[[irst;], [ix; iy1; iy2]])))
plot(Gray.(1e10*abs.(rz0[[ibil;], [ix; iy1; iy2]])))


plot(Gray.(1e10*abs.(rθ0[[idyn; irst; ibil], :])))
plot(Gray.(1e10*abs.(rθ0[idyn,:])))
plot(Gray.(1e10*abs.(rθ0[irst,:])))
plot(Gray.(1e10*abs.(rθ0[ibil,:])))



# # get model
# model_hopper = get_model("hopper_2D")
#
# # get trajectory
# ref_traj_hopper = get_trajectory("hopper_2D", "gait_in_place", load_type = :joint_traj)
#
# nz = num_var(model_hopper)
# nθ = num_data(model_hopper)
#
# rz0 = zeros(nz,nz)
# rθ0 = zeros(nz,nθ)
# model_hopper.res.rz!(rz0, ref_traj_hopper.z[1], ref_traj_hopper.θ[1])
# model_hopper.res.rθ!(rθ0, ref_traj_hopper.z[1], ref_traj_hopper.θ[1])
#
# ix, iy1, iy2 = linearization_var_index(model_hopper)
# idyn, irst, ibil, ialt = linearization_term_index(model_hopper)
# plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [ix; iy1; iy2]])))
# plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [ix;  ]])))
# plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [iy1; ]])))
# plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [iy2; ]])))
# plot(Gray.(1e10*abs.(rz0[[idyn;], [ix; iy1; iy2]])))
# plot(Gray.(1e10*abs.(rz0[[irst;], [ix; iy1; iy2]])))
# plot(Gray.(1e10*abs.(rz0[[ibil;], [ix; iy1; iy2]])))
#
#
# plot(Gray.(1e10*abs.(rθ0[[idyn; irst; ibil], :])))
# plot(Gray.(1e10*abs.(rθ0[idyn,:])))
# plot(Gray.(1e10*abs.(rθ0[irst,:])))
# plot(Gray.(1e10*abs.(rθ0[ibil,:])))




function rz!_(model, z, θ, k)
    f(z) = residual(model, z, θ, [k])
    return ForwardDiff.jacobian(f, z)
end


rz0 = rz!_(model, ref_traj.z[1], ref_traj.θ[1], 1e-4)
ix, iy1, iy2 = linearization_var_index(model)
idyn, irst, ibil, ialt = linearization_term_index(model)
plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [ix; iy1; iy2]])))
plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [ix;  ]])))
plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [iy1; ]])))
plot(Gray.(1e10*abs.(rz0[[idyn; irst; ibil;], [iy2; ]])))
plot(Gray.(1e10*abs.(rz0[[idyn;], [ix; iy1; iy2]])))
plot(Gray.(1e10*abs.(rz0[[irst;], [ix; iy1; iy2]])))
plot(Gray.(1e10*abs.(rz0[[ibil;], [ix; iy1; iy2]])))


plot(Gray.(1e10*abs.(rθ0[[idyn; irst; ibil], :])))
plot(Gray.(1e10*abs.(rθ0[idyn,:])))
plot(Gray.(1e10*abs.(rθ0[irst,:])))
plot(Gray.(1e10*abs.(rθ0[ibil,:])))







p = linearized_mpc_policy(ref_traj, model, obj,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
    n_opts = NewtonOptions(
        r_tol = 3e-4,
        solver = :ldl_solver,
        max_iter = 5),
    mpc_opts = LinearizedMPCOptions())

q1_ref = copy(ref_traj.q[2])
q0_ref = copy(ref_traj.q[1])
q1_sim = SVector{model.dim.q}(q1_ref)
q0_sim = SVector{model.dim.q}(copy(q1_sim - (q1_ref - q0_ref) / N_sample))
@assert norm((q1_sim - q0_sim) / h_sim - (q1_ref - q0_ref) / h) < 1.0e-8

sim = ContactControl.simulator(model, q0_sim, q1_sim, h_sim, H_sim,
    p = p,
    ip_opts = ContactControl.InteriorPointOptions(
        r_tol = 1.0e-8,
        κ_init = 1.0e-6,
        κ_tol = 2.0e-6,
        diff_sol = true),
    sim_opts = ContactControl.SimulatorOptions(warmstart = true))

time = @elapsed status = ContactControl.simulate!(sim)
# @elapsed status = ContactControl.simulate!(sim)
# @profiler status = ContactControl.simulate!(sim)

plot_lines!(vis, model, sim.traj.q[1:25:end])
plot_surface!(vis, model.env, ylims=[0.3, -0.05])
anim = visualize_meshrobot!(vis, model, sim.traj, sample=5)
# anim = visualize_robot!(vis, model, sim.traj, anim=anim)
anim = visualize_force!(vis, model, sim.traj, anim=anim, h=h_sim)

# Display ghosts
t_ghosts = [1, 1333, 2666]
mvis_ghosts = []
for (i,t) in enumerate(t_ghosts)
    α = i/(length(t_ghosts)+1)
    name = Symbol("ghost$i")
    mvis = build_meshrobot!(vis, model, name=name, α=α)
    push!(mvis_ghosts, mvis)
end

for (i,t) in enumerate(t_ghosts)
    name = Symbol("ghost$i")
    set_meshrobot!(vis, mvis_ghosts[i], model, sim.traj.q[t], name=name)
end
