
"""
	instantiate_residual!(model,
		path::AbstractString="model/residual.jld2")
Evaluates the residual expressions to generate functions, stores them into the model.
"""
function instantiate_residual!(fct::ResidualMethods, expr::Dict{Symbol,Expr};
	jacobians = :full)

	fct.r!  = eval(expr[:r])

	if jacobians == :full
		fct.rz! = eval(expr[:rz])
		fct.rθ! = eval(expr[:rθ])
	else
		# fct.rz! = rz_approx!
		# fct.rθ! = rθ_approx!
	end

	return nothing
end


include(joinpath(@__DIR__, "..", "visuals.jl"))
T = Float64
vis = Visualizer()
open(vis)



# get hopper model
model = get_model("flamingo")

nq = model.dim.q
nu = model.dim.u
nc = model.dim.c
nb = model.dim.b
nd = nq + nc + nb
nr = nq + nu + nc + nb + nd

# time
ref_traj = get_trajectory("flamingo", "gait0", load_type=:split_traj_alt, model=model)
H = ref_traj.H
h = ref_traj.h
N_sample = 2
H_mpc = 10
h_sim = h / N_sample
H_sim = 250

q0_sim = SVector{model.dim.q}(
	[0.0, 0.849, -0.00, 0.1, 0.30, -0.3, -0.1, π/2, π/2])
q1_sim = SVector{model.dim.q}(
	[0.0, 0.849, -0.00, 0.1, 0.30, -0.3, -0.1, π/2, π/2])
visualize_robot!(vis, model, [q0_sim])
kinematics_3(model, q0_sim, body=:foot_1, mode=:toe)[2]
kinematics_3(model, q0_sim, body=:foot_2, mode=:toe)[2]


p = flamingo_policy(model, h_sim, qref=q0_sim)

sim = ContactControl.simulator(model, q0_sim, q1_sim, h_sim, H_sim,
    p = p,
    ip_opts = ContactControl.InteriorPointOptions(
        r_tol = 1.0e-8,
        κ_init = 1.0e-8,
        κ_tol = 2.0e-8),
    sim_opts = ContactControl.SimulatorOptions(warmstart = true))
@time status = ContactControl.simulate!(sim)

anim = visualize_robot!(vis, model, sim.traj, name=:mpc, sample=20)
visualize_force!(vis, model, sim.traj, name=:mpc, anim=anim, sample=20)

B0 = B_func(model, q0_sim)

inv(B0*B0')


plt = plot(layout=(3,1), legend=false)
plot!(plt[1,1], hcat(Vector.([q[1:1] for q in sim.traj.q])...)',
	color=:blue, linewidth=1.0)
plot!(plt[2,1], hcat(Vector.([u[1:nu] for u in sim.traj.u]*N_sample)...)',
	color=:blue, linewidth=1.0)
plot!(plt[3,1], hcat(Vector.([γ[1:nc] for γ in sim.traj.γ]*N_sample)...)',
	color=:blue, linewidth=1.0)

anim = visualize_robot!(vis, model, sim.traj, name=:mpc)
visualize_force!(vis, model, sim.traj, name=:mpc, anim=anim)

filename = "flamingo_pd"
filename = "flamingo_pd"
filename = "flamingo_pd"
filename = "flamingo_pd"
filename = "flamingo_reliable"
const ContactControl = Main
MeshCat.convert_frames_to_video(
    "/home/simon/Downloads/$filename.tar",
    "/home/simon/Documents/$filename.mp4", overwrite=true)

convert_video_to_gif(
    "/home/simon/Documents/$filename.mp4",
    "/home/simon/Documents/$filename.gif", overwrite=true, width=720)




k = kinematic_map(model, q0_sim, body=:foot_2)
J = jacobian_map(model, q0_sim, body=:foot_2)
