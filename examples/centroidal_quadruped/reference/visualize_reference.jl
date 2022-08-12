using JSON 
using ContactImplicitMPC
include("trajopt_model_v2.jl")
include(joinpath(@__DIR__, "..", "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))

# fullbody_hopturn = JSON.parsefile("src/a1_cpp/lib/ContactImplicitMPC.jl/examples/A1-imitation/fullbody_ref_traj/hopturn.txt")
# fullbody_pace = JSON.parsefile("src/a1_cpp/lib/ContactImplicitMPC.jl/examples/A1-imitation/fullbody_ref_traj/pace.txt")

cent_hopturn = JSON.parsefile("examples/A1-imitation/centroidal_ref_traj/hopturn.json")
cent_pace = JSON.parsefile("examples/A1-imitation/centroidal_ref_traj/pace.json")

# fullbody_hopturn_ref_traj = fullbody_hopturn["Frames"]
# fullbody_pace_ref_traj = fullbody_pace["Frames"]

cent_hopturn_ref_traj = cent_hopturn["Frames"]
cent_pace_ref_traj = cent_pace["Frames"]

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env
h = 0.05

vis = Visualizer()
render(vis)

visualize!(vis, model, cent_pace_ref_traj, Δt=h);
visualize!(vis, model, cent_hopturn_ref_traj, Δt=h);