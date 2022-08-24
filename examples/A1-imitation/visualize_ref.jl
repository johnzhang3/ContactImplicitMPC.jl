using JSON 
using MeshCat

include(joinpath(@__DIR__, "..", "..", "examples/centroidal_quadruped/reference/trajopt_model_v2.jl"))
include(joinpath(@__DIR__, "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))
include(joinpath("..", "..",  "examples/A1-imitation/utils/utilities.jl"))
include(joinpath("..", "..", "examples/A1-imitation/utils/plot_utils.jl"))

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat");
model = s.model;

# from video
video_traj_path = joinpath(@__DIR__, "from_gengshan/video_traj.json")
q_ref, h, T = convert_q_from_json(video_traj_path);

vis = Visualizer();
render(vis);
visualize!(vis, model, q_ref, Δt = h);

video_tar_path = joinpath(@__DIR__, "from_gengshan/video_traj.tar")
video_mp4_path = joinpath(@__DIR__, "from_gengshan/video_traj.mp4")
MeshCat.convert_frames_to_video(video_tar_path, video_mp4_path, overwrite=true)