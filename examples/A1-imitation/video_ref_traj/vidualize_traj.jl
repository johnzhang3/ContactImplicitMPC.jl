include(joinpath("..", "..", "..", "examples/A1-imitation/utils/utilities.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/plot_utils.jl"))

gait = 1005
ref_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/video_ref_traj/trajs_json/$(gait).json")

q_ref, h, T = convert_q_from_json(ref_path);
save_dir = joinpath(ref_path, "../plots")

# function correct_ref_frame(path)
#     q_any = JSON.parsefile(path)["Frames"];
#     T = size(q_any)[1] - 1
#     h = JSON.parsefile(path)["FrameDuration"]
#     q_ref = zeros(size(q_any)[1],18);

#     q_ref = zeros(size(q_any)[1],18);
#     for i = 1:size(q_any)[1]
#         q = Float64.(q_any[i])
#         body_pos = q[1:3]
#         body_rot = q[4:6]

#         # transfomation matrices
#         translation = Translation(body_pos[1], body_pos[2], body_pos[3])
#         rotation = LinearMap(RotZYX(body_rot[1], body_rot[2], body_rot[3]))
#         # rotation = rotation_matrix(body_rot)
#         composed = compose(rotation, translation)
#         # composed_inv = inv(composed)

#         FR = composed(q[6 .+ (1:3)])
#         FL = composed(q[6 + 3 .+ (1:3)])
#         BR = composed(q[6 + 3 + 3 .+ (1:3)])
#         BL = composed(q[6 + 3 + 3 + 3 .+ (1:3)])
#         q_ref[i,:] = cat(body_pos, body_rot, FL, FR, BL, BR, dims=1)
#     end
#     q_ref = [q_ref[i,:] for i in 1:size(q_ref,1)];
#     return q_ref, h, T
# end
# q_ref, h, T = correct_ref_frame(ref_path);

plt_ref_traj(q_ref, save_dir, gait)


s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat");
model = s.model;
# ## visualize
vis = Visualizer();
render(vis);
visualize!(vis, model, q_ref);

using MeshCat
video_tar_path = joinpath(@__DIR__, "tars/$(gait).tar")
video_mp4_path = joinpath(@__DIR__, "mp4s/$(gait).mp4")
MeshCat.convert_frames_to_video(video_tar_path, video_mp4_path, overwrite=true)