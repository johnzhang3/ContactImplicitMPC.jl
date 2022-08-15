using MeshCat

video_path = joinpath(@__DIR__, "..", "..", "examples/A1-imitation/videos")

# pace forward centroidal reference
pace_forward_cent_ref_tar_path = joinpath(video_path, "pace_forward_cent_ref.tar")
pace_forward_cent_out_mp4_path = joinpath(video_path, "pace_forward_cent_ref.mp4")
MeshCat.convert_frames_to_video(pace_forward_cent_ref_tar_path, pace_forward_cent_out_mp4_path, overwrite=true)

# pace forward centroidal optimal 
pace_forward_cent_opt_tar_path = joinpath(video_path, "pace_forward_cent_optimal.tar")
pace_forward_cent_opt_mp4_path = joinpath(video_path, "pace_forward_cent_optimal.mp4")
MeshCat.convert_frames_to_video(pace_forward_cent_opt_tar_path, pace_forward_cent_opt_mp4_path, overwrite=true)

# pace backward centroidal reference
pace_backward_cent_ref_tar_path = joinpath(video_path, "pace_backward_cent_ref.tar")
pace_backward_cent_ref_mp4_path = joinpath(video_path, "pace_backward_cent_ref.mp4")
MeshCat.convert_frames_to_video(pace_backward_cent_ref_tar_path, pace_backward_cent_ref_mp4_path, overwrite=true)


# pace backward centroidal optimal
pace_backward_cent_opt_tar_path = joinpath(video_path, "pace_backward_cent_optimal.tar")
pace_backward_cent_opt_mp4_path = joinpath(video_path, "pace_backward_cent_optimal.mp4")
MeshCat.convert_frames_to_video(pace_backward_cent_opt_tar_path, pace_backward_cent_opt_mp4_path, overwrite=true)


# sidesteps centroidal reference
sidesteps_cent_ref_tar_path = joinpath(video_path, "sidesteps_cent_ref.tar")
sidesteps_cent_ref_mp4_path = joinpath(video_path, "sidesteps_cent_ref.mp4")
MeshCat.convert_frames_to_video(sidesteps_cent_ref_tar_path, sidesteps_cent_ref_mp4_path, overwrite=true)



# sidesteps centroidal optimal
sidesteps_cent_opt_tar_path = joinpath(video_path, "sidesteps_cent_optimal.tar")
sidesteps_cent_opt_mp4_path = joinpath(video_path, "sidesteps_cent_optimal.mp4")
MeshCat.convert_frames_to_video(sidesteps_cent_opt_tar_path, sidesteps_cent_opt_mp4_path, overwrite=true)

