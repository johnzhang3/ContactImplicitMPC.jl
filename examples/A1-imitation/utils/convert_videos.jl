using MeshCat

video_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/videos")
# video_path = joinpath(@__DIR__, "..", "results/videos_8_17")

# pace forward centroidal reference
# pace_forward_cent_ref_tar_path = joinpath(video_path, "pace_forward_cent_ref.tar")
# pace_forward_cent_out_mp4_path = joinpath(video_path, "pace_forward_cent_ref.mp4")
# MeshCat.convert_frames_to_video(pace_forward_cent_ref_tar_path, pace_forward_cent_out_mp4_path, overwrite=true)

# pace forward centroidal optimal 
pace_forward_cent_opt_tar_path = joinpath(video_path, "pace_forward.tar")
pace_forward_cent_opt_mp4_path = joinpath(video_path, "pace_forward.mp4")
MeshCat.convert_frames_to_video(pace_forward_cent_opt_tar_path, pace_forward_cent_opt_mp4_path, overwrite=true)

# pace backward centroidal reference
# pace_backward_cent_ref_tar_path = joinpath(video_path, "pace_backward_cent_ref.tar")
# pace_backward_cent_ref_mp4_path = joinpath(video_path, "pace_backward_cent_ref.mp4")
# MeshCat.convert_frames_to_video(pace_backward_cent_ref_tar_path, pace_backward_cent_ref_mp4_path, overwrite=true)


# pace backward centroidal optimal
pace_backward_cent_opt_tar_path = joinpath(video_path, "pace_backward.tar")
pace_backward_cent_opt_mp4_path = joinpath(video_path, "pace_backward.mp4")
MeshCat.convert_frames_to_video(pace_backward_cent_opt_tar_path, pace_backward_cent_opt_mp4_path, overwrite=true)


# sidesteps centroidal reference
# sidesteps_cent_ref_tar_path = joinpath(video_path, "sidesteps_cent_ref.tar")
# sidesteps_cent_ref_mp4_path = joinpath(video_path, "sidesteps_cent_ref.mp4")
# MeshCat.convert_frames_to_video(sidesteps_cent_ref_tar_path, sidesteps_cent_ref_mp4_path, overwrite=true)



# sidesteps centroidal optimal
sidesteps_cent_opt_tar_path = joinpath(video_path, "sidesteps.tar")
sidesteps_cent_opt_mp4_path = joinpath(video_path, "sidesteps.mp4")
MeshCat.convert_frames_to_video(sidesteps_cent_opt_tar_path, sidesteps_cent_opt_mp4_path, overwrite=true)

# hopturn reference 
hopturn_cent_ref_tar_path = joinpath(video_path, "hopturn_cent_ref.tar")
hopturn_cent_ref_mp4_path = joinpath(video_path, "hopturn_cent_ref.mp4")
MeshCat.convert_frames_to_video(hopturn_cent_ref_tar_path, hopturn_cent_ref_mp4_path, overwrite=true)


# hopturn optiomal 
hopturn_cent_opt_tar_path = joinpath(video_path, "hopturn.tar")
hopturn_cent_opt_mp4_path = joinpath(video_path, "hopturn.mp4")
MeshCat.convert_frames_to_video(hopturn_cent_opt_tar_path, hopturn_cent_opt_mp4_path, overwrite=true)
