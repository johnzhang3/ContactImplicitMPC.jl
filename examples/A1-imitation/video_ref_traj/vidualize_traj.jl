
using MeshCat
video_tar_path = joinpath(@__DIR__, "tars/$(gait)_ref.tar")
video_mp4_path = joinpath(@__DIR__, "mp4s/$(gait)_ref.mp4")
MeshCat.convert_frames_to_video(video_tar_path, video_mp4_path, overwrite=true)

video_tar_path = joinpath(@__DIR__, "tars/$(gait)_opt.tar")
video_mp4_path = joinpath(@__DIR__, "mp4s/$(gait)_opt.mp4")
MeshCat.convert_frames_to_video(video_tar_path, video_mp4_path, overwrite=true)

video_tar_path = joinpath(@__DIR__, "tars/$(gait)_pre_adjust.tar")
video_mp4_path = joinpath(@__DIR__, "mp4s/$(gait)_pre_adjust.mp4")
MeshCat.convert_frames_to_video(video_tar_path, video_mp4_path, overwrite=true)