using JSON 

include(joinpath(@__DIR__, "..", "..", "examples/centroidal_quadruped/reference/trajopt_model_v2.jl"))
include(joinpath(@__DIR__, "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/utilities.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/plot_utils.jl"))

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat");
model = s.model;

# from video
video_path = joinpath(@__DIR__, "from_gengshan/video_path.json")
