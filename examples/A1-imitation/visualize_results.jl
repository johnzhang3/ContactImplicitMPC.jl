
include(joinpath(@__DIR__, "..", "..", "examples/centroidal_quadruped/reference/trajopt_model_v2.jl"))
include(joinpath(@__DIR__, "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat");
model = s.model;

# pace forward 
pace_opt_path = joinpath(@__DIR__, "results", "pace_forward", "run21", "pace_forward_tol0.001.jld2")
@load pace_opt_path qm qm um γm bm ψm ηm μm hm;
vis = Visualizer();
render(vis);
visualize!(vis, model, qm, Δt=hm);

# pace backward 
pace_opt_path = joinpath(@__DIR__, "results", "pace_backward", "run5", "pace_backward_tol0.001.jld2")
@load pace_opt_path qm qm um γm bm ψm ηm μm hm;
vis = Visualizer();
render(vis);
visualize!(vis, model, qm, Δt=hm);

# sidesteps 
pace_opt_path = joinpath(@__DIR__, "results", "sidesteps", "run12", "sidesteps_tol0.001.jld2")
@load pace_opt_path qm qm um γm bm ψm ηm μm hm;
vis = Visualizer();
render(vis);
visualize!(vis, model, qm, Δt=hm);

# hopturn 
pace_opt_path = joinpath(@__DIR__, "results", "hopturn", "run8", "hopturn_tol0.001.jld2")
@load pace_opt_path qm qm um γm bm ψm ηm μm hm;
vis = Visualizer();
render(vis);
visualize!(vis, model, qm, Δt=hm);

# video trajectory
# vidoe_traj_path = 