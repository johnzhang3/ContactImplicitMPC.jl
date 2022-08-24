#=
This script plots reference and optimal trajectories
=#

using JSON 
using ContactImplicitMPC

function plt_ref_foot_height(gait)
    ref_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/centroidal_ref_traj", "$(gait).json")
    q_ref_any = JSON.parsefile(ref_path)["Frames"];
    q_ref = zeros(size(q_ref_any)[1],18);

    for i = 1:size(q_ref_any)[1]
        q_ref[i,:] = Float64.(q_ref_any[i]);
    end

    plot(q_ref[:, 9], label = "foot 1", title = "$(gait) foot height", lw=4)
    plot!(q_ref[:, 12], label = "foot 2", lw=4)
    plot!(q_ref[:, 15], label = "foot 3", lw=4)
    plot!(q_ref[:, 18], label = "foot 4", lw=4)

    fig_path = joinpath(@__DIR__, "..", "..", "..", "examples", "A1-imitation", "results", "figures")
    fig_file_path = joinpath(fig_path, "ref_$(gait)_foot_height.png")
    savefig(fig_file_path)

end

function plt_opt_foot_height(gait, tol, path)
    opt_path = joinpath(path, "$(gait)_tol$(tol).jld2")
    @load opt_path qm um γm bm ψm ηm μm hm
    q_opt = zeros(size(qm)[1],18)
    for i = 1:size(qm)[1]
        q_opt[i,:] = Float64.(qm[i]);
    end

    plot(q_opt[:, 9], label = "foot 1", title = "$(gait) foot height tol = $(tol)", lw=4)
    plot!(q_opt[:, 12], label = "foot 2", lw=4)
    plot!(q_opt[:, 15], label = "foot 3", lw=4)
    plot!(q_opt[:, 18], label = "foot 4", lw=4)

    fig_file_path = joinpath(path, "opt_$(gait)_tol$(tol)_foot_height.png")
    savefig(fig_file_path)
end

function plt_ref_foot_height(q, dir, title)
    q_new = zeros(size(q)[1], 18)
    for i = 1:size(q)[1]
        q_new[i, :] = Float64.(q[i])
    end
    plot(q_new[:, 9], label = "foot 1", title = title, lw=4)
    plot!(q_new[:, 12], label = "foot 2", lw=4)
    plot!(q_new[:, 15], label = "foot 3", lw=4)
    plot!(q_new[:, 18], label = "foot 4", lw=4)
    savefig(joinpath(dir, "$(title).png"))
end
# gaits = ["pace_forward", "pace_backward", "hopturn", "sidesteps"];

# for gait in gaits
#     plt_ref_foot_height(gait)
#     plt_opt_foot_height(gait)
# end
