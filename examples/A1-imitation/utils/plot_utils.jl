#=
This script plots reference and optimal trajectories
=#

using JSON 
using ContactImplicitMPC

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

function plt_ref_traj(q, dir, gait)
    q_new = zeros(size(q)[1], 18)
    for i = 1:size(q)[1]
        q_new[i, :] = Float64.(q[i])
    end

    # heights
    plot(q_new[:, 3], label = "body", title = "ref $(gait) heights", lw=4)
    plot!(q_new[:, 9], label = "foot 1", lw=4)
    plot!(q_new[:, 12], label = "foot 2", lw=4)
    plot!(q_new[:, 15], label = "foot 3", lw=4)
    plot!(q_new[:, 18], label = "foot 4", lw=4)
    savefig(joinpath(dir, "ref_$(gait)_height.png"))

    # x 
    plot(q_new[:, 1], label = "body", title = "ref $(gait) x", lw=4)
    plot!(q_new[:, 7], label = "foot 1", lw=4)
    plot!(q_new[:, 10], label = "foot 2", lw=4)
    plot!(q_new[:, 13], label = "foot 3", lw=4)
    plot!(q_new[:, 16], label = "foot 4", lw=4)
    fig_file_path = joinpath(dir, "ref_$(gait)_x.png")
    savefig(fig_file_path)

    # y 
    plot(q_new[:, 2], label = "body", title = "ref $(gait) y", lw=4)
    plot!(q_new[:, 8], label = "foot 1", lw=4)
    plot!(q_new[:, 11], label = "foot 2", lw=4)
    plot!(q_new[:, 14], label = "foot 3", lw=4)
    plot!(q_new[:, 17], label = "foot 4", lw=4)
    fig_file_path = joinpath(dir, "ref_$(gait)_y.png")
    savefig(fig_file_path)

    # roll pitch yaw angles
    plot(q_new[:, 4], label = "yaw", title = "ref $(gait) euler angles", lw=4)
    plot!(q_new[:, 5], label = "pitch", lw=4)
    plot!(q_new[:, 6], label = "roll", lw=4)
    fig_file_path = joinpath(dir, "ref_$(gait)euler_angle.png")
    savefig(fig_file_path)

    # support polygon
    plot(q_new[:, 1], q_new[:, 2], label = "body", title = "$(gait) support polygon", lw=6)
    plot!(q_new[:, 7], q_new[:, 8], label = "foot 1", lw=6)
    plot!(q_new[:, 10], q_new[:, 11], label = "foot 2", lw=6)
    plot!(q_new[:, 13], q_new[:, 14], label = "foot 3", lw=6)
    plot!(q_new[:, 16], q_new[:, 17], label = "foot 4", lw=6)

    fig_file_path = joinpath(dir, "ref_$(gait)_support_polygon.png")
    savefig(fig_file_path)
end

function plt_opt_results(gait, tol, path)
    opt_path = joinpath(path, "$(gait)_tol$(tol).jld2")
    @load opt_path qm um γm bm ψm ηm μm hm

    q_opt = zeros(size(qm)[1],size(qm[1])[1])
    for i = 1:size(qm)[1]
        q_opt[i,:] = Float64.(qm[i]);
    end

    # heights
    plot(q_opt[:, 3], label = "body", title = "$(gait) heights", lw=4)
    plot!(q_opt[:, 9], label = "foot 1", lw=4)
    plot!(q_opt[:, 12], label = "foot 2", lw=4)
    plot!(q_opt[:, 15], label = "foot 3", lw=4)
    plot!(q_opt[:, 18], label = "foot 4", lw=4)
    fig_file_path = joinpath(path, "opt_$(gait)_height.png")
    savefig(fig_file_path)

    # body euler angles
    plot(q_opt[:, 4], label = "yaw", title = "$(gait) euler angles", lw=4)
    plot!(q_opt[:, 5], label = "pitch", lw=4)
    plot!(q_opt[:, 6], label = "roll", lw=4)
    fig_file_path = joinpath(path, "opt_$(gait)_euler_angle.png")
    savefig(fig_file_path)

    # x 
    plot(q_opt[:, 1], label = "body", title = "$(gait) x", lw=4)
    plot!(q_opt[:, 7], label = "foot 1", lw=4)
    plot!(q_opt[:, 10], label = "foot 2", lw=4)
    plot!(q_opt[:, 13], label = "foot 3", lw=4)
    plot!(q_opt[:, 16], label = "foot 4", lw=4)
    fig_file_path = joinpath(path, "opt_$(gait)_x.png")
    savefig(fig_file_path)

    # y 
    plot(q_opt[:, 2], label = "body", title = "$(gait) y", lw=4)
    plot!(q_opt[:, 8], label = "foot 1", lw=4)
    plot!(q_opt[:, 11], label = "foot 2", lw=4)
    plot!(q_opt[:, 14], label = "foot 3", lw=4)
    plot!(q_opt[:, 17], label = "foot 4", lw=4)

    fig_file_path = joinpath(path, "opt_$(gait)_y.png")
    savefig(fig_file_path)

    # support polygon
    plot(q_opt[1:end-1, 1], q_opt[1:end-1, 2], label = "body", title = "$(gait) support polygon", lw=6)
    plot!(q_opt[1:end-1, 7], q_opt[1:end-1, 8], label = "foot 1", lw=6)
    plot!(q_opt[1:end-1, 10], q_opt[1:end-1, 11], label = "foot 2", lw=6)
    plot!(q_opt[1:end-1, 13], q_opt[1:end-1, 14], label = "foot 3", lw=6)
    plot!(q_opt[1:end-1, 16], q_opt[1:end-1, 17], label = "foot 4", lw=6)

    fig_file_path = joinpath(path, "opt_$(gait)_support_polygon.png")
    savefig(fig_file_path)

    # controls 
    u_opt = zeros(size(um)[1],size(um[1])[1])
    for i = 1:size(um)[1]
        u_opt[i,:] = Float64.(um[i]);
    end
    plot(u_opt[:,1], label = "", title = "$(gait) control", lw=4)
    for i = 2:size(um[1])[1]
        plot!(u_opt[:,i], label = "", lw=4)
    end
    fig_file_path = joinpath(path, "opt_$(gait)_control.png")
    savefig(fig_file_path)

    # contact γ
    γ_opt = zeros(size(γm)[1],size(γm[1])[1])
    for i = 1:size(γm)[1]
        γ_opt[i,:] = Float64.(γm[i]);
    end
    plot(γ_opt[:,1], label = "", title = "$(gait) contact", lw=4)
    for i = 2:size(γm[1])[1]
        plot!(γ_opt[:,i], label = "", lw=4)
    end
    fig_file_path = joinpath(path, "opt_$(gait)_contact.png")
    savefig(fig_file_path)

    # b? 
    b_opt = zeros(size(bm)[1],size(bm[1])[1])
    for i = 1:size(bm)[1]
        b_opt[i,:] = Float64.(bm[i]);
    end
    plot(b_opt[:,1], label = "", title = "$(gait) b", lw=4)
    for i = 2:size(bm[1])[1]
        plot!(b_opt[:,i], label = "", lw=4)
    end
    fig_file_path = joinpath(path, "opt_$(gait)_b.png")
    savefig(fig_file_path)

    # ψ
    ψ_opt = zeros(size(ψm)[1],size(ψm[1])[1])
    for i = 1:size(ψm)[1]
        ψ_opt[i,:] = Float64.(ψm[i]);
    end
    plot(ψ_opt[:,1], label = "", title = "$(gait) ψ", lw=4)
    for i = 2:size(ψm[1])[1]
        plot!(ψ_opt[:,i], label = "", lw=4)
    end
    fig_file_path = joinpath(path, "opt_$(gait)_ψ.png")
    savefig(fig_file_path)

    # η
    η_opt = zeros(size(ηm)[1],size(ηm[1])[1])
    for i = 1:size(ηm)[1]
        η_opt[i,:] = Float64.(ηm[i]);
    end
    plot(η_opt[:,1], label = "", title = "$(gait) η", lw=4)
    for i = 2:size(ηm[1])[1]
        plot!(η_opt[:,i], label = "", lw=4)
    end
    fig_file_path = joinpath(path, "opt_$(gait)_η.png")
    savefig(fig_file_path)

end
