using JSON

function save_to_jld2(model, x_sol, u_sol, gait, tol, path)
    q_opt = [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...]
    v_opt = [(x[model.nq .+ (1:model.nq)] - x[0 .+ (1:model.nq)]) ./ h for x in x_sol]
    u_opt = [u[1:model.nu] for u in u_sol]
    γ_opt = [u[model.nu .+ (1:4)] for u in u_sol]
    b_opt = [u[model.nu + 4 .+ (1:16)] for u in u_sol]
    ψ_opt = [u[model.nu + 4 + 16 .+ (1:4)] for u in u_sol]
    η_opt = [u[model.nu + 4 + 16 + 4 .+ (1:16)] for u in u_sol]

    qm = copy(q_opt)
    um = copy(u_opt)
    γm = copy(γ_opt)
    bm = copy(b_opt)
    ψm = copy(ψ_opt)
    ηm = copy(η_opt)
    μm = model.μ_world

    hm = h
    save_path = joinpath(path, "$(gait)_tol$(tol).jld2")
    @save save_path qm um γm bm ψm ηm μm hm

end

function convert_q_from_json(path)
    # input:  path of json fine
    # output: q as vector of vecotor Float64
    q_any = JSON.parsefile(path)["Frames"];
    T = size(q_any)[1] - 1
    h = JSON.parsefile(path)["FrameDuration"]
    q_ref = zeros(size(q_any)[1],18);

    for i = 1:size(q_any)[1]
        q = Float64.(q_any[i])
        body = q[1:6]
        FR = q[6 .+ (1:3)]
        FL = q[9 .+ (1:3)]
        BR = q[12 .+ (1:3)]
        BL = q[15 .+ (1:3)]
        FR[2] = deepcopy(FR[2]*1.7)
        FL[2] = deepcopy(FL[2]*1.7)
        BR[2] = deepcopy(BR[2]*1.7)
        BL[2] = deepcopy(BL[2]*1.7)
        q_ref[i,:] = cat(body, FL, FR, BL, BR, dims=1)
    end

    q_ref = [q_ref[i,:] for i in 1:size(q_ref,1)];

    return q_ref, h, T
end

function mk_new_dir(path)
    run_num = 1
    run_dir = joinpath(path, "run$(run_num)")
    while isdir(run_dir)
        run_num += 1
        run_dir = joinpath(path, "run$(run_num)")
    end
    mkdir(run_dir)
    return run_dir
end

function save_IPOPT_output(dst_path)
    # assume ouput.txt is in ContactImplicitMPC.jl 
    #TODO: figure out why output.txt is stored here

    output_src = joinpath(@__DIR__, "../../../output.txt")
    dst_path = joinpath(dst_path, "output.txt")
    mv(output_src, dst_path)
end

function world_to_body_frame(q)
    R = 1
end

function rotation_matrix(euler_angle)
    ϕ = euler_angle[1]
    θ = euler_angle[2]
    ψ = euler_angle[3]

    return LinearMap([cos(ψ)cos(θ) cos(ψ)sin(θ)sin(ϕ)-sin(ψ)cos(ϕ) cos(ψ)()])
end

# q = Vector{Float64}([0,0,0.3, 0.1, 0.12, 0.0, 0.19, -0.126, 0.05])
# pos = q[1:3]
# rot = q[4:6]
# f1 = q[7:9]

# R = RotXYZ(q[4], q[5], q[6])

# trans = Translation(-q[1], -q[2], -q[3])
# trans = recenter(trans, pos)
# rot = LinearMap(RotXYZ(q[4], q[5], q[6]))
# rot = recenter(rot, pos)
# composed = compose(trans, rot)

# composed(f1)
# composed(pos)

# composed2 = compose(rot, trans)
# f_pos = composed2(f1)
# norm(-f_pos)
# composed2(pos)
