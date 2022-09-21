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

function save_jld2_to_txt(path, gait, tol)
    data_path = joinpath(path, "$(gait)_tol$(tol).jld2")
    txt_path = joinpath(path, "$(gait)_tol$(tol).txt")
    @load data_path qm um γm bm ψm ηm μm hm
    dict = Dict("FrameDuration" => hm,
    "Frames" =>qm)
    open(txt_path, "w") do file
        JSON.print(file, dict)
    end
end

function convert_q_from_json(path, ft_flip)
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

        if ft_flip == true
            # TODO: resolve this hack
            
            q_ref[i,:] = cat(body, FL, FR, BL, BR, dims=1)
        else
            q_ref[i, :] = cat(body, FR, FL, BL, BR, dims=1)
        end
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


function huber_obj(model, nx, nu, nθ, T, α, weights)
    obj = DTO.cost{Float64}[]
    for t = 1:T
        if t == T
            function objT(x, u, w)
                J = 0.0
                

            end
        elseif t == 1
            function obj1(x, u, w)
                J = 0.0
            end
        else
            function objt(x, u, w)
                J = 0.0
            end
        end
    end
end

function huber_cost(x_ref, x, α, nx)
    abs_diff = abs.(x - x_ref)
    # J = 0.0
    if sum(abs_diff) < α
        # quadratic
        return 0.5 * sum(abs_diff)^2 
    else
        # linear 
        return α * sum(abs_diff) - 0.5 * α^2
    end
    # println(typeof(abs_diff))
    # mask = abs_diff .> α
    # println(mask)
    # huber = abs_diff .* mask
    # huber = sum(abs_diff[mask]) 
    # println(typeof(huber))
    # quadratic = abs_diff .* .!mask 
    # quadratic = sum(abs_diff .* .!mask)
    # J = 0.0
    # # pritln("hello")
    # J += 0.5 * quadratic^2
    # J += 0.5 * transpose(quadratic) * Diagonal(ones(nx)) * (quadratic)
    # J += 0.5 * transpose(abs_diff) * Diagonal(ones(nx)) * (abs_diff)
    # J += transpose(huber) * ones(nx)* α - 0.5 * α^2
    # return J 
end

