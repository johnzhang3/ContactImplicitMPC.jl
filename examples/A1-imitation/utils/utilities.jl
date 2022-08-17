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
    println(path)
    q_any = JSON.parsefile(path)["Frames"];
    T = size(q_any)[1] - 1
    h = JSON.parsefile(path)["FrameDuration"]
    q_ref = zeros(size(q_any)[1],18);

    for i = 1:size(q_any)[1]
        q_ref[i,:] = Float64.(q_any[i]);
    end

    q_ref = [q_ref[i,:] for i in 1:size(q_ref,1)];
    println(T)
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