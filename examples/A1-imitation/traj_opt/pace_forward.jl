using ContactImplicitMPC
using JSON
using YAML

include(joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/traj_opt/models/centroidal_quadruped.jl"))
include(joinpath(@__DIR__, "..", "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/utilities.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/plot_utils.jl"))

gait = "pace_forward";

# make new directory to store results
result_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/results", gait)
run_path = mk_new_dir(result_path)

ref_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/centroidal_ref_traj/$(gait).json")
config_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/traj_opt/config/$(gait).yaml")
q_ref, h, T = convert_q_from_json(ref_path);

function adjust_ref!(q_ref)
    # rotate and offset reference trajectory
    for i = 1:size(q_ref)[1]
        # body 
        q_ref[i][1] = q_ref[i][1] 
        q_ref[i][2] = q_ref[i][2] 
        q_ref[i][3] = q_ref[i][3]

        # feet y
        q_ref[i][8] = q_ref[i][8] + 0.05
        q_ref[i][11] = q_ref[i][11] - 0.05
        q_ref[i][14] = q_ref[i][14] + 0.05
        q_ref[i][17] = q_ref[i][17] - 0.05
        
    end
end
adjust_ref!(q_ref)
plt_ref_traj(q_ref, run_path, "refFootHeight")
weights_dict = YAML.load_file(config_path; dicttype= Dict{String, Float64});


h=0.05;
q1 = deepcopy(q_ref[1]);
q1[9] = 0.0;
# q1[12] = 0.0;
q1[15] = 0.0;
# q1[18] = 0.0;
qT = deepcopy(q_ref[end]);
qT[9] = 0.0;
# qT[12] = 0.0;
qT[15] = 0.0;
# qT[18] = 0.0;

pushfirst!(q_ref, q1);
push!(q_ref, qT);

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat");
model = s.model;
env = s.env;

nx = 2 * model.nq;
nc = 4; #model.nc
nu = model.nu + nc + 4 * nc + nc + 4 * nc + 1;
nθ = 53;

# ## model
d1 = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dyn1(model, env, [h], y, x, u, w), nx + nθ + nx, nx, nu);
dt = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dynt(model, env, [h], y, x, u, w), nx + nθ + nx, nx + nθ + nx, nu);

dyn = [d1, [dt for t = 2:T-1]...];

x_ref = [[q_ref[t]; q_ref[t+1]] for t = 1:T];

# ## objective
obj = DTO.Cost{Float64}[];
for t = 1:T
    if t == T
        function objT(x, u, w)
            J = 0.0;
            v = (x[model.nq .+ (1:model.nq)] - x[1:model.nq]) ./ h;
            J += 0.5 * weights_dict["weight_v_T"] * dot(v, v);
            J += weights_dict["weight_x_T"] * transpose(x[1:nx] - x_ref[t]) * Diagonal(ones(nx)) * (x[1:nx] - x_ref[t]);
            return J / T
        end
        push!(obj, DTO.Cost(objT, nx + nθ + nx, 0));
    elseif t == 1
        function obj1(x, u, w)
            J = 0.0;
            v = (x[model.nq .+ (1:model.nq)] - x[1:model.nq]) ./ h;
            J += 0.5 * weights_dict["weight_v_i"] * dot(v, v);
            J += weights_dict["weight_x_i"] * transpose(x[1:nx] - x_ref[t]) * Diagonal(ones(nx)) * (x[1:nx] - x_ref[t]);
            J += 0.5 * weights_dict["weight_u_i"] * transpose(u[1:model.nu]) * Diagonal(ones(model.nu)) * u[1:model.nu];
            # J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)];

            J += weights_dict["weight_s_i"] * u[end]; # slack
            return J / T
        end
        push!(obj, DTO.Cost(obj1, nx, nu));
    else
        function objt(x, u, w)
            J = 0.0;
            v = (x[model.nq .+ (1:model.nq)] - x[1:model.nq]) ./ h;
            J += 0.5 * weights_dict["weight_v_t"] * dot(v, v);
            u_previous = x[nx .+ (1:53)];
            u_control = u;
            w = (u_control - u_previous) ./ h;
            J += 0.5 * weights_dict["weight_w_t"] * dot(w, w);
            J += weights_dict["weight_x_t"] * transpose(x[1:nx] - x_ref[t]) * Diagonal(ones(nx)) * (x[1:nx] - x_ref[t]);
            J += 0.5 * weights_dict["weight_u_t"] * transpose(u[1:model.nu]) * Diagonal(ones(model.nu)) * u[1:model.nu];
            # J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)];
            J += weights_dict["weight_s_t"] * u[end]; # slack
            return J / T
        end
        push!(obj, DTO.Cost(objt, nx + nθ + nx, nu));
    end
end

# ## constraints
# initial condition
xl1 = [q1; q1];
xu1 = [q1; q1];
xlt = [-Inf * ones(nx); -Inf * ones(nθ); -Inf * ones(nx)];
xut = [Inf * ones(nx); Inf * ones(nθ); Inf * ones(nx)];

# final condition
xlT = [-Inf * ones(nq); qT; -Inf * ones(nθ); -Inf * ones(nx)];
xuT = [Inf * ones(nq); qT; Inf * ones(nθ); Inf * ones(nx)];

ul = [-Inf * ones(model.nu); zeros(nu - model.nu)];
uu = [Inf * ones(model.nu); Inf * ones(nu - model.nu)];

bnds = DTO.Bound{Float64}[];
push!(bnds, DTO.Bound(nx, nu, state_lower=xl1, state_upper=xu1, action_lower=ul, action_upper=uu));
for t = 2:T-1
    push!(bnds, DTO.Bound(nx + nθ + nx, nu,
        state_lower=[-Inf * ones(nq); -Inf * ones(nq); -Inf * ones(nθ); -Inf * ones(nx)],
        state_upper=[Inf * ones(nq); Inf * ones(nq); Inf * ones(nθ); Inf * ones(nx)],
        action_lower=ul, action_upper=uu));
end
push!(bnds, DTO.Bound(nx + nθ + nx, 0, state_lower=xlT, state_upper=xuT));


cons = DTO.Constraint{Float64}[];
for t = 1:T
    if t == 1
        function constraints_1(x, u, w)
            [
            # equality (16)
            contact_constraints_equality(model, env, h, x, u, w);
            # inequality (28)
            contact_constraints_inequality_1(model, env, h, x, u, w);
            # inequality (16)
            feet_position_inequality(model, env, h, x, u, w);
            ]
        end
        push!(cons, DTO.Constraint(constraints_1, nx, nu, indices_inequality=collect(16 .+ (1:28+16))))
    elseif t == T
        function constraints_T(x, u, w)
            [
            # inequality (8)
            contact_constraints_inequality_T(model, env, h, x, u, w);
            # inequality (16)
            feet_position_inequality(model, env, h, x, u, w);
            ]
        end
        push!(cons, DTO.Constraint(constraints_T, nx + nθ + nx, nu, indices_inequality=collect(0 .+ (1:8+16))));
    else
        function constraints_t(x, u, w)
            [
            # equality (16)
            contact_constraints_equality(model, env, h, x, u, w);
            # inequality (32)
            contact_constraints_inequality_t(model, env, h, x, u, w);
            # inequality (16)
            feet_position_inequality(model, env, h, x, u, w);
            ]
        end
        push!(cons, DTO.Constraint(constraints_t, nx + nθ + nx, nu, indices_inequality=collect(16 .+ (1:32+16))) );
    end
end

# ## problem
tolerance = 1.0e-3;
p = DTO.Solver(dyn, obj, cons, bnds,
    options=DTO.Options(
        max_iter=10000,
        max_cpu_time = 30000.0,
        tol=tolerance,
        constr_viol_tol=tolerance));

using Random
Random.seed!(Int(weights_dict["seed"]))  ;      
# ## initialize
x_interpolation = [x_ref[1], [[x_ref[t]; zeros(nθ); zeros(nx)] for t = 2:T]...];
u_guess = [1.0e-1 * rand(nu) for t = 1:T-1]; # may need to run more than once to get good trajectory

DTO.initialize_states!(p, x_interpolation);
DTO.initialize_controls!(p, u_guess);

## solve
@time DTO.solve!(p);

# ## solution
x_sol, u_sol = DTO.get_trajectory(p);
@show max_slack = maximum([u[end] for u in u_sol[1:end-1]]);
@show tot_slack = sum([u[end] for u in u_sol[1:end-1]]);

save_to_jld2(model, x_sol, u_sol, gait, tolerance,  run_path);

plt_opt_results(gait, tolerance, run_path)
YAML.write_file(joinpath(run_path, "config.yaml"), weights_dict);

# ## visualize
vis = Visualizer();
render(vis);
visualize!(vis, model, [x_sol[1][1:nq], [x[nq .+ (1:nq)] for x in x_sol]...], Δt=h);
save_IPOPT_output(run_path)