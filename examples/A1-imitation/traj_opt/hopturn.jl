using ContactImplicitMPC
using JSON
using YAML

include(joinpath(@__DIR__, "..", "..", "..", "examples/centroidal_quadruped/reference/trajopt_model_v2.jl"))
include(joinpath(@__DIR__, "..", "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/utilities.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/plot_utils.jl"))

# make new directory to store results
result_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/results", "hopturn")
run_path = mk_new_dir(result_path)

ref_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/centroidal_ref_traj/hopturn.json")
config_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/traj_opt/config/hopturn.yaml")
q_ref, h, T = convert_q_from_json(ref_path);
# data = YAML.load_file(config_path; dicttype= Dict{String, Float64})
# h=0.05;

q1 = q_ref[1];
qT = q_ref[T+1];

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat");
model = s.model;
model.μ_world = 1;
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
            J += 0.5 * 1.0e-3 * dot(v, v);
            J += 100 * transpose(x[1:nx] - x_ref[t]) * Diagonal(1000.0 * ones(nx)) * (x[1:nx] - x_ref[t]);
            return J / T
        end
        push!(obj, DTO.Cost(objT, nx + nθ + nx, 0));
    elseif t == 1
        function obj1(x, u, w)
            J = 0.0;
            v = (x[model.nq .+ (1:model.nq)] - x[1:model.nq]) ./ h;
            J += 0.5 * 1.0e-3 * dot(v, v);
            J += 1 * transpose(x[1:nx] - x_ref[t]) * Diagonal(100.0 * ones(nx)) * (x[1:nx] - x_ref[t]);
            J += 0.5 * transpose(u[1:model.nu]) * Diagonal(1.0e-3 * ones(model.nu)) * u[1:model.nu];
            # J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)];

            J += 10000.0 * u[end]; # slack
            return J / T
        end
        push!(obj, DTO.Cost(obj1, nx, nu));
    else
        function objt(x, u, w)
            J = 0.0;
            v = (x[model.nq .+ (1:model.nq)] - x[1:model.nq]) ./ h;
            J += 0.5 * 1.0e-1 * dot(v, v);
            u_previous = x[nx .+ (1:53)];
            u_control = u;
            w = (u_control - u_previous) ./ h;
            J += 0.5 * 1.0e-3 * dot(w, w);
            J += 1 * transpose(x[1:nx] - x_ref[t]) * Diagonal(1000.0 * ones(nx)) * (x[1:nx] - x_ref[t]);
            J += 0.5 * transpose(u[1:model.nu]) * Diagonal(1.0e-3 * ones(model.nu)) * u[1:model.nu];
            # J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)];
            J += 10000.0 * u[end]; # slack
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

# bnd1 = DTO.Bound(nx, nu, state_lower=xl1, state_upper=xu1, action_lower=ul, action_upper=uu)
# bndt = DTO.Bound(nx + nθ + nx, nu, state_lower=xlt, state_upper=xut, action_lower=ul, action_upper=uu)
# bndT = DTO.Bound(nx + nθ + nx, 0, state_lower=xlT, state_upper=xuT)
# bnds = [bnd1, [bndt for t = 2:T-1]..., bndT];

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

            # body/feet constraints
            # x[3] - x_ref[t][3]; # body height
            # x[model.nq + 3] - x_ref[t][model.nq + 3]; # body height
            # x[9:3:18] - x_ref[t][9:3:18];
            ]
        end
        push!(cons, DTO.Constraint(constraints_1, nx, nu, indices_inequality=collect(16 .+ (1:28))))
    elseif t == T
        function constraints_T(x, u, w)
            [
            # inequality (8)
            contact_constraints_inequality_T(model, env, h, x, u, w);

            # body/feet constraints
            # x[3] - x_ref[t][3]; # body height
            # x[model.nq + 3] - x_ref[t][model.nq + 3]; # body height
            # x[9:3:18] - x_ref[t][9:3:18];
            ]
        end
        push!(cons, DTO.Constraint(constraints_T, nx + nθ + nx, nu, indices_inequality=collect(0 .+ (1:8))));
    else
        function constraints_t(x, u, w)
            [
            # equality (16)
            contact_constraints_equality(model, env, h, x, u, w);
            # inequality (32)
            contact_constraints_inequality_t(model, env, h, x, u, w);

            # body/feet constraints
            # x[3] - x_ref[t][3]; # body height
            # x[model.nq + 3] - x_ref[t][model.nq + 3]; # body height
            # x[9:3:18] - x_ref[t][9:3:18];
            ]
        end
        push!(cons, DTO.Constraint(constraints_t, nx + nθ + nx, nu, indices_inequality=collect(16 .+ (1:32))) );
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
Random.seed!(10)  ;      
# ## initialize
x_interpolation = [x_ref[1], [[x_ref[t]; zeros(nθ); zeros(nx)] for t = 2:T]...];
u_guess = [1.0e-1 * rand(nu) for t = 1:T-1]; # may need to run more than once to get good trajectory

DTO.initialize_states!(p, x_interpolation);
DTO.initialize_controls!(p, u_guess);

## solve
@time DTO.solve!(p);

# ## solution
x_sol, u_sol = DTO.get_trajectory(p);
@show max_slack = maximum([u[end] for u in u_sol[1:end-1]])
@show tot_slack = sum([u[end] for u in u_sol[1:end-1]])

save_to_jld2(model, x_sol, u_sol, "hopturn", tolerance,  run_path)
plt_opt_foot_height("hopturn", tolerance, run_path)

# ## visualize
vis = Visualizer();
render(vis);
visualize!(vis, model, [x_sol[1][1:nq], [x[nq .+ (1:nq)] for x in x_sol]...], Δt=h);
