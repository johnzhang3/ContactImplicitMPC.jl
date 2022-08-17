using ContactImplicitMPC

using JSON

include(joinpath(@__DIR__, "..", "..", "examples/centroidal_quadruped/reference/trajopt_model_v2.jl"))
include(joinpath(@__DIR__, "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))
include(joinpath("..", "..", "examples/A1-imitation/utils/utilities.jl"))
include(joinpath("..", "..", "examples/A1-imitation/utils/plot_utils.jl"))


ref_path = joinpath(@__DIR__, "..", "..", "examples/A1-imitation/centroidal_ref_traj/pace_forward.json")

cent_pace = JSON.parsefile(ref_path);
q_ref_any = cent_pace["Frames"];

# h = cent_pace["FrameDuration"];
h = 0.05
T = size(q_ref_any)[1] - 1

q_ref = zeros(39,18);

for i = 1:39
    q_ref[i,:] = Float64.(q_ref_any[i]);
end

q_ref = [q_ref[i,:] for i in 1:size(q_ref,1)];

# q_ref = [[Float64.(q_ref_any[t]); Float64.(q_ref_any[t+1])] for t = 1:T]

# vis = Visualizer()
# render(vis)
# visualize!(vis, model, q_ref, Δt=h);
# plot(hcat(q_ref...)', labels="")
# initial and final q
q1 = q_ref[1];
qT = q_ref[39]

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat");
model = s.model;
env = s.env;

Tm = 4;

nx = 2 * model.nq;
nc = 4 ; #model.nc
nu = model.nu + nc + 4 * nc + nc + 4 * nc + 1;
nθ = 53;

# ## model
d1 = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dyn1(model, env, [h], y, x, u, w), nx + nθ + nx, nx, nu);
dt = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dynt(model, env, [h], y, x, u, w), nx + nθ + nx, nx + nθ + nx, nu);

dyn = [d1, [dt for t = 2:T-1]...];

x_ref = [[q_ref[t]; q_ref[t+1]] for t = 1:T];
x1 = x_ref[1];
xM = x_ref[Tm];
xT = x_ref[T];

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
            J += 100 * transpose(x[1:nx] - x_ref[t]) * Diagonal(1000.0 * ones(nx)) * (x[1:nx] - x_ref[t]);
            J += 0.5 * transpose(u[1:model.nu]) * Diagonal(1.0e-3 * ones(model.nu)) * u[1:model.nu];
            J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)];

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
            J += 100 * transpose(x[1:nx] - x_ref[t]) * Diagonal(1000.0 * ones(nx)) * (x[1:nx] - x_ref[t]);
            J += 0.5 * transpose(u[1:model.nu]) * Diagonal(1.0e-3 * ones(model.nu)) * u[1:model.nu];
            J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)];
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
            x[3] - x_ref[t][3]; # body height
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
            x[3] - x_ref[t][3]; # body height
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
            x[3] - x_ref[t][3]; # body height
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
        max_iter=4000,
        max_cpu_time = 3000.0,
        tol=tolerance,
        constr_viol_tol=tolerance,
        
        ));


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
maximum([u[end] for u in u_sol[1:end-1]])
sum([u[end] for u in u_sol[1:end-1]])
save_to_jld2(model, x_sol, u_sol, "pace_forward", tolerance)
u_sol[]
plt_opt_foot_height("pace_forward", tolerance)


# tols = [1e-3,1e-4, 1e-5, 1e-6]
# u_sol = nothing

# for tol in tols
#     println("currently solving tolerance $(tol)")
#     p = DTO.Solver(dyn, obj, cons, bnds,
#     options=DTO.Options(
#         max_iter=1000000,
#         max_cpu_time = 30000.0,
#         tol=tol,
#         constr_viol_tol=tolerance,
#         ));
#     if tol == 1e-3
#         println("first solve!")
#         DTO.initialize_states!(p, x_interpolation);
#         DTO.initialize_controls!(p, u_guess);

#     else
#         DTO.initialize_states!(p, x_sol);
#         DTO.initialize_controls!(p, u_sol);
#     end
    
#     @time DTO.solve!(p);
#     # ## solution
#     global x_sol, u_sol = DTO.get_trajectory(p)
#     save_to_jld2(model, x_sol, u_sol, "pace_forward", tol)
#     plt_opt_foot_height("pace_forward", tol)
# end

# maximum([u[end] for u in u_sol[1:end-1]])

# ## visualize
vis = Visualizer();
render(vis);
visualize!(vis, model, [x_sol[1][1:nq], [x[nq .+ (1:nq)] for x in x_sol]...], Δt=h);


