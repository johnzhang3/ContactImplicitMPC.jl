include(joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/traj_opt/models/centroidal_quadruped.jl"))
include(joinpath(@__DIR__, "..", "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/utilities.jl"))
include(joinpath("..", "..", "..", "examples/A1-imitation/utils/plot_utils.jl"))

gait = "stand";

# make new directory to store results
result_path = joinpath(@__DIR__, "..", "..", "..", "examples/A1-imitation/results", gait)
run_path = mk_new_dir(result_path)

# ## horizon
T = 11
h = 0.10
# ## centroidal_quadruped
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env
nx = 2 * model.nq
nc = 4 # model.nc
nu = model.nu + nc + 4 * nc + nc + 4 * nc + 1 + model.nq
nθ = nu

# ## model
d1 = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dyn1(
	model, env, [h], y, x, u, w), nx + nθ + nx, nx, nu)
dt = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dynt(
	model, env, [h], y, x, u, w), nx + nθ + nx, nx + nθ + nx, nu)

dyn = [d1, [dt for t = 2:T-1]...]

# ## initial conditions

function nominal_configuration(model::CentroidalQuadruped)
    body_height = 0.5
    foot_x = 0.2
    foot_y = 0.3

    [
        0.0; 0.0; body_height;
        0.0; 0.0; 0.0;
        foot_x; foot_y; 0.0;
        foot_x;-foot_y; 0.0;
       -foot_x; foot_y; 0.0;
       -foot_x;-foot_y; 0.0;
    ]
end

function default_configuration()
    body_height = 0.3
    foot_x = 0.17
    foot_y = 0.15
    [
        0.0; 0.0; body_height;
        0.0; 0.0; 0.0;
        foot_x; foot_y; 0.0;
        foot_x;-foot_y; 0.0;
       -foot_x; foot_y; 0.0;
       -foot_x;-foot_y; 0.0;
    ]
end

q̄ = default_configuration()

q1 = nominal_configuration(model)
qM = nominal_configuration(model)
qT = nominal_configuration(model)
q_ref = nominal_configuration(model)

x1 = [q1; q1]
xM = [qM; qM]
xT = [qT; qT]
x_ref = [q_ref; q_ref]


# ## objective
function obj1(x, u, w)
    # Δ = zeros(nx)
    # Δ[1:model.nq] .= u[53 .+ (model.nq)]
    Δ = u[53 .+ (1:model.nq)]
	J = 0.0
    # q_ref = q_ref - Δ
    x_ref = [q_ref-Δ; q_ref-Δ]
    # println(x_ref)
	J += 0.5 * transpose(x[1:nx] - x_ref) * Diagonal(ones(nx)) * (x[1:nx] - x_ref)
    # J += 0.5 * transpose(x_ref-Δ) * Diagonal(ones(nx)) * (x_ref-Δ)
    # J += 0.5 * transpose(x[1:nx] - x_ref) * Diagonal(ones(nx)) * (x[1:nx] - x_ref)
	# J += 0.5 * transpose(u) * Diagonal([ones(model.nu); 100.0 *ones(nu - model.nu)]) * u
    J = 0.5 * transpose(u[1:model.nu]) * Diagonal(ones(model.nu)) * u[1:model.nu];
    J += 1000.0 * u[end] # slack
    # J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)]

	return J
end

function objt(x, u, w)
	J = 0.0
    Δ = u[53 .+ (1:model.nq)]
    # q_ref = q_ref - Δ
    x_ref = [q_ref-Δ; q_ref-Δ]
    
    # u_prev = x[nx .+ (1:53)]
    # w = (u - u_prev) ./ h
    # J += 0.5 * 10.0 * dot(w[1:end-1], w[1:end-1])
    J += 0.5 * transpose(x[1:nx] - x_ref) * Diagonal(ones(nx)) * (x[1:nx] - x_ref)
	# J += 0.5 * transpose(x[1:nx] - x_ref) * Diagonal(ones(nx)) * (x[1:nx] - x_ref)
	# J += 0.5 * transpose(u) * Diagonal([ones(model.nu); 100.0 *ones(nu - model.nu)]) * u
    J = 0.5 * transpose(u[1:model.nu]) * Diagonal(ones(model.nu)) * u[1:model.nu];
    J = 0.5 * 1000* transpose(q_ref-Δ - q̄) * Diagonal(ones(model.nq)) * (q_ref-Δ - q̄)
    J += 1000.0 * u[end] # slack
    # J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)]

	return J
end

function objT(x, u, w)
	J = 0.0
    # Δ = u[53 .+ (1:model.nq*2)]
	J += 0.5 * transpose(x[1:nx] - x_ref) * Diagonal(ones(nx)) * (x[1:nx] - x_ref)
    # J += 0.5 * transpose(x[1:nx] - x_ref + Δ) * Diagonal(ones(nx)) * (x[1:nx] - x_ref + Δ)
    return J
end

c1 = DTO.Cost(obj1, nx, nu)
ct = DTO.Cost(objt, nx + nθ + nx, nu)
cT = DTO.Cost(objT, nx + nθ + nx, 0)
obj = [c1, [ct for t = 2:T-1]..., cT];

# ## constraints
ql = q1
qu = q1

# initial condition
xl1 = [q1; q1]
xu1 = [q1; q1]
xlt = [-Inf * ones(nx); -Inf * ones(nθ); -Inf * ones(nx)]
xut = [Inf * ones(nx); Inf * ones(nθ); Inf * ones(nx)]

# final condition
xlT = [q1; q1; -Inf * ones(nθ); -Inf * ones(nx)]
xuT = [q1; q1; Inf * ones(nθ); Inf * ones(nx)]

ul = [-Inf * ones(model.nu); zeros(nu - model.nu)]
uu = [Inf * ones(model.nu); Inf * ones(nu - model.nu)]

bnd1 = DTO.Bound(nx, nu, state_lower=xl1, state_upper=xu1, action_lower=ul, action_upper=uu)
bndt = DTO.Bound(nx + nθ + nx, nu, state_lower=xlt, state_upper=xut, action_lower=ul, action_upper=uu)
bndT = DTO.Bound(nx + nθ + nx, 0, state_lower=xlT, state_upper=xuT)
bnds = [bnd1, [bndt for t = 2:T-1]..., bndT];

function constraints_1(x, u, w)
    [
     # equality (16)
     contact_constraints_equality(model, env, h, x, u, w);
     # inequality (28)
     contact_constraints_inequality_1(model, env, h, x, u, w);
    #  x[6 .+ (1:12)] - q1[6 .+ (1:12)];
    #  x[18 + 6 .+ (1:12)] - q1[6 .+ (1:12)];
    ]
end

function constraints_t(x, u, w)
    [
     # equality (16)
     contact_constraints_equality(model, env, h, x, u, w);
     # inequality (32)
     contact_constraints_inequality_t(model, env, h, x, u, w);
    #  x[6 .+ (1:12)] - q1[6 .+ (1:12)];
    #  x[18 + 6 .+ (1:12)] - q1[6 .+ (1:12)];
    ]
end

function constraints_T(x, u, w)
    [
     # inequality (8)
     contact_constraints_inequality_T(model, env, h, x, u, w);
    #  x[6 .+ (1:12)] - q1[6 .+ (1:12)];
    #  x[18 + 6 .+ (1:12)] - q1[6 .+ (1:12)];
    ]
end

con1 = DTO.Constraint(constraints_1, nx, nu, indices_inequality=collect(16 .+ (1:28)))
cont = DTO.Constraint(constraints_t, nx + nθ + nx, nu, indices_inequality=collect(16 .+ (1:32)))
conT = DTO.Constraint(constraints_T, nx + nθ + nx, nu, indices_inequality=collect(0 .+ (1:8)))
cons = [con1, [cont for t = 2:T-1]..., conT];

tolerance = 1.0e-3
# ## problem
direct_solver = DTO.Solver(dyn, obj, cons, bnds,
    options=DTO.Options(
        tol=tolerance,
        constr_viol_tol=tolerance,
        ))

# ## initialize
x_interpolation = [x1, [[x1; zeros(nθ); zeros(nx)] for t = 2:T]...]
u_guess = [1.0e-4 * rand(nu) for t = 1:T-1] # may need to run more than once to get good trajectory
DTO.initialize_states!(direct_solver, x_interpolation)
DTO.initialize_controls!(direct_solver, u_guess)

# ## solve
@time DTO.solve!(direct_solver)

# ## solution
x_sol, u_sol = DTO.get_trajectory(direct_solver);
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
