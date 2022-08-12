# ## model
include("trajopt_model_v2.jl")
include(joinpath(@__DIR__, "..", "..", "..", "src/dynamics/centroidal_quadruped/visuals.jl"))
vis = Visualizer()
open(vis)

# ## horizon
T = 11
h = 0.1

# ## centroidal_quadruped
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env
nx = 2 * model.nq
nc = 4 # model.nc
nu = model.nu + nc + 4 * nc + nc + 4 * nc + 1
nθ = 53

# ## model
d1 = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dyn1(
	model, env, [h], y, x, u, w), nx + nθ + nx, nx, nu)
dt = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dynt(
	model, env, [h], y, x, u, w), nx + nθ + nx, nx + nθ + nx, nu)

dyn = [d1, [dt for t = 2:T-1]...]

# ## initial conditions
body_height = 0.3
foot_x = 0.17
foot_y = 0.17

function nominal_configuration(model::CentroidalQuadruped)
    [
        0.0; 0.0; body_height;
        0.0; 0.0; 0.0;
        foot_x; foot_y; 0.0;
        foot_x;-foot_y; 0.0;
       -foot_x; foot_y; 0.0;
       -foot_x;-foot_y; 0.0;
    ]
end

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
	J = 0.0
	J += 0.5 * transpose(x[1:nx] - x_ref) * Diagonal(ones(nx)) * (x[1:nx] - x_ref)
	J += 0.5 * transpose(u) * Diagonal([ones(model.nu); 100.0 *ones(nu - model.nu)]) * u
    J += 1000.0 * u[end] # slack
    J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)]

	return J
end

function objt(x, u, w)
	J = 0.0

    u_prev = x[nx .+ (1:53)]
    w = (u - u_prev) ./ h
    J += 0.5 * 10.0 * dot(w[1:end-1], w[1:end-1])

	J += 0.5 * transpose(x[1:nx] - x_ref) * Diagonal(ones(nx)) * (x[1:nx] - x_ref)
	J += 0.5 * transpose(u) * Diagonal([ones(model.nu); 100.0 *ones(nu - model.nu)]) * u
    J += 1000.0 * u[end] # slack
    J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)]

	return J
end

function objT(x, u, w)
	J = 0.0
	J += 0.5 * transpose(x[1:nx] - x_ref) * Diagonal(ones(nx)) * (x[1:nx] - x_ref)
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

bnd1 = DTO.Bound(nx, nu, xl=xl1, xu=xu1, ul=ul, uu=uu)
bndt = DTO.Bound(nx + nθ + nx, nu, xl=xlt, xu=xut, ul=ul, uu=uu)
bndT = DTO.Bound(nx + nθ + nx, 0, xl=xlT, xu=xuT)
bnds = [bnd1, [bndt for t = 2:T-1]..., bndT];

function constraints_1(x, u, w)
    [
     # equality (16)
     contact_constraints_equality(model, env, h, x, u, w);
     # inequality (28)
     contact_constraints_inequality_1(model, env, h, x, u, w);
     x[6 .+ (1:12)] - q1[6 .+ (1:12)];
     x[18 + 6 .+ (1:12)] - q1[6 .+ (1:12)];
    ]
end

function constraints_t(x, u, w)
    [
     # equality (16)
     contact_constraints_equality(model, env, h, x, u, w);
     # inequality (32)
     contact_constraints_inequality_t(model, env, h, x, u, w);
     x[6 .+ (1:12)] - q1[6 .+ (1:12)];
     x[18 + 6 .+ (1:12)] - q1[6 .+ (1:12)];
    ]
end

function constraints_T(x, u, w)
    [
     # inequality (8)
     contact_constraints_inequality_T(model, env, h, x, u, w);
     x[6 .+ (1:12)] - q1[6 .+ (1:12)];
     x[18 + 6 .+ (1:12)] - q1[6 .+ (1:12)];
    ]
end

con1 = DTO.Constraint(constraints_1, nx, nu, idx_ineq=collect(16 .+ (1:28)))
cont = DTO.Constraint(constraints_t, nx + nθ + nx, nu, idx_ineq=collect(16 .+ (1:32)))
conT = DTO.Constraint(constraints_T, nx + nθ + nx, nu, idx_ineq=collect(0 .+ (1:8)))
cons = [con1, [cont for t = 2:T-1]..., conT];

# ## problem
direct_solver = DTO.solver(dyn, obj, cons, bnds,
    options=DTO.Options(
        tol=1.0e-3,
        constr_viol_tol=1.0e-3,
        ))

# ## initialize
x_interpolation = [x1, [[x1; zeros(nθ); zeros(nx)] for t = 2:T]...]
u_guess = [1.0e-4 * rand(nu) for t = 1:T-1] # may need to run more than once to get good trajectory
DTO.initialize_states!(direct_solver, x_interpolation)
DTO.initialize_controls!(direct_solver, u_guess)

# ## solve
@time DTO.solve!(direct_solver)

# ## solution
x_sol, u_sol = DTO.get_trajectory(direct_solver)
@show x_sol[1]
@show x_sol[T]
maximum([u[nu] for u in u_sol[1:end-1]])

# ## visualize
# q_sol = state_to_configuration([x[1:nx] for x in x_sol])
visualize!(vis, model, x_sol, Δt=h)

q_opt = [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...]
v_opt = [(x[model.nq .+ (1:model.nq)] - x[0 .+ (1:model.nq)]) ./ h for x in x_sol]
u_opt = [u[1:model.nu] for u in u_sol]
λ_opt = [u[model.nu .+ (1:4)] for u in u_sol]
b_opt = [u[model.nu + 4 .+ (1:16)] for u in u_sol]


q_opt = [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...]
v_opt = [(x[model.nq .+ (1:model.nq)] - x[0 .+ (1:model.nq)]) ./ h for x in x_sol]
u_opt = [u[1:model.nu] for u in u_sol]
γ_opt = [u[model.nu .+ (1:4)] for u in u_sol]
b_opt = [u[model.nu + 4 .+ (1:16)] for u in u_sol]
ψ_opt = [u[model.nu + 4 + 16 .+ (1:4)] for u in u_sol]
η_opt = [u[model.nu + 4 + 16 + 4 .+ (1:16)] for u in u_sol]

qm = q_opt
vm = v_opt
um = u_opt
γm = γ_opt
bm = b_opt
ψm = ψ_opt
ηm = η_opt

μm = model.μ_world
hm = h
timesteps = range(0.0, stop=(h * (length(qm) - 2)), length=(length(qm) - 2))
plot(timesteps, hcat(qm[2:end-1]...)', labels="")
plot(timesteps, hcat(um...)', labels="")
plot(timesteps, hcat(γm...)' ./ h, labels="")
plot(timesteps, hcat(bm...)', labels="")
plot(timesteps, hcat(ψm...)', labels="")
plot(timesteps, hcat(ηm...)', labels="")

using JLD2
@save joinpath(@__DIR__, "stand_10Hz.jld2") qm um γm bm ψm ηm μm hm
@load joinpath(@__DIR__, "stand_10Hz.jld2") qm um γm bm ψm ηm μm hm
