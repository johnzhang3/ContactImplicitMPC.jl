# This is primarily a copy of stand_wall_two_steps.jl from 
# examples/centroidal_quadruped/reference
# This can only run in VSCode, with the ContactImplicitMPC environment
# in Julia AND the module set (similar to a namespace)

using DirectTrajectoryOptimization 
const DTO = DirectTrajectoryOptimization
using ContactImplicitMPC
const LciMPC = ContactImplicitMPC

# Creates the alias DTO for DirectTrajectoryOptimization
# Brings in the following functions for the DTO model:
# centroidal_quadruped_dyn, centroidal_quadruped_dynt,
# contact_constraints_inequality_1, contact_constraints_inequality_t,
# contact_constraints_inequality_T, contact_constraints_equality
include("trajopt_model_wall.jl")

# Brings in the visuals for MeshCat, including the functions:
# build_robot!, set_robot!, visualize!
include(joinpath(@__DIR__, "..", "..", "..", "src/dynamics/centroidal_quadruped_wall/visuals.jl"))

# Create and open the Visualizer.
vis = LciMPC.Visualizer()
LciMPC.open(vis)

# ## horizon
T = 20
h = 0.05

# Get simulation and model
s = get_simulation("centroidal_quadruped_wall", "flat_3D_lc", "flat")
model = s.model
env = s.env
nx = 2 * model.nq # Number of states
nc = model.nc # Number of contacts
nu = model.nu + nc + 4 * nc + nc + 4 * nc + 1 # Additional decision vars
nθ = 53

# Set up array of DTO dynamics structures
d1 = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dyn1(
	model, env, [h], y, x, u, w), nx + nθ + nx, nx, nu)
dt = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dynt(
	model, env, [h], y, x, u, w), nx + nθ + nx, nx + nθ + nx, nu)
dyn = [d1, [dt for t = 2:T-1]...]

# Initial conditions
body_height = 0.3
foot_x = 0.17
foot_y = 0.1

# Configurations through trajectory (will be interpolated between)
function nominal_configuration(model::CentroidalQuadrupedWall)
    x_shift = -0.0
    y_shift = -0.0
    return [
        0.0 + x_shift; y_shift; body_height; # Body XYZ
        0.0; 0.0; 0.0; # Body orientation (MRP)
        foot_x ; foot_y; 0.0; # Left front XYZ
        foot_x ;-foot_y; 0.0; # Right front XYZ
       -foot_x ; foot_y; 0.0; # Left back XYZ
       -foot_x ;-foot_y; 0.0; # Right back XYZ
    ]
end

function middle1_configuration(model::CentroidalQuadrupedWall)
    x_shift = -0.05
    y_shift = -0.0
    return [
        0.0 + x_shift; y_shift; body_height; # Body XYZ
        0.0; 0.0; 0.0; # Body orientation (MRP)
        foot_x ; foot_y; 0.0; # Left front XYZ
        foot_x ;-foot_y; 0.0; # Right front XYZ
       -foot_x ; foot_y; 0.0; # Left back XYZ
       -foot_x ;-foot_y; 0.0; # Right back XYZ
    ]
end

function sinusoidal_interpolation(q0, q1, N)
    Λ = (sin.(range(-π/2, π/2, length=N)) .+ 1) ./ 2
    Q = [q0*(1-λ) + q1*λ for λ in Λ]
    return Q
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

con1 = DTO.Constraint(constraints_1, nx, nu, indices_inequality=collect(16 .+ (1:28)))
cont = DTO.Constraint(constraints_t, nx + nθ + nx, nu, indices_inequality=collect(16 .+ (1:32)))
conT = DTO.Constraint(constraints_T, nx + nθ + nx, nu, indices_inequality=collect(0 .+ (1:8)))
cons = [con1, [cont for t = 2:T-1]..., conT];

# Set up DTO problem
direct_solver = DTO.Solver(dyn, obj, cons, bnds,
    options=DTO.Options(
        tol=1.0e-3,
        constr_viol_tol=1.0e-3,
        max_iter=100000,
        max_cpu_time = 60000.0
        ));

# ## initialize
x_interpolation = [x1, [[x1; zeros(nθ); zeros(nx)] for t = 2:T]...]
u_guess = [1.0e-4 * rand(nu) for t = 1:T-1] # may need to run more than once to get good trajectory
DTO.initialize_states!(direct_solver, x_interpolation)
DTO.initialize_controls!(direct_solver, u_guess)

# ## solve
@time DTO.solve!(direct_solver)

# Get solution
x_sol, u_sol = DTO.get_trajectory(direct_solver)

# Visualize solution
visualize!(vis, model, [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...], Δt=h)
vis
# Extract decision variables for CI-MPC
N_first = 10
N_last = 20
qm = [[x_sol[1][model.nq .+ (1:model.nq)] for t = 1:N_first]..., x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]..., [x_sol[end][model.nq .+ (1:model.nq)] for t = 1:N_last]...]
vm = [[(x_sol[1][model.nq .+ (1:model.nq)] - x_sol[1][0 .+ (1:model.nq)]) ./ h for t = 1:N_first]..., [(x[model.nq .+ (1:model.nq)] - x[0 .+ (1:model.nq)]) ./ h for x in x_sol]..., [(x_sol[end][model.nq .+ (1:model.nq)] - x_sol[end][0 .+ (1:model.nq)]) ./ h for t = 1:N_last]...]
um = [[u_sol[1][1:model.nu] for t = 1:N_first]..., [u[1:model.nu] for u in u_sol]..., [u_sol[end][1:model.nu] for t = 1:N_last]...]
γm = [[u_sol[1][model.nu .+ (1:model.nc)] for t = 1:N_first]..., [u[model.nu .+ (1:model.nc)] for u in u_sol]..., [u_sol[end][model.nu .+ (1:model.nc)] for t = 1:N_last]...]
bm = [[u_sol[1][model.nu + model.nc .+ (1:model.nc*4)] for t = 1:N_first]..., [u[model.nu + model.nc .+ (1:model.nc*4)] for u in u_sol]..., [u_sol[end][model.nu + model.nc .+ (1:model.nc*4)] for t = 1:N_last]...]
ψm = [[u_sol[1][model.nu + model.nc + model.nc*4 .+ (1:model.nc)] for t = 1:N_first]..., [u[model.nu + model.nc + model.nc*4 .+ (1:model.nc)] for u in u_sol]..., [u_sol[end][model.nu + model.nc + model.nc*4 .+ (1:model.nc)] for t = 1:N_last]...]
ηm = [[u_sol[1][model.nu + model.nc + model.nc*4 + model.nc .+ (1:model.nc*4)] for t = 1:N_first]..., [u[model.nu + model.nc + model.nc*4 + model.nc .+ (1:model.nc*4)] for u in u_sol]..., [u_sol[end][model.nu + model.nc + model.nc*4 + model.nc .+ (1:model.nc*4)] for t = 1:N_last]...]
μm = model.μ_world
hm = h

plot([qm[i][3] for i in 1:size(qm)[1]])

# Save reference
using JLD2
@save joinpath(@__DIR__, "stand_wall_two_steps_v1.jld2") qm um γm bm ψm ηm μm hm