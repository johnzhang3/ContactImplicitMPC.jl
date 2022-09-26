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
include("trajopt_model_wall_4feet.jl")

# Brings in the visuals for MeshCat, including the functions:
# build_robot!, set_robot!, visualize!
include(joinpath(@__DIR__, "..", "..", "..", "src/dynamics/centroidal_quadruped_wall/visuals.jl"))

# Create and open the Visualizer.
vis = LciMPC.Visualizer()
LciMPC.render(vis)

# Set horizon variables
T = 120
Tm = 30 #TODO: What is this?
h = 0.05

# Get simulation and model
s = get_simulation("centroidal_quadruped_wall", "flat_3D_lc", "flat")
model = s.model
env = s.env
nx = 2 * model.nq # Number of states
nc = model.nc # Number of contacts
nu = model.nu + nc + 4 * nc + nc + 4 * nc + 1 # Additional decision vars
nθ = 93

# Set up array of DTO dynamics structures
d1 = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dynt(
	model, env, [h], y, x, u, w), nx + nθ, nx, nu)
dt = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dynt(
	model, env, [h], y, x, u, w), nx + nθ , nx + nθ , nu)
dyn = [d1, [dt for t = 2:T-1]...]

# Initial conditions
body_height = 0.3
foot_x = 0.17
foot_y = 0.15

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

q1 = nominal_configuration(model)
qT = nominal_configuration(model)
visualize!(vis, model, [q1], Δt = h)

# Create reference trajectory
q_ref = [q1 for i in 1:T + 1];

# Create reference state for DTO
x_ref = [[q_ref[t]; q_ref[t+1]] for t = 1:T]

# Create array of costs
obj = DTO.Cost{Float64}[]
for t = 1:T
    if t == 1
        push!(obj, DTO.Cost((x, u, w) -> begin
            J = 0.0
            J += 0.5 * transpose(x[1:6] - x_ref[1][1:6]) * Diagonal(1000.0 * ones(6)) * (x[1:6] - x_ref[1][1:6])
            J += 0.5 * transpose(x[6 .+ 1:12] - x_ref[1][6 .+ 1:12]) * Diagonal(1000.0 * ones(6)) * (x[6 .+ 1:12] - x_ref[1][6 .+ 1:12])

            J += 0.5 * transpose(u) * Diagonal([1.0 * ones(model.nu); zeros(nu - model.nu)]) * u
            J += 10000.0 * u[end] # slack
            J += 0.5 * transpose(u[model.nu + 8 .+ (1:40)]) * Diagonal(1.0e-3 * ones(40)) * u[model.nu + 8 .+ (1:40)]

            return J
        end,
        nx, nu))
    elseif t == T
        push!(obj, DTO.Cost((x, u, w) -> begin
            J = 0.0
            J += 0.5 * transpose(x[1:6] - x_ref[T][1:6]) * Diagonal(1000.0 * ones(6)) * (x[1:6] - x_ref[T][1:6])
            J += 0.5 * transpose(x[6 .+ 1:12] - x_ref[T][6 .+ 1:12]) * Diagonal(1000.0 * ones(6)) * (x[6 .+ 1:12] - x_ref[T][6 .+ 1:12])


            J -= 100.0 * x[36 + model.nu + 5]
            return J
        end, nx + nθ, 0))
    else
        push!(obj, DTO.Cost((x, u, w) -> begin
            J = 0.0

            u_prev = x[nx .+ (1:nθ)]
            w = (u - u_prev) ./ h
            J += 0.5 * 1.0 * dot(w[1:end-1], w[1:end-1])

            J += 0.5 * transpose(x[1:6] - x_ref[t][1:6]) * Diagonal(1000.0 * ones(6)) * (x[1:6] - x_ref[t][1:6])
            J += 0.5 * transpose(x[6 .+ 1:12] - x_ref[t][6 .+ 1:12]) * Diagonal(100000.0 * ones(6)) * (x[6 .+ 1:12] - x_ref[t][6 .+ 1:12])

            J += 0.5 * transpose(u) * Diagonal([1.0 * ones(model.nu); zeros(nu - model.nu)]) * u
            J += 10000.0 * u[end] # slack
            J += 0.5 * transpose(u[model.nu + 8 .+ (1:40)]) * Diagonal(1.0e-3 * ones(40)) * u[model.nu + 8 .+ (1:40)]

            return J
        end, nx + nθ, nu))
    end
end

# Initial and intermediary state constraints
xl1 = [q1; q1]
xu1 = [q1; q1]
xlt = [-Inf * ones(nx); -Inf * ones(nθ)]
xut = [Inf * ones(nx); Inf * ones(nθ)]

# Final state constraints
xlT = [-Inf * ones(nx); -Inf * ones(nθ)]
xuT = [Inf * ones(nx); Inf * ones(nθ)]

# Other decision variable constraints
ul = [-Inf * ones(model.nu); zeros(nu - model.nu)]
uu = [Inf * ones(model.nu); Inf * ones(nu - model.nu)]

# Create DTO.Bound array for constraints
bnd1 = DTO.Bound(nx, nu, state_lower=xl1, state_upper=xu1, action_lower=ul, action_upper=uu)
bndt = DTO.Bound(nx + nθ , nu, state_lower=xlt, state_upper=xut, action_lower=ul, action_upper=uu)
bndT = DTO.Bound(nx + nθ , 0, state_lower=xlT, state_upper=xuT)
bnds = [bnd1, [bndt for t = 2:T-1]..., bndT];

# Contact constraints
contact_constraints_equality(model, env, h, rand(nx), rand(nu), zeros(0))
contact_constraints_inequality_1(model, env, h, rand(nx), rand(nu), zeros(0))
contact_constraints_inequality_t(model, env, h, rand(nx + nθ ), rand(nu), zeros(0))
contact_constraints_inequality_T(model, env, h, rand(nx + nθ ), rand(nu), zeros(0))

function constraints_1(x, u, w)
    [
     # equality (20)
     contact_constraints_equality(model, env, h, x, u, w);
     # inequality (35)
     contact_constraints_inequality_1(model, env, h, x, u, w);
     x[6 .+ (1:12)] - q1[6 .+ (1:12)];
     x[18 + 6 .+ (1:12)] - q1[6 .+ (1:12)];
    #  x[18 + 3] - q1[3];
    ]
end

function constraints_t(x, u, w)
    [
     # equality (20)
     contact_constraints_equality(model, env, h, x, u, w);
     # inequality (40)
     contact_constraints_inequality_t(model, env, h, x, u, w);
    #  x[6 .+ (4:12)] - q1[6 .+ (4:12)];
    #  x[18 + 6 .+ (4:12)] - q1[6 .+ (4:12)];
    #  x[18 + 3] - q1[3];
    ]
end

function constraints_T(x, u, w)
    [
     # inequality (10)
     contact_constraints_inequality_T(model, env, h, x, u, w);
     x[6 .+ (4:12)] - qT[6 .+ (4:12)];
     x[18 + 6 .+ (4:12)] - qT[6 .+ (4:12)];
     x[18 + 6 .+ 1] - qT[6 + 1];
     x[18 + 6 .+ 3] - qT[6 + 3];
    #  x[18 + 2] - qT[2];
    #  x[18 + 3] - qT[3];
    x[18 .+ (1:3)] - qT[1:3];
    ]
end

con1 = DTO.Constraint(constraints_1, nx, nu, indices_inequality=collect(32 .+ (1:56)))
cont = DTO.Constraint(constraints_t, nx + nθ , nu, indices_inequality=collect(32 .+ (1:64)))
conT = DTO.Constraint(constraints_T, nx + nθ , nu, indices_inequality=collect(0 .+ (1:16)))
cons = [con1, [cont for t = 2:T-1]..., conT];

# Set up DTO problem
direct_solver = DTO.Solver(dyn, obj, cons, bnds,
    options=DTO.Options(
        tol=1.0e-3,
        constr_viol_tol=1.0e-3,
        max_iter=1000,
        max_cpu_time = 2000.0
        ))

# Initialize problem
x_interpolation = copy(x_ref)
u_guess = [1.0e-4 * rand(nu) for t = 1:T-1] # may need to run more than once to get good trajectory
DTO.initialize_states!(direct_solver, x_interpolation)
DTO.initialize_controls!(direct_solver, u_guess)

# Solve the problem
@time DTO.solve!(direct_solver)

# Get solution
x_sol, u_sol = DTO.get_trajectory(direct_solver)

# Visualize solution
visualize!(vis, model, [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...], Δt=h)

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

# Save reference
using JLD2
@save joinpath(@__DIR__, "stand_wall_two_steps_v0.jld2") qm um γm bm ψm ηm μm hm