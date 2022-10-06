# This is primarily a copy of stand.jl from 
# examples/A1-imitation/traj_opt/
# This can only run in VSCode, with the ContactImplicitMPC environment
# in Julia AND the module set (similar to a namespace)
#
# Used to test the centroidal_quadruped_wall model for a dynamic case

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
T = 30
Tm = 10
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

function middle2_configuration(model::CentroidalQuadrupedWall)
    x_shift = -0.05
    y_shift = -0.0
    return [
        0.0 + x_shift; y_shift; body_height; # Body XYZ
        0.0; 0.0; 0.0; # Body orientation (MRP)
        0.3  ; foot_y; 0.2; # Left front XYZ
        foot_x ;-foot_y; 0.0; # Right front XYZ
       -foot_x ; foot_y; 0.0; # Left back XYZ
       -foot_x ;-foot_y; 0.0; # Right back XYZ
    ]
end

function middle3_configuration(model::CentroidalQuadrupedWall)
    x_shift = -0.05
    y_shift = -0.0
    return [
        0.0 + x_shift; y_shift; body_height; # Body XYZ
        0.0; 0.0; 0.0; # Body orientation (MRP)
        0.35  ; foot_y; 0.2; # Left front XYZ
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
qT = middle3_configuration(model)
visualize!(vis, model, [q1], Δt = h)
qM1 = middle1_configuration(model)
qM2 = nomina2_configuration(model)
visualize!(vis, model, [qM1], Δt = h)

# Create reference trajectory
q_ref = [sinusoidal_interpolation(q1, qM1, Tm)...,
    sinusoidal_interpolation(qM1, qM2, Tm)...,
    sinusoidal_interpolation(qM2, qT, Tm)...];
q_ref = [q1, q_ref...]
visualize!(vis, model, q_ref, Δt = h)
vis
# Create reference state for DTO
x_ref = [[q_ref[t]; q_ref[t+1]] for t = 1:T]

# Create array of objectives/cost
obj = DTO.Cost{Float64}[]
for t = 1:T
    if t == T
        push!(obj, DTO.Cost((x, u, w) -> begin
            J = 0.0
            J += 100 * transpose(x[1:nx] - x_ref[T]) * Diagonal(1000.0 * ones(nx)) * (x[1:nx] - x_ref[T])
            
            return J
        end, nx + nθ + nx, 0))
    elseif t == 1
        push!(obj, DTO.Cost((x, u, w) -> begin
            J = 0.0
            v = (x[model.nq .+ (1:model.nq)] - x[1:model.nq]) ./ h
            J += 0.5 * 1.0e-3 * dot(v, v)
            J += 100 * transpose(x[1:nx] - x_ref[1]) * Diagonal(1000.0 * ones(nx)) * (x[1:nx] - x_ref[1])
	        J += 0.5 * transpose(u[1:model.nu]) * Diagonal(1.0e-3 * ones(model.nu)) * u[1:model.nu]
            J += 1000.0 * u[end] # slack
            J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)]

            return J
        end, nx, nu))
    else
        push!(obj, DTO.Cost((x, u, w) -> begin
            J = 0.0
            v = (x[model.nq .+ (1:model.nq)] - x[1:model.nq]) ./ h
            J += 0.5 * 1.0e-1 * dot(v, v)
            u_previous = x[nx .+ (1:53)]
            u_control = u
            w = (u_control - u_previous) ./ h
            J += 0.5 * 1.0e-3 * dot(w, w)
            J += 100 * transpose(x[1:nx] - x_ref[t]) * Diagonal(1000.0 * ones(nx)) * (x[1:nx] - x_ref[t])
            J += 0.5 * transpose(u[1:model.nu]) * Diagonal(1.0e-3 * ones(model.nu)) * u[1:model.nu]
            J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)]
            J += 1000.0 * u[end] # slack
            return J
        end, nx + nθ + nx, nu))
    end
end

# ## constraints
ql = q1
qu = q1

# initial condition
xl1 = [q1; q1]
xu1 = [q1; q1]
xlt = [-Inf * ones(nx); -Inf * ones(nθ); -Inf * ones(nx)]
xut = [Inf * ones(nx); Inf * ones(nθ); Inf * ones(nx)]

# final condition
xlT = [-Inf * ones(nq); qT; -Inf * ones(nθ); -Inf * ones(nx)]
xuT = [Inf * ones(nq); qT; Inf * ones(nθ); Inf * ones(nx)]

ul = [-Inf * ones(model.nu); zeros(nu - model.nu)]
uu = [Inf * ones(model.nu); Inf * ones(nu - model.nu)]

bnds = DTO.Bound{Float64}[]
push!(bnds, DTO.Bound(nx, nu, state_lower=xl1, state_upper=xu1, action_lower=ul, action_upper=uu))
for t = 2:T-1
    push!(bnds, DTO.Bound(nx + nθ + nx, nu,
        state_lower=[-Inf * ones(nq); -Inf * ones(nq); -Inf * ones(nθ); -Inf * ones(nx)],
        state_upper=[Inf * ones(nq); Inf * ones(nq); Inf * ones(nθ); Inf * ones(nx)],
        action_lower=ul, action_upper=uu))
end
push!(bnds, DTO.Bound(nx + nθ + nx, 0, state_lower=xlT, state_upper=xuT))


cons = DTO.Constraint{Float64}[]
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
            ]
        end
        push!(cons, DTO.Constraint(constraints_T, nx + nθ + nx, nu, indices_inequality=collect(0 .+ (1:8))))
    else
function constraints_t(x, u, w)
            [
            # equality (16)
            contact_constraints_equality(model, env, h, x, u, w);
            # inequality (32)
            contact_constraints_inequality_t(model, env, h, x, u, w);

            # body/feet constraints
            x[3] - x_ref[t][3]; # body height
            ]
        end
        push!(cons, DTO.Constraint(constraints_t, nx + nθ + nx, nu, indices_inequality=collect(16 .+ (1:32))) )
    end
end

# ## problem
tolerance = 1.0e-3
direct_solver = DTO.Solver(dyn, obj, cons, bnds,
    options=DTO.Options(
        tol=tolerance,
        constr_viol_tol=tolerance,
        max_iter=20000,
        max_cpu_time = 60000.0
        ));

# ## initialize
x_interpolation = copy(x_ref)
Random.seed!(0)
u_guess = [1.0e-1 * rand(nu) for t = 1:T-1] # may need to run more than once to get good trajectory
DTO.initialize_states!(direct_solver, x_interpolation)
DTO.initialize_controls!(direct_solver, u_guess)

DTO.initialize_states!(direct_solver, x_sol)
DTO.initialize_controls!(direct_solver, u_sol)

# ## solve
@time DTO.solve!(direct_solver)

# Get solution
x_sol, u_sol = DTO.get_trajectory(direct_solver);

# Visualize solution
visualize!(vis, model, [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...], Δt=h);
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