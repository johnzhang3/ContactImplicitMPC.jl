# Combine the parts of the stand_wall_two_step reference and resolve the DTO

using DirectTrajectoryOptimization 
const DTO = DirectTrajectoryOptimization
using ContactImplicitMPC
const LciMPC = ContactImplicitMPC
using JLD2

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

# Get simulation and model
s = get_simulation("centroidal_quadruped_wall", "flat_3D_lc", "flat")
model = s.model
env = s.env
nx = 2 * model.nq # Number of states
nc = model.nc # Number of contacts
nu = model.nu + nc + 4 * nc + nc + 4 * nc + 1 # Additional decision vars
nθ = 53

# Load and combine trajectory
@load joinpath(@__DIR__, "stand_wall_two_steps_part1.jld2") x_sol u_sol x_ref
x_sol_1 = x_sol; u_sol_1 = u_sol; x_ref_1 = x_ref;
@load joinpath(@__DIR__, "stand_wall_two_steps_part2.jld2") x_sol u_sol x_ref
x_sol_2 = x_sol; u_sol_2 = u_sol; x_ref_2 = x_ref;
@load joinpath(@__DIR__, "stand_wall_two_steps_part3.jld2") x_sol u_sol x_ref
x_sol_3 = x_sol; u_sol_3 = u_sol; x_ref_3 = x_ref;
@load joinpath(@__DIR__, "stand_wall_two_steps_part4.jld2") x_sol u_sol x_ref
x_sol_4 = x_sol; u_sol_4 = u_sol; x_ref_4 = x_ref;
@load joinpath(@__DIR__, "stand_wall_two_steps_part5.jld2") x_sol u_sol x_ref
x_sol_5 = x_sol; u_sol_5 = u_sol; x_ref_5 = x_ref;

x_sol = [x_sol_1..., x_sol_2..., x_sol_3..., x_sol_4..., x_sol_5...]
u_sol = [u_sol_1..., u_sol_1[end], u_sol_2..., u_sol_2[end], u_sol_3...,
         u_sol_3[end], u_sol_4..., u_sol_4[end], u_sol_5...]
x_ref = [x_ref_1..., x_ref_2..., x_ref_3..., x_ref_4..., x_ref_5...]

# Pad the end
x_sol = [x_sol..., [x_sol[end] for i = 1:10]...]
u_sol = [u_sol..., [u_sol[end] for i = 1:10]...]
x_ref = [x_ref..., [x_ref[end] for i = 1:10]...]

# Visualize trajectory
visualize!(vis, model, [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...], Δt=h);
vis

# Get time and horizon
T = size(x_sol, 1)
h = 0.05

# Set up array of DTO dynamics structures
d1 = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dyn1(
	model, env, [h], y, x, u, w), nx + nθ + nx, nx, nu)
dt = DTO.Dynamics((y, x, u, w) -> centroidal_quadruped_dynt(
	model, env, [h], y, x, u, w), nx + nθ + nx, nx + nθ + nx, nu)
dyn = [d1, [dt for t = 2:T-1]...]

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
            J += 1000 * transpose(x[1:nx] - x_ref[t]) * Diagonal(1000.0 * ones(nx)) * (x[1:nx] - x_ref[t])
            J += 0.5 * transpose(u[1:model.nu]) * Diagonal(1.0e-3 * ones(model.nu)) * u[1:model.nu]
            J += 0.5 * transpose(u[model.nu + 4 .+ (1:20)]) * Diagonal(1.0 * ones(20)) * u[model.nu + 4 .+ (1:20)]
            J += 1000.0 * u[end] # slack
            return J
        end, nx + nθ + nx, nu))
    end
end

# ## constraints
q1 = x_ref[1][1:convert(Int, nx/2)]
qT = x_ref[T][1:convert(Int, nx/2)]
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
            [x[i + 6] - x_ref[t][i + 6] for i in 1:12]... #Foot positions
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
            [x[i + 6] - x_ref[t][i + 6] for i in 1:12]... #Foot positions
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

            # # body/feet constraints
            x[3] - x_ref[t][3]; # body height
            [x[i + 6] - x_ref[t][i + 6] for i in 1:12]... #Foot positions
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
DTO.initialize_states!(direct_solver, x_sol)
DTO.initialize_controls!(direct_solver, u_sol)

# ## solve
@time DTO.solve!(direct_solver)

# Get solution
x_sol, u_sol = DTO.get_trajectory(direct_solver);

# Visualize solution
visualize!(vis, model, [x_sol[1][1:model.nq], [x[model.nq .+ (1:model.nq)] for x in x_sol]...], Δt=h);
vis

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

@save joinpath(@__DIR__, "stand_wall_two_steps_combined_v3.jld2") qm um γm bm ψm ηm μm hm x_sol u_sol x_ref
@load joinpath(@__DIR__, "stand_wall_two_steps_combined_v1.jld2") qm um γm bm ψm ηm μm hm x_sol u_sol x_ref
