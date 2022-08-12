# ## model
using Revise
using ContactImplicitMPC
include("trajopt_model_v2.jl")

# ## horizon
h = 0.05
T_raise = 5
T_wave = 5

# ## centroidal_quadruped
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env

nx = 2 * model.nq
nc = 4 #model.nc
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
foot_y = 0.15

q1 = zeros(model.nq)
q1[1:3] = [-0.02, -0.02, body_height]
q1[7:9] = [foot_x; foot_y; 0]     # FL 
q1[10:12] = [foot_x; -foot_y; 0]  # FR 
q1[13:15] = [-foot_x; foot_y; 0]  # RL 
q1[16:18] = [-foot_x; -foot_y; 0] # RR 

# ## Hand raise configuration
q2 = copy(q1)
q2[7:9] = [foot_x + 0.05, foot_y + 0.02, body_height + 0.03]

# ## Waving configuration 
q3 = copy(q2)
q3[7:9] = [q2[7] - 0.1, q2[8], q2[9]]

q3 = copy(q2)
q3[7:9] = [q2[7] - 0.2, q2[8], q2[9]]

# ## Create reference trajectory
q_ref = 