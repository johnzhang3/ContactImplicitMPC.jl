using DirectTrajectoryOptimization 
using LinearAlgebra
const DTO = DirectTrajectoryOptimization
const max_foot_dist_from_body = 2
### This model includes constraints on feet positions 

function centroidal_quadruped_dyn(model, env, h, y, x, u, w) 

    # dimensions
    nq = model.nq
    nu = model.nu 

    # configurations
    
    q1⁻ = x[1:nq] 
    q2⁻ = x[nq .+ (1:nq)]
    q2⁺ = y[1:nq]
    q3⁺ = y[nq .+ (1:nq)]

    # control 
    u_control = u[1:nu] 
    γ = u[nu .+ (1:4)] 
    β = u[nu + 4 .+ (1:16)] 
    
    E = [1.0 0.0 -1.0 0.0; 
         0.0 1.0 0.0 -1.0] # friction mapping 
    J = J_func(model, env, q2⁺)
    λ = transpose(J[1:12, :]) * [
                        [E * β[0  .+ (1:4)]; γ[1]];
                        [E * β[4  .+ (1:4)]; γ[2]];
                        [E * β[8  .+ (1:4)]; γ[3]];
                        [E * β[12 .+ (1:4)]; γ[4]]
                       ]
    [
     q2⁺ - q2⁻;
     dynamics(model, h, q1⁻, q2⁺, u_control, zeros(model.nw), λ, q3⁺)
    ]
end

function centroidal_quadruped_dyn1(model, env, h, y, x, u, w)
    nx = 2 * model.nq
    [
     centroidal_quadruped_dyn(model, env, h, y, x, u, w);
     y[nx .+ (1:53)] - u;
     y[nx + 53 .+ (1:nx)] - x[1:nx];
    ]
end

function centroidal_quadruped_dynt(model, env, h, y, x, u, w)
    nx = 2 * model.nq
    [
     centroidal_quadruped_dyn(model, env, h, y, x, u, w);
     y[nx .+ (1:53)] - u;
     y[nx + 53 .+ (1:nx)] - x[nx + 53 .+ (1:nx)];
    ]
end

function contact_constraints_inequality_1(model, env, h, x, u, w) 
    nq = model.nq
    nu = model.nu 
    nx = 2nq

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    u_control = u[1:nu] 
    γ = u[nu .+ (1:4)] 
    β = u[nu + 4 .+ (1:16)] 
    ψ = u[nu + 4 + 16 .+ (1:4)] 
    η = u[nu + 4 + 16 + 4 .+ (1:16)] 
    sα = u[nu + 4 + 16 + 4 + 16 .+ (1:1)]

    ϕ = ϕ_func(model, env, q3)[1:4]
  
    μ = model.μ_world
    fc = μ .* γ[1:4] - [sum(β[0 .+ (1:4)]); sum(β[4 .+ (1:4)]); sum(β[8 .+ (1:4)]); sum(β[12 .+ (1:4)]);]

    [
     -ϕ; 
     -fc;
     β .* η .- sα;
     ψ .* fc  .- sα;
    ]
end

function contact_constraints_inequality_t(model, env, h, x, u, w) 
    nq = model.nq
    nu = model.nu 
    nx = 2nq

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    u_control = u[1:nu] 
    γ = u[nu .+ (1:4)] 
    β = u[nu + 4 .+ (1:16)] 
    ψ = u[nu + 4 + 16 .+ (1:4)] 
    η = u[nu + 4 + 16 + 4 .+ (1:16)] 
    sα = u[nu + 4 + 16 + 4 + 16 .+ (1:1)]

    ϕ = ϕ_func(model, env, q3)[1:4]
    γ⁻ = x[nx + nu .+ (1:4)] 
    sα⁻ = x[nx + nu + 4 + 16 + 4 + 16 .+ (1:1)]
    
    μ = model.μ_world
    fc = μ .* γ[1:4] - [sum(β[0 .+ (1:4)]); sum(β[4 .+ (1:4)]); sum(β[8 .+ (1:4)]); sum(β[12 .+ (1:4)]);]

    [
     -ϕ; 
     -fc;
     γ⁻ .* ϕ .- sα⁻;
     β .* η .- sα;
     ψ .* fc  .- sα;
    ]
end


function contact_constraints_inequality_T(model, env, h, x, u, w) 
    nq = model.nq
    nu = model.nu
    nx = 2nq

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    ϕ = ϕ_func(model, env, q3)[1:4]
    γ⁻ = x[nx + nu .+ (1:4)] 
    sα⁻ = x[nx + nu + 4 + 16 + 4 + 16 .+ (1:1)]
   
    [
     -ϕ; 
     γ⁻ .* ϕ .- sα⁻;
    ]
end

function contact_constraints_equality(model, env, h, x, u, w) 
    nq = model.nq
    nu = model.nu 

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    γ = u[nu .+ (1:4)] 
    β = u[nu + 4 .+ (1:16)] 
    ψ = u[nu + 4 + 16 .+ (1:4)] 
    η = u[nu + 4 + 16 + 4 .+ (1:16)] 
    sα = u[nu + 4 + 16 + 4 + 16 .+ (1:1)]
   
    E = [1.0 0.0 -1.0 0.0; 
         0.0 1.0 0.0 -1.0]
    v = (q3 - q2) ./ h[1]
    vT = vcat([E' * v[6 + (i-1) * 3 .+ (1:2)] for i = 1:4]...)
    
    ψ_stack = vcat([ψi * ones(4) for ψi in ψ]...)
    
    [
     η - vT - ψ_stack;
    ]
end

function feet_position_inequality(model, env, h, x, u, w)
    nq = model.nq
    nu = model.nu 

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    # body and feet positions
    body_pos = q2[1:3]
    body_rot = q2[4:6]

    # transfomation matrices
    translation = Translation(-body_pos[1], -body_pos[2], -body_pos[3])
    # rotation = LinearMap(RotXYZ(body_rot[1], body_rot[2], body_rot))
    # composed = compose(rotation, translation)
    
    # foot position in body frame
    f1_pos = composed(q2[6 .+ (1:3)])
    f2_pos = composed(q2[6 + 3 .+ (1:3)])
    f3_pos = composed(q2[6 + 3 + 3 .+ (1:3)])
    f4_pos = composed(q2[6 + 3 + 3 + 3 .+ (1:3)])


    body_x = body_pos[1]
    f1_x = f1_pos[1]
    f2_x = f2_pos[1]
    f3_x = f3_pos[1]
    f4_x = f4_pos[1]

    body_y = body_pos[2]
    f1_y = f1_pos[2]
    f2_y = f2_pos[2]
    f3_y = f3_pos[2]
    f4_y = f4_pos[2]

    body_z = body_pos[3]
    f1_z = f1_pos[3]
    f2_z = f2_pos[3]
    f3_z = f3_pos[3]
    f4_z = f4_pos[3]

    # feet distances from body position in body frame (0,0,0)
    # f1_dist = norm(body_pos .- f1_pos)
    # f2_dist = norm(body_pos .- f2_pos)
    # f3_dist = norm(body_pos .- f3_pos)
    # f4_dist = norm(body_pos .- f4_pos)
    f1_dist = norm(f1_pos)
    f2_dist = norm(f2_pos)
    f3_dist = norm(f3_pos)
    f4_dist = norm(f4_pos)

    [
        # feet can't be too far from body
        f1_dist - max_foot_dist_from_body;
        f2_dist - max_foot_dist_from_body;
        f3_dist - max_foot_dist_from_body;
        f4_dist - max_foot_dist_from_body;

        # feet can't exceed body height 
        # f1_z - body_z;
        # f2_z - body_z;
        # f3_z - body_z;
        # f4_z - body_z;
        f1_z;
        f2_z;
        f3_z;
        f4_z;
        # feet can't cross body mid-point in x
        # body_x - f1_x;
        # body_x - f2_x;
        # f3_x - body_x;
        # f4_x - body_x;
        -f1_x;
        -f2_x;
        f3_x;
        f4_x;
        # feet can't cross body mid-point in y
        # f1_y - body_y;
        # body_y - f2_y;
        # f3_y - body_y;
        # body_y - f4_y;
        
        # body_y - f1_y;
        # f2_y - body_y;
        # body_y - f3_y;
        # f4_y - body_y;
        -f1_y;
        f2_y;
        -f3_y;
        f4_y;
    ]
end