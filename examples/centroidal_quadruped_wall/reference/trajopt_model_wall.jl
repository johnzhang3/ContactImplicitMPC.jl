using DirectTrajectoryOptimization 
const DTO = DirectTrajectoryOptimization

function centroidal_quadruped_dyn(model::CentroidalQuadrupedWall, env, h, y, x, u, w) 

    # dimensions
    nq = model.nq
    nu = model.nu 
    nc = model.nc

    # configurations
    
    q1⁻ = x[1:nq] 
    q2⁻ = x[nq .+ (1:nq)]
    q2⁺ = y[1:nq]
    q3⁺ = y[nq .+ (1:nq)]

    # control 
    u_control = u[1:nu] 
    γ = u[nu .+ (1:nc)] 
    β = u[nu + nc .+ (1:nc*4)] 
    
    E = [1.0 0.0 -1.0 0.0; 
         0.0 1.0 0.0 -1.0] # friction mapping 
    J = J_func(model, env, q2⁺)
    # λ = transpose(J) * [
    #                     [E * β[0  .+ (1:4)]; γ[1]];
    #                     [E * β[4  .+ (1:4)]; γ[2]];
    #                     [E * β[8  .+ (1:4)]; γ[3]];
    #                     [E * β[12 .+ (1:4)]; γ[4]]
    #                    ]
    λ = transpose(J) * contact_forces(model, env, γ, β, q3⁺, 0)
    [
     q2⁺ - q2⁻;
     dynamics(model, h, q1⁻, q2⁺, u_control, zeros(model.nw), λ, q3⁺)
    ]
end

function centroidal_quadruped_dyn1(model, env, h, y, x, u, w)
    nx = 2 * model.nq
    nu = size(u)[1]
    # println(size(u))
    [
     centroidal_quadruped_dyn(model, env, h, y, x, u, w);
    #  y[nx .+ (1:53)] - u;
    #  y[nx + 53 .+ (1:nx)] - x[1:nx];
     y[nx .+ (1:nu)] - u;
     y[nx + nu .+ (1:nx)] - x[1:nx];
    ]
end

function centroidal_quadruped_dynt(model::CentroidalQuadrupedWall, env, h, y, x, u, w)
    nx = 2 * model.nq
    nu = 
    nu = size(u)[1]
    [
     centroidal_quadruped_dyn(model, env, h, y, x, u, w);
    #  y[nx .+ (1:53)] - u;
    #  y[nx + 53 .+ (1:nx)] - x[nx + 53 .+ (1:nx)];
     y[nx .+ (1:nu)] - u;
     y[nx + nu .+ (1:nx)] - x[1:nx];
    ]
end

function contact_constraints_inequality_1(model::CentroidalQuadrupedWall, env, h, x, u, w) 
    nq = model.nq
    nu = model.nu 
    nc = model.nc
    nx = 2nq

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    u_control = u[1:nu] 
    γ = u[nu .+ (1:nc)] 
    β = u[nu + nc .+ (1:4*nc)] 
    ψ = u[nu + nc + 4*nc .+ (1:nc)] 
    η = u[nu + nc + 4*nc + nc .+ (1:4*nc)] 
    sα = u[nu + nc + 4*nc + nc + 4*nc .+ (1:1)]

    ϕ = ϕ_func(model, env, q3)
  
    μ = model.μ_world
    fc = μ .* γ[1:nc] - [
            sum(β[0 .+ (1:nc)]); 
            sum(β[1*nc .+ (1:nc)]); 
            sum(β[2*nc .+ (1:nc)]); 
            sum(β[3*nc .+ (1:nc)]);
            ]

    [
     -ϕ; 
     -fc;
     β .* η .- sα;
     ψ .* fc  .- sα;
    ]
end

function contact_constraints_inequality_t(model::CentroidalQuadrupedWall, env, h, x, u, w) 
    nq = model.nq
    nu = model.nu 
    nx = 2nq
    nc = model.nc

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    u_control = u[1:nu] 
    γ = u[nu .+ (1:nc)] 
    β = u[nu + nc .+ (1:4*nc)] 
    ψ = u[nu + nc + 4*nc .+ (1:nc)] 
    η = u[nu + nc + 4*nc + nc .+ (1:4*nc)] 
    sα = u[nu + nc + 4*nc + nc + 4*nc .+ (1:1)]

    ϕ = ϕ_func(model, env, q3)[1:nc]
    γ⁻ = x[nx + nu .+ (1:nc)] 
    sα⁻ = x[nx + nu + nc + 4*nc + nc + 4*nc .+ (1:1)]
    
    μ = model.μ_world
    fc = μ .* γ[1:nc] - [
            sum(β[0 .+ (1:nc)]); 
            sum(β[1*nc .+ (1:nc)]); 
            sum(β[2*nc .+ (1:nc)]); 
            sum(β[3*nc .+ (1:nc)]);
            ]

    [
     -ϕ; 
     -fc;
     γ⁻ .* ϕ .- sα⁻;
     β .* η .- sα;
     ψ .* fc  .- sα;
    ]
end


function contact_constraints_inequality_T(model::CentroidalQuadrupedWall, env, h, x, u, w) 
    nq = model.nq
    nu = model.nu
    nx = 2nq
    nc = model.nc

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    ϕ = ϕ_func(model, env, q3)[1:nc]
    γ⁻ = x[nx + nu .+ (1:nc)] 
    sα⁻ = x[nx + nu + nc + 4*nc + nc + 4*nc .+ (1:1)]
   
    [
     -ϕ; 
     γ⁻ .* ϕ .- sα⁻;
    ]
end

function contact_constraints_equality(model::CentroidalQuadrupedWall, env, h, x, u, w) 
    nq = model.nq
    nu = model.nu 

    q2 = x[1:nq] 
    q3 = x[nq .+ (1:nq)] 

    u_control = u[1:nu] 
    γ = u[nu .+ (1:4*nc)] 
    β = u[nu + nc .+ (1:4*nc)] 
    ψ = u[nu + nc + 4*nc .+ (1:nc)] 
    η = u[nu + nc + 4*nc + nc .+ (1:4*nc)] 
    sα = u[nu + nc + 4*nc + nc + 4*nc .+ (1:1)]
   
    # E = [1.0 0.0 -1.0 0.0; 
    #      0.0 1.0 0.0 -1.0]
    # v = (q3 - q2) ./ h[1]
    # vT = [
    #         vcat([transpose(E) * v[6 + (i-1) * 3 .+ (1:2)] for i = 1:4]...);
    # ]
    vT = velocity_stack(model, env, q3, q2, 0, h)
    
    ψ_stack = vcat([ψi * ones(4) for ψi in ψ]...)
    
    [
     η - vT - ψ_stack;
    ]
end