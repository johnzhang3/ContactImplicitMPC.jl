"""
    Flamingo24 hopper policy for Hopper2D
"""

@with_kw mutable struct Flamingo24Options
    live_plotting::Bool=false # Use the live plotting tool to debug
end

mutable struct Flamingo24 <: Policy
	model::Flamingo
	phase::Symbol
	count::Int
	h_sim::Real
	u::AbstractVector
	contact::Vector{Bool}
	front::Symbol
	rear::Symbol
	q0::AbstractVector
	q1::AbstractVector
	qref::AbstractVector
	xdref::Real
	opts::Flamingo24Options
end

function flamingo_policy(model::Flamingo, h_sim;
		qref=[0.0, 0.849, -0.00, 0.1, 0.295, -0.3, 0.1, π/2, π/2],
		xdref=0.30,
		mpc_opts = Flamingo24Options(),
		)

	u  = zeros(model.dim.u)
	q0 = zeros(model.dim.q)
	q1 = zeros(model.dim.q)
	contact = [false for i=1:model.dim.c]
	Flamingo24(model, :settle, 0, h_sim, u, contact, :foot_1, :foot_2,
		copy(q0), copy(q1), qref, xdref, mpc_opts)
end

function policy(p::Flamingo24, x, traj, t)
	nc = p.model.dim.c
	iθ = Vector(3:9)
	il1 = [2,3,6]
	il2 = [4,5,7]
	m_flamingo = model.m_torso + model.m_thigh1 +
		model.m_thigh2 + model.m_calf1 + model.m_calf2
	# Initialization
	p.q0 = copy(traj.q[t])
	p.q1 = copy(x)
	qd = (p.q1 - p.q0)/traj.h

	# Detect contact
	p.contact = detect_contact(model, traj.γ[max(1,t-1)])
	# Compute feet positions
	xr = kinematics_3(model, p.q1, body=p.rear, mode=:com)[1]
	xf = kinematics_3(model, p.q1, body=p.front, mode=:com)[1]

	if p.front == :foot_1
		ilf = il1
		ilr = il2
	elseif p.front == :foot_2
		ilf = il2
		ilr = il1
	end

	# Update phase
	p.count += 1
	# if p.phase == :settle && all(p.contact) && p.count >= 30
	if p.phase == :settle && all(p.contact) && p.count*p.h_sim >= 0.278
		p.front, p.rear = foot_order(model, p.q1)
		p.phase = :translation
		p.count = 0
	elseif p.phase == :translation && ((p.q1[1] - xr > 0.26) || (xf - p.q1[1] < 0.15)) && p.count*p.h_sim >= 0.926
	# elseif p.phase == :translation && ((p.q1[1] - xr > 0.26) || (xf - p.q1[1] < 0.15)) && p.count >= 100
		p.front, p.rear = foot_order(model, p.q1)
		p.phase = :swing
		p.count = 0
	# elseif p.phase == :swing && ((xr - p.q1[1] > 0.10) || (p.q1[1] - xf > 0.15) || (p.q1[ilr[1]] > -0.1)) && p.count*p.h_sim >= 0.926
elseif p.phase == :swing && ((xr - p.q1[1] > 0.20) || (p.q1[1] - xf > 0.10) || (p.q1[ilr[1]] > +0.0)) && p.count*p.h_sim >= 0.926
	# elseif p.phase == :swing && ((xr - p.q1[1] > 0.10) || (p.q1[1] - xf > 0.15)) && p.count >= 100
		p.front, p.rear = foot_order(model, p.q1)
		p.phase = :translation
		p.count = 0
	end
	@show t
	@show p.count
	@show p.phase
	@show p.front
	@show p.contact

	if p.front == :foot_1
		ilf = il1
		ilr = il2
	elseif p.front == :foot_2
		ilf = il2
		ilr = il1
	end

	# PD joint controller
	kp = -100.0
	kd =  0.04*kp
	p.u[1] = kp * (p.q1[3] - p.qref[3]) + kd * qd[3]

	if p.phase == :settle
		# PD joint controller
		kα = 100.0
		kβ = 10.0
		kp = -[kα, kα, kα, kα, kα, kβ, kβ]
		kd =  0.04*kp
		p.u[1:7] = kp .* (p.q1[iθ] - p.qref[iθ]) + kd .* (qd[iθ])
	end

	if p.phase == :translation
		kpfx = -200.0
		kdfx = 0.04*kpfx
		kpfz = -400.0
		kdfz = -200.0

		xref = p.q1[1] + p.xdref*p.h_sim
		fx = kpfx*(p.q1[1] - xref) + kdfx*(qd[1] - p.xdref)
		fz = kpfz*(p.q1[2] - p.qref[2]+0.02) + kdfz*qd[2] + model.g*m_flamingo
		f = [fx, fz]
		α = 0.5
		α = 0.25 + 0.5*(p.q1[1]-xr)/(xf-xr)
		ff = α*f
		fr = (1-α)*f
		τf = virtual_actuator_torque(p.model, p.q1, ff, body=p.front)
		τr = virtual_actuator_torque(p.model, p.q1, fr, body=p.rear)
		p.u[ilf] .= τf
		p.u[ilr] .= τr
		p.u[ilr[2]] += -0.2*qd[2 + ilr[2]]
	end

	if p.phase == :swing
		kpfx = -200.0
		kdfx = 0.04*kpfx
		kpfz = -400.0
		kdfz = -200.0

		xref = p.q1[1] + p.xdref*p.h_sim
		fx = kpfx*(p.q1[1] - xref) + kdfx*(qd[1] - p.xdref)
		fz = kpfz*(p.q1[2] - p.qref[2]+0.02) + kdfz*qd[2] + model.g*m_flamingo
		f = [fx, fz]
		α = 1.0
		ff = α*f
		τf = virtual_actuator_torque(p.model, p.q1, ff, body=p.front)
		p.u[ilf] .= τf

		p.u[ilf[1]] += -2*(p.q1[2 + ilf[1]] + 0.2)     - 0.2*qd[2 + ilf[1]]
		p.u[ilf[2]] += -2*(p.q1[2 + ilf[2]] + 0.2)     - 0.2*qd[2 + ilf[2]]

		p.u[ilr[1]] = -2*(p.q1[2 + ilr[1]] + 0.6)     - 0.2*(qd[2 + ilr[1]] - 0.3)
		p.u[ilr[2]] = -3*(p.q1[2 + ilr[2]] - 0.9)     - 0.2*qd[2 + ilr[2]]
		p.u[ilr[3]] = -0.3*(p.q1[end] - π/2-0.3) - 0.2*qd[end]
	end

	# Rescale
	p.u .*= traj.h
    return p.u
end

function detect_contact(model::Flamingo, γ::AbstractVector; contact_threshold=1.5e-2)
	nc = model.dim.c
	contact = [false for i=1:nc]
	for i = 1:nc
		if γ[i] .> contact_threshold
			contact[i] = true
		else
			contact[i] = false
		end
	end
	return contact
end

function foot_order(model::Flamingo,  q::AbstractVector)
	c1 = kinematics_3(model, q, body=:foot_1, mode=:com)
	c2 = kinematics_3(model, q, body=:foot_2, mode=:com)
	if c1[1] > c2[1]
		return :foot_1, :foot_2
	else
		return :foot_2, :foot_1
	end
	return nothing
end

function kinematic_map(model::Flamingo, q; body=:foot_1)
	#TODO need to compute wrt to the center of pressure not com
	x, z = q[1:2] - kinematics_3(model, q, body=body, mode=:com)
	return [x,z]
end
function jacobian_map(model::Flamingo, q; body=:foot_1)
	k(q) = kinematic_map(model, q, body=body)
	J = ForwardDiff.jacobian(k, q)[:,3:end]
	return J
end

function virtual_actuator_torque(model::Flamingo, q::AbstractVector,
		f::AbstractVector; body=:foot_1)
	J = jacobian_map(model, q, body=body)
	if body == :foot_1
		iJ = [2,3,6]
	elseif body == :foot_2
		iJ = [4,5,7]
	else
		@error "incorrect body specification"
	end
	τ = J[:,iJ]'*f
	return τ
end
