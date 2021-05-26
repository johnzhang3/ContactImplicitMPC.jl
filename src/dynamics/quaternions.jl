function conjugate(q)
	s = q[1]
	v = q[2:4]

	return [s; -v]
end

function L_multiply(q)
	s = q[1]
	v = q[2:4]

	SMatrix{4,4}([s -transpose(v);
	              v s * I + skew(v)])
end

function R_multiply(q)
	s = q[1]
	v = q[2:4]

	SMatrix{4,4}([s -transpose(v);
	              v s * I - skew(v)])
end

function multiply(q1, q2)
	L_multiply(q1) * q2
end

# eq. 14 http://roboticexplorationlab.org/papers/planning_with_attitude.pdf
function attitude_jacobian(q)
	s = q[1]
	v = q[2:4]

	[-transpose(v);
	 s * I + skew(v)]
end

# eq. 16 http://roboticexplorationlab.org/papers/maximal_coordinate_dynamics.pdf
function ω_finite_difference(q1, q2, h)
	2.0 * multiply(conjugate(q1), (q2 - q1) ./ h)[2:4]
end

# https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
function quaternion_rotation_matrix(q)
	r, i, j, k  = q

	r11 = 1.0 - 2.0 * (j^2.0 + k^2.0)
	r12 = 2.0 * (i * j - k * r)
	r13 = 2.0 * (i * k + j * r)

	r21 = 2.0 * (i * j + k * r)
	r22 = 1.0 - 2.0 * (i^2.0 + k^2.0)
	r23 = 2.0 * (j * k - i * r)

	r31 = 2.0 * (i * k - j * r)
	r32 = 2.0 * (j * k + i * r)
	r33 = 1.0 - 2.0 * (i^2.0 + j^2.0)

	SMatrix{3,3}([r11 r12 r13;
	              r21 r22 r23;
				  r31 r32 r33])
end

# Cayley map
function φ(ϕ)
	1.0 / sqrt(1.0 + norm(ϕ)^2.0) * [1.0; ϕ]
end

# Square root of quaternion: https://www.johndcook.com/blog/2021/01/06/quaternion-square-roots/
function sqrt_quat(q; ϵ=1e-16)
	r = norm(q)
	# Angle
	theta = acos(q[1]/r)
	# Axis
	u = q[2:4]
	u ./= norm(u) + ϵ
	# Half axis-angle rotation
	x = [cos(theta/2);  sin(theta/2)*u]
	# Sqrt on the norm of the quaternion
	x .*= r^0.5 # useless for unit quaternion
	return x
end

function midpoint(q0, q1)
	# Small delta rotation
	qψ = R_multiply(q0)' * q1
	# Divide delta in two
	sqψ = sqrt_quat(qψ)
	# Apply small rotation
	qmid = L_multiply(sqψ) * q0
	return qmid
end

struct RnQuaternion <: Space
	n::Int
	mapping
	r_idx
	Δr_idx
	quat_idx
	Δquat_idx
end

function rn_quaternion_space(dim, mapping, r_idx, Δr_idx, quat_idx, Δquat_idx)
	RnQuaternion(dim, mapping, r_idx, Δr_idx, quat_idx, Δquat_idx)
end

function candidate_point!(z̄::Vector{T}, s::RnQuaternion, z::Vector{T}, Δ::Vector{T}, α::T) where T
    z̄[s.r_idx] .= z[s.r_idx] - α .* Δ[s.Δr_idx]
	z̄[s.quat_idx] .= L_multiply(z[s.quat_idx]) * φ(-1.0 * α .* Δ[s.Δquat_idx])
	return nothing
end

function mapping!(δz, s::RnQuaternion, δzs, z)
	δz .= s.mapping(z) * δzs
end
