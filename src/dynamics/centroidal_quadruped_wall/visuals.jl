function build_robot!(vis::Visualizer, model::CentroidalQuadrupedWall;
	body_height=0.025,
	body_length=0.1,
	body_width=0.1,
	foot_radius=0.025,
	color_opacity=1.0)

	body_mat = MeshPhongMaterial(color=RGBA(0.0, 0.0, 0.0, color_opacity))
	foot_mat = MeshPhongMaterial(color=RGBA(1.0, 165.0 / 255.0, 0.0, color_opacity))

	default_background!(vis)

	setobject!(vis[:body],
    	Rect(Vec(-body_length, -body_width, -body_height),Vec(2.0 * body_length, 2.0 * body_width, body_height)),
    	body_mat)

	foot1 = setobject!(vis["foot1"], Sphere(Point3f0(0),
        convert(Float32, foot_radius)),
        foot_mat)

	foot2 = setobject!(vis["foot2"], Sphere(Point3f0(0),
		convert(Float32, foot_radius)),
		foot_mat)

	foot3 = setobject!(vis["foot3"], Sphere(Point3f0(0),
		convert(Float32, foot_radius)),
		foot_mat)

	foot4 = setobject!(vis["foot4"], Sphere(Point3f0(0),
		convert(Float32, foot_radius)),
		foot_mat)

	box = setobject!(vis["wall"], GeometryBasics.HyperRectangle(Vec(0.35 + foot_radius, -1.0, 0.0),
		Vec(0.25 + foot_radius, 2.0, 1.0)), MeshPhongMaterial(color = RGBA(0.5, 0.5, 0.5, 1.0)))
end

function set_robot!(vis::Visualizer, model::CentroidalQuadrupedWall, q::AbstractVector;
	p_shift=[0.0; 0.0; 0.025])

	R = mrp_rotation_matrix(q[4:6])

	p_body  = q[0  .+ (1:3)] + p_shift
	p_foot1 = q[6  .+ (1:3)] + p_shift
	p_foot2 = q[9  .+ (1:3)] + p_shift
	p_foot3 = q[12 .+ (1:3)] + p_shift
	p_foot4 = q[15 .+ (1:3)] + p_shift

	settransform!(vis["body"], compose(Translation(p_body), LinearMap(euler_rotation_matrix(q[4:6]))))
	settransform!(vis["foot1"], Translation(p_foot1))
	settransform!(vis["foot2"], Translation(p_foot2))
	settransform!(vis["foot3"], Translation(p_foot3))
	settransform!(vis["foot4"], Translation(p_foot4))

end

function visualize!(vis, model::CentroidalQuadrupedWall, q;
	Δt=0.1,
	body_height=0.025,
	body_length=0.17,
	body_width=0.15,
	foot_radius=0.025,
	color_opacity=1.0,
	fixed_camera=false)

	build_robot!(vis, model,
		body_height=body_height,
		body_length=body_length,
		body_width=body_width,
		foot_radius=foot_radius,
		color_opacity=color_opacity)

	anim = MeshCat.Animation(convert(Int, floor(1.0 / Δt)))

	for (t, qt) in enumerate(q)
		MeshCat.atframe(anim, t) do
			set_robot!(vis, model, qt)
		end
	end

	MeshCat.setanimation!(vis, anim)

	return anim
end
