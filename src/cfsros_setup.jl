using CFSTrajOpt: load_mesh_tuples, t3d2cfg, set_configurations!, meshgraph

using CFSTrajOpt.RigidBodyDynamics
using CFSTrajOpt.MechanismGeometries
using CFSTrajOpt.GeometryBasics
using CFSTrajOpt.StaticArrays
using CFSTrajOpt.Rotations
using CFSTrajOpt.Serialization
using CFSTrajOpt.LazySets
using CFSTrajOpt: RobotCaches, MeshGraphCaches, RigidBodyCaches
using CFSTrajOpt.KernelAbstractions

function robot_get_state(
    setup::RobotSetup,
    g::AbstractVector{T},
    q::AbstractVector{T}) where {T}
    s = MechanismState(setup.model)
    # Robot
    arm_joints = Vector{Joint}(filter(x ->
            x.joint_type isa Revolute, tree_joints(setup.model)))
    arm_joints = filter(x -> any(map(y -> occursin(y, x.name),
            ["shoulder", "elbow", "wrist"])), arm_joints)
    arm_r_q = [0.0, -0.2, 0.0, 1.57, 0.0, 0.0, 0.0]
    arm_l_q = [0.0, 0.2, 0.0, 1.57, 0.0, 0.0, 0.0]
    arm_q = [arm_l_q arm_r_q]' |> vec
    set_configurations!(s, arm_joints, arm_q)
    set_configurations!(s, setup.dof_layouts, g)
    set_configurations!(s, setup.dof_joints, q)
    s
end

function robotsetup_setup(urdf_path::String, mode::Symbol=:left) # :left, :right, :both
    cd(dirname(urdf_path)) do
        urdf = basename(urdf_path)
        model = parse_urdf(urdf, remove_fixed_tree_joints=false)
        meshes = load_mesh_tuples(visual_elements(model, URDFVisuals(urdf, tag="collision")))
        meshes_cvx = load_mesh_tuples(visual_elements(model, URDFVisuals(urdf, tag="collision")))

        dof_layouts = Vector{Joint}(filter(x ->
                (x.joint_type isa Prismatic), tree_joints(model)))
        dof_joints = Vector{Joint}(filter(x ->
                x.joint_type isa Revolute, tree_joints(model)))
        dof_joints = if mode == :left
            filter(x -> occursin("left", x.name), dof_joints)
        elseif mode == :right
            filter(x -> occursin("right", x.name), dof_joints)
        elseif mode == :both
            filter(x -> occursin("right", x.name) || occursin("left", x.name), dof_joints)
        else
            error("Invalid mode: $mode")
        end
        println(dof_joints)
        dyn_constr = let
            pos = SVector((dof_joints |>
                           x -> map(y -> y.position_bounds, x) |>
                                Iterators.flatten |> collect .|>
                                x -> (x.lower, x.upper))...)
            vel = SVector((dof_joints |>
                           x -> map(y -> y.velocity_bounds, x) |>
                                Iterators.flatten |> collect .|>
                                x -> (x.lower, x.upper))...)
            acc = SVector(map(x -> (-5.0, 5.0), pos))
            jerk = SVector(map(x -> (-10.0, 10.0), pos))
            eff = SVector(map(x -> (-100000.0, 100000.0), pos))
            (pos, vel, acc, jerk, eff)
        end


        meshgraphs_cvx = map(x -> (x[1], meshgraph(x[2])...), meshes_cvx)

        collision_idxs = Vector{Tuple{Int,Int}}()
        # robot_robot_collision

        bodies_names_idxs_pairs = map(x -> begin
                i, m = x
                body_ = body_fixed_frame_to_body(model, first(m).frame)
                (body_.name, i)
            end, enumerate(meshes_cvx))

        for i in 1:lastindex(meshes_cvx)
            for j in i+1:lastindex(meshes_cvx)
                col_pair = (first(bodies_names_idxs_pairs[i]),
                    first(bodies_names_idxs_pairs[j]))
                if (
                    ((occursin("shoulder", col_pair[1]) ||
                      occursin("shoulder", col_pair[2])) ||
                     (occursin("elbow", col_pair[1]) ||
                      occursin("elbow", col_pair[2])) ||
                     (occursin("wrist", col_pair[1]) ||
                      occursin("wrist", col_pair[2]))) &&
                    (occursin("head", col_pair[1]) ||
                     occursin("head", col_pair[2]) ||
                     occursin("waist", col_pair[1]) ||
                     occursin("waist", col_pair[2]) ||
                     occursin("pelvis", col_pair[1]) ||
                     occursin("pelvis", col_pair[2]) ||
                     occursin("torso", col_pair[1]) ||
                     occursin("torso", col_pair[2]) ||
                     occursin("env", col_pair[1]) ||
                     occursin("env", col_pair[2]))
                )
                    push!(collision_idxs, (i, j))
                end
            end
        end

        collisions_bodies = (x -> (y -> body_fixed_frame_to_body(model, first(meshes_cvx[y]).frame)).(x)).(collision_idxs)
        collisions_paths = (x -> path(model, x[1], x[2])).(collisions_bodies)
        collisions_paths = filter(x -> length(x) > 2, collisions_paths)
        collision_pairs = map(x -> (x[1], x[2]), zip(collision_idxs, collisions_paths))

        dof = if mode == :left
            7
        elseif mode == :right
            7
        elseif mode == :both
            14
        else
            error("Invalid mode: $mode")
        end
        RobotSetup{Float64,dof}(
            model, meshes, meshes_cvx, meshgraphs_cvx, dyn_constr,
            dof_joints, dof_layouts,
            collision_pairs
        )
    end
end;