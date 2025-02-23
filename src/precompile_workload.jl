# include("CFSRos.jl")
using .CFSRos: robotsetup_setup, plan_motion_cart, plan_motion_joint
using BoTrajOpt.Serialization

setup = robotsetup_setup("precompile/test.urdf")
plan_motion_cart_req = deserialize("precompile/plan_motion_cart.jld2")
plan_motion_joint_req = deserialize("precompile/plan_motion_joint.jld2")

plan_motion_cart(
    setup,
    plan_motion_cart_req["start_joint_names"],
    plan_motion_cart_req["start_joint_positions"],
    plan_motion_cart_req["frame_id"],
    plan_motion_cart_req["goal_link_name"],
    plan_motion_cart_req["goal_pos"],
    plan_motion_cart_req["goal_ori"],
)

plan_motion_joint(
    setup,
    plan_motion_joint_req["start_joint_names"],
    plan_motion_joint_req["start_joint_positions"],
    plan_motion_joint_req["goal_joint_names"],
    plan_motion_joint_req["goal_joint_positions"],
)