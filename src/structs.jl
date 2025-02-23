using CFSTrajOpt.RigidBodyDynamics
using CFSTrajOpt.MechanismGeometries
using CFSTrajOpt.GeometryBasics
using CFSTrajOpt.StaticArrays
using CFSTrajOpt.SparseArrays
using CFSTrajOpt.LazySets
using CFSTrajOpt.RigidBodyDynamics: StateCache, WrenchesCache
using CFSTrajOpt: RobotCaches

# RobotSetup
struct RobotSetup{T,D}
    # Robot
    model::Mechanism{T}
    meshes::Vector{Tuple{VisualElement,AbstractMesh}}
    meshes_cvx::Vector{Tuple{VisualElement,AbstractMesh}}
    meshgraphs_cvx::Vector{Tuple{VisualElement,SparseMatrixCSC{Bool,Int},Vector{SVector{3,T}}}}

    dyn_constr::NTuple{5,SVector{D,Tuple{T,T}}}

    dof_joints::Vector{Joint}
    dof_layouts::Vector{Joint}

    # Collisions
    collision_pairs::Vector{Tuple{Tuple{Int,Int}, RigidBodyDynamics.Graphs.TreePath}} # indexes to meshgraphs_cvx
end
