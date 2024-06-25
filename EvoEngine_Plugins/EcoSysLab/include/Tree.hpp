#pragma once
#include "BillboardCloud.hpp"
#include "Climate.hpp"
#include "FoliageDescriptor.hpp"
#include "LSystemString.hpp"
#include "RadialBoundingVolume.hpp"
#include "ShootDescriptor.hpp"
#include "Soil.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "TreeDescriptor.hpp"
#include "TreeGraph.hpp"
#include "TreeIOTree.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreeVisualizer.hpp"

#ifdef BUILD_WITH_PHYSICS
#  include "PhysicsLayer.hpp"
#  include "RigidBody.hpp"
#endif
using namespace evo_engine;
namespace eco_sys_lab {

struct BranchPhysicsParameters {
#pragma region Physics
  float density = 1.0f;
  float linear_damping = 1.0f;
  float angular_damping = 1.0f;
  int position_solver_iteration = 8;
  int velocity_solver_iteration = 8;
  float joint_drive_stiffness = 3000.0f;
  float joint_drive_stiffness_thickness_factor = 3.0f;
  float joint_drive_damping = 10.0f;
  float joint_drive_damping_thickness_factor = 3.0f;
  bool enable_acceleration_for_drive = true;
  float minimum_thickness = 0.01f;
#pragma endregion
  void Serialize(YAML::Emitter& out);

  void Deserialize(const YAML::Node& in);

  template <typename SkeletonData, typename FlowData, typename NodeData>
  void Link(const std::shared_ptr<Scene>& scene, const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
            const std::unordered_map<unsigned, SkeletonFlowHandle>& corresponding_flow_handles, const Entity& entity,
            const Entity& child);
  void OnInspect();
};

struct SkeletalGraphSettings {
  float line_thickness = 0.0f;
  float fixed_line_thickness = 0.002f;
  float branch_point_size = 1.0f;
  float junction_point_size = 1.f;

  bool fixed_point_size = true;
  float fixed_point_size_factor = 0.005f;
  glm::vec4 line_color = glm::vec4(1.f, .5f, 0.5f, 1.0f);
  glm::vec4 branch_point_color = glm::vec4(1.f, 1.f, 0.f, 1.f);
  glm::vec4 junction_point_color = glm::vec4(0.f, .7f, 1.f, 1.f);

  glm::vec4 line_focus_color = glm::vec4(1.f, 0.f, 0.f, 1.f);
  glm::vec4 branch_focus_color = glm::vec4(1.f, 0.f, 0.f, 1.f);
  void OnInspect();
};
struct JunctionLine {
  int line_index = -1;
  glm::vec3 start_position;
  glm::vec3 end_position;
  float start_radius;
  float end_radius;

  glm::vec3 start_direction;
  glm::vec3 end_direction;
};

struct TreePartData {
  int tree_part_index;
  bool is_junction = false;
  JunctionLine base_line;
  std::vector<JunctionLine> children_lines;
  std::vector<SkeletonNodeHandle> node_handles;
  std::vector<bool> is_end;
  std::vector<int> line_index;

  int num_of_leaves = 0;
};

class Tree : public IPrivateComponent {
  void CalculateProfiles();
  friend class EcoSysLabLayer;
  void PrepareController(const std::shared_ptr<ShootDescriptor>& shoot_descriptor, const std::shared_ptr<Soil>& soil,
                         const std::shared_ptr<Climate>& climate);
  ShootGrowthController shoot_growth_controller_{};

  void GenerateTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, std::vector<TreePartData>& tree_parts);

 public:
  StrandModelParameters strand_model_parameters{};
  static void SerializeTreeGrowthSettings(const TreeGrowthSettings& tree_growth_settings, YAML::Emitter& out);
  static void DeserializeTreeGrowthSettings(TreeGrowthSettings& tree_growth_settings, const YAML::Node& in);
  static bool OnInspectTreeGrowthSettings(TreeGrowthSettings& tree_growth_settings);
  bool generate_mesh = true;
  float low_branch_pruning = 0.f;
  float crown_shyness_distance = 0.f;
  float start_time = 0.f;
  void BuildStrandModel();

  std::shared_ptr<Strands> GenerateStrands() const;
  void GenerateTrunkMeshes(const std::shared_ptr<Mesh>& trunk_mesh,
                           const TreeMeshGeneratorSettings& mesh_generator_settings);
  std::shared_ptr<Mesh> GenerateBranchMesh(const TreeMeshGeneratorSettings& mesh_generator_settings);
  std::shared_ptr<Mesh> GenerateFoliageMesh(const TreeMeshGeneratorSettings& mesh_generator_settings);
  std::shared_ptr<ParticleInfoList> GenerateFoliageParticleInfoList(
      const TreeMeshGeneratorSettings& mesh_generator_settings);
  std::shared_ptr<Mesh> GenerateStrandModelBranchMesh(
      const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings) const;
  std::shared_ptr<Mesh> GenerateStrandModelFoliageMesh(
      const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings);
  void ExportObj(const std::filesystem::path& path, const TreeMeshGeneratorSettings& mesh_generator_settings);
  void ExportStrandModelObj(const std::filesystem::path& path,
                            const StrandModelMeshGeneratorSettings& mesh_generator_settings);

  void ExportTrunkObj(const std::filesystem::path& path, const TreeMeshGeneratorSettings& mesh_generator_settings);
  bool TryGrow(float delta_time, bool pruning);

  bool TryGrowSubTree(float delta_time, SkeletonNodeHandle base_internode_handle, bool pruning);
  [[nodiscard]] bool ParseBinvox(const std::filesystem::path& file_path,
                                 VoxelGrid<TreeOccupancyGridBasicData>& voxel_grid, float voxel_size = 1.0f);

  void Reset();

  TreeVisualizer tree_visualizer{};

  void Serialize(YAML::Emitter& out) const override;
  bool split_root_test = true;
  bool record_biomass_history = true;
  float left_side_biomass;
  float right_side_biomass;
  TreeMeshGeneratorSettings tree_mesh_generator_settings{};
  StrandModelMeshGeneratorSettings strand_model_mesh_generator_settings{};
  SkeletalGraphSettings skeletal_graph_settings{};
  BranchPhysicsParameters branch_physics_parameters{};
  int temporal_progression_iteration = 0;
  bool temporal_progression = false;
  void Update() override;

  std::vector<float> root_biomass_history;
  std::vector<float> shoot_biomass_history;

  PrivateComponentRef soil;
  PrivateComponentRef climate;
  AssetRef tree_descriptor;
  bool enable_history = false;
  int history_iteration = 30;

  void ClearSkeletalGraph() const;
  void GenerateSkeletalGraph(const SkeletalGraphSettings& skeletal_graph_settings, SkeletonNodeHandle base_node_handle,
                             const std::shared_ptr<Mesh>& point_mesh_sample,
                             const std::shared_ptr<Mesh>& line_mesh_sample) const;

  TreeModel tree_model{};
  StrandModel strand_model{};
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void OnDestroy() override;

  void OnCreate() override;

  void GenerateGeometryEntities(const TreeMeshGeneratorSettings& mesh_generator_settings, int iteration = -1);
  void ClearGeometryEntities() const;

  void InitializeStrandRenderer();
  void InitializeStrandRenderer(const std::shared_ptr<Strands>& strands) const;
  void ClearStrandRenderer() const;

  void InitializeStrandModelMeshRenderer(const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings);

  void ClearStrandModelMeshRenderer() const;

  void RegisterVoxel();
  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void FromSkeleton(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton);
  void FromLSystemString(const std::shared_ptr<LSystemString>& l_system_string);
  void FromTreeGraph(const std::shared_ptr<TreeGraph>& tree_graph);
  void FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& tree_graph_v2);
  void ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, YAML::Emitter& out);
  void ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, treeio::json& out);

  void ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, const std::filesystem::path& path);
  [[maybe_unused]] bool ExportIoTree(const std::filesystem::path& path) const;
  void ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void Deserialize(const YAML::Node& in) override;

  void GenerateBillboardClouds(const BillboardCloud::GenerateSettings& foliage_generate_settings);

  void GenerateAnimatedGeometryEntities(const TreeMeshGeneratorSettings& mesh_generator_settings, int iteration,
                                        bool enable_physics = true);
  void ClearAnimatedGeometryEntities() const;
};

template <typename SkeletonData, typename FlowData, typename NodeData>
void BranchPhysicsParameters::Link(const std::shared_ptr<Scene>& scene,
                                   const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                                   const std::unordered_map<unsigned, SkeletonFlowHandle>& corresponding_flow_handles,
                                   const Entity& entity, const Entity& child) {
#ifdef BUILD_WITH_PHYSICS
  if (!scene->HasPrivateComponent<RigidBody>(entity)) {
    scene->RemovePrivateComponent<RigidBody>(child);
    scene->RemovePrivateComponent<Joint>(child);
    return;
  }

  const auto& flow = skeleton.PeekFlow(corresponding_flow_handles.at(child.GetIndex()));

  const float child_thickness = flow.info.start_thickness;
  const float child_length = flow.info.flow_length;

  if (child_thickness < minimum_thickness)
    return;
  const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(child).lock();
  rigid_body->SetEnableGravity(false);
  rigid_body->SetDensityAndMassCenter(density * child_thickness * child_thickness * child_length);
  rigid_body->SetLinearDamping(linear_damping);
  rigid_body->SetAngularDamping(angular_damping);
  rigid_body->SetSolverIterations(position_solver_iteration, velocity_solver_iteration);
  rigid_body->SetAngularVelocity(glm::vec3(0.0f));
  rigid_body->SetLinearVelocity(glm::vec3(0.0f));

  auto joint = scene->GetOrSetPrivateComponent<Joint>(child).lock();
  joint->Link(entity);
  joint->SetType(JointType::D6);
  joint->SetMotion(MotionAxis::SwingY, MotionType::Free);
  joint->SetMotion(MotionAxis::SwingZ, MotionType::Free);
  joint->SetDrive(
      DriveType::Swing, glm::pow(child_thickness, joint_drive_stiffness_thickness_factor) * joint_drive_stiffness,
      glm::pow(child_thickness, joint_drive_damping_thickness_factor) * joint_drive_damping, enable_acceleration_for_drive);
#endif
}

template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
void Tree::FromSkeleton(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton) {
  if (auto td = tree_descriptor.Get<TreeDescriptor>(); !td) {
    EVOENGINE_WARNING("Growing tree without tree descriptor!");
    td = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
    tree_descriptor = td;
    const auto shoot_descriptor = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
    td->shoot_descriptor = shoot_descriptor;
    const auto foliage_descriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
    td->foliage_descriptor = foliage_descriptor;
  }
  tree_model.Initialize(src_skeleton);
  // TODO: Set up buds here.
}
}  // namespace eco_sys_lab
