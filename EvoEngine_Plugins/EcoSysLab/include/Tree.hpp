
#pragma once
#ifdef BILLBOARD_CLOUDS_PLUGIN
#  include "BillboardCloud.hpp"
using namespace billboard_clouds_plugin;
#endif
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
#include "TreePart.hpp"
#include "TreeStatistics.hpp"
#include "TreeVisualizer.hpp"
#ifdef PHYSX_PHYSICS_PLUGIN
#  include "PhysicsLayer.hpp"
#  include "RigidBody.hpp"
#endif
using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @struct BranchPhysicsParameters
 * @brief Defines physics parameters for branches in the tree simulation.
 */
struct BranchPhysicsParameters {
#pragma region Physics
  float density = 1.0f;                                 ///< Density of the branch.
  float linear_damping = 1.0f;                          ///< Linear damping applied in simulations.
  float angular_damping = 1.0f;                         ///< Angular damping applied in simulations.
  int position_solver_iteration = 8;                    ///< Number of solver iterations for position calculations.
  int velocity_solver_iteration = 8;                    ///< Number of solver iterations for velocity calculations.
  float joint_drive_stiffness = 3000.0f;                ///< Stiffness of the joint drive.
  float joint_drive_stiffness_thickness_factor = 3.0f;  ///< Factor affecting stiffness based on thickness.
  float joint_drive_damping = 10.0f;                    ///< Damping of the joint drive.
  float joint_drive_damping_thickness_factor = 3.0f;    ///< Factor affecting damping based on thickness.
  bool enable_acceleration_for_drive = true;            ///< Flag to enable acceleration for drive.
  float minimum_thickness = 0.01f;                      ///< Minimum thickness threshold.

#pragma endregion

  /**
   * @brief Serializes the physics parameters to a YAML emitter.
   * @param out The YAML emitter to serialize data into.
   */
  void Serialize(YAML::Emitter& out);

  /**
   * @brief Deserializes the physics parameters from a YAML node.
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Links the branch physics parameters with the given skeleton structure.
   * @tparam SkeletonData Data type representing the skeleton.
   * @tparam FlowData Data type representing the flow in the skeleton structure.
   * @tparam NodeData Data type representing nodes in the skeleton.
   * @param scene The scene where simulation occurs.
   * @param skeleton The skeleton structure to link.
   * @param corresponding_flow_handles Mapping of flow handles corresponding to skeleton.
   * @param entity The parent entity.
   * @param child The child entity linked to the parent.
   */
  template <typename SkeletonData, typename FlowData, typename NodeData>
  void Link(const std::shared_ptr<Scene>& scene, const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
            const std::unordered_map<unsigned, SkeletonFlowHandle>& corresponding_flow_handles, const Entity& entity,
            const Entity& child);

  /**
   * @brief Handles the parameter inspection in the editor.
   */
  void OnInspect();
};

/**
 * @struct SkeletalGraphSettings
 * @brief Defines visualization settings for the skeletal graph of a tree.
 */
struct SkeletalGraphSettings {
  float line_thickness = 0.0f;          ///< Thickness of the skeletal graph lines.
  float fixed_line_thickness = 0.002f;  ///< Fixed thickness value for lines.
  float branch_point_size = 1.0f;       ///< Size of branch points.
  float junction_point_size = 1.f;      ///< Size of junction points.

  bool fixed_point_size = true;                                    ///< Determines if point size is fixed.
  float fixed_point_size_factor = 0.005f;                          ///< Factor affecting fixed point size.
  glm::vec4 line_color = glm::vec4(1.f, .5f, 0.5f, 1.0f);          ///< Color of skeletal graph lines.
  glm::vec4 branch_point_color = glm::vec4(1.f, 1.f, 0.f, 1.f);    ///< Color of branch points.
  glm::vec4 junction_point_color = glm::vec4(0.f, .7f, 1.f, 1.f);  ///< Color of junction points.

  glm::vec4 line_focus_color = glm::vec4(1.f, 0.f, 0.f, 1.f);    ///< Color when a line is in focus.
  glm::vec4 branch_focus_color = glm::vec4(1.f, 0.f, 0.f, 1.f);  ///< Color when a branch is in focus.

  /**
   * @brief Handles inspection of graphical settings in the editor.
   */
  void OnInspect();
};

/**
 * @class Tree
 * @brief Represents a procedural tree with various simulation and rendering capabilities.
 */
class Tree : public IPrivateComponent {
  void CalculateProfiles();
  friend class EcoSysLabLayer;

  /**
   * @brief Prepares the growth controller with specified simulation settings.
   * @param simulation_settings Settings for the simulation.
   * @param shoot_descriptor Descriptor defining shoot behavior.
   * @param soil The soil environmental factor.
   * @param climate The climate environmental factor.
   */
  void PrepareController(const SimulationSettings& simulation_settings,
                         const std::shared_ptr<ShootDescriptor>& shoot_descriptor, const std::shared_ptr<Soil>& soil,
                         const std::shared_ptr<Climate>& climate);

  ShootGrowthController shoot_growth_controller_{};

  /**
   * @brief Generates tree parts based on mesh generation settings.
   * @param mesh_generator_settings Settings for generating meshes.
   * @param tree_parts List to store generated tree parts.
   */
  void GenerateTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings,
                         std::vector<TreePartData>& tree_parts);

 public:
  StrandModelParameters strand_model_parameters{};  ///< Parameters defining strand-based growth modeling.

  /**
   * @brief Serializes tree growth settings into a YAML emitter.
   * @param tree_growth_settings The settings to serialize.
   * @param out The YAML emitter to store data.
   */
  static void SerializeTreeGrowthSettings(const TreeGrowthSettings& tree_growth_settings, YAML::Emitter& out);

  /**
   * @brief Deserializes tree growth settings from a YAML node.
   * @param tree_growth_settings The settings to populate.
   * @param in The YAML node containing serialized settings.
   */
  static void DeserializeTreeGrowthSettings(TreeGrowthSettings& tree_growth_settings, const YAML::Node& in);

  /**
   * @brief Handles tree growth settings inspection in the editor.
   * @param tree_growth_settings The settings to inspect.
   * @return True if settings were not modified, false otherwise.
   */
  static bool OnInspectTreeGrowthSettings(TreeGrowthSettings& tree_growth_settings);

  bool generate_mesh = true;  ///< Flag to determine if a mesh should be generated.

  /**
   * @struct PruningSettings
   * @brief Defines settings for pruning operations on the tree.
   */
  struct PruningSettings {
    float low_branch_pruning = 0.f;  ///< Factor defining low branch pruning.

    /**
     * @brief Inspects pruning settings in an editor.
     * @param editor_layer The editor layer managing inspection.
     * @return True if data was not modified during inspection.
     */
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

    /**
     * @brief Saves pruning settings to a YAML emitter.
     * @param name The name of the settings entry.
     * @param out The YAML emitter to serialize data into.
     */
    void Save(const std::string& name, YAML::Emitter& out) const;

    /**
     * @brief Loads pruning settings from a YAML node.
     * @param name The name of the settings entry.
     * @param in The YAML node containing serialized data.
     */
    void Load(const std::string& name, const YAML::Node& in);
  } pruning_settings{};

  float start_time = 0.f;  ///< The starting time reference for tree growth simulation.

  /**
   * @brief Builds the internal strand model representation of the tree.
   */
  void BuildStrandModel();

  /**
   * @brief Generates strand structures from the tree model.
   * @return A shared pointer to the generated strands.
   */
  std::shared_ptr<Strands> GenerateStrands() const;

  /**
   * @brief Generates strand particles for simulation.
   * @return A shared pointer to the generated particle information list.
   */
  std::shared_ptr<ParticleInfoList> GenerateStrandParticles() const;

  /**
   * @brief Generates trunk meshes for the tree.
   * @param trunk_mesh The output trunk mesh.
   * @param mesh_generator_settings Settings for mesh generation.
   */
  void GenerateTrunkMeshes(const std::shared_ptr<Mesh>& trunk_mesh,
                           const TreeMeshGeneratorSettings& mesh_generator_settings);

  /**
   * @brief Generates a mesh for branches.
   * @param mesh_generator_settings Settings for mesh generation.
   * @return A shared pointer to the generated branch mesh.
   */
  std::shared_ptr<Mesh> GenerateBranchMesh(const TreeMeshGeneratorSettings& mesh_generator_settings);

  /**
   * @brief Generates a mesh for foliage.
   * @param mesh_generator_settings Settings for mesh generation.
   * @return A shared pointer to the generated foliage mesh.
   */
  std::shared_ptr<Mesh> GenerateFoliageMesh(const TreeMeshGeneratorSettings& mesh_generator_settings);

  /**
   * @brief Generates particle information for foliage.
   * @param mesh_generator_settings Settings for mesh generation.
   * @return A shared pointer to the generated particle information list.
   */
  std::shared_ptr<ParticleInfoList> GenerateFoliageParticleInfoList(
      const TreeMeshGeneratorSettings& mesh_generator_settings);

  /**
   * @brief Generates a strand model branch mesh.
   * @param strand_model_mesh_generator_settings Settings for strand model mesh generation.
   * @return A shared pointer to the generated mesh.
   */
  std::shared_ptr<Mesh> GenerateStrandModelBranchMesh(
      const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings) const;

  /**
   * @brief Generates a strand model foliage mesh.
   * @param strand_model_mesh_generator_settings Settings for strand model mesh generation.
   * @return A shared pointer to the generated mesh.
   */
  std::shared_ptr<Mesh> GenerateStrandModelFoliageMesh(
      const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings);

  /**
   * @brief Exports the tree model as an OBJ file.
   * @param path The file path to export the tree model to.
   * @param mesh_generator_settings Settings for mesh generation.
   */
  void ExportObj(const std::filesystem::path& path, const TreeMeshGeneratorSettings& mesh_generator_settings);

  /**
   * @brief Exports the strand model as an OBJ file.
   * @param path The file path to export the strand model to.
   * @param mesh_generator_settings Settings for strand model mesh generation.
   */
  void ExportStrandModelObj(const std::filesystem::path& path,
                            const StrandModelMeshGeneratorSettings& mesh_generator_settings);

  /**
   * @brief Exports only the trunk of the tree as an OBJ file.
   * @param path The file path to export the trunk to.
   * @param mesh_generator_settings Settings for mesh generation.
   */
  void ExportTrunkObj(const std::filesystem::path& path, const TreeMeshGeneratorSettings& mesh_generator_settings);

  /**
   * @brief Attempts to grow the tree based on simulation parameters.
   * @param simulation_settings Settings controlling simulation behavior.
   * @param pruning Whether pruning is enabled during growth.
   * @return True if the tree successfully grows, false otherwise.
   */
  bool TryGrow(const SimulationSettings& simulation_settings, bool pruning);

  /**
   * @brief Attempts to grow a subtree from the base internode handle.
   * @param simulation_settings Settings controlling simulation behavior.
   * @param base_internode_handle Handle representing the root of the subtree.
   * @param pruning Whether pruning is enabled during growth.
   * @return True if the subtree successfully grows, false otherwise.
   */
  bool TryGrowSubTree(const SimulationSettings& simulation_settings, SkeletonNodeHandle base_internode_handle,
                      bool pruning);

  /**
   * @brief Parses a binvox file to generate a voxel grid.
   * @param file_path The path to the binvox file.
   * @param voxel_grid The voxel grid to populate.
   * @param voxel_size The size of each voxel unit.
   * @return True if parsing was successful, false otherwise.
   */
  [[nodiscard]] bool ParseBinvox(const std::filesystem::path& file_path,
                                 VoxelGrid<TreeOccupancyGridBasicData>& voxel_grid, float voxel_size = 1.0f);
  /**
   * @brief Calculate statistics for current tree.
   * @return The statistics of the tree.
   */
  TreeStatistics GetTreeStatistics() const;

  /**
   * @brief Resets the tree state.
   */
  void Reset();

  TreeVisualizer tree_visualizer{};  ///< Visualizer used for debugging and display of the tree model.

  bool split_root_test = true;         ///< Flag to enable or disable root split testing.
  bool record_biomass_history = true;  ///< Flag to enable or disable biomass history recording.
  float left_side_biomass;             ///< Recorded biomass for the left section of the tree.
  float right_side_biomass;            ///< Recorded biomass for the right section of the tree.

  TreeMeshGeneratorSettings tree_mesh_generator_settings{};  ///< Mesh generation settings for the tree.
  StrandModelMeshGeneratorSettings
      strand_model_mesh_generator_settings{};           ///< Mesh generation settings for strand models.
  SkeletalGraphSettings skeletal_graph_settings{};      ///< Graphical settings for skeletal structure visualization.
  BranchPhysicsParameters branch_physics_parameters{};  ///< Physics parameters applied to tree branches.

  int temporal_progression_iteration = 0;  ///< The current iteration count for temporal progression.
  bool temporal_progression = false;       ///< Flag to enable or disable temporal progression.

  /**
   * @brief Updates the tree state.
   */
  void Update() override;

  std::vector<float> root_biomass_history;   ///< History record of root biomass over time.
  std::vector<float> shoot_biomass_history;  ///< History record of shoot biomass over time.

  PrivateComponentRef soil;      ///< Reference to the associated soil component.
  PrivateComponentRef climate;   ///< Reference to the associated climate component.
  AssetRef tree_descriptor_ref;  ///< Reference to the tree descriptor asset.

  bool enable_history = false;  ///< Flag to enable or disable history recording.
  int history_iteration = 30;   ///< Number of iterations to retain in the history record.

  /**
   * @brief Clears the skeletal graph representation of the tree.
   */
  void ClearSkeletalGraph() const;

  /**
   * @brief Generates the skeletal graph representation of the tree.
   * @param skeletal_graph_settings Settings for skeletal graph generation.
   * @param base_node_handle The base node to start graph generation from.
   * @param point_mesh_sample Sample mesh used for representing points.
   * @param line_mesh_sample Sample mesh used for representing lines.
   */
  void GenerateSkeletalGraph(const SkeletalGraphSettings& skeletal_graph_settings, SkeletonNodeHandle base_node_handle,
                             const std::shared_ptr<Mesh>& point_mesh_sample,
                             const std::shared_ptr<Mesh>& line_mesh_sample) const;

  TreeModel tree_model{};      ///< The procedural tree model instance.
  StrandModel strand_model{};  ///< The strand-based model representation.

  /**
   * @brief Handles the inspection of tree properties in the editor.
   * @param editor_layer The editor layer managing the inspection.
   * @return True if the asset's content is not modified during inspection, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Handles destruction logic when the tree instance is removed.
   */
  void OnDestroy() override;

  /**
   * @brief Handles initialization logic when the tree instance is created.
   */
  void OnCreate() override;

  /**
   * @brief Generates geometry entities for the tree.
   * @param mesh_generator_settings Settings for mesh generation.
   * @param iteration The iteration number for geometry updates (-1 for default).
   */
  void GenerateGeometryEntities(const TreeMeshGeneratorSettings& mesh_generator_settings, int iteration = -1);

  /**
   * @brief Clears generated geometry entities.
   */
  void ClearGeometryEntities() const;

  /**
   * @brief Initializes the strand renderer for visualization.
   */
  void InitializeStrandRenderer();

  /**
   * @brief Initializes the strand renderer using given strand data.
   * @param strands The strand data to visualize.
   */
  void InitializeStrandRenderer(const std::shared_ptr<Strands>& strands) const;

  /**
   * @brief Clears the strand renderer.
   */
  void ClearStrandRenderer() const;

  /**
   * @brief Initializes strand particles for simulation.
   */
  void InitializeStrandParticles();

  /**
   * @brief Initializes strand particles using provided particle information.
   * @param particle_info_list The particle information list.
   */
  void InitializeStrandParticles(const std::shared_ptr<ParticleInfoList>& particle_info_list) const;

  /**
   * @brief Clears the initialized strand particles.
   */
  void ClearStrandParticles() const;

  /**
   * @brief Initializes mesh rendering for strand model.
   * @param strand_model_mesh_generator_settings Settings for generating strand model meshes.
   */
  void InitializeStrandModelMeshRenderer(const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings);

  /**
   * @brief Clears the strand model mesh renderer.
   */
  void ClearStrandModelMeshRenderer() const;

  /**
   * @brief Registers voxel data for tree occupancy.
   */
  void RegisterVoxel();

  /**
   * @brief Imports a tree model from a skeleton structure.
   * @tparam SrcSkeletonData Data type for the source skeleton.
   * @tparam SrcFlowData Data type for the skeleton flow.
   * @tparam SrcNodeData Data type for the skeleton nodes.
   * @param src_skeleton The source skeleton to import from.
   */
  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void FromSkeleton(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton);

  /**
   * @brief Imports a tree model from an L-System string.
   * @param l_system_string The L-System string representing the tree structure.
   */
  void FromLSystemString(const std::shared_ptr<LSystemString>& l_system_string);

  /**
   * @brief Imports a tree model from a tree graph.
   * @param tree_graph The tree graph representation.
   */
  void FromTreeGraph(const std::shared_ptr<TreeGraph>& tree_graph);

  /**
   * @brief Imports a tree model from a version 2 tree graph.
   * @param tree_graph_v2 The version 2 tree graph representation.
   */
  void FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& tree_graph);

  /**
   * @brief Exports tree parts data as YAML.
   * @param mesh_generator_settings Settings for generating meshes.
   * @param out The YAML emitter.
   */
  void ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, YAML::Emitter& out);

  /**
   * @brief Exports tree parts data as JSON.
   * @param mesh_generator_settings Settings for generating meshes.
   * @param out The JSON object to store data.
   */
  void ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, nlohmann::json& out);

  /**
   * @brief Exports flow graph data as YAML.
   * @param out The YAML emitter.
   */
  void ExportFlowGraph(YAML::Emitter& out) const;

  /**
   * @brief Exports flow graph data to a file.
   * @param path The file path for exporting.
   */
  void ExportFlowGraph(const std::filesystem::path& path) const;

  /**
   * @brief Exports tree parts data to a file.
   * @param mesh_generator_settings Settings for generating meshes.
   * @param path The file path for exporting.
   */
  void ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, const std::filesystem::path& path);

  /**
   * @brief Exports the tree I/O representation to a file.
   * @param path The file path for exporting.
   * @return True if export is successful, false otherwise.
   */
  [[maybe_unused]] bool ExportIoTree(const std::filesystem::path& path) const;

  /**
   * @brief Exports the radial bounding volume of the tree.
   * @param rbv The radial bounding volume to export.
   */
  void ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const;

  /**
   * @brief Collects asset references used by the tree.
   * @param list The list to store asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Serializes the tree state into a YAML emitter.
   * @param out The YAML emitter to store serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes tree data from a YAML node.
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

#ifdef BILLBOARD_CLOUDS_PLUGIN
  /**
   * @brief Generates billboard clouds for the tree.
   * @param foliage_generate_settings Settings for generating foliage billboards.
   */
  void GenerateBillboardClouds(const BillboardCloud::GenerateSettings& foliage_generate_settings);
#endif

  /**
   * @brief Generates animated geometry entities for the tree.
   * @param mesh_generator_settings Mesh generation settings.
   * @param iteration The iteration number (-1 for default behavior).
   * @param enable_physics Whether to enable physics simulation.
   */
  void GenerateAnimatedGeometryEntities(const TreeMeshGeneratorSettings& mesh_generator_settings, int iteration,
                                        bool enable_physics = true);

  /**
   * @brief Clears the animated geometry entities.
   */
  void ClearAnimatedGeometryEntities() const;
};

/**
 * @brief Links physics parameters for branches in the tree skeleton.
 * @tparam SkeletonData Data type for skeleton representation.
 * @tparam FlowData Data type for skeleton flow.
 * @tparam NodeData Data type for skeleton nodes.
 * @param scene The scene where physics interactions occur.
 * @param skeleton The skeleton representation.
 * @param corresponding_flow_handles Mapping of skeleton flow handles.
 * @param entity The parent entity in simulation.
 * @param child The child entity linked physically.
 */
template <typename SkeletonData, typename FlowData, typename NodeData>
void BranchPhysicsParameters::Link(const std::shared_ptr<Scene>& scene,
                                   const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                                   const std::unordered_map<unsigned, SkeletonFlowHandle>& corresponding_flow_handles,
                                   const Entity& entity, const Entity& child) {
#ifdef PHYSX_PHYSICS_PLUGIN
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
  joint->SetDrive(DriveType::Swing,
                  glm::pow(child_thickness, joint_drive_stiffness_thickness_factor) * joint_drive_stiffness,
                  glm::pow(child_thickness, joint_drive_damping_thickness_factor) * joint_drive_damping,
                  enable_acceleration_for_drive);
#endif
}

/**
 * @brief Imports a tree model from a skeleton structure.
 * @tparam SrcSkeletonData Data type for source skeleton representation.
 * @tparam SrcFlowData Data type for flow in the skeleton.
 * @tparam SrcNodeData Data type for nodes in the skeleton.
 * @param src_skeleton The source skeleton to import from.
 */
template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
void Tree::FromSkeleton(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton) {
  if (auto td = tree_descriptor_ref.Get<TreeDescriptor>(); !td) {
    EVOENGINE_WARNING("Growing tree without tree descriptor!");
    td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
    tree_descriptor_ref = td;
    const auto shoot_descriptor = AssetManager::CreateTemporaryAsset<ShootDescriptor>();
    td->shoot_descriptor = shoot_descriptor;
    const auto foliage_descriptor = AssetManager::CreateTemporaryAsset<FoliageDescriptor>();
    td->foliage_descriptor = foliage_descriptor;
  }
  tree_model.Initialize(src_skeleton);
  // TODO: Set up buds here.
}

}  // namespace eco_sys_lab_plugin
