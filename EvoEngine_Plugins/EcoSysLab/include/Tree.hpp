#pragma once
#include "BasicPruningDescriptor.hpp"
#include "RootModel.hpp"
#ifdef BILLBOARD_CLOUDS_PLUGIN
#  include "BillboardCloud.hpp"
#endif
#include "BasicFoliageDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "RadialBoundingVolume.hpp"
#include "SkeletalGraphSettings.hpp"
#include "Soil.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "TreeControllers.hpp"
#include "TreeDescriptor.hpp"
#include "TreeIOTree.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreePart.hpp"
#include "TreeStatistics.hpp"
#include "TreeVisualizer.hpp"

namespace eco_sys_lab_plugin {
#ifdef BILLBOARD_CLOUDS_PLUGIN
using namespace billboard_clouds_plugin;
#endif
using namespace evo_engine;

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
   */
  void PrepareController(const SimulationSettings& simulation_settings);

  ShootGrowthController shoot_growth_controller_{};
  FoliageController foliage_controller_{};
  ShootReproductionController shoot_reproduction_controller_{};
  ShootPruningController shoot_pruning_controller_{};

  RootGrowthController root_growth_controller_{};
  FineRootController fine_root_controller_{};
  RootReproductionController root_reproduction_controller_{};
  RootPruningController root_pruning_controller_{};

  /**
   * @brief Generates tree parts based on mesh generation settings.
   * @param mesh_generator_settings Settings for generating meshes.
   * @param tree_parts List to store generated tree parts.
   */
  void GenerateTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings,
                         std::vector<TreePartData>& tree_parts);

 public:
  StrandModelParameters strand_model_parameters{};  ///< Parameters defining strand-based growth modeling.
  ShootVisualizer shoot_visualizer{};               ///< Visualizer used for debugging and display of the tree model.
  RootVisualizer root_visualizer{};
  bool split_root_test = true;         ///< Flag to enable or disable root split testing.
  bool record_biomass_history = true;  ///< Flag to enable or disable biomass history recording.
  float left_side_biomass;             ///< Recorded biomass for the left section of the tree.
  float right_side_biomass;            ///< Recorded biomass for the right section of the tree.

  TreeMeshGeneratorSettings tree_mesh_generator_settings{};  ///< Mesh generation settings for the tree.
  StrandModelMeshGeneratorSettings
      strand_model_mesh_generator_settings{};       ///< Mesh generation settings for strand models.
  SkeletalGraphSettings skeletal_graph_settings{};  ///< Graphical settings for skeletal structure visualization.

  int temporal_progression_iteration = 0;  ///< The current iteration count for temporal progression.
  bool temporal_progression = false;       ///< Flag to enable or disable temporal progression.

  std::vector<float> root_biomass_history;   ///< History record of root biomass over time.
  std::vector<float> shoot_biomass_history;  ///< History record of shoot biomass over time.

  PrivateComponentRef soil;      ///< Reference to the associated soil component.
  PrivateComponentRef climate;   ///< Reference to the associated climate component.
  AssetRef tree_descriptor_ref;  ///< Reference to the tree descriptor asset.

  bool enable_history = false;  ///< Flag to enable or disable history recording.
  int history_iteration = 30;   ///< Number of iterations to retain in the history record.

  bool generate_mesh = true;  ///< Flag to determine if a mesh should be generated.

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
   * @brief Generates a mesh for shoot branches.
   * @param mesh_generator_settings Settings for mesh generation.
   * @return A shared pointer to the generated branch mesh.
   */
  std::shared_ptr<Mesh> GenerateShootMesh(const TreeMeshGeneratorSettings& mesh_generator_settings);
  /**
   * @brief Generates a mesh for root branches.
   * @param mesh_generator_settings Settings for mesh generation.
   * @return A shared pointer to the generated branch mesh.
   */
  std::shared_ptr<Mesh> GenerateRootMesh(const TreeMeshGeneratorSettings& mesh_generator_settings);
  /**
   * @brief Generates a mesh for fine roots.
   * @param mesh_generator_settings Settings for mesh generation.
   * @return A shared pointer to the generated branch mesh.
   */
  std::shared_ptr<Mesh> GenerateFineRootMesh(const TreeMeshGeneratorSettings& mesh_generator_settings);

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
  std::shared_ptr<Mesh> GenerateStrandModelShootMesh(
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
   * @brief Attempts to grow a subtree from the base internode handle.
   * @param simulation_settings Settings controlling simulation behavior.
   * @param base_internode_handle Handle representing the root of the subtree.
   * @param pruning Whether pruning is enabled during growth.
   * @return True if the subtree successfully grows, false otherwise.
   */
  bool TryGrow(const SimulationSettings& simulation_settings, SkeletonNodeHandle base_internode_handle, bool pruning);

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

  /**
   * @brief Updates the tree state.
   */
  void Update() override;

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

  ShootModel shoot_model{};  ///< The procedural tree model instance.
  RootModel root_model{};
  StrandModel shoot_strand_model{};  ///< The strand-based model representation.

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
   * @param src_skeleton The source skeleton to import from.
   */
  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void FromSkeleton(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton);

  /**
   * @brief Exports tree parts data as YAML.
   * @param mesh_generator_settings Settings for generating meshes.
   * @param out The YAML emitter.
   */
  void ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, YAML::Emitter& out);

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
   * @brief Exports node graph data as YAML.
   * @param out The YAML emitter.
   */
  void ExportNodeGraph(YAML::Emitter& out) const;

  /**
   * @brief Exports node graph data to a file.
   * @param path The file path for exporting.
   */
  void ExportNodeGraph(const std::filesystem::path& path) const;

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
 * @brief Imports a tree model from a skeleton structure.
 * @param src_skeleton The source skeleton to import from.
 */
template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
void Tree::FromSkeleton(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton) {
  if (auto td = tree_descriptor_ref.Get<TreeDescriptor>(); !td) {
    EVOENGINE_WARNING("Growing tree without tree descriptor!");
    td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
    tree_descriptor_ref = td;
    const auto shoot_descriptor = AssetManager::CreateTemporaryAsset<BasicShootDescriptor>();
    td->shoot_descriptor = shoot_descriptor;
    const auto foliage_descriptor = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
    td->foliage_descriptor = foliage_descriptor;
  }
  shoot_model.Initialize(src_skeleton);
  // TODO: Set up buds here.
}
}  // namespace eco_sys_lab_plugin