#pragma once

#include "Curve.hpp"
#include "Skeleton.hpp"
#include "Tree.hpp"
#include "TreeMeshGenerator.hpp"
#include "VoxelGrid.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
typedef int PointHandle;
typedef int BranchHandle;
typedef int TreePartHandle;
struct ScatteredPoint {
  PointHandle handle = -1;
  std::vector<PointHandle> neighbor_scatter_points;
  std::vector<std::pair<float, BranchHandle>> p3;

  // For reversed branch
  std::vector<std::pair<float, BranchHandle>> p0;
  glm::vec3 position = glm::vec3(0.0f);
};
struct AllocatedPoint {
  glm::vec3 color;
  glm::vec3 position;

  glm::vec2 plane_position;
  float plane_center_distance;
  PointHandle handle = -1;
  TreePartHandle tree_part_handle = -1;
  BranchHandle branch_handle = -1;
  SkeletonNodeHandle node_handle = -1;
  int skeleton_index = -1;
};
struct PredictedBranch {
  glm::vec3 color;
  float foliage = 0.0f;
  TreePartHandle tree_part_handle = -1;
  BranchHandle handle = -1;
  BezierCurve bezier_curve;
  float start_thickness = 0.0f;
  float end_thickness = 0.0f;

  float final_thickness = 0.0f;
  std::vector<PointHandle> allocated_points;

  std::vector<std::pair<float, PointHandle>> points_to_p3;
  std::unordered_map<BranchHandle, float> p3_to_p0;

  // For reversed branch
  std::vector<std::pair<float, PointHandle>> points_to_p0;
  std::unordered_map<BranchHandle, float> p3_to_p3;
  std::unordered_map<BranchHandle, float> p0_to_p0;
  std::unordered_map<BranchHandle, float> p0_to_p3;
};

struct OperatorBranch {
  glm::vec3 color;
  float foliage = 0.0f;
  TreePartHandle tree_part_handle = -1;
  BranchHandle handle = -1;

  BranchHandle reversed_branch_handle = -1;

  BezierCurve bezier_curve;
  float thickness = 0.0f;

  BranchHandle parent_handle = -1;
  std::vector<BranchHandle> child_handles;
  BranchHandle largest_child_handle = -1;
  int skeleton_index = -1;
  std::vector<SkeletonNodeHandle> chain_node_handles;

  float best_distance = FLT_MAX;

  std::vector<std::pair<BranchHandle, float>> parent_candidates;
  bool used = false;
  bool orphan = false;

  bool apical = false;

  float distance_to_parent_branch = 0.0f;
  float root_distance = 0.0f;

  int descendant_size = 0;
};

struct TreePart {
  glm::vec3 color;
  float foliage = 0.f;
  TreePartHandle handle = -1;
  std::vector<PointHandle> allocated_points;
  std::vector<BranchHandle> branch_handles;
};

struct ConnectivityGraphSettings {
  bool reverse_connection = false;
  bool point_existence_check = true;
  float point_existence_check_radius = 0.1f;
  bool zigzag_check = true;
  float zigzag_branch_shortening = 0.1f;
  float parallel_shift_check_height_limit = 1.5f;
  bool parallel_shift_check = true;
  float parallel_shift_limit_range = 2.0f;
  float point_point_connection_detection_radius = 0.05f;
  float point_branch_connection_detection_radius = 0.1f;
  float branch_branch_connection_max_length_range = 5.0f;
  float direction_connection_angle_limit = 65.0f;
  float indirect_connection_angle_limit = 65.0f;

  float connection_range_limit = 1.0f;

  float max_scatter_point_connection_height = 1.5f;
  void OnInspect();
};

struct PointData {
  glm::vec3 position = glm::vec3(0.0f);
  glm::vec3 direction = glm::vec3(0.0f);
  int handle = -1;
  int index = -1;
  float min_distance = FLT_MAX;
};
struct BranchEndData {
  bool is_p0 = true;
  glm::vec3 position = glm::vec3(0.0f);
  int branch_handle = -1;
};

struct ReconstructionSettings {
  float internode_length = 0.03f;
  float min_height = 0.1f;
  float minimum_tree_distance = 0.1f;
  float branch_shortening = 0.3f;
  int max_parent_candidate_size = 100;
  int max_child_size = 10;

  float end_node_thickness = 0.004f;
  float thickness_sum_factor = 0.4f;
  float thickness_accumulation_factor = 0.00005f;
  float override_thickness_root_distance = 0.0f;

  int space_colonization_timeout = 10;
  float space_colonization_factor = 0.0f;
  float space_colonization_removal_distance_factor = 2;
  float space_colonization_detection_distance_factor = 4;
  float space_colonization_theta = 20.0f;

  int minimum_node_count = 10;
  bool limit_parent_thickness = true;
  float minimum_root_thickness = 0.02f;

  int node_back_track_limit = 30;
  int branch_back_track_limit = 1;

  /*
  bool m_candidateSearch = true;
  int m_candidateSearchLimit = 1;
  bool m_forceConnectAllBranches = false;
  */
  bool use_root_distance = true;
  int optimization_timeout = 999;

  float direction_smoothing = 0.1f;
  float position_smoothing = 0.1f;
  int smooth_iteration = 10;

  bool use_foliage = true;

  void OnInspect();
};

struct ReconstructionSkeletonData {
  glm::vec3 root_position = glm::vec3(0.0f);
  float max_end_distance = 0.0f;
};
struct ReconstructionFlowData {};
struct ReconstructionNodeData {
  glm::vec3 global_start_position = glm::vec3(0.f);
  glm::vec3 global_end_position = glm::vec3(0.0f);

  float draft_thickness = 0.0f;
  float allocated_point_thickness = 0.0f;
  std::vector<PointHandle> allocated_points;
  std::vector<PointHandle> filtered_points;
  BranchHandle branch_handle;

  bool regrowth = false;
  int marker_size = 0;
  glm::vec3 regrow_direction = glm::vec3(0.0f);
};
typedef Skeleton<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData> ReconstructionSkeleton;

class TreeStructor : public IPrivateComponent {
  bool DirectConnectionCheck(const BezierCurve& parentCurve, const BezierCurve& childCurve, bool reverse);

  static void FindPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& pointVoxelGrid, float radius,
                         const std::function<void(const PointData& voxel)>& func);
  static bool HasPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& pointVoxelGrid, float radius);
  static void ForEachBranchEnd(const glm::vec3& position, VoxelGrid<std::vector<BranchEndData>>& branchEndsVoxelGrid,
                               float radius, const std::function<void(const BranchEndData& voxel)>& func);

  void CalculateNodeTransforms(ReconstructionSkeleton& skeleton);

  void BuildConnectionBranch(BranchHandle processingBranchHandle, SkeletonNodeHandle& prevNodeHandle);

  void Unlink(BranchHandle childHandle, BranchHandle parentHandle);
  void Link(BranchHandle childHandle, BranchHandle parentHandle);

  void GetSortedBranchList(BranchHandle branchHandle, std::vector<BranchHandle>& list);

  void ConnectBranches(BranchHandle branchHandle);

  void ApplyCurve(const OperatorBranch& branch);

  void BuildVoxelGrid();

  static void CloneOperatingBranch(const ReconstructionSettings& reconstructionSettings, OperatorBranch& operatorBranch,
                                   const PredictedBranch& target);

  void SpaceColonization();

  void CalculateBranchRootDistance(const std::vector<std::pair<glm::vec3, BranchHandle>>& rootBranchHandles);

  void CalculateSkeletonGraphs();

 public:
  std::shared_ptr<ParticleInfoList> allocated_point_info_list;
  std::shared_ptr<ParticleInfoList> scattered_point_info_list;
  std::shared_ptr<ParticleInfoList> scattered_point_connection_info_list;
  std::shared_ptr<ParticleInfoList> candidate_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> reversed_candidate_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> filtered_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> selected_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> scatter_point_to_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> selected_branch_info_list;
  glm::vec4 scatter_point_to_branch_connection_color = glm::vec4(1, 0, 1, 1);
  glm::vec4 allocated_point_color = glm::vec4(0, 0.5, 0.25, 1);
  glm::vec4 scatter_point_color = glm::vec4(0.25, 0.5, 0, 1);
  glm::vec4 scattered_point_connection_color = glm::vec4(0, 0, 0, 1);
  glm::vec4 candidate_branch_connection_color = glm::vec4(1, 1, 0, 1);
  glm::vec4 reversed_candidate_branch_connection_color = glm::vec4(0, 1, 1, 1);
  glm::vec4 filtered_branch_connection_color = glm::vec4(0, 0, 1, 1);
  glm::vec4 selected_branch_connection_color = glm::vec4(0.3, 0, 0, 1);
  glm::vec4 selected_branch_color = glm::vec4(0.6, 0.3, 0.0, 1.0f);

  bool enable_allocated_points = false;
  bool enable_scattered_points = false;
  bool enable_scattered_point_connections = false;
  bool enable_scatter_point_to_branch_connections = false;
  bool enable_candidate_branch_connections = false;
  bool enable_reversed_candidate_branch_connections = false;
  bool enable_filtered_branch_connections = false;
  bool enable_selected_branch_connections = true;
  bool enable_selected_branches = true;

  bool debug_allocated_points = true;
  bool debug_scattered_points = true;
  bool debug_scattered_point_connections = false;
  bool debug_scatter_point_to_branch_connections = false;
  bool debug_candidate_connections = false;
  bool debug_reversed_candidate_connections = false;
  bool debug_filtered_connections = false;
  bool debug_selected_branch_connections = true;
  bool debug_selected_branches = true;

  VoxelGrid<std::vector<PointData>> scatter_points_voxel_grid;
  VoxelGrid<std::vector<PointData>> allocated_points_voxel_grid;
  VoxelGrid<std::vector<PointData>> space_colonization_voxel_grid;
  VoxelGrid<std::vector<BranchEndData>> branch_ends_voxel_grid;

  ReconstructionSettings reconstruction_settings{};
  ConnectivityGraphSettings connectivity_graph_settings{};
  void ImportGraph(const std::filesystem::path& path, float scaleFactor = 0.1f);
  void ExportForestOBJ(const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::filesystem::path& path);

  glm::vec3 min;
  glm::vec3 max;
  std::vector<ScatteredPoint> scattered_points;
  std::vector<AllocatedPoint> allocated_points;
  std::vector<PredictedBranch> predicted_branches;

  std::vector<OperatorBranch> operating_branches;
  std::vector<TreePart> tree_parts;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  std::vector<ReconstructionSkeleton> skeletons;

  std::vector<std::pair<glm::vec3, glm::vec3>> scattered_point_to_branch_end_connections;
  std::vector<std::pair<glm::vec3, glm::vec3>> scattered_point_to_branch_start_connections;
  std::vector<std::pair<glm::vec3, glm::vec3>> scattered_points_connections;
  std::vector<std::pair<glm::vec3, glm::vec3>> candidate_branch_connections;
  std::vector<std::pair<glm::vec3, glm::vec3>> reversed_candidate_branch_connections;
  std::vector<std::pair<glm::vec3, glm::vec3>> filtered_branch_connections;
  std::vector<std::pair<glm::vec3, glm::vec3>> branch_connections;
  void EstablishConnectivityGraph();

  void BuildSkeletons();
  void GenerateForest() const;
  void FormInfoEntities() const;
  void ClearForest() const;

  void OnCreate() override;
  AssetRef tree_descriptor;

  std::vector<std::shared_ptr<Mesh>> GenerateForestBranchMeshes(
      const TreeMeshGeneratorSettings& meshGeneratorSettings) const;
  std::vector<std::shared_ptr<Mesh>> GenerateFoliageMeshes();

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace eco_sys_lab