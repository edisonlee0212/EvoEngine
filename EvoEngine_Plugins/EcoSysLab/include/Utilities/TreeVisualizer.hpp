#pragma once

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Jobs.hpp"
#include "Platform.hpp"
#include "RootModel.hpp"
#include "ShootModel.hpp"
#include "StrandModel.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

class BasicFoliageDescriptor;

/**
 * @brief Enumeration for different shoot visualization modes.
 */
enum class ShootVisualizerMode {
  Default,                      ///< Default visualization mode.
  Order,                        ///< Visualize by shoot order.
  Level,                        ///< Visualize by hierarchical level.
  MaxDescendantLightIntensity,  ///< Visualize based on maximum descendant light intensity.
  LightIntensity,               ///< Visualize based on light intensity.
  LightDirection,               ///< Visualize based on light direction.
  DesiredGrowthRate,            ///< Visualize based on desired growth rate.
  GrowthPotential,              ///< Visualize based on growth potential.
  GrowthRate,                   ///< Visualize based on growth rate.
  IsMaxChild,                   ///< Visualize based on max child node property.
  AllocatedVigor,               ///< Visualize based on allocated vigor.
  SaggingStress,                ///< Visualize based on sagging stress.
  SourceSink_Concentration,     ///< Visualize based on source sink concentration.
  SourceSink_Flux,              ///< Visualize based on source sink flux.
  MaxCarbohydrateCapacity,      ///< Visualize based on max carbohydrate capacity.
  Locked                        ///< Locked visualization mode.
};

/**
 * @brief Enumeration for different root visualization modes.
 */
enum class RootVisualizerMode {
  Default,            ///< Default visualization mode.
  Order,              ///< Visualize by shoot order.
  Level,              ///< Visualize by hierarchical level.
  DesiredGrowthRate,  ///< Visualize based on desired growth rate.
  GrowthPotential,    ///< Visualize based on growth potential.
  GrowthRate,         ///< Visualize based on growth rate.
  IsMaxChild,         ///< Visualize based on max child node property.
  AllocatedVigor,     ///< Visualize based on allocated vigor.
  SourceSink_Concentration,     ///< Visualize based on source sink concentration.
  SourceSink_Flux,              ///< Visualize based on source sink flux.
  MaxCarbohydrateCapacity,      ///< Visualize based on max carbohydrate capacity.
  Locked              ///< Locked visualization mode.
};

/**
 * @brief Structure to hold tree visualizer color settings.
 */
struct ShootVisualizerColorSettings {
  int visualization_mode = static_cast<int>(ShootVisualizerMode::Default);  ///< The active shoot visualization mode.
  float color_multiplier = 1.0f;                                            ///< Multiplier for shoot color intensity.
};

/**
 * @brief Structure to hold tree visualizer color settings.
 */
struct RootVisualizerColorSettings {
  int visualization_mode = static_cast<int>(RootVisualizerMode::Default);  ///< The active shoot visualization mode.
  float color_multiplier = 1.0f;                                           ///< Multiplier for shoot color intensity.
};

class TreeVisualizer {
 protected:
  bool initialized_ = false;  ///< Flag to check if the visualizer is initialized.

  std::vector<glm::vec4> random_colors_;  ///< Stores generated random colors.

  std::shared_ptr<ParticleInfoList> node_matrices_;  ///< Stores internode transformation matrices.
  std::shared_ptr<ParticleInfoList>
      base_skeleton_matrices_;  ///< Stores base skeleton internode transformation matrices.
  std::shared_ptr<ParticleInfoList> leaf_matrices_;    ///< Stores leaf transformation matrices.
  std::shared_ptr<ParticleInfoList> flower_matrices_;  ///< Stores flower transformation matrices.
  std::shared_ptr<ParticleInfoList> fruit_matrices_;   ///< Stores fruit transformation matrices.

  static glm::vec4 GetFluxColor(float net_flow, float max_flux);
  static glm::vec4 GetConcentrationColor(float concentration, float max_concentration_capacity);

 public:
  std::vector<SkeletonNodeHandle> selected_node_hierarchy_list;  ///< List of selected internode hierarchy nodes.
  SkeletonNodeHandle selected_node_handle = -1;                  ///< Handle of the selected internode.
  bool visualization = true;                                     ///< Flag to enable or disable visualization.
  float line_thickness = 0.f;                                    ///< Thickness of visualized lines.
  bool profile_gui = true;                                       ///< Flag to toggle profile GUI.
  bool tree_hierarchy_gui = false;                               ///< Flag to toggle tree hierarchy GUI.
  float selected_node_length_factor = 0.0f;                      ///< Length factor for the selected internode.
  int checkpoint_iteration = 0;                                  ///< Iteration count for checkpoints.
  bool need_update = false;                                      ///< Flag indicating if an update is needed.

  /**
   * @brief Checks if the visualizer is initialized.
   * @return True if initialized, otherwise false.
   */
  [[nodiscard]] bool Initialized() const;

  /**
   * @brief Clears all selections in the visualizer.
   */
  void ClearSelections();

  /**
   * @brief Initializes the tree visualizer.
   */
  void Initialize();
  /**
   * @brief Clears all visualization and data.
   */
  void Clear();

  /**
   * @brief Performs a ray-casting selection for an internode.
   * @param camera_component Shared pointer to the camera component.
   * @param mouse_position The mouse position in screen coordinates.
   * @param skeleton The shoot skeleton.
   * @param global_transform The global transformation matrix.
   * @return True if an internode is selected, otherwise false.
   */
  template <typename SkeletonData, typename FlowData, typename NodeData>
  bool RayCastSelection(const std::shared_ptr<Camera>& camera_component, const glm::vec2& mouse_position,
                        const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                        const GlobalTransform& global_transform);

  /**
   * @brief Handles selection of internodes along a screen-drawn curve.
   * @param handler Callback function executed on selected nodes.
   * @param mouse_positions List of mouse positions forming a curve.
   * @param skeleton The shoot skeleton.
   * @param global_transform The global transformation matrix.
   * @param projection_view Projection View matrix
   * @return True if selection is successful, otherwise false.
   */
  template <typename SkeletonData, typename FlowData, typename NodeData>
  bool ScreenCurveSelection(const std::function<void(SkeletonNodeHandle)>& handler,
                            std::vector<glm::vec2>& mouse_positions,
                            Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                            const GlobalTransform& global_transform, const glm::mat4& projection_view);

  /**
   * @brief Sets the selected node in the shoot skeleton.
   * @param skeleton Reference to the shoot skeleton.
   * @param node_handle Handle of the node to select.
   */
  template <typename SkeletonData, typename FlowData, typename NodeData>
  void SetSelectedNode(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, SkeletonNodeHandle node_handle);
};

template <typename SkeletonData, typename FlowData, typename NodeData>
bool TreeVisualizer::RayCastSelection(const std::shared_ptr<Camera>& camera_component, const glm::vec2& mouse_position,
                                      const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                                      const GlobalTransform& global_transform) {
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  bool changed = false;
#pragma region Ray selection
  SkeletonNodeHandle current_focusing_node_handle = -1;
  std::mutex write_mutex;
  float min_distance = FLT_MAX;
  GlobalTransform camera_ltw;
  camera_ltw.value =
      glm::translate(editor_layer->GetSceneCameraPosition()) * glm::mat4_cast(editor_layer->GetSceneCameraRotation());
  const Ray camera_ray = camera_component->ScreenPointToRay(camera_ltw, mouse_position);
  const auto& sorted_node_list = skeleton.PeekSortedNodeList();
  Jobs::RunParallelFor(sorted_node_list.size(), [&](unsigned i) {
    const auto node_handle = sorted_node_list[i];
    SkeletonNodeHandle walker = node_handle;
    bool sub_tree = false;
    while (walker != -1) {
      if (walker == selected_node_handle) {
        sub_tree = true;
        break;
      }
      walker = skeleton.PeekNode(walker).GetParentHandle();
    }
    const auto& node = skeleton.PeekNode(node_handle);
    auto rotation = global_transform.GetRotation() * node.info.global_rotation;
    glm::vec3 position = (global_transform.value * glm::translate(node.info.global_position))[3];
    const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
    const glm::vec3 position2 = position + node.info.length * direction;
    const auto center = (position + position2) / 2.0f;
    auto radius = node.info.thickness;
    if (line_thickness != 0.0f) {
      radius = line_thickness * (sub_tree ? 0.625f : 0.5f);
    }
    const auto height = glm::distance(position2, position);
    radius *= height / node.info.length;
    if (!camera_ray.Intersect(center, height / 2.0f) && !camera_ray.Intersect(center, radius)) {
      return;
    }
    const auto& dir = -camera_ray.direction;
#pragma region Line Line intersection
    /*
     * http://geomalgorithms.com/a07-_distance.html
     */
    glm::vec3 v = position - position2;
    glm::vec3 w = (camera_ray.start + dir) - position2;
    const auto a = glm::dot(dir, dir);  // always >= 0
    const auto b = glm::dot(dir, v);
    const auto c = glm::dot(v, v);  // always >= 0
    const auto d = glm::dot(dir, w);
    const auto e = glm::dot(v, w);
    const auto dot_p = a * c - b * b;  // always >= 0
    float sc, tc;
    // compute the line parameters of the two closest points
    if (dot_p < 0.00001f) {
      // the lines are almost parallel
      sc = 0.0f;
      tc = (b > c ? d / b : e / c);  // use the largest denominator
    } else {
      sc = (b * e - c * d) / dot_p;
      tc = (a * e - b * d) / dot_p;
    }
    // get the difference of the two closest points
    glm::vec3 d_p = w + sc * dir - tc * v;  // =  L1(sc) - L2(tc)
    if (glm::length(d_p) > radius)
      return;
#pragma endregion

    const auto distance = glm::distance(glm::vec3(camera_ltw.value[3]), glm::vec3(center));
    std::lock_guard<std::mutex> lock(write_mutex);
    if (distance < min_distance) {
      min_distance = distance;
      selected_node_length_factor = glm::clamp(1.0f - tc, 0.0f, 1.0f);
      current_focusing_node_handle = sorted_node_list[i];
    }
  });
  if (current_focusing_node_handle != -1) {
    SetSelectedNode(skeleton, current_focusing_node_handle);
    changed = true;
#pragma endregion
  }
  return changed;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
bool TreeVisualizer::ScreenCurveSelection(const std::function<void(SkeletonNodeHandle)>& handler,
                                          std::vector<glm::vec2>& mouse_positions,
                                          Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                                          const GlobalTransform& global_transform, const glm::mat4& projection_view) {
  const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
  bool changed = false;
  for (const auto& internode_handle : sorted_internode_list) {
    if (internode_handle == 0)
      continue;
    auto& internode = skeleton.RefNode(internode_handle);
    glm::vec3 position = internode.info.global_position;
    auto rotation = internode.info.global_rotation;
    const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
    auto position2 = position + internode.info.length * direction;

    position = (global_transform.value * glm::translate(position))[3];
    position2 = (global_transform.value * glm::translate(position2))[3];
    const glm::vec4 internode_screen_start4 = projection_view * glm::vec4(position, 1.0f);
    const glm::vec4 internode_screen_end4 = projection_view * glm::vec4(position2, 1.0f);
    glm::vec3 internode_screen_start = internode_screen_start4 / internode_screen_start4.w;
    glm::vec3 internode_screen_end = internode_screen_end4 / internode_screen_end4.w;
    internode_screen_start.x *= -1.0f;
    internode_screen_end.x *= -1.0f;
    if (internode_screen_start.x < -1.0f || internode_screen_start.x > 1.0f || internode_screen_start.y < -1.0f ||
        internode_screen_start.y > 1.0f || internode_screen_start.z < 0.0f)
      continue;
    if (internode_screen_end.x < -1.0f || internode_screen_end.x > 1.0f || internode_screen_end.y < -1.0f ||
        internode_screen_end.y > 1.0f || internode_screen_end.z < 0.0f)
      continue;
    bool intersect = false;
    for (int i = 0; i < mouse_positions.size() - 1; i++) {
      auto& line_start = mouse_positions[i];
      auto& line_end = mouse_positions[i + 1];
      float a1 = internode_screen_end.y - internode_screen_start.y;
      float b1 = internode_screen_start.x - internode_screen_end.x;
      float c1 = a1 * (internode_screen_start.x) + b1 * (internode_screen_start.y);

      // Line CD represented as a2x + b2y = c2
      float a2 = line_end.y - line_start.y;
      float b2 = line_start.x - line_end.x;
      float c2 = a2 * (line_start.x) + b2 * (line_start.y);

      float determinant = a1 * b2 - a2 * b1;
      if (determinant == 0.0f)
        continue;
      float x = (b2 * c1 - b1 * c2) / determinant;
      float y = (a1 * c2 - a2 * c1) / determinant;
      if (x <= glm::max(internode_screen_start.x, internode_screen_end.x) &&
          x >= glm::min(internode_screen_start.x, internode_screen_end.x) &&
          y <= glm::max(internode_screen_start.y, internode_screen_end.y) &&
          y >= glm::min(internode_screen_start.y, internode_screen_end.y) && x <= glm::max(line_start.x, line_end.x) &&
          x >= glm::min(line_start.x, line_end.x) && y <= glm::max(line_start.y, line_end.y) &&
          y >= glm::min(line_start.y, line_end.y)) {
        intersect = true;
        break;
      }
    }
    if (intersect) {
      handler(internode_handle);
      changed = true;
    }
  }
  if (changed) {
    selected_node_handle = -1;
    selected_node_hierarchy_list.clear();
  }
  return changed;
}

template <typename SkeletonData, typename FlowData, typename NodeData>
void TreeVisualizer::SetSelectedNode(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
                                     const SkeletonNodeHandle node_handle) {
  if (node_handle != selected_node_handle) {
    selected_node_hierarchy_list.clear();
    if (node_handle < 0) {
      selected_node_handle = -1;
    } else {
      selected_node_handle = node_handle;
      auto walker = node_handle;
      while (walker != -1) {
        selected_node_hierarchy_list.push_back(walker);
        const auto& internode = skeleton.PeekNode(walker);
        walker = internode.GetParentHandle();
      }
    }
  }
}

/**
 * @brief Class for visualizing tree structures and internodes.
 */
class ShootVisualizer : public TreeVisualizer {
 public:
  bool leaf_visualization_ = true;
  bool flower_visualization_ = false;
  bool fruit_visualization_ = true;

 public:

  float global_max_flux_ = 0.0f;
  float global_min_capacity_ = FLT_MAX;
  float global_max_capacity_ = 0.0f;
  const RootSkeleton* stats_root_skeleton_ = nullptr;
  void CalculateStatistics(const ShootSkeleton& skeleton);
  void SetStatsRootSkeleton(const RootSkeleton& skeleton) {
    stats_root_skeleton_ = &skeleton;
  }

  /**
   * @brief Draws the GUI for inspecting an internode.
   * @param tree_model Reference to the tree model.
   * @param internode_handle Handle of the internode to inspect.
   * @param deleted Output flag indicating if the internode is deleted.
   * @param hierarchy_level The hierarchy level of the internode.
   * @return True if the inspection was successful, otherwise false.
   */
  bool DrawInternodeInspectionGui(ShootModel& tree_model, SkeletonNodeHandle internode_handle, bool& deleted,
                                  const unsigned& hierarchy_level);

  /**
   * @brief Displays GUI elements for inspecting a tree node.
   * @param skeleton Reference to the shoot skeleton.
   * @param node_handle Handle of the node to inspect.
   * @param hierarchy_level The hierarchy level of the node.
   */
  void PeekNodeInspectionGui(const ShootSkeleton& skeleton, SkeletonNodeHandle node_handle,
                             const unsigned& hierarchy_level);

  /**
   * @brief Peeks at an internode without modifying it.
   * @param skeleton Reference to the shoot skeleton.
   * @param internode_handle Handle of the internode.
   */
  void PeekInternode(const ShootSkeleton& skeleton, SkeletonNodeHandle internode_handle) const;

  /**
   * @brief Inspects a specific internode.
   * @param skeleton Reference to the shoot skeleton.
   * @param internode_handle Handle of the internode to inspect.
   * @return True if inspection is successful, otherwise false.
   */
  bool InspectInternode(ShootSkeleton& skeleton, SkeletonNodeHandle internode_handle);

 public:
  ShootVisualizerColorSettings tree_visualizer_color_settings;  ///< Settings for visualization color.

  /**
   * @brief Handles inspection of the tree model.
   * @param model Reference to the tree model.
   * @return True if contents remain unmodified, otherwise false.
   */
  bool OnInspect(ShootModel& model);

  /**
   * @brief Visualizes the given tree model.
   * @param model The tree model to visualize.
   * @param global_transform The global transformation matrix.
   * @param root_model Optional root model for shared statistics.
   */
  void Visualize(const ShootModel& model, const GlobalTransform& global_transform,
                 const RootModel* root_model = nullptr, const StrandModel* strand_model = nullptr,
                 const std::shared_ptr<BasicFoliageDescriptor>& foliage_descriptor = nullptr);

  /**
   * @brief Visualizes the given strand model.
   * @param strand_model The strand model to visualize.
   */
  void Visualize(StrandModel& strand_model);

  /**
   * @brief Resets the visualization of a tree model.
   * @param model The tree model to reset.
   */
  void Reset(const ShootModel& model);

  /**
   * @brief Synchronizes transformation matrices between skeleton and internode list.
   * @param skeleton Reference to the shoot skeleton.
   * @param particle_info_list Shared pointer to the list of particles.
   */
  void SyncMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particle_info_list);

  void SyncFoliageMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particle_info_list,
                          const StrandModel* strand_model = nullptr,
                          const std::shared_ptr<BasicFoliageDescriptor>& foliage_descriptor = nullptr);
  void SyncFlowerMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particle_info_list);
  void SyncFruitMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particle_info_list);

};

class RootVisualizer : public TreeVisualizer {
 public:
  float global_max_flux_ = 0.0f;
  float global_min_capacity_ = FLT_MAX;
  float global_max_capacity_ = 0.0f;
  const ShootSkeleton* stats_shoot_skeleton_ = nullptr;
  void CalculateStatistics(const RootSkeleton& skeleton);
  void SetStatsShootSkeleton(const ShootSkeleton& skeleton) { stats_shoot_skeleton_ = &skeleton; }

  /**
   * @brief Draws the GUI for inspecting an internode.
   * @param root_model Reference to the tree model.
   * @param node_handle Handle of the internode to inspect.
   * @param deleted Output flag indicating if the internode is deleted.
   * @param hierarchy_level The hierarchy level of the internode.
   * @return True if the inspection was successful, otherwise false.
   */
  bool DrawNodeInspectionGui(RootModel& root_model, SkeletonNodeHandle node_handle, bool& deleted,
                             const unsigned& hierarchy_level);

  /**
   * @brief Displays GUI elements for inspecting a tree node.
   * @param skeleton Reference to the shoot skeleton.
   * @param node_handle Handle of the node to inspect.
   * @param hierarchy_level The hierarchy level of the node.
   */
  void PeekNodeInspectionGui(const RootSkeleton& skeleton, SkeletonNodeHandle node_handle,
                             const unsigned& hierarchy_level);

  /**
   * @brief Peeks at an internode without modifying it.
   * @param skeleton Reference to the shoot skeleton.
   * @param node_handle Handle of the internode.
   */
  void PeekRootNode(const RootSkeleton& skeleton, SkeletonNodeHandle node_handle) const;

  /**
   * @brief Inspects a specific internode.
   * @param skeleton Reference to the shoot skeleton.
   * @param node_handle Handle of the internode to inspect.
   * @return True if inspection is successful, otherwise false.
   */
  bool InspectRootNode(RootSkeleton& skeleton, SkeletonNodeHandle node_handle);

 public:
  RootVisualizerColorSettings root_visualizer_color_settings;  ///< Settings for visualization color.

  /**
   * @brief Handles inspection of the tree model.
   * @param model Reference to the tree model.
   * @return True if contents remain unmodified, otherwise false.
   */
  bool OnInspect(RootModel& model);

  /**
   * @brief Visualizes the given tree model.
   * @param model The tree model to visualize.
   * @param global_transform The global transformation matrix.
   * @param shoot_model Optional shoot model for shared statistics.
   */
  void Visualize(const RootModel& model, const GlobalTransform& global_transform,
                 const ShootModel* shoot_model = nullptr);
  /**
   * @brief Synchronizes transformation matrices between skeleton and internode list.
   * @param skeleton Reference to the shoot skeleton.
   * @param particle_info_list Shared pointer to the list of particles.
   */
  void SyncMatrices(const RootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particle_info_list);
  /**
   * @brief Resets the visualization of a tree model.
   * @param root_model The tree model to reset.
   */
  void Reset(const RootModel& root_model);
};
}  // namespace eco_sys_lab_plugin