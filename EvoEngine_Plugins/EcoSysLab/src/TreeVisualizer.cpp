//
// Created by lllll on 11/20/2022.
//

#include "TreeVisualizer.hpp"
#include "Application.hpp"
#include "EcoSysLabLayer.hpp"
#include "ProfileConstraints.hpp"
#include "Utilities.hpp"
using namespace eco_sys_lab_plugin;

bool TreeVisualizer::ScreenCurveSelection(const std::function<void(SkeletonNodeHandle)>& handler,
                                          std::vector<glm::vec2>& mouse_positions, ShootSkeleton& skeleton,
                                          const GlobalTransform& global_transform) {
  auto editor_layer = Application::GetLayer<EditorLayer>();
  auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  const auto camera_rotation = editor_layer->GetSceneCameraRotation();
  const auto camera_position = editor_layer->GetSceneCameraPosition();
  const glm::vec3 camera_front = camera_rotation * glm::vec3(0, 0, -1);
  const glm::vec3 camera_up = camera_rotation * glm::vec3(0, 1, 0);
  glm::mat4 projection_view = eco_sys_lab_layer->visualization_camera_->GetProjection() *
                              glm::lookAt(camera_position, camera_position + camera_front, camera_up);

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
    selected_internode_handle = -1;
    selected_internode_hierarchy_list.clear();
  }
  return changed;
}

bool TreeVisualizer::RayCastSelection(const std::shared_ptr<Camera>& camera_component, const glm::vec2& mouse_position,
                                      const ShootSkeleton& skeleton, const GlobalTransform& global_transform) {
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
      if (walker == selected_internode_handle) {
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
    if (dot_p < 0.00001f) {  // the lines are almost parallel
      sc = 0.0f;
      tc = (b > c ? d / b : e / c);  // use the largest denominator
    } else {
      sc = (b * e - c * d) / dot_p;
      tc = (a * e - b * d) / dot_p;
    }
    // get the difference of the two closest points
    glm::vec3 dP = w + sc * dir - tc * v;  // =  L1(sc) - L2(tc)
    if (glm::length(dP) > radius)
      return;
#pragma endregion

    const auto distance = glm::distance(glm::vec3(camera_ltw.value[3]), glm::vec3(center));
    std::lock_guard<std::mutex> lock(write_mutex);
    if (distance < min_distance) {
      min_distance = distance;
      selected_internode_length_factor = glm::clamp(1.0f - tc, 0.0f, 1.0f);
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

void TreeVisualizer::PeekNodeInspectionGui(const ShootSkeleton& skeleton, const SkeletonNodeHandle node_handle,
                                           const unsigned& hierarchy_level) {
  const int index = selected_internode_hierarchy_list.size() - hierarchy_level - 1;
  if (!selected_internode_hierarchy_list.empty() && index >= 0 && index < selected_internode_hierarchy_list.size() &&
      selected_internode_hierarchy_list[index] == node_handle) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      ("Handle: " + std::to_string(node_handle)).c_str(),
      ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
          (selected_internode_handle == node_handle ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    SetSelectedNode(skeleton, node_handle);
  }
  if (opened) {
    ImGui::TreePush(std::to_string(node_handle).c_str());
    const auto& internode = skeleton.PeekNode(node_handle);
    const auto& internode_children = internode.PeekChildHandles();
    for (const auto& child : internode_children) {
      PeekNodeInspectionGui(skeleton, child, hierarchy_level + 1);
    }
    ImGui::TreePop();
  }
}

void TreeVisualizer::SetSelectedNode(const ShootSkeleton& skeleton, const SkeletonNodeHandle node_handle) {
  if (node_handle != selected_internode_handle) {
    selected_internode_hierarchy_list.clear();
    if (node_handle < 0) {
      selected_internode_handle = -1;
    } else {
      selected_internode_handle = node_handle;
      auto walker = node_handle;
      while (walker != -1) {
        selected_internode_hierarchy_list.push_back(walker);
        const auto& internode = skeleton.PeekNode(walker);
        walker = internode.GetParentHandle();
      }
    }
  }
}

void TreeVisualizer::SyncMatrices(const ShootSkeleton& skeleton,
                                  const std::shared_ptr<ParticleInfoList>& particle_info_list,
                                  SkeletonNodeHandle selected_node_handle) {
  if (random_colors_.empty()) {
    for (int i = 0; i < 1000; i++) {
      random_colors_.emplace_back(glm::abs(glm::ballRand(1.0f)), 1.0f);
    }
  }
  const auto& sorted_node_list = skeleton.PeekSortedNodeList();
  std::vector<ParticleInfo> matrices;

  matrices.resize(sorted_node_list.size());
  Jobs::RunParallelFor(sorted_node_list.size(), [&](unsigned i) {
    const auto node_handle = sorted_node_list[i];
    const auto& node = skeleton.PeekNode(node_handle);
    bool subTree = false;
    SkeletonNodeHandle walker = node_handle;
    while (walker != -1) {
      if (walker == selected_internode_handle) {
        subTree = true;
        break;
      }
      walker = skeleton.PeekNode(walker).GetParentHandle();
    }
    auto rotation = node.info.global_rotation;
    rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
    const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
    if (line_thickness != 0.0f) {
      matrices[i].instance_matrix.value =
          glm::translate(node.info.global_position + (node.info.length / 2.0f) * node.info.GetGlobalDirection()) *
          rotation_transform *
          glm::scale(glm::vec3(line_thickness * (subTree ? 1.25f : 1.0f), node.info.length,
                               line_thickness * (subTree ? 1.25f : 1.0f)));
    } else {
      matrices[i].instance_matrix.value =
          glm::translate(node.info.global_position + (node.info.length / 2.0f) * node.info.GetGlobalDirection()) *
          rotation_transform * glm::scale(glm::vec3(node.info.thickness, node.info.length, node.info.thickness));
    }
  });
  Jobs::RunParallelFor(sorted_node_list.size(), [&](unsigned i) {
    const auto node_handle = sorted_node_list[i];
    const auto& node = skeleton.PeekNode(node_handle);
    switch (static_cast<ShootVisualizerMode>(tree_visualizer_color_settings.shoot_visualization_mode)) {
      case ShootVisualizerMode::Default:
        matrices[i].instance_color = random_colors_[node_handle % random_colors_.size()];
        break;
      case ShootVisualizerMode::Order:
        matrices[i].instance_color = random_colors_[node.data.order];
        break;
      case ShootVisualizerMode::Locked:
        matrices[i].instance_color = node.info.locked ? glm::vec4(1, 0, 0, 1) : glm::vec4(0, 1, 0, 1);
        break;
      case ShootVisualizerMode::Level:
        matrices[i].instance_color = random_colors_[node.data.level];
        break;
      case ShootVisualizerMode::MaxDescendantLightIntensity:
        matrices[i].instance_color =
            glm::mix(glm::vec4(0, 0, 0, 1), glm::vec4(1, 1, 1, 1),
                     glm::clamp(glm::pow(node.data.descendant_total_light_intake,
                                         tree_visualizer_color_settings.shoot_color_multiplier),
                                0.0f, 1.f));
        break;
      case ShootVisualizerMode::LightIntensity:
        matrices[i].instance_color =
            glm::mix(glm::vec4(0, 0, 0, 1), glm::vec4(1, 1, 1, 1),
                     glm::clamp(glm::pow(node.data.light_intake, tree_visualizer_color_settings.shoot_color_multiplier),
                                0.0f, 1.f));
        break;
      case ShootVisualizerMode::LightDirection:
        matrices[i].instance_color = glm::vec4(glm::vec3(glm::clamp(node.data.light_direction, 0.0f, 1.f)), 1.0f);
        break;
      case ShootVisualizerMode::IsMaxChild:
        matrices[i].instance_color = glm::vec4(glm::vec3(node.data.max_child ? 1.0f : 0.0f), 1.0f);
        break;
      case ShootVisualizerMode::DesiredGrowthRate:
        matrices[i].instance_color = glm::mix(
            glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1),
            glm::clamp(glm::pow(node.data.desired_growth_rate, tree_visualizer_color_settings.shoot_color_multiplier),
                       0.0f, 1.f));
        break;
      case ShootVisualizerMode::GrowthPotential:
        matrices[i].instance_color = glm::mix(
            glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1),
            glm::clamp(glm::pow(node.data.growth_potential, tree_visualizer_color_settings.shoot_color_multiplier),
                       0.0f, 1.f));
        break;
      case ShootVisualizerMode::SaggingStress:
        matrices[i].instance_color = glm::mix(
            glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1),
            glm::clamp(glm::pow(node.data.sagging_stress, tree_visualizer_color_settings.shoot_color_multiplier), 0.0f,
                       1.f));
        break;
      case ShootVisualizerMode::GrowthRate:
        matrices[i].instance_color =
            glm::mix(glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1),
                     glm::clamp(glm::pow(node.data.growth_rate, tree_visualizer_color_settings.shoot_color_multiplier),
                                0.0f, 1.f));
        break;
      default:
        matrices[i].instance_color = random_colors_[node.data.order];
        break;
    }
    matrices[i].instance_color.a = 1.0f;
    if (selected_node_handle != -1)
      matrices[i].instance_color.a = 1.0f;
  });
  particle_info_list->SetParticleInfos(matrices);
}

bool TreeVisualizer::DrawInternodeInspectionGui(ShootModel& tree_model, const SkeletonNodeHandle internode_handle,
                                                bool& deleted, const unsigned& hierarchy_level) {
  auto& treeSkeleton = tree_model.RefShootSkeleton();
  const int index = selected_internode_hierarchy_list.size() - hierarchy_level - 1;
  if (!selected_internode_hierarchy_list.empty() && index >= 0 && index < selected_internode_hierarchy_list.size() &&
      selected_internode_hierarchy_list[index] == internode_handle) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      ("Handle: " + std::to_string(internode_handle)).c_str(),
      ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
          (selected_internode_handle == internode_handle ? ImGuiTreeNodeFlags_Framed
                                                         : ImGuiTreeNodeFlags_FramePadding));
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    SetSelectedNode(treeSkeleton, internode_handle);
  }

  if (ImGui::BeginPopupContextItem(std::to_string(internode_handle).c_str())) {
    ImGui::Text(("Handle: " + std::to_string(internode_handle)).c_str());
    if (ImGui::Button("Delete")) {
      deleted = true;
    }
    ImGui::EndPopup();
  }
  bool modified = deleted;
  if (opened && !deleted) {
    ImGui::TreePush(std::to_string(internode_handle).c_str());
    const auto& internode_children = treeSkeleton.RefNode(internode_handle).PeekChildHandles();
    for (const auto& child : internode_children) {
      bool child_deleted = false;
      DrawInternodeInspectionGui(tree_model, child, child_deleted, hierarchy_level + 1);
      if (child_deleted) {
        tree_model.Step();
        tree_model.RemoveNodes({child});
        checkpoint_iteration = tree_model.CurrentIteration();
        modified = true;
        break;
      }
    }
    ImGui::TreePop();
  }
  return modified;
}
void TreeVisualizer::ClearSelections() {
  selected_internode_handle = -1;
}
bool TreeVisualizer::OnInspect(ShootModel& tree_model) {
  bool updated = false;
  if (ImGui::Combo("Visualizer mode",
                   {"Default", "Order", "Level", "Max descendant light intensity", "Light intensity", "Light direction",
                    "Desired growth rate", "Growth potential", "Growth rate", "Is max child", "Allocated vigor",
                    "Sagging stress", "Locked"},
                   tree_visualizer_color_settings.shoot_visualization_mode)) {
    need_update = true;
  }
  if (ImGui::TreeNodeEx("Checkpoints")) {
    if (ImGui::SliderInt("Current checkpoint", &checkpoint_iteration, 0, tree_model.CurrentIteration())) {
      checkpoint_iteration = glm::clamp(checkpoint_iteration, 0, tree_model.CurrentIteration());
      selected_internode_handle = -1;
      selected_internode_hierarchy_list.clear();
      need_update = true;
    }
    if (checkpoint_iteration != tree_model.CurrentIteration() && ImGui::Button("Reverse")) {
      tree_model.Reverse(checkpoint_iteration);
      need_update = true;
    }
    if (ImGui::Button("Clear checkpoints")) {
      checkpoint_iteration = 0;
      tree_model.ClearHistory();
    }
    ImGui::TreePop();
  }
  if (ImGui::Button("Add Checkpoint")) {
    tree_model.Step();
    checkpoint_iteration = tree_model.CurrentIteration();
  }
  if (ImGui::TreeNodeEx("Visualizer Settings")) {
    ImGui::DragInt("History Limit", &tree_model.history_limit, 1, -1, 1024);

    if (ImGui::TreeNode("Shoot Color settings")) {
      if (ImGui::DragFloat("Multiplier", &tree_visualizer_color_settings.shoot_color_multiplier, 0.001f)) {
        need_update = true;
      }
      switch (static_cast<ShootVisualizerMode>(tree_visualizer_color_settings.shoot_visualization_mode)) {
        default:
          break;
      }
      ImGui::TreePop();
    }

    ImGui::Checkbox("Visualization", &visualization);
    ImGui::Checkbox("Profile", &profile_gui);
    ImGui::Checkbox("Tree Hierarchy", &tree_hierarchy_gui);

    if (visualization) {
      const auto& tree_skeleton = tree_model.PeekShootSkeleton(checkpoint_iteration);
      const auto editor_layer = Application::GetLayer<EditorLayer>();
      const auto& sorted_branch_list = tree_skeleton.PeekSortedFlowList();
      const auto& sorted_internode_list = tree_skeleton.PeekSortedNodeList();
      ImGui::Text("Internode count: %d", sorted_internode_list.size());
      ImGui::Text("Shoot stem count: %d", sorted_branch_list.size());
    }

    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Inspection")) {
    if (selected_internode_handle >= 0) {
      if (checkpoint_iteration == tree_model.CurrentIteration()) {
        InspectInternode(tree_model.RefShootSkeleton(), selected_internode_handle);
      } else {
        PeekInternode(tree_model.PeekShootSkeleton(checkpoint_iteration), selected_internode_handle);
      }
    }

    if (tree_hierarchy_gui) {
      if (ImGui::TreeNodeEx("Tree Hierarchy")) {
        bool deleted = false;
        if (checkpoint_iteration == tree_model.CurrentIteration()) {
          if (DrawInternodeInspectionGui(tree_model, 0, deleted, 0)) {
            need_update = true;
            updated = true;
          }
        } else
          PeekNodeInspectionGui(tree_model.PeekShootSkeleton(checkpoint_iteration), 0, 0);
        selected_internode_hierarchy_list.clear();
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  return updated;
}

void TreeVisualizer::Visualize(const ShootModel& tree_model, const GlobalTransform& global_transform) {
  const auto& tree_skeleton = tree_model.PeekShootSkeleton(checkpoint_iteration);
  if (visualization) {
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
    if (need_update) {
      SyncMatrices(tree_skeleton, internode_matrices_, selected_internode_handle);
      need_update = false;
    }
    GizmoSettings gizmo_settings;
    gizmo_settings.draw_settings.blending = true;
    gizmo_settings.depth_test = true;
    gizmo_settings.depth_write = true;
    if (!internode_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cylinder,
                                                  eco_sys_lab_layer->visualization_camera_, internode_matrices_,
                                                  global_transform.value, 1.0f, gizmo_settings);
      if (selected_internode_handle != -1) {
        const auto& node = tree_skeleton.PeekNode(selected_internode_handle);
        auto rotation = node.info.global_rotation;
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        const glm::vec3 selected_center = node.info.global_position + node.info.length *
                                                                          selected_internode_length_factor *
                                                                          node.info.GetGlobalDirection();
        const auto matrix = global_transform.value * glm::translate(selected_center) * rotation_transform *
                            glm::scale(glm::vec3(2.0f * node.info.thickness + 0.01f, node.info.length / 5.0f,
                                                 2.0f * node.info.thickness + 0.01f));
        constexpr auto color = glm::vec4(1.0f);
        editor_layer->DrawGizmoMesh(Resources::Primitives::cylinder, eco_sys_lab_layer->visualization_camera_, color,
                                    matrix, 1, gizmo_settings);
      }
    }
  }
}

void TreeVisualizer::Visualize(StrandModel& strand_model) {
  if (visualization) {
    auto& skeleton = strand_model.strand_model_skeleton;
    static bool show_grid = false;
    if (profile_gui) {
      const std::string tag = "Profile";
      if (ImGui::Begin(tag.c_str())) {
        if (selected_internode_handle != -1 && selected_internode_handle < skeleton.RefRawNodes().size()) {
          auto& node = skeleton.RefNode(selected_internode_handle);
          glm::vec2 mouse_position{};
          static bool last_frame_clicked = false;
          bool mouse_down = false;
          static bool add_attractor = false;
          ImGui::Checkbox("Attractor", &add_attractor);
          if (ImGui::Button("Clear boundaries")) {
            node.data.profile_constraints.boundaries.clear();
            node.data.boundaries_updated = true;
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear attractors")) {
            node.data.profile_constraints.attractors.clear();
            node.data.boundaries_updated = true;
          }
          ImGui::SameLine();
          ImGui::Checkbox("Show Grid", &show_grid);
          if (node.GetParentHandle() != -1) {
            if (ImGui::Button("Copy from root")) {
              std::vector<SkeletonNodeHandle> parent_node_to_root_chain;
              parent_node_to_root_chain.emplace_back(selected_internode_handle);
              SkeletonNodeHandle walker = node.GetParentHandle();
              while (walker != -1) {
                parent_node_to_root_chain.emplace_back(walker);
                walker = skeleton.PeekNode(walker).GetParentHandle();
              }
              for (auto it = parent_node_to_root_chain.rbegin() + 1; it != parent_node_to_root_chain.rend(); ++it) {
                const auto& from_node = skeleton.PeekNode(*(it - 1));
                auto& to_node = skeleton.RefNode(*it);
                to_node.data.profile_constraints = from_node.data.profile_constraints;
                to_node.data.boundaries_updated = true;
              }
            }
            const auto& parent_node = skeleton.RefNode(node.GetParentHandle());
            if (!parent_node.data.profile_constraints.boundaries.empty() ||
                !parent_node.data.profile_constraints.attractors.empty()) {
              ImGui::SameLine();
              if (ImGui::Button("Copy parent settings")) {
                node.data.profile_constraints = parent_node.data.profile_constraints;
                node.data.boundaries_updated = true;
              }
            }
          }
          node.data.profile.OnInspect(
              [&](const glm::vec2 position) {
                mouse_down = true;
                mouse_position = position;
              },
              [&](const ImVec2 origin, const float zoom_factor, ImDrawList* draw_list) {
                node.data.profile.RenderEdges(origin, zoom_factor, draw_list, IM_COL32(0.0f, 0.0f, 128.0f, 128.0f),
                                              1.0f);
                node.data.profile.RenderBoundary(origin, zoom_factor, draw_list, IM_COL32(255.f, 255.f, 255.0f, 255.0f),
                                                 4.0f);

                if (node.GetParentHandle() != -1) {
                  if (const auto& parent_node = skeleton.RefNode(node.GetParentHandle());
                      !parent_node.data.profile_constraints.boundaries.empty()) {
                    for (const auto& parent_boundary : parent_node.data.profile_constraints.boundaries) {
                      parent_boundary.RenderBoundary(origin, zoom_factor, draw_list, IM_COL32(128.0f, 0.0f, 0, 128.0f),
                                                     4.0f);
                    }
                    for (const auto& parent_attractor : parent_node.data.profile_constraints.attractors) {
                      parent_attractor.RenderAttractor(origin, zoom_factor, draw_list,
                                                       IM_COL32(0.0f, 128.0f, 0, 128.0f), 4.0f);
                    }
                  }
                }
                for (const auto& boundary : node.data.profile_constraints.boundaries) {
                  boundary.RenderBoundary(origin, zoom_factor, draw_list, IM_COL32(255.0f, 0.0f, 0, 255.0f), 2.0f);
                }

                for (const auto& attractor : node.data.profile_constraints.attractors) {
                  attractor.RenderAttractor(origin, zoom_factor, draw_list, IM_COL32(0.0f, 255.0f, 0, 255.0f), 2.0f);
                }
              },
              show_grid);
          auto& profile_boundaries = node.data.profile_constraints;
          static glm::vec2 attractor_start_mouse_position;
          if (last_frame_clicked) {
            if (mouse_down) {
              if (!add_attractor) {
                // Continue recording.
                if (glm::distance(mouse_position, profile_boundaries.boundaries.back().points.back()) > 1.0f)
                  profile_boundaries.boundaries.back().points.emplace_back(mouse_position);
              } else {
                if (auto& attractor_points = profile_boundaries.attractors.back().attractor_points;
                    attractor_points.empty()) {
                  if (glm::distance(attractor_start_mouse_position, mouse_position) > 1.0f) {
                    attractor_points.emplace_back(attractor_start_mouse_position, mouse_position);
                  }
                } else if (glm::distance(mouse_position, attractor_points.back().second) > 1.0f) {
                  attractor_points.emplace_back(attractor_points.back().second, mouse_position);
                }
              }
            } else if (!profile_boundaries.boundaries.empty()) {
              if (!add_attractor) {
                // Stop and check boundary.
                if (!profile_boundaries.Valid(profile_boundaries.boundaries.size() - 1)) {
                  profile_boundaries.boundaries.pop_back();
                } else {
                  profile_boundaries.boundaries.back().CalculateCenter();
                  node.data.boundaries_updated = true;
                }
              } else {
                // Stop and check attractors.
                node.data.boundaries_updated = true;
              }
            }
          } else if (mouse_down) {
            // Start recording.
            if (!add_attractor) {
              node.data.profile_constraints.boundaries.emplace_back();
              node.data.profile_constraints.boundaries.back().points.push_back(mouse_position);
            } else {
              node.data.profile_constraints.attractors.emplace_back();
              attractor_start_mouse_position = mouse_position;
            }
          }
          last_frame_clicked = mouse_down;
        } else {
          ImGui::Text("Select an internode to show its profile!");
        }
      }
      ImGui::End();
    }
  }
}

bool TreeVisualizer::InspectInternode(ShootSkeleton& shoot_skeleton, SkeletonNodeHandle internode_handle) {
  bool changed = false;

  auto& internode = shoot_skeleton.RefNode(internode_handle);
  if (internode.info.locked && ImGui::Button("Unlock")) {
    const auto sub_tree = shoot_skeleton.GetSubTree(internode_handle);
    for (const auto& handle : sub_tree) {
      shoot_skeleton.RefNode(handle).info.locked = false;
    }
    need_update = true;
  }
  if (!internode.info.locked && ImGui::Button("Lock")) {
    const auto chain_to_root = shoot_skeleton.GetChainToRoot(internode_handle);
    for (const auto& handle : chain_to_root) {
      shoot_skeleton.RefNode(handle).info.locked = true;
    }
    need_update = true;
  }
  if (ImGui::TreeNode("Internode info")) {
    ImGui::Checkbox("Is max child", &internode.data.max_child);
    ImGui::Text("Thickness: %.3f", internode.info.thickness);
    ImGui::Text("Length: %.3f", internode.info.length);
    ImGui::InputFloat3("Position", &internode.info.global_position.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto global_rotation_angle = glm::eulerAngles(internode.info.global_rotation);
    ImGui::InputFloat3("Global rotation", &global_rotation_angle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto local_rotation_angle = glm::eulerAngles(internode.data.desired_local_rotation);
    ImGui::InputFloat3("Local rotation", &local_rotation_angle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto& internode_data = internode.data;
    ImGui::InputFloat("Start Age", &internode_data.start_age, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Distance to end", &internode.info.end_distance, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Descendent biomass", &internode_data.descendant_total_biomass, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Biomass", &internode_data.biomass, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);

    ImGui::InputFloat("Root distance", &internode.info.root_distance, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);

    ImGui::InputFloat("Light Intensity", &internode_data.light_intake, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat3("Light direction", &internode_data.light_direction.x, "%.3f", ImGuiInputTextFlags_ReadOnly);

    ImGui::InputFloat("Growth rate control", &internode_data.growth_potential, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Desired growth rate", &internode_data.desired_growth_rate, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Growth rate", &internode_data.growth_rate, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Sagging Stress", &internode_data.sagging_stress, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    if (ImGui::DragFloat("Sagging", &internode_data.sagging)) {
      changed = true;
    }
    if (ImGui::DragFloat("Extra mass", &internode_data.extra_mass)) {
      changed = true;
    }

    if (ImGui::TreeNodeEx("Buds")) {
      int index = 1;
      for (const auto& bud : internode_data.buds) {
        if (ImGui::TreeNode(("Bud " + std::to_string(index)).c_str())) {
          switch (bud.type) {
            case BudType::Apical:
              ImGui::Text("Apical");
              break;
            case BudType::Lateral:
              ImGui::Text("Lateral");
              break;
          }
          switch (bud.status) {
            case OrganStatus::Flushed:
              ImGui::Text("Flushed");
              break;

            case OrganStatus::Dormant:
              ImGui::Text("Dormant");
              break;
          }

          auto bud_rotation_angle = glm::eulerAngles(bud.local_rotation);
          ImGui::InputFloat3("Rotation", &bud_rotation_angle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
          /*
          ImGui::InputFloat("Base resource requirement", (float *) &bud.m_maintenanceVigorRequirementWeight, 1, 100,
                                                  "%.3f", ImGuiInputTextFlags_ReadOnly);
                                                  */
          ImGui::TreePop();
        }
        index++;
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Flow info")) {
    const auto& flow = shoot_skeleton.PeekFlow(internode.GetFlowHandle());
    ImGui::Text("Child flow size: %d", flow.PeekChildHandles().size());
    ImGui::Text("Internode size: %d", flow.PeekNodeHandles().size());
    if (ImGui::TreeNode("Internodes")) {
      int i = 0;
      for (const auto& chained_internode_handle : flow.PeekNodeHandles()) {
        ImGui::Text("No.%d: Handle: %d", i, chained_internode_handle);
        i++;
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  return changed;
}

void TreeVisualizer::PeekInternode(const ShootSkeleton& shoot_skeleton,
                                   const SkeletonNodeHandle internode_handle) const {
  const auto& internode = shoot_skeleton.PeekNode(internode_handle);
  if (ImGui::TreeNode("Internode info")) {
    ImGui::Checkbox("Is max child", (bool*)&internode.data.max_child);
    ImGui::Text("Thickness: %.3f", internode.info.thickness);
    ImGui::Text("Length: %.3f", internode.info.length);
    ImGui::InputFloat3("Position", (float*)&internode.info.global_position.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto global_rotation_angle = glm::eulerAngles(internode.info.global_rotation);
    ImGui::InputFloat3("Global rotation", (float*)&global_rotation_angle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto local_rotation_angle = glm::eulerAngles(internode.data.desired_local_rotation);
    ImGui::InputFloat3("Local rotation", (float*)&local_rotation_angle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    auto& internode_data = internode.data;
    ImGui::InputInt("Start Age", (int*)&internode_data.start_age, 1, 100, ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Sagging", (float*)&internode_data.sagging, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Distance to end", (float*)&internode.info.end_distance, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Descendent biomass", (float*)&internode_data.descendant_total_biomass, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Biomass", (float*)&internode_data.biomass, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Root distance", (float*)&internode.info.root_distance, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat3("Light dir", (float*)&internode_data.light_direction.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Growth Potential", (float*)&internode_data.light_intake, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);

    if (ImGui::TreeNodeEx("Buds")) {
      int index = 1;
      for (const auto& bud : internode_data.buds) {
        if (ImGui::TreeNode(("Bud " + std::to_string(index)).c_str())) {
          switch (bud.type) {
            case BudType::Apical:
              ImGui::Text("Apical");
              break;
            case BudType::Lateral:
              ImGui::Text("Lateral");
              break;
          }
          switch (bud.status) {
            case OrganStatus::Flushed:
              ImGui::Text("Flushed");
              break;

            case OrganStatus::Dormant:
              ImGui::Text("Dormant");
              break;
          }

          auto bud_rotation_angle = glm::eulerAngles(bud.local_rotation);
          ImGui::InputFloat3("Rotation", &bud_rotation_angle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
          /*
          ImGui::InputFloat("Base resource requirement", (float *) &bud.m_maintenanceVigorRequirementWeight, 1, 100,
                                                  "%.3f", ImGuiInputTextFlags_ReadOnly);
                                                  */
          ImGui::TreePop();
        }
        index++;
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Stem info", ImGuiTreeNodeFlags_DefaultOpen)) {
    const auto& flow = shoot_skeleton.PeekFlow(internode.GetFlowHandle());
    ImGui::Text("Child stem size: %d", flow.PeekChildHandles().size());
    ImGui::Text("Internode size: %d", flow.PeekNodeHandles().size());
    if (ImGui::TreeNode("Internodes")) {
      int i = 0;
      for (const auto& chained_internode_handle : flow.PeekNodeHandles()) {
        ImGui::Text("No.%d: Handle: %d", i, chained_internode_handle);
        i++;
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
}

void TreeVisualizer::Reset(const ShootModel& tree_model) {
  selected_internode_handle = -1;
  selected_internode_hierarchy_list.clear();
  checkpoint_iteration = tree_model.CurrentIteration();
  internode_matrices_->SetParticleInfos({});
  need_update = true;
}

void TreeVisualizer::Clear() {
  selected_internode_handle = -1;
  selected_internode_hierarchy_list.clear();
  checkpoint_iteration = 0;
  internode_matrices_->SetParticleInfos({});
}

bool TreeVisualizer::Initialized() const {
  return initialized_;
}

void TreeVisualizer::Initialize() {
  tree_visualizer_color_settings = {};
  internode_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
}
