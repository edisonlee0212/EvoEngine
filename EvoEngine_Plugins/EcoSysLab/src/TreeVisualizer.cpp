//
// Created by lllll on 11/20/2022.
//

#include "TreeVisualizer.hpp"
#include "Application.hpp"
#include "EcoSysLabLayer.hpp"
#include "ProfileConstraints.hpp"
#include "Utilities.hpp"
using namespace eco_sys_lab_plugin;

void ShootVisualizer::PeekNodeInspectionGui(const ShootSkeleton& skeleton, const SkeletonNodeHandle node_handle,
                                            const unsigned& hierarchy_level) {
  const int index = selected_node_hierarchy_list.size() - hierarchy_level - 1;
  if (!selected_node_hierarchy_list.empty() && index >= 0 && index < selected_node_hierarchy_list.size() &&
      selected_node_hierarchy_list[index] == node_handle) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      ("Handle: " + std::to_string(node_handle)).c_str(),
      ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
          (selected_node_handle == node_handle ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
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

bool ShootVisualizer::DrawInternodeInspectionGui(ShootModel& tree_model, const SkeletonNodeHandle internode_handle,
                                                 bool& deleted, const unsigned& hierarchy_level) {
  auto& treeSkeleton = tree_model.RefShootSkeleton();
  const int index = selected_node_hierarchy_list.size() - hierarchy_level - 1;
  if (!selected_node_hierarchy_list.empty() && index >= 0 && index < selected_node_hierarchy_list.size() &&
      selected_node_hierarchy_list[index] == internode_handle) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      ("Handle: " + std::to_string(internode_handle)).c_str(),
      ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
          (selected_node_handle == internode_handle ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
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
        tree_model.RefShootSkeleton().RemoveNodes({child});
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
  selected_node_handle = -1;
}
bool ShootVisualizer::OnInspect(ShootModel& tree_model) {
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
      selected_node_handle = -1;
      selected_node_hierarchy_list.clear();
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
    if (selected_node_handle >= 0) {
      if (checkpoint_iteration == tree_model.CurrentIteration()) {
        InspectInternode(tree_model.RefShootSkeleton(), selected_node_handle);
      } else {
        PeekInternode(tree_model.PeekShootSkeleton(checkpoint_iteration), selected_node_handle);
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
        selected_node_hierarchy_list.clear();
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  return updated;
}

void ShootVisualizer::Visualize(const ShootModel& tree_model, const GlobalTransform& global_transform) {
  const auto& tree_skeleton = tree_model.PeekShootSkeleton(checkpoint_iteration);
  if (visualization) {
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
    if (need_update) {
      SyncMatrices(tree_skeleton, node_matrices_);
      need_update = false;
    }
    GizmoSettings gizmo_settings;
    gizmo_settings.draw_settings.blending = true;
    gizmo_settings.depth_test = true;
    gizmo_settings.depth_write = true;
    if (!node_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cylinder,
                                                  eco_sys_lab_layer->visualization_camera_, node_matrices_,
                                                  global_transform.value, 1.0f, gizmo_settings);
      if (selected_node_handle != -1) {
        const auto& node = tree_skeleton.PeekNode(selected_node_handle);
        auto rotation = node.info.global_rotation;
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        const glm::vec3 selected_center =
            node.info.global_position + node.info.length * selected_node_length_factor * node.info.GetGlobalDirection();
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

void ShootVisualizer::Visualize(StrandModel& strand_model) {
  if (visualization) {
    auto& skeleton = strand_model.strand_model_skeleton;
    static bool show_grid = false;
    if (profile_gui) {
      const std::string tag = "Profile";
      if (ImGui::Begin(tag.c_str())) {
        if (selected_node_handle != -1 && selected_node_handle < skeleton.RefRawNodes().size()) {
          auto& node = skeleton.RefNode(selected_node_handle);
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
              parent_node_to_root_chain.emplace_back(selected_node_handle);
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

bool ShootVisualizer::InspectInternode(ShootSkeleton& shoot_skeleton, SkeletonNodeHandle internode_handle) {
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
    ImGui::Checkbox("Is max child", &internode.info.max_child);
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
    ImGui::InputFloat("Descendent biomass_factor", &internode_data.descendant_total_biomass_factor, 1, 100, "%.3f",
                      ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Biomass", &internode_data.biomass_factor, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);

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

void ShootVisualizer::PeekInternode(const ShootSkeleton& shoot_skeleton,
                                    const SkeletonNodeHandle internode_handle) const {
  const auto& internode = shoot_skeleton.PeekNode(internode_handle);
  if (ImGui::TreeNode("Internode info")) {
    ImGui::Checkbox("Is max child", (bool*)&internode.info.max_child);
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
    ImGui::InputFloat("Descendent biomass_factor", (float*)&internode_data.descendant_total_biomass_factor, 1, 100,
                      "%.3f", ImGuiInputTextFlags_ReadOnly);
    ImGui::InputFloat("Biomass", (float*)&internode_data.biomass_factor, 1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
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

void ShootVisualizer::Reset(const ShootModel& tree_model) {
  selected_node_handle = -1;
  selected_node_hierarchy_list.clear();
  checkpoint_iteration = tree_model.CurrentIteration();
  node_matrices_->SetParticleInfos({});
  need_update = true;
}

void TreeVisualizer::Clear() {
  selected_node_handle = -1;
  selected_node_hierarchy_list.clear();
  checkpoint_iteration = 0;
  node_matrices_->SetParticleInfos({});
}

bool TreeVisualizer::Initialized() const {
  return initialized_;
}

void TreeVisualizer::Initialize() {
  tree_visualizer_color_settings = {};
  node_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
}
