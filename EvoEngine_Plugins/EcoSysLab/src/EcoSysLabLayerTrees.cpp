//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "AdvancedShootDescriptor.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "ForestDescriptor.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Times.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"
using namespace eco_sys_lab_plugin;
PrivateComponentRegistration<Tree> tree_registry("Tree");
AssetRegistration<BasicBarkDescriptor> bark_descriptor_registry("BasicBarkDescriptor", {".bark"});
AssetRegistration<ForestDescriptor> forest_d_registry("ForestDescriptor", {".forest"});
AssetRegistration<TreeDescriptor> tree_d_registry("TreeDescriptor", {".tree"});
AssetRegistration<BasicPruningDescriptor> pruning_d_registry("BasicPruningDescriptor", {".pruning"});
AssetRegistration<BasicShootDescriptor> shoot_d_registry("BasicShootDescriptor", {".shoot"});
AssetRegistration<BasicRootDescriptor> root_d_registry("BasicRootDescriptor", {".root"});
AssetRegistration<BasicFineRootDescriptor> fine_root_d_registry("BasicFineRootDescriptor", {".froot"});
AssetRegistration<BasicReproductionModuleDescriptor> fruit_d_registry("BasicReproductionModuleDescriptor", {".repro"});
AssetRegistration<BasicFoliageDescriptor> foliage_d_registry("BasicFoliageDescriptor", {".foliage"});
AssetRegistration<AdvancedShootDescriptor> a_shoot_d_registry("AdvancedShootDescriptor", {".ashoot"});
AssetRegistration<ModulusGraph> modulus_graph_registry("ModulusGraph", {".evemodulus"});
AssetRegistration<StrengthGraph> strength_graph_registry("StrengthGraph", {".evestrength"});
AssetRegistration<BiologicalPropertiesGraph> biological_properties_graph_registry("TrunkGraph", {".evetrunk"});

void EcoSysLabLayer::TreeVisualization(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto scene = GetScene();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  const auto branch_strands = shoot_stem_strands_.Get<Strands>();
  if (tree_entities && !tree_entities->empty()) {
    // Tree selection
    if (shoot_versions_.size() != tree_entities->size()) {
      shoot_versions_.clear();
      for (int i = 0; i < tree_entities->size(); i++) {
        shoot_versions_.emplace_back(-1);
      }
      need_full_flow_update = true;
    }
    for (int i = 0; i < tree_entities->size(); i++) {
      auto tree_entity = tree_entities->at(i);
      auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      if (auto& tree_model = tree->shoot_model; shoot_versions_[i] != tree_model.RefShootSkeleton().GetVersion()) {
        shoot_versions_[i] = tree_model.RefShootSkeleton().GetVersion();
        need_full_flow_update = true;
      }
    }

    bool flow_updated = false;
    if (need_full_flow_update) {
      UpdateFlows(tree_entities, branch_strands);
      UpdateGroundFruitAndLeaves();
      need_full_flow_update = false;
      flow_updated = true;
    }
    if (need_flow_update_for_selection_) {
      UpdateFlows(tree_entities, branch_strands);
      need_flow_update_for_selection_ = false;
      flow_updated = true;
    }
    if (flow_updated) {
      if (const auto climate_candidate = FindClimate(); !climate_candidate.expired()) {
        const auto climate = climate_candidate.lock();
        const auto& voxel_grid = climate->climate_model.environment_grid.voxel_grid;
        const auto num_voxels = voxel_grid.GetVoxelCount();
        {
          std::vector<ParticleInfo> particle_infos;
          particle_infos.resize(num_voxels);

          Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
            const auto coordinate = voxel_grid.GetCoordinate(i);
            particle_infos[i].instance_matrix.value =
                glm::translate(voxel_grid.GetPosition(coordinate) +
                               glm::linearRand(-glm::vec3(0.5f * voxel_grid.GetVoxelSize()),
                                               glm::vec3(0.5f * voxel_grid.GetVoxelSize()))) *
                glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(0.25f * voxel_grid.GetVoxelSize()));
            particle_infos[i].instance_color = glm::vec4(
                1.f, 1.f, 1.f, 1.f - glm::clamp(voxel_grid.Peek(static_cast<int>(i)).light_intensity, 0.0f, 1.0f));
          });
          shadow_grid_particle_info_list_->SetParticleInfos(particle_infos);
        }
        {
          std::vector<ParticleInfo> particle_infos;
          particle_infos.resize(num_voxels);

          Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
            const auto coordinate = voxel_grid.GetCoordinate(i);
            const auto& voxel = voxel_grid.Peek(coordinate);
            const auto direction = voxel.light_direction;
            auto rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
            rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
            const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
            const auto voxel_size = voxel_grid.GetVoxelSize();
            particle_infos[i].instance_matrix.value =
                glm::translate(voxel_grid.GetPosition(coordinate) +
                               glm::linearRand(-glm::vec3(0.5f * voxel_grid.GetVoxelSize()),
                                               glm::vec3(0.5f * voxel_grid.GetVoxelSize()))) *
                rotation_transform * glm::scale(glm::vec3(0.05f * voxel_size, voxel_size * 0.5f, 0.05f * voxel_size));
            if (voxel_grid.Peek(static_cast<int>(i)).light_intensity == 0.0f)
              particle_infos[i].instance_color = glm::vec4(0.0f);
            else
              particle_infos[i].instance_color = glm::vec4(
                  1.f, 1.f, 1.f, 1.f - glm::clamp(voxel_grid.Peek(static_cast<int>(i)).light_intensity, 0.0f, 1.0f));
          });
          lighting_grid_particle_info_list_->SetParticleInfos(particle_infos);
        }
      }
    }
    GizmoSettings gizmo_settings;
    gizmo_settings.draw_settings.blending = true;
    gizmo_settings.draw_settings.blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
    gizmo_settings.draw_settings.blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;
    gizmo_settings.depth_test = true;
    gizmo_settings.depth_write = true;
    if (editor_layer && scene->IsEntityValid(selected_tree)) {
      const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
      auto& shoot_model = tree->shoot_model;
      auto& root_model = tree->root_model;
      auto& shoot_visualizer = tree->shoot_visualizer;
      auto& root_visualizer = tree->root_visualizer;
      const auto global_transform = scene->GetDataComponent<GlobalTransform>(selected_tree);
      if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Release &&
          shoot_visualizer.checkpoint_iteration == shoot_model.CurrentIteration()) {
        static bool may_need_geometry_generation = false;
        static std::vector<glm::vec2> mouse_positions{};
        auto& tree_skeleton = shoot_model.PeekShootSkeleton(tree->shoot_visualizer.checkpoint_iteration);
        switch (static_cast<TreeOperatorMode>(tree_operator_mode)) {
          case TreeOperatorMode::Select: {
            if (visualization_camera_window_focused_) {
              if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                if (shoot_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                      tree_skeleton, global_transform)) {
                  shoot_visualizer.need_update = true;
                }
              } else if (EditorLayer::GetKey(GLFW_KEY_R) == Input::KeyActionType::Press) {
                if (shoot_visualizer.selected_node_handle > 0) {
                  shoot_model.Step();
                  auto& pruning_internode =
                      shoot_model.RefShootSkeleton().RefNode(shoot_visualizer.selected_node_handle);
                  shoot_model.RefShootSkeleton().RemoveNodes(pruning_internode.PeekChildHandles());
                  pruning_internode.data.internode_length *= shoot_visualizer.selected_node_length_factor;
                  shoot_model.CalculateTransform(tree->shoot_growth_controller_, true);
                  shoot_visualizer.selected_node_length_factor = 1.0f;
                  pruning_internode.data.buds.clear();
                  shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                  shoot_visualizer.need_update = true;
                  if (auto_generate_mesh_after_editing_) {
                    tree->GenerateGeometryEntities(mesh_generator_settings, -1);
                  }
                  if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
                    if (auto_generate_strands_after_editing_) {
                      tree->InitializeStrandRenderer();
                    }
                    if (auto_generate_strand_mesh_after_editing_) {
                      tree->InitializeStrandModelMeshRenderer(strand_mesh_generator_settings);
                    }
                  }
                }
              } else if (EditorLayer::GetKey(GLFW_KEY_T) == Input::KeyActionType::Press) {
                if (shoot_visualizer.selected_node_handle > 0) {
                  shoot_model.Step();
                  shoot_model.RefShootSkeleton().RemoveNodes({shoot_visualizer.selected_node_handle});
                  shoot_visualizer.selected_node_handle = -1;
                  shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                  shoot_visualizer.need_update = true;
                  if (auto_generate_mesh_after_editing_) {
                    tree->GenerateGeometryEntities(mesh_generator_settings, -1);
                  }
                  if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
                    if (auto_generate_strands_after_editing_) {
                      tree->InitializeStrandRenderer();
                    }
                    if (auto_generate_strand_mesh_after_editing_) {
                      tree->InitializeStrandModelMeshRenderer(strand_mesh_generator_settings);
                    }
                  }
                }
              } else if (EditorLayer::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
                shoot_visualizer.SetSelectedNode(tree_skeleton, -1);
              }
            }
          } break;
          case TreeOperatorMode::Rotate: {
            if (shoot_visualizer.selected_node_handle > 0) {
              if (visualization_camera_window_focused_) {
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
                if (ImGui::Begin("Plant Visual")) {
                  if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
                    ImGuizmo::SetOrthographic(false);
                    ImGuizmo::SetDrawlist();
                    ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y,
                                      visualization_camera_resolution_x, visualization_camera_resolution_y);
                    glm::mat4 camera_view = glm::inverse(glm::translate(editor_layer->GetSceneCameraPosition()) *
                                                         glm::mat4_cast(editor_layer->GetSceneCameraRotation()));
                    glm::mat4 camera_projection = visualization_camera_->GetProjection();
                    constexpr auto op = ImGuizmo::OPERATION::ROTATE;
                    auto& current_skeleton = tree->shoot_model.RefShootSkeleton();
                    auto& internode = current_skeleton.RefNode(shoot_visualizer.selected_node_handle);

                    auto transform = glm::translate(internode.info.global_position) *
                                     glm::mat4_cast(internode.data.desired_global_rotation) *
                                     glm::scale(glm::vec3(1.0f));
                    const auto tree_global_transform = scene->GetDataComponent<GlobalTransform>(selected_tree);
                    auto internode_global_transform = tree_global_transform.value * transform;
                    ImGuizmo::Manipulate(glm::value_ptr(camera_view), glm::value_ptr(camera_projection), op,
                                         ImGuizmo::LOCAL, glm::value_ptr(internode_global_transform));
                    static bool last_gizmos_used = false;
                    if (ImGuizmo::IsUsing()) {
                      if (!last_gizmos_used) {
                        shoot_model.Step();
                        shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                      }
                      shoot_visualizer.need_update = true;
                      Transform new_internode_transform{};
                      new_internode_transform.value =
                          glm::inverse(tree_global_transform.value) * internode_global_transform;
                      auto scale_holder = glm::vec3(1.0f);
                      new_internode_transform.Decompose(internode.info.global_position,
                                                        internode.data.desired_global_rotation, scale_holder);
                      if (auto parent_handle = internode.GetParentHandle(); parent_handle != -1) {
                        internode.data.desired_local_rotation =
                            glm::inverse(current_skeleton.PeekNode(parent_handle).data.desired_global_rotation) *
                            internode.data.desired_global_rotation;
                      }
                      shoot_model.CalculateTransform(tree->shoot_growth_controller_, true);
                      last_gizmos_used = true;
                    } else if (last_gizmos_used) {
                      shoot_model.CalculateTransform(tree->shoot_growth_controller_, true);
                      may_need_geometry_generation = true;
                      last_gizmos_used = false;
                      shoot_visualizer.need_update = true;
                      shoot_model.RefShootSkeleton().CalculateRegulatedGlobalRotation();
                    }
                  }
                  ImGui::EndChild();
                }
                ImGui::End();
                ImGui::PopStyleVar();
              }
            }
            if (visualization_camera_window_focused_) {
              if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                if (shoot_visualizer.selected_node_handle <= 0) {
                  if (shoot_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                        tree_skeleton, global_transform)) {
                    shoot_visualizer.need_update = true;
                  }
                }
              } else if (EditorLayer::GetKey(GLFW_KEY_T) == Input::KeyActionType::Press) {
                if (shoot_visualizer.selected_node_handle > 0) {
                  shoot_model.Step();
                  shoot_model.RefShootSkeleton().RemoveNodes({shoot_visualizer.selected_node_handle});
                  shoot_visualizer.selected_node_handle = -1;
                  shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                  shoot_visualizer.need_update = true;
                  if (auto_generate_mesh_after_editing_) {
                    tree->GenerateGeometryEntities(mesh_generator_settings, -1);
                  }
                  if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
                    if (auto_generate_strands_after_editing_) {
                      tree->InitializeStrandRenderer();
                    }
                    if (auto_generate_strand_mesh_after_editing_) {
                      tree->InitializeStrandModelMeshRenderer(strand_mesh_generator_settings);
                    }
                  }
                }
              } else if (EditorLayer::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
                shoot_visualizer.SetSelectedNode(tree_skeleton, -1);
              }
            }
          } break;
          case TreeOperatorMode::Prune: {
            if (visualization_camera_window_focused_) {
              if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                mouse_positions.clear();
                glm::vec2 mouse_position = visualization_camera_mouse_position;
                const float half_x = visualization_camera_->GetSize().x / 2.0f;
                const float half_y = visualization_camera_->GetSize().y / 2.0f;
                mouse_position = {-1.0f * (mouse_position.x - half_x) / half_x,
                                  -1.0f * (mouse_position.y - half_y) / half_y};
                if (mouse_position.x > -1.0f && mouse_position.x < 1.0f && mouse_position.y > -1.0f &&
                    mouse_position.y < 1.0f) {
                  mouse_positions.emplace_back(mouse_position);
                }
              } else if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Hold) {
                glm::vec2 mouse_position = visualization_camera_mouse_position;
                const float half_x = visualization_camera_->GetSize().x / 2.0f;
                const float half_y = visualization_camera_->GetSize().y / 2.0f;
                mouse_position = {-1.0f * (mouse_position.x - half_x) / half_x,
                                  -1.0f * (mouse_position.y - half_y) / half_y};
                if (mouse_position.x > -1.0f && mouse_position.x < 1.0f && mouse_position.y > -1.0f &&
                    mouse_position.y < 1.0f && (!mouse_positions.empty() && mouse_position != mouse_positions.back())) {
                  mouse_positions.emplace_back(mouse_position);
                }
              } else if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Release) {
                // Once released, check if empty.
                if (!mouse_positions.empty()) {
                  shoot_model.Step();
                  auto& skeleton = shoot_model.RefShootSkeleton();
                  std::vector<SkeletonNodeHandle> pruning_node_handles;
                  const auto camera_rotation = editor_layer->GetSceneCameraRotation();
                  const auto camera_position = editor_layer->GetSceneCameraPosition();
                  const glm::vec3 camera_front = camera_rotation * glm::vec3(0, 0, -1);
                  const glm::vec3 camera_up = camera_rotation * glm::vec3(0, 1, 0);
                  const glm::mat4 projection_view =
                      visualization_camera_->GetProjection() *
                      glm::lookAt(camera_position, camera_position + camera_front, camera_up);
                  if (shoot_visualizer.ScreenCurveSelection(
                          [&](const SkeletonNodeHandle node_handle) {
                            pruning_node_handles.emplace_back(node_handle);
                          },
                          mouse_positions, skeleton, global_transform, projection_view)) {
                    shoot_model.RefShootSkeleton().RemoveNodes(pruning_node_handles);
                    shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                    shoot_visualizer.need_update = true;
                    may_need_geometry_generation = true;
                  } else {
                    shoot_model.Pop();
                  }
                  mouse_positions.clear();
                }
              }
            }
          } break;
          case TreeOperatorMode::Invigorate: {
            if (visualization_camera_window_focused_) {
              static bool last_frame_invigorate = false;
              if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                if (shoot_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                      tree_skeleton, global_transform)) {
                  if (!tree->enable_history)
                    shoot_model.Step();
                  shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                  shoot_visualizer.need_update = true;
                }
              } else if (shoot_visualizer.selected_node_handle >= 0 &&
                         EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Hold) {
                const auto climate_candidate = FindClimate();
                if (!climate_candidate.expired()) {
                  climate_candidate.lock()->PrepareForGrowth();
                  if (tree->TryGrow(simulation_settings, shoot_visualizer.selected_node_handle, false)) {
                    shoot_visualizer.need_update = true;
                    may_need_geometry_generation = true;
                  }
                }
                last_frame_invigorate = true;
              } else if (last_frame_invigorate &&
                         EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Release) {
                shoot_visualizer.SetSelectedNode(tree_skeleton, -1);
                last_frame_invigorate = false;
                shoot_visualizer.need_update = true;
              }
            }
          } break;
          case TreeOperatorMode::Reduce: {
            if (visualization_camera_window_focused_) {
              static bool last_frame_reduce = false;
              static float target_age = 0.0f;
              if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                if (shoot_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                      tree_skeleton, global_transform)) {
                  if (!tree->enable_history)
                    shoot_model.Step();
                  shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                  shoot_visualizer.need_update = true;
                  if (shoot_visualizer.selected_node_handle >= 0) {
                    target_age = tree->shoot_model.GetSubTreeMaxAge(shoot_visualizer.selected_node_handle);
                  }
                }
              } else if (shoot_visualizer.selected_node_handle >= 0 &&
                         EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Hold) {
                if (tree->shoot_model.Reduce(tree->shoot_growth_controller_, shoot_visualizer.selected_node_handle,
                                             target_age)) {
                  shoot_visualizer.need_update = true;
                  may_need_geometry_generation = true;
                }
                target_age -= tree_reduce_rate;
                last_frame_reduce = true;
              } else if (last_frame_reduce &&
                         EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Release) {
                shoot_visualizer.SetSelectedNode(tree_skeleton, -1);
                last_frame_reduce = false;
                shoot_visualizer.need_update = true;
              }
            }
          } break;
        }

        if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Release &&
            may_need_geometry_generation) {
          if (auto_generate_mesh_after_editing_) {
            tree->GenerateGeometryEntities(mesh_generator_settings, -1);
          }
          if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
            tree->BuildStrandModel();
            if (auto_generate_strands_after_editing_) {
              auto strands = tree->GenerateStrands();
              tree->InitializeStrandRenderer(strands);
            }
            if (auto_generate_strand_mesh_after_editing_) {
              tree->InitializeStrandModelMeshRenderer(strand_mesh_generator_settings);
            }
          }
        } else if (shoot_visualizer.need_update && auto_generate_skeletal_graph_every_frame_) {
          tree->GenerateSkeletalGraph(skeletal_graph_settings, shoot_visualizer.selected_node_handle,
                                      Resources::Primitives::sphere, Resources::Primitives::cube);
        }
        may_need_geometry_generation = false;
      }
      root_visualizer.Visualize(root_model, global_transform);
      shoot_visualizer.Visualize(shoot_model, global_transform);
    }
    if (tree_visualization_settings_.show_shadow_grid) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cube, visualization_camera_,
                                                  shadow_grid_particle_info_list_, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);
    }
    if (tree_visualization_settings_.show_lighting_grid) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cube, visualization_camera_,
                                                  lighting_grid_particle_info_list_, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);
    }
    if (tree_visualization_settings_.display_flowers && !flower_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cone, visualization_camera_, flower_matrices_,
                                                  glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    if (tree_visualization_settings_.display_fruits && !fruit_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cube, visualization_camera_, fruit_matrices_,
                                                  glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
    if (tree_visualization_settings_.display_foliage && !foliage_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::quad, visualization_camera_, foliage_matrices_,
                                                  glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;
    if (tree_visualization_settings_.display_ground_leaves && !ground_leaf_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::quad, visualization_camera_,
                                                  ground_leaf_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;
    if (tree_visualization_settings_.display_ground_fruits && !ground_fruit_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cube, visualization_camera_,
                                                  ground_fruit_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    if (tree_visualization_settings_.display_ground_flowers &&
        !ground_flower_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cone, visualization_camera_,
                                                  ground_flower_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }

    if (tree_visualization_settings_.display_ground_leaves && !ground_leaf_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::quad, visualization_camera_,
                                                  ground_leaf_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }

    if (tree_visualization_settings_.display_bounding_box && !bounding_box_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cube, visualization_camera_,
                                                  bounding_box_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }

    if (tree_visualization_settings_.display_shoot_stem && !shoot_stem_points_.empty()) {
      gizmo_settings.color_mode = GizmoSettings::ColorMode::Default;
      editor_layer->DrawGizmoStrands(branch_strands, visualization_camera_, glm::vec4(1.0f, 1.0f, 1.0f, 0.75f),
                                     glm::mat4(1.0f), 1, gizmo_settings);
    }

    gizmo_settings.color_mode = GizmoSettings::ColorMode::Default;
  }
}

void EcoSysLabLayer::ResetAllTrees(const std::vector<Entity>* tree_entities) {
  const auto scene = Application::GetActiveScene();
  simulated_time_ = 0;
  auto_time_grow_ = false;
  auto_grow_target_time_ = 0.0f;
  on_auto_grow_finished_ = {};
  if (tree_entities) {
    for (const auto& i : *tree_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
      tree->Reset();
    }
  }

  need_full_flow_update = true;
  simulation_stats = {};

  shoot_stem_segments_.clear();
  shoot_stem_points_.clear();

  shoot_stem_strands_ = AssetManager::CreateTemporaryAsset<Strands>();

  bounding_box_matrices_->SetParticleInfos({});
  foliage_matrices_->SetParticleInfos({});
  flower_matrices_->SetParticleInfos({});
  fruit_matrices_->SetParticleInfos({});

  const auto climate_candidate = FindClimate();
  if (!climate_candidate.expired()) {
    const auto climate = climate_candidate.lock();
    climate->climate_model.environment_grid = {};
  }
}

bool EcoSysLabLayer::Simulate(const SimulationSettings& target_simulation_settings,
                              SimulationStats& target_simulation_stats) {
  const auto scene = GetScene();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  simulated_time_ += target_simulation_settings.delta_time;
  bool tree_grown = false;
  if (tree_entities && !tree_entities->empty()) {
    float time = Times::Now();

    std::shared_ptr<Climate> climate;
    std::shared_ptr<Soil> soil;
    if (const auto climate_candidate = FindClimate(); !climate_candidate.expired())
      climate = climate_candidate.lock();
    if (const auto soil_candidate = FindSoil(); !soil_candidate.expired())
      soil = soil_candidate.lock();
    if (!soil) {
      EVOENGINE_ERROR("Simulation Failed! No soil in scene!");
      return tree_grown;
    }
    if (!climate) {
      EVOENGINE_ERROR("Simulation Failed! No climate in scene!");
      return tree_grown;
    }
    climate->climate_model.time = simulated_time_;

    if (target_simulation_settings.soil_simulation) {
      soil->soil_model.Irrigation();
      soil->soil_model.Step();
    }
    for (const auto& tree_entity : *tree_entities) {
      auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->climate = climate;
      tree->soil = soil;
    }
    climate->PrepareForGrowth();
    std::vector<bool> grown_stat{};
    grown_stat.resize(Jobs::GetWorkerSize());
    Jobs::RunParallelFor(tree_entities->size(), [&](unsigned i, unsigned thread_index) {
      const auto tree_entity = tree_entities->at(i);
      if (!scene->IsEntityEnabled(tree_entity))
        return;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      if (!tree->IsEnabled())
        return;
      if (tree->start_time > simulated_time_)
        return;
      if (target_simulation_settings.max_node_count > 0 &&
          tree->shoot_model.RefShootSkeleton().PeekSortedNodeList().size() >= target_simulation_settings.max_node_count)
        return;
      if (target_simulation_settings.max_flow_count > 0 &&
          tree->shoot_model.RefShootSkeleton().PeekSortedFlowList().size() >= target_simulation_settings.max_flow_count)
        return;
      grown_stat[thread_index] = tree->TryGrow(target_simulation_settings, -1, true);
    });

    auto height_field = soil->soil_descriptor_ref.Get<SoilDescriptor>()->height_field.Get<HeightField>();
    for (const auto& tree_entity : *tree_entities) {
      if (!scene->IsEntityEnabled(tree_entity))
        continue;
      auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      auto tree_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
      if (!tree->IsEnabled())
        continue;
      // Collect fruit and leaves here.
      if (!target_simulation_settings.auto_clear_fruit_and_leaves) {
        for (const auto& flower : tree->shoot_model.RefShootSkeleton().data.dropped_flowers) {
          Flower new_flower;
          Transform flower_transform;
          flower_transform.value =
              glm::translate(flower.position) * glm::mat4_cast(flower.rotation) * glm::scale(flower.scale);
          new_flower.global_transform.value = tree_global_transform.value * flower_transform.value;

          auto position = new_flower.global_transform.GetPosition();
          const auto ground_height = height_field->GetValue({position.x, position.z});
          const auto height = position.y - ground_height;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = ground_height + 0.1f;
          new_flower.global_transform.SetPosition(position);

          new_flower.flower_maturity = flower.maturity;
          new_flower.flower_health = flower.health;
          flowers_.emplace_back(new_flower);
        }

        for (const auto& fruit : tree->shoot_model.RefShootSkeleton().data.dropped_fruits) {
          Fruit new_fruit;
          Transform fruit_transform;
          fruit_transform.value =
              glm::translate(fruit.position) * glm::mat4_cast(fruit.rotation) * glm::scale(fruit.scale);
          new_fruit.global_transform.value = tree_global_transform.value * fruit_transform.value;

          auto position = new_fruit.global_transform.GetPosition();
          const auto ground_height = height_field->GetValue({position.x, position.z});
          const auto height = position.y - ground_height;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = ground_height + 0.1f;
          new_fruit.global_transform.SetPosition(position);

          new_fruit.fruit_maturity = fruit.maturity;
          new_fruit.fruit_health = fruit.health;
          fruits_.emplace_back(new_fruit);
        }

        for (const auto& leaf : tree->shoot_model.RefShootSkeleton().data.dropped_leaves) {
          Leaf new_leaf;
          Transform leaf_transform;
          leaf_transform.value = glm::translate(leaf.position) * glm::mat4_cast(leaf.rotation) * glm::scale(leaf.scale);
          new_leaf.global_transform.value = tree_global_transform.value * leaf_transform.value;

          auto position = new_leaf.global_transform.GetPosition();
          const auto ground_height = height_field ? height_field->GetValue({position.x, position.z}) : 0.0f;
          const auto height = position.y - ground_height;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = ground_height + 0.1f;
          new_leaf.global_transform.SetPosition(position);

          new_leaf.leaf_maturity = leaf.maturity;
          new_leaf.leaf_health = leaf.health;
          leaves_.emplace_back(new_leaf);
        }
        tree->shoot_visualizer.need_update = true;
        tree->root_visualizer.need_update = true;
      }
      tree->shoot_model.RefShootSkeleton().data.dropped_fruits.clear();
      tree->shoot_model.RefShootSkeleton().data.dropped_flowers.clear();
      tree->shoot_model.RefShootSkeleton().data.dropped_leaves.clear();
    }
    target_simulation_stats.last_used_time = Times::Now() - time;
    target_simulation_stats.total_time += target_simulation_stats.last_used_time;

    for (auto&& i : grown_stat) {
      if (i) {
        tree_grown = true;
      }
    }
    if (tree_grown) {
      need_full_flow_update = true;

      int total_internode_size = 0;
      int total_flow_size = 0;
      int total_root_node_size = 0;
      int total_root_flow_size = 0;
      int total_leaf_size = 0;
      int total_fruit_size = 0;
      for (auto tree_entity : *tree_entities) {
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        auto& tree_model = tree->shoot_model;
        total_internode_size += tree_model.RefShootSkeleton().PeekSortedNodeList().size();
        total_flow_size += tree_model.RefShootSkeleton().PeekSortedFlowList().size();
        total_leaf_size += tree_model.GetLeafCount();
        total_fruit_size += tree_model.GetFruitCount();
      }
      target_simulation_stats.internode_size = total_internode_size;
      target_simulation_stats.shoot_stem_size = total_flow_size;
      target_simulation_stats.root_node_size = total_root_node_size;
      target_simulation_stats.root_stem_size = total_root_flow_size;
      target_simulation_stats.leaf_size = total_leaf_size;
      target_simulation_stats.fruit_size = total_fruit_size;
    }
  }
  if (simulation_settings.auto_clear_fruit_and_leaves) {
    ClearGroundFruitAndLeaf();
  }
  if (scene->IsEntityValid(selected_tree)) {
    auto tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
    tree->shoot_visualizer.checkpoint_iteration = tree->shoot_model.CurrentIteration();
    tree->shoot_visualizer.need_update = true;
    tree->root_visualizer.checkpoint_iteration = tree->root_model.CurrentIteration();
    tree->root_visualizer.need_update = true;

    if (auto_generate_skeletal_graph_every_frame_) {
      tree->GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::Primitives::sphere,
                                  Resources::Primitives::cube);
    }
  }
  return tree_grown;
}

bool EcoSysLabLayer::Simulate() {
  return Simulate(simulation_settings, simulation_stats);
}

void EcoSysLabLayer::GenerateMeshes(const TreeMeshGeneratorSettings& target_mesh_generator_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->GenerateGeometryEntities(target_mesh_generator_settings);
    }
  }
}

void EcoSysLabLayer::GenerateSkeletalGraphs(const SkeletalGraphSettings& target_skeletal_graph_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::Primitives::sphere,
                                    Resources::Primitives::cube);
    }
  }
}

void EcoSysLabLayer::ClearMeshes() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearGeometryEntities();
    }
  }
}

void EcoSysLabLayer::ClearSkeletalGraphs() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearSkeletalGraph();
    }
  }
}

bool EcoSysLabLayer::TreeVisualizationSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  ImGui::Checkbox("Display shoot stem", &display_shoot_stem);

  ImGui::Checkbox("Display foliage", &display_foliage);
  ImGui::Checkbox("Display flowers", &display_flowers);
  ImGui::Checkbox("Display fruits", &display_fruits);

  ImGui::Checkbox("Display ground leaves", &display_ground_leaves);
  ImGui::Checkbox("Display ground flowers", &display_ground_flowers);
  ImGui::Checkbox("Display ground fruit", &display_ground_fruits);

  ImGui::Checkbox("Display Bounding Box", &display_bounding_box);
  ImGui::Checkbox("Show Shadow Grid", &show_shadow_grid);
  ImGui::Checkbox("Show Lighting Direction Grid", &show_lighting_grid);

  return changed;
}

void EcoSysLabLayer::ExportAllTrees(const std::filesystem::path& path) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    if (path.extension() == ".obj") {
      std::ofstream of;
      of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
      if (of.is_open()) {
        std::string start = "#Forest OBJ exporter, by Bosheng Li";
        start += "\n";
        of.write(start.c_str(), start.size());
        of.flush();
        unsigned start_index = 1;
        if (mesh_generator_settings.enable_shoot_branch) {
          unsigned tree_index = 0;
          for (const auto& entity : *tree_entities) {
            const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
            const auto mesh = tree->GenerateShootMesh(mesh_generator_settings);
            auto& vertices = mesh->UnsafeGetVertices();
            auto& triangles = mesh->UnsafeGetTriangles();
            const auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o branch " + std::to_string(tree_index) + "\n";
#pragma region Data collection
              for (auto& vertex : vertices) {
                auto vertex_position = glm::vec4(vertex.position, 1.0f);
                vertex_position = gt.value * vertex_position;
                auto& color = vertex.color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto triangle : triangles) {
                const auto f1 = triangle.x + start_index;
                const auto f2 = triangle.y + start_index;
                const auto f3 = triangle.z + start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              start_index += vertices.size();
            }
            tree_index++;
          }
        }
        if (mesh_generator_settings.enable_foliage) {
          unsigned tree_index = 0;
          for (const auto& entity : *tree_entities) {
            const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
            const auto mesh = tree->GenerateFoliageMesh(mesh_generator_settings);
            auto& vertices = mesh->UnsafeGetVertices();
            auto& triangles = mesh->UnsafeGetTriangles();
            const auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o foliage " + std::to_string(tree_index) + "\n";
#pragma region Data collection
              for (auto& vertex : vertices) {
                auto vertex_position = glm::vec4(vertex.position, 1.0f);
                vertex_position = gt.value * vertex_position;
                auto& color = vertex.color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto triangle : triangles) {
                const auto f1 = triangle.x + start_index;
                const auto f2 = triangle.y + start_index;
                const auto f3 = triangle.z + start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              start_index += vertices.size();
            }
            tree_index++;
          }
        }
        of.close();
      }
    }
  }
}