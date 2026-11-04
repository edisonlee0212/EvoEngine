#include "Profiler.hpp"
#include "TreeEditorState.hpp"
//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabEditorLayer.hpp"
#include "EcoSysLabSystem.hpp"

#include "AdvancedShootDescriptor.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "BranchRenderer.hpp"
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "FoliageRenderer.hpp"
#include "ForestDescriptor.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Times.hpp"
#include "Tree.hpp"
#include "TreeDebugRenderer.hpp"
#include "TreeStructor.hpp"
using namespace eco_sys_lab_package;

void EcoSysLabEditorLayer::TreeVisualization(const std::shared_ptr<EditorLayer>& editor_layer) {
  const ProfilerScope scope("EcoSysLab / Visual overlay update", "EcoSysLab");
  const auto runtime = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!runtime)
    return;

  const auto scene = GetScene();
  tree_debug_requested_ = Platform::MeshShaderEnabled();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  const auto branch_strands = shoot_stem_strands_.Get<Strands>();
  if (tree_entities && !tree_entities->empty()) {
    std::vector<TreeVisualizationInput> inputs;
    inputs.reserve(tree_entities->size());
    for (const auto entity : *tree_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
      inputs.push_back({entity, tree->GetShootModelRevision().content,
                        tree->shoot_model.RefShootSkeleton().GetVersion(),
                        scene->GetDataComponent<GlobalTransform>(entity).value,
                        scene->IsEntityEnabled(entity) && tree->IsEnabled()});
    }
    if (inputs != tree_visualization_inputs_) {
      tree_visualization_inputs_ = std::move(inputs);
      need_full_flow_update = true;
    }
    const bool full_update = need_full_flow_update;
    if (full_update || need_flow_update_for_selection_) {
      UpdateFlows(tree_entities, branch_strands, !full_update);
      if (full_update)
        UpdateGroundFruitAndLeaves();
      need_full_flow_update = need_flow_update_for_selection_ = false;
    }
    if (full_update) {
      if (const auto system = scene->GetSystem<EcoSysLabSystem>()) {
        const auto& voxel_grid = system->environment.climate.environment_grid.voxel_grid;
        const auto num_voxels = voxel_grid.GetVoxelCount();
        if (tree_visualization_settings_.show_shadow_grid) {
          std::vector<ParticleInfo> particle_infos;
          particle_infos.resize(num_voxels);

          Jobs::RunParallelFor(num_voxels, [&](size_t i) {
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
        } else {
          shadow_grid_particle_info_list_->SetParticleInfos({});
        }
        if (tree_visualization_settings_.show_lighting_grid) {
          std::vector<ParticleInfo> particle_infos;
          particle_infos.resize(num_voxels);

          Jobs::RunParallelFor(num_voxels, [&](size_t i) {
            const auto coordinate = voxel_grid.GetCoordinate(i);
            const auto& voxel = voxel_grid.Peek(coordinate);
            const auto direction = glm::length(voxel.light_direction) > glm::epsilon<float>()
                                       ? glm::normalize(voxel.light_direction)
                                       : glm::vec3(0, 1, 0);
            const auto up = glm::abs(direction.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
            auto rotation = glm::quatLookAt(direction, up);
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
        } else {
          lighting_grid_particle_info_list_->SetParticleInfos({});
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
    if (editor_layer && scene->IsEntityValid(selected_tree) && scene->IsEntityEnabled(selected_tree) &&
        scene->HasPrivateComponent<Tree>(selected_tree) &&
        scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock()->IsEnabled()) {
      const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
      auto& shoot_model = tree->shoot_model;
      auto& root_model = tree->root_model;
      auto& shoot_visualizer = GetTreeEditorState(*tree).shoot_visualizer;
      auto& root_visualizer = GetTreeEditorState(*tree).root_visualizer;
      const auto global_transform = scene->GetDataComponent<GlobalTransform>(selected_tree);
      const bool can_edit = CanEditTree(*tree);
      if (!can_edit && tree_operator_mode != static_cast<unsigned>(TreeOperatorMode::Disabled))
        tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Select);
      if (Platform::MeshShaderEnabled())
        tree_debug_renderer_->PollPick(tree, shoot_visualizer, root_visualizer);
      if (EditorLayer::GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Release &&
          (shoot_visualizer.checkpoint_iteration == shoot_model.CurrentIteration() ||
           tree_operator_mode == static_cast<unsigned>(TreeOperatorMode::Select))) {
        auto& tree_skeleton =
            shoot_model.PeekShootSkeleton(GetTreeEditorState(*tree).shoot_visualizer.checkpoint_iteration);
        switch (static_cast<TreeOperatorMode>(tree_operator_mode)) {
          case TreeOperatorMode::Select: {
            if (visualization_camera_window_focused_) {
              if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                if (Platform::MeshShaderEnabled())
                  tree_debug_renderer_->RequestPick(tree, visualization_camera_mouse_position);
                else if (shoot_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                           tree_skeleton, global_transform)) {
                  shoot_visualizer.need_update = true;
                }
              } else if (can_edit && shoot_visualizer.checkpoint_iteration == shoot_model.CurrentIteration() &&
                         EditorLayer::GetKey(GLFW_KEY_R) == Input::KeyActionType::Press) {
                if (shoot_visualizer.selected_node_handle > 0) {
                  shoot_model.Step();
                  auto& pruning_internode =
                      shoot_model.RefShootSkeleton().RefNode(shoot_visualizer.selected_node_handle);
                  shoot_model.RefShootSkeleton().RemoveNodes(pruning_internode.PeekChildHandles());
                  pruning_internode.data.internode_length *= shoot_visualizer.selected_node_length_factor;
                  shoot_model.CalculateTransform(tree->shoot_growth_controller_, true);
                  shoot_visualizer.selected_node_length_factor = 1.0f;
                  pruning_internode.data.buds.clear();
                  tree->NotifyShootModelChanged();
                  shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                  shoot_visualizer.need_update = true;
                  if (auto_generate_mesh_after_editing_ &&
                      !scene->HasPrivateComponent<BranchRenderer>(tree->GetOwner()) &&
                      !scene->HasPrivateComponent<FoliageRenderer>(tree->GetOwner())) {
                    tree->GenerateGeometryEntities(runtime->mesh_generator_settings, -1);
                  }
                  if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
                    if (auto_generate_strands_after_editing_) {
                      tree->InitializeStrandRenderer();
                    }
                    if (auto_generate_strand_mesh_after_editing_) {
                      tree->InitializeStrandModelMeshRenderer(runtime->strand_mesh_generator_settings);
                    }
                  }
                }
              } else if (can_edit && shoot_visualizer.checkpoint_iteration == shoot_model.CurrentIteration() &&
                         EditorLayer::GetKey(GLFW_KEY_T) == Input::KeyActionType::Press) {
                if (shoot_visualizer.selected_node_handle > 0) {
                  shoot_model.Step();
                  shoot_model.RefShootSkeleton().RemoveNodes({shoot_visualizer.selected_node_handle});
                  tree->NotifyShootModelChanged();
                  shoot_visualizer.selected_node_handle = -1;
                  shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                  shoot_visualizer.need_update = true;
                  if (auto_generate_mesh_after_editing_ &&
                      !scene->HasPrivateComponent<BranchRenderer>(tree->GetOwner()) &&
                      !scene->HasPrivateComponent<FoliageRenderer>(tree->GetOwner())) {
                    tree->GenerateGeometryEntities(runtime->mesh_generator_settings, -1);
                  }
                  if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
                    if (auto_generate_strands_after_editing_) {
                      tree->InitializeStrandRenderer();
                    }
                    if (auto_generate_strand_mesh_after_editing_) {
                      tree->InitializeStrandModelMeshRenderer(runtime->strand_mesh_generator_settings);
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
                      tree->NotifyShootModelChanged(false);
                      last_gizmos_used = true;
                    } else if (last_gizmos_used) {
                      shoot_model.CalculateTransform(tree->shoot_growth_controller_, true);
                      may_need_geometry_generation = true;
                      last_gizmos_used = false;
                      shoot_visualizer.need_update = true;
                      shoot_model.RefShootSkeleton().CalculateRegulatedGlobalRotation();
                      tree->NotifyShootModelChanged(false);
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
                  tree->NotifyShootModelChanged();
                  shoot_visualizer.selected_node_handle = -1;
                  shoot_visualizer.checkpoint_iteration = shoot_model.CurrentIteration();
                  shoot_visualizer.need_update = true;
                  if (auto_generate_mesh_after_editing_ &&
                      !scene->HasPrivateComponent<BranchRenderer>(tree->GetOwner()) &&
                      !scene->HasPrivateComponent<FoliageRenderer>(tree->GetOwner())) {
                    tree->GenerateGeometryEntities(runtime->mesh_generator_settings, -1);
                  }
                  if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
                    if (auto_generate_strands_after_editing_) {
                      tree->InitializeStrandRenderer();
                    }
                    if (auto_generate_strand_mesh_after_editing_) {
                      tree->InitializeStrandModelMeshRenderer(runtime->strand_mesh_generator_settings);
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
                    tree->NotifyShootModelChanged();
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
                if (runtime->GetApplication().GetApplicationStatus() == Application::ExecutionStatus::Playing ||
                    runtime->GetApplication().GetApplicationStatus() == Application::ExecutionStatus::Step) {
                  scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f)->growth.RequestInvigoration(
                      tree->GetOwner(), shoot_visualizer.selected_node_handle);
                  shoot_visualizer.need_update = true;
                  may_need_geometry_generation = true;
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
                  tree->NotifyShootModelChanged();
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
          if (auto_generate_mesh_after_editing_ && !scene->HasPrivateComponent<BranchRenderer>(tree->GetOwner()) &&
              !scene->HasPrivateComponent<FoliageRenderer>(tree->GetOwner())) {
            tree->GenerateGeometryEntities(runtime->mesh_generator_settings, -1);
          }
          if (auto_generate_strands_after_editing_ || auto_generate_strand_mesh_after_editing_) {
            tree->BuildStrandModel();
            if (auto_generate_strands_after_editing_) {
              auto strands = tree->GenerateStrands();
              tree->InitializeStrandRenderer(strands);
            }
            if (auto_generate_strand_mesh_after_editing_) {
              tree->InitializeStrandModelMeshRenderer(runtime->strand_mesh_generator_settings);
            }
          }
        } else if (shoot_visualizer.need_update && auto_generate_skeletal_graph_every_frame_) {
          tree->GenerateSkeletalGraph(runtime->skeletal_graph_settings, shoot_visualizer.selected_node_handle,
                                      Resources::GetInstance().GetPrimitives().sphere,
                                      Resources::GetInstance().GetPrimitives().cube);
        }
        may_need_geometry_generation = false;
      }
      if (Platform::MeshShaderEnabled())
        tree_debug_requested_ = true;
      else {
        root_visualizer.Visualize(root_model, global_transform);
        shoot_visualizer.Visualize(shoot_model, global_transform);
      }
    }
    if (tree_visualization_settings_.show_shadow_grid) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube, visualization_camera_,
                                                  shadow_grid_particle_info_list_, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);
    }
    if (tree_visualization_settings_.show_lighting_grid) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube, visualization_camera_,
                                                  lighting_grid_particle_info_list_, glm::mat4(1.0f), 1.0f,
                                                  gizmo_settings);
    }
    if (tree_visualization_settings_.display_flowers && !flower_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cone, visualization_camera_,
                                                  flower_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    if (tree_visualization_settings_.display_fruits && !fruit_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube, visualization_camera_,
                                                  fruit_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
    if (tree_visualization_settings_.display_foliage && !foliage_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().quad, visualization_camera_,
                                                  foliage_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;
    if (tree_visualization_settings_.display_ground_leaves && !ground_leaf_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().quad, visualization_camera_,
                                                  ground_leaf_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;
    if (tree_visualization_settings_.display_ground_fruits && !ground_fruit_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube, visualization_camera_,
                                                  ground_fruit_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }
    if (tree_visualization_settings_.display_ground_flowers &&
        !ground_flower_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cone, visualization_camera_,
                                                  ground_flower_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }

    if (tree_visualization_settings_.display_bounding_box && !bounding_box_matrices_->PeekParticleInfoList().empty()) {
      editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube, visualization_camera_,
                                                  bounding_box_matrices_, glm::mat4(1.0f), 1.0f, gizmo_settings);
    }

    if (!Platform::MeshShaderEnabled() && tree_visualization_settings_.display_shoot_stem &&
        !shoot_stem_points_.empty()) {
      gizmo_settings.color_mode = GizmoSettings::ColorMode::Default;
      editor_layer->DrawGizmoStrands(branch_strands, visualization_camera_, glm::vec4(1.0f, 1.0f, 1.0f, 0.75f),
                                     glm::mat4(1.0f), 1, gizmo_settings);
    }

    gizmo_settings.color_mode = GizmoSettings::ColorMode::Default;
  }
}

bool EcoSysLabEditorLayer::TreeVisualizationSettings::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  changed |= ImGui::Checkbox(Platform::MeshShaderEnabled() ? "Display other trees" : "Display shoot stem",
                             &display_shoot_stem);

  changed |= ImGui::Checkbox("Display foliage", &display_foliage);
  changed |= ImGui::Checkbox("Display flowers", &display_flowers);
  changed |= ImGui::Checkbox("Display fruits", &display_fruits);

  changed |= ImGui::Checkbox("Display ground leaves", &display_ground_leaves);
  changed |= ImGui::Checkbox("Display ground flowers", &display_ground_flowers);
  changed |= ImGui::Checkbox("Display ground fruit", &display_ground_fruits);

  changed |= ImGui::Checkbox("Display Bounding Box", &display_bounding_box);
  changed |= ImGui::Checkbox("Show Shadow Grid", &show_shadow_grid);
  changed |= ImGui::Checkbox("Show Lighting Direction Grid", &show_lighting_grid);

  return changed;
}
