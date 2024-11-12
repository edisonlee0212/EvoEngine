//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include <RayTracerLayer.hpp>
#endif
#include "BarkDescriptor.hpp"
#include "Times.hpp"

#include "BillboardCloudsConverter.hpp"
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "DynamicStrandsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "FlowerDescriptor.hpp"
#include "FoliageDescriptor.hpp"
#include "ForestDescriptor.hpp"
#include "FruitDescriptor.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "StrandsRenderer.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"
using namespace eco_sys_lab_plugin;
PrivateComponentRegistration<Tree> tree_registry("Tree");
PrivateComponentRegistration<TreeStructor> tree_structor_registry("TreeStructor");
PrivateComponentRegistration<Soil> soil_registry("Soil");
PrivateComponentRegistration<Climate> climate_registry("Climate");

PrivateComponentRegistration<DynamicTreeStrands> dynamic_tree_strands_registry("DynamicTreeStrands");

PrivateComponentRegistration<SpatialPlantDistributionSimulator> spds_registry("SpatialPlantDistributionSimulator");

AssetRegistration<ProceduralNoise2D> procedural_noise2d_registry("ProceduralNoise2D", {".noise2D"});
AssetRegistration<ProceduralNoise3D> procedural_noise3d_registry("ProceduralNoise3D", {".noise3D"});
AssetRegistration<BarkDescriptor> bark_descriptor_registry("BarkDescriptor", {".bark"});
AssetRegistration<ForestDescriptor> forest_d_registry("ForestDescriptor", {".forest"});
AssetRegistration<TreeDescriptor> tree_d_registry("TreeDescriptor", {".tree"});
AssetRegistration<ShootDescriptor> shoot_d_registry("ShootDescriptor", {".shoot"});
AssetRegistration<FruitDescriptor> fruit_d_registry("FruitDescriptor", {".fruit"});
AssetRegistration<FlowerDescriptor> flower_d_registry("FlowerDescriptor", {".flower"});
AssetRegistration<FoliageDescriptor> foliage_d_registry("FoliageDescriptor", {".foliage"});
AssetRegistration<SoilDescriptor> soil_d_registry("SoilDescriptor", {".soil"});
AssetRegistration<ClimateDescriptor> climate_d_registry("ClimateDescriptor", {".climate"});
AssetRegistration<RadialBoundingVolume> rbv_registry("RadialBoundingVolume", {".rbv"});
AssetRegistration<CubeVolume> cube_volume_registry("CubeVolume", {".cubevolume"});
AssetRegistration<HeightField> height_field_registry("HeightField", {".heightfield"});
AssetRegistration<SoilLayerDescriptor> soil_layer_d_registry("SoilLayerDescriptor", {".soillayer"});

AssetRegistration<ForestPatch> forest_patch_registry("ForestPatch", {".forestpatch"});

PrivateComponentRegistration<BillboardCloudsConverter> billboard_clouds_converter_register("BillboardCloudsConverter");

void EcoSysLabLayer::OnCreate() {
  Shader::RegisterShaderIncludePath(std::filesystem::path("./EcoSysLabResources/Shaders/Includes"));
  if (random_colors_.empty()) {
    for (int i = 0; i < 20000; i++) {
      random_colors_.emplace_back(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
    }
  }

  if (soil_layer_colors_.empty()) {
    for (int i = 0; i < 10; i++) {
      glm::vec4 color = {glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)), 1.0f};
      soil_layer_colors_.emplace_back(color);
    }
  }
  shoot_stem_strands_ = ProjectManager::CreateTemporaryAsset<Strands>();

  bounding_box_matrices_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  foliage_matrices_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  fruit_matrices_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  ground_fruit_matrices_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  ground_leaf_matrices_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  vector_matrices_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  scalar_matrices_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  shadow_grid_particle_info_list_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  lighting_grid_particle_info_list_ = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
#pragma region Internode camera
  visualization_camera_ = Serialization::ProduceSerializable<Camera>();

  visualization_camera_->OnCreate();
  visualization_camera_->use_clear_color = true;
  visualization_camera_->clear_color = glm::vec3(0.5f, 0.5f, 0.5f);
#pragma endregion

  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    editor_layer->RegisterEditorCamera(visualization_camera_);
  }
}

void EcoSysLabLayer::TreeVisualization(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto scene = GetScene();
  const auto selected_entity = editor_layer->GetSelectedEntity();
  if (selected_entity != selected_tree) {
    if (scene->IsEntityValid(selected_entity) && scene->HasPrivateComponent<Tree>(selected_entity)) {
      selected_tree = selected_entity;
      last_selected_tree_index_ = selected_tree.GetIndex();
      need_flow_update_for_selection_ = true;
      tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Select);
    } else if (selected_tree.GetIndex() != 0) {
      selected_tree = Entity();
      need_flow_update_for_selection_ = true;
    }
    if (scene->IsEntityValid(selected_tree)) {
      const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
      auto& tree_visualizer = tree->tree_visualizer;
      tree_visualizer.m_selectedInternodeHandle = -1;
      tree_visualizer.m_selectedInternodeHierarchyList.clear();
      tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Select);
    }
  }
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  const auto branch_strands = shoot_stem_strands_.Get<Strands>();
  if (tree_entities && !tree_entities->empty()) {
    // Tree selection
    if (shoot_versions_.size() != tree_entities->size()) {
      internode_size_ = 0;
      root_node_size_ = 0;
      total_time_ = 0.0f;
      shoot_versions_.clear();
      for (int i = 0; i < tree_entities->size(); i++) {
        shoot_versions_.emplace_back(-1);
      }
      need_full_flow_update = true;
    }
    for (int i = 0; i < tree_entities->size(); i++) {
      auto tree_entity = tree_entities->at(i);
      auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      if (auto& tree_model = tree->tree_model; shoot_versions_[i] != tree_model.RefShootSkeleton().GetVersion()) {
        shoot_versions_[i] = tree_model.RefShootSkeleton().GetVersion();
        need_full_flow_update = true;
      }
    }

    if (show_trees) {
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

      if (editor_layer && scene->IsEntityValid(selected_tree)) {
        const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
        auto& tree_model = tree->tree_model;
        auto& tree_visualizer = tree->tree_visualizer;
        const auto global_transform = scene->GetDataComponent<GlobalTransform>(selected_tree);
#ifdef OPTIX_RAY_TRACER_PLUGIN
        const auto ray_tracer_layer = Application::GetLayer<RayTracerLayer>();
#endif
        if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == Input::KeyActionType::Release &&
            tree_visualizer.m_checkpointIteration == tree_model.CurrentIteration()) {
          static bool may_need_geometry_generation = false;
          static std::vector<glm::vec2> mouse_positions{};
          auto& tree_skeleton = tree_model.PeekShootSkeleton(tree->tree_visualizer.m_checkpointIteration);
          switch (static_cast<TreeOperatorMode>(tree_operator_mode)) {
            case TreeOperatorMode::Select: {
              if (visualization_camera_window_focused_) {
                if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                  if (tree_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                       tree_skeleton, global_transform)) {
                    tree_visualizer.m_needUpdate = true;
                  }
                } else if (editor_layer->GetKey(GLFW_KEY_R) == Input::KeyActionType::Press) {
                  if (tree_visualizer.m_selectedInternodeHandle > 0) {
                    tree_model.Step();
                    auto& pruning_internode =
                        tree_model.RefShootSkeleton().RefNode(tree_visualizer.m_selectedInternodeHandle);
                    tree_model.RefShootSkeleton().RemoveNodes(pruning_internode.PeekChildHandles());
                    pruning_internode.data.internode_length *= tree_visualizer.m_selectedInternodeLengthFactor;
                    tree_model.CalculateTransform(tree->shoot_growth_controller_, true);
                    tree_visualizer.m_selectedInternodeLengthFactor = 1.0f;
                    pruning_internode.data.buds.clear();
                    tree_visualizer.m_checkpointIteration = tree_model.CurrentIteration();
                    tree_visualizer.m_needUpdate = true;
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
                } else if (editor_layer->GetKey(GLFW_KEY_T) == Input::KeyActionType::Press) {
                  if (tree_visualizer.m_selectedInternodeHandle > 0) {
                    tree_model.Step();
                    tree_model.RefShootSkeleton().RemoveNodes({tree_visualizer.m_selectedInternodeHandle});
                    tree_visualizer.m_selectedInternodeHandle = -1;
                    tree_visualizer.m_checkpointIteration = tree_model.CurrentIteration();
                    tree_visualizer.m_needUpdate = true;
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
                } else if (editor_layer->GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
                  tree_visualizer.SetSelectedNode(tree_skeleton, -1);
                }
              }
            } break;
            case TreeOperatorMode::Rotate: {
              if (tree_visualizer.m_selectedInternodeHandle > 0) {
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
                      auto& current_skeleton = tree->tree_model.RefShootSkeleton();
                      auto& internode = current_skeleton.RefNode(tree_visualizer.m_selectedInternodeHandle);

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
                          tree_model.Step();
                          tree_visualizer.m_checkpointIteration = tree_model.CurrentIteration();
                        }
                        tree_visualizer.m_needUpdate = true;
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
                        tree_model.CalculateTransform(tree->shoot_growth_controller_, true);
                        last_gizmos_used = true;
                      } else if (last_gizmos_used) {
                        tree_model.CalculateTransform(tree->shoot_growth_controller_, true);
                        may_need_geometry_generation = true;
                        last_gizmos_used = false;
                        tree_visualizer.m_needUpdate = true;
                        tree_model.RefShootSkeleton().CalculateRegulatedGlobalRotation();
                      }
                    }
                    ImGui::EndChild();
                  }
                  ImGui::End();
                  ImGui::PopStyleVar();
                }
#ifdef OPTIX_RAY_TRACER_PLUGIN
                else if (ray_tracer_layer) {
                  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
                  if (ImGui::Begin("Scene (RT)")) {
                    if (ImGui::BeginChild("RaySceneRenderer", ImVec2(0, 0), false)) {
                      ImGuizmo::SetOrthographic(false);
                      ImGuizmo::SetDrawlist();
                      ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y,
                                        ray_tracer_layer->GetSceneCameraResolution().x,
                                        ray_tracer_layer->GetSceneCameraResolution().y);
                      glm::mat4 camera_view = glm::inverse(glm::translate(editor_layer->GetSceneCameraPosition()) *
                                                           glm::mat4_cast(editor_layer->GetSceneCameraRotation()));
                      glm::mat4 camera_projection = visualization_camera_->GetProjection();
                      const auto op = ImGuizmo::OPERATION::ROTATE;
                      auto& current_skeleton = tree->tree_model.RefShootSkeleton();
                      auto& internode = current_skeleton.RefNode(tree_visualizer.m_selectedInternodeHandle);

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
                          tree_model.Step();
                          tree_visualizer.m_checkpointIteration = tree_model.CurrentIteration();
                        }
                        tree_visualizer.m_needUpdate = true;
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
                        tree_model.CalculateTransform(tree->shoot_growth_controller_, true);
                        last_gizmos_used = true;
                      } else if (last_gizmos_used) {
                        tree_model.CalculateTransform(tree->shoot_growth_controller_, true);
                        may_need_geometry_generation = true;
                        last_gizmos_used = false;
                        tree_visualizer.m_needUpdate = true;
                        tree_model.RefShootSkeleton().CalculateRegulatedGlobalRotation();
                      }
                    }
                    ImGui::EndChild();
                  }
                  ImGui::End();
                  ImGui::PopStyleVar();
                }
#endif
              }
              if (visualization_camera_window_focused_) {
                if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                  if (tree_visualizer.m_selectedInternodeHandle <= 0) {
                    if (tree_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                         tree_skeleton, global_transform)) {
                      tree_visualizer.m_needUpdate = true;
                    }
                  }
                } else if (editor_layer->GetKey(GLFW_KEY_T) == Input::KeyActionType::Press) {
                  if (tree_visualizer.m_selectedInternodeHandle > 0) {
                    tree_model.Step();
                    tree_model.RefShootSkeleton().RemoveNodes({tree_visualizer.m_selectedInternodeHandle});
                    tree_visualizer.m_selectedInternodeHandle = -1;
                    tree_visualizer.m_checkpointIteration = tree_model.CurrentIteration();
                    tree_visualizer.m_needUpdate = true;
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
                } else if (editor_layer->GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
                  tree_visualizer.SetSelectedNode(tree_skeleton, -1);
                }
              }
            } break;
            case TreeOperatorMode::Prune: {
              if (visualization_camera_window_focused_) {
                if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
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
                } else if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Hold) {
                  glm::vec2 mouse_position = visualization_camera_mouse_position;
                  const float half_x = visualization_camera_->GetSize().x / 2.0f;
                  const float half_y = visualization_camera_->GetSize().y / 2.0f;
                  mouse_position = {-1.0f * (mouse_position.x - half_x) / half_x,
                                    -1.0f * (mouse_position.y - half_y) / half_y};
                  if (mouse_position.x > -1.0f && mouse_position.x < 1.0f && mouse_position.y > -1.0f &&
                      mouse_position.y < 1.0f &&
                      (!mouse_positions.empty() && mouse_position != mouse_positions.back())) {
                    mouse_positions.emplace_back(mouse_position);
                  }
                } else if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Release) {
                  // Once released, check if empty.
                  if (!mouse_positions.empty()) {
                    tree_model.Step();
                    auto& skeleton = tree_model.RefShootSkeleton();
                    std::vector<SkeletonNodeHandle> pruning_node_handles;
                    if (tree_visualizer.ScreenCurveSelection(
                            [&](const SkeletonNodeHandle node_handle) {
                              pruning_node_handles.emplace_back(node_handle);
                            },
                            mouse_positions, skeleton, global_transform)) {
                      tree_model.RefShootSkeleton().RemoveNodes(pruning_node_handles);
                      tree_visualizer.m_checkpointIteration = tree_model.CurrentIteration();
                      tree_visualizer.m_needUpdate = true;
                      may_need_geometry_generation = true;
                    } else {
                      tree_model.Pop();
                    }
                    mouse_positions.clear();
                  }
                }
              }
            } break;
            case TreeOperatorMode::Invigorate: {
              if (visualization_camera_window_focused_) {
                static bool last_frame_invigorate = false;
                if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                  if (tree_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                       tree_skeleton, global_transform)) {
                    if (!tree->enable_history)
                      tree_model.Step();
                    tree_visualizer.m_checkpointIteration = tree_model.CurrentIteration();
                    tree_visualizer.m_needUpdate = true;
                  }
                } else if (tree_visualizer.m_selectedInternodeHandle >= 0 &&
                           editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Hold) {
                  const auto climate_candidate = FindClimate();
                  if (!climate_candidate.expired()) {
                    climate_candidate.lock()->PrepareForGrowth();
                    if (tree->TryGrowSubTree(simulation_settings.delta_time, tree_visualizer.m_selectedInternodeHandle,
                                             false)) {
                      tree_visualizer.m_needUpdate = true;
                      may_need_geometry_generation = true;
                    }
                  }
                  last_frame_invigorate = true;
                } else if (last_frame_invigorate &&
                           editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Release) {
                  tree_visualizer.SetSelectedNode(tree_skeleton, -1);
                  last_frame_invigorate = false;
                  tree_visualizer.m_needUpdate = true;
                }
              }
            } break;
            case TreeOperatorMode::Reduce: {
              if (visualization_camera_window_focused_) {
                static bool last_frame_reduce = false;
                static float target_age = 0.0f;
                if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press) {
                  if (tree_visualizer.RayCastSelection(visualization_camera_, visualization_camera_mouse_position,
                                                       tree_skeleton, global_transform)) {
                    if (!tree->enable_history)
                      tree_model.Step();
                    tree_visualizer.m_checkpointIteration = tree_model.CurrentIteration();
                    tree_visualizer.m_needUpdate = true;
                    if (tree_visualizer.m_selectedInternodeHandle >= 0) {
                      target_age = tree->tree_model.GetSubTreeMaxAge(tree_visualizer.m_selectedInternodeHandle);
                    }
                  }
                } else if (tree_visualizer.m_selectedInternodeHandle >= 0 &&
                           editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Hold) {
                  if (tree->tree_model.Reduce(tree->shoot_growth_controller_, tree_visualizer.m_selectedInternodeHandle,
                                              target_age)) {
                    tree_visualizer.m_needUpdate = true;
                    may_need_geometry_generation = true;
                  }
                  target_age -= tree_reduce_rate;
                  last_frame_reduce = true;
                } else if (last_frame_reduce &&
                           editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Release) {
                  tree_visualizer.SetSelectedNode(tree_skeleton, -1);
                  last_frame_reduce = false;
                  tree_visualizer.m_needUpdate = true;
                }
              }
            } break;
          }

          if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Release &&
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
          } else if (tree_visualizer.m_needUpdate && auto_generate_skeletal_graph_every_frame_) {
            tree->GenerateSkeletalGraph(skeletal_graph_settings, tree_visualizer.m_selectedInternodeHandle,
                                        Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"),
                                        Resources::GetResource<Mesh>("PRIMITIVE_CUBE"));
          }
          may_need_geometry_generation = false;
        }
        tree_visualizer.Visualize(tree_model, global_transform);
      }

      if (tree_visualizer_settings_.show_shadow_grid) {
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
                                                    visualization_camera_, shadow_grid_particle_info_list_,
                                                    glm::mat4(1.0f), 1.0f, gizmo_settings);
      }
      if (tree_visualizer_settings_.show_lighting_grid) {
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
                                                    visualization_camera_, lighting_grid_particle_info_list_,
                                                    glm::mat4(1.0f), 1.0f, gizmo_settings);
      }
      if (tree_visualizer_settings_.display_shoot_stem && !shoot_stem_points_.empty()) {
        gizmo_settings.color_mode = GizmoSettings::ColorMode::Default;
        editor_layer->DrawGizmoStrands(branch_strands, visualization_camera_, glm::vec4(1.0f, 1.0f, 1.0f, 0.75f),
                                       glm::mat4(1.0f), 1, gizmo_settings);
      }
      if (tree_visualizer_settings_.display_fruit && !fruit_matrices_->PeekParticleInfoList().empty()) {
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
                                                    visualization_camera_, fruit_matrices_, glm::mat4(1.0f), 1.0f,
                                                    gizmo_settings);
      }
      gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
      if (tree_visualizer_settings_.display_foliage && !foliage_matrices_->PeekParticleInfoList().empty()) {
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"),
                                                    visualization_camera_, foliage_matrices_, glm::mat4(1.0f), 1.0f,
                                                    gizmo_settings);
      }
      if (tree_visualizer_settings_.display_ground_leaves && !ground_leaf_matrices_->PeekParticleInfoList().empty()) {
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"),
                                                    visualization_camera_, ground_leaf_matrices_, glm::mat4(1.0f), 1.0f,
                                                    gizmo_settings);
      }
      gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;

      if (tree_visualizer_settings_.display_ground_fruit && !ground_fruit_matrices_->PeekParticleInfoList().empty()) {
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
                                                    visualization_camera_, ground_fruit_matrices_, glm::mat4(1.0f),
                                                    1.0f, gizmo_settings);
      }

      if (tree_visualizer_settings_.display_bounding_box && !bounding_box_matrices_->PeekParticleInfoList().empty()) {
        editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
                                                    visualization_camera_, bounding_box_matrices_, glm::mat4(1.0f),
                                                    1.0f, gizmo_settings);
      }

      gizmo_settings.color_mode = GizmoSettings::ColorMode::Default;
      if (tree_visualizer_settings_.display_soil) {
        SoilVisualization();
      }
    }
  }
}

void EcoSysLabLayer::ResetAllTrees(const std::vector<Entity>* tree_entities) {
  const auto scene = Application::GetActiveScene();
  simulated_time_ = 0;
  if (tree_entities) {
    for (const auto& i : *tree_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
      tree->Reset();
    }
  }

  need_full_flow_update = true;
  total_time_ = 0;
  internode_size_ = 0;
  leaf_size_ = 0;
  fruit_size_ = 0;
  shoot_stem_size_ = 0;
  root_node_size_ = 0;
  root_stem_size_ = 0;

  shoot_stem_segments_.clear();
  shoot_stem_points_.clear();

  shoot_stem_strands_ = ProjectManager::CreateTemporaryAsset<Strands>();

  bounding_box_matrices_->SetParticleInfos({});
  foliage_matrices_->SetParticleInfos({});
  fruit_matrices_->SetParticleInfos({});

  const auto climate_candidate = FindClimate();
  if (!climate_candidate.expired()) {
    const auto climate = climate_candidate.lock();
    climate->climate_model.environment_grid = {};
  }
}

std::weak_ptr<Climate> EcoSysLabLayer::FindClimate() {
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* climate_entities = scene->UnsafeGetPrivateComponentOwnersList<Climate>();
  if (climate_entities && !climate_entities->empty()) {
    return scene->GetOrSetPrivateComponent<Climate>(climate_entities->at(0));
  }
  return {};
}

std::weak_ptr<Soil> EcoSysLabLayer::FindSoil() {
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soil_entities && !soil_entities->empty()) {
    return scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0));
  }
  return {};
}

const std::vector<glm::vec3>& EcoSysLabLayer::RandomColors() {
  return random_colors_;
}

void EcoSysLabLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto scene = GetScene();
  bool simulate = false;
  static bool auto_time_grow = false;
  static float target_time = 0.0f;
  static float extra_time = 4.f;
  if (ImGui::Begin("EcoSysLab Layer")) {
    ImGui::Checkbox("Show Trees", &show_trees);
    ImGui::Checkbox("Show Strands", &show_strands);
    if (show_trees) {
      const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      if (tree_entities && !tree_entities->empty()) {
        if (scene->IsEntityValid(selected_tree)) {
          const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
          auto& tree_visualizer = tree->tree_visualizer;
          if (tree_visualizer.m_checkpointIteration == tree->tree_model.CurrentIteration()) {
            if (ImGui::TreeNodeEx("Tree Operator", ImGuiTreeNodeFlags_DefaultOpen)) {
              if (ImGui::Combo("Mode", {"Select", "Rotate", "Prune", "Invigorate", "Reduce"}, tree_operator_mode)) {
                tree_visualizer.m_selectedInternodeHandle = -1;
                tree_visualizer.m_selectedInternodeHierarchyList.clear();
              }
              switch (static_cast<TreeOperatorMode>(tree_operator_mode)) {
                case TreeOperatorMode::Select:
                  ImGui::Text("Press T to cut off entire node, press R to cut at point of selection.");
                  break;
                case TreeOperatorMode::Rotate:
                  ImGui::Text("Press T to cut off entire node.");
                  break;
                case TreeOperatorMode::Prune:
                  break;
                case TreeOperatorMode::Invigorate:
                  break;
                case TreeOperatorMode::Reduce:
                  break;
              }
              if (tree_operator_mode == static_cast<unsigned>(TreeOperatorMode::Reduce)) {
                ImGui::DragFloat("Reduce speed", &tree_reduce_rate, 0.001f, 0.001f, 1.0f);
              }

              ImGui::TreePop();
            }
          } else {
            ImGui::Text("Go to current skeleton to enable operator!");
          }
          ImGui::Separator();
          if (ImGui::TreeNodeEx("Tree Visualizer")) {
            tree_visualizer.OnInspect(tree->tree_model);
            ImGui::TreePop();
          }
        } else {
          ImGui::Text("Select a tree entity to enable editing & visualization!");
        }
        if (ImGui::TreeNodeEx("Tree Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
          if (ImGui::TreeNode("Simulation Settings")) {
            simulation_settings.OnInspect(editor_layer);
            ImGui::TreePop();
          }
          if (ImGui::Button("Reset all trees")) {
            ResetAllTrees(tree_entities);
            ClearMeshes();
            ClearGroundFruitAndLeaf();
            target_time = 0.0f;
          }
          ImGui::Text(("Simulated time: " + std::to_string(simulated_time_) + " years").c_str());
          ImGui::DragInt("target nodes", &simulation_settings.max_node_count, 500, 0, INT_MAX);
          ImGui::DragFloat("target years", &extra_time, 0.1f, simulated_time_, 999);
          if (auto_time_grow) {
            if (ImGui::Button("Force stop")) {
              auto_time_grow = false;
              target_time = simulated_time_;
            }
          } else {
            if (ImGui::Button(("Grow " + std::to_string(extra_time) + " years").c_str())) {
              auto_time_grow = true;
              target_time += extra_time;
            }
          }
          if (ImGui::Button("Grow 1 iteration")) {
            simulate = true;
          }
          ImGui::TreePop();
        }
        if (!simulation_settings.auto_clear_fruit_and_leaves && ImGui::Button("Clear ground leaves and fruits")) {
          ClearGroundFruitAndLeaf();
        }
        if (ImGui::TreeNode("Tree Geometries")) {
          if (ImGui::TreeNode("Skeletal graph")) {
            skeletal_graph_settings.OnInspect();
            ImGui::TreePop();
          }
          if (ImGui::Button("Generate Skeletal graphs")) {
            GenerateSkeletalGraphs(skeletal_graph_settings);
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear Skeletal graphs")) {
            ClearSkeletalGraphs();
          }
          ImGui::Separator();
          if (ImGui::TreeNodeEx("Mesh generation")) {
            mesh_generator_settings.OnInspect(editor_layer);
            ImGui::TreePop();
          }
          if (ImGui::Button("Generate Meshes")) {
            GenerateMeshes(mesh_generator_settings);
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear Meshes")) {
            ClearMeshes();
          }
          ImGui::Separator();
          if (ImGui::TreeNodeEx("Strand Model Mesh generation")) {
            strand_mesh_generator_settings.OnInspect(editor_layer);
            ImGui::TreePop();
          }
          if (ImGui::Button("Build Strand Renderer")) {
            GenerateStrandRenderers();
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear Strand Renderer")) {
            ClearStrandRenderers();
          }
          if (ImGui::Button("Generate Strand Model Meshes")) {
            GenerateStrandModelMeshes(strand_mesh_generator_settings);
          }
          ImGui::SameLine();
          if (ImGui::Button("Clear Strand Model Meshes")) {
            ClearStrandModelMeshes();
          }
          ImGui::Separator();

          if (ImGui::TreeNode("Auto geometry generation")) {
            ImGui::Checkbox("Auto generate mesh", &auto_generate_mesh_after_editing_);
            ImGui::Checkbox("Auto generate Skeletal Graph Per Frame", &auto_generate_skeletal_graph_every_frame_);
            ImGui::Checkbox("Auto generate strands", &auto_generate_strands_after_editing_);
            ImGui::Checkbox("Auto generate strands mesh", &auto_generate_strand_mesh_after_editing_);

            ImGui::TreePop();
          }
          FileUtils::SaveFile(
              "Export all trees as OBJ", "OBJ", {".obj"},
              [&](const std::filesystem::path& path) {
                ExportAllTrees(path);
              },
              false);
          ImGui::TreePop();
        }

        if (ImGui::TreeNodeEx("Stats")) {
          ImGui::Text("Growth time: %.4f", last_used_time_);
          ImGui::Text("Total time: %.4f", total_time_);
          ImGui::Text("Tree count: %d", tree_entities->size());
          ImGui::Text("Total internode size: %d", internode_size_);
          ImGui::Text("Total shoot branch size: %d", shoot_stem_size_);
          ImGui::Text("Total fruit size: %d", fruit_size_);
          ImGui::Text("Total leaf size: %d", leaf_size_);
          ImGui::Text("Total root node size: %d", root_node_size_);
          ImGui::Text("Total root branch size: %d", root_stem_size_);
          ImGui::Text("Total ground leaf size: %d", leaves_.size());
          ImGui::Text("Total ground fruit size: %d", fruits_.size());
          ImGui::TreePop();
        }
      } else {
        ImGui::Text("No trees in the scene!");
        ResetAllTrees(nullptr);
        target_time = 0.0f;
      }
      if (ImGui::TreeNodeEx("Tree Visualization settings")) {
        if (ImGui::Button("Update")) {
          need_full_flow_update = true;
        }
        tree_visualizer_settings_.OnInspect(editor_layer);
        if (tree_visualizer_settings_.display_soil &&
            ImGui::TreeNodeEx("Soil visualization settings", ImGuiTreeNodeFlags_DefaultOpen)) {
          OnSoilVisualizationMenu();
          ImGui::TreePop();
        }
        ImGui::TreePop();
      }
    }
    if (show_strands) {
      if (ImGui::TreeNodeEx("Strand Visualization settings")) {
        strand_visualizer_settings_.OnInspect(editor_layer);
        ImGui::TreePop();
      }
    }
  }
  ImGui::End();
  if (simulate || auto_time_grow) {
    Simulate(simulation_settings);
    if (simulation_settings.auto_clear_fruit_and_leaves) {
      ClearGroundFruitAndLeaf();
    }
    if (scene->IsEntityValid(selected_tree)) {
      auto tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
      tree->tree_visualizer.m_checkpointIteration = tree->tree_model.CurrentIteration();
      tree->tree_visualizer.m_needUpdate = true;
      if (auto_generate_skeletal_graph_every_frame_) {
        tree->GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"),
                                    Resources::GetResource<Mesh>("PRIMITIVE_CUBE"));
      }
    }
  }
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    if (target_time <= simulated_time_ && auto_time_grow) {
      auto_time_grow = false;
      for (const auto& tree_entity : *tree_entities) {
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
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
      }
    }
  }
#pragma region Internode debugging camera
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Plant Visual")) {
    if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
      ImVec2 view_port_size;
      view_port_size = ImGui::GetWindowSize();
      visualization_camera_resolution_x = view_port_size.x;
      visualization_camera_resolution_y = view_port_size.y;
      ImGui::Image(visualization_camera_->GetRenderTexture()->GetColorImTextureId(),
                   ImVec2(view_port_size.x, view_port_size.y), ImVec2(0, 1), ImVec2(1, 0));
      visualization_camera_mouse_position = glm::vec2(FLT_MAX, -FLT_MAX);
      auto scene_camera_rotation = editor_layer->GetSceneCameraRotation();
      auto scene_camera_position = editor_layer->GetSceneCameraPosition();
      if (ImGui::IsWindowFocused()) {
        visualization_camera_window_focused_ = true;
        bool valid = true;
        auto mp = ImGui::GetMousePos();
        auto wp = ImGui::GetWindowPos();
        visualization_camera_mouse_position = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        if (valid) {
          static bool is_dragging_previously = false;
          bool mouse_drag = true;
          if (visualization_camera_mouse_position.x < 0 || visualization_camera_mouse_position.y < 0 ||
              visualization_camera_mouse_position.x > view_port_size.x ||
              visualization_camera_mouse_position.y > view_port_size.y ||
              editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Hold) {
            mouse_drag = false;
          }
          static float prev_x = 0;
          static float prev_y = 0;
          if (mouse_drag && !is_dragging_previously) {
            prev_x = visualization_camera_mouse_position.x;
            prev_y = visualization_camera_mouse_position.y;
          }
          const float x_offset = visualization_camera_mouse_position.x - prev_x;
          const float y_offset = visualization_camera_mouse_position.y - prev_y;
          prev_x = visualization_camera_mouse_position.x;
          prev_y = visualization_camera_mouse_position.y;
          is_dragging_previously = mouse_drag;
#pragma region Scene Camera Controller
          if (mouse_drag && !editor_layer->lock_camera) {
            glm::vec3 front = scene_camera_rotation * glm::vec3(0, 0, -1);
            glm::vec3 right = scene_camera_rotation * glm::vec3(1, 0, 0);
            if (editor_layer->GetKey(GLFW_KEY_W) == Input::KeyActionType::Hold) {
              scene_camera_position += front * static_cast<float>(Times::DeltaTime()) * editor_layer->velocity;
            }
            if (editor_layer->GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
              scene_camera_position -= front * static_cast<float>(Times::DeltaTime()) * editor_layer->velocity;
            }
            if (editor_layer->GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
              scene_camera_position -= right * static_cast<float>(Times::DeltaTime()) * editor_layer->velocity;
            }
            if (editor_layer->GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
              scene_camera_position += right * static_cast<float>(Times::DeltaTime()) * editor_layer->velocity;
            }
            if (editor_layer->GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
              scene_camera_position.y += editor_layer->velocity * static_cast<float>(Times::DeltaTime());
            }
            if (editor_layer->GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
              scene_camera_position.y -= editor_layer->velocity * static_cast<float>(Times::DeltaTime());
            }
            if (x_offset != 0.0f || y_offset != 0.0f) {
              front = glm::rotate(front, glm::radians(-x_offset * editor_layer->sensitivity), glm::vec3(0, 1, 0));
              const glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
              if ((front.y < 0.99f && y_offset < 0.0f) || (front.y > -0.99f && y_offset > 0.0f)) {
                front = glm::rotate(front, glm::radians(-y_offset * editor_layer->sensitivity), right);
              }
              const glm::vec3 up = glm::normalize(glm::cross(right, front));
              scene_camera_rotation = glm::quatLookAt(front, up);
            }
            editor_layer->SetCameraRotation(editor_layer->GetSceneCamera(), scene_camera_rotation);
            editor_layer->SetCameraPosition(editor_layer->GetSceneCamera(), scene_camera_position);
          }
#pragma endregion
        }
      } else {
        visualization_camera_window_focused_ = false;
      }
      editor_layer->SetCameraRotation(visualization_camera_, scene_camera_rotation);
      editor_layer->SetCameraPosition(visualization_camera_, scene_camera_position);
    }
    ImGui::EndChild();
    auto* window = ImGui::FindWindowByName("Plant Visual");
    visualization_camera_->SetEnabled(!(window->Hidden && !window->Collapsed));
  }
  ImGui::End();
  ImGui::PopStyleVar();
  TreeVisualization(editor_layer);
  StrandVisualization(editor_layer);
#pragma endregion
}

void EcoSysLabLayer::OnSoilVisualizationMenu() {
  static bool force_update;
  ImGui::Checkbox("Force Update", &force_update);

  if (ImGui::Checkbox("Vector Visualization", &vector_enable_)) {
    if (vector_enable_)
      update_vector_matrices_ = true;
  }

  if (ImGui::Checkbox("Scalar Visualization", &scalar_enable_)) {
    if (scalar_enable_)
      update_scalar_matrices_ = true;
  }

  if (vector_enable_) {
    update_vector_matrices_ = update_vector_matrices_ || force_update;

    if (ImGui::TreeNodeEx("Vector", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Reset")) {
        vector_multiplier_ = 50.0f;
        vector_base_color_ = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
        vector_soil_property_ = 4;
        vector_line_width_factor_ = 0.1f;
        vector_line_max_width_ = 0.1f;
        update_vector_matrices_ = true;
      }
      if (ImGui::ColorEdit4("Vector Base Color", &vector_base_color_.x)) {
        update_vector_matrices_ = true;
      }
      if (ImGui::DragFloat("Multiplier", &vector_multiplier_, 0.1f, 0.0f, 100.0f, "%.3f")) {
        update_vector_matrices_ = true;
      }
      if (ImGui::DragFloat("Line Width Factor", &vector_line_width_factor_, 0.01f, 0.0f, 5.0f)) {
        update_vector_matrices_ = true;
      }
      if (ImGui::DragFloat("Max Line Width", &vector_line_max_width_, 0.01f, 0.0f, 5.0f)) {
        update_vector_matrices_ = true;
      }
      if (ImGui::Combo("Vector Mode",
                       {"N/A", "N/A", "Water Density Gradient", "Flux", "Divergence", "N/A", "N/A", "N/A"},
                       vector_soil_property_)) {
        update_vector_matrices_ = true;
      }
      ImGui::TreePop();
    }
  }
  if (scalar_enable_) {
    update_scalar_matrices_ = update_scalar_matrices_ || force_update;

    if (scalar_enable_ && ImGui::TreeNodeEx("Scalar", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Reset")) {
        scalar_multiplier_ = 1.0f;
        scalar_box_size_ = 0.5f;
        scalar_min_alpha_ = 0.00f;
        scalar_base_color_ = glm::vec3(0.0f, 0.0f, 1.0f);
        scalar_soil_property_ = 1;
        update_scalar_matrices_ = true;
      }
      if (ImGui::SliderFloat("X Depth", &soil_cutout_x_depth_, 0.0f, 1.0f)) {
        update_scalar_matrices_ = true;
      }
      if (ImGui::SliderFloat("Z Depth", &soil_cutout_z_depth_, 0.0f, 1.0f)) {
        update_scalar_matrices_ = true;
      }

      if (ImGui::TreeNodeEx("Layer colors", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int i = 0; i < 10; i++) {
          ImGui::ColorEdit4(("Layer " + std::to_string(i)).c_str(), &soil_layer_colors_[i].x);
        }
        ImGui::TreePop();
      }

      if (ImGui::ColorEdit3("Scalar Base Color", &scalar_base_color_.x)) {
        update_scalar_matrices_ = true;
      }
      if (ImGui::SliderFloat("Multiplier", &scalar_multiplier_, 0.001, 10000, "%.4f", ImGuiSliderFlags_Logarithmic)) {
        update_scalar_matrices_ = true;
      }
      if (ImGui::DragFloat("Min alpha", &scalar_min_alpha_, 0.001f, 0.0f, 1.0f)) {
        update_scalar_matrices_ = true;
      }
      if (ImGui::DragFloat("Box size", &scalar_box_size_, 0.001f, 0.0f, 1.0f)) {
        update_scalar_matrices_ = true;
      }
      // disable less useful visualizations to avoid clutter in the gui
      if (ImGui::Combo(
              "Scalar Mode",
              {"Blank", "Water Density", "N/A", "N/A", "N/A", "Nutrient Density", "Soil Density", "Soil Layer"},
              scalar_soil_property_)) {
        update_scalar_matrices_ = true;
      }
      ImGui::TreePop();
    }
  }
}

void EcoSysLabLayer::UpdateFlows(const std::vector<Entity>* tree_entities,
                                 const std::shared_ptr<Strands>& branch_strands) {
  {
    const auto scene = Application::GetActiveScene();

    bounding_box_matrices_->SetParticleInfos({});

    std::vector<int> branch_start_indices;
    int branch_last_start_index = 0;
    branch_start_indices.emplace_back(branch_last_start_index);

    std::vector<int> fruit_start_indices;
    int fruit_last_start_index = 0;
    fruit_start_indices.emplace_back(fruit_last_start_index);

    std::vector<int> leaf_start_indices;
    int leaf_last_start_index = 0;
    leaf_start_indices.emplace_back(leaf_last_start_index);

    if (tree_entities->empty()) {
      shoot_stem_segments_.clear();
      shoot_stem_points_.clear();

      foliage_matrices_->SetParticleInfos({});
      fruit_matrices_->SetParticleInfos({});
    }
    std::vector<ParticleInfo> bounding_box_matrices;
    for (int list_index = 0; list_index < tree_entities->size(); list_index++) {
      auto tree_entity = tree_entities->at(list_index);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      auto& tree_model = tree->tree_model;
      const auto& branch_skeleton = tree_model.RefShootSkeleton();
      const auto& branch_list = branch_skeleton.PeekSortedFlowList();

      auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
      auto& [instanceMatrix, instanceColor] = bounding_box_matrices.emplace_back();
      instanceMatrix.value =
          entity_global_transform.value * (glm::translate((branch_skeleton.max + branch_skeleton.min) / 2.0f) *
                                           glm::scale(branch_skeleton.max - branch_skeleton.min));
      instanceColor = glm::vec4(random_colors_[list_index], 0.05f);
      if (tree_entity != selected_tree) {
        branch_last_start_index += branch_list.size();
        branch_start_indices.emplace_back(branch_last_start_index);

        fruit_last_start_index += tree_model.GetFruitCount();
        fruit_start_indices.emplace_back(fruit_last_start_index);

        leaf_last_start_index += tree_model.GetLeafCount();
        leaf_start_indices.emplace_back(leaf_last_start_index);
      } else {
        branch_start_indices.emplace_back(branch_last_start_index);
        fruit_start_indices.emplace_back(fruit_last_start_index);
        leaf_start_indices.emplace_back(leaf_last_start_index);
      }
    }

    bounding_box_matrices_->SetParticleInfos(bounding_box_matrices);

    shoot_stem_segments_.resize(3 * branch_last_start_index);
    shoot_stem_points_.resize(6 * branch_last_start_index);

    {
      std::vector<ParticleInfo> foliage_matrices;
      std::vector<ParticleInfo> fruit_matrices;
      foliage_matrices.resize(leaf_last_start_index);
      fruit_matrices.resize(fruit_last_start_index);
      Jobs::RunParallelFor(tree_entities->size(), [&](unsigned tree_index) {
        auto tree_entity = tree_entities->at(tree_index);
        if (tree_entity == selected_tree)
          return;
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        auto& tree_model = tree->tree_model;
        const auto& branch_skeleton = tree_model.RefShootSkeleton();
        const auto& branch_flow_list = branch_skeleton.PeekSortedFlowList();
        const auto& internode_list = branch_skeleton.PeekSortedNodeList();
        auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
        auto branch_start_index = branch_start_indices[tree_index];
        for (int i = 0; i < branch_flow_list.size(); i++) {
          auto& flow = branch_skeleton.PeekFlow(branch_flow_list[i]);
          auto cp1 = flow.info.global_start_position;
          auto cp4 = flow.info.global_end_position;
          float distance = glm::distance(cp1, cp4);
          glm::vec3 cp0, cp2;
          if (flow.GetParentHandle() > 0) {
            cp0 = cp1 + branch_skeleton.PeekFlow(flow.GetParentHandle()).info.global_end_rotation * glm::vec3(0, 0, 1) *
                            distance / 3.0f;
            cp2 = cp1 + branch_skeleton.PeekFlow(flow.GetParentHandle()).info.global_end_rotation *
                            glm::vec3(0, 0, -1) * distance / 3.0f;
          } else {
            cp0 = cp1 + flow.info.global_start_rotation * glm::vec3(0, 0, 1) * distance / 3.0f;
            cp2 = cp1 + flow.info.global_start_rotation * glm::vec3(0, 0, -1) * distance / 3.0f;
          }
          auto cp3 = cp4 + flow.info.global_end_rotation * glm::vec3(0, 0, 1) * distance / 3.0f;
          auto cp5 = cp4 + flow.info.global_end_rotation * glm::vec3(0, 0, -1) * distance / 3.0f;

          auto& p0 = shoot_stem_points_[branch_start_index * 6 + i * 6];
          auto& p1 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 1];
          auto& p2 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 2];
          auto& p3 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 3];
          auto& p4 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 4];
          auto& p5 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 5];
          p0.position = (entity_global_transform.value * glm::translate(cp0))[3];
          p1.position = (entity_global_transform.value * glm::translate(cp1))[3];
          p2.position = (entity_global_transform.value * glm::translate(cp2))[3];
          p3.position = (entity_global_transform.value * glm::translate(cp3))[3];
          p4.position = (entity_global_transform.value * glm::translate(cp4))[3];
          p5.position = (entity_global_transform.value * glm::translate(cp5))[3];
          if (flow.GetParentHandle() > 0) {
            p1.thickness = branch_skeleton.PeekFlow(flow.GetParentHandle()).info.end_thickness * 0.5f;
          } else {
            p1.thickness = flow.info.start_thickness * 0.5f;
          }
          p4.thickness = flow.info.end_thickness * 0.5f;

          p2.thickness = p3.thickness = (p1.thickness + p4.thickness) * 0.5f;
          p0.thickness = 2.0f * p1.thickness - p2.thickness;
          p5.thickness = 2.0f * p4.thickness - p3.thickness;

          p0.color = glm::vec4(random_colors_[flow.data.order], 1.0f);
          p1.color = glm::vec4(random_colors_[flow.data.order], 1.0f);
          p2.color = glm::vec4(random_colors_[flow.data.order], 1.0f);
          p3.color = glm::vec4(random_colors_[flow.data.order], 1.0f);
          p4.color = glm::vec4(random_colors_[flow.data.order], 1.0f);
          p5.color = glm::vec4(random_colors_[flow.data.order], 1.0f);

          shoot_stem_segments_[branch_start_index * 3 + i * 3] = branch_start_index * 6 + i * 6;
          shoot_stem_segments_[branch_start_index * 3 + i * 3 + 1] = branch_start_index * 6 + i * 6 + 1;
          shoot_stem_segments_[branch_start_index * 3 + i * 3 + 2] = branch_start_index * 6 + i * 6 + 2;
        }
        auto leaf_start_index = leaf_start_indices[tree_index];
        auto fruit_start_index = fruit_start_indices[tree_index];

        int leaf_index = 0;
        int fruit_index = 0;

        for (const auto& internode_handle : internode_list) {
          const auto& internode = branch_skeleton.PeekNode(internode_handle);
          const auto& internode_data = internode.data;

          for (const auto& bud : internode_data.buds) {
            if (bud.status != BudStatus::Died)
              continue;
            if (bud.reproductive_module.maturity <= 0.0f)
              continue;

            if (bud.type == BudType::Leaf) {
              foliage_matrices[leaf_start_index + leaf_index].instance_matrix.value =
                  entity_global_transform.value * bud.reproductive_module.transform;
              foliage_matrices[leaf_start_index + leaf_index].instance_color = glm::vec4(
                  glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                           glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - bud.reproductive_module.health),
                  1.0f);

              leaf_index++;
            } else if (bud.type == BudType::Fruit) {
              fruit_matrices[fruit_start_index + fruit_index].instance_matrix.value =
                  entity_global_transform.value * bud.reproductive_module.transform;
              fruit_matrices[fruit_start_index + fruit_index].instance_color =
                  glm::vec4(255 / 255.0f, 165 / 255.0f, 0 / 255.0f, 1.0f);

              fruit_index++;
            }
          }
        }
      });
      StrandPointAttributes strand_point_attributes{};
      strand_point_attributes.normal = false;
      branch_strands->SetSegments(strand_point_attributes, shoot_stem_segments_, shoot_stem_points_);
      foliage_matrices_->SetParticleInfos(foliage_matrices);
      fruit_matrices_->SetParticleInfos(fruit_matrices);
    }
  }
}

void EcoSysLabLayer::ClearGroundFruitAndLeaf() {
  fruits_.clear();
  leaves_.clear();
  UpdateGroundFruitAndLeaves();
}

void EcoSysLabLayer::UpdateGroundFruitAndLeaves() const {
  std::vector<ParticleInfo> fruit_matrices;
  fruit_matrices.resize(fruits_.size());
  for (int i = 0; i < fruits_.size(); i++) {
    fruit_matrices[i].instance_matrix.value = fruits_[i].global_transform.value;
    fruit_matrices[i].instance_color = glm::vec4(255 / 255.0f, 165 / 255.0f, 0 / 255.0f, 1.0f);
  }

  std::vector<ParticleInfo> leaf_matrices;
  leaf_matrices.resize(leaves_.size());
  for (int i = 0; i < leaves_.size(); i++) {
    leaf_matrices[i].instance_matrix.value = leaves_[i].global_transform.value;
    leaf_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                           glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - leaves_[i].m_health),
                  1.0f);
  }
  ground_fruit_matrices_->SetParticleInfos(fruit_matrices);
  ground_leaf_matrices_->SetParticleInfos(leaf_matrices);
}

void EcoSysLabLayer::SoilVisualization() {
  std::shared_ptr<Soil> soil;
  if (const auto soil_candidate = FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();

  if (!soil)
    return;

  const auto& soil_model = soil->soil_model;
  if (soil_version_ != soil_model.m_version) {
    update_vector_matrices_ = true;
    update_scalar_matrices_ = true;
    soil_version_ = soil_model.m_version;
  }

  if (vector_enable_) {
    SoilVisualizationVector(soil_model);
  }
  if (scalar_enable_) {
    SoilVisualizationScalar(soil_model);
  }
}

void EcoSysLabLayer::SoilVisualizationScalar(const VoxelSoilModel& soil_model) {
  const auto num_voxels = soil_model.m_resolution.x * soil_model.m_resolution.y * soil_model.m_resolution.z;
  if (update_scalar_matrices_) {
    std::vector<ParticleInfo> particle_infos;
    particle_infos.resize(num_voxels);
    Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
      const auto coordinate = soil_model.GetCoordinateFromIndex(i);
      if (static_cast<float>(coordinate.x) / soil_model.m_resolution.x < soil_cutout_x_depth_ ||
          static_cast<float>(coordinate.z) / soil_model.m_resolution.z > (1.0f - soil_cutout_z_depth_)) {
        particle_infos[i].instance_matrix.value = glm::mat4(0.0f);
      } else {
        particle_infos[i].instance_matrix.value = glm::translate(soil_model.GetPositionFromCoordinate(coordinate)) *
                                                  glm::mat4_cast(glm::quat(glm::vec3(0.0f))) *
                                                  glm::scale(glm::vec3(soil_model.GetVoxelSize() * scalar_box_size_));
      }
    });
    auto visualize_vec3 = [&](const Field& x, const Field& y, const Field& z) {
      Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
        const auto value = glm::vec3(x[i], y[i], z[i]);
        particle_infos[i].instance_color = {
            glm::normalize(value), glm::clamp(glm::length(value) * scalar_multiplier_, scalar_min_alpha_, 1.0f)};
      });
    };

    auto visualize_float = [&](const Field& v) {
      Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
        const auto value = glm::vec3(v[i]);
        particle_infos[i].instance_color = {
            scalar_base_color_, glm::clamp(glm::length(value) * scalar_multiplier_, scalar_min_alpha_, 1.0f)};
      });
    };

    switch (static_cast<SoilProperty>(scalar_soil_property_)) {
      case SoilProperty::Blank: {
        Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
          particle_infos[i].instance_color = {scalar_base_color_, 0.01f};
        });
      } break;
      case SoilProperty::WaterDensity: {
        visualize_float(soil_model.m_w);
      } break;
      case SoilProperty::NutrientDensity: {
        visualize_float(soil_model.m_n);
      } break;
      case SoilProperty::SoilDensity: {
        visualize_float(soil_model.m_d);
      } break;
      case SoilProperty::SoilLayer: {
        Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
          const auto layerIndex = soil_model.m_material_id[i];
          if (layerIndex == 0)
            particle_infos[i].instance_color = glm::vec4(0.0f);
          else {
            particle_infos[i].instance_color = soil_layer_colors_[layerIndex - 1];
          }
        });
      } break;
        /*case SoilProperty::DiffusionDivergence:
        {
                visualize_vec3(soilModel.m_div_diff_x, soilModel.m_div_diff_y, soilModel.m_div_diff_z);
        }break;*/
      default: {
        Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
          particle_infos[i].instance_color = {scalar_base_color_, 0.01f};
        });
      } break;
    }
    ground_fruit_matrices_->SetParticleInfos(particle_infos);
  }
  update_scalar_matrices_ = false;
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  gizmo_settings.draw_settings.blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), scalar_matrices_,
                                              glm::mat4(1.0f), 1.0f, gizmo_settings);
}

void EcoSysLabLayer::SoilVisualizationVector(const VoxelSoilModel& soil_model) {
  const auto num_voxels = soil_model.m_resolution.x * soil_model.m_resolution.y * soil_model.m_resolution.z;

  if (update_vector_matrices_) {
    std::vector<ParticleInfo> particle_infos;
    particle_infos.resize(num_voxels);

    const auto actual_vector_multiplier = vector_multiplier_ * soil_model.m_dx;
    switch (static_cast<SoilProperty>(vector_soil_property_)) {
        /*
        case SoilProperty::WaterDensityGradient:
        {
                Jobs::ParallelFor(numVoxels, [&](unsigned i)
                        {
                                const auto targetVector = glm::vec3(soilModel.m_w_grad_x[i], soilModel.m_w_grad_y[i],
        soilModel.m_w_grad_z[i]); const auto start =
        soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)); const auto end = start + targetVector
        * actualVectorMultiplier; const auto direction = glm::normalize(end - start); glm::quat rotation =
        glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x)); rotation *=
        glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f)); const auto length = glm::distance(end, start) / 2.0f;
                                const auto width = glm::min(vector_line_max_width_, length * vector_line_width_factor_);
                                const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
                                        glm::scale(glm::vec3(width, length, width));
                                particleInfos[i] = model;
                        }, results);
        }break;*/
        /*
        case SoilProperty::Divergence:
        {
                Jobs::ParallelFor(numVoxels, [&](unsigned i)
                        {
                                const auto targetVector = glm::vec3(soilModel.m_div_diff_x[i],
        soilModel.m_div_diff_y[i], soilModel.m_div_diff_z[i]); const auto start =
        soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)); const auto end = start + targetVector
        * actualVectorMultiplier; const auto direction = glm::normalize(end - start); glm::quat rotation =
        glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x)); rotation *=
        glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f)); const auto length = glm::distance(end, start) / 2.0f;
                                const auto width = glm::min(vector_line_max_width_, length * vector_line_width_factor_);
                                const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
                                        glm::scale(glm::vec3(width, length, width));
                                particleInfos[i] = model;
                        }, results);
        }break;
        */
      default: {
        Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
          particle_infos[i].instance_matrix.value =
              glm::translate(soil_model.GetPositionFromCoordinate(soil_model.GetCoordinateFromIndex(i))) *
              glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(0.0f));
        });
      } break;
    }
    Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
      particle_infos[i].instance_color = vector_base_color_;
    });

    ground_fruit_matrices_->SetParticleInfos(particle_infos);
    update_vector_matrices_ = false;
  }

  const auto editor_layer = Application::GetLayer<EditorLayer>();
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  gizmo_settings.draw_settings.blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;

  editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), vector_matrices_,
                                              glm::mat4(1.0f), 1.0f, gizmo_settings);
}

float EcoSysLabLayer::GetSimulatedTime() const {
  return simulated_time_;
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
        if (mesh_generator_settings.enable_branch) {
          unsigned tree_index = 0;
          for (const auto& entity : *tree_entities) {
            const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
            const auto mesh = tree->GenerateBranchMesh(mesh_generator_settings);
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

glm::vec2 EcoSysLabLayer::GetMouseSceneCameraPosition() const {
  return visualization_camera_mouse_position;
}

void EcoSysLabLayer::Simulate(const SimulationSettings& target_simulation_settings) {
  const auto scene = GetScene();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  simulated_time_ += target_simulation_settings.delta_time;
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
      return;
    }
    if (!climate) {
      EVOENGINE_ERROR("Simulation Failed! No climate in scene!");
      return;
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
      tree->crown_shyness_distance = target_simulation_settings.crown_shyness_distance;
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
          tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= target_simulation_settings.max_node_count)
        return;
      grown_stat[thread_index] = tree->TryGrow(target_simulation_settings.delta_time, true);
    });

    auto height_field = soil->soil_descriptor.Get<SoilDescriptor>()->height_field.Get<HeightField>();
    for (const auto& tree_entity : *tree_entities) {
      if (!scene->IsEntityEnabled(tree_entity))
        continue;
      auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      auto tree_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
      if (!tree->IsEnabled())
        continue;
      // Collect fruit and leaves here.
      if (!target_simulation_settings.auto_clear_fruit_and_leaves) {
        for (const auto& fruit : tree->tree_model.RefShootSkeleton().data.dropped_fruits) {
          Fruit new_fruit;
          new_fruit.global_transform.value = tree_global_transform.value * fruit.transform;

          auto position = new_fruit.global_transform.GetPosition();
          const auto ground_height = height_field->GetValue({position.x, position.z});
          const auto height = position.y - ground_height;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = ground_height + 0.1f;
          new_fruit.global_transform.SetPosition(position);

          new_fruit.m_maturity = fruit.maturity;
          new_fruit.m_health = fruit.health;
          fruits_.emplace_back(new_fruit);
        }

        for (const auto& leaf : tree->tree_model.RefShootSkeleton().data.dropped_leaves) {
          Leaf new_leaf;
          new_leaf.global_transform.value = tree_global_transform.value * leaf.transform;

          auto position = new_leaf.global_transform.GetPosition();
          const auto ground_height = height_field ? height_field->GetValue({position.x, position.z}) : 0.0f;
          const auto height = position.y - ground_height;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = ground_height + 0.1f;
          new_leaf.global_transform.SetPosition(position);

          new_leaf.m_maturity = leaf.maturity;
          new_leaf.m_health = leaf.health;
          leaves_.emplace_back(new_leaf);
        }
        tree->tree_visualizer.m_needUpdate = true;
      }
      tree->tree_model.RefShootSkeleton().data.dropped_fruits.clear();
      tree->tree_model.RefShootSkeleton().data.dropped_leaves.clear();
    }
    last_used_time_ = Times::Now() - time;
    total_time_ += last_used_time_;
    bool tree_grown = false;
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
        auto& tree_model = tree->tree_model;
        total_internode_size += tree_model.RefShootSkeleton().PeekSortedNodeList().size();
        total_flow_size += tree_model.RefShootSkeleton().PeekSortedFlowList().size();
        total_leaf_size += tree_model.GetLeafCount();
        total_fruit_size += tree_model.GetFruitCount();
      }
      internode_size_ = total_internode_size;
      shoot_stem_size_ = total_flow_size;
      root_node_size_ = total_root_node_size;
      root_stem_size_ = total_root_flow_size;
      leaf_size_ = total_leaf_size;
      fruit_size_ = total_fruit_size;
    }
  }
}

void EcoSysLabLayer::Simulate() {
  Simulate(simulation_settings);
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
        tree->GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"),
                                    Resources::GetResource<Mesh>("PRIMITIVE_CUBE"));
    }
  }
}

void EcoSysLabLayer::GenerateStrandModelProfiles() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->BuildStrandModel();
    }
  }
}

void EcoSysLabLayer::GenerateStrandModelMeshes(
    const StrandModelMeshGeneratorSettings& target_strand_model_mesh_generator_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->InitializeStrandModelMeshRenderer(target_strand_model_mesh_generator_settings);
    }
  }
}

void EcoSysLabLayer::GenerateStrandRenderers() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      if (const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock(); tree->generate_mesh)
        tree->InitializeStrandRenderer();
    }
  }
}

void EcoSysLabLayer::ClearStrandRenderers() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearStrandRenderer();
    }
  }
}

void EcoSysLabLayer::ClearStrandModelMeshes() const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      tree_entities && !tree_entities->empty()) {
    const auto copied_entities = *tree_entities;
    for (auto tree_entity : copied_entities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      tree->ClearStrandModelMeshRenderer();
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

void EcoSysLabLayer::TreeVisualizerSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Display shoot stem", &display_shoot_stem);
  ImGui::Checkbox("Display fruits", &display_fruit);
  ImGui::Checkbox("Display foliage", &display_foliage);

  ImGui::Checkbox("Display ground fruit", &display_ground_fruit);
  ImGui::Checkbox("Display ground leaves", &display_ground_leaves);

  ImGui::Checkbox("Display Soil", &display_soil);

  ImGui::Checkbox("Display Bounding Box", &display_bounding_box);
  ImGui::Checkbox("Show Shadow Grid", &show_shadow_grid);
  ImGui::Checkbox("Show Lighting Direction Grid", &show_lighting_grid);
}

void EcoSysLabLayer::StrandVisualizerSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::DragFloat("Drag force multiplier", &drag_force_multiplier, 0.001f, 0.0f, 1.0f);
}

void EcoSysLabLayer::PreUpdate() {
  if (const auto editor_layer = Application::GetLayer<EditorLayer>(); !editor_layer)
    return;
  visualization_camera_->Resize({visualization_camera_resolution_x, visualization_camera_resolution_y});

  const auto scene = GetScene();
  const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();

  const auto for_each_dts_entity =
      [&](const std::function<void(const std::shared_ptr<DynamicTreeStrands>& dts)>& action) {
        if (dts_entities && !dts_entities->empty()) {
          for (const auto& i : *dts_entities) {
            const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(i).lock();
            action(dts);
          }
        }
      };
  for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
    if (dts->enable_physics)
      dts->PhysicsStep();
  });
}

void EcoSysLabLayer::StrandVisualization(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (!show_strands)
    return;
  const auto scene = GetScene();
  const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();

  const auto for_each_dts_entity =
      [&](const std::function<void(const std::shared_ptr<DynamicTreeStrands>& dts)>& action) {
        if (dts_entities && !dts_entities->empty()) {
          for (const auto& i : *dts_entities) {
            const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(i).lock();
            action(dts);
          }
        }
      };
  for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
    dts->box_selection_operator->enabled = false;
    dts->drag_operator->enabled = false;
  });
  static glm::vec2 strands_operator_start;
  static glm::vec2 strands_operator_current;

  enum MouseOperatorMode { Idle, Selecting, ConfirmSelection, Dragging };

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Plant Visual")) {
    if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
      const ImVec2 canvas_p0 = ImGui::GetWindowPos() + ImVec2(1, 0);  // ImDrawList API uses screen coordinates!
      const ImVec2 canvas_size = ImGui::GetWindowSize();              // Resize canvas to what's available
      const ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_size.x - 2, canvas_p0.y + canvas_size.y - 1);
      ImDrawList* draw_list = ImGui::GetWindowDrawList();
      // Draw border and background color
      // draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));
      draw_list->PushClipRect(canvas_p0, canvas_p1, true);
      if (visualization_camera_window_focused_ &&
          editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Hold &&
          editor_layer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Press) {
        static bool is_dragging_previously = false;
        bool mouse_drag = true;
        glm::vec2 mouse_valid_position =
            glm::clamp(visualization_camera_mouse_position, {0, 0},
                       {visualization_camera_resolution_x - 1, visualization_camera_resolution_y - 1});
        if (editor_layer->GetKey(GLFW_MOUSE_BUTTON_LEFT) != Input::KeyActionType::Hold) {
          mouse_drag = false;
        }
        const auto camera_rotation = editor_layer->GetSceneCameraRotation();
        const auto camera_position = editor_layer->GetSceneCameraPosition();
        const glm::vec3 camera_front = camera_rotation * glm::vec3(0, 0, -1);
        const glm::vec3 camera_up = camera_rotation * glm::vec3(0, 1, 0);
        const glm::vec3 camera_right = camera_rotation * glm::vec3(1, 0, 0);
        if (mouse_drag && !is_dragging_previously) {
          strands_operator_start = mouse_valid_position;
        }
        if (editor_layer->GetKey(GLFW_KEY_E) == Input::KeyActionType::Hold) {
          if (mouse_drag) {
            strands_operator_current = mouse_valid_position;
            draw_list->AddLine(canvas_p0 + ImVec2(strands_operator_start.x, strands_operator_start.y),
                               canvas_p0 + ImVec2(strands_operator_current.x, strands_operator_current.y),
                               IM_COL32(255, 255, 255, 255));
            const auto screen_vector = strands_operator_current - strands_operator_start;
            const float line_distance = glm::length(screen_vector);
            draw_list->AddCircle(canvas_p0 + ImVec2(strands_operator_current.x, strands_operator_current.y),
                                 line_distance * 0.05f,
                                 IM_COL32(255, 255, 255, 255));
            draw_list->AddCircle(canvas_p0 + ImVec2(strands_operator_start.x, strands_operator_start.y),
                                 5.0f, IM_COL32(255, 255, 255, 255));
            
            const glm::vec3 force = strand_visualizer_settings_.drag_force_multiplier * 0.001f *
                                    (camera_right * screen_vector.x - camera_up * screen_vector.y);
            for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
              dts->drag_operator->enabled = true;
              dts->drag_operator->Update(force);
            });
          }
        } else {
          const auto camera_projection_view = visualization_camera_->GetProjection() *
                                              glm::lookAt(camera_position, camera_position + camera_front, camera_up);
          if (mouse_drag) {
            strands_operator_current = mouse_valid_position;
            draw_list->AddQuad(canvas_p0 + ImVec2(strands_operator_start.x, strands_operator_start.y),
                               canvas_p0 + ImVec2(strands_operator_start.x, strands_operator_current.y),
                               canvas_p0 + ImVec2(strands_operator_current.x, strands_operator_current.y),
                               canvas_p0 + ImVec2(strands_operator_current.x, strands_operator_start.y),
                               IM_COL32(255, 255, 255, 255));
            for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
              dts->box_selection_operator->enabled = true;
              dts->box_selection_operator->Update(
                  strands_operator_start / glm::vec2(canvas_size.x, canvas_size.y),
                  strands_operator_current / glm::vec2(canvas_size.x, canvas_size.y), camera_projection_view,
                  editor_layer->GetKey(GLFW_KEY_R) != Input::KeyActionType::Hold ? 0 : 1);
            });
          } else if (is_dragging_previously) {
            for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
              dts->box_selection_operator->enabled = true;
              dts->box_selection_operator->Update(
                  strands_operator_start / glm::vec2(canvas_size.x, canvas_size.y),
                  strands_operator_current / glm::vec2(canvas_size.x, canvas_size.y), camera_projection_view,
                  editor_layer->GetKey(GLFW_KEY_R) != Input::KeyActionType::Hold ? 2 : 3);
            });
          }
          if (editor_layer->GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
            for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
              dts->box_selection_operator->enabled = true;
              dts->box_selection_operator->Update(strands_operator_start / glm::vec2(canvas_size.x, canvas_size.y),
                                                  strands_operator_current / glm::vec2(canvas_size.x, canvas_size.y),
                                                  {}, 4);
            });
          }
        }
        is_dragging_previously = mouse_drag;
      }

      draw_list->PopClipRect();
    }
    ImGui::EndChild();
  }
  ImGui::End();
  ImGui::PopStyleVar();
  for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
    dts->Visualization(visualization_camera_);
  });
}
