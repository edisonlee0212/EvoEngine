//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#ifdef CUDA_MODULE_PLUGIN
#  include <RayTracerLayer.hpp>
#endif
#include "Times.hpp"
#ifdef BILLBOARD_CLOUDS_PLUGIN
#  include "BillboardCloudsConverter.hpp"
#endif
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "ForestDescriptor.hpp"
#include "Prefab.hpp"
#include "Shader.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"
using namespace eco_sys_lab_plugin;

PrivateComponentRegistration<TreeStructor> tree_structor_registry("TreeStructor");
PrivateComponentRegistration<Climate> climate_registry("Climate");

PrivateComponentRegistration<SpatialPlantDistributionSimulator> spds_registry("SpatialPlantDistributionSimulator");
PrivateComponentRegistration<DynamicTreeSkeleton> dynamic_tree_skeleton_registry("DynamicTreeSkeleton");

AssetRegistration<ClimateDescriptor> climate_d_registry("ClimateDescriptor", {".climate"});
AssetRegistration<RadialBoundingVolume> rbv_registry("RadialBoundingVolume", {".rbv"});
AssetRegistration<CubeVolume> cube_volume_registry("CubeVolume", {".cubevolume"});

AssetRegistration<ForestPatch> forest_patch_registry("ForestPatch", {".forestpatch"});

PrivateComponentRegistration<DynamicStrandsDemo> dynamic_strands_demo_registry("DynamicStrandsDemo");

#ifdef BILLBOARD_CLOUDS_PLUGIN
PrivateComponentRegistration<BillboardCloudsConverter> billboard_clouds_converter_register("BillboardCloudsConverter");
#endif

void EcoSysLabLayer::OnCreate() {
  Shader::RegisterShaderIncludePath(std::filesystem::path("./EcoSysLabResources/Shaders/Includes"));
  if (random_colors_.empty()) {
    for (int i = 0; i < 20000; i++) {
      random_colors_.emplace_back(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
    }
  }

  shoot_stem_strands_ = AssetManager::CreateTemporaryAsset<Strands>();
  soil_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  bounding_box_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  foliage_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  flower_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  fruit_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();

  ground_fruit_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  ground_flower_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  ground_leaf_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  vector_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  scalar_matrices_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  shadow_grid_particle_info_list_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  lighting_grid_particle_info_list_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
#pragma region Internode camera
  visualization_camera_ = Serialization::ProduceSerializable<Camera>();

  visualization_camera_->OnCreate();
  visualization_camera_->camera_settings.use_clear_color = true;
  visualization_camera_->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
#pragma endregion

  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    editor_layer->RegisterEditorCamera(visualization_camera_);
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

const std::vector<glm::vec3>& EcoSysLabLayer::RandomColors() {
  return random_colors_;
}

void EcoSysLabLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto scene = GetScene();
  bool simulate = false;
  static bool auto_time_grow = false;
  static float target_time = 0.0f;
  static float extra_time = 4.f;
  visualization_camera_->Resize({visualization_camera_resolution_x, visualization_camera_resolution_y});

  ImGui::Checkbox("Show Trees", &tree_visualization_settings_.enable);
  if (tree_visualization_settings_.enable) {
    const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (ImGui::TreeNodeEx("Tree settings")) {
      if (tree_entities && !tree_entities->empty()) {
        if (scene->IsEntityValid(selected_tree)) {
          const auto& tree = scene->GetOrSetPrivateComponent<Tree>(selected_tree).lock();
          auto& shoot_visualizer = tree->shoot_visualizer;
          if (shoot_visualizer.checkpoint_iteration == tree->shoot_model.CurrentIteration()) {
            if (ImGui::TreeNodeEx("Tree Operator", ImGuiTreeNodeFlags_DefaultOpen)) {
              if (ImGui::Combo("Mode", {"None", "Select", "Rotate", "Prune", "Invigorate", "Reduce"},
                               tree_operator_mode)) {
                shoot_visualizer.selected_node_handle = -1;
                shoot_visualizer.selected_node_hierarchy_list.clear();
              }
              switch (static_cast<TreeOperatorMode>(tree_operator_mode)) {
                case TreeOperatorMode::Select:
                  ImGui::Text("Press T to cut off entire node, press R to cut at point of selection.");
                  break;
                case TreeOperatorMode::Rotate:
                  ImGui::Text("Press T to cut off entire node.");
                  break;
                default:
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
            shoot_visualizer.OnInspect(tree->shoot_model);
            ImGui::TreePop();
          }
        } else {
          ImGui::Text("Select a tree entity to enable editing & visualization!");
        }
        if (!simulation_settings.auto_clear_fruit_and_leaves && ImGui::Button("Clear ground leaves and fruits")) {
          ClearGroundFruitAndLeaf();
        }
        if (ImGui::TreeNode("Tree Geometries")) {
          if (ImGui::TreeNode("Skeletal graph")) {
            skeletal_graph_settings.OnInspect(editor_layer);
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
        simulation_stats.OnInspect(editor_layer);

        if (ImGui::TreeNodeEx("Tree Visualization settings")) {
          if (ImGui::Button("Update")) {
            need_full_flow_update = true;
          }
          tree_visualization_settings_.OnInspect(editor_layer);
          ImGui::TreePop();
        }
      } else {
        ImGui::Text("No trees in the scene!");
        ResetAllTrees(nullptr);
        target_time = 0.0f;
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Tree Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (tree_entities && !tree_entities->empty()) {
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
        ImGui::Text(("Simulated time: " + std::to_string(simulated_time_ / 365.f) + " years").c_str());
        ImGui::DragFloat("Target years", &extra_time, 0.1f, simulated_time_ / 365.f, 999);
        if (auto_time_grow) {
          if (ImGui::Button("Force stop")) {
            auto_time_grow = false;
            target_time = simulated_time_;
          }
        } else {
          if (ImGui::Button(("Grow " + std::to_string(extra_time) + " years").c_str())) {
            auto_time_grow = true;
            target_time += extra_time * 365.f;
          }
        }
        if (ImGui::Button("Grow 1 iteration")) {
          simulate = true;
        }
      } else {
        ImGui::Text("No trees in the scene!");
        ResetAllTrees(nullptr);
        target_time = 0.0f;
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Fungus Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::DragFloat("Fungus growth rate", &dynamic_strands_settings_.physics_parameters.fungus_growth_rate,
                           0.01f, 0.0f, 1.0f)) {
        // need_full_flow_update = true;
        // TODO: do we need to set a variable here?
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Soil visualization settings")) {
      soil_visualization_settings_.OnInspect(editor_layer);
      ImGui::TreePop();
    }
  }
  if (ImGui::TreeNodeEx("Dynamic Strands settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Initialize all")) {
      GenerateDynamicStrandsForAllTrees();
    }
    if (ImGui::Button("Refresh meshes")) {
      RefreshMeshForAllDynamicStrands();
    }
    if (const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
        dts_entities && !dts_entities->empty()) {
      dynamic_strands_settings_.OnInspect(editor_layer);
    } else {
      ImGui::Text("No dynamic strands in the scene!");
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Dynamic Skeleton settings")) {
    if (ImGui::Button("Initialize dynamic skeleton for all trees")) {
      GenerateDynamicSkeletonForAllTrees();
    }
    dynamic_skeleton_settings_.OnInspect(editor_layer);
    ImGui::TreePop();
  }

  if (simulate || auto_time_grow) {
    Simulate();
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

      VisualizationCameraDragAndDrop();

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
            editor_layer->SetSceneCameraRotation(scene_camera_rotation);
            editor_layer->SetSceneCameraPosition(scene_camera_position);
          }
#pragma endregion
        }
      } else {
        visualization_camera_window_focused_ = false;
      }
      editor_layer->RefEditorCameraRotation(visualization_camera_->GetHandle()) = scene_camera_rotation;
      editor_layer->RefEditorCameraPosition(visualization_camera_->GetHandle()) = scene_camera_position;
    }
    ImGui::EndChild();
    auto* window = ImGui::FindWindowByName("Plant Visual");
    visualization_camera_->SetEnabled(!(window->Hidden && !window->Collapsed));
  }
  ImGui::End();
  ImGui::PopStyleVar();
  if (const auto selected_entity = editor_layer->GetSelectedEntity(); selected_entity != selected_tree) {
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
      auto& shoot_visualizer = tree->shoot_visualizer;
      shoot_visualizer.selected_node_handle = -1;
      shoot_visualizer.selected_node_hierarchy_list.clear();
      tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Select);
    }
  }
  if (tree_visualization_settings_.enable)
    TreeVisualization(editor_layer);
  if (dynamic_strands_settings_.enable_visualization)
    DynamicStrandsVisualization(editor_layer);
  if (soil_visualization_settings_.enable) {
    SoilVisualization();
  }
#pragma endregion
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

    std::vector<int> flower_start_indices;
    int flower_last_start_index = 0;
    flower_start_indices.emplace_back(flower_last_start_index);
    if (tree_entities->empty()) {
      shoot_stem_segments_.clear();
      shoot_stem_points_.clear();

      foliage_matrices_->SetParticleInfos({});
      fruit_matrices_->SetParticleInfos({});
      flower_matrices_->SetParticleInfos({});
    }
    std::vector<ParticleInfo> bounding_box_matrices;
    for (int list_index = 0; list_index < tree_entities->size(); list_index++) {
      auto tree_entity = tree_entities->at(list_index);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      auto& tree_model = tree->shoot_model;
      const auto& branch_skeleton = tree_model.RefShootSkeleton();
      const auto& branch_list = branch_skeleton.PeekSortedFlowList();

      auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
      auto& [instanceMatrix, instanceColor] = bounding_box_matrices.emplace_back();
      instanceMatrix.value =
          entity_global_transform.value * (glm::translate((branch_skeleton.max + branch_skeleton.min) / 2.0f) *
                                           glm::scale(branch_skeleton.max - branch_skeleton.min));
      instanceColor = glm::vec4(random_colors_[list_index], 0.05f);

      fruit_last_start_index += tree_model.GetFruitCount();
      fruit_start_indices.emplace_back(fruit_last_start_index);
      fruit_start_indices.emplace_back(fruit_last_start_index);

      leaf_last_start_index += tree_model.GetLeafCount();
      leaf_start_indices.emplace_back(leaf_last_start_index);
      leaf_start_indices.emplace_back(leaf_last_start_index);

      flower_last_start_index += tree_model.GetFlowerCount();
      flower_start_indices.emplace_back(flower_last_start_index);
      flower_start_indices.emplace_back(flower_last_start_index);

      if (tree_entity != selected_tree) {
        branch_last_start_index += branch_list.size();
        branch_start_indices.emplace_back(branch_last_start_index);
      } else {
        branch_start_indices.emplace_back(branch_last_start_index);
      }
    }

    bounding_box_matrices_->SetParticleInfos(bounding_box_matrices);

    shoot_stem_segments_.resize(3 * branch_last_start_index);
    shoot_stem_points_.resize(6 * branch_last_start_index);

    {
      std::vector<ParticleInfo> foliage_matrices;
      std::vector<ParticleInfo> flower_matrices;
      std::vector<ParticleInfo> fruit_matrices;
      foliage_matrices.resize(leaf_last_start_index);
      flower_matrices.resize(flower_last_start_index);
      fruit_matrices.resize(fruit_last_start_index);
      Jobs::RunParallelFor(tree_entities->size(), [&](unsigned tree_index) {
        auto tree_entity = tree_entities->at(tree_index);
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        auto& tree_model = tree->shoot_model;
        const auto& branch_skeleton = tree_model.RefShootSkeleton();
        const auto& branch_flow_list = branch_skeleton.PeekSortedFlowList();
        auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
        auto branch_start_index = branch_start_indices[tree_index];

        auto leaf_start_index = leaf_start_indices[tree_index];
        auto fruit_start_index = fruit_start_indices[tree_index];
        auto flower_start_index = flower_start_indices[tree_index];
        int leaf_index = 0;
        int fruit_index = 0;
        int flower_index = 0;

        const auto& sorted_internode_list = branch_skeleton.PeekSortedNodeList();
        for (const auto& internode_handle : sorted_internode_list) {
          const auto& internode_data = branch_skeleton.PeekNode(internode_handle).data;
          for (const auto& leaf : internode_data.leaves) {
            if (leaf.status != OrganStatus::Inactive) {
              glm::mat4 leaf_transform =
                  glm::translate(leaf.position) * glm::mat4_cast(leaf.rotation) * glm::scale(leaf.scale * .5f);
              foliage_matrices[leaf_start_index + leaf_index].instance_matrix.value =
                  entity_global_transform.value * leaf_transform;
              foliage_matrices[leaf_start_index + leaf_index].instance_color =
                  glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                                     glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - leaf.health),
                            0.5f);
              leaf_index++;
            }
          }

          for (const auto& flower : internode_data.flowers) {
            if (flower.status != OrganStatus::Inactive) {
              glm::mat4 flower_transform =
                  glm::translate(flower.position) * glm::mat4_cast(flower.rotation) * glm::scale(flower.scale);
              flower_matrices[flower_start_index + flower_index].instance_matrix.value =
                  entity_global_transform.value * flower_transform;
              flower_matrices[flower_start_index + flower_index].instance_color =
                  glm::vec4(glm::mix(glm::vec3(255 / 255.0f, 255 / 255.0f, 255 / 255.0f),
                                     glm::vec3(255 / 255.0f, 192 / 255.0f, 203 / 255.0f), flower.maturity),
                            0.75f);
              flower_index++;
            }
          }

          for (const auto& fruit : internode_data.fruits) {
            if (fruit.status != OrganStatus::Inactive) {
              glm::mat4 fruit_transform =
                  glm::translate(fruit.position) * glm::mat4_cast(fruit.rotation) * glm::scale(fruit.scale * .25f);
              fruit_matrices[fruit_start_index + fruit_index].instance_matrix.value =
                  entity_global_transform.value * fruit_transform;
              fruit_matrices[fruit_start_index + fruit_index].instance_color =
                  glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 255 / 255.0f, 152 / 255.0f),
                                     glm::vec3(255 / 255.0f, 165 / 255.0f, 0 / 255.0f), fruit.maturity),
                            0.75f);
              fruit_index++;
            }
          }
        }
        if (tree_entity == selected_tree)
          return;
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

          p0.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p1.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p2.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p3.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p4.color = glm::vec4(random_colors_[flow.info.order], 1.0f);
          p5.color = glm::vec4(random_colors_[flow.info.order], 1.0f);

          shoot_stem_segments_[branch_start_index * 3 + i * 3] = branch_start_index * 6 + i * 6;
          shoot_stem_segments_[branch_start_index * 3 + i * 3 + 1] = branch_start_index * 6 + i * 6 + 1;
          shoot_stem_segments_[branch_start_index * 3 + i * 3 + 2] = branch_start_index * 6 + i * 6 + 2;
        }
      });
      StrandPointAttributes strand_point_attributes{};
      strand_point_attributes.normal = false;
      branch_strands->SetSegments(strand_point_attributes, shoot_stem_segments_, shoot_stem_points_);
      foliage_matrices_->SetParticleInfos(foliage_matrices);
      fruit_matrices_->SetParticleInfos(fruit_matrices);
      flower_matrices_->SetParticleInfos(flower_matrices);
    }
  }
}

void EcoSysLabLayer::ClearGroundFruitAndLeaf() {
  fruits_.clear();
  leaves_.clear();
  flowers_.clear();
  UpdateGroundFruitAndLeaves();
}

void EcoSysLabLayer::UpdateGroundFruitAndLeaves() const {
  std::vector<ParticleInfo> fruit_matrices;
  fruit_matrices.resize(fruits_.size());
  for (int i = 0; i < fruits_.size(); i++) {
    fruit_matrices[i].instance_matrix.value = fruits_[i].global_transform.value;
    fruit_matrices[i].instance_matrix.SetScale(fruit_matrices[i].instance_matrix.GetScale() * 0.25f);
    fruit_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 255 / 255.0f, 152 / 255.0f),
                           glm::vec3(255 / 255.0f, 165 / 255.0f, 0 / 255.0f), fruits_[i].fruit_maturity),
                  0.75f);
  }
  std::vector<ParticleInfo> flower_matrices;
  flower_matrices.resize(flowers_.size());
  for (int i = 0; i < flowers_.size(); i++) {
    flower_matrices[i].instance_matrix.value = flowers_[i].global_transform.value;
    flower_matrices[i].instance_matrix.SetScale(flower_matrices[i].instance_matrix.GetScale() * 0.5f);
    flower_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(255 / 255.0f, 255 / 255.0f, 255 / 255.0f),
                           glm::vec3(255 / 255.0f, 192 / 255.0f, 203 / 255.0f), flowers_[i].flower_maturity),
                  0.75f);
  }
  std::vector<ParticleInfo> leaf_matrices;
  leaf_matrices.resize(leaves_.size());
  for (int i = 0; i < leaves_.size(); i++) {
    leaf_matrices[i].instance_matrix.value = leaves_[i].global_transform.value;
    leaf_matrices[i].instance_matrix.SetScale(leaf_matrices[i].instance_matrix.GetScale() * 0.5f);
    leaf_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                           glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - leaves_[i].leaf_health),
                  0.5f);
  }
  ground_fruit_matrices_->SetParticleInfos(fruit_matrices);
  ground_leaf_matrices_->SetParticleInfos(leaf_matrices);
  ground_flower_matrices_->SetParticleInfos(flower_matrices);
}

void EcoSysLabLayer::VisualizationCameraDragAndDrop() const {
  if (AssetRef asset_ref; EditorLayer::UnsafeDroppableAsset(asset_ref, {"Scene", "Prefab", "Mesh", "TreeDescriptor"})) {
    const auto scene = GetScene();
    if (const auto asset = asset_ref.Get<IAsset>(); asset->GetTypeName() == "TreeDescriptor") {
      std::dynamic_pointer_cast<TreeDescriptor>(asset)->Instantiate();
    }
  }
}

float EcoSysLabLayer::GetSimulatedTime() const {
  return simulated_time_;
}

glm::vec2 EcoSysLabLayer::GetMouseSceneCameraPosition() const {
  return visualization_camera_mouse_position;
}

void EcoSysLabLayer::Update() {
  if (const auto scene = GetScene(); !scene)
    return;
  RegisterStrandRenderingProcedure();
  DynamicSkeletonPhysics();
  DynamicStrandSimulation();
}

void EcoSysLabLayer::LateUpdate() {
  if (const auto scene = GetScene(); !scene)
    return;
  DynamicSkeletonVisualization();
  DynamicStrandVisualization();
}
