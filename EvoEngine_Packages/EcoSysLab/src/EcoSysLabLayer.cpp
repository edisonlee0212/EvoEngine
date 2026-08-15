//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "AdvancedShootDescriptor.hpp"
#include "Application.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFineRootDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicRootDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "Times.hpp"
#ifdef BILLBOARD_CLOUDS_PACKAGE
#  include "BillboardCloudsConverter.hpp"
#endif
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "DsColliders.hpp"
#include "DsOperators.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrandGraph.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "ForestDescriptor.hpp"
#include "HeightField.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RadialBoundingVolume.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "Soil.hpp"
#include "SoilDescriptor.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeStructor.hpp"

#include <algorithm>

using namespace eco_sys_lab_package;

void EcoSysLabLayer::OnDestroy() {
  DsFungusInjection::ReleaseStaticGpuResources();
  DynamicStrands::ReleaseStaticGpuResources();
}

namespace {
template <typename T>
void RegisterAssetPreviewHandler(const std::string& type_name) {
  evo_engine::Serialization::RegisterAssetPreviewHandler<T>(
      [](const std::shared_ptr<T>& asset, const evo_engine::OffscreenPreviewSettings&) {
        return asset ? asset->GenerateThumbnailTexture() : nullptr;
      },
      {}, type_name);
}

void RegisterEcoSysLabAssetPreviewHandlers() {
  RegisterAssetPreviewHandler<ClimateDescriptor>("ClimateDescriptor");
  RegisterAssetPreviewHandler<ForestPatch>("ForestPatch");
  RegisterAssetPreviewHandler<BasicBarkDescriptor>("BasicBarkDescriptor");
  RegisterAssetPreviewHandler<ForestDescriptor>("ForestDescriptor");
  RegisterAssetPreviewHandler<TreeDescriptor>("TreeDescriptor");
  RegisterAssetPreviewHandler<BasicPruningDescriptor>("BasicPruningDescriptor");
  RegisterAssetPreviewHandler<BasicShootDescriptor>("BasicShootDescriptor");
  RegisterAssetPreviewHandler<BasicRootDescriptor>("BasicRootDescriptor");
  RegisterAssetPreviewHandler<BasicFineRootDescriptor>("BasicFineRootDescriptor");
  RegisterAssetPreviewHandler<BasicReproductionModuleDescriptor>("BasicReproductionModuleDescriptor");
  RegisterAssetPreviewHandler<BasicFoliageDescriptor>("BasicFoliageDescriptor");
  RegisterAssetPreviewHandler<AdvancedShootDescriptor>("AdvancedShootDescriptor");
  RegisterAssetPreviewHandler<HeightField>("HeightField");
  RegisterAssetPreviewHandler<SoilDescriptor>("SoilDescriptor");
}

void RegisterEcoSysLabSerializationHandlers() {
  evo_engine::Serialization::RegisterSerializationHandler<TreeStructor>(SerializeTreeStructor, DeserializeTreeStructor,
                                                                        {}, "TreeStructor");
  evo_engine::Serialization::RegisterSerializationHandler<Climate>(SerializeClimate, DeserializeClimate, {}, "Climate");
  evo_engine::Serialization::RegisterSerializationHandler<SpatialPlantDistributionSimulator>(
      SerializeSpatialPlantDistributionSimulator, DeserializeSpatialPlantDistributionSimulator, {},
      "SpatialPlantDistributionSimulator");
  evo_engine::Serialization::RegisterSerializationHandler<DynamicTreeSkeleton>(
      SerializeDynamicTreeSkeleton, DeserializeDynamicTreeSkeleton, {}, "DynamicTreeSkeleton");
  evo_engine::Serialization::RegisterSerializationHandler<DynamicStrandsDemo>(
      SerializeDynamicStrandsDemo, DeserializeDynamicStrandsDemo, {}, "DynamicStrandsDemo");
  evo_engine::Serialization::RegisterSerializationHandler<Tree>(SerializeTree, DeserializeTree, {}, "Tree");
  evo_engine::Serialization::RegisterSerializationHandler<Soil>(SerializeSoil, DeserializeSoil, {}, "Soil");
  evo_engine::Serialization::RegisterSerializationHandler<DsBoxCollider>(SerializeDsBoxCollider,
                                                                         DeserializeDsBoxCollider, {}, "DsBoxCollider");
  evo_engine::Serialization::RegisterSerializationHandler<DsSphereCollider>(
      SerializeDsSphereCollider, DeserializeDsSphereCollider, {}, "DsSphereCollider");
  evo_engine::Serialization::RegisterSerializationHandler<DsCylinderCollider>(
      SerializeDsCylinderCollider, DeserializeDsCylinderCollider, {}, "DsCylinderCollider");
  evo_engine::Serialization::RegisterSerializationHandler<DynamicTreeStrands>(
      SerializeDynamicTreeStrands, DeserializeDynamicTreeStrands, {}, "DynamicTreeStrands");
  evo_engine::Serialization::RegisterSerializationHandler<ClimateDescriptor>(
      SerializeClimateDescriptor, DeserializeClimateDescriptor, {}, "ClimateDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<RadialBoundingVolume>(
      SerializeRadialBoundingVolume, DeserializeRadialBoundingVolume, {}, "RadialBoundingVolume");
  evo_engine::Serialization::RegisterSerializationHandler<CubeVolume>(SerializeCubeVolume, DeserializeCubeVolume, {},
                                                                      "CubeVolume");
  evo_engine::Serialization::RegisterSerializationHandler<ForestPatch>(SerializeForestPatch, DeserializeForestPatch, {},
                                                                       "ForestPatch");
  evo_engine::Serialization::RegisterSerializationHandler<BasicBarkDescriptor>(
      SerializeBasicBarkDescriptor, DeserializeBasicBarkDescriptor, {}, "BasicBarkDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<ForestDescriptor>(
      SerializeForestDescriptor, DeserializeForestDescriptor, {}, "ForestDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<TreeDescriptor>(
      SerializeTreeDescriptor, DeserializeTreeDescriptor, {}, "TreeDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicPruningDescriptor>(
      SerializeBasicPruningDescriptor, DeserializeBasicPruningDescriptor, {}, "BasicPruningDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicShootDescriptor>(
      SerializeBasicShootDescriptor, DeserializeBasicShootDescriptor, {}, "BasicShootDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicRootDescriptor>(
      SerializeBasicRootDescriptor, DeserializeBasicRootDescriptor, {}, "BasicRootDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicFineRootDescriptor>(
      SerializeBasicFineRootDescriptor, DeserializeBasicFineRootDescriptor, {}, "BasicFineRootDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicReproductionModuleDescriptor>(
      SerializeBasicReproductionModuleDescriptor, DeserializeBasicReproductionModuleDescriptor, {},
      "BasicReproductionModuleDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<BasicFoliageDescriptor>(
      SerializeBasicFoliageDescriptor, DeserializeBasicFoliageDescriptor, {}, "BasicFoliageDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<AdvancedShootDescriptor>(
      SerializeAdvancedShootDescriptor, DeserializeAdvancedShootDescriptor, {}, "AdvancedShootDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<ModulusGraph>(SerializeModulusGraph, DeserializeModulusGraph,
                                                                        {}, "ModulusGraph");
  evo_engine::Serialization::RegisterSerializationHandler<StrengthGraph>(SerializeStrengthGraph,
                                                                         DeserializeStrengthGraph, {}, "StrengthGraph");
  evo_engine::Serialization::RegisterSerializationHandler<BiologicalPropertiesGraph>(
      SerializeBiologicalPropertiesGraph, DeserializeBiologicalPropertiesGraph, {}, "TrunkGraph");
  evo_engine::Serialization::RegisterSerializationHandler<HeightField>(SerializeHeightField, DeserializeHeightField, {},
                                                                       "HeightField");
  evo_engine::Serialization::RegisterSerializationHandler<SoilLayerDescriptor>(
      SerializeSoilLayerDescriptor, DeserializeSoilLayerDescriptor, {}, "SoilLayerDescriptor");
  evo_engine::Serialization::RegisterSerializationHandler<SoilDescriptor>(
      SerializeSoilDescriptor, DeserializeSoilDescriptor, {}, "SoilDescriptor");
}
}  // namespace

void EcoSysLabLayer::RegisterTypes(Application& application) {
  application.RegisterPrivateComponent<TreeStructor>("TreeStructor");
  application.RegisterPrivateComponent<Climate>("Climate");
  application.RegisterPrivateComponent<SpatialPlantDistributionSimulator>("SpatialPlantDistributionSimulator");
  application.RegisterPrivateComponent<DynamicTreeSkeleton>("DynamicTreeSkeleton");
  application.RegisterPrivateComponent<DynamicStrandsDemo>("DynamicStrandsDemo");
  application.RegisterPrivateComponent<Tree>("Tree");
  application.RegisterPrivateComponent<Soil>("Soil");
  application.RegisterPrivateComponent<DsBoxCollider>("DsBoxCollider");
  application.RegisterPrivateComponent<DsSphereCollider>("DsSphereCollider");
  application.RegisterPrivateComponent<DsCylinderCollider>("DsCylinderCollider");
  application.RegisterPrivateComponent<DynamicTreeStrands>("DynamicTreeStrands");
#ifdef BILLBOARD_CLOUDS_PACKAGE
  application.RegisterPrivateComponent<BillboardCloudsConverter>("BillboardCloudsConverter");
#endif

  application.RegisterAsset<ClimateDescriptor>("ClimateDescriptor", {".climate"});
  application.RegisterAsset<RadialBoundingVolume>("RadialBoundingVolume", {".rbv"});
  application.RegisterAsset<CubeVolume>("CubeVolume", {".cubevolume"});
  application.RegisterAsset<ForestPatch>("ForestPatch", {".forestpatch"});
  application.RegisterAsset<BasicBarkDescriptor>("BasicBarkDescriptor", {".bark"});
  application.RegisterAsset<ForestDescriptor>("ForestDescriptor", {".forest"});
  application.RegisterAsset<TreeDescriptor>("TreeDescriptor", {".tree"});
  application.RegisterAsset<BasicPruningDescriptor>("BasicPruningDescriptor", {".pruning"});
  application.RegisterAsset<BasicShootDescriptor>("BasicShootDescriptor", {".shoot"});
  application.RegisterAsset<BasicRootDescriptor>("BasicRootDescriptor", {".root"});
  application.RegisterAsset<BasicFineRootDescriptor>("BasicFineRootDescriptor", {".froot"});
  application.RegisterAsset<BasicReproductionModuleDescriptor>("BasicReproductionModuleDescriptor", {".repro"});
  application.RegisterAsset<BasicFoliageDescriptor>("BasicFoliageDescriptor", {".foliage"});
  application.RegisterAsset<AdvancedShootDescriptor>("AdvancedShootDescriptor", {".ashoot"});
  application.RegisterAsset<ModulusGraph>("ModulusGraph", {".evemodulus"});
  application.RegisterAsset<StrengthGraph>("StrengthGraph", {".evestrength"});
  application.RegisterAsset<BiologicalPropertiesGraph>("TrunkGraph", {".evetrunk"});
  application.RegisterAsset<HeightField>("HeightField", {".heightfield"});
  application.RegisterAsset<SoilLayerDescriptor>("SoilLayerDescriptor", {".soillayer"});
  application.RegisterAsset<SoilDescriptor>("SoilDescriptor", {".soil"});
  RegisterEcoSysLabSerializationHandlers();
  RegisterEcoSysLabAssetPreviewHandlers();
}

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
  visualization_camera_->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
  visualization_camera_->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
#pragma endregion

  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    editor_layer->RegisterEditorCamera(visualization_camera_);
  }
}

std::weak_ptr<Climate> EcoSysLabLayer::FindClimate() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const std::vector<Entity>* climate_entities = scene->UnsafeGetPrivateComponentOwnersList<Climate>();
  if (climate_entities && !climate_entities->empty()) {
    return scene->GetOrSetPrivateComponent<Climate>(climate_entities->at(0));
  }
  return {};
}

const std::vector<glm::vec3>& EcoSysLabLayer::RandomColors() {
  return random_colors_;
}

void EcoSysLabLayer::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto scene = GetScene();
  if (!scene || !visualization_camera_) {
    return;
  }
  bool simulate = false;
  static bool auto_time_grow = false;
  static float target_time = 0.0f;
  static float extra_time = 4.f;
  visualization_camera_->Resize({visualization_camera_resolution_x, visualization_camera_resolution_y});

  const auto window_title = GetLayerName();
  bool open = enable_inspection;
  if (ImGui::Begin(window_title.c_str(), &open)) {
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
              shoot_visualizer.DrawGui(tree->shoot_model);
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
              skeletal_graph_settings.DrawGui(editor_layer);
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
              mesh_generator_settings.DrawGui(editor_layer);
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
              strand_mesh_generator_settings.DrawGui(editor_layer);
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
          simulation_stats.DrawGui(editor_layer);

          if (ImGui::TreeNodeEx("Tree Visualization settings")) {
            if (ImGui::Button("Update")) {
              need_full_flow_update = true;
            }
            tree_visualization_settings_.DrawGui(editor_layer);
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
            simulation_settings.DrawGui(editor_layer);
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
        soil_visualization_settings_.DrawGui(editor_layer);
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
        DrawDynamicStrandsSettingsGui(editor_layer);
      } else {
        ImGui::Text("No dynamic strands in the scene!");
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Dynamic Skeleton settings")) {
      if (ImGui::Button("Initialize dynamic skeleton for all trees")) {
        GenerateDynamicSkeletonForAllTrees();
      }
      dynamic_skeleton_settings_.DrawGui(editor_layer);
      ImGui::TreePop();
    }
  }
  ImGui::End();
  enable_inspection = open;

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
      const ImVec2 overlay_pos = ImGui::GetWindowPos();
      static int corner = 1;
      visualization_camera_resolution_x = view_port_size.x;
      visualization_camera_resolution_y = view_port_size.y;
      ImGui::Image(visualization_camera_->GetRenderTexture()->GetColorImTextureId(),
                   ImVec2(view_port_size.x, view_port_size.y), ImVec2(0, 1), ImVec2(1, 0));

      VisualizationCameraDragAndDrop();
      const auto window_pos = ImVec2((corner & 1) ? (overlay_pos.x + view_port_size.x) : (overlay_pos.x),
                                     (corner & 2) ? (overlay_pos.y + view_port_size.y) : (overlay_pos.y));
      if (show_visualization_camera_info) {
        const auto window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
        ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        ImGui::SetNextWindowBgAlpha(0.35f);
        constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                                                  ImGuiWindowFlags_NoSavedSettings |
                                                  ImGuiWindowFlags_NoFocusOnAppearing;
        if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_None;
            ImGui::BeginChild("Render Info", ImVec2(300, 150), child_flags, window_flags)) {
          ImGui::Text("Info & Settings");
          ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
          ImGui::Checkbox("Background", &enable_visualization_background);
          uint32_t mode = static_cast<uint32_t>(visualization_camera_->camera_render_mode);
          if (ImGui::Combo("Render Mode", {"Rasterization", "Ray Tracing"}, mode)) {
            visualization_camera_->camera_render_mode = static_cast<Camera::CameraRenderMode>(mode);
            visualization_camera_->ResetFrameCount();
          }
        }
        ImGui::EndChild();
      }
      visualization_camera_mouse_position = glm::vec2(FLT_MAX, -FLT_MAX);
      if (ImGui::IsWindowFocused()) {
        visualization_camera_window_focused_ = true;
        auto mp = ImGui::GetMousePos();
        auto wp = ImGui::GetWindowPos();
        visualization_camera_mouse_position = glm::vec2(mp.x - wp.x, mp.y - wp.y);
      } else {
        visualization_camera_window_focused_ = false;
      }
      if (const auto scene_camera = editor_layer->GetSceneCamera()) {
        editor_layer->ApplyEditorCameraFreeFlyControl(
            scene_camera->GetHandle(), visualization_camera_free_fly_state_, visualization_camera_mouse_position,
            {view_port_size.x, view_port_size.y}, visualization_camera_window_focused_);
        editor_layer->RefEditorCameraRotation(visualization_camera_->GetHandle()) =
            editor_layer->GetSceneCameraRotation();
        editor_layer->RefEditorCameraPosition(visualization_camera_->GetHandle()) =
            editor_layer->GetSceneCameraPosition();
      }
    }
    ImGui::EndChild();
    auto* window = ImGui::FindWindowByName("Plant Visual");
    if (!(window->Hidden && !window->Collapsed)) {
      if (enable_visualization_background) {
        visualization_camera_->SetRequireRendering(true);
      } else {
        visualization_camera_->SetRendered();
      }
    }
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

void EcoSysLabLayer::DrawDynamicStrandsSettingsGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::TreeNode("Operators")) {
    ImGui::Combo("Transform Mode", {"None", "Translate", "Rotate"}, dynamic_strands_settings_.transform_mode);
    ImGui::Combo("Operator Mode", {"Drag", "Saw", "Line Cut", "Point Cut", "Fungus Injection"},
                 dynamic_strands_settings_.operator_mode);
    switch (static_cast<DynamicStrandsSettings::OperatorMode>(dynamic_strands_settings_.operator_mode)) {
      case DynamicStrandsSettings::OperatorMode::Drag: {
        ImGui::DragFloat("Drag acceleration multiplier", &dynamic_strands_settings_.drag_multiplier, 0.001f, 0.0f,
                         1.0f);
        break;
      }
      case DynamicStrandsSettings::OperatorMode::Saw:
      case DynamicStrandsSettings::OperatorMode::LineCut: {
        ImGui::Checkbox("Cut Bend/Twist/Bundle only", &dynamic_strands_settings_.cut_bend_twist_bundle_only);
        break;
      }
      case DynamicStrandsSettings::OperatorMode::PointCut: {
        ImGui::DragFloat("Cutter thickness", &dynamic_strands_settings_.point_cut_thickness, 1.f, 1.0f, 100.0f);
        break;
      }
      case DynamicStrandsSettings::OperatorMode::FungusInjection: {
        ImGui::DragFloat("Injection thickness", &dynamic_strands_settings_.point_cut_thickness, 1.f, 1.0f, 100.0f);
        ImGui::DragFloat("Fungus injection amount", &dynamic_strands_settings_.fungus_injection_amount, 0.1f, 0.0f,
                         100.0f);
        // Check boxes for each rot type
        ImGui::Text("Rot types:");
        ImGui::Checkbox("White rot", &dynamic_strands_settings_.fungus_white_rot);
        ImGui::Checkbox("Brown rot", &dynamic_strands_settings_.fungus_brown_rot);
        break;
      }
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Physics", &dynamic_strands_settings_.enable_physics);
  if (!dynamic_strands_settings_.enable_physics && ImGui::Button("Physics step")) {
    dynamic_strands_settings_.remaining_step++;
  }
  if (ImGui::TreeNode("Physics settings")) {
    dynamic_strands_settings_.physics_parameters.DrawGui(editor_layer);
    ImGui::TreePop();
  }

  ImGui::Checkbox("Rendering", &dynamic_strands_settings_.enable_rendering);
  if (ImGui::TreeNode("Rendering settings")) {
    if (ImGui::TreeNode("Alpha Shape Meshing Settings")) {
      DsAlphaShapeMeshing::DrawRenderSettingsGui(editor_layer);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Kinetic Voronoi Meshing Settings")) {
      DsKineticVoronoiMeshing::DrawRenderSettingsGui(editor_layer);
      ImGui::TreePop();
    }

    ImGui::Checkbox("Render foliage", &dynamic_strands_settings_.foliage_render_parameters.enabled);
    if (dynamic_strands_settings_.foliage_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Foliage render settings")) {
        if (ImGui::Button("Rebuild foliage pipelines")) {
          DynamicStrands::BuildFoliageRenderingPipelines();
        }
        dynamic_strands_settings_.foliage_render_parameters.DrawGui(editor_layer);
        ImGui::TreePop();
      }
    }

    ImGui::Checkbox("Render segment pairs", &dynamic_strands_settings_.segment_pairs_render_parameters.enabled);
    if (dynamic_strands_settings_.segment_pairs_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Segment pairs render settings")) {
        if (ImGui::Button("Rebuild segment pairs pipelines")) {
          DynamicStrands::BuildSegmentPairsRenderingPipeline();
        }
        dynamic_strands_settings_.segment_pairs_render_parameters.DrawGui(editor_layer);
        ImGui::TreePop();
      }
    }

    ImGui::TreePop();
  }

  ImGui::Checkbox("Visualization", &dynamic_strands_settings_.enable_visualization);
  if (ImGui::TreeNode("Visualization settings")) {
    dynamic_strands_settings_.visualization_parameters.DrawGui(editor_layer);
    ImGui::TreePop();
  }
}

void EcoSysLabLayer::UpdateFlows(const std::vector<Entity>* tree_entities,
                                 const std::shared_ptr<Strands>& branch_strands) {
  {
    const auto scene = ApplicationContext::Get().GetActiveScene();

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
      Jobs::RunParallelFor(tree_entities->size(), [&](size_t tree_index) {
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

void EcoSysLabLayer::UpdateDemoTreeGrowth() {
  if (demo_tree_growth_finished_ || !ProjectManager::IsProjectIdle()) {
    return;
  }
  const auto project_path = ProjectManager::GetProjectPath();
  if (project_path.filename() != "test.eveproj" || project_path.parent_path().filename() != "EcoSysLabProject") {
    demo_tree_growth_finished_ = true;
    return;
  }
  const auto scene = GetScene();
  if (!scene) {
    return;
  }
  if (!demo_tree_initialized_) {
    const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
    if (!editor_layer || !editor_layer->IsPlantVisualSplitLayoutReady()) {
      return;
    }
    const auto descriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset("TreeDescriptors/Basic/Acacia.tree"));
    demo_tree_entity_ = descriptor ? descriptor->Instantiate() : Entity{};
    if (!scene->IsEntityValid(demo_tree_entity_)) {
      EVOENGINE_ERROR("Failed to instantiate the EcoSysLab demo Acacia tree.")
      demo_tree_growth_finished_ = true;
      return;
    }
    demo_tree_initialized_ = true;
    EVOENGINE_LOG("Started the EcoSysLab demo Acacia eight-year growth animation.")
  }

  constexpr float target_growth_time = 8.0f * 365.0f;
  if (simulated_time_ < target_growth_time) {
    auto growth_settings = simulation_settings;
    growth_settings.delta_time = std::min(simulation_settings.delta_time, target_growth_time - simulated_time_);
    Simulate(growth_settings, simulation_stats);
  }
  if (simulated_time_ < target_growth_time) {
    return;
  }
  auto demo_mesh_generator_settings = mesh_generator_settings;
  demo_mesh_generator_settings.foliage_instancing = false;
  scene->GetOrSetPrivateComponent<Tree>(demo_tree_entity_)
      .lock()
      ->GenerateGeometryEntities(demo_mesh_generator_settings);
  demo_tree_growth_finished_ = true;
  EVOENGINE_LOG("Finished the EcoSysLab demo Acacia eight-year growth animation and generated its mesh.")
}

void EcoSysLabLayer::Update() {
  if (const auto scene = GetScene(); !scene)
    return;
  UpdateDemoTreeGrowth();
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
