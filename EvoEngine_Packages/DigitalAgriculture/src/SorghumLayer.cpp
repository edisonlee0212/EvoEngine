#ifdef CUDA_MODULE_SERVICE
#  include <TriangleIlluminationEstimator.hpp>
#  include "BtfMeshRenderer.hpp"
#  include "RayTracerLayer.hpp"
#endif
#include <SorghumLayer.hpp>
#include "Application.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "Platform.hpp"
#include "Serialization.hpp"
#include "SkyIlluminance.hpp"
#include "SorghumGenerator.hpp"
#include "CropDescriptor.hpp"
#include "Times.hpp"

#include "Material.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumDescriptor.hpp"
#ifdef CUDA_MODULE_SERVICE
#  include "SorghumFieldGrid.hpp"
#  include "SorghumTraitDescriptor.hpp"
#  include "CBTFGroup.hpp"
#  include "PARSensorGroup.hpp"
#endif
using namespace digital_agriculture_package;
using namespace evo_engine;

namespace {
template <typename T>
void RegisterAssetPreviewHandler(const std::string& type_name) {
  Serialization::RegisterAssetPreviewHandler<T>(
      [](const std::shared_ptr<T>& asset, const OffscreenPreviewSettings&) {
        return asset ? asset->GenerateThumbnailTexture() : nullptr;
      },
      {}, type_name);
}

void RegisterDigitalAgricultureAssetPreviewHandlers() {
  RegisterAssetPreviewHandler<SorghumDescriptor>("SorghumDescriptor");
  RegisterAssetPreviewHandler<SorghumGrowthStages>("SorghumGrowthStages");
  RegisterAssetPreviewHandler<SorghumState>("SorghumState");
  RegisterAssetPreviewHandler<SorghumGenerator>("SorghumGenerator");
  RegisterAssetPreviewHandler<SorghumField>("SorghumField");
}

void RegisterDigitalAgricultureSerializationHandlers() {
  Serialization::RegisterSerializationHandler<SorghumDescriptor>(SerializeSorghumDescriptor,
                                                                 DeserializeSorghumDescriptor, {}, "SorghumDescriptor");
  Serialization::RegisterSerializationHandler<Sorghum>(SerializeSorghum, DeserializeSorghum, {}, "Sorghum");
  Serialization::RegisterSerializationHandler<SorghumGrowthStages>(
      SerializeSorghumGrowthStages, DeserializeSorghumGrowthStages, {}, "SorghumGrowthStages");
  Serialization::RegisterSerializationHandler<SorghumState>(SerializeSorghumState, DeserializeSorghumState, {},
                                                            "SorghumState");
  Serialization::RegisterSerializationHandler<SorghumGenerator>(SerializeSorghumGenerator, DeserializeSorghumGenerator,
                                                                {}, "SorghumGenerator");
  Serialization::RegisterSerializationHandler<SorghumField>(SerializeSorghumField, DeserializeSorghumField, {},
                                                            "SorghumField");
#ifdef CUDA_MODULE_SERVICE
  Serialization::RegisterSerializationHandler<PARSensorGroup>(SerializePARSensorGroup, DeserializePARSensorGroup, {},
                                                              "PARSensorGroup");
  Serialization::RegisterSerializationHandler<CBTFGroup>(SerializeCBTFGroup, DeserializeCBTFGroup, {}, "CBTFGroup");
#endif
  Serialization::RegisterSerializationHandler<SkyIlluminance>(SerializeSkyIlluminance, DeserializeSkyIlluminance, {},
                                                              "SkyIlluminance");
  Serialization::RegisterSerializationHandler<SorghumCoordinates>(
      SerializeSorghumCoordinates, DeserializeSorghumCoordinates, {}, "SorghumCoordinates");
}
}  // namespace

void SorghumLayer::RegisterTypes(Application& application) {
  application.RegisterAsset<SorghumDescriptor>("SorghumDescriptor", {".sorghum"});
  application.RegisterPrivateComponent<Sorghum>("Sorghum");
  application.RegisterAsset<SorghumGrowthStages>("SorghumGrowthStages", {".sgs"});
  application.RegisterAsset<SorghumState>("SorghumState", {".ss"});
  application.RegisterAsset<SorghumGenerator>("SorghumGenerator", {".sg"});
  application.RegisterAsset<SorghumField>("SorghumField", {".sorghumfield"});
  application.RegisterAsset<CropDescriptor>("CropDescriptor", {".cropdesc"});
#ifdef CUDA_MODULE_SERVICE
  application.RegisterAsset<PARSensorGroup>("PARSensorGroup", {".parsensorgroup"});
  application.RegisterAsset<CBTFGroup>("CBTFGroup", {".cbtfgroup"});

#endif
  application.RegisterAsset<SkyIlluminance>("SkyIlluminance", {".skyilluminance"});
  application.RegisterAsset<SorghumCoordinates>("SorghumCoordinates", {".sorghumcoords"});
  RegisterDigitalAgricultureSerializationHandlers();
  RegisterDigitalAgricultureAssetPreviewHandlers();
}

AssetRegistration<SorghumTraitDescriptor> st_registry("SorghumTraitDescriptor", {".st"});

void SorghumLayer::OnCreate() {
  if (!leaf_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    leaf_material = material;
    material->SetAlbedoTexture(leaf_albedo_texture.Get<Texture2D>());
    material->material_properties.albedo_color = glm::vec3(113.0f / 255, 169.0f / 255, 44.0f / 255);
    material->material_properties.roughness = 0.8f;
    material->material_properties.metallic = 0.1f;
  }

  if (!leaf_bottom_face_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    leaf_bottom_face_material = material;
    material->SetAlbedoTexture(leaf_albedo_texture.Get<Texture2D>());
    material->material_properties.albedo_color = glm::vec3(113.0f / 255, 169.0f / 255, 44.0f / 255);
    material->material_properties.roughness = 0.8f;
    material->material_properties.metallic = 0.1f;
  }

  if (!panicle_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    panicle_material = material;
    material->material_properties.albedo_color = glm::vec3(255.0 / 255, 210.0 / 255, 0.0 / 255);
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
  }

  for (auto& i : segmented_leaf_materials) {
    if (!i.Get<Material>()) {
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
      i = material;
      material->material_properties.albedo_color = glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f));
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
  }
}

void SorghumLayer::GenerateMeshForAllSorghums(
    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings) const {
  const auto scene = GetScene();
  if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
      sorghum_entities && !sorghum_entities->empty()) {
    for (const auto& sorghum_entity : *sorghum_entities) {
      const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
      sorghum->GenerateGeometryEntities(sorghum_mesh_generator_settings);
    }
  }
}

bool digital_agriculture_package::InspectSorghumLayer(InspectorContext& context, SorghumLayer& layer) {
  const auto& editor_layer = context.editor_layer;
  auto& enable_compressed_btf = layer.enable_compressed_btf;
  auto& sorghum_mesh_generator_settings = layer.sorghum_mesh_generator_settings;
  auto& leaf_albedo_texture = layer.leaf_albedo_texture;
  auto& leaf_normal_texture = layer.leaf_normal_texture;
  auto& leaf_material = layer.leaf_material;
  auto& vertical_subdivision_length = layer.vertical_subdivision_length;
  auto& horizontal_subdivision_step = layer.horizontal_subdivision_step;
  auto& skeleton_width = layer.skeleton_width;
  auto& skeleton_color = layer.skeleton_color;
#ifdef CUDA_MODULE_SERVICE
  auto& m_seed = layer.m_seed;
  auto& push_distance = layer.push_distance;
  auto& ray_properties = layer.ray_properties;
  auto& leaf_cbtf_group = layer.leaf_cbtf_group;
  auto& processing = layer.processing;
  auto& processing_index = layer.processing_index;
  auto& processing_entities = layer.processing_entities;
  auto& per_plant_calculation_time = layer.per_plant_calculation_time;
#endif
  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }
  const auto scene = layer.GetScene();
#ifdef CUDA_MODULE_SERVICE
  if (ImGui::TreeNodeEx("Illumination Estimation")) {
    ImGui::DragInt("Seed", &m_seed);
    ImGui::DragFloat("Push distance along normal", &push_distance, 0.0001f, -1.0f, 1.0f, "%.5f");
    ray_properties.DrawGui();

    if (ImGui::Button("Calculate illumination")) {
      layer.CalculateIlluminationFrameByFrame();
    }
    if (ImGui::Button("Calculate illumination instantly")) {
      layer.CalculateIllumination();
    }

    static bool show_probes = false;

    ImGui::Checkbox("Show probes", &show_probes);

    if (show_probes) {
      static bool depth_test = true;
      static std::shared_ptr<ParticleInfoList> probe_debug_info_list;
      static float node_render_size = .01f;
      static float energy_scale_factor = 1.f;
      if (!probe_debug_info_list)
        probe_debug_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
      if (ImGui::TreeNode("Debug settings")) {
        ImGui::DragFloat("Probe size", &node_render_size, 0.001f, 0.0f, 1.f);
        ImGui::DragFloat("Energy scale factor", &energy_scale_factor, 0.01f, 0.0f, 100.f);
        if (ImGui::Button("Refresh debug info")) {
          if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>()) {
            std::vector<ParticleInfo> particle_infos;
            for (const auto sorghum_entity : *owners) {
              if (const auto tie =
                      scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(sorghum_entity).lock()) {
                const auto start_index = particle_infos.size();
                particle_infos.resize(start_index + tie->PeekProbes().light_probes.size());
                for (int i = 0; i < tie->PeekProbes().light_probes.size(); i++) {
                  const auto& probe = tie->PeekProbes().light_probes.at(i);
                  auto& matrix = particle_infos[start_index + i].instance_matrix;
                  matrix.value = glm::translate(probe.GetCenter()) * glm::scale(glm::vec3(node_render_size));
                  particle_infos[start_index + i].instance_color =
                      glm::vec4(glm::vec3(glm::length(probe.energy) * energy_scale_factor), 1.0f);
                }
              }
            }
            probe_debug_info_list->SetParticleInfos(particle_infos);
          }
        }
        ImGui::Checkbox("Depth test", &depth_test);
        ImGui::TreePop();
      }

      GizmoSettings gizmo_settings{};
      gizmo_settings.depth_test = depth_test;
      editor_layer->DrawGizmoCubes(probe_debug_info_list, glm::mat4(1), 1, gizmo_settings);
    }

    ImGui::TreePop();
  }
  ImGui::Checkbox("Enable BTF", &enable_compressed_btf);
  if (enable_compressed_btf) {
    editor_layer->DragAndDropButton<CBTFGroup>(leaf_cbtf_group, "Leaf CBTFGroup");
  }
#endif
  ImGui::Separator();
  DrawSorghumMeshGeneratorSettingsGui(sorghum_mesh_generator_settings);
  ImGui::Checkbox("Auto increase crop Target GDD (Ctrl+F)", &auto_increase_crop_target_gdd_);
  ImGui::DragFloat("Crop Target GDD increase speed", &crop_target_gdd_increase_speed_, 1.0f, 0.0f, 5000.0f,
                   "%.2f gdd/s");
  crop_target_gdd_increase_speed_ = glm::max(0.0f, crop_target_gdd_increase_speed_);
  ImGui::DragFloat("Crop growth daily temperature (C)", &crop_growth_daily_temperature_, 0.5f, 0.0f, 45.0f);
  ImGui::DragFloat("Mesh rebuild interval (s)", &mesh_regen_interval_, 0.01f, 0.0f, 1.0f, "%.2f");
  mesh_regen_interval_ = glm::max(0.0f, mesh_regen_interval_);

  if (ImGui::Button("Generate mesh for all sorghums")) {
    layer.GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
  }
  if (ImGui::DragFloat("Vertical subdivision max unit length", &vertical_subdivision_length, 0.001f, 0.001f, 1.0f,
                       "%.4f")) {
    vertical_subdivision_length = glm::max(0.0001f, vertical_subdivision_length);
  }

  if (ImGui::DragInt("Horizontal subdivision step", &horizontal_subdivision_step)) {
    horizontal_subdivision_step = glm::max(2, horizontal_subdivision_step);
  }

  if (ImGui::DragFloat("Skeleton width", &skeleton_width, 0.001f, 0.001f, 1.0f, "%.4f")) {
    skeleton_width = glm::max(0.0001f, skeleton_width);
  }
  ImGui::ColorEdit3("Skeleton color", &skeleton_color.x);

  if (editor_layer->DragAndDropButton<Texture2D>(leaf_albedo_texture, "Replace Leaf Albedo Texture")) {
    auto tex = leaf_albedo_texture.Get<Texture2D>();
    if (tex) {
      leaf_material.Get<Material>()->SetAlbedoTexture(leaf_albedo_texture.Get<Texture2D>());
      if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
          sorghum_entities && !sorghum_entities->empty()) {
        for (const auto& sorghum_entity : *sorghum_entities) {
          for (const auto child : scene->GetChildren(sorghum_entity)) {
            if (scene->HasPrivateComponent<MeshRenderer>(child)) {
              scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->material.Get<Material>()->SetAlbedoTexture(
                  leaf_albedo_texture.Get<Texture2D>());
            }
          }
        }
      }
    }
  }

  if (editor_layer->DragAndDropButton<Texture2D>(leaf_normal_texture, "Replace Leaf Normal Texture")) {
    auto tex = leaf_normal_texture.Get<Texture2D>();
    if (tex) {
      leaf_material.Get<Material>()->SetNormalTexture(leaf_normal_texture.Get<Texture2D>());
      if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
          sorghum_entities && !sorghum_entities->empty()) {
        for (const auto& sorghum_entity : *sorghum_entities) {
          for (const auto child : scene->GetChildren(sorghum_entity)) {
            if (scene->HasPrivateComponent<MeshRenderer>(child)) {
              scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->material.Get<Material>()->SetNormalTexture(
                  leaf_albedo_texture.Get<Texture2D>());
            }
          }
        }
      }
    }
  }

  FileUtils::SaveFile(
      "Export OBJ for all sorghums", "3D Model", {".obj"},
      [&layer](const std::filesystem::path& path) {
        layer.ExportAllSorghumsModel(path.string());
      },
      false);

  static bool opened = false;
#ifdef CUDA_MODULE_SERVICE
  if (processing && !opened) {
    ImGui::OpenPopup("Illumination Estimation");
    opened = true;
  }
  if (ImGui::BeginPopupModal("Illumination Estimation", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Progress: ");
    const float fraction = 1.0f - static_cast<float>(processing_index) / processing_entities.size();
    const std::string text = std::to_string(static_cast<int>(fraction * 100.0f)) + "% - " +
                             std::to_string(processing_entities.size() - processing_index) + "/" +
                             std::to_string(processing_entities.size());
    ImGui::ProgressBar(fraction, ImVec2(240, 0), text.c_str());
    ImGui::SetItemDefaultFocus();
    ImGui::Text(("Estimation time for 1 plant: " + std::to_string(per_plant_calculation_time) + " seconds").c_str());
    if (ImGui::Button("Cancel") || processing == false) {
      processing = false;
      opened = false;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
#endif
  ImGui::End();
  layer.enable_inspection = open;
  return false;
}

void SorghumLayer::ExportSorghum(const Entity& sorghum, std::ofstream& of, unsigned& start_index) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const std::string start = "#Sorghum\n";
  of.write(start.c_str(), start.size());
  of.flush();
  const auto position = scene->GetDataComponent<GlobalTransform>(sorghum).GetPosition();

  const auto stem_mesh = scene->GetOrSetPrivateComponent<MeshRenderer>(sorghum).lock()->mesh.Get<Mesh>();
  ObjExportHelper(position, stem_mesh, of, start_index);

  scene->ForEachDescendant(sorghum, [&](const Entity child) {
    if (!scene->HasPrivateComponent<MeshRenderer>(child))
      return;
    const auto leaf_mesh = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->mesh.Get<Mesh>();
    ObjExportHelper(position, leaf_mesh, of, start_index);
  });
}

void SorghumLayer::ObjExportHelper(glm::vec3 position, const std::shared_ptr<Mesh>& mesh, std::ofstream& of,
                                   unsigned& start_index) {
  if (mesh && !mesh->UnsafeGetTriangles().empty()) {
    std::string header = "#Vertices: " + std::to_string(mesh->GetVerticesAmount()) +
                         ", tris: " + std::to_string(mesh->GetTriangleAmount());
    header += "\n";
    of.write(header.c_str(), header.size());
    of.flush();
    std::string o = "o ";
    o += "[" + std::to_string(position.x) + "," + std::to_string(position.z) + "]" + "\n";
    of.write(o.c_str(), o.size());
    of.flush();
    std::string data;
#pragma region Data collection

    for (auto& i : mesh->UnsafeGetVertices()) {
      const auto& vertex_position = i.position;
      const auto& color = i.color;
      data += "v " + std::to_string(vertex_position.x + position.x) + " " +
              std::to_string(vertex_position.y + position.y) + " " + std::to_string(vertex_position.z + position.z) +
              " " + std::to_string(color.x) + " " + std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
    }
    for (const auto& vertex : mesh->UnsafeGetVertices()) {
      data += "vn " + std::to_string(vertex.normal.x) + " " + std::to_string(vertex.normal.y) + " " +
              std::to_string(vertex.normal.z) + "\n";
    }

    for (const auto& vertex : mesh->UnsafeGetVertices()) {
      data += "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
    }
    // data += "s off\n";
    data += "# List of indices for faces vertices, with (x, y, z).\n";
    const auto& triangles = mesh->UnsafeGetTriangles();
    for (auto i = 0; i < mesh->GetTriangleAmount(); i++) {
      const auto triangle = triangles[i];
      const auto f1 = triangle.x + start_index;
      const auto f2 = triangle.y + start_index;
      const auto f3 = triangle.z + start_index;
      data += "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
              std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " + std::to_string(f3) +
              "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
    }
    start_index += mesh->GetVerticesAmount();
#pragma endregion
    of.write(data.c_str(), data.size());
    of.flush();
  }
}

void SorghumLayer::ExportAllSorghumsModel(const std::string& filename) const {
  std::ofstream of;
  of.open(filename, std::ofstream::out | std::ofstream::trunc);
  if (of.is_open()) {
    std::string start = "#Sorghum field, by Bosheng Li";
    start += "\n";
    of.write(start.c_str(), start.size());
    of.flush();
    const auto scene = GetScene();
    if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
        sorghum_entities && !sorghum_entities->empty()) {
      unsigned start_index = 1;
      for (const auto& sorghum_entity : *sorghum_entities) {
        ExportSorghum(sorghum_entity, of, start_index);
      }
    }
    of.close();
    EVOENGINE_LOG("Sorghums saved as " + filename);
  } else {
    EVOENGINE_ERROR("Can't open file!");
  }
}

#ifdef CUDA_MODULE_SERVICE
void SorghumLayer::CalculateIlluminationFrameByFrame() {
  const auto scene = GetScene();
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
  if (!owners)
    return;
  processing_entities.clear();

  processing_entities.insert(processing_entities.begin(), owners->begin(), owners->end());
  processing_index = processing_entities.size();
  processing = true;
}
void SorghumLayer::CalculateIllumination() {
  const auto scene = GetScene();
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
  if (!owners)
    return;
  processing_entities.clear();

  processing_entities.insert(processing_entities.begin(), owners->begin(), owners->end());
  processing_index = processing_entities.size();

  processing = true;
  while (processing) {
    processing_index--;
    if (processing_index == -1) {
      processing = false;
    } else {
      const float timer = ApplicationContext::Get().GetTimes().Now();
      const auto estimator =
          scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(processing_entities[processing_index]).lock();
      estimator->PrepareLightProbeGroup();
      estimator->SampleLightProbeGroup(ray_properties, m_seed, push_distance);
    }
  }
}
#endif
void SorghumLayer::Update() {
  const auto scene = GetScene();


#ifdef ECOSYSLAB_PACKAGE
  if (EditorLayer::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold ||
      EditorLayer::GetKey(GLFW_KEY_RIGHT_CONTROL) == Input::KeyActionType::Hold) {
    if (EditorLayer::GetKey(GLFW_KEY_W) == Input::KeyActionType::Press) {
      auto_increase_crop_target_gdd_ = false;
      if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
          sorghum_entities && !sorghum_entities->empty()) {
        for (const auto& sorghum_entity : *sorghum_entities) {
          const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
          if (!sorghum || !sorghum->crop_descriptor.Get<CropDescriptor>()) {
            continue;
          }
          sorghum->GrowCropToGdd(0.0f, crop_growth_daily_temperature_);
          sorghum->sorghum_descriptor.Clear();
          sorghum->GenerateGeometryEntities(sorghum_mesh_generator_settings);
        }
      }
    }
    if (EditorLayer::GetKey(GLFW_KEY_F) == Input::KeyActionType::Press) {
      auto_increase_crop_target_gdd_ = !auto_increase_crop_target_gdd_;
    }
  }

  if (auto_increase_crop_target_gdd_ && crop_target_gdd_increase_speed_ > 0.0f) {
    if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
        sorghum_entities && !sorghum_entities->empty()) {
      const float delta_gdd = crop_target_gdd_increase_speed_ * static_cast<float>(Times::DeltaTime());
      // Advance the growth model every frame (cheap).
      for (const auto& sorghum_entity : *sorghum_entities) {
        const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
        if (!sorghum || !sorghum->crop_descriptor.Get<CropDescriptor>()) {
          continue;
        }
        sorghum->GrowCropByGdd(delta_gdd, crop_growth_daily_temperature_);
      }
      crop_growth_dirty_ = true;

      // Only rebuild geometry at a capped rate to avoid killing the framerate.
      const float now = Times::Now();
      if (now - last_mesh_regen_time_ >= mesh_regen_interval_) {
        last_mesh_regen_time_ = now;
        crop_growth_dirty_ = false;
        for (const auto& sorghum_entity : *sorghum_entities) {
          const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
          if (!sorghum || !sorghum->crop_descriptor.Get<CropDescriptor>()) {
            continue;
          }
          sorghum->sorghum_descriptor.Clear();
          sorghum->GenerateGeometryEntities(sorghum_mesh_generator_settings);
        }
      }
    }
  } else if (crop_growth_dirty_) {
    // Auto-growth just stopped — do one final mesh rebuild to show the latest state.
    crop_growth_dirty_ = false;
    if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
        sorghum_entities && !sorghum_entities->empty()) {
      for (const auto& sorghum_entity : *sorghum_entities) {
        const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
        if (!sorghum || !sorghum->crop_descriptor.Get<CropDescriptor>()) {
          continue;
        }
        sorghum->sorghum_descriptor.Clear();
        sorghum->GenerateGeometryEntities(sorghum_mesh_generator_settings);
      }
    }
  }
#endif  

#ifdef CUDA_MODULE_SERVICE
  if (processing) {
    processing_index--;
    if (processing_index == -1) {
      processing = false;
    } else {
      const float timer = ApplicationContext::Get().GetTimes().Now();
      const auto estimator =
          scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(processing_entities[processing_index]).lock();
      estimator->PrepareLightProbeGroup();
      estimator->SampleLightProbeGroup(ray_properties, m_seed, push_distance);
      per_plant_calculation_time = ApplicationContext::Get().GetTimes().Now() - timer;
    }
  }
#endif
}
