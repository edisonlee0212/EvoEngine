#ifdef BUILD_WITH_RAYTRACER
#  include <TriangleIlluminationEstimator.hpp>
#  include "BTFMeshRenderer.hpp"
#  include "RayTracerLayer.hpp"
#endif
#include <SorghumLayer.hpp>
#include "ClassRegistry.hpp"
#include "Graphics.hpp"
#include "SkyIlluminance.hpp"
#include "SorghumStateGenerator.hpp"
#include "Times.hpp"

#include "Material.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumPointCloudScanner.hpp"
#ifdef BUILD_WITH_RAYTRACER
#  include "CBTFGroup.hpp"
#  include "DoubleCBTF.hpp"
#  include "PARSensorGroup.hpp"
#endif
using namespace eco_sys_lab;
using namespace evo_engine;

void SorghumLayer::OnCreate() {
  ClassRegistry::RegisterAsset<SorghumState>("SorghumState", {".ss"});
  ClassRegistry::RegisterPrivateComponent<Sorghum>("Sorghum");

  ClassRegistry::RegisterAsset<SorghumDescriptor>("SorghumDescriptor", {".sorghum"});
  ClassRegistry::RegisterAsset<SorghumGrowthStages>("SorghumGrowthStages", {".sgs"});
  ClassRegistry::RegisterAsset<SorghumStateGenerator>("SorghumStateGenerator", {".ssg"});
  ClassRegistry::RegisterAsset<SorghumField>("SorghumField", {".sorghumfield"});
#ifdef BUILD_WITH_RAYTRACER
  ClassRegistry::RegisterAsset<PARSensorGroup>("PARSensorGroup", {".parsensorgroup"});
  ClassRegistry::RegisterAsset<CBTFGroup>("CBTFGroup", {".cbtfg"});
  ClassRegistry::RegisterAsset<DoubleCBTF>("DoubleCBTF", {".dcbtf"});
#endif
  ClassRegistry::RegisterAsset<SkyIlluminance>("SkyIlluminance", {".skyilluminance"});
  ClassRegistry::RegisterAsset<SorghumCoordinates>("SorghumCoordinates", {".sorghumcoords"});
  ClassRegistry::RegisterPrivateComponent<SorghumPointCloudScanner>("SorghumPointCloudScanner");
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    auto texture_2d = ProjectManager::CreateTemporaryAsset<Texture2D>();
    texture_2d->Import(std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources/Textures") /
                                                 "SorghumGrowthDescriptor.png"));
    editor_layer->AssetIcons()["SorghumGrowthDescriptor"] = texture_2d;
    texture_2d = ProjectManager::CreateTemporaryAsset<Texture2D>();
    texture_2d->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources/Textures") / "SorghumDescriptor.png"));
    editor_layer->AssetIcons()["SorghumDescriptor"] = texture_2d;
    texture_2d = ProjectManager::CreateTemporaryAsset<Texture2D>();
    texture_2d->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources/Textures") / "PositionsField.png"));
    editor_layer->AssetIcons()["PositionsField"] = texture_2d;

    texture_2d->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources/Textures") / "GeneralDataPipeline.png"));
    editor_layer->AssetIcons()["GeneralDataCapture"] = texture_2d;
  }

  if (!leaf_albedo_texture.Get<Texture2D>()) {
    const auto albedo = ProjectManager::CreateTemporaryAsset<Texture2D>();
    albedo->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources/Textures") / "leafSurface.jpg"));
    leaf_albedo_texture.Set(albedo);
  }

  if (!leaf_material.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    leaf_material = material;
    material->SetAlbedoTexture(leaf_albedo_texture.Get<Texture2D>());
    material->material_properties.albedo_color = glm::vec3(113.0f / 255, 169.0f / 255, 44.0f / 255);
    material->material_properties.roughness = 0.8f;
    material->material_properties.metallic = 0.1f;
  }

  if (!leaf_bottom_face_material.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    leaf_bottom_face_material = material;
    material->SetAlbedoTexture(leaf_albedo_texture.Get<Texture2D>());
    material->material_properties.albedo_color = glm::vec3(113.0f / 255, 169.0f / 255, 44.0f / 255);
    material->material_properties.roughness = 0.8f;
    material->material_properties.metallic = 0.1f;
  }

  if (!panicle_material.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    panicle_material = material;
    material->material_properties.albedo_color = glm::vec3(255.0 / 255, 210.0 / 255, 0.0 / 255);
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
  }

  for (auto& i : segmented_leaf_materials) {
    if (!i.Get<Material>()) {
      const auto material = ProjectManager::CreateTemporaryAsset<Material>();
      i = material;
      material->material_properties.albedo_color = glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f));
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
  }
}

void SorghumLayer::GenerateMeshForAllSorghums(
    const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings) const {
  std::vector<Entity> plants;
  const auto scene = GetScene();
  if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
      sorghum_entities && !sorghum_entities->empty()) {
    for (const auto& sorghum_entity : *sorghum_entities) {
      const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
      sorghum->GenerateGeometryEntities(sorghum_mesh_generator_settings);
    }
  }
}

void SorghumLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto scene = GetScene();
  if (ImGui::Begin("Sorghum Layer")) {
#ifdef BUILD_WITH_RAYTRACER
    if (ImGui::TreeNodeEx("Illumination Estimation")) {
      ImGui::DragInt("Seed", &m_seed);
      ImGui::DragFloat("Push distance along normal", &push_distance, 0.0001f, -1.0f, 1.0f, "%.5f");
      ray_properties.OnInspect();

      if (ImGui::Button("Calculate illumination")) {
        CalculateIlluminationFrameByFrame();
      }
      if (ImGui::Button("Calculate illumination instantly")) {
        CalculateIllumination();
      }
      ImGui::TreePop();
    }
    editor_layer->DragAndDropButton<CBTFGroup>(leaf_cbtf_group, "Leaf CBTFGroup");

    ImGui::Checkbox("Enable BTF", &enable_compressed_btf);
#endif
    ImGui::Separator();
    ImGui::Checkbox("Auto regenerate sorghum", &auto_refresh_sorghums);
    sorghum_mesh_generator_settings.OnInspect(editor_layer);
    if (ImGui::Button("Generate mesh for all sorghums")) {
      GenerateMeshForAllSorghums(sorghum_mesh_generator_settings);
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
        std::vector<Entity> sorghum_entities;

        for (const auto& i : sorghum_entities) {
          if (scene->HasPrivateComponent<MeshRenderer>(i)) {
            scene->GetOrSetPrivateComponent<MeshRenderer>(i).lock()->material.Get<Material>()->SetAlbedoTexture(
                leaf_albedo_texture.Get<Texture2D>());
          }
        }
      }
    }

    if (editor_layer->DragAndDropButton<Texture2D>(leaf_normal_texture, "Replace Leaf Normal Texture")) {
      auto tex = leaf_normal_texture.Get<Texture2D>();
      if (tex) {
        leaf_material.Get<Material>()->SetNormalTexture(leaf_normal_texture.Get<Texture2D>());
        const std::vector<Entity> sorghum_entities;

        for (const auto& i : sorghum_entities) {
          if (scene->HasPrivateComponent<MeshRenderer>(i)) {
            scene->GetOrSetPrivateComponent<MeshRenderer>(i).lock()->material.Get<Material>()->SetNormalTexture(
                leaf_normal_texture.Get<Texture2D>());
          }
        }
      }
    }

    FileUtils::SaveFile("Export OBJ for all sorghums", "3D Model", {".obj"}, [this](const std::filesystem::path& path) {
      ExportAllSorghumsModel(path.string());
    });

    static bool opened = false;
#ifdef BUILD_WITH_RAYTRACER
    if (processing && !opened) {
      ImGui::OpenPopup("Illumination Estimation");
      opened = true;
    }
    if (ImGui::BeginPopupModal("Illumination Estimation", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("Progress: ");
      float fraction = 1.0f - static_cast<float>(processing_index) / processing_entities.size();
      std::string text = std::to_string(static_cast<int>(fraction * 100.0f)) + "% - " +
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
  }
  ImGui::End();
}

void SorghumLayer::ExportSorghum(const Entity& sorghum, std::ofstream& of, unsigned& start_index) {
  auto scene = Application::GetActiveScene();
  const std::string start = "#Sorghum\n";
  of.write(start.c_str(), start.size());
  of.flush();
  const auto position = scene->GetDataComponent<GlobalTransform>(sorghum).GetPosition();

  const auto stem_mesh = scene->GetOrSetPrivateComponent<MeshRenderer>(sorghum).lock()->mesh.Get<Mesh>();
  ObjExportHelper(position, stem_mesh, of, start_index);

  scene->ForEachDescendant(sorghum, [&](Entity child) {
    if (!scene->HasPrivateComponent<MeshRenderer>(child))
      return;
    const auto leaf_mesh = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->mesh.Get<Mesh>();
    ObjExportHelper(position, leaf_mesh, of, start_index);
  });
}

void SorghumLayer::ObjExportHelper(glm::vec3 position, std::shared_ptr<Mesh> mesh, std::ofstream& of,
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

    for (auto i = 0; i < mesh->UnsafeGetVertices().size(); i++) {
      auto& vertex_position = mesh->UnsafeGetVertices().at(i).position;
      auto& color = mesh->UnsafeGetVertices().at(i).color;
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
    auto& triangles = mesh->UnsafeGetTriangles();
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

void SorghumLayer::ExportAllSorghumsModel(const std::string& filename) {
  std::ofstream of;
  of.open(filename, std::ofstream::out | std::ofstream::trunc);
  if (of.is_open()) {
    std::string start = "#Sorghum field, by Bosheng Li";
    start += "\n";
    of.write(start.c_str(), start.size());
    of.flush();
    auto scene = GetScene();
    unsigned start_index = 1;
    std::vector<Entity> sorghums;
    for (const auto& plant : sorghums) {
      ExportSorghum(plant, of, start_index);
    }
    of.close();
    EVOENGINE_LOG("Sorghums saved as " + filename);
  } else {
    EVOENGINE_ERROR("Can't open file!");
  }
}

#ifdef BUILD_WITH_RAYTRACER
void SorghumLayer::CalculateIlluminationFrameByFrame() {
  auto scene = GetScene();
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<TriangleIlluminationEstimator>();
  if (!owners)
    return;
  processing_entities.clear();

  processing_entities.insert(processing_entities.begin(), owners->begin(), owners->end());
  processing_index = processing_entities.size();
  processing = true;
}
void SorghumLayer::CalculateIllumination() {
  auto scene = GetScene();
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<TriangleIlluminationEstimator>();
  if (!owners)
    return;
  processing_entities.clear();

  processing_entities.insert(processing_entities.begin(), owners->begin(), owners->end());
  processing_index = processing_entities.size();
  while (processing) {
    processing_index--;
    if (processing_index == -1) {
      processing = false;
    } else {
      const float timer = Times::Now();
      auto estimator =
          scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(processing_entities[processing_index]).lock();
      estimator->PrepareLightProbeGroup();
      estimator->SampleLightProbeGroup(ray_properties, m_seed, push_distance);
    }
  }
}
#endif
void SorghumLayer::Update() {
  auto scene = GetScene();
#ifdef BUILD_WITH_RAYTRACER
  if (processing) {
    processing_index--;
    if (processing_index == -1) {
      processing = false;
    } else {
      const float timer = Times::Now();
      auto estimator =
          scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(processing_entities[processing_index]).lock();
      estimator->PrepareLightProbeGroup();
      estimator->SampleLightProbeGroup(ray_properties, m_seed, push_distance);
      per_plant_calculation_time = Times::Now() - timer;
    }
  }
#endif
}

void SorghumLayer::LateUpdate() {
  if (auto_refresh_sorghums) {
    auto scene = GetScene();
    std::vector<Entity> plants;
    for (auto& plant : plants) {
    }
  }
}
