#include <SorghumLayer.hpp>
#include "Application.hpp"
#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "CBTFImporter.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "Platform.hpp"
#include "Serialization.hpp"
#include "SkyIlluminance.hpp"
#include "SorghumGenerator.hpp"
#include "Times.hpp"

#include "CBTFGroup.hpp"
#include "Material.hpp"
#include "PARSensorGroup.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumDescriptor.hpp"
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
  Serialization::RegisterSerializationHandler<PARSensorGroup>(SerializePARSensorGroup, DeserializePARSensorGroup, {},
                                                              "PARSensorGroup");
  Serialization::RegisterSerializationHandler<CBTFGroup>(SerializeCBTFGroup, DeserializeCBTFGroup, {}, "CBTFGroup");
  Serialization::RegisterSerializationHandler<BtfMeshRenderer>(SerializeBtfMeshRenderer, DeserializeBtfMeshRenderer, {},
                                                               "BtfMeshRenderer");
  Serialization::RegisterSerializationHandler<BtfMaterial>(SerializeBtfMaterial, DeserializeBtfMaterial, {},
                                                           "BtfMaterial");
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
  application.RegisterAsset<PARSensorGroup>("PARSensorGroup", {".parsensorgroup"});
  application.RegisterAsset<CBTFGroup>("CBTFGroup", {".cbtfgroup"});
  application.RegisterAsset<BtfMaterial>("BtfMaterial", {".btf"});
  application.RegisterPrivateComponent<BtfMeshRenderer>("BtfMeshRenderer");
  application.RegisterPrivateComponent<CBTFImporter>("CBTFImporter");
  application.RegisterAsset<SkyIlluminance>("SkyIlluminance", {".skyilluminance"});
  application.RegisterAsset<SorghumCoordinates>("SorghumCoordinates", {".sorghumcoords"});
  RegisterDigitalAgricultureSerializationHandlers();
  RegisterDigitalAgricultureAssetPreviewHandlers();
}

void SorghumLayer::OnCreate() {
  const auto configure_material = [](const std::shared_ptr<Material>& material,
                                     const std::shared_ptr<Texture2D>& texture, const glm::vec3& color,
                                     const float roughness, const float metallic) {
    material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, texture);
    auto& shade_material = material->material_data.shade_material;
    shade_material.pbr_base_color_factor = glm::vec4(color, 1.0f);
    shade_material.pbr_roughness_factor = roughness;
    shade_material.pbr_metallic_factor = metallic;
    material->MarkDirty();
  };
  if (!leaf_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    leaf_material = material;
    configure_material(material, leaf_albedo_texture.Get<Texture2D>(),
                       glm::vec3(113.0f / 255, 169.0f / 255, 44.0f / 255), 0.8f, 0.1f);
  }

  if (!leaf_bottom_face_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    leaf_bottom_face_material = material;
    configure_material(material, leaf_albedo_texture.Get<Texture2D>(),
                       glm::vec3(113.0f / 255, 169.0f / 255, 44.0f / 255), 0.8f, 0.1f);
  }

  if (!panicle_material.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    panicle_material = material;
    configure_material(material, nullptr, glm::vec3(255.0 / 255, 210.0 / 255, 0.0 / 255), 0.5f, 0.0f);
  }

  for (auto& i : segmented_leaf_materials) {
    if (!i.Get<Material>()) {
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
      i = material;
      configure_material(material, nullptr, glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)), 1.0f, 0.0f);
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
  auto& leaf_cbtf_group = layer.leaf_cbtf_group;
  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }
  const auto scene = layer.GetScene();
  ImGui::Checkbox("Enable BTF", &enable_compressed_btf);
  if (enable_compressed_btf) {
    editor_layer->DragAndDropButton<CBTFGroup>(leaf_cbtf_group, "Leaf CBTFGroup");
  }
  ImGui::Separator();
  DrawSorghumMeshGeneratorSettingsGui(sorghum_mesh_generator_settings);
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
      leaf_material.Get<Material>()->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, tex);
      if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
          sorghum_entities && !sorghum_entities->empty()) {
        for (const auto& sorghum_entity : *sorghum_entities) {
          for (const auto child : scene->GetChildren(sorghum_entity)) {
            if (scene->HasPrivateComponent<MeshRenderer>(child)) {
              scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->material.Get<Material>()->SetTexture(
                  &GltfShadeMaterial::pbr_base_color_texture, tex);
            }
          }
        }
      }
    }
  }

  if (editor_layer->DragAndDropButton<Texture2D>(leaf_normal_texture, "Replace Leaf Normal Texture")) {
    auto tex = leaf_normal_texture.Get<Texture2D>();
    if (tex) {
      leaf_material.Get<Material>()->SetTexture(&GltfShadeMaterial::normal_texture, tex);
      if (const std::vector<Entity>* sorghum_entities = scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
          sorghum_entities && !sorghum_entities->empty()) {
        for (const auto& sorghum_entity : *sorghum_entities) {
          for (const auto child : scene->GetChildren(sorghum_entity)) {
            if (scene->HasPrivateComponent<MeshRenderer>(child)) {
              scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->material.Get<Material>()->SetTexture(
                  &GltfShadeMaterial::normal_texture, tex);
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
