#include <SorghumLayer.hpp>
#include "Application.hpp"
#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "CBTFImporter.hpp"
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
