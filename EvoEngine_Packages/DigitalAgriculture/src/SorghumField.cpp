//
// Created by lllll on 9/16/2021.
//

#include "SorghumField.hpp"

#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Soil.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
#include "TransformGraph.hpp"
using namespace digital_agriculture_package;
using namespace eco_sys_lab_package;
void SorghumGrid::GenerateField(std::vector<glm::mat4>& matrices_list) const {
  std::shared_ptr<Soil> soil;
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  std::shared_ptr<SoilDescriptor> soil_descriptor;
  if (soil) {
    soil_descriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>();
  }
  std::shared_ptr<HeightField> height_field{};
  if (soil_descriptor) {
    height_field = soil_descriptor->height_field.Get<HeightField>();
  }
  matrices_list.resize(grid_size.x * grid_size.y);
  const glm::vec2 start_point =
      glm::vec2((grid_size.x - 1) * grid_distance.x, (grid_size.y - 1) * grid_distance.y) * 0.5f;
  for (int i = 0; i < grid_size.x; i++) {
    for (int j = 0; j < grid_size.y; j++) {
      glm::vec3 position = glm::vec3(-start_point.x + i * grid_distance.x, 0.0f, -start_point.y + j * grid_distance.y);
      position.x += glm::linearRand(-grid_distance.x * position_offset_mean, grid_distance.x * position_offset_mean);
      position.z += glm::linearRand(-grid_distance.y * position_offset_mean, grid_distance.y * position_offset_mean);
      position += glm::gaussRand(glm::vec3(0.0f), glm::vec3(position_offset_variance, 0.0f, position_offset_variance));
      if (height_field)
        position.y = height_field->GetValue({position.x, position.z}) - 0.01f;
      Transform transform{};
      transform.SetPosition(position);
      auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(
          glm::vec3(0.0f), glm::vec3(rotation_variance_xz, rotation_variance_y, rotation_variance_xz)))));
      transform.SetRotation(rotation);
      transform.SetScale(glm::vec3(1.f));
      matrices_list[i * grid_size.y + j] = transform.value;
    }
  }
}

bool SorghumField::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragInt("Size limit", &size_limit, 1, 0, 10000))
    changed = false;
  if (ImGui::DragFloat("Sorghum size", &sorghum_size, 0.01f, 0, 10))
    changed = false;
  if (ImGui::Button("Instantiate")) {
    InstantiateField();
  }

  static int index = 200;
  static float radius = 2.5f;
  ImGui::DragInt("Index", &index);
  ImGui::DragFloat("Radius", &radius);
  static AssetRef temp_coordinates;
  if (editor_layer->DragAndDropButton<SorghumCoordinates>(temp_coordinates, "Apply from sorghum coordinates")) {
    if (const auto field = temp_coordinates.Get<SorghumCoordinates>()) {
      glm::dvec2 offset;
      field->Apply(std::dynamic_pointer_cast<SorghumField>(GetSelf()), offset, index, radius);
      temp_coordinates.Clear();
    }
  }
  ImGui::Text("Matrices count: %d", (int)matrices.size());

  return changed;
}
void SorghumField::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "size_limit" << YAML::Value << size_limit;
  out << YAML::Key << "sorghum_size" << YAML::Value << sorghum_size;
  out << YAML::Key << "matrices" << YAML::Value << YAML::BeginSeq;
  for (auto& i : matrices) {
    out << YAML::BeginMap;
    i.first.Save("SPD", out);
    out << YAML::Key << "Transform" << YAML::Value << i.second;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}
void SorghumField::Deserialize(const YAML::Node& in) {
  if (in["size_limit"])
    size_limit = in["size_limit"].as<int>();
  if (in["sorghum_size"])
    sorghum_size = in["sorghum_size"].as<float>();

  matrices.clear();
  if (in["matrices"]) {
    for (const auto& i : in["matrices"]) {
      AssetRef spd;
      spd.Load("SPD", i);
      matrices.emplace_back(spd, i["Transform"].as<glm::mat4>());
    }
  }
}

std::shared_ptr<Texture2D> SorghumField::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./DigitalAgricultureResources") / "Icons/SorghumField.png"));
  }
  return thumbnail;
}

void SorghumField::CollectAssetRef(std::vector<AssetRef>& list) {
  for (auto& i : matrices) {
    list.push_back(i.first);
  }
}
Entity SorghumField::InstantiateField(uint32_t base_seed) const {
  if (matrices.empty()) {
    EVOENGINE_ERROR("No matrices generated!");
    return {};
  }

  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  const auto scene = sorghum_layer->GetScene();
  if (sorghum_layer) {
    std::shared_ptr<Soil> soil;
    if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
      soil = soil_candidate.lock();
    std::shared_ptr<SoilDescriptor> soil_descriptor;
    if (soil) {
      soil_descriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>();
    }
    std::shared_ptr<HeightField> height_field{};
    if (soil_descriptor) {
      height_field = soil_descriptor->height_field.Get<HeightField>();
    }

    const auto field_asset = std::dynamic_pointer_cast<SorghumField>(GetSelf());
    const auto field = scene->CreateEntity("Field");
    // Create sorghums here.
    int size = 0;
    for (int matrix_index = 0; matrix_index < field_asset->matrices.size(); matrix_index++) {
      auto& new_sorghum = field_asset->matrices[matrix_index];
      const auto sorghum_generator = new_sorghum.first.Get<SorghumGenerator>();
      if (!sorghum_generator)
        continue;
      Entity sorghum_entity = sorghum_generator->CreateEntity(size);
      auto sorghum_transform = scene->GetDataComponent<Transform>(sorghum_entity);
      sorghum_transform.value = new_sorghum.second;

      if (height_field)
        sorghum_transform.value[3].y =
            height_field->GetValue({sorghum_transform.value[3].x, sorghum_transform.value[3].z}) - 0.01f;

      sorghum_transform.SetScale(glm::vec3(sorghum_size));
      scene->SetDataComponent(sorghum_entity, sorghum_transform);
      scene->SetParent(sorghum_entity, field);

      const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
      sorghum->sorghum_generator = sorghum_generator;
      const auto sorghum_descriptor = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
      sorghum_generator->Apply(sorghum_descriptor, base_seed + matrix_index);
      sorghum->sorghum_descriptor = sorghum_generator;
      size++;
      if (size >= size_limit)
        break;
    }

    TransformGraph::CalculateTransformGraphForDescendants(scene, field);
    return field;
  }
  EVOENGINE_ERROR("No sorghum layer!");
  return {};
}
