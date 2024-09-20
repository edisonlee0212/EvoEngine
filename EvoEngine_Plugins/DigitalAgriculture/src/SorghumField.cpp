//
// Created by lllll on 9/16/2021.
//

#include "SorghumField.hpp"

#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Soil.hpp"
#include "Sorghum.hpp"
#include "SorghumLayer.hpp"
#include "SorghumDescriptorGenerator.hpp"
#include "TransformGraph.hpp"
using namespace digital_agriculture_plugin;
using namespace eco_sys_lab_plugin;
void SorghumFieldPatch::GenerateField(std::vector<glm::mat4>& matrices_list) const {
  std::shared_ptr<Soil> soil;
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  std::shared_ptr<SoilDescriptor> soil_descriptor;
  if (soil) {
    soil_descriptor = soil->soil_descriptor.Get<SoilDescriptor>();
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
      position.x +=
          glm::linearRand(-grid_distance.x * position_offset_mean.x, grid_distance.x * position_offset_mean.x);
      position.z +=
          glm::linearRand(-grid_distance.y * position_offset_mean.y, grid_distance.y * position_offset_mean.y);
      position +=
          glm::gaussRand(glm::vec3(0.0f), glm::vec3(position_offset_variance.x, 0.0f, position_offset_variance.y));
      if (height_field)
        position.y = height_field->GetValue({position.x, position.z}) - 0.01f;
      Transform transform{};
      transform.SetPosition(position);
      auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), rotation_variance))));
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
void SorghumField::CollectAssetRef(std::vector<AssetRef>& list) {
  for (auto& i : matrices) {
    list.push_back(i.first);
  }
}
Entity SorghumField::InstantiateField() const {
  if (matrices.empty()) {
    EVOENGINE_ERROR("No matrices generated!");
    return {};
  }

  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  const auto scene = sorghum_layer->GetScene();
  if (sorghum_layer) {
    const auto field_asset = std::dynamic_pointer_cast<SorghumField>(GetSelf());
    const auto field = scene->CreateEntity("Field");
    // Create sorghums here.
    int size = 0;
    for (auto& new_sorghum : field_asset->matrices) {
      const auto sorghum_descriptor = new_sorghum.first.Get<SorghumDescriptorGenerator>();
      if (!sorghum_descriptor)
        continue;
      Entity sorghum_entity = sorghum_descriptor->CreateEntity(size);
      auto sorghum_transform = scene->GetDataComponent<Transform>(sorghum_entity);
      sorghum_transform.value = new_sorghum.second;
      sorghum_transform.SetScale(glm::vec3(sorghum_size));
      scene->SetDataComponent(sorghum_entity, sorghum_transform);
      scene->SetParent(sorghum_entity, field);

      const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
      sorghum->sorghum_state_generator = sorghum_descriptor;
      const auto sorghum_state = ProjectManager::CreateTemporaryAsset<SorghumDescriptor>();
      sorghum_descriptor->Apply(sorghum_state, 0);
      sorghum->sorghum_descriptor = sorghum_state;
      size++;
      if (size >= size_limit)
        break;
    }

    TransformGraph::CalculateTransformGraphForDescendants(scene, field);
    return field;
  } else {
    EVOENGINE_ERROR("No sorghum layer!");
    return {};
  }
}
