#include "SorghumFieldGrid.hpp"

#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Soil.hpp"
#include "Sorghum.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
#include "TransformGraph.hpp"
#include "TriangleIlluminationEstimator.hpp"

using namespace digital_agriculture_plugin;
using namespace eco_sys_lab_plugin;

void SorghumFieldGrid::RecreateField() {
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  if (!sorghum_layer) {
    EVOENGINE_ERROR("No sorghum layer!");
    return;
  }
  const auto field_asset = sorghum_field_asset.Get<SorghumField>();
  if (!field_asset || field_asset->matrices.empty()) {
    EVOENGINE_ERROR("No SorghumField asset or empty matrices!");
    return;
  }

  const auto scene = GetScene();
  const auto field_entity = GetOwner();

  // Delete existing children.
  const auto children = scene->GetChildren(field_entity);
  for (const auto& child : children) {
    scene->DeleteEntity(child);
  }

  // Collect unique generators from the asset.
  std::vector<std::shared_ptr<SorghumGenerator>> generators;
  for (auto& entry : field_asset->matrices) {
    auto gen = entry.first.Get<SorghumGenerator>();
    if (gen)
      generators.push_back(gen);
  }
  if (generators.empty()) {
    EVOENGINE_ERROR("No valid SorghumGenerators in field asset!");
    return;
  }

  // Get height field if available.
  std::shared_ptr<HeightField> height_field{};
  {
    std::shared_ptr<Soil> soil;
    if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
      soil = soil_candidate.lock();
    if (soil) {
      if (const auto soil_descriptor = soil->soil_descriptor_ref.Get<SoilDescriptor>()) {
        height_field = soil_descriptor->height_field.Get<HeightField>();
      }
    }
  }

  // Generate grid positions and create sorghum entities.
  const glm::vec2 start_point(
      (columns - 1) * column_spacing * 0.5f,
      (rows - 1) * row_spacing * 0.5f);

  int size = 0;
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < columns; c++) {
      if (size >= size_limit)
        break;

      float actual_row_spacing = row_spacing + glm::gaussRand(0.0f, row_spacing_std);
      float actual_col_spacing = column_spacing + glm::gaussRand(0.0f, column_spacing_std);

      glm::vec3 position(
          -start_point.x + c * actual_col_spacing,
          0.0f,
          -start_point.y + r * actual_row_spacing);

      if (height_field)
        position.y = height_field->GetValue({position.x, position.z}) - 0.01f;

      // Cycle through available generators.
      const auto& generator = generators[size % generators.size()];

      Entity sorghum_entity = generator->CreateEntity(size);
      auto sorghum_transform = scene->GetDataComponent<Transform>(sorghum_entity);

      Transform transform{};
      transform.SetPosition(position);
      transform.SetScale(glm::vec3(sorghum_size));
      sorghum_transform.value = transform.value;
      scene->SetDataComponent(sorghum_entity, sorghum_transform);
      scene->SetParent(sorghum_entity, field_entity);

      const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
      sorghum->sorghum_generator = generator;
      const auto sorghum_descriptor = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
      generator->Apply(sorghum_descriptor, base_seed + size);
      sorghum->sorghum_descriptor = generator;

      size++;
    }
    if (size >= size_limit)
      break;
  }

  TransformGraph::CalculateTransformGraphForDescendants(scene, field_entity);
}

bool SorghumFieldGrid::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (editor_layer->DragAndDropButton<SorghumField>(sorghum_field_asset, "SorghumField"))
    changed = true;

  if (ImGui::DragInt("Rows", &rows, 1, 1, 1000))
    changed = true;
  if (ImGui::DragInt("Columns", &columns, 1, 1, 1000))
    changed = true;
  if (ImGui::DragFloat("Row spacing", &row_spacing, 0.01f, 0.01f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Column spacing", &column_spacing, 0.01f, 0.01f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Row spacing std", &row_spacing_std, 0.001f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Column spacing std", &column_spacing_std, 0.001f, 0.0f, 10.0f))
    changed = true;

  ImGui::Separator();
  if (ImGui::DragFloat("Sorghum size", &sorghum_size, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::DragInt("Size limit", &size_limit, 1, 1, 10000))
    changed = true;
  if (ImGui::DragScalar("Base seed", ImGuiDataType_U32, &base_seed))
    changed = true;

  if (ImGui::Button("Recreate field")) {
    RecreateField();
  }

  if (ImGui::Button("Calculate illumination for the field")) {
    auto sorghum_layer = Application::GetLayer<SorghumLayer>();
    auto scene = Application::GetActiveScene();
    sorghum_layer->CalculateIllumination();

    illumination_stats.total_area = 0.0f;
    illumination_stats.total_flux = glm::vec3(0.0f);
    illumination_stats.average_flux = glm::vec3(0.0f);

    for (const auto & sorghum_entity : sorghum_layer->processing_entities){
      if (scene->GetParent(sorghum_entity) == this->GetOwner()) {
        const auto estimator = scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(sorghum_entity).lock();
        illumination_stats.total_area += estimator->total_area;
        illumination_stats.total_flux += estimator->total_flux;
      }
      


    }
    illumination_stats.average_flux = illumination_stats.total_flux / illumination_stats.total_area;
  }
  ImGui::Text("%s", ("Surface area: " + std::to_string(illumination_stats.total_area)).c_str());
  ImGui::Text("%s", ("Total energy: " + std::to_string(glm::length(illumination_stats.total_flux))).c_str());
  ImGui::Text("%s", ("Radiant flux: " + std::to_string(glm::length(illumination_stats.average_flux))).c_str());

  return changed;
}

void SorghumFieldGrid::Serialize(YAML::Emitter& out) const {
  sorghum_field_asset.Save("sorghum_field_asset", out);
  out << YAML::Key << "rows" << YAML::Value << rows;
  out << YAML::Key << "columns" << YAML::Value << columns;
  out << YAML::Key << "row_spacing" << YAML::Value << row_spacing;
  out << YAML::Key << "column_spacing" << YAML::Value << column_spacing;
  out << YAML::Key << "row_spacing_std" << YAML::Value << row_spacing_std;
  out << YAML::Key << "column_spacing_std" << YAML::Value << column_spacing_std;
  out << YAML::Key << "sorghum_size" << YAML::Value << sorghum_size;
  out << YAML::Key << "size_limit" << YAML::Value << size_limit;
  out << YAML::Key << "base_seed" << YAML::Value << base_seed;
}

void SorghumFieldGrid::Deserialize(const YAML::Node& in) {
  sorghum_field_asset.Load("sorghum_field_asset", in);
  if (in["rows"]) rows = in["rows"].as<int>();
  if (in["columns"]) columns = in["columns"].as<int>();
  if (in["row_spacing"]) row_spacing = in["row_spacing"].as<float>();
  if (in["column_spacing"]) column_spacing = in["column_spacing"].as<float>();
  if (in["row_spacing_std"]) row_spacing_std = in["row_spacing_std"].as<float>();
  if (in["column_spacing_std"]) column_spacing_std = in["column_spacing_std"].as<float>();
  if (in["sorghum_size"]) sorghum_size = in["sorghum_size"].as<float>();
  if (in["size_limit"]) size_limit = in["size_limit"].as<int>();
  if (in["base_seed"]) base_seed = in["base_seed"].as<uint32_t>();
}

void SorghumFieldGrid::CollectAssetRef(std::vector<AssetRef>& list) {
  if (sorghum_field_asset.Get<SorghumField>())
    list.push_back(sorghum_field_asset);
}
