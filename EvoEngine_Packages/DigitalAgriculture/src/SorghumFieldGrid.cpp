#include "SorghumFieldGrid.hpp"

#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Soil.hpp"
#include "Sorghum.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
#include "TransformGraph.hpp"



#ifdef CUDA_MODULE_SERVICE
#  include "RayTracerLayer.hpp"
#  include "TriangleIlluminationEstimator.hpp"
#endif

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

using namespace digital_agriculture_package;
using namespace eco_sys_lab_package;


namespace {
struct FieldIlluminationTestResult {
  Entity sorghum_entity{};
  float isolated_radiant_flux = 0.0f;
  float field_radiant_flux = 0.0f;
};

float CalculateSorghumRadiantFlux(const std::shared_ptr<Scene>& scene,
                                  const std::shared_ptr<SorghumLayer>& sorghum_layer,
                                  const Entity& sorghum_entity) {
  const auto estimator = scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(sorghum_entity).lock();
  estimator->PrepareLightProbeGroup();
  estimator->SampleLightProbeGroup(sorghum_layer->ray_properties, sorghum_layer->m_seed,
                                   sorghum_layer->push_distance);
  return glm::length(estimator->average_flux);
}

std::filesystem::path FindEvoEngineRoot(const std::filesystem::path& start_path) {
  auto current = start_path;
  while (!current.empty()) {
    if (std::filesystem::exists(current / "Resources") && std::filesystem::exists(current / "EvoEngine_SDK")) {
      return current;
    }
    const auto parent = current.parent_path();
    if (parent.empty() || parent == current) {
      break;
    }
    current = parent;
  }
  return {};
}

std::filesystem::path ResolveEvoEngineRoot() {
  if (const auto current_root = FindEvoEngineRoot(std::filesystem::current_path()); !current_root.empty()) {
    return current_root;
  }

  const auto project_path = ProjectManager::GetProjectPath();
  if (!project_path.empty()) {
    if (const auto project_root = FindEvoEngineRoot(project_path.parent_path()); !project_root.empty()) {
      return project_root;
    }
  }

  return std::filesystem::current_path();
}

glm::vec3 HeatColorFromRatio(const float ratio) {
  const glm::vec3 low_color(214.0f / 255.0f, 84.0f / 255.0f, 93.0f / 255.0f);
  const glm::vec3 high_color(46.0f / 255.0f, 173.0f / 255.0f, 114.0f / 255.0f);
  return glm::mix(low_color, high_color, glm::clamp(ratio, 0.0f, 1.0f));
}

std::string RgbToExcelHex(const glm::vec3& color) {
  const auto to_channel = [](const float value) -> int {
    return glm::clamp(static_cast<int>(std::round(value * 255.0f)), 0, 255);
  };

  std::stringstream ss;
  ss << "#" << std::uppercase << std::setfill('0') << std::hex << std::setw(2) << to_channel(color.r)
     << std::setw(2) << to_channel(color.g) << std::setw(2) << to_channel(color.b);
  return ss.str();
}

void WriteSpreadsheetStyles(std::ofstream& output_stream) {
  output_stream << "<Styles>\n";
  output_stream << "<Style ss:ID=\"Default\" ss:Name=\"Normal\">"
                   "<Alignment ss:Vertical=\"Center\" ss:Horizontal=\"Center\"/>"
                   "<Font ss:FontName=\"Calibri\" ss:Size=\"11\" ss:Color=\"#1F2933\"/>"
                   "</Style>\n";

  for (int style_index = 0; style_index <= 100; style_index++) {
    const auto color = RgbToExcelHex(HeatColorFromRatio(static_cast<float>(style_index) / 100.0f));
    output_stream << "<Style ss:ID=\"heat_" << style_index << "\">"
                  << "<Alignment ss:Vertical=\"Center\" ss:Horizontal=\"Center\"/>"
                  << "<Font ss:FontName=\"Calibri\" ss:Size=\"11\" ss:Color=\"#1F2933\"/>"
                  << "<Interior ss:Color=\"" << color << "\" ss:Pattern=\"Solid\"/>"
                  << "<Borders>"
                  << "<Border ss:Position=\"Bottom\" ss:LineStyle=\"Continuous\" ss:Weight=\"1\"/>"
                  << "<Border ss:Position=\"Left\" ss:LineStyle=\"Continuous\" ss:Weight=\"1\"/>"
                  << "<Border ss:Position=\"Right\" ss:LineStyle=\"Continuous\" ss:Weight=\"1\"/>"
                  << "<Border ss:Position=\"Top\" ss:LineStyle=\"Continuous\" ss:Weight=\"1\"/>"
                  << "</Borders>"
                  << "</Style>\n";
  }

  output_stream << "</Styles>\n";
}

float NormalizeFromRange(const float value, const float min_value, const float max_value) {
  const float range = max_value - min_value;
  if (range <= 1e-6f) {
    return 0.5f;
  }
  return glm::clamp((value - min_value) / range, 0.0f, 1.0f);
}
}  // namespace

void SorghumFieldGrid::RecreateField() {
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
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

void SorghumFieldGrid::CalculateIlluminationForField() {
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!sorghum_layer || !scene) {
    EVOENGINE_ERROR("Failed to calculate field illumination: no sorghum layer or active scene.");
    return;
  }

  sorghum_layer->CalculateIllumination();

  illumination_stats.total_area = 0.0f;
  illumination_stats.total_flux = glm::vec3(0.0f);
  illumination_stats.average_flux = glm::vec3(0.0f);

  for (const auto& sorghum_entity : sorghum_layer->processing_entities) {
    if (scene->GetParent(sorghum_entity) == GetOwner()) {
      const auto estimator = scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(sorghum_entity).lock();
      illumination_stats.total_area += estimator->total_area;
      illumination_stats.total_flux += estimator->total_flux;
    }
  }

  if (illumination_stats.total_area > 0.0f) {
    illumination_stats.average_flux = illumination_stats.total_flux / illumination_stats.total_area;
  }
}

bool SorghumFieldGrid::CalculateAndExportFieldIlluminationTest() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();

#ifndef CUDA_MODULE_SERVICE
  if (!scene || !sorghum_layer) {
    EVOENGINE_ERROR("Failed to export illumination test: missing active scene or SorghumLayer.");
    return false;
  }
  EVOENGINE_ERROR("Failed to export illumination test: requires CUDA_MODULE_SERVICE.");
  return false;
#else
  const auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  if (!scene || !sorghum_layer || !ray_tracer_layer) {
    EVOENGINE_ERROR("Failed to export illumination test: missing active scene, SorghumLayer, or RayTracerLayer.");
    return false;
  }

  std::vector<Entity> field_sorghum_entities;
  {
    const auto children = scene->GetChildren(GetOwner());
    field_sorghum_entities.reserve(children.size());
    for (const auto& child_entity : children) {
      if (scene->HasPrivateComponent<Sorghum>(child_entity)) {
        field_sorghum_entities.emplace_back(child_entity);
      }
    }
  }

  if (field_sorghum_entities.empty()) {
    EVOENGINE_ERROR("Failed to export illumination test: field contains no sorghum entities.");
    return false;
  }

  const int grid_size = static_cast<int>(std::round(std::sqrt(static_cast<float>(field_sorghum_entities.size()))));
  if (grid_size <= 0 || static_cast<size_t>(grid_size * grid_size) != field_sorghum_entities.size()) {
    EVOENGINE_ERROR("Failed to export illumination test: field plant count is not a perfect square.");
    return false;
  }

  std::vector<bool> original_enabled_states;
  original_enabled_states.reserve(field_sorghum_entities.size());
  for (const auto& sorghum_entity : field_sorghum_entities) {
    original_enabled_states.emplace_back(scene->IsEntityEnabled(sorghum_entity));
  }

  for (const auto& sorghum_entity : field_sorghum_entities) {
    scene->SetEnable(sorghum_entity, true);
  }
  ray_tracer_layer->UpdateScene(scene);

  std::vector<FieldIlluminationTestResult> test_results(field_sorghum_entities.size());
  for (size_t index = 0; index < field_sorghum_entities.size(); index++) {
    test_results[index].sorghum_entity = field_sorghum_entities[index];
    test_results[index].field_radiant_flux =
        CalculateSorghumRadiantFlux(scene, sorghum_layer, field_sorghum_entities[index]);
  }

  for (size_t index = 0; index < field_sorghum_entities.size(); index++) {
    for (size_t sibling_index = 0; sibling_index < field_sorghum_entities.size(); sibling_index++) {
      scene->SetEnable(field_sorghum_entities[sibling_index], sibling_index == index);
    }
    ray_tracer_layer->UpdateScene(scene);
    test_results[index].isolated_radiant_flux =
        CalculateSorghumRadiantFlux(scene, sorghum_layer, field_sorghum_entities[index]);
  }

  for (size_t index = 0; index < field_sorghum_entities.size(); index++) {
    scene->SetEnable(field_sorghum_entities[index], original_enabled_states[index]);
  }
  ray_tracer_layer->UpdateScene(scene);

  const auto output_folder = ResolveEvoEngineRoot() / "output";
  std::filesystem::create_directories(output_folder);
  const auto output_path =
      ProjectManager::GenerateNewAbsolutePath((output_folder / "field_illumination_test").string(), ".xml");

  std::ofstream output_stream(output_path, std::ofstream::out | std::ofstream::trunc);
  if (!output_stream.is_open()) {
    EVOENGINE_ERROR("Failed to export illumination test: unable to create " + output_path.string());
    return false;
  }

  float global_min_value = std::numeric_limits<float>::max();
  float global_max_value = std::numeric_limits<float>::lowest();
  for (const auto& result : test_results) {
    global_min_value = glm::min(global_min_value, glm::min(result.field_radiant_flux, result.isolated_radiant_flux));
    global_max_value = glm::max(global_max_value, glm::max(result.field_radiant_flux, result.isolated_radiant_flux));
  }
  EVOENGINE_LOG("Heatmap normalization uses shared range across " +
                std::to_string(test_results.size() * static_cast<size_t>(2)) + " values.");

  output_stream << "<?xml version=\"1.0\"?>\n";
  output_stream << "<?mso-application progid=\"Excel.Sheet\"?>\n";
  output_stream << "<Workbook xmlns=\"urn:schemas-microsoft-com:office:spreadsheet\" "
                   "xmlns:o=\"urn:schemas-microsoft-com:office:office\" "
                   "xmlns:x=\"urn:schemas-microsoft-com:office:excel\" "
                   "xmlns:ss=\"urn:schemas-microsoft-com:office:spreadsheet\" "
                   "xmlns:html=\"http://www.w3.org/TR/REC-html40\">\n";

  WriteSpreadsheetStyles(output_stream);

  const int expanded_row_count = grid_size * 2 + 1;
  output_stream << "<Worksheet ss:Name=\"FieldIlluminationTest\">\n";
  output_stream << "<Table ss:ExpandedColumnCount=\"" << grid_size << "\" ss:ExpandedRowCount=\""
                << expanded_row_count
                << "\" x:FullColumns=\"1\" x:FullRows=\"1\" ss:DefaultColumnWidth=\"96\" "
                   "ss:DefaultRowHeight=\"24\">\n";

  for (int column_index = 0; column_index < grid_size; column_index++) {
    output_stream << "<Column ss:Width=\"96\"/>\n";
  }

  output_stream << std::fixed << std::setprecision(6);

  auto write_grid_row = [&](const int row_index, const bool full_field_run) {
    output_stream << "<Row>";
    for (int col_index = 0; col_index < grid_size; col_index++) {
      const size_t plant_index = static_cast<size_t>(row_index * grid_size + col_index);
      const float value = full_field_run ? test_results[plant_index].field_radiant_flux
                                         : test_results[plant_index].isolated_radiant_flux;
      const float normalized = NormalizeFromRange(value, global_min_value, global_max_value);
      const int style_id = glm::clamp(static_cast<int>(std::round(normalized * 100.0f)), 0, 100);

      output_stream << "<Cell ss:StyleID=\"heat_" << style_id << "\"><Data ss:Type=\"Number\">" << value
                    << "</Data></Cell>";
    }
    output_stream << "</Row>\n";
  };

  for (int row_index = 0; row_index < grid_size; row_index++) {
    write_grid_row(row_index, true);
  }

  output_stream << "<Row ss:Height=\"12\"></Row>\n";

  for (int row_index = 0; row_index < grid_size; row_index++) {
    write_grid_row(row_index, false);
  }

  output_stream << "</Table>\n";
  output_stream << "</Worksheet>\n";
  output_stream << "</Workbook>\n";
  output_stream.close();

  EVOENGINE_LOG("Exported field illumination test workbook to " + output_path.string());
  return true;
#endif
}

bool SorghumFieldGrid::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
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
    CalculateIlluminationForField();
  }

  if (ImGui::Button("Calculate and export field illumination test")) {
    CalculateAndExportFieldIlluminationTest();
  }

  ImGui::Text("%s", ("Surface area: " + std::to_string(illumination_stats.total_area)).c_str());
  ImGui::Text("%s", ("Total energy: " + std::to_string(glm::length(illumination_stats.total_flux))).c_str());
  ImGui::Text("%s", ("Radiant flux: " + std::to_string(glm::length(illumination_stats.average_flux))).c_str());

  return changed;
}

void SorghumFieldGrid::CollectAssetRef(std::vector<AssetRef>& list) {
  if (sorghum_field_asset.Get<SorghumField>())
    list.push_back(sorghum_field_asset);
}
