#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
#include "TransformGraph.hpp"

using namespace digital_agriculture_package;

void SorghumCoordinates::Apply(const std::shared_ptr<SorghumField>& sorghum_field) {
  Apply(*sorghum_field);
}

void SorghumCoordinates::Apply(SorghumField& sorghum_field) {
  sorghum_field.matrices.clear();
  for (const auto& position : positions) {
    if (position.x < sample_x.x || position.y < sample_y.x || position.x > sample_x.y || position.y > sample_y.y)
      continue;
    auto pos = glm::vec3(position.x - sample_x.x, 0, position.y - sample_y.x) * factor;
    auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), rotation_variance))));
    sorghum_field.matrices.emplace_back(sorghum_generator,
                                        glm::translate(pos) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(1.0f)));
  }
}

void SorghumCoordinates::Apply(const std::shared_ptr<SorghumField>& sorghum_field, glm::dvec2& offset, const unsigned i,
                               const float radius, const float position_variance) {
  Apply(*sorghum_field, offset, i, radius, position_variance);
}

void SorghumCoordinates::Apply(SorghumField& sorghum_field, glm::dvec2& offset, const unsigned i, const float radius,
                               const float position_variance) {
  sorghum_field.matrices.clear();
  const glm::dvec2 center = offset = positions[i];
  // Create sorghums here.
  for (const auto& position : positions) {
    if (glm::distance(center, position) > radius)
      continue;
    const glm::dvec2 pos_offset = glm::gaussRand(glm::dvec2(.0f), glm::dvec2(position_variance));
    auto pos = glm::vec3(position.x - center.x + pos_offset.x, 0, position.y - center.y + pos_offset.y) * factor;
    auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), rotation_variance))));
    sorghum_field.matrices.emplace_back(sorghum_generator,
                                        glm::translate(pos) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(1.0f)));
  }
}

bool digital_agriculture_package::InspectSorghumCoordinates(InspectorContext& context,
                                                            SorghumCoordinates& coordinates) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (editor_layer->DragAndDropButton<SorghumGenerator>(coordinates.sorghum_generator, "SorghumGenerator")) {
    changed = true;
  }
  ImGui::Text("Available count: %d", coordinates.positions.size());
  ImGui::DragFloat("Distance factor", &coordinates.factor, 0.01f, 0.0f, 20.0f);
  ImGui::DragFloat3("Rotation variance", &coordinates.rotation_variance.x, 0.01f, 0.0f, 180.0f);

  ImGui::Text("X range: [%.3f, %.3f]", coordinates.x_range.x, coordinates.x_range.y);
  ImGui::Text("Y Range: [%.3f, %.3f]", coordinates.y_range.x, coordinates.y_range.y);

  if (ImGui::DragScalarN("Width range", ImGuiDataType_Double, &coordinates.sample_x.x, 2, 0.1f)) {
    coordinates.sample_x.x = glm::min(coordinates.sample_x.x, coordinates.sample_x.y);
    coordinates.sample_x.y = glm::max(coordinates.sample_x.x, coordinates.sample_x.y);
  }
  if (ImGui::DragScalarN("Length Range", ImGuiDataType_Double, &coordinates.sample_y.x, 2, 0.1f)) {
    coordinates.sample_y.x = glm::min(coordinates.sample_y.x, coordinates.sample_y.y);
    coordinates.sample_y.y = glm::max(coordinates.sample_y.x, coordinates.sample_y.y);
  }

  FileUtils::OpenFile(
      "Load Positions", "Position list", {".txt"},
      [&coordinates](const std::filesystem::path& path) {
        coordinates.ImportFromFile(path);
      },
      false);

  return changed;
}
void digital_agriculture_package::SerializeSorghumCoordinates(YAML::Emitter& out, const SorghumCoordinates& target) {
  target.sorghum_generator.Save("sorghum_generator", out);
  out << YAML::Key << "rotation_variance" << YAML::Value << target.rotation_variance;
  out << YAML::Key << "sample_x" << YAML::Value << target.sample_x;
  out << YAML::Key << "sample_y" << YAML::Value << target.sample_y;
  out << YAML::Key << "x_range" << YAML::Value << target.x_range;
  out << YAML::Key << "y_range" << YAML::Value << target.y_range;
  out << YAML::Key << "factor" << YAML::Value << target.factor;
  SaveListAsBinary<glm::dvec2>("positions", target.positions, out);
}
void digital_agriculture_package::DeserializeSorghumCoordinates(const YAML::Node& in, SorghumCoordinates& target) {
  target.sorghum_generator.Load("sorghum_generator", in);
  target.rotation_variance = in["rotation_variance"].as<glm::vec3>();
  if (in["sample_x"])
    target.sample_x = in["sample_x"].as<glm::dvec2>();
  if (in["sample_y"])
    target.sample_y = in["sample_y"].as<glm::dvec2>();
  if (in["x_range"])
    target.x_range = in["x_range"].as<glm::dvec2>();
  if (in["y_range"])
    target.y_range = in["y_range"].as<glm::dvec2>();
  target.factor = in["factor"].as<float>();
  LoadListFromBinary<glm::dvec2>("positions", target.positions, in);
}
void SorghumCoordinates::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(sorghum_generator);
}
void SorghumCoordinates::ImportFromFile(const std::filesystem::path& path) {
  std::ifstream ifs;
  ifs.open(path.c_str());
  EVOENGINE_LOG("Loading from " + path.string());
  if (ifs.is_open()) {
    int amount;
    ifs >> amount;
    positions.resize(amount);
    x_range = glm::vec2(99999999, -99999999);
    y_range = glm::vec2(99999999, -99999999);
    for (auto& position : positions) {
      ifs >> position.x >> position.y;
      x_range.x = glm::min(position.x, x_range.x);
      x_range.y = glm::max(position.x, x_range.y);
      y_range.x = glm::min(position.y, y_range.x);
      y_range.y = glm::max(position.y, y_range.y);
    }
  }
}
