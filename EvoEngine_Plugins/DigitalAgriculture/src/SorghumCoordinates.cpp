#include "SorghumCoordinates.hpp"
#include "SorghumLayer.hpp"
#include "SorghumDescriptorGenerator.hpp"
#include "TransformGraph.hpp"

using namespace digital_agriculture_plugin;

void SorghumCoordinates::Apply(const std::shared_ptr<SorghumField>& sorghum_field) {
  sorghum_field->matrices.clear();
  for (const auto& position : positions) {
    if (position.x < sample_x.x || position.y < sample_y.x || position.x > sample_x.y || position.y > sample_y.y)
      continue;
    auto pos = glm::vec3(position.x - sample_x.x, 0, position.y - sample_y.x) * factor;
    auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), rotation_variance))));
    sorghum_field->matrices.emplace_back(sorghum_state_generator,
                                         glm::translate(pos) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(1.0f)));
  }
}

void SorghumCoordinates::Apply(const std::shared_ptr<SorghumField>& sorghum_field, glm::dvec2& offset, const unsigned i,
                               const float radius, const float position_variance) {
  sorghum_field->matrices.clear();
  const glm::dvec2 center = offset = positions[i];
  // Create sorghums here.
  for (const auto& position : positions) {
    if (glm::distance(center, position) > radius)
      continue;
    const glm::dvec2 pos_offset = glm::gaussRand(glm::dvec2(.0f), glm::dvec2(position_variance));
    auto pos = glm::vec3(position.x - center.x + pos_offset.x, 0, position.y - center.y + pos_offset.y) * factor;
    auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), rotation_variance))));
    sorghum_field->matrices.emplace_back(sorghum_state_generator,
                                         glm::translate(pos) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(1.0f)));
  }
}

bool SorghumCoordinates::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  editor_layer->DragAndDropButton<SorghumDescriptorGenerator>(sorghum_state_generator, "SorghumDescriptorGenerator");
  ImGui::Text("Available count: %d", positions.size());
  ImGui::DragFloat("Distance factor", &factor, 0.01f, 0.0f, 20.0f);
  ImGui::DragFloat3("Rotation variance", &rotation_variance.x, 0.01f, 0.0f, 180.0f);

  ImGui::Text("X range: [%.3f, %.3f]", x_range.x, x_range.y);
  ImGui::Text("Y Range: [%.3f, %.3f]", y_range.x, y_range.y);

  if (ImGui::DragScalarN("Width range", ImGuiDataType_Double, &sample_x.x, 2, 0.1f)) {
    sample_x.x = glm::min(sample_x.x, sample_x.y);
    sample_x.y = glm::max(sample_x.x, sample_x.y);
  }
  if (ImGui::DragScalarN("Length Range", ImGuiDataType_Double, &sample_y.x, 2, 0.1f)) {
    sample_y.x = glm::min(sample_y.x, sample_y.y);
    sample_y.y = glm::max(sample_y.x, sample_y.y);
  }

  static int index = 200;
  static float radius = 2.5f;
  ImGui::DragInt("Index", &index);
  ImGui::DragFloat("Radius", &radius);
  static AssetRef temp_field;
  if (editor_layer->DragAndDropButton<SorghumField>(temp_field, "Apply to SorghumField")) {
    if (const auto field = temp_field.Get<SorghumField>()) {
      glm::dvec2 offset;
      Apply(field, offset, index, radius);
      temp_field.Clear();
    }
  }
  FileUtils::OpenFile(
      "Load Positions", "Position list", {".txt"},
      [this](const std::filesystem::path& path) {
        ImportFromFile(path);
      },
      false);

  return changed;
}
void SorghumCoordinates::Serialize(YAML::Emitter& out) const {
  sorghum_state_generator.Save("sorghum_state_generator", out);
  out << YAML::Key << "rotation_variance" << YAML::Value << rotation_variance;
  out << YAML::Key << "sample_x" << YAML::Value << sample_x;
  out << YAML::Key << "sample_y" << YAML::Value << sample_y;
  out << YAML::Key << "x_range" << YAML::Value << x_range;
  out << YAML::Key << "y_range" << YAML::Value << y_range;
  out << YAML::Key << "factor" << YAML::Value << factor;
  SaveListAsBinary<glm::dvec2>("positions", positions, out);
}
void SorghumCoordinates::Deserialize(const YAML::Node& in) {
  sorghum_state_generator.Load("sorghum_state_generator", in);
  rotation_variance = in["rotation_variance"].as<glm::vec3>();
  if (in["sample_x"])
    sample_x = in["sample_x"].as<glm::dvec2>();
  if (in["sample_y"])
    sample_y = in["sample_y"].as<glm::dvec2>();
  if (in["x_range"])
    x_range = in["x_range"].as<glm::dvec2>();
  if (in["y_range"])
    y_range = in["y_range"].as<glm::dvec2>();
  factor = in["factor"].as<float>();
  LoadListFromBinary<glm::dvec2>("positions", positions, in);
}
void SorghumCoordinates::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(sorghum_state_generator);
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
