#include "SorghumCoordinates.hpp"
#include "SorghumLayer.hpp"
#include "SorghumStateGenerator.hpp"
#include "TransformGraph.hpp"

using namespace eco_sys_lab;

void SorghumCoordinates::Apply(const std::shared_ptr<SorghumField>& sorghumField) {
  sorghumField->m_matrices.clear();
  for (auto& position : m_positions) {
    if (position.x < m_sampleX.x || position.y < m_sampleY.x || position.x > m_sampleX.y || position.y > m_sampleY.y)
      continue;
    auto pos = glm::vec3(position.x - m_sampleX.x, 0, position.y - m_sampleY.x) * m_factor;
    auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), m_rotationVariance))));
    sorghumField->m_matrices.emplace_back(m_sorghumStateGenerator,
                                          glm::translate(pos) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(1.0f)));
  }
}

void SorghumCoordinates::Apply(const std::shared_ptr<SorghumField>& sorghumField, glm::dvec2& offset, const unsigned i,
                               const float radius, const float positionVariance) {
  sorghumField->m_matrices.clear();
  glm::dvec2 center = offset = m_positions[i];
  // Create sorghums here.
  for (const auto& position : m_positions) {
    if (glm::distance(center, position) > radius)
      continue;
    glm::dvec2 posOffset = glm::gaussRand(glm::dvec2(.0f), glm::dvec2(positionVariance));
    auto pos = glm::vec3(position.x - center.x + posOffset.x, 0, position.y - center.y + posOffset.y) * m_factor;
    auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), m_rotationVariance))));
    sorghumField->m_matrices.emplace_back(m_sorghumStateGenerator,
                                          glm::translate(pos) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(1.0f)));
  }
}

bool SorghumCoordinates::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  editorLayer->DragAndDropButton<SorghumStateGenerator>(m_sorghumStateGenerator, "SorghumStateGenerator");
  ImGui::Text("Available count: %d", m_positions.size());
  ImGui::DragFloat("Distance factor", &m_factor, 0.01f, 0.0f, 20.0f);
  ImGui::DragFloat3("Rotation variance", &m_rotationVariance.x, 0.01f, 0.0f, 180.0f);

  ImGui::Text("X range: [%.3f, %.3f]", m_xRange.x, m_xRange.y);
  ImGui::Text("Y Range: [%.3f, %.3f]", m_yRange.x, m_yRange.y);

  if (ImGui::DragScalarN("Width range", ImGuiDataType_Double, &m_sampleX.x, 2, 0.1f)) {
    m_sampleX.x = glm::min(m_sampleX.x, m_sampleX.y);
    m_sampleX.y = glm::max(m_sampleX.x, m_sampleX.y);
  }
  if (ImGui::DragScalarN("Length Range", ImGuiDataType_Double, &m_sampleY.x, 2, 0.1f)) {
    m_sampleY.x = glm::min(m_sampleY.x, m_sampleY.y);
    m_sampleY.y = glm::max(m_sampleY.x, m_sampleY.y);
  }

  static int index = 200;
  static float radius = 2.5f;
  ImGui::DragInt("Index", &index);
  ImGui::DragFloat("Radius", &radius);
  static AssetRef tempField;
  if (editorLayer->DragAndDropButton<SorghumField>(tempField, "Apply to SorghumField")) {
    if (const auto field = tempField.Get<SorghumField>()) {
      glm::dvec2 offset;
      Apply(field, offset, index, radius);
      tempField.Clear();
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
  m_sorghumStateGenerator.Save("m_sorghumStateGenerator", out);
  out << YAML::Key << "m_rotationVariance" << YAML::Value << m_rotationVariance;
  out << YAML::Key << "m_sampleX" << YAML::Value << m_sampleX;
  out << YAML::Key << "m_sampleY" << YAML::Value << m_sampleY;
  out << YAML::Key << "m_xRange" << YAML::Value << m_xRange;
  out << YAML::Key << "m_yRange" << YAML::Value << m_yRange;
  out << YAML::Key << "m_factor" << YAML::Value << m_factor;
  SaveListAsBinary<glm::dvec2>("m_positions", m_positions, out);
}
void SorghumCoordinates::Deserialize(const YAML::Node& in) {
  m_sorghumStateGenerator.Load("m_sorghumStateGenerator", in);
  m_rotationVariance = in["m_rotationVariance"].as<glm::vec3>();
  if (in["m_sampleX"])
    m_sampleX = in["m_sampleX"].as<glm::dvec2>();
  if (in["m_sampleY"])
    m_sampleY = in["m_sampleY"].as<glm::dvec2>();
  if (in["m_xRange"])
    m_xRange = in["m_xRange"].as<glm::dvec2>();
  if (in["m_yRange"])
    m_yRange = in["m_yRange"].as<glm::dvec2>();
  m_factor = in["m_factor"].as<float>();
  LoadListFromBinary<glm::dvec2>("m_positions", m_positions, in);
}
void SorghumCoordinates::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(m_sorghumStateGenerator);
}
void SorghumCoordinates::ImportFromFile(const std::filesystem::path& path) {
  std::ifstream ifs;
  ifs.open(path.c_str());
  EVOENGINE_LOG("Loading from " + path.string());
  if (ifs.is_open()) {
    int amount;
    ifs >> amount;
    m_positions.resize(amount);
    m_xRange = glm::vec2(99999999, -99999999);
    m_yRange = glm::vec2(99999999, -99999999);
    for (auto& position : m_positions) {
      ifs >> position.x >> position.y;
      m_xRange.x = glm::min(position.x, m_xRange.x);
      m_xRange.y = glm::max(position.x, m_xRange.y);
      m_yRange.x = glm::min(position.y, m_yRange.x);
      m_yRange.y = glm::max(position.y, m_yRange.y);
    }
  }
}
