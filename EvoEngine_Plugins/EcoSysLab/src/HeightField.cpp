#include "HeightField.hpp"

#include "glm/gtc/noise.hpp"

using namespace eco_sys_lab;

float HeightField::GetValue(const glm::vec2& position) const {
  float retVal = 0.0f;
  if (position.x < 0) {
    // retVal += glm::max(position.x, -5.0f);
  }
  retVal += noises_2d.GetValue(position);
  return retVal;
}

void HeightField::RandomOffset(const float min, const float max) {
  noises_2d.RandomOffset(min, max);
}

bool HeightField::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  changed = ImGui::DragInt("Precision level", &precision_level) || changed;
  changed = noises_2d.OnInspect() | changed;
  return changed;
}

void HeightField::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "precision_level" << YAML::Value << precision_level;
  noises_2d.Save("noises_2d", out);
}

void HeightField::Deserialize(const YAML::Node& in) {
  if (in["precision_level"])
    precision_level = in["precision_level"].as<int>();
  noises_2d.Load("noises_2d", in);
}

void HeightField::GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unitSize,
                               std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles, float xDepth,
                               float zDepth) const {
  for (unsigned i = 0; i < resolution.x * precision_level; i++) {
    for (unsigned j = 0; j < resolution.y * precision_level; j++) {
      Vertex archetype;
      archetype.position.x = start.x + unitSize * i / precision_level;
      archetype.position.z = start.y + unitSize * j / precision_level;
      archetype.position.y = GetValue({archetype.position.x, archetype.position.z});
      archetype.tex_coord = glm::vec2(static_cast<float>(i) / (resolution.x * precision_level),
                                       static_cast<float>(j) / (resolution.y * precision_level));
      vertices.push_back(archetype);
    }
  }

  for (int i = 0; i < resolution.x * precision_level - 1; i++) {
    for (int j = 0; j < resolution.y * precision_level - 1; j++) {
      if (static_cast<float>(i) / (resolution.x * precision_level - 2) > (1.0 - zDepth) &&
          static_cast<float>(j) / (resolution.y * precision_level - 2) < xDepth)
        continue;
      const int n = resolution.x * precision_level;
      triangles.emplace_back(i + j * n, i + 1 + j * n, i + (j + 1) * n);
      triangles.emplace_back(i + 1 + (j + 1) * n, i + (j + 1) * n, i + 1 + j * n);
    }
  }
}
