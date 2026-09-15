#include "EcoSysLabSerializationAdapters.hpp"
using namespace eco_sys_lab_package;

float HeightField::GetValue(const glm::vec2& position) const {
  return noises_graph.GetValue(position + position_offset);
}

void HeightField::RandomOffset(const float min, const float max) {
  position_offset = glm::vec2(glm::linearRand(min, max), glm::linearRand(min, max));
}

void eco_sys_lab_package::SerializeHeightField(YAML::Emitter& out, const HeightField& target) {
  out << YAML::Key << "precision_level" << YAML::Value << target.precision_level;
  out << YAML::Key << "position_offset" << YAML::Value << target.position_offset;
  target.noises_graph.Save("noises_graph", out);
}

void eco_sys_lab_package::DeserializeHeightField(const YAML::Node& in, HeightField& target) {
  if (in["precision_level"])
    target.precision_level = in["precision_level"].as<int>();
  if (in["position_offset"])
    target.position_offset = in["position_offset"].as<glm::vec2>();
  target.noises_graph.Load("noises_graph", in);
}

void HeightField::GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unit_size,
                               std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles, float x_depth,
                               float z_depth) const {
  for (unsigned i = 0; i < resolution.x * precision_level; i++) {
    for (unsigned j = 0; j < resolution.y * precision_level; j++) {
      Vertex archetype;
      archetype.position.x = start.x + unit_size * i / precision_level;
      archetype.position.z = start.y + unit_size * j / precision_level;
      archetype.position.y = GetValue({archetype.position.x, archetype.position.z});
      archetype.tex_coord = glm::vec2(static_cast<float>(i) / (resolution.x * precision_level),
                                      static_cast<float>(j) / (resolution.y * precision_level));
      vertices.push_back(archetype);
    }
  }

  for (int i = 0; i < resolution.x * precision_level - 1; i++) {
    for (int j = 0; j < resolution.y * precision_level - 1; j++) {
      if (static_cast<float>(i) / (resolution.x * precision_level - 2) > (1.0 - z_depth) &&
          static_cast<float>(j) / (resolution.y * precision_level - 2) < x_depth)
        continue;
      const int n = resolution.x * precision_level;
      triangles.emplace_back(i + j * n, i + 1 + j * n, i + (j + 1) * n);
      triangles.emplace_back(i + 1 + (j + 1) * n, i + (j + 1) * n, i + 1 + j * n);
    }
  }
}
