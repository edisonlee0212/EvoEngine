#pragma once

#include <Vertex.hpp>
#include <glm/glm.hpp>
#include <vector>

#include "Noises.hpp"
using namespace evo_engine;

namespace eco_sys_lab {
class HeightField : public IAsset {
 public:
  Noise2D noises_2d;
  int precision_level = 2;
  [[nodiscard]] float GetValue(const glm::vec2& position) const;
  void RandomOffset(float min, float max);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unitSize, std::vector<Vertex>& vertices,
                    std::vector<glm::uvec3>& triangles, float xDepth = 1.0f, float zDepth = 1.0f) const;
};
}  // namespace eco_sys_lab