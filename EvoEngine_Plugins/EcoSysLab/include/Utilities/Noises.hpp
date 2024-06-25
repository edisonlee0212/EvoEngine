#pragma once

#include <glm/glm.hpp>
#include <vector>

using namespace evo_engine;

namespace eco_sys_lab {
enum class NoiseType {
  Constant,
  Linear,
  Simplex,
  Perlin,
};

struct NoiseDescriptor {
  unsigned type = 0;
  float frequency = 0.1f;
  float intensity = 1.0f;
  float multiplier = 1.0f;
  float min = -10;
  float max = 10;
  float offset = 0.0f;
  glm::vec3 shift = glm::vec3(0.0f);
  bool ridgid = false;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};
class Noise2D {
 public:
  glm::vec2 min_max = glm::vec2(-1000, 1000);

  std::vector<NoiseDescriptor> noise_descriptors;
  Noise2D();
  bool OnInspect();
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
  void RandomOffset(float min, float max);
  [[nodiscard]] float GetValue(const glm::vec2& position) const;
};

class Noise3D {
 public:
  glm::vec2 min_max = glm::vec2(-1000, 1000);
  std::vector<NoiseDescriptor> noise_descriptors;
  Noise3D();
  bool OnInspect();
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);

  [[nodiscard]] float GetValue(const glm::vec3& position) const;
};
}  // namespace eco_sys_lab