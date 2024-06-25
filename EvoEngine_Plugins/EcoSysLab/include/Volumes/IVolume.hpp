#pragma once

#include <Transform.hpp>

using namespace evo_engine;
namespace eco_sys_lab {
class IVolume : public IAsset {
 public:
  virtual glm::vec3 GetRandomPoint() {
    return glm::vec3(0.0f);
  }

  virtual bool InVolume(const GlobalTransform& global_transform, const glm::vec3& position);

  virtual bool InVolume(const glm::vec3& position);
  virtual void InVolume(const GlobalTransform& global_transform, const std::vector<glm::vec3>& positions,
                        std::vector<bool>& results);

  virtual void InVolume(const std::vector<glm::vec3>& positions, std::vector<bool>& results);
};
class SphericalVolume : public IVolume {
 public:
  glm::vec3 m_radius = glm::vec3(1.0f);
  glm::vec3 GetRandomPoint() override;
  bool InVolume(const GlobalTransform& global_transform, const glm::vec3& position) override;
  bool InVolume(const glm::vec3& position) override;
};
}  // namespace eco_sys_lab
