#pragma once

#include <Mesh.hpp>

#include "IVolume.hpp"
#include "Skeleton.hpp"
#include "TreeModel.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
struct RadialBoundingVolumeSlice {
  float m_maxDistance;
};

class RadialBoundingVolume : public IVolume {
  std::vector<std::shared_ptr<Mesh>> m_boundMeshes;
  bool m_meshGenerated = false;

  void CalculateSizes();

 public:
  glm::vec4 m_displayColor = glm::vec4(0.0f, 0.0f, 1.0f, 0.5f);
  float m_offset = 0.1f;
  [[nodiscard]] glm::vec3 GetRandomPoint() override;

  [[nodiscard]] glm::ivec2 SelectSlice(const glm::vec3& position) const;
  [[nodiscard]] glm::vec3 TipPosition(int layer, int slice) const;
  float m_maxHeight = 0.0f;
  float m_maxRadius = 0.0f;
  void GenerateMesh();

  void FormEntity();

  std::string AsString();
  void FromString(const std::string& string);
  void ExportAsObj(const std::string& filename);

  float m_displayScale = 0.2f;
  int m_layerAmount = 8;
  int m_sectorAmount = 8;
  std::vector<std::vector<RadialBoundingVolumeSlice>> m_layers;
  std::vector<std::pair<float, std::vector<float>>> m_sizes;
  float m_totalSize = 0;
  void CalculateVolume(const std::vector<glm::vec3>& points);

  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  void ResizeVolumes();

  bool InVolume(const GlobalTransform& globalTransform, const glm::vec3& position) override;

  bool InVolume(const glm::vec3& position) override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;

  void Augmentation(float value);
};
}  // namespace eco_sys_lab
