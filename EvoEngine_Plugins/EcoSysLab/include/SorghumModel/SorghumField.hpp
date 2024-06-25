#pragma once
using namespace evo_engine;
namespace eco_sys_lab {
class SorghumFieldPatch {
 public:
  glm::vec2 m_gridDistance = glm::vec2(1.0f);
  glm::vec2 m_positionOffsetMean = glm::vec2(0.f);
  glm::vec2 m_positionOffsetVariance = glm::vec2(0.0f);
  glm::vec3 m_rotationVariance = glm::vec3(0.0f);
  glm::ivec2 m_gridSize = glm::ivec2(10, 10);
  void GenerateField(std::vector<glm::mat4>& matricesList) const;
};

class SorghumField : public IAsset {
  friend class SorghumLayer;

 public:
  int m_sizeLimit = 2000;
  float m_sorghumSize = 1.0f;
  std::vector<std::pair<AssetRef, glm::mat4>> m_matrices;
  Entity InstantiateField() const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace eco_sys_lab