#pragma once
using namespace evo_engine;
namespace digital_agriculture_plugin {
class SorghumFieldPatch {
 public:
  glm::vec2 grid_distance = glm::vec2(1.0f);
  glm::vec2 position_offset_mean = glm::vec2(0.f);
  glm::vec2 position_offset_variance = glm::vec2(0.0f);
  glm::vec3 rotation_variance = glm::vec3(0.0f);
  glm::ivec2 grid_size = glm::ivec2(10, 10);
  void GenerateField(std::vector<glm::mat4>& matrices_list) const;
};

class SorghumField : public IAsset {
  friend class SorghumLayer;

 public:
  int size_limit = 2000;
  float sorghum_size = 1.0f;
  std::vector<std::pair<AssetRef, glm::mat4>> matrices;
  Entity InstantiateField() const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace digital_agriculture_plugin