#pragma once
using namespace evo_engine;
namespace digital_agriculture_plugin {
class SorghumGrid {
 public:
  float grid_distance_x = 1.f;
  float grid_distance_y = 1.f;
  float position_offset_mean = 0.f;
  float position_offset_variance = 0.f;
  float rotation_variance_xz = 0.f;
  float rotation_variance_y = 0.f;
  int grid_size_x = 10;
  int grid_size_y = 10;
  void GenerateField(std::vector<glm::mat4>& matrices_list) const;
};

class SorghumField : public IAsset {
  friend class SorghumLayer;

 public:
  int size_limit = 2000;
  float sorghum_size = 1.0f;
  std::vector<std::pair<AssetRef, glm::mat4>> matrices;
  Entity InstantiateField(uint32_t base_seed = 0) const;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace digital_agriculture_plugin