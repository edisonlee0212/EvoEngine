#pragma once

using namespace evo_engine;
namespace eco_sys_lab_plugin {
class BarkDescriptor : public IAsset {
 public:
  float bark_x_frequency = 3.0f;
  float bark_y_frequency = 5.0f;
  float bark_depth = 0.1f;

  float base_frequency = 1.0f;
  float base_max_distance = 1.f;
  float base_distance_decrease_factor = 2.f;
  float base_depth = .1f;

  AssetRef bark_material_ref;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  float GetValue(float x_factor, float distance_to_root) const;
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace eco_sys_lab_plugin