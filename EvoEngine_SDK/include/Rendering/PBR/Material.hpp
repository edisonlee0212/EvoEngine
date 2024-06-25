#pragma once
#include "AssetRef.hpp"
#include "Graphics.hpp"
#include "IAsset.hpp"
#include "MaterialProperties.hpp"
#include "Texture2D.hpp"
namespace evo_engine {
struct MaterialInfoBlock {
  alignas(4) int albedo_texture_index = -1;
  alignas(4) int normal_texture_index = -1;
  alignas(4) int metallic_texture_index = -1;
  alignas(4) int roughness_texture_index = -1;

  alignas(4) int ao_texture_index = -1;
  alignas(4) int cast_shadow = true;
  alignas(4) int receive_shadow = true;
  alignas(4) int enable_shadow = true;

  glm::vec4 albedo_color_val = glm::vec4(1.0f);
  glm::vec4 subsurface_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.0f);
  glm::vec4 subsurface_radius = glm::vec4(1.0f, 1.0f, 1.0f, 0.0f);

  alignas(4) float metallic_val = 0.5f;
  alignas(4) float roughness_val = 0.5f;
  alignas(4) float ao_val = 1.0f;
  alignas(4) float emission_val = 0.0f;
};

struct DrawSettings {
  float line_width = 1.0f;
  VkCullModeFlags cull_mode = VK_CULL_MODE_NONE;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;

  bool blending = false;
  VkBlendOp blend_op = VK_BLEND_OP_ADD;

  VkBlendFactor blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  VkBlendFactor blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  bool OnInspect();
  void ApplySettings(GraphicsPipelineStates& global_pipeline_state) const;

  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
};

class Material final : public IAsset {
  friend class RenderLayer;
  bool need_update_ = true;
  AssetRef albedo_texture_;
  AssetRef normal_texture_;
  AssetRef metallic_texture_;
  AssetRef roughness_texture_;
  AssetRef ao_texture_;

 public:
  ~Material() override;
  void SetAlbedoTexture(const std::shared_ptr<Texture2D>& texture);
  void SetNormalTexture(const std::shared_ptr<Texture2D>& texture);
  void SetMetallicTexture(const std::shared_ptr<Texture2D>& texture);
  void SetRoughnessTexture(const std::shared_ptr<Texture2D>& texture);
  void SetAoTexture(const std::shared_ptr<Texture2D>& texture);
  [[nodiscard]] std::shared_ptr<Texture2D> GetAlbedoTexture();
  [[nodiscard]] std::shared_ptr<Texture2D> GetNormalTexture();
  [[nodiscard]] std::shared_ptr<Texture2D> GetMetallicTexture();
  [[nodiscard]] std::shared_ptr<Texture2D> GetRoughnessTexture();
  [[nodiscard]] std::shared_ptr<Texture2D> GetAoTexture();

  bool vertex_color_only = false;
  MaterialProperties material_properties;
  DrawSettings draw_settings;

  void UpdateMaterialInfoBlock(MaterialInfoBlock& material_info_block);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace evo_engine
