
#pragma once
#include "AssetRef.hpp"
#include "GltfMaterial.hpp"
#include "IAsset.hpp"
#include "Platform.hpp"
#include "Texture2D.hpp"
namespace evo_engine {

/**
 * @brief A structure to define various draw settings for rendering.
 */
struct DrawSettings {
  float line_width = 1.0f;                            ///< Line width for rendering.
  VkCullModeFlags cull_mode = VK_CULL_MODE_NONE;      ///< Culling mode for rendering.
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;  ///< Polygon fill mode.

  bool blending = false;                 ///< Indicates whether blending should be enabled.
  VkBlendOp blend_op = VK_BLEND_OP_ADD;  ///< Blending operation.

  VkBlendFactor blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;            ///< Source factor for blending.
  VkBlendFactor blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;  ///< Destination factor for blending.

  /**
   * @brief Applies the current draw settings to the global pipeline state.
   * @param global_pipeline_state The global pipeline state to apply the settings to.
   */
  void ApplySettings(GraphicsPipelineStates& global_pipeline_state) const;

  /**
   * @brief Saves the current draw settings to a YAML emitter.
   * @param name Name for the settings.
   * @param out The YAML emitter to which the settings are saved.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads draw settings from a YAML node.
   * @param name Name of the settings to be loaded.
   * @param in The YAML node from which the settings are loaded.
   */
  void Load(const std::string& name, const YAML::Node& in);
};

/**
 * @class Material
 * @brief Manages material properties and settings for rendering, including textures and shaders.
 */
class Material final : public IAsset {
  friend class RenderLayer;

  bool need_update_ = true;  ///< Indicates if the material needs to be updated.
  std::vector<AssetRef> texture_refs_{AssetRef{}};

  void ResizeTextureRefs();

 public:
  Material();

  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  /**
   * @brief Generates a thumbnail texture representing this material.
   * @return A shared pointer to the generated thumbnail texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();

  /**
   * @brief Destructor for the Material class.
   */
  ~Material() override;

  uint16_t SetTexture(uint16_t GltfShadeMaterial::* slot, const std::shared_ptr<Texture2D>& texture,
                      int32_t tex_coord = 0, const glm::mat3x2& uv_transform = glm::mat3x2(1.0f));
  void SetTexture(uint16_t texture_info_slot, const std::shared_ptr<Texture2D>& texture);
  uint16_t SetTextureRef(uint16_t GltfShadeMaterial::* slot, const AssetRef& texture_ref, int32_t tex_coord = 0,
                         const glm::mat3x2& uv_transform = glm::mat3x2(1.0f));
  void SetTextureRef(uint16_t texture_info_slot, const AssetRef& texture_ref);
  [[nodiscard]] std::shared_ptr<Texture2D> GetTexture(uint16_t GltfShadeMaterial::* slot);
  [[nodiscard]] std::shared_ptr<Texture2D> GetTexture(uint16_t texture_info_slot);
  [[nodiscard]] const std::vector<AssetRef>& PeekTextureRefs() const;
  [[nodiscard]] std::vector<AssetRef>& RefTextureRefs();
  [[nodiscard]] GltfMaterialData BuildGltfMaterialData();
  void SetGltfMaterialData(const GltfMaterialData& data);
  void SyncRenderStateFromGltfMaterial();

  /**
   * @brief Marks the material as requiring a render data update.
   */
  void MarkDirty();

  bool vertex_color_only = false;  ///< When true, only vertex colors are used for rendering this material.

  GltfMaterialData material_data;
  DrawSettings draw_settings;  ///< Rendering draw settings for the material.

  /**
   * @brief Collects references to all assets used by this material.
   * @param list A vector to which the asset references are added.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);
};

}  // namespace evo_engine
