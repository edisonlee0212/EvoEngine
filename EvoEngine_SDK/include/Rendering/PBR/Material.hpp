
#pragma once
#include "AssetRef.hpp"
#include "IAsset.hpp"
#include "MaterialProperties.hpp"
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
   * @brief Displays inspection UI controls for modifying the draw settings.
   * @return True if any settings were modified, otherwise false.
   */
  bool OnInspect();

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

  bool need_update_ = true;     ///< Indicates if the material needs to be updated.
  AssetRef albedo_texture_;     ///< Reference to the albedo (diffuse) texture.
  AssetRef normal_texture_;     ///< Reference to the normal map texture.
  AssetRef metallic_texture_;   ///< Reference to the metallic texture.
  AssetRef roughness_texture_;  ///< Reference to the roughness texture.
  AssetRef ao_texture_;         ///< Reference to the ambient occlusion texture.
  AssetRef rma_texture_ref_;    ///< Temporary editor reference for RMA texture unpacking.

 public:
  [[nodiscard]] bool SupportsStagedLoading() const override {
    return true;
  }

  /**
   * @brief Generates a thumbnail texture representing this material.
   * @return A shared pointer to the generated thumbnail texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Destructor for the Material class.
   */
  ~Material() override;

  /**
   * @brief Sets the albedo texture for the material.
   * @param texture A shared pointer to the texture to be set.
   */
  void SetAlbedoTexture(const std::shared_ptr<Texture2D>& texture);

  /**
   * @brief Sets the normal texture for the material.
   * @param texture A shared pointer to the texture to be set.
   */
  void SetNormalTexture(const std::shared_ptr<Texture2D>& texture);

  /**
   * @brief Sets the metallic texture for the material.
   * @param texture A shared pointer to the texture to be set.
   */
  void SetMetallicTexture(const std::shared_ptr<Texture2D>& texture);

  /**
   * @brief Sets the roughness texture for the material.
   * @param texture A shared pointer to the texture to be set.
   */
  void SetRoughnessTexture(const std::shared_ptr<Texture2D>& texture);

  /**
   * @brief Sets the ambient occlusion texture for the material.
   * @param texture A shared pointer to the texture to be set.
   */
  void SetAoTexture(const std::shared_ptr<Texture2D>& texture);

  /**
   * @brief Retrieves the albedo texture.
   * @return A shared pointer to the albedo texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GetAlbedoTexture();

  /**
   * @brief Retrieves the normal texture.
   * @return A shared pointer to the normal texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GetNormalTexture();

  /**
   * @brief Retrieves the metallic texture.
   * @return A shared pointer to the metallic texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GetMetallicTexture();

  /**
   * @brief Retrieves the roughness texture.
   * @return A shared pointer to the roughness texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GetRoughnessTexture();

  /**
   * @brief Retrieves the ambient occlusion texture.
   * @return A shared pointer to the ambient occlusion texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GetAoTexture();

  bool vertex_color_only = false;  ///< When true, only vertex colors are used for rendering this material.

  MaterialProperties material_properties;  ///< Material-specific properties.
  DrawSettings draw_settings;              ///< Rendering draw settings for the material.

  /**
   * @brief Displays an inspection UI to modify this material's settings.
   * @param editor_layer A shared pointer to the editor layer.
   * @return True if the material was modified, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Collects references to all assets used by this material.
   * @param list A vector to which the asset references are added.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Serializes this material's data to a YAML emitter.
   * @param out The YAML emitter to store the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes this material's data from a YAML node.
   * @param in The YAML node containing the data.
   */
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace evo_engine
