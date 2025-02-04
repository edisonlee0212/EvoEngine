
#pragma once
#include "GraphicsResources.hpp"
#include "IPrivateComponent.hpp"

namespace evo_engine {
/**
 * @struct DirectionalLightInfoBlock
 * @brief A data structure containing information for a directional light.
 */
struct DirectionalLightInfoBlock {
  glm::vec4 direction;              /**< The direction of the light. */
  glm::vec4 diffuse;                /**< The diffuse color of the light. */
  glm::vec4 specular;               /**< The specular color of the light. */
  glm::mat4 light_space_matrix[4];  /**< The light space transformation matrices. */
  glm::vec4 light_frustum_width;    /**< The width of the frustum for the light. */
  glm::vec4 light_frustum_distance; /**< The distance of the frustum for the light. */
  glm::vec4 reserved_parameters;    /**< Reserved parameters. */
  glm::ivec4 viewport;              /**< Viewport information for the light. */

  /**
   * @brief Compares two DirectionalLightInfoBlock objects for inequality.
   * @param other The other DirectionalLightInfoBlock to compare with.
   * @return True if the two blocks are not equal, false otherwise.
   */
  bool operator!=(const DirectionalLightInfoBlock& other) const;
};

/**
 * @class DirectionalLight
 * @brief A class representing a directional light component.
 */
class DirectionalLight : public IPrivateComponent {
 public:
  bool cast_shadow = true;             /**< Whether the light casts shadows. */
  glm::vec3 diffuse = glm::vec3(1.0f); /**< Diffuse color of the light. */
  float diffuse_brightness = 3.f;      /**< Brightness factor for diffuse lighting. */
  float bias = 0.1f;                   /**< Bias for shadow mapping to reduce artifacts. */
  float normal_offset = 0.05f;         /**< Offset added to the surface normal for shadow calculations. */
  float light_size = 0.01f;            /**< Size of the light source. */

  /**
   * @brief Called when the component is created.
   */
  void OnCreate() override;

  /**
   * @brief Renders the light object in the editor's inspector.
   * @param editor_layer The editor layer pointer.
   * @return True if the inspection was successful, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes the directional light to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the directional light from a YAML node.
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Performs actions necessary after cloning the component.
   * @param target The cloned component to operate on.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

/**
 * @struct PointLightInfoBlock
 * @brief A data structure containing information for a point light.
 */
struct PointLightInfoBlock {
  glm::vec4 position;                       /**< The position of the light in world space. */
  glm::vec4 constant_linear_quad_far_plane; /**< Light attenuation and far plane parameters. */
  glm::vec4 diffuse;                        /**< Diffuse color of the light. */
  glm::vec4 specular;                       /**< Specular color of the light. */
  glm::mat4 light_space_matrix[6];          /**< Light space transformation matrices. */
  glm::vec4 reserved_parameters;            /**< Reserved parameters. */
  glm::ivec4 viewport;                      /**< Viewport information for the light. */

  /**
   * @brief Compares two PointLightInfoBlock objects for inequality.
   * @param other The other PointLightInfoBlock to compare with.
   * @return True if the two blocks are not equal, false otherwise.
   */
  bool operator!=(const PointLightInfoBlock& other) const;
};

/**
 * @class PointLight
 * @brief A class representing a point light component.
 */
class PointLight : public IPrivateComponent {
 public:
  bool cast_shadow = true;             /**< Whether the light casts shadows. */
  float constant = 1.0f;               /**< Constant attenuation factor. */
  float linear = 0.07f;                /**< Linear attenuation factor. */
  float quadratic = 0.0015f;           /**< Quadratic attenuation factor. */
  float bias = 0.002f;                 /**< Bias for shadow mapping to reduce artifacts. */
  glm::vec3 diffuse = glm::vec3(1.0f); /**< Diffuse color of the light. */
  float diffuse_brightness = 3.f;      /**< Brightness factor for diffuse lighting. */
  float light_size = 0.01f;            /**< Size of the light source. */
  float shadow_distance = 100.f;       /**< Maximum shadow draw distance. */

  /**
   * @brief Renders the light object in the editor's inspector.
   * @param editor_layer The editor layer pointer.
   * @return True if the inspection was successful, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Called when the component is created.
   */
  void OnCreate() override;

  /**
   * @brief Serializes the point light to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the point light from a YAML node.
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Gets the far plane distance for shadow mapping.
   * @return The far plane distance.
   */
  [[nodiscard]] float GetFarPlane() const;

  /**
   * @brief Performs actions necessary after cloning the component.
   * @param target The cloned component to operate on.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

/**
 * @struct SpotLightInfoBlock
 * @brief A data structure containing information for a spot light.
 */
struct SpotLightInfoBlock {
  glm::vec4 position;                              /**< The position of the light in world space. */
  glm::vec4 direction;                             /**< The direction of the spotlight. */
  glm::mat4 light_space_matrix;                    /**< The light space transformation matrix. */
  glm::vec4 cut_off_outer_cut_off_light_size_bias; /**< Spotlight parameters and bias. */
  glm::vec4 constant_linear_quad_far_plane;        /**< Light attenuation and far plane parameters. */
  glm::vec4 diffuse;                               /**< Diffuse color of the light. */
  glm::vec4 specular;                              /**< Specular color of the light. */
  glm::ivec4 viewport;                             /**< Viewport information for the light. */

  /**
   * @brief Compares two SpotLightInfoBlock objects for inequality.
   * @param other The other SpotLightInfoBlock to compare with.
   * @return True if the two blocks are not equal, false otherwise.
   */
  bool operator!=(const SpotLightInfoBlock& other) const;
};

/**
 * @class SpotLight
 * @brief A class representing a spot light component.
 */
class SpotLight : public IPrivateComponent {
 public:
  bool cast_shadow = true;             /**< Whether the light casts shadows. */
  float inner_degrees = 20;            /**< Inner cutoff angle in degrees. */
  float outer_degrees = 30;            /**< Outer cutoff angle in degrees. */
  float constant = 1.0f;               /**< Constant attenuation factor. */
  float linear = 0.07f;                /**< Linear attenuation factor. */
  float quadratic = 0.0015f;           /**< Quadratic attenuation factor. */
  float bias = 0.002f;                 /**< Bias for shadow mapping to reduce artifacts. */
  glm::vec3 diffuse = glm::vec3(1.0f); /**< Diffuse color of the light. */
  float diffuse_brightness = 3.f;      /**< Brightness factor for diffuse lighting. */
  float light_size = 0.01f;            /**< Size of the light source. */
  float shadow_distance = 100.f;       /**< Maximum shadow draw distance. */

  /**
   * @brief Renders the light object in the editor's inspector.
   * @param editor_layer The editor layer pointer.
   * @return True if the inspection was successful, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Called when the component is created.
   */
  void OnCreate() override;

  /**
   * @brief Serializes the spotlight to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the spotlight from a YAML node.
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Gets the far plane distance for shadow mapping.
   * @return The far plane distance.
   */
  [[nodiscard]] float GetFarPlane() const;

  /**
   * @brief Performs actions necessary after cloning the component.
   * @param target The cloned component to operate on.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

/**
 * @class Lighting
 * @brief A system managing all light components and shadow maps.
 */
class Lighting {
  std::shared_ptr<Image> directional_light_shadow_map_ = {};          /**< Shadow map for directional lights. */
  std::shared_ptr<ImageView> directional_light_shadow_map_view_ = {}; /**< View for directional light shadow map. */
  std::vector<std::shared_ptr<ImageView>> directional_light_shadow_map_layered_views_ =
      {}; /**< Layered shadow map views for directional lights. */
  std::shared_ptr<Sampler> directional_shadow_map_sampler_ = {}; /**< Sampler for directional light shadow map. */

  std::shared_ptr<Image> point_light_shadow_map_ = {};          /**< Shadow map for point lights. */
  std::shared_ptr<ImageView> point_light_shadow_map_view_ = {}; /**< View for point light shadow map. */
  std::vector<std::shared_ptr<ImageView>> point_light_shadow_map_layered_views_ =
      {};                                                        /**< Layered shadow map views for point lights. */
  std::shared_ptr<Sampler> point_light_shadow_map_sampler_ = {}; /**< Sampler for point light shadow map. */

  std::shared_ptr<Image> spot_light_shadow_map_ = {};           /**< Shadow map for spotlights. */
  std::shared_ptr<ImageView> spot_light_shadow_map_view_ = {};  /**< View for spotlight shadow map. */
  std::shared_ptr<Sampler> spot_light_shadow_map_sampler_ = {}; /**< Sampler for spotlight shadow map. */
  friend class RenderLayer;

  /**
   * @brief Consumes available space on the shadow atlas.
   * @param location Initial coordinates on the atlas.
   * @param resolution Resolution of shadow map.
   * @param remaining_size Available remaining size.
   * @param results Vector to store output results.
   */
  static void Consume(glm::vec2 location, uint32_t resolution, uint32_t remaining_size,
                      std::vector<glm::uvec3>& results);

 public:
  std::shared_ptr<DescriptorSet> lighting_descriptor_set =
      VK_NULL_HANDLE; /**< Descriptor set for lighting resources. */

  /**
   * @brief Allocates space on the shadow atlas.
   * @param size Size of the atlas.
   * @param max_resolution Maximum resolution of allocated blocks.
   * @param results Output results vector.
   */
  static void AllocateAtlas(uint32_t size, uint32_t max_resolution, std::vector<glm::uvec3>& results);

  /**
   * @brief Default constructor for Lighting.
   */
  Lighting();

  /**
   * @brief Initializes lighting resources.
   */
  void Initialize();

  /**
   * @brief Gets the depth attachment info for directional light shadow mapping.
   * @param load_op Load operation.
   * @param store_op Store operation.
   * @return Depth attachment info.
   */
  [[nodiscard]] VkRenderingAttachmentInfo GetDirectionalLightDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                                 VkAttachmentStoreOp store_op) const;

  /**
   * @brief Gets the depth attachment info for point light shadow mapping.
   * @param load_op Load operation.
   * @param store_op Store operation.
   * @return Depth attachment info.
   */
  [[nodiscard]] VkRenderingAttachmentInfo GetPointLightDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                           VkAttachmentStoreOp store_op) const;

  /**
   * @brief Gets the layered depth attachment info for directional light shadow mapping.
   * @param split Split level in the shadow map.
   * @param load_op Load operation.
   * @param store_op Store operation.
   * @return Depth attachment info.
   */
  [[nodiscard]] VkRenderingAttachmentInfo GetLayeredDirectionalLightDepthAttachmentInfo(
      uint32_t split, VkAttachmentLoadOp load_op, VkAttachmentStoreOp store_op) const;

  /**
   * @brief Gets the layered depth attachment info for point light shadow mapping.
   * @param face Cube map face.
   * @param load_op Load operation.
   * @param store_op Store operation.
   * @return Depth attachment info.
   */
  [[nodiscard]] VkRenderingAttachmentInfo GetLayeredPointLightDepthAttachmentInfo(uint32_t face,
                                                                                  VkAttachmentLoadOp load_op,
                                                                                  VkAttachmentStoreOp store_op) const;

  /**
   * @brief Gets the depth attachment info for spotlight shadow mapping.
   * @param load_op Load operation.
   * @param store_op Store operation.
   * @return Depth attachment info.
   */
  [[nodiscard]] VkRenderingAttachmentInfo GetSpotLightDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                          VkAttachmentStoreOp store_op) const;
};
}  // namespace evo_engine
