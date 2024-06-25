#pragma once
#include "GraphicsResources.hpp"
#include "IPrivateComponent.hpp"

namespace evo_engine {
struct DirectionalLightInfo {
  glm::vec4 direction;
  glm::vec4 diffuse;
  glm::vec4 m_specular;
  glm::mat4 light_space_matrix[4];
  glm::vec4 light_frustum_width;
  glm::vec4 light_frustum_distance;
  glm::vec4 reserved_parameters;
  glm::ivec4 viewport;
};
class DirectionalLight : public IPrivateComponent {
 public:
  bool cast_shadow = true;
  glm::vec3 diffuse = glm::vec3(1.0f);
  float diffuse_brightness = 1.f;
  float bias = 0.1f;
  float normal_offset = 0.05f;
  float light_size = 0.01f;
  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};
struct PointLightInfo {
  glm::vec4 position;
  glm::vec4 constant_linear_quad_far_plane;
  glm::vec4 diffuse;
  glm::vec4 specular;
  glm::mat4 light_space_matrix[6];
  glm::vec4 reserved_parameters;
  glm::ivec4 viewport;
};

class PointLight : public IPrivateComponent {
 public:
  bool cast_shadow = true;
  float constant = 1.0f;
  float linear = 0.07f;
  float quadratic = 0.0015f;
  float bias = 0.05f;
  glm::vec3 diffuse = glm::vec3(1.0f);
  float diffuse_brightness = 0.8f;
  float light_size = 0.01f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  [[nodiscard]] float GetFarPlane() const;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};
struct SpotLightInfo {
  glm::vec4 position;
  glm::vec4 direction;
  glm::mat4 light_space_matrix;
  glm::vec4 cut_off_outer_cut_off_light_size_bias;
  glm::vec4 constant_linear_quad_far_plane;
  glm::vec4 diffuse;
  glm::vec4 specular;
  glm::ivec4 viewport;
};
class SpotLight : public IPrivateComponent {
 public:
  bool cast_shadow = true;
  float inner_degrees = 20;
  float outer_degrees = 30;
  float constant = 1.0f;
  float linear = 0.07f;
  float quadratic = 0.0015f;
  float bias = 0.001f;
  glm::vec3 diffuse = glm::vec3(1.0f);
  float diffuse_brightness = 0.8f;
  float light_size = 0.01f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  [[nodiscard]] float GetFarPlane() const;

  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

class Lighting {
  std::shared_ptr<Image> directional_light_shadow_map_ = {};
  std::shared_ptr<ImageView> directional_light_shadow_map_view_ = {};
  std::vector<std::shared_ptr<ImageView>> directional_light_shadow_map_layered_views_ = {};
  std::shared_ptr<Sampler> directional_shadow_map_sampler_ = {};

  std::shared_ptr<Image> point_light_shadow_map_ = {};
  std::shared_ptr<ImageView> point_light_shadow_map_view_ = {};
  std::vector<std::shared_ptr<ImageView>> point_light_shadow_map_layered_views_ = {};
  std::shared_ptr<Sampler> point_light_shadow_map_sampler_ = {};

  std::shared_ptr<Image> spot_light_shadow_map_ = {};
  std::shared_ptr<ImageView> spot_light_shadow_map_view_ = {};
  std::shared_ptr<Sampler> spot_light_shadow_map_sampler_ = {};
  friend class RenderLayer;

  static void Consume(glm::vec2 location, uint32_t resolution, uint32_t remaining_size,
                      std::vector<glm::uvec3>& results);

 public:
  std::shared_ptr<DescriptorSet> lighting_descriptor_set = VK_NULL_HANDLE;

  static void AllocateAtlas(uint32_t size, uint32_t max_resolution, std::vector<glm::uvec3>& results);

  Lighting();
  void Initialize();
  [[nodiscard]] VkRenderingAttachmentInfo GetDirectionalLightDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                                 VkAttachmentStoreOp store_op) const;
  [[nodiscard]] VkRenderingAttachmentInfo GetPointLightDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                           VkAttachmentStoreOp store_op) const;
  [[nodiscard]] VkRenderingAttachmentInfo GetLayeredDirectionalLightDepthAttachmentInfo(
      uint32_t split, VkAttachmentLoadOp load_op, VkAttachmentStoreOp store_op) const;
  [[nodiscard]] VkRenderingAttachmentInfo GetLayeredPointLightDepthAttachmentInfo(uint32_t face,
                                                                                  VkAttachmentLoadOp load_op,
                                                                                  VkAttachmentStoreOp store_op) const;
  [[nodiscard]] VkRenderingAttachmentInfo GetSpotLightDepthAttachmentInfo(VkAttachmentLoadOp load_op,
                                                                          VkAttachmentStoreOp store_op) const;
};
}  // namespace evo_engine
