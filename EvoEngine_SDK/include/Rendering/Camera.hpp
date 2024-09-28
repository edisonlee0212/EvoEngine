#pragma once
#include "Bound.hpp"
#include "IPrivateComponent.hpp"
#include "RenderTexture.hpp"
#include "Transform.hpp"

namespace evo_engine {
struct CameraInfoBlock {
  glm::mat4 projection = {};
  glm::mat4 view = {};
  glm::mat4 projection_view = {};
  glm::mat4 inverse_projection = {};
  glm::mat4 inverse_view = {};
  glm::mat4 inverse_projection_view = {};
  glm::vec4 clear_color = {};
  glm::vec4 reserved_parameters1 = {};
  glm::vec4 reserved_parameters2 = {};
  int skybox_texture_index = 0;
  int environmental_irradiance_texture_index = 0;
  int environmental_prefiltered_index = 0;
  int camera_use_clear_color = 0;
  [[nodiscard]] glm::vec3 Project(const glm::vec3& position) const;
  [[nodiscard]] glm::vec3 UnProject(const glm::vec3& position) const;
};

class Camera final : public IPrivateComponent {
  

  friend class Platform;
  friend class RenderLayer;
  friend class EditorLayer;
  friend struct CameraInfoBlock;
  friend class PostProcessingStack;
  friend class Bloom;
  friend class Ssao;
  friend class Ssr;

  std::shared_ptr<RenderTexture> render_texture_;

  // Deferred shading GBuffer
  std::shared_ptr<Image> g_buffer_normal_ = {};
  std::shared_ptr<ImageView> g_buffer_normal_view_ = {};
  std::shared_ptr<Sampler> g_buffer_normal_sampler_ = {};
  ImTextureID g_buffer_normal_im_texture_id_ = {};

  std::shared_ptr<Image> g_buffer_material_ = {};
  std::shared_ptr<ImageView> g_buffer_material_view_ = {};
  std::shared_ptr<Sampler> g_buffer_material_sampler_ = {};
  ImTextureID g_buffer_material_im_texture_id_ = {};

  size_t frame_count_ = 0;
  bool rendered_ = false;
  bool require_rendering_ = false;

  glm::uvec2 size_ = glm::uvec2(1, 1);

  std::shared_ptr<DescriptorSet> g_buffer_descriptor_set_ = VK_NULL_HANDLE;
  void UpdateGBuffer();

 public:
  enum class CameraRenderMode { Rasterization, RayTracing };
  CameraRenderMode camera_render_mode = CameraRenderMode::Rasterization;

  void TransitGBufferImageLayout(VkCommandBuffer vk_command_buffer, VkImageLayout target_layout) const;

  void UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block, const GlobalTransform& global_transform);
  void AppendGBufferColorAttachmentInfos(std::vector<VkRenderingAttachmentInfo>& attachment_infos,
                                         VkAttachmentLoadOp load_op, VkAttachmentStoreOp store_op) const;

  [[nodiscard]] float GetSizeRatio() const;

  [[nodiscard]] const std::shared_ptr<RenderTexture>& GetRenderTexture() const;
  [[nodiscard]] glm::uvec2 GetSize() const;
  void Resize(const glm::uvec2& size);
  void OnCreate() override;
  [[nodiscard]] bool Rendered() const;
  void SetRequireRendering(bool value);
  float near_distance = 0.1f;
  float far_distance = 200.0f;
  float fov = 120;
  bool use_clear_color = false;
  glm::vec3 clear_color = glm::vec3(0.0f);
  float background_intensity = 1.0f;
  AssetRef skybox;
  AssetRef post_processing_stack;
  static void CalculatePlanes(std::vector<Plane>& planes, const glm::mat4& projection, const glm::mat4& view);
  static void CalculateFrustumPoints(const std::shared_ptr<Camera>& camera_component, float near_plane, float far_plane,
                                     glm::vec3 camera_pos, glm::quat camera_rot, glm::vec3* points);
  static glm::quat ProcessMouseMovement(float yaw_angle, float pitch_angle, bool constrain_pitch = true);
  static void ReverseAngle(const glm::quat& rotation, float& pitch_angle, float& yaw_angle,
                           const bool& constrain_pitch = true);
  [[nodiscard]] glm::mat4 GetProjection() const;

  glm::vec3 GetMouseWorldPoint(GlobalTransform& ltw, glm::vec2 mouse_position) const;
  Ray ScreenPointToRay(GlobalTransform& ltw, glm::vec2 mouse_position) const;

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void OnDestroy() override;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace evo_engine
