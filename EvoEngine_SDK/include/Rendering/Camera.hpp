
#pragma once
#include <functional>
#include <string>
#include <vector>
#include "Bound.hpp"
#include "CameraSettings.hpp"
#include "IPrivateComponent.hpp"
#include "RenderTexture.hpp"
#include "Transform.hpp"

namespace evo_engine {
class DescriptorSet;
class DescriptorSetLayout;
class PostProcessingStack;
class RenderGraphTransientResourceStore;
struct PostProcessingCameraResources;

enum class RayCameraHistoryTechnique : uint32_t { RayTracing, RayQuery };

struct RayCameraOutputDescriptorSlot {
  std::shared_ptr<DescriptorSet> descriptor_set;
  uint64_t recording_frame_serial = 0;
  bool recorded = false;
};

struct RayCameraHistoryResources {
  VkExtent3D extent{};
  std::shared_ptr<Image> radiance_image;
  std::shared_ptr<ImageView> radiance_view;
  std::shared_ptr<Image> convergence_image;
  std::shared_ptr<ImageView> convergence_view;
  RayCameraHistoryTechnique technique = RayCameraHistoryTechnique::RayTracing;
  uint64_t scene_handle = 0;
  uint32_t temporal_history_version = 0;
  uint32_t frame_id = 0;
  bool valid = false;
  uint64_t resource_generation = 0;
  std::vector<RayCameraOutputDescriptorSlot> output_descriptor_slots;
};

struct RayCameraHistoryStats {
  uint64_t live_camera_count = 0;
  uint64_t live_history_count = 0;
  uint64_t live_ray_tracing_history_count = 0;
  uint64_t live_ray_query_history_count = 0;
  uint64_t valid_history_count = 0;
  uint64_t radiance_image_count = 0;
  uint64_t convergence_image_count = 0;
  uint64_t radiance_view_count = 0;
  uint64_t convergence_view_count = 0;
  uint64_t live_byte_size = 0;
  uint64_t peak_live_history_count = 0;
  uint64_t peak_live_byte_size = 0;
  uint64_t creation_count = 0;
  uint64_t reuse_count = 0;
  uint64_t invalidation_count = 0;
  uint64_t retirement_count = 0;
  uint64_t live_output_descriptor_count = 0;
  uint64_t peak_live_output_descriptor_count = 0;
  uint64_t output_descriptor_creation_count = 0;
  uint64_t output_descriptor_reuse_count = 0;
};

/**
 * @brief Represents the camera information block with matrices and settings used for rendering.
 */
struct CameraInfoBlock {
  static constexpr uint32_t kRasterLightingGtaoVisibility = 1u << 0u;

  glm::mat4 projection = {};                           ///< The projection matrix of the camera.
  glm::mat4 view = {};                                 ///< The view matrix of the camera.
  glm::mat4 projection_view = {};                      ///< The combined projection and view matrix.
  glm::mat4 inverse_projection = {};                   ///< The inverse of the projection matrix.
  glm::mat4 inverse_view = {};                         ///< The inverse of the view matrix.
  glm::mat4 inverse_projection_view = {};              ///< The inverse of the combined projection and view matrix.
  glm::mat4 previous_projection_view = {};             ///< The previous frame's combined projection and view matrix.
  glm::mat4 unjittered_projection_view = {};           ///< The current frame's unjittered projection-view matrix.
  glm::mat4 previous_unjittered_projection_view = {};  ///< The previous frame's unjittered projection-view matrix.
  glm::vec4 clear_color = {};                          ///< The clear color for rendering.
  glm::vec4 jitter = {};                               ///< xy current jitter, zw previous jitter in clip-space units.
  glm::vec2 resolution;                                ///< The resolution of the camera.
  float fade_ratio;                                    ///< The fade ratio for transitions.
  float fade_factor;                                   ///< The fade factor for effects.
  int skybox_texture_index = 0;                        ///< Index of the skybox texture.
  int environmental_irradiance_texture_index = 0;      ///< Index of the environmental irradiance texture.
  int environmental_prefiltered_index = 0;             ///< Index of the environmental prefiltered texture.
  int background_source = 0;                           ///< 0 samples the resolved cubemap, 1 uses clear color.

  // Ray tracing
  uint32_t firefly_clamp_enabled = 1;
  float gamma = 2.2f;
  uint32_t sample_size = 4;
  uint32_t bounce = 4;
  float firefly_clamp_threshold = 10.0f;
  uint32_t auto_spp_enabled = 0;
  uint32_t auto_spp_min_samples = 16;
  uint32_t auto_spp_max_samples = 256;
  float auto_spp_convergence_threshold = 0.01f;
  uint32_t emissive_triangle_nee_enabled = 1;
  uint32_t ray_debug_view = 0;
  uint32_t raster_lighting_flags = 0;
  glm::vec4 shadow_split_distances = {};

  /**
   * @brief Projects a 3D world position into 2D screen space.
   * @param position The 3D position to project.
   * @return The projected screen-space position as a 2D vector.
   */
  [[nodiscard]] glm::vec3 Project(const glm::vec3& position) const;

  /**
   * @brief Unprojects a 2D screen-space position back to 3D world space.
   * @param position The 2D screen-space position to unproject.
   * @return The unprojected 3D world position.
   */
  [[nodiscard]] glm::vec3 UnProject(const glm::vec3& position) const;
  bool operator!=(const CameraInfoBlock& other) const;
};

/**
 * @brief Camera class for managing rendering and view functionality.
 */
class Camera final : public IPrivateComponent {
 public:
  using BackgroundSource = CameraSettings::BackgroundSource;

  /**
   * @brief Enum to define the camera rendering mode.
   */
  enum class CameraRenderMode {
    Rasterization,  ///< Render using rasterization.
    RayTracing,     ///< Render using the ray tracing pipeline.
    RayQuery        ///< Render using the ray-query camera path when supported, otherwise falls back.
  };

  static constexpr uint32_t kCameraRenderModeCount = 3;
  static constexpr uint32_t kShaderExecutionReorderingModeCount = 3;
  static constexpr uint32_t kRayDebugViewCount = 20;
  static constexpr uint32_t kBackgroundSourceCount = 5;

  [[nodiscard]] static const std::vector<std::string>& GetCameraRenderModeNames();
  [[nodiscard]] static const char* GetCameraRenderModeName(CameraRenderMode mode);
  [[nodiscard]] static const std::vector<std::string>& GetBackgroundSourceNames();
  [[nodiscard]] static const char* GetBackgroundSourceName(BackgroundSource source);
  [[nodiscard]] static BackgroundSource ParseBackgroundSource(const std::string& value,
                                                              BackgroundSource fallback = BackgroundSource::Cubemap);
  [[nodiscard]] static BackgroundSource NormalizeBackgroundSource(uint32_t source);
  [[nodiscard]] static BackgroundSource ResolveBackgroundSource(const CameraSettings& settings);
  [[nodiscard]] static const std::vector<std::string>& GetShaderExecutionReorderingModeNames();
  [[nodiscard]] static const char* GetShaderExecutionReorderingModeName(
      CameraSettings::ShaderExecutionReorderingMode mode);
  [[nodiscard]] static const std::vector<std::string>& GetRayDebugViewNames();
  [[nodiscard]] static const char* GetRayDebugViewName(CameraSettings::RayDebugView view);
  [[nodiscard]] static CameraSettings::RayDebugView ParseRayDebugView(
      const std::string& value, CameraSettings::RayDebugView fallback = CameraSettings::RayDebugView::Beauty);
  [[nodiscard]] static CameraSettings::RayDebugView NormalizeRayDebugView(uint32_t view);
  [[nodiscard]] static CameraSettings::ShaderExecutionReorderingMode ParseShaderExecutionReorderingMode(
      const std::string& value,
      CameraSettings::ShaderExecutionReorderingMode fallback = CameraSettings::ShaderExecutionReorderingMode::Disabled);
  [[nodiscard]] static CameraSettings::ShaderExecutionReorderingMode NormalizeShaderExecutionReorderingMode(
      uint32_t mode);
  [[nodiscard]] static bool ResolveShaderExecutionReorderingEnabled(
      CameraSettings::ShaderExecutionReorderingMode requested_mode);
  [[nodiscard]] static CameraRenderMode ParseCameraRenderMode(
      const std::string& value, CameraRenderMode fallback = CameraRenderMode::Rasterization);
  [[nodiscard]] static CameraRenderMode NormalizeCameraRenderMode(uint32_t mode);
  [[nodiscard]] static bool IsRayCameraRenderMode(CameraRenderMode mode);
  [[nodiscard]] static CameraRenderMode ResolveCameraRenderMode(CameraRenderMode requested_mode);

  CameraRenderMode camera_render_mode = CameraRenderMode::Rasterization;  ///< The current rendering mode.

  /**
   * @brief Transits the GBuffer image layout.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param target_layout The target image layout to transition to.
   */
  void TransitGBufferImageLayout(VkCommandBuffer vk_command_buffer, VkImageLayout target_layout) const;

  /**
   * @brief Updates the camera information block based on the given global transform.
   * @param camera_info_block Reference to the camera information block.
   * @param global_transform The global transform of the camera.
   */
  void UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block, const GlobalTransform& global_transform);

  /**
   * @brief Appends GBuffer color attachment information for rendering.
   * @param attachment_infos Vector to store attachment information.
   * @param load_op The load operation for the attachment.
   * @param store_op The store operation for the attachment.
   */
  void AppendGBufferColorAttachmentInfos(std::vector<VkRenderingAttachmentInfo>& attachment_infos,
                                         VkAttachmentLoadOp load_op, VkAttachmentStoreOp store_op) const;

  /**
   * @brief Gets the size ratio of the camera's resolution.
   * @return The size ratio as a float.
   */
  [[nodiscard]] float GetSizeRatio() const;

  /**
   * @brief Gets a shared pointer to the render texture used by the camera.
   * @return A shared pointer to the render texture.
   */
  [[nodiscard]] const std::shared_ptr<RenderTexture>& GetRenderTexture() const;

  /**
   * @brief Gets the resolution size of the camera.
   * @return The resolution size as a 2D vector.
   */
  [[nodiscard]] glm::uvec2 GetSize() const;
  [[nodiscard]] uint32_t GetFrameCount() const;
  [[nodiscard]] uint32_t GetTemporalHistoryVersion() const;

  /**
   * @brief Resizes the camera to the specified resolution size.
   * @param size The new size as a 2D vector.
   */
  void Resize(const glm::uvec2& size);

  /**
   * @brief Called when the camera is created.
   */
  void OnCreate() override;

  /**
   * @brief Determines if the camera has rendered in the current frame.
   * @return True if the camera has rendered, otherwise false.
   */
  [[nodiscard]] bool Rendered() const;

  /**
   * @brief Sets whether the camera requires rendering.
   * @param value True if rendering is required, otherwise false.
   */
  void SetRequireRendering(bool value);

  CameraSettings camera_settings{};    ///< Settings for the camera.
  AssetRef skybox;                     ///< Explicit cubemap background source.
  AssetRef background_environment;     ///< Explicit environmental-map background source.
  AssetRef post_processing_stack_ref;  ///< Reference to the post-processing stack.

  /**
   * @brief Calculates the planes of the camera frustum based on projection and view matrices.
   * @param planes Vector to store the calculated planes.
   * @param projection The projection matrix.
   * @param view The view matrix.
   */
  static void CalculatePlanes(std::vector<Plane>& planes, const glm::mat4& projection, const glm::mat4& view);

  /**
   * @brief Calculates the frustum points of the camera.
   * @param camera_component Shared pointer to the camera component.
   * @param near_plane The near clipping plane distance.
   * @param far_plane The far clipping plane distance.
   * @param camera_pos The position of the camera.
   * @param camera_rot The rotation of the camera.
   * @param points Array to store the calculated frustum points.
   */
  static void CalculateFrustumPoints(const std::shared_ptr<Camera>& camera_component, float near_plane, float far_plane,
                                     glm::vec3 camera_pos, glm::quat camera_rot, glm::vec3* points);

  /**
   * @brief Processes mouse movement to calculate a new rotation quaternion.
   * @param yaw_angle The yaw angle.
   * @param pitch_angle The pitch angle.
   * @param constrain_pitch Flag to constrain the pitch angle within limits.
   * @return The resulting quaternion for the rotation.
   */
  static glm::quat ProcessMouseMovement(float yaw_angle, float pitch_angle, bool constrain_pitch = true);

  /**
   * @brief Reverses the rotation quaternion into pitch and yaw angles.
   * @param rotation The rotation quaternion.
   * @param pitch_angle Reference to store the pitch angle.
   * @param yaw_angle Reference to store the yaw angle.
   * @param constrain_pitch Flag to constrain the pitch angle within limits.
   */
  static void ReverseAngle(const glm::quat& rotation, float& pitch_angle, float& yaw_angle,
                           const bool& constrain_pitch = true);
  /**
   * @brief Gets the projection matrix of the camera.
   * @return The projection matrix as a 4x4 matrix.
   */
  [[nodiscard]] glm::mat4 GetProjection() const;

  /**
   * @brief Gets the world point in 3D space based on a mouse position in 2D screen space.
   * @param ltw The global transform.
   * @param mouse_position The 2D mouse position.
   * @return The world point in 3D space.
   */
  glm::vec3 GetMouseWorldPoint(GlobalTransform& ltw, glm::vec2 mouse_position) const;

  /**
   * @brief Converts a screen point to a 3D ray in world space.
   * @param ltw The global transform.
   * @param mouse_position The 2D screen-space position.
   * @return The ray originating from the screen point in 3D space.
   */
  Ray ScreenPointToRay(GlobalTransform& ltw, glm::vec2 mouse_position) const;

  /**
   * @brief Called when the camera is destroyed.
   */
  void OnDestroy() override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& source) override;

  /**
   * @brief Collects asset references used by the camera.
   * @param list The list to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Gets the Vulkan GBuffer descriptor set.
   * @return A shared pointer to the GBuffer descriptor set.
   */
  const std::shared_ptr<DescriptorSet>& GetGBufferDescriptorSet() const;

  [[nodiscard]] const std::shared_ptr<Image>& GetGBufferUtilityImage() const;
  [[nodiscard]] RayCameraHistoryStats GetRayCameraHistoryStats() const;
  [[nodiscard]] ImTextureID GetGBufferBaseColorAoImTextureId() const;
  [[nodiscard]] ImTextureID GetGBufferNormalRoughnessImTextureId() const;
  [[nodiscard]] ImTextureID GetGBufferPbrFlagsImTextureId() const;
  [[nodiscard]] ImTextureID GetGBufferEmissiveImTextureId() const;
  [[nodiscard]] ImTextureID GetGBufferUtilityImTextureId() const;

  void SetRendered();
  void ResetRenderState();
  void ResetFrameCount();

 private:
  friend class Platform;     ///< Grants access to the Platform class.
  friend class RenderLayer;  ///< Grants access to the RenderLayer class.
  friend class RayCameraHistoryTestAccess;
  friend class PostProcessingRuntimeTestAccess;
  friend class PostProcessingPass;
  friend class PostProcessingStack;
  friend struct CameraInfoBlock;  ///< Grants access to the CameraInfoBlock struct.

  std::shared_ptr<RenderTexture> render_texture_;  ///< The render texture used by the camera.

  std::shared_ptr<Sampler> g_buffer_sampler_ = {};                  ///< GBuffer sampler.
  std::shared_ptr<Image> g_buffer_base_color_ao_ = {};              ///< Expanded GBuffer base color/AO image.
  std::shared_ptr<ImageView> g_buffer_base_color_ao_view_ = {};     ///< Expanded GBuffer base color/AO view.
  ImTextureID g_buffer_base_color_ao_im_texture_id_ = {};           ///< ImTextureID for base color/AO.
  std::shared_ptr<Image> g_buffer_normal_roughness_ = {};           ///< Expanded GBuffer normal/roughness image.
  std::shared_ptr<ImageView> g_buffer_normal_roughness_view_ = {};  ///< Expanded GBuffer normal/roughness view.
  ImTextureID g_buffer_normal_roughness_im_texture_id_ = {};        ///< ImTextureID for normal/roughness.
  std::shared_ptr<Image> g_buffer_pbr_flags_ = {};                  ///< Expanded GBuffer PBR/flags image.
  std::shared_ptr<ImageView> g_buffer_pbr_flags_view_ = {};         ///< Expanded GBuffer PBR/flags view.
  ImTextureID g_buffer_pbr_flags_im_texture_id_ = {};               ///< ImTextureID for PBR/flags.
  std::shared_ptr<Image> g_buffer_emissive_ = {};                   ///< Expanded GBuffer emissive image.
  std::shared_ptr<ImageView> g_buffer_emissive_view_ = {};          ///< Expanded GBuffer emissive view.
  ImTextureID g_buffer_emissive_im_texture_id_ = {};                ///< ImTextureID for emissive.
  std::shared_ptr<Image> g_buffer_utility_ = {};                    ///< Expanded GBuffer utility image.
  std::shared_ptr<ImageView> g_buffer_utility_view_ = {};           ///< Expanded GBuffer utility view.
  ImTextureID g_buffer_utility_im_texture_id_ = {};                 ///< ImTextureID for utility.

  uint32_t frame_count_ = 0;               ///< Frame count used for tracking rendering updates.
  uint32_t temporal_history_version_ = 0;  ///< Version incremented by explicit camera history resets.
  RayCameraHistoryResources ray_camera_history_{};
  RayCameraHistoryStats ray_camera_history_counters_{};
  uint64_t next_ray_camera_history_resource_generation_ = 0;
  bool ray_camera_history_owner_alive_ = false;
  std::shared_ptr<PostProcessingCameraResources> post_processing_resources_;

  glm::mat4 prev_global_transform_{};
  bool rendered_ = false;               ///< Indicates whether the camera has rendered.
  bool require_rendering_ = false;      ///< Indicates whether the camera requires rendering.
  glm::uvec2 size_ = glm::uvec2(1, 1);  ///< The size of the camera's resolution.
  std::shared_ptr<DescriptorSet> g_buffer_descriptor_set_ = VK_NULL_HANDLE;  ///< GBuffer descriptor set.

  /**
   * @brief Updates the deferred shading GBuffer resources.
   */
  void UpdateGBuffer();
  RayCameraHistoryResources& AcquireRayCameraHistory(
      RayCameraHistoryTechnique technique, uint64_t scene_handle, VkExtent3D extent,
      const std::function<RayCameraHistoryResources(VkExtent3D)>& resource_factory = {});
  std::shared_ptr<DescriptorSet> AcquireRayCameraOutputDescriptor(
      uint32_t frame_index, uint64_t frame_serial, const std::shared_ptr<DescriptorSetLayout>& layout,
      const std::function<std::shared_ptr<DescriptorSet>()>& resource_factory = {});
  void InvalidateRayCameraHistory();
  void ReleaseRayCameraHistory();
  PostProcessingCameraResources& AcquirePostProcessingResources(const std::shared_ptr<PostProcessingStack>& stack);
  void SynchronizePostProcessingResources(const std::shared_ptr<PostProcessingStack>& stack);
  void RetainPostProcessingResources(RenderGraphTransientResourceStore& transient_resources) const;
  void ReleasePostProcessingResources();
};

}  // namespace evo_engine

// The provided code has been fully documented with Doxygen comments in the previous responses.
// There is no remaining code to continue documenting.
