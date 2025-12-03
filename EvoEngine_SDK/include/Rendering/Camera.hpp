
#pragma once
#include "Bound.hpp"
#include "CameraSettings.hpp"
#include "IPrivateComponent.hpp"
#include "RenderTexture.hpp"
#include "Transform.hpp"

namespace evo_engine {

/**
 * @brief Represents the camera information block with matrices and settings used for rendering.
 */
struct CameraInfoBlock {
  glm::mat4 projection = {};                       ///< The projection matrix of the camera.
  glm::mat4 view = {};                             ///< The view matrix of the camera.
  glm::mat4 projection_view = {};                  ///< The combined projection and view matrix.
  glm::mat4 inverse_projection = {};               ///< The inverse of the projection matrix.
  glm::mat4 inverse_view = {};                     ///< The inverse of the view matrix.
  glm::mat4 inverse_projection_view = {};          ///< The inverse of the combined projection and view matrix.
  glm::vec4 clear_color = {};                      ///< The clear color for rendering.
  glm::vec2 resolution;                            ///< The resolution of the camera.
  float fade_ratio;                                ///< The fade ratio for transitions.
  float fade_factor;                               ///< The fade factor for effects.
  int skybox_texture_index = 0;                    ///< Index of the skybox texture.
  int environmental_irradiance_texture_index = 0;  ///< Index of the environmental irradiance texture.
  int environmental_prefiltered_index = 0;         ///< Index of the environmental prefiltered texture.
  int camera_use_clear_color = 0;                  ///< Flag to indicate if the camera uses the clear color.

  // Ray tracing
  uint32_t padding = 0;
  float gamma = 2.2f;
  uint32_t sample_size = 4;
  uint32_t bounce = 4;

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
  inline static std::shared_ptr<DescriptorSetLayout> g_buffer_layout;  ///< Global buffer layout.

  /**
   * @brief Enum to define the camera rendering mode.
   */
  enum class CameraRenderMode {
    Rasterization,  ///< Render using rasterization.
    RayTracing      ///< Render using ray tracing.
  };

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
  AssetRef skybox;                     ///< Reference to the skybox asset.
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
   * @brief Serializes the camera settings to a YAML emitter.
   * @param out The YAML emitter object.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the camera settings from a YAML node.
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Called when the camera is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Called to inspect the camera properties in the editor.
   * @param editor_layer The editor layer reference.
   * @return True if the inspection occurred, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Collects asset references used by the camera.
   * @param list The list to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Gets the Vulkan GBuffer descriptor set.
   * @return A shared pointer to the GBuffer descriptor set.
   */
  const std::shared_ptr<DescriptorSet>& GetGBufferDescriptorSet() const;

  void SetRendered();
  void ResetFrameCount();

 private:
  /**
   * @brief Displays debug views for camera operations, scaled for debugging.
   * @param debug_scale The scaling factor for the debug views.
   */
  void DebugViews(float debug_scale) const;

  friend class Platform;          ///< Grants access to the Platform class.
  friend class RenderLayer;       ///< Grants access to the RenderLayer class.
  friend class EditorLayer;       ///< Grants access to the EditorLayer class.
  friend struct CameraInfoBlock;  ///< Grants access to the CameraInfoBlock struct.

  std::shared_ptr<RenderTexture> render_texture_;  ///< The render texture used by the camera.

  // Deferred shading GBuffer resources
  std::shared_ptr<Image> g_buffer_normal_ = {};                       ///< GBuffer normal image.
  std::shared_ptr<ImageView> g_buffer_normal_view_ = {};              ///< GBuffer normal image view.
  std::shared_ptr<Sampler> g_buffer_normal_sampler_ = {};             ///< GBuffer normal sampler.
  ImTextureID g_buffer_normal_im_texture_id_ = {};                    ///< ImTextureID for GBuffer normal.
  std::shared_ptr<Image> g_buffer_material_ = {};                     ///< GBuffer material image.
  std::shared_ptr<ImageView> g_buffer_material_view_ = {};            ///< GBuffer material image view.
  std::shared_ptr<ImageView> g_buffer_material_tex_coord_view_ = {};  ///< GBuffer texcoord image view.
  std::shared_ptr<ImageView> g_buffer_material_indices_view_ = {};    ///< GBuffer material indices view.

  std::shared_ptr<Sampler> g_buffer_material_sampler_ = {};     ///< GBuffer material sampler.
  ImTextureID g_buffer_material_tex_coord_im_texture_id_ = {};  ///< ImTextureID for GBuffer texcoords.
  ImTextureID g_buffer_material_indices_im_texture_id_ = {};    ///< ImTextureID for GBuffer material indices.

  uint32_t frame_count_ = 0;  ///< Frame count used for tracking rendering updates.

  glm::mat4 prev_global_transform_{};
  bool rendered_ = false;               ///< Indicates whether the camera has rendered.
  bool require_rendering_ = false;      ///< Indicates whether the camera requires rendering.
  glm::uvec2 size_ = glm::uvec2(1, 1);  ///< The size of the camera's resolution.
  std::shared_ptr<DescriptorSet> g_buffer_descriptor_set_ = VK_NULL_HANDLE;  ///< GBuffer descriptor set.

  /**
   * @brief Updates the deferred shading GBuffer resources.
   */
  void UpdateGBuffer();
};

}  // namespace evo_engine

// The provided code has been fully documented with Doxygen comments in the previous responses.
// There is no remaining code to continue documenting.
