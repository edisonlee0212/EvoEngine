
#pragma once
#include "Camera.hpp"
#include "IGeometry.hpp"
#include "ILayer.hpp"

#include "DdgiSettings.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "PointCloudSample.hpp"
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"

#include <array>
#include <limits>
#include <string>

namespace evo_engine {
struct ApplicationInitializationSettings;
class ComputePipeline;
class OffscreenPreviewRenderer;
class Sampler;

/**
 * \class RenderLayer
 * \brief Represents a render layer in the rendering engine.
 *
 * Handles various rendering tasks, such as camera iteration, shadow map rendering,
 * deferred rendering, forward rendering, and more.
 */
class RenderLayer final : public ILayer {
 public:
  /**
   * \brief Iterates through all collected cameras and applies the specified action.
   * \param action Function to apply to each camera.
   */
  void ForEachCollectedCamera(const std::function<void(const std::shared_ptr<Camera>& camera)>& action) const;

  /**
   * \brief Retrieves the current render instance storage.
   * \return A shared pointer to the current render instance storage.
   */
  [[nodiscard]] std::shared_ptr<RenderInstanceStorage> GetCurrentRenderInstanceStorage() const;

  /**
   * \brief Retrieves the previous render instance storage.
   * \return A shared pointer to the previous render instance storage.
   */
  [[nodiscard]] std::shared_ptr<RenderInstanceStorage> GetPreviousRenderInstanceStorage() const;

  using DdgiSettings = evo_engine::DdgiSettings;

  struct DdgiAtlasLayout {
    uint32_t probe_count = 1;
    uint32_t tile_resolution = 1;
    uint32_t columns = 1;
    uint32_t rows = 1;
    glm::uvec2 resolution = {1, 1};
  };

  struct DdgiProbeDebugCoordinates {
    uint32_t probe_index = 0;
    glm::uvec3 grid_index = {0, 0, 0};
    glm::uvec2 atlas_tile_offset = {0, 0};
    DdgiAtlasLayout atlas_layout{};
  };

  struct DdgiFrameResourceLayout {
    uint32_t probe_count = 1;
    DdgiAtlasLayout irradiance_atlas{};
    DdgiAtlasLayout visibility_atlas{};
    DdgiAtlasLayout variability_atlas{};
    glm::uvec2 variability_reduction_extent = {1, 1};
    uint64_t probe_metadata_byte_size = 0;
    uint64_t probe_state_byte_size = 0;
    uint64_t probe_update_index_byte_size = 0;
    uint64_t ray_output_byte_size = 0;
    uint64_t variability_atlas_byte_size = 0;
    uint64_t variability_reduction_byte_size = 0;
    uint64_t variability_readback_byte_size = 0;
  };

  struct DdgiProbeUpdateWindow {
    uint32_t start_probe_index = 0;
    uint32_t probe_count = 0;
    uint32_t next_start_probe_index = 0;
    uint32_t remaining_probe_count = 0;
  };

  struct DdgiProbeUpdateStats {
    uint32_t probe_count = 0;
    uint32_t first_probe_index = 0;
    uint32_t last_probe_index = 0;
  };

  struct DdgiPerformanceStats {
    uint32_t active_probe_count = 0;
    uint32_t storage_probe_count = 0;
    uint32_t updated_probe_count = 0;
    uint32_t pending_probe_count = 0;
    uint32_t ray_count = 0;
    uint32_t ray_sample_count = 0;
    uint32_t selected_ray_sample_count = 0;
    uint32_t visualized_probe_count = 0;
    uint64_t probe_metadata_byte_size = 0;
    uint64_t probe_state_byte_size = 0;
    uint64_t ray_output_byte_size = 0;
    uint64_t variability_atlas_byte_size = 0;
    uint64_t variability_reduction_byte_size = 0;
    glm::uvec2 irradiance_atlas_extent = {0, 0};
    glm::uvec2 visibility_atlas_extent = {0, 0};
    glm::uvec2 variability_atlas_extent = {0, 0};
    glm::uvec2 variability_reduction_extent = {0, 0};
    float probe_variability_average = 0.0f;
    uint32_t probe_variability_sample_count = 0;
    uint32_t probe_variability_stable_sample_count = 0;
    uint32_t probe_variability_required_stable_sample_count = 0;
    bool probe_variability_converged = false;
    uint32_t probe_warmup_frame_index = 0;
    uint32_t probe_warmup_frame_count = 0;
    bool probe_warmup_active = false;
    float probe_update_hysteresis = 0.0f;
    float atlas_prepare_record_ms = 0.0f;
    float ray_diagnostics_record_ms = 0.0f;
    float probe_update_record_ms = 0.0f;
    float probe_relocation_record_ms = 0.0f;
    float probe_classification_record_ms = 0.0f;
    float probe_variability_record_ms = 0.0f;
    float frame_graph_execute_ms = 0.0f;
    float probe_visualization_record_ms = 0.0f;
    float probe_ray_visualization_record_ms = 0.0f;
  };

  enum DdgiUpdateReason : uint32_t {
    DdgiUpdateReasonNone = 0u,
    DdgiUpdateReasonSource = 1u << 0u,
    DdgiUpdateReasonManualReset = 1u << 1u,
    DdgiUpdateReasonSteadyState = 1u << 2u,
    DdgiUpdateReasonConverged = 1u << 3u,
    DdgiUpdateReasonWarmup = 1u << 4u,
    DdgiUpdateReasonSceneInput = 1u << 5u
  };

  struct DdgiVolumeRuntimeInfo {
    uint32_t sorted_index = 0;
    uint32_t owner_index = 0;
    glm::ivec3 probe_counts = {1, 1, 1};
    uint32_t probe_count = 1;
  };

  struct DdgiProbeDebugDataView {
    const std::vector<glm::vec4>* metadata = nullptr;
    const std::vector<float>* update_ages = nullptr;
    const std::vector<PointCloudSample>* selected_ray_samples = nullptr;
    uint32_t probe_count = 0;
    uint32_t selected_ray_probe_index = 0;
    uint32_t selected_ray_physical_probe_index = 0;
    uint32_t selected_ray_sample_count = 0;
    bool selected_ray_samples_available = false;
  };

  [[nodiscard]] static uint32_t GetDdgiProbeCount(const glm::ivec3& probe_counts);
  [[nodiscard]] static uint32_t GetDdgiAllocatedProbeCount(const DdgiSettings& settings);
  [[nodiscard]] static uint32_t GetDdgiAllocatedProbeCount(const DdgiSettings& settings, uint32_t probe_count);
  [[nodiscard]] static glm::uvec3 GetDdgiProbeGridIndex(const glm::ivec3& probe_counts, uint32_t probe_index);
  [[nodiscard]] static DdgiAtlasLayout CalculateDdgiAtlasLayout(uint32_t probe_count, uint32_t tile_resolution,
                                                                uint32_t preferred_columns);
  [[nodiscard]] static DdgiProbeDebugCoordinates CalculateDdgiProbeDebugCoordinates(const DdgiSettings& settings,
                                                                                    uint32_t tile_resolution);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateDdgiFrameResourceLayout(const DdgiSettings& settings);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateDdgiFrameResourceLayout(const DdgiSettings& settings,
                                                                                uint32_t probe_count);
  [[nodiscard]] static float CalculateDdgiUpdateHysteresis(const DdgiSettings& settings, uint32_t update_reasons);
  [[nodiscard]] static float CalculateDdgiUpdateHysteresis(const DdgiSettings& settings, uint32_t update_reasons,
                                                           uint32_t warmup_frame_index);
  [[nodiscard]] static float CalculateDdgiUpdateBrightnessThreshold(const DdgiSettings& settings,
                                                                    uint32_t update_reasons);
  [[nodiscard]] static std::string FormatDdgiUpdateReasons(uint32_t reasons);
  [[nodiscard]] static float CalculateDdgiVolumeBlendWeight(const glm::vec3& probe_coordinate,
                                                            const glm::ivec3& probe_counts,
                                                            const glm::vec3& probe_step_lengths);
  [[nodiscard]] static std::vector<DdgiVolumeRuntimeInfo> CollectDdgiVolumeRuntimeInfos(
      const std::shared_ptr<Scene>& scene, const DdgiSettings& settings);

  /// Specifies whether wireframe rendering is enabled.
  bool wire_frame = false;

  /// Specifies whether shadow-rendering draw calls should be counted.
  bool count_shadow_rendering_draw_calls = true;

  /// Specifies whether meshlet rendering is enabled.
  bool enable_meshlet = true;

  /// Specifies whether indirect rendering is enabled.
  bool enable_indirect_rendering = true;

  /// Forces the inspection window into a DDGI-first layout for automated visual captures.
  bool force_ddgi_inspection_layout = false;
  glm::vec2 forced_inspection_window_position = {420.0f, 78.0f};
  glm::vec2 forced_inspection_window_size = {620.0f, 760.0f};

  /// Specifies the rendering settings.
  RenderSettings render_settings{};

  [[nodiscard]] DdgiSettings& GetDdgiSettings();
  [[nodiscard]] const DdgiSettings& GetDdgiSettings() const;
  [[nodiscard]] glm::ivec3 GetDdgiProbeScrollOffset() const;
  [[nodiscard]] glm::ivec3 GetDdgiLastProbeScrollDelta() const;
  [[nodiscard]] uint32_t GetDdgiPendingProbeUpdateCount() const;
  [[nodiscard]] DdgiProbeUpdateStats GetDdgiLastProbeUpdateStats() const;
  [[nodiscard]] DdgiPerformanceStats GetDdgiLastPerformanceStats() const;
  [[nodiscard]] uint32_t GetDdgiLastProbeUpdateReasons() const;
  [[nodiscard]] std::string GetDdgiLastProbeUpdateReasonText() const;
  [[nodiscard]] DdgiProbeDebugDataView GetDdgiProbeDebugData(bool refresh_readback) const;

  /**
   * \brief Draws a mesh.
   * \param mesh The mesh to draw.
   * \param material The material to use for rendering the mesh.
   * \param global_transform The global transform of the mesh.
   * \param cast_shadow Specifies whether the mesh casts a shadow.
   * \return Number of primitives drawn.
   */
  [[maybe_unused]] uint32_t DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                     const GlobalTransform& global_transform, bool cast_shadow) const;

  /**
   * \brief Draws a mesh with instancing.
   * \param mesh The mesh to draw.
   * \param material The material to use for rendering the mesh.
   * \param global_transform The global transform of the mesh.
   * \param particle_info_list Instance particle information.
   * \param cast_shadow Specifies whether the mesh casts a shadow.
   * \return Number of primitives drawn.
   */
  [[maybe_unused]] uint32_t DrawMeshInstanced(const std::shared_ptr<Mesh>& mesh,
                                              const std::shared_ptr<Material>& material,
                                              const GlobalTransform& global_transform,
                                              const std::shared_ptr<ParticleInfoList>& particle_info_list,
                                              bool cast_shadow) const;

  /**
   * \brief Retrieves the per-frame descriptor set.
   * \return A shared pointer to the per-frame descriptor set.
   */
  [[nodiscard]] static const std::shared_ptr<DescriptorSet>& GetPerFrameDescriptorSet();

  /**
   * \brief Retrieves the lighting descriptor set.
   * \return A shared pointer to the lighting descriptor set.
   */
  [[nodiscard]] static const std::shared_ptr<DescriptorSet>& GetLightingDescriptorSet();

  /// Represents the view for a specific point light shadow map.
  struct PointLightShadowMapView {
    int light_index;      ///< Index of the light.
    int face_index;       ///< Index of the face for cube map rendering.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /// Represents the view for a specific spot light shadow map.
  struct SpotLightShadowMapView {
    int light_index;      ///< Index of the light.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /// Represents the view for a specific directional light shadow map.
  struct DirectionalLightShadowMapView {
    int light_index;      ///< Index of the light.
    int split_index;      ///< Index of the split for cascaded shadow maps.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /// Represents the view for deferred rendering.
  struct DeferredRenderingView {
    int camera_index;     ///< Index of the target camera.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /// Represents the view for forward rendering.
  struct ForwardRenderingView {
    int camera_index;     ///< Index of the target camera.
    glm::ivec4 viewport;  ///< The viewport rectangle for rendering.
  };

  /**
   * \brief Register per-frame function to render to all point light shadow maps.
   * \param func Render function targeting point light shadow map. Return primitive count.
   */
  void RenderToPointLightShadowMap(std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                                                          const PointLightShadowMapView& shadow_map_view)>&& func);

  /**
   * \brief Register per-frame function to render to all spot light shadow maps.
   * \param func Render function targeting point light shadow map. Return primitive count.
   */
  void RenderToSpotLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>&& func);

  /**
   * \brief Register per-frame function to render to all directional light shadow maps.
   * \param func Render function targeting point light shadow map. Return primitive count.
   */
  void RenderToDirectionalLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>&&
          func);

  /**
   * \brief Register per-frame function to render to all cameras using deferred rendering.
   * \param func Render function targeting deferred rendering cameras. Return primitive count.
   */
  void DeferredRenderingAllCameras(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                             const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                             const DeferredRenderingView& forward_rendering_view)>&& func);

  /**
   * \brief Register per-frame function to render to all cameras using forward rendering.
   * \param func Render function targeting forward rendering cameras. Return primitive count.
   */
  void ForwardRenderingAllCameras(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                             const ForwardRenderingView& forward_rendering_view)>&& func);

  /**
   * \brief Register a render graph resource descriptor for this frame.
   * \param descriptor Resource metadata used by frame and camera graph passes.
   */
  void RegisterRenderResource(RenderResourceDescriptor descriptor);

  /**
   * \brief Register a per-frame render pass with explicit render graph metadata.
   * \param descriptor Render graph pass descriptor for the custom frame pass.
   * \param func Render function executed once per frame. Return primitive count.
   */
  void RegisterFrameRenderPass(RenderPassDescriptor descriptor,
                               std::function<uint32_t(VkCommandBuffer vk_command_buffer)>&& func);

  /**
   * \brief Register a per-frame render pass with access to render graph execution metadata.
   * \param descriptor Render graph pass descriptor for the custom frame pass.
   * \param func Render function executed once per frame. Return primitive count.
   */
  void RegisterFrameRenderPass(
      RenderPassDescriptor descriptor,
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context)>&& func);

  /**
   * \brief Register a per-frame camera render pass with explicit render graph metadata.
   * \param descriptor Render graph pass descriptor for the custom camera pass.
   * \param func Render function targeting each camera. Return primitive count.
   */
  void RegisterCameraRenderPass(
      RenderPassDescriptor descriptor,
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                             const ForwardRenderingView& forward_rendering_view)>&& func);

  /**
   * \brief Register a per-frame camera render pass with access to render graph execution metadata.
   * \param descriptor Render graph pass descriptor for the custom camera pass.
   * \param func Render function targeting each camera. Return primitive count.
   */
  void RegisterCameraRenderPass(
      RenderPassDescriptor descriptor,
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                             const ForwardRenderingView& forward_rendering_view,
                             const RenderGraphExecutionContext& context)>&& func);

  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetPerFrameDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetMeshletDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetLightingDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetRayTracingDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetRayTracingPointCloudDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetParticleInstancedDataDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetBoneMatricesDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetCameraGBufferDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetRenderTextureStorageDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetRenderTexturePresentDescriptorSetLayout() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetRasterMaterialDescriptorSetLayout() const;

 private:
  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>>
      point_light_shadow_map_external_functions;

  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>>
      spot_light_shadow_map_external_functions;

  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>>
      directional_light_shadow_map_external_functions;

  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                                     const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                                     const DeferredRenderingView& forward_rendering_view)>>
      deferred_rendering_external_functions;

  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                                     const ForwardRenderingView& forward_rendering_view)>>
      forward_rendering_external_functions;
  struct FrameRenderPassExternalFunction {
    RenderPassDescriptor descriptor;
    std::function<uint32_t(VkCommandBuffer vk_command_buffer)> func;
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context)> context_func;
  };
  struct CameraRenderPassExternalFunction {
    RenderPassDescriptor descriptor;
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                           const ForwardRenderingView& forward_rendering_view)>
        func;
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                           const ForwardRenderingView& forward_rendering_view,
                           const RenderGraphExecutionContext& context)>
        context_func;
  };
  std::vector<RenderResourceDescriptor> external_render_resource_descriptors;
  std::vector<FrameRenderPassExternalFunction> frame_render_pass_external_functions;
  std::vector<CameraRenderPassExternalFunction> camera_render_pass_external_functions;
  mutable std::vector<RenderGraphTransientResourceStore> render_graph_transient_resource_stores_;
  mutable std::shared_ptr<Buffer> ddgi_probe_metadata_buffer_;
  mutable std::shared_ptr<Buffer> ddgi_probe_state_buffer_;
  mutable std::shared_ptr<Buffer> ddgi_fallback_probe_state_buffer_;
  mutable std::shared_ptr<Buffer> ddgi_probe_update_index_buffer_;
  mutable std::shared_ptr<Image> ddgi_irradiance_atlas_;
  mutable std::shared_ptr<Image> ddgi_visibility_atlas_;
  mutable std::shared_ptr<Image> ddgi_variability_atlas_;
  mutable std::shared_ptr<Sampler> ddgi_atlas_sampler_;
  mutable std::shared_ptr<Buffer> ddgi_probe_metadata_readback_buffer_;
  mutable std::shared_ptr<Buffer> ddgi_probe_ray_readback_buffer_;
  mutable std::shared_ptr<Buffer> ddgi_variability_readback_buffer_;
  mutable std::shared_ptr<Buffer> ddgi_frame_ray_output_visualization_buffer_;
  mutable std::vector<glm::vec4> ddgi_probe_debug_metadata_;
  mutable std::vector<float> ddgi_probe_debug_update_ages_;
  mutable std::vector<PointCloudSample> ddgi_probe_debug_ray_samples_;
  mutable uint64_t ddgi_probe_debug_metadata_byte_size_ = 0;
  mutable uint32_t ddgi_probe_debug_metadata_probe_count_ = 0;
  mutable uint32_t ddgi_probe_debug_ray_probe_index_ = 0;
  mutable uint32_t ddgi_probe_debug_ray_physical_probe_index_ = 0;
  mutable uint32_t ddgi_probe_debug_ray_sample_count_ = 0;
  mutable bool ddgi_probe_debug_ray_samples_available_ = false;
  friend class Platform;
  friend class Resources;
  friend class Camera;
  friend class GraphicsPipeline;
  friend class EditorLayer;
  friend class Material;
  friend class Lighting;
  friend class PostProcessingStack;
  friend class Application;
  friend class OffscreenPreviewRenderer;
  friend class RenderInstanceStorage;
  friend class TextureStorage;
#pragma region DescriptorSet Layouts
  std::shared_ptr<DescriptorSetLayout> empty_descriptor_set_layout_;
  std::shared_ptr<DescriptorSetLayout> per_frame_layout_;
  std::shared_ptr<DescriptorSetLayout> raster_material_per_frame_layout_;
  std::shared_ptr<DescriptorSetLayout> meshlet_layout_;
  std::shared_ptr<DescriptorSetLayout> lighting_layout_;
  std::shared_ptr<DescriptorSetLayout> ray_tracing_layout_;
  std::shared_ptr<DescriptorSetLayout> ray_tracing_camera_output_layout_;
  std::shared_ptr<DescriptorSetLayout> ray_tracing_point_cloud_layout_;
  std::shared_ptr<DescriptorSetLayout> ddgi_probe_ray_output_layout_;
  std::shared_ptr<DescriptorSetLayout> particle_instanced_data_layout_;
  std::shared_ptr<DescriptorSetLayout> bone_matrices_layout_;
  std::shared_ptr<DescriptorSetLayout> camera_g_buffer_layout_;
  std::shared_ptr<DescriptorSetLayout> render_texture_storage_layout_;
  std::shared_ptr<DescriptorSetLayout> render_texture_present_layout_;
  std::shared_ptr<DescriptorSetLayout> raster_lighting_texture_layout_;
  std::shared_ptr<DescriptorSetLayout> motion_vectors_layout_;
  std::shared_ptr<DescriptorSetLayout> depth_pyramid_layout_;
  std::shared_ptr<DescriptorSetLayout> volumetric_clouds_layout_;
  std::shared_ptr<DescriptorSetLayout> ddgi_probe_update_layout_;
  std::shared_ptr<DescriptorSetLayout> ddgi_probe_relocation_layout_;
  std::shared_ptr<DescriptorSetLayout> ddgi_probe_classification_layout_;
  std::shared_ptr<DescriptorSetLayout> ddgi_probe_variability_layout_;
  std::shared_ptr<DescriptorSetLayout> ddgi_probe_visualization_layout_;
  std::shared_ptr<DescriptorSetLayout> ddgi_probe_ray_visualization_layout_;
  std::shared_ptr<DescriptorSetLayout> gaussian_splat_layout_;
  std::shared_ptr<DescriptorSetLayout> gaussian_splat_radix_sort_layout_;
  std::shared_ptr<DescriptorSetLayout> raster_material_layout_;
  bool per_frame_bindless_texture_descriptors_enabled_ = false;
  mutable std::shared_ptr<Texture2D> raster_material_white_fallback_texture_;
  mutable std::shared_ptr<Texture2D> raster_material_black_fallback_texture_;
  mutable std::shared_ptr<Texture2D> raster_material_flat_normal_fallback_texture_;

  void InitializeCommonDescriptorSetLayouts(
      const ApplicationInitializationSettings& application_initialization_settings);
  void EnsureRasterMaterialFallbackTextures() const;
  [[nodiscard]] std::array<VkDescriptorImageInfo, RenderInstanceStorage::kRasterMaterialTextureSlotCount>
  GetRasterMaterialFallbackDescriptorImageInfos() const;
#pragma endregion

  std::vector<std::shared_ptr<RenderInstanceStorage>> render_instances_list_;
  bool need_fade_ = false;
  uint32_t ddgi_active_probe_update_reasons_ = DdgiUpdateReasonNone;
  uint32_t ddgi_last_probe_update_reasons_ = DdgiUpdateReasonNone;
  bool ddgi_has_previous_ray_source_ = false;
  bool ddgi_frame_trace_probe_rays_ = false;
  bool ddgi_clear_probe_atlas_this_frame_ = false;
  DdgiSettings fallback_ddgi_settings_{};
  uint32_t ddgi_next_probe_update_index_ = 0;
  uint32_t ddgi_pending_probe_update_count_ = 0;
  glm::ivec3 ddgi_previous_probe_counts_ = {0, 0, 0};
  glm::vec3 ddgi_previous_first_probe_ = glm::vec3(0.0f);
  glm::vec3 ddgi_previous_probe_step_x_ = glm::vec3(0.0f);
  glm::vec3 ddgi_previous_probe_step_y_ = glm::vec3(0.0f);
  glm::vec3 ddgi_previous_probe_step_z_ = glm::vec3(0.0f);
  glm::vec4 ddgi_previous_trace_parameters_ = glm::vec4(0.0f);
  glm::vec4 ddgi_previous_update_parameters_ = glm::vec4(0.0f);
  glm::vec4 ddgi_previous_probe_state_parameters_ = glm::vec4(0.0f);
  glm::vec4 ddgi_previous_probe_blend_parameters_ = glm::vec4(0.0f);
  int ddgi_previous_movement_type_ = static_cast<int>(DdgiVolumeMovementType::Default);
  bool ddgi_has_previous_scene_inputs_ = false;
  RenderInstanceStorage::EnvironmentInfoBlock ddgi_previous_environment_info_block_{};
  std::vector<GltfShadeMaterial> ddgi_previous_gltf_shade_materials_;
  std::vector<GltfTextureInfo> ddgi_previous_gltf_texture_infos_;
  uint32_t ddgi_previous_texture_storage_version_ = 0;
  bool ddgi_scene_material_inputs_changed_ = false;
  bool ddgi_deferred_scene_readiness_refresh_ = false;
  uint32_t ddgi_scene_input_settle_frame_count_ = 0;
  std::vector<uint64_t> ddgi_previous_active_light_keys_;
  std::vector<uint64_t> ddgi_previous_light_signatures_;
  std::vector<uint64_t> ddgi_previous_geometry_signatures_;
  glm::vec3 ddgi_probe_scroll_base_first_probe_ = glm::vec3(0.0f);
  glm::ivec3 ddgi_probe_scroll_offset_ = glm::ivec3(0);
  glm::ivec3 ddgi_probe_scroll_clear_ = glm::ivec3(0);
  glm::ivec3 ddgi_probe_scroll_directions_ = glm::ivec3(1);
  glm::ivec3 ddgi_last_probe_scroll_delta_ = glm::ivec3(0);
  uint32_t ddgi_previous_ray_count_ = 0;
  std::vector<uint32_t> ddgi_pending_probe_update_indices_;
  uint32_t ddgi_pending_probe_update_cursor_ = 0;
  std::vector<uint32_t> ddgi_frame_probe_update_indices_;
  float ddgi_probe_variability_average_ = 0.0f;
  uint32_t ddgi_probe_variability_sample_count_ = 0;
  uint32_t ddgi_probe_variability_stable_sample_count_ = 0;
  bool ddgi_probe_variability_converged_ = false;
  uint32_t ddgi_probe_warmup_frame_index_ = 0;
  uint32_t ddgi_frame_probe_warmup_frame_index_ = 0;
  uint32_t ddgi_frame_probe_warmup_frame_count_ = 0;
  bool ddgi_frame_probe_warmup_active_ = false;
  float ddgi_frame_probe_update_hysteresis_ = 0.0f;
  bool ddgi_frame_probe_relocation_reset_ = false;
  bool ddgi_frame_probe_relocation_enabled_ = false;
  bool ddgi_frame_probe_classification_reset_ = false;
  bool ddgi_frame_probe_classification_enabled_ = false;
  bool ddgi_frame_probe_variability_enabled_ = false;
  int ddgi_scene_change_triggers_ = DdgiVolumeTriggerConditionNone;
  uint32_t ddgi_frame_selected_probe_ray_local_index_ = (std::numeric_limits<uint32_t>::max)();
  uint32_t ddgi_frame_selected_probe_ray_sample_count_ = 0;
  DdgiFrameResourceLayout ddgi_frame_resource_layout_{};
  mutable DdgiPerformanceStats ddgi_last_performance_stats_{};
  DdgiProbeRayTracingPushConstant ddgi_frame_ray_push_constant_{};
  DdgiProbeAtlasUpdatePushConstant ddgi_frame_probe_update_push_constant_{};
  DdgiProbeRelocationPushConstant ddgi_frame_probe_relocation_reset_push_constant_{};
  DdgiProbeRelocationPushConstant ddgi_frame_probe_relocation_update_push_constant_{};
  DdgiProbeClassificationPushConstant ddgi_frame_probe_classification_reset_push_constant_{};
  DdgiProbeClassificationPushConstant ddgi_frame_probe_classification_update_push_constant_{};
  std::unique_ptr<Lighting> lighting_;
  std::shared_ptr<Texture2D> environmental_brdf_lut_ = {};
  /**
   * \brief Called after the RenderLayer object is created.
   */
  void OnCreate() override;

 private:
  /**
   * \brief Prepares shadow maps for point and spot lights.
   */
  void PreparePointAndSpotLightShadowMap() const;

  /**
   * \brief Prepares the environmental BRDF LUT texture.
   */
  void PrepareEnvironmentalBrdfLut();

  /**
   * \brief Renders to the specified camera.
   * \param scene The scene whose environment settings apply to the camera render.
   * \param camera_global_transform The global transform of the camera.
   * \param camera The target camera to render to.
   */
  void RenderToCamera(const std::shared_ptr<Scene>& scene, const GlobalTransform& camera_global_transform,
                      const std::shared_ptr<Camera>& camera, bool immediate = false) const;

  /**
   * \brief Renders to the specified camera using ray tracing.
   * \param scene The scene whose environment settings apply to the camera render.
   * \param camera_global_transform The global transform of the camera.
   * \param camera The target camera to render to.
   */
  void RenderToCameraRayTracing(const std::shared_ptr<Scene>& scene, const GlobalTransform& camera_global_transform,
                                const std::shared_ptr<Camera>& camera) const;

  /**
   * \brief Called before updating this render layer.
   */
  void PreUpdate() override;

  /**
   * \brief Clears all editor cameras associated with this render layer.
   */
  void ClearAllEditorCameras() const;

  /**
   * \brief Clears all cameras associated with this render layer.
   */
  void ClearAllCameras() const;

  /**
   * \brief Prepares the render layer for rendering.
   */
  void PrepareForRendering();
  void PrepareSceneForRendering(const std::shared_ptr<Scene>& scene, bool include_editor_cameras = true,
                                bool update_editor_selection = true, bool update_ray_tracing = true,
                                bool track_ddgi_scene_inputs = true);

  void PrepareDdgiFrameState(const std::shared_ptr<Scene>& scene,
                             const std::shared_ptr<RenderInstanceStorage>& render_instances,
                             int ddgi_scene_change_triggers);
  void RenderSceneToCameraImmediately(const std::shared_ptr<Scene>& scene,
                                      const GlobalTransform& camera_global_transform,
                                      const std::shared_ptr<Camera>& camera);

  /**
   * \brief Performs all rendering operations for this render layer.
   */
  void RenderAll();

  /**
   * \brief Renders all gizmos associated with this render layer.
   */
  void RenderGizmos() const;

  /**
   * \brief Updates the render instance storage based on the provided scene.
   * \param scene The scene associated with this render layer.
   * \param current_frame_index The index of the current frame.
   * \param track_ddgi_scene_inputs Whether this scene should update active DDGI change tracking.
   * \return True if scene-wide render changes require all collected camera histories to reset.
   */
  bool UpdateRenderInstanceStorage(const std::shared_ptr<Scene>& scene, uint32_t current_frame_index,
                                   bool include_editor_cameras = true, bool update_editor_selection = true,
                                   bool track_ddgi_scene_inputs = true);
  void BindRenderInstanceStorage(uint32_t current_frame_index,
                                 const std::shared_ptr<RenderInstanceStorage>& render_instances) const;
  [[nodiscard]] std::shared_ptr<DescriptorSet> GetRasterLightingTextureDescriptorSet(
      uint32_t current_frame_index, int camera_index,
      const std::shared_ptr<RenderInstanceStorage>& render_instances) const;

  /**
   * \brief Applies all animators associated with this render layer.
   */
  void ApplyAnimators() const;

  /**
   * \brief Drains GPU work before render-layer resources are released.
   */
  void OnDestroy() override;

  friend class TextureStorage;
  std::vector<std::shared_ptr<DescriptorSet>> per_frame_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> raster_material_per_frame_descriptor_sets_ = {};
  mutable std::vector<std::vector<std::shared_ptr<DescriptorSet>>> raster_lighting_texture_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> meshlet_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> ray_tracing_descriptor_sets_ = {};
  std::vector<std::shared_ptr<Buffer>> kernel_descriptor_buffers_ = {};

#pragma region Graphics Pipelines
  /// Depth-only pipeline for rendering point light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_normal_opaque;

  /// Graphics pipeline for rendering point light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_mesh_shader;

  /// Depth-only pipeline for rendering spot light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_normal_opaque;

  /// Graphics pipeline for rendering spot light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_mesh_shader;

  /// Depth-only pipeline for rendering directional light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_normal_opaque;

  /// Graphics pipeline for rendering directional light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_mesh_shader;

  /// Depth-only pipeline for rendering instanced point light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_point_light_shadow_pipeline_opaque;

  /// Depth-only pipeline for rendering instanced spot light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_spot_light_shadow_pipeline_opaque;

  /// Depth-only pipeline for rendering instanced directional light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_directional_light_shadow_pipeline_opaque;

  /// Depth-only pipeline for rendering point light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_point_light_shadow_pipeline_opaque;

  /// Depth-only pipeline for rendering spot light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_spot_light_shadow_pipeline_opaque;

  /// Depth-only pipeline for rendering directional light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_directional_light_shadow_pipeline_opaque;

  /// Graphics pipeline for rendering point light shadows with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_point_light_shadow_pipeline;

  /// Graphics pipeline for rendering spot light shadows with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_spot_light_shadow_pipeline;

  /// Graphics pipeline for rendering directional light shadows with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_directional_light_shadow_pipeline;

  /// Graphics pipeline for the deferred shading GBuffer pre-pass using normal meshes.
  std::shared_ptr<GraphicsPipeline> deferred_prepass_pipeline_normal;

  /// Graphics pipeline for the deferred shading GBuffer pre-pass using mesh shaders.
  std::shared_ptr<GraphicsPipeline> deferred_prepass_pipeline_mesh;

  /// Graphics pipeline for rendering instanced deferred shading GBuffer pre-pass.
  std::shared_ptr<GraphicsPipeline> instanced_deferred_prepass_pipeline;

  /// Graphics pipeline for rendering deferred shading GBuffer pre-pass with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_deferred_prepass_pipeline;

  /// Graphics pipeline for rendering deferred shading GBuffer pre-pass with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_deferred_prepass_pipeline;

  /// Graphics pipeline for performing the deferred shading lighting pass.
  std::shared_ptr<GraphicsPipeline> deferred_lighting_pass_pipeline;

  /// Graphics pipeline for performing the deferred shading lighting pass with scene cameras.
  std::shared_ptr<GraphicsPipeline> deferred_lighting_pass_pipeline_scene_camera;

  /// Graphics pipeline for rendering transparent normal meshes after deferred lighting.
  std::shared_ptr<GraphicsPipeline> transparent_geometry_pipeline_normal;

  /// Graphics pipeline for rendering gizmos.
  std::shared_ptr<GraphicsPipeline> gizmos;

  /// Graphics pipeline for rendering gizmos with normal-colored shaders.
  std::shared_ptr<GraphicsPipeline> gizmos_normal_colored;

  /// Graphics pipeline for rendering gizmos with vertex-colored shaders.
  std::shared_ptr<GraphicsPipeline> gizmos_vertex_colored;

  /// Graphics pipeline for rendering instanced gizmos.
  std::shared_ptr<GraphicsPipeline> gizmos_instanced_colored;

  std::shared_ptr<GraphicsPipeline> ddgi_probe_visualization_pipeline_;
  std::shared_ptr<GraphicsPipeline> ddgi_probe_ray_visualization_pipeline_;
  std::shared_ptr<GraphicsPipeline> gaussian_splat_pipeline_;
  std::shared_ptr<GraphicsPipeline> gaussian_splat_overlay_pipeline_;
  std::shared_ptr<GraphicsPipeline> gaussian_splat_mesh_pipeline_;
  std::shared_ptr<GraphicsPipeline> gaussian_splat_mesh_overlay_pipeline_;

  /// Graphics pipeline for rendering gizmos on hair strands.
  std::shared_ptr<GraphicsPipeline> gizmos_strands;

  /// Graphics pipeline for rendering gizmos with normal-colored shaders on hair strands.
  std::shared_ptr<GraphicsPipeline> gizmos_strands_normal_colored;

  /// Graphics pipeline for rendering gizmos with vertex-colored shaders on hair strands.
  std::shared_ptr<GraphicsPipeline> gizmos_strands_vertex_colored;
#pragma endregion

  std::shared_ptr<ComputePipeline> depth_pyramid_pipeline_;
  std::shared_ptr<ComputePipeline> motion_vectors_pipeline_;
  std::shared_ptr<ComputePipeline> volumetric_clouds_pipeline_;
  std::shared_ptr<ComputePipeline> volumetric_clouds_composite_pipeline_;
  std::shared_ptr<ComputePipeline> gaussian_splat_cull_pipeline_;
  std::shared_ptr<ComputePipeline> gaussian_splat_radix_upsweep_pipeline_;
  std::shared_ptr<ComputePipeline> gaussian_splat_radix_spine_pipeline_;
  std::shared_ptr<ComputePipeline> gaussian_splat_radix_downsweep_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_update_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_relocation_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_classification_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_variability_reduce_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_variability_extra_reduce_pipeline_;
  std::shared_ptr<ComputePipeline> ray_query_camera_pipeline_;

#pragma region Ray Tracing Pipelines
  /// Ray tracing pipeline for rendering cameras with ray tracing.
  std::shared_ptr<RayTracingPipeline> ray_tracing_camera_pipeline;
  /// Ray tracing pipeline for rendering cameras with ray tracing.
  friend class PointCloud;
  std::shared_ptr<RayTracingPipeline> ray_tracing_point_cloud_pipeline;
  std::shared_ptr<RayTracingPipeline> ddgi_probe_ray_diagnostic_pipeline_;
#pragma endregion
};
}  // namespace evo_engine
