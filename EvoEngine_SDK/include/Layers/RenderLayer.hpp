
#pragma once
#include "Camera.hpp"
#include "IGeometry.hpp"
#include "ILayer.hpp"

#include "DdgiRuntime.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "PointCloudSample.hpp"
#include "RayCameraShaderVariantCache.hpp"
#include "RenderGraph.hpp"
#include "RenderInstanceStorage.hpp"
#include "ResolvedEnvironmentalLighting.hpp"

#include <array>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

namespace evo_engine {
struct ApplicationInitializationSettings;
class ComputePipeline;
class GlobalReflectionProbe;
class OffscreenPreviewRenderer;
class ReflectionProbe;
class Sampler;
struct FrameSubmissionState;
struct PostProcessingRendererResources;

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

  [[nodiscard]] bool RequiresCameraWideTemporalHistoryRejection() const;

  using DdgiSettings = evo_engine::DdgiSettings;
  using DdgiAtlasLayout = evo_engine::DdgiAtlasLayout;
  using DdgiFrameResourceLayout = evo_engine::DdgiFrameResourceLayout;
  using DdgiRuntimePolicy = evo_engine::DdgiRuntimePolicy;
  using DdgiPerformanceStats = evo_engine::DdgiPerformanceStats;
  using DdgiUpdateReason = evo_engine::DdgiUpdateReason;
  using DdgiVolumeRuntimeInfo = evo_engine::DdgiVolumeRuntimeInfo;
  using DdgiVolumeRuntimeStats = evo_engine::DdgiVolumeRuntimeStats;
  using DdgiVolumeSetValidation = evo_engine::DdgiVolumeSetValidation;
  using DdgiVolumeSelection = evo_engine::DdgiVolumeSelection;
  using DdgiProbeUpdateVariant = evo_engine::DdgiProbeUpdateVariant;
  using DdgiProbeVariabilityObservation = evo_engine::DdgiProbeVariabilityObservation;
  using DdgiProbeConvergenceState = evo_engine::DdgiProbeConvergenceState;
  using DdgiProbeConvergenceUpdate = evo_engine::DdgiProbeConvergenceUpdate;
  using DdgiProbeUpdateDeviceLimits = evo_engine::DdgiProbeUpdateDeviceLimits;
  using DdgiProbeDebugDataView = evo_engine::DdgiProbeDebugDataView;

  static constexpr uint32_t DdgiUpdateReasonNone = evo_engine::DdgiUpdateReasonNone;
  static constexpr uint32_t DdgiUpdateReasonSource = evo_engine::DdgiUpdateReasonSource;
  static constexpr uint32_t DdgiUpdateReasonManualReset = evo_engine::DdgiUpdateReasonManualReset;
  static constexpr uint32_t DdgiUpdateReasonSteadyState = evo_engine::DdgiUpdateReasonSteadyState;
  static constexpr uint32_t DdgiUpdateReasonConverged = evo_engine::DdgiUpdateReasonConverged;
  static constexpr uint32_t DdgiUpdateReasonWarmup = evo_engine::DdgiUpdateReasonWarmup;
  static constexpr uint32_t DdgiUpdateReasonSceneInput = evo_engine::DdgiUpdateReasonSceneInput;
  static constexpr uint32_t DdgiUpdateReasonPeriodicRefresh = evo_engine::DdgiUpdateReasonPeriodicRefresh;
  static constexpr uint32_t DdgiUpdateReasonVariabilityPolicy = evo_engine::DdgiUpdateReasonVariabilityPolicy;

  static constexpr uint32_t kDdgiProbeUpdateGroupSize = DdgiRuntime::kProbeUpdateGroupSize;
  static constexpr uint32_t kDdgiProbeUpdateSharedMemoryBytes = DdgiRuntime::kProbeUpdateSharedMemoryBytes;
  static constexpr uint32_t kDdgiProbeVariabilityStableSampleCount = DdgiRuntime::kProbeVariabilityStableSampleCount;
  static constexpr float kDdgiProbeVariabilityExitThresholdScale = DdgiRuntime::kProbeVariabilityExitThresholdScale;
  static constexpr uint32_t kDdgiProbeRefreshInterval = DdgiRuntime::kProbeRefreshInterval;
  static constexpr uint32_t kDdgiMaxVolumeCount = DdgiRuntime::kMaxVolumeCount;
  static constexpr uint32_t kDdgiMaxResidentProbeCount = DdgiRuntime::kMaxResidentProbeCount;
  static constexpr uint32_t kDdgiProbeRayFlagSkipInactive = DdgiRuntime::kProbeRayFlagSkipInactive;
  static constexpr uint32_t kDdgiProbeRayFlagEmissiveMeshSampling = DdgiRuntime::kProbeRayFlagEmissiveMeshSampling;

  [[nodiscard]] static uint32_t GetDdgiProbeCount(const glm::ivec3& probe_counts);
  [[nodiscard]] static uint32_t GetDdgiFixedRayCount(uint32_t ray_count, bool fixed_rays_enabled);
  [[nodiscard]] static DdgiProbeUpdateVariant ParseDdgiProbeUpdateVariant(std::string_view value);
  [[nodiscard]] static DdgiProbeConvergenceUpdate AdvanceDdgiProbeConvergence(
      const DdgiProbeConvergenceState& state, const DdgiProbeVariabilityObservation& observation,
      uint32_t minimum_sample_count, float entry_threshold);
  [[nodiscard]] static bool IsDdgiPeriodicRefreshDue(bool gating_enabled, bool converged, bool waiting_for_observation,
                                                     uint32_t refresh_age);
  [[nodiscard]] static bool IsDdgiReflectionProbeRuntimeReady(bool has_valid_history, bool lighting_descriptors_bound,
                                                              bool variability_gating_enabled,
                                                              bool variability_converged);
  [[nodiscard]] static DdgiProbeUpdateVariant ResolveDdgiProbeUpdateVariant(DdgiProbeUpdateVariant requested,
                                                                            const DdgiProbeUpdateDeviceLimits& limits,
                                                                            uint32_t probe_count,
                                                                            bool irradiance_pipeline_ready,
                                                                            bool visibility_pipeline_ready);
  [[nodiscard]] static uint32_t GetDdgiAllocatedProbeCount(const DdgiSettings& settings);
  [[nodiscard]] static uint32_t GetDdgiAllocatedProbeCount(const DdgiSettings& settings, uint32_t probe_count);
  [[nodiscard]] static bool ValidateDdgiProbeGrid(const glm::ivec3& probe_counts, uint32_t max_probe_count,
                                                  std::string* error = nullptr);
  [[nodiscard]] static DdgiRuntimePolicy ResolveDdgiRuntimePolicy(const DdgiSettings& settings);
  [[nodiscard]] static bool ResolveDdgiEmissiveMeshSampling(bool global_enabled, int volume_mode);
  [[nodiscard]] static uint32_t GetDdgiProbeRayFlags(bool skip_inactive_probes, bool emissive_mesh_sampling);
  [[nodiscard]] static uint64_t CalculateDdgiEmissiveSamplingCandidateRayCount(uint32_t updated_probe_count,
                                                                               uint32_t ray_count,
                                                                               uint32_t fixed_ray_count,
                                                                               bool emissive_mesh_sampling,
                                                                               bool trace_probe_rays);
  [[nodiscard]] static bool RequiresDdgiFullScrollReset(const glm::ivec3& probe_counts, const glm::ivec3& scroll_delta);
  [[nodiscard]] static glm::uvec3 GetDdgiProbeGridIndex(const glm::ivec3& probe_counts, uint32_t probe_index);
  [[nodiscard]] static DdgiAtlasLayout CalculateDdgiAtlasLayout(uint32_t probe_count, uint32_t tile_resolution,
                                                                uint32_t preferred_columns);
  [[nodiscard]] static DdgiAtlasLayout CalculateDdgiAtlasLayout(uint32_t probe_count, uint32_t tile_resolution,
                                                                uint32_t preferred_columns,
                                                                uint32_t max_image_dimension_2d);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateDdgiFrameResourceLayout(const DdgiSettings& settings);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateDdgiFrameResourceLayout(const DdgiSettings& settings,
                                                                                uint32_t probe_count);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateDdgiFrameResourceLayout(const DdgiSettings& settings,
                                                                                uint32_t probe_count,
                                                                                uint32_t max_image_dimension_2d);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateDdgiFrameResourceLayout(const DdgiSettings& settings,
                                                                                uint32_t probe_count,
                                                                                uint32_t max_image_dimension_2d,
                                                                                uint64_t max_storage_buffer_range);
  [[nodiscard]] static bool AreDdgiPersistentLayoutsCompatible(const DdgiFrameResourceLayout& previous,
                                                               const DdgiFrameResourceLayout& current);
  [[nodiscard]] static float CalculateDdgiUpdateHysteresis(const DdgiSettings& settings, uint32_t update_reasons);
  [[nodiscard]] static float CalculateDdgiUpdateHysteresis(const DdgiSettings& settings, uint32_t update_reasons,
                                                           uint32_t warmup_frame_index);
  [[nodiscard]] static float CalculateDdgiUpdateBrightnessThreshold(const DdgiSettings& settings);
  [[nodiscard]] static std::string FormatDdgiUpdateReasons(uint32_t reasons);
  [[nodiscard]] static float CalculateDdgiVolumeBlendWeight(const glm::vec3& probe_coordinate,
                                                            const glm::ivec3& probe_counts,
                                                            const glm::vec3& probe_step_lengths);
  [[nodiscard]] static float CalculateDdgiProbeDensity(const glm::vec3& probe_step_x, const glm::vec3& probe_step_y,
                                                       const glm::vec3& probe_step_z);
  static void SortDdgiVolumeRuntimeInfos(std::vector<DdgiVolumeRuntimeInfo>& infos);
  [[nodiscard]] static DdgiVolumeSetValidation ValidateDdgiVolumeSet(const std::vector<DdgiVolumeRuntimeInfo>& infos,
                                                                     uint32_t configured_probe_limit);
  [[nodiscard]] static DdgiVolumeSelection SelectDdgiVolumes(const std::vector<DdgiVolumeRuntimeInfo>& infos,
                                                             const glm::vec3& world_position);
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

  /// Uses the permanent all-feature camera-ray shaders instead of scene-specialized variants.
  bool force_full_ray_camera_shader_variant = false;

  struct RayCameraFramePathStats {
    RenderGraphPlanCacheStats render_graph_plan_cache{};
    uint64_t live_output_descriptor_count = 0;
    uint64_t peak_live_output_descriptor_count = 0;
    uint64_t output_descriptor_creation_count = 0;
    uint64_t output_descriptor_reuse_count = 0;
    uint32_t retained_frame_slot_count = 0;
  };

  [[nodiscard]] RayCameraShaderVariantStats GetRayCameraShaderVariantStats(RayCameraShaderTechnique technique) const;
  [[nodiscard]] bool IsRayCameraShaderVariantReady(RayCameraShaderTechnique technique) const;
  [[nodiscard]] RayCameraHistoryStats GetRayCameraHistoryStats() const;
  [[nodiscard]] RayCameraFramePathStats GetRayCameraFramePathStats() const;

  [[nodiscard]] DdgiSettings& GetDdgiSettings();
  [[nodiscard]] const DdgiSettings& GetDdgiSettings() const;
  [[nodiscard]] glm::ivec3 GetDdgiProbeScrollOffset() const;
  [[nodiscard]] glm::ivec3 GetDdgiLastProbeScrollDelta() const;
  [[nodiscard]] DdgiPerformanceStats GetDdgiLastPerformanceStats() const;
  [[nodiscard]] std::vector<DdgiVolumeRuntimeStats> GetDdgiVolumeRuntimeStats() const;
  [[nodiscard]] uint32_t GetDdgiLastProbeUpdateReasons() const;
  [[nodiscard]] std::string GetDdgiLastProbeUpdateReasonText() const;
  [[nodiscard]] DdgiProbeDebugDataView GetDdgiProbeDebugData(bool refresh_readback);

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
    int light_index;               ///< Camera-local index of the light.
    int split_index;               ///< Index of the split for cascaded shadow maps.
    glm::ivec4 viewport;           ///< The viewport rectangle for rendering.
    glm::mat4 light_space_matrix;  ///< Matrix for this camera, light, and split.
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
  [[nodiscard]] const std::shared_ptr<PostProcessingRendererResources>& GetPostProcessingRendererResources() const;
  [[nodiscard]] bool QueueGlobalReflectionProbeBake(const std::shared_ptr<Scene>& scene, const glm::vec3& position,
                                                    const std::shared_ptr<GlobalReflectionProbe>& target);

 private:
  struct DdgiReadbackTicket {
    std::shared_ptr<Buffer> buffer{};
    std::shared_ptr<FrameSubmissionState> submission{};
    uint64_t generation = 0;
    uint64_t consumed_generation = 0;
    uint64_t byte_size = 0;
    uint32_t frame_index = 0;
    uint32_t element_count = 0;
    uint32_t logical_probe_index = 0;
    uint32_t physical_probe_index = 0;
  };

  struct DdgiVolumeRuntimeState {
    uint64_t stable_entity_id = 0;
    uint32_t sorted_index = 0;
    int artist_priority = 0;
    float probe_density = 0.0f;
    RenderInstanceStorage::DdgiVolumeInfoBlock gpu_info{};
    bool contributes_lighting = false;
    std::array<uint64_t, 5> resource_ids{};

    std::shared_ptr<Buffer> probe_metadata_buffer{};
    std::shared_ptr<Buffer> probe_state_buffer{};
    std::shared_ptr<Image> irradiance_atlas{};
    std::shared_ptr<Image> visibility_atlas{};
    std::shared_ptr<Image> variability_atlas{};
    std::vector<std::shared_ptr<Buffer>> probe_metadata_readback_buffers{};
    std::vector<std::shared_ptr<Buffer>> probe_ray_readback_buffers{};
    std::vector<std::shared_ptr<Buffer>> selected_ray_diagnostics_buffers{};
    DdgiReadbackTicket metadata_readback_ticket{};
    DdgiReadbackTicket ray_readback_ticket{};
    std::vector<DdgiReadbackTicket> variability_readback_tickets{};
    uint64_t next_debug_readback_generation = 0;
    uint64_t frame_variability_readback_generation = 0;
    std::shared_ptr<Buffer> frame_selected_ray_diagnostics_buffer{};
    std::vector<glm::vec4> probe_debug_metadata{};
    std::vector<PointCloudSample> probe_debug_ray_samples{};
    uint32_t probe_debug_ray_probe_index = 0;
    uint32_t probe_debug_ray_physical_probe_index = 0;

    uint32_t last_probe_update_reasons = DdgiUpdateReasonNone;
    bool has_previous_ray_source = false;
    bool has_valid_probe_history = false;
    bool emissive_mesh_sampling_enabled = true;
    bool previous_emissive_mesh_sampling_enabled = true;
    bool frame_trace_probe_rays = false;
    bool frame_clear_scrolled_probes = false;
    bool clear_probe_atlas_this_frame = false;
    glm::ivec3 previous_probe_counts = {0, 0, 0};
    glm::vec3 previous_first_probe = glm::vec3(0.0f);
    glm::vec3 previous_probe_step_x = glm::vec3(0.0f);
    glm::vec3 previous_probe_step_y = glm::vec3(0.0f);
    glm::vec3 previous_probe_step_z = glm::vec3(0.0f);
    glm::vec4 previous_trace_parameters = glm::vec4(0.0f);
    glm::vec4 previous_update_parameters = glm::vec4(0.0f);
    glm::vec4 previous_probe_state_parameters = glm::vec4(0.0f);
    glm::vec4 previous_probe_blend_parameters = glm::vec4(0.0f);
    glm::vec4 previous_probe_variability_parameters = glm::vec4(0.0f);
    int previous_movement_type = static_cast<int>(DdgiVolumeMovementType::Default);
    bool has_previous_environment_signature = false;
    uint64_t previous_environment_signature = 0;
    bool deferred_scene_readiness_refresh = false;
    bool manual_reset_pending = false;
    uint32_t scene_input_settle_frame_count = 0;
    glm::vec3 probe_scroll_base_first_probe = glm::vec3(0.0f);
    glm::ivec3 probe_scroll_offset = glm::ivec3(0);
    glm::ivec3 probe_scroll_clear = glm::ivec3(0);
    glm::ivec3 probe_scroll_directions = glm::ivec3(1);
    glm::ivec3 last_probe_scroll_delta = glm::ivec3(0);
    uint32_t previous_ray_count = 0;
    bool previous_deterministic_ray_seed_enabled = false;
    uint32_t previous_deterministic_ray_seed = 0;
    uint32_t probe_ray_sequence_index = 0;
    float probe_variability_average = 0.0f;
    bool probe_variability_gating_enabled = false;
    uint32_t probe_variability_sample_count = 0;
    uint32_t probe_variability_stable_sample_count = 0;
    bool probe_variability_converged = false;
    uint32_t probe_variability_refresh_age = 0;
    bool probe_variability_refresh_waiting = false;
    uint64_t next_variability_generation = 0;
    uint64_t last_consumed_variability_generation = 0;
    uint64_t probe_variability_refresh_generation = 0;
    uint32_t probe_warmup_frame_index = 0;
    uint32_t frame_probe_warmup_frame_index = 0;
    uint32_t frame_probe_warmup_frame_count = 0;
    bool frame_probe_warmup_active = false;
    float frame_probe_update_hysteresis = 0.0f;
    bool frame_probe_relocation_reset = false;
    bool frame_probe_relocation_enabled = false;
    bool frame_probe_classification_reset = false;
    bool frame_probe_classification_enabled = false;
    bool frame_probe_variability_enabled = false;
    int latched_scene_change_triggers = DdgiVolumeTriggerConditionNone;
    bool latched_scene_geometry_changed = false;
    uint32_t frame_selected_probe_ray_sample_count = 0;
    uint32_t frame_selected_probe_ray_logical_index = 0;
    uint32_t frame_selected_probe_ray_physical_index = 0;
    DdgiFrameResourceLayout frame_resource_layout{};
    DdgiPerformanceStats last_performance_stats{};
    DdgiProbeRayTracingPushConstant frame_ray_push_constant{};
    DdgiProbeScrollPushConstant frame_probe_scroll_push_constant{};
    DdgiProbeAtlasUpdatePushConstant frame_probe_update_push_constant{};
    DdgiProbeRelocationPushConstant frame_probe_relocation_reset_push_constant{};
    DdgiProbeRelocationPushConstant frame_probe_relocation_update_push_constant{};
    DdgiProbeClassificationPushConstant frame_probe_classification_reset_push_constant{};
    DdgiProbeClassificationPushConstant frame_probe_classification_update_push_constant{};
  };

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
  mutable std::vector<std::vector<RenderGraphTransientResourceStore>> render_graph_transient_resource_stores_;
  std::shared_ptr<PostProcessingRendererResources> post_processing_renderer_resources_;
  mutable RenderGraphPlanCache ray_camera_render_graph_plan_cache_{16};
  mutable std::unordered_map<uint64_t, std::weak_ptr<Camera>> ray_camera_history_cameras_;
  mutable RayCameraHistoryStats retired_ray_camera_history_stats_{};
  mutable uint64_t peak_live_ray_camera_history_count_ = 0;
  mutable uint64_t peak_live_ray_camera_history_byte_size_ = 0;
  mutable uint64_t peak_live_ray_camera_output_descriptor_count_ = 0;
  std::unordered_map<uint64_t, std::unique_ptr<DdgiVolumeRuntimeState>> ddgi_volume_runtime_states_{};
  std::vector<uint64_t> ddgi_ordered_volume_ids_{};
  uint64_t next_ddgi_resource_id_ = 0u;
  std::weak_ptr<Scene> ddgi_runtime_scene_{};
  std::string ddgi_volume_set_validation_error_{};
  mutable std::shared_ptr<Buffer> ddgi_fallback_probe_state_buffer_;
  mutable std::shared_ptr<Sampler> ddgi_atlas_sampler_;
  friend class Platform;
  friend class Resources;
  friend class Camera;
  friend class RayCameraHistoryTestAccess;
  friend class GraphicsPipeline;
  friend class EditorLayer;
  friend class Material;
  friend class Lighting;
  friend class PostProcessingStack;
  friend class Application;
  friend class OffscreenPreviewRenderer;
  friend class ReflectionProbe;
  friend class RenderInstanceStorage;
  friend class TextureStorage;
#pragma region DescriptorSet Layouts
  std::shared_ptr<DescriptorSetLayout> empty_descriptor_set_layout_;
  std::shared_ptr<DescriptorSetLayout> per_frame_layout_;
  std::shared_ptr<DescriptorSetLayout> raster_material_per_frame_layout_;
  std::shared_ptr<DescriptorSetLayout> meshlet_layout_;
  std::shared_ptr<DescriptorSetLayout> strand_meshlet_layout_;
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
  std::shared_ptr<DescriptorSetLayout> motion_coverage_layout_;
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
  DdgiSettings fallback_ddgi_settings_{};
  bool ddgi_has_previous_scene_inputs_ = false;
  bool ddgi_referenced_scene_inputs_pending_ = false;
  std::vector<uint64_t> ddgi_previous_material_keys_;
  std::vector<uint64_t> ddgi_previous_material_signatures_;
  std::vector<uint64_t> ddgi_previous_material_texture_signatures_;
  uint64_t ddgi_previous_emissive_inventory_signature_ = 0;
  bool ddgi_deferred_scene_readiness_refresh_ = false;
  std::vector<uint64_t> ddgi_previous_active_light_keys_;
  std::vector<uint64_t> ddgi_previous_light_signatures_;
  std::vector<uint64_t> ddgi_previous_geometry_signatures_;
  std::vector<uint64_t> ddgi_previous_probe_state_geometry_signatures_;
  int ddgi_latched_scene_change_triggers_ = DdgiVolumeTriggerConditionNone;
  bool ddgi_latched_scene_geometry_changed_ = false;
  mutable DdgiPerformanceStats ddgi_last_performance_stats_{};
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
  void PreparePointAndSpotLightShadowMap(bool immediate = false, bool include_external = true) const;

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
                      const std::shared_ptr<Camera>& camera, bool immediate = false,
                      bool reflection_probe_capture = false) const;

  /**
   * \brief Renders to the specified camera using ray tracing.
   * \param scene The scene whose environment settings apply to the camera render.
   * \param camera_global_transform The global transform of the camera.
   * \param camera The target camera to render to.
   */
  void RenderToCameraRayTracing(const std::shared_ptr<Scene>& scene, const GlobalTransform& camera_global_transform,
                                const std::shared_ptr<Camera>& camera) const;
  void PruneRayCameraHistories(const std::shared_ptr<RenderInstanceStorage>& render_instances) const;
  void ForgetRayCameraHistoryCamera(uint64_t camera_handle, const Camera* camera) const;
  void ArchiveRayCameraHistory(const std::shared_ptr<Camera>& camera) const;
  void UpdateRayCameraHistoryPeaks() const;
  void ClearRayCameraHistories() const;

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
                                bool track_ddgi_scene_inputs = true,
                                const std::pair<GlobalTransform, std::shared_ptr<Camera>>* injected_camera = nullptr,
                                bool include_reflection_probes = true);

  void PrepareDdgiFrameState(const std::shared_ptr<Scene>& scene,
                             const std::shared_ptr<RenderInstanceStorage>& render_instances);
  void PrepareDdgiVolumeFrameState(const std::shared_ptr<Scene>& scene,
                                   const std::shared_ptr<RenderInstanceStorage>& render_instances,
                                   DdgiVolumeRuntimeState& runtime_state,
                                   const ResolvedEnvironmentalLighting::DdgiVolume& volume,
                                   const DdgiSettings& ddgi_settings, const DdgiFrameResourceLayout& preflight_layout,
                                   uint32_t sorted_index, bool reset_probe_history);
  static void ResetDdgiRuntimeFrameState(DdgiVolumeRuntimeState& runtime_state);
  [[nodiscard]] uint64_t NextDdgiResourceId();
  [[nodiscard]] const DdgiVolumeRuntimeState* GetPrimaryDdgiVolumeRuntimeState() const;
  void RenderSceneToCameraImmediately(const std::shared_ptr<Scene>& scene,
                                      const GlobalTransform& camera_global_transform,
                                      const std::shared_ptr<Camera>& camera, bool reflection_probe_capture = false);
  bool BakeReflectionProbe(const std::shared_ptr<Scene>& scene, const glm::vec3& position,
                           const std::shared_ptr<GlobalReflectionProbe>& target, uint64_t& source_fingerprint,
                           std::string& error, bool& retry);
  void QueueGlobalReflectionProbeBakeAttempt(const std::shared_ptr<Scene>& scene, const glm::vec3& position,
                                             const std::shared_ptr<GlobalReflectionProbe>& target,
                                             uint32_t retry_count);
  [[nodiscard]] uint64_t GetReflectionProbeCaptureFingerprint(const std::shared_ptr<Scene>& scene,
                                                              const glm::vec3& position) const;

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
                                   bool track_ddgi_scene_inputs = true,
                                   const std::pair<GlobalTransform, std::shared_ptr<Camera>>* injected_camera = nullptr,
                                   bool include_reflection_probes = true);
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
  std::vector<std::shared_ptr<DescriptorSet>> strand_meshlet_descriptor_sets_ = {};
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

  /// Mesh-shader pipeline for rendering directional light shadows from strands.
  std::shared_ptr<GraphicsPipeline> strands_directional_light_shadow_pipeline;

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

  std::shared_ptr<GraphicsPipeline> ddgi_gather_timing_pipeline_;

  /// Graphics pipeline for rendering transparent normal meshes after deferred lighting.
  std::shared_ptr<GraphicsPipeline> transparent_geometry_pipeline_normal;

  std::shared_ptr<GraphicsPipeline> skinned_motion_vectors_pipeline_;
  std::shared_ptr<GraphicsPipeline> transparent_motion_vectors_pipeline_;

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
  std::shared_ptr<ComputePipeline> ddgi_probe_update_irradiance_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_update_visibility_pipeline_;
  DdgiProbeUpdateVariant ddgi_probe_update_variant_ = DdgiProbeUpdateVariant::Serial;
  bool ddgi_probe_update_path_reported_ = false;
  std::shared_ptr<ComputePipeline> ddgi_probe_scroll_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_relocation_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_classification_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_variability_reduce_pipeline_;
  std::shared_ptr<ComputePipeline> ddgi_probe_variability_extra_reduce_pipeline_;
  std::shared_ptr<ComputePipeline> ray_query_camera_pipeline_;
  std::shared_ptr<ComputePipeline> ray_query_camera_fallback_pipeline_;

#pragma region Ray Tracing Pipelines
  /// Ray tracing pipeline for rendering cameras with ray tracing.
  std::shared_ptr<RayTracingPipeline> ray_tracing_camera_pipeline;
  std::shared_ptr<RayTracingPipeline> ray_tracing_camera_fallback_pipeline_;
  std::shared_ptr<RayCameraShaderVariantCache> ray_camera_shader_variant_cache_;
  /// Ray tracing pipeline for rendering cameras with ray tracing.
  friend class PointCloud;
  std::shared_ptr<RayTracingPipeline> ray_tracing_point_cloud_pipeline;
  std::shared_ptr<RayTracingPipeline> ddgi_probe_ray_diagnostic_pipeline_;
#pragma endregion
};
}  // namespace evo_engine
