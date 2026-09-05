
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
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace evo_engine {
struct ApplicationInitializationSettings;
class EVOENGINE_API ComputePipeline;
class EVOENGINE_API GlobalReflectionProbe;
class EVOENGINE_API ReflectionProbePack;
class EVOENGINE_API OffscreenPreviewRenderer;
class ReflectionProbe;
class EVOENGINE_API Sampler;
struct FrameSubmissionState;
struct PostProcessingRendererResources;

/**
 * \class RenderLayer
 * \brief Represents a render layer in the rendering engine.
 *
 * Handles various rendering tasks, such as camera iteration, shadow map rendering,
 * deferred rendering, forward rendering, and more.
 */
class EVOENGINE_API RenderLayer final : public ILayer {
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

  /**
   * @brief Returns true after the active editor camera has rendered this scene with all configured lighting ready.
   */
  [[nodiscard]] bool HasPresentedScene(const std::shared_ptr<Scene>& scene) const;

  [[nodiscard]] bool RequiresCameraWideTemporalHistoryRejection() const;
  /**
   * \brief Notifies the renderer that a static entity or its descendants were modified directly.
   * \param scene Scene containing the entity.
   * \param entity Root of the modified static subtree.
   */
  void NotifyStaticEntityChanged(const std::shared_ptr<Scene>& scene, const Entity& entity);

  struct DdgiSessionState {
    bool pause_updates = false;
    bool reset_history_requested = false;
    bool show_probes = false;
    bool show_selected_probe = true;
    bool show_selected_probe_state = false;
    bool selected_probe_readback_requested = false;
    bool show_rays = false;
    uint64_t selected_volume_id = 0;
    glm::ivec3 selected_probe_grid = glm::ivec3(0);
    int probe_visualization_mode = 0;
    int probe_visualization_depth_mode = 0;
    float probe_visualization_radius_fraction = 0.08f;
    float probe_visualization_intensity = 1.0f;
    float probe_visualization_alpha = 0.95f;
    float ray_visualization_alpha = 0.85f;
    float selected_probe_visualization_scale = 2.5f;
  };

  struct DdgiInspectorSnapshot {
    bool enabled = false;
    uint32_t last_probe_update_reasons = DdgiUpdateReasonNone;
    bool last_probe_history_cleared = false;
    std::string validation_error{};
    DdgiPerformanceStats aggregate{};
    std::vector<DdgiVolumeRuntimeStats> volumes{};
  };

  /// Specifies whether wireframe rendering is enabled.
  bool wire_frame = false;

  /// Specifies whether shadow-rendering draw calls should be counted.
  bool count_shadow_rendering_draw_calls = true;

  /// Specifies whether meshlet rendering is enabled.
  bool enable_meshlet = true;

  /// Specifies whether indirect rendering is enabled.
  bool enable_indirect_rendering = true;

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

  [[nodiscard]] DdgiSessionState& GetDdgiSessionState();
  [[nodiscard]] const DdgiSessionState& GetDdgiSessionState() const;
  void RequestDdgiHistoryReset();
  [[nodiscard]] DdgiInspectorSnapshot GetDdgiInspectorSnapshot() const;
  [[nodiscard]] DdgiProbeDebugDataView RefreshDdgiProbeDebugData();

  /**
   * \brief Draws strands without a scene entity.
   * \param strands The strands to draw.
   * \param material The material to use for rendering the strands.
   * \param global_transform The global transform of the strands.
   * \param cast_shadow Specifies whether the strands cast a shadow.
   * \return One when the draw was registered, otherwise zero.
   */
  [[maybe_unused]] uint32_t DrawStrands(const std::shared_ptr<Strands>& strands,
                                        const std::shared_ptr<Material>& material,
                                        const GlobalTransform& global_transform, bool cast_shadow = false) const;

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
  void RenderOpaqueToPointLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>&&
          func);
  void RenderAlphaMaskedToPointLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>&&
          func);

  /**
   * \brief Register per-frame function to render to all spot light shadow maps.
   * \param func Render function targeting point light shadow map. Return primitive count.
   */
  void RenderOpaqueToSpotLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>&& func);
  void RenderAlphaMaskedToSpotLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>&& func);

  /**
   * \brief Register per-frame function to render to all directional light shadow maps.
   * \param func Render function targeting point light shadow map. Return primitive count.
   */
  void RenderOpaqueToDirectionalLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>&&
          func);
  void RenderAlphaMaskedToDirectionalLightShadowMap(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>&&
          func);

  /**
   * \brief Register per-frame raw opaque geometry rendering for all deferred cameras.
   *
   * The callback must write the raw G-buffer ABI without evaluating materials or sampling textures.
   * \param func Raw opaque geometry callback. Return primitive count.
   */
  void RawOpaqueRenderingAllCameras(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                             const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                             const DeferredRenderingView& deferred_rendering_view)>&& func);

  /**
   * \brief Register per-frame alpha-masked raw geometry rendering for all deferred cameras.
   *
   * The callback may evaluate only material alpha and must otherwise write the raw G-buffer ABI.
   * \param func Alpha-masked raw geometry callback. Return primitive count.
   */
  void AlphaMaskedRenderingAllCameras(
      std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                             const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                             const DeferredRenderingView& deferred_rendering_view)>&& func);

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
  [[nodiscard]] bool SharedTextureDescriptorArraysEnabled() const;
  [[nodiscard]] const std::vector<uint64_t>& GetPerFrameTexture2DAppliedRevisions() const;
  [[nodiscard]] const std::vector<uint64_t>& GetPerFrameCubemapAppliedRevisions() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSetLayout>& GetRasterLightingTextureDescriptorSetLayout() const;
  [[nodiscard]] std::shared_ptr<DescriptorSet> GetExistingRasterLightingTextureDescriptorSet(
      uint32_t current_frame_index, int camera_index) const;
  [[nodiscard]] const std::shared_ptr<PostProcessingRendererResources>& GetPostProcessingRendererResources() const;
  struct ReflectionProbeBakeRequest {
    glm::vec3 position{};
    std::shared_ptr<GlobalReflectionProbe> target{};
    std::shared_ptr<ReflectionProbePack> owner_pack{};
    uint64_t stable_id = 0u;
  };
  [[nodiscard]] bool QueueGlobalReflectionProbeBake(const std::shared_ptr<Scene>& scene, const glm::vec3& position,
                                                    const std::shared_ptr<GlobalReflectionProbe>& target);
  [[nodiscard]] uint32_t QueueGlobalReflectionProbeBakeBatch(const std::shared_ptr<Scene>& scene,
                                                             const std::vector<ReflectionProbeBakeRequest>& requests);
  [[nodiscard]] bool IsGlobalReflectionProbeBakePending(const std::shared_ptr<GlobalReflectionProbe>& target) const;
  [[nodiscard]] bool HasPendingGlobalReflectionProbeBake() const;
  struct DynamicReflectionProbeStats {
    bool active = false;
    uint32_t queued_probe_count = 0;
    uint32_t in_progress_probe_count = 0;
    uint32_t filtering_probe_count = 0;
    uint32_t generation_a_probe_count = 0;
    uint32_t generation_b_probe_count = 0;
    uint32_t transitioning_probe_count = 0;
    uint64_t current_probe_stable_id = 0;
    uint32_t completed_face_count = 0;
    uint64_t current_filter_probe_stable_id = 0;
    uint32_t completed_filter_face_count = 0;
    uint64_t published_generation_count = 0;
    uint64_t transient_gpu_bytes = 0;
    float minimum_transition_weight = 0.0f;
    float maximum_transition_weight = 0.0f;
    double last_update_gpu_ms = 0.0;
    double last_capture_gpu_ms = 0.0;
    double last_prefilter_gpu_ms = 0.0;
  };
  [[nodiscard]] DynamicReflectionProbeStats GetDynamicReflectionProbeStats() const;
  void ResetDynamicReflectionProbeHistory();

 private:
  using RenderCommandRecorder =
      std::function<void(const std::function<void(VkCommandBuffer vk_command_buffer)>& action)>;

  struct ReflectionProbeBakeBatch {
    std::shared_ptr<Scene> scene{};
    std::vector<ReflectionProbeBakeRequest> requests{};
    uint32_t retry_count = 0;
  };

  struct PreparedReflectionProbeBake {
    std::shared_ptr<Scene> scene{};
    std::vector<ReflectionProbeBakeRequest> requests{};
    std::vector<std::shared_ptr<Cubemap>> output_cubemaps{};
    std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> injected_cameras{};
    uint32_t retry_count = 0;
  };

  struct SubmittedReflectionProbeBake {
    std::vector<ReflectionProbeBakeRequest> requests{};
    std::vector<std::shared_ptr<Cubemap>> output_cubemaps{};
  };

  struct ReflectionProbeCaptureGraphContext {
    std::shared_ptr<Camera> camera{};
    std::shared_ptr<RenderInstanceStorage> render_instances{};
    std::shared_ptr<DescriptorSet> lighting_descriptor_set{};
    std::shared_ptr<DescriptorSet> raster_lighting_texture_descriptor_set{};
    RenderCommandRecorder record_commands{};
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    int camera_index = -1;
    int directional_shadow_camera_index = -1;
    uint32_t current_frame_index = 0;
    bool use_mesh_shader = false;
  };

  struct DynamicReflectionProbeRuntimeState {
    glm::vec3 position{};
    int artist_priority = 0;
    std::array<std::shared_ptr<Cubemap>, 2> filtered_generations{};
    uint32_t initial_source_texture_index = 0u;
    bool initial_source_valid = false;
    int published_generation = -1;
    uint32_t next_face = 0;
    uint32_t filtered_face_count = 0;
    bool filtering = false;
    bool completion_in_flight = false;
    float transition_weight = 0.0f;
    float transition_start_weight = 0.0f;
    uint64_t transition_start_face_serial = 0u;
    uint64_t transition_end_face_serial = 0u;
    uint64_t published_generation_count = 0;
    uint64_t capture_revision = 0u;
  };

  struct DynamicReflectionProbeCaptureJob {
    uint64_t stable_id = 0;
    uint32_t first_face = 0;
    uint32_t face_count = 0;
    uint32_t first_camera = 0;
    int output_generation = 0;
    uint64_t end_face_serial = 0u;
    uint64_t capture_revision = 0u;
    uint32_t raw_slot = 0u;
  };

  struct DynamicReflectionProbeFilterJob {
    uint32_t raw_slot = 0u;
    uint32_t first_face = 0u;
    uint32_t face_count = 0u;
  };

  struct DynamicReflectionProbeRawSlot {
    std::shared_ptr<Cubemap> cubemap{};
    std::shared_ptr<DescriptorSet> descriptor_set{};
    uint64_t stable_id = 0u;
    uint64_t capture_revision = 0u;
    int output_generation = 0;
    bool capturing = false;
    bool filtering = false;
  };

  struct PreparedDynamicReflectionProbeUpdate {
    uint64_t epoch = 0;
    std::vector<DynamicReflectionProbeCaptureJob> jobs{};
    std::vector<DynamicReflectionProbeFilterJob> filter_jobs{};
    std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> injected_cameras{};
  };

  struct SubmittedDynamicReflectionProbeUpdate {
    struct Completion {
      uint64_t stable_id = 0;
      int generation = 0;
      uint64_t capture_revision = 0u;
      std::shared_ptr<Cubemap> output{};
    };
    uint64_t epoch = 0;
    std::vector<Completion> completions{};
  };

  struct RetiredDynamicReflectionProbeResources {
    std::vector<std::shared_ptr<Cubemap>> cubemaps{};
    std::shared_ptr<FrameSubmissionState> submission{};
  };

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
    uint64_t variability_budget_cycle = 0;
    bool counts_toward_variability_budget = false;
  };

  struct DdgiVolumeRuntimeState {
    std::string name{};
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
    bool previous_pause_probe_updates_after_convergence = true;
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
    uint32_t previous_emissive_ray_count = 0;
    bool previous_deterministic_ray_seed_enabled = false;
    uint32_t previous_deterministic_ray_seed = 0;
    uint32_t probe_ray_sequence_index = 0;
    float probe_variability_average = 0.0f;
    float probe_variability_maximum = 0.0f;
    float probe_variability_unstable_fraction = 0.0f;
    bool probe_variability_gating_enabled = false;
    uint32_t probe_variability_sample_count = 0;
    uint32_t probe_variability_stable_sample_count = 0;
    bool probe_variability_converged = false;
    bool probe_variability_maximum_reached = false;
    DdgiProbeVariabilityBudgetState probe_variability_budget{};
    uint64_t next_variability_generation = 0;
    uint64_t last_consumed_variability_generation = 0;
    uint32_t probe_warmup_frame_index = 0;
    uint32_t frame_probe_warmup_frame_index = 0;
    uint32_t frame_probe_warmup_frame_count = 0;
    bool frame_probe_warmup_active = false;
    float frame_probe_update_hysteresis = 0.0f;
    float current_probe_hysteresis = 0.97f;
    bool hysteresis_boost_active = false;
    bool frame_hysteresis_boost_active = false;
    bool frame_hysteresis_boost_restoring = false;
    bool frame_probe_relocation_reset = false;
    bool frame_probe_relocation_enabled = false;
    bool frame_probe_classification_reset = false;
    bool frame_probe_classification_enabled = false;
    bool frame_probe_variability_enabled = false;
    bool frame_probe_variability_counts_toward_budget = false;
    float frame_probe_variability_threshold = 0.0f;
    int latched_scene_change_triggers = DdgiVolumeTriggerConditionNone;
    uint32_t frame_selected_probe_ray_sample_count = 0;
    uint32_t frame_selected_probe_ray_logical_index = 0;
    uint32_t frame_selected_probe_ray_physical_index = 0;
    uint32_t frame_uniform_ray_count = 0;
    uint32_t frame_emissive_ray_count = 0;
    uint32_t frame_fixed_ray_count = 0;
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
      opaque_point_light_shadow_map_external_functions;
  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>>
      alpha_masked_point_light_shadow_map_external_functions;

  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>>
      opaque_spot_light_shadow_map_external_functions;
  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>>
      alpha_masked_spot_light_shadow_map_external_functions;

  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>>
      opaque_directional_light_shadow_map_external_functions;
  std::vector<
      std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>>
      alpha_masked_directional_light_shadow_map_external_functions;

  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                                     const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                                     const DeferredRenderingView& forward_rendering_view)>>
      raw_opaque_rendering_external_functions;
  std::vector<std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                                     const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                                     const DeferredRenderingView& deferred_rendering_view)>>
      alpha_masked_rendering_external_functions;

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
  friend class StaticSceneRenderTestAccess;
  friend class SdfgiTestAccess;
#pragma region DescriptorSet Layouts
  std::shared_ptr<DescriptorSetLayout> empty_descriptor_set_layout_;
  std::shared_ptr<DescriptorSetLayout> per_frame_layout_;
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
  std::shared_ptr<DescriptorSetLayout> deferred_compute_lighting_layout_;
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
  bool per_frame_bindless_texture_descriptors_enabled_ = false;
  mutable std::shared_ptr<Texture2D> raster_lighting_white_fallback_texture_;

  void InitializeCommonDescriptorSetLayouts(
      const ApplicationInitializationSettings& application_initialization_settings);
  void EnsureRasterLightingFallbackTexture() const;
  void EnsureDdgiPipelines();
#pragma endregion

  std::vector<std::shared_ptr<RenderInstanceStorage>> render_instances_list_;
  std::weak_ptr<Scene> presented_scene_;
  std::weak_ptr<Scene> sdfgi_scene_;
  std::vector<std::vector<std::shared_ptr<class SdfgiResources>>> sdfgi_frame_resources_;
  [[nodiscard]] bool IsSceneLightingReadyForPresentation(
      const std::shared_ptr<Scene>& scene, const std::shared_ptr<RenderInstanceStorage>& render_instances) const;
  std::weak_ptr<Scene> pending_static_entity_change_scene_;
  std::vector<Entity> pending_static_entity_changes_;
  std::weak_ptr<Scene> selection_highlight_coverage_scene_;
  uint64_t selection_highlight_coverage_selection_revision_ = 0;
  uint64_t selection_highlight_coverage_hierarchy_revision_ = 0;
  std::shared_ptr<const EntitySelectionHighlightCoverage> selection_highlight_coverage_;
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
  int ddgi_latched_scene_change_triggers_ = DdgiVolumeTriggerConditionNone;
  mutable DdgiPerformanceStats ddgi_last_performance_stats_{};
  mutable DdgiSessionState ddgi_session_state_{};
  std::unique_ptr<Lighting> lighting_;
  std::shared_ptr<Texture2D> environmental_brdf_lut_ = {};
  std::vector<std::shared_ptr<Camera>> reflection_probe_capture_cameras_ = {};
  Handle reflection_probe_shadow_camera_handle_{};
  std::shared_ptr<Cubemap> reflection_probe_capture_raw_cubemap_ = {};
  std::array<DynamicReflectionProbeRawSlot, 2> dynamic_reflection_probe_raw_slots_{};
  std::deque<uint32_t> dynamic_reflection_probe_filter_queue_{};
  std::shared_ptr<Cubemap> reflection_probe_capture_filtered_cubemap_ = {};
  std::vector<std::vector<std::shared_ptr<ImageView>>> reflection_probe_capture_filtered_mip_views_ = {};
  std::shared_ptr<Image> reflection_probe_capture_filter_depth_image_ = {};
  std::shared_ptr<ImageView> reflection_probe_capture_filter_depth_view_ = {};
  std::shared_ptr<DescriptorSet> reflection_probe_capture_filter_descriptor_set_ = {};
  std::shared_ptr<GraphicsPipeline> reflection_probe_capture_prefilter_pipeline_ = {};
  RenderGraph reflection_probe_capture_render_graph_{};
  RenderGraphExecutionPlan reflection_probe_capture_render_graph_plan_{};
  ReflectionProbeCaptureGraphContext* reflection_probe_capture_graph_context_ = nullptr;
  std::deque<ReflectionProbeBakeBatch> reflection_probe_bake_queue_{};
  std::optional<PreparedReflectionProbeBake> prepared_reflection_probe_bake_{};
  std::vector<std::optional<SubmittedReflectionProbeBake>> submitted_reflection_probe_bakes_{};
  std::unordered_set<uint64_t> pending_reflection_probe_bake_targets_{};
  std::unordered_map<uint64_t, DynamicReflectionProbeRuntimeState> dynamic_reflection_probe_runtime_states_{};
  std::deque<uint64_t> dynamic_reflection_probe_queue_{};
  std::optional<PreparedDynamicReflectionProbeUpdate> prepared_dynamic_reflection_probe_update_{};
  std::vector<std::optional<SubmittedDynamicReflectionProbeUpdate>> submitted_dynamic_reflection_probe_updates_{};
  std::vector<RetiredDynamicReflectionProbeResources> retired_dynamic_reflection_probe_resources_{};
  std::unordered_map<uint64_t, RenderInstanceStorage::ReflectionProbeTextureOverride>
      dynamic_reflection_probe_texture_overrides_{};
  std::weak_ptr<Scene> dynamic_reflection_probe_scene_{};
  Handle dynamic_reflection_probe_lighting_handle_{};
  Handle dynamic_reflection_probe_pack_handle_{};
  uint32_t dynamic_reflection_probe_pack_version_ = 0u;
  uint64_t dynamic_reflection_probe_epoch_ = 1;
  uint64_t dynamic_reflection_probe_scheduled_face_serial_ = 0u;
  bool dynamic_reflection_probe_contributing_ = false;
  bool dynamic_reflection_probe_reset_requested_ = false;
  /**
   * \brief Called after the RenderLayer object is created.
   */
  void OnCreate() override;

 private:
  /**
   * \brief Prepares shadow maps for point and spot lights.
   */
  void PreparePointAndSpotLightShadowMap(bool immediate = false, bool include_external = true,
                                         const RenderCommandRecorder* command_recorder = nullptr) const;

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
                      bool reflection_probe_capture = false, int camera_index_override = -1,
                      int directional_shadow_camera_index = -1,
                      const RenderCommandRecorder* command_recorder = nullptr) const;

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
  void ConsumeStaticEntityChanges(const std::shared_ptr<Scene>& scene);
  void PrepareSceneForRendering(
      const std::shared_ptr<Scene>& scene, bool include_editor_cameras = true, bool update_editor_selection = true,
      bool update_ray_tracing = true, bool track_ddgi_scene_inputs = true,
      const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>* injected_cameras = nullptr,
      bool include_reflection_probes = true, bool immediate_upload = false);

  void PrepareDdgiFrameState(const std::shared_ptr<Scene>& scene,
                             const std::shared_ptr<RenderInstanceStorage>& render_instances);
  void PrepareDdgiVolumeFrameState(const std::shared_ptr<Scene>& scene,
                                   const std::shared_ptr<RenderInstanceStorage>& render_instances,
                                   DdgiVolumeRuntimeState& runtime_state,
                                   const ResolvedEnvironmentalLighting::DdgiVolume& volume,
                                   const DdgiSettings& ddgi_settings, const DdgiFrameResourceLayout& preflight_layout,
                                   uint32_t sorted_index, bool reset_probe_history);
  static void ResetDdgiRuntimeFrameState(DdgiVolumeRuntimeState& runtime_state);
  [[nodiscard]] std::vector<DdgiVolumeRuntimeStats> BuildDdgiVolumeRuntimeStats() const;
  [[nodiscard]] uint64_t NextDdgiResourceId();
  [[nodiscard]] const DdgiVolumeRuntimeState* GetPrimaryDdgiVolumeRuntimeState() const;
  void RenderSceneToCameraImmediately(const std::shared_ptr<Scene>& scene,
                                      const GlobalTransform& camera_global_transform,
                                      const std::shared_ptr<Camera>& camera, bool reflection_probe_capture = false);
  [[nodiscard]] const std::vector<std::shared_ptr<Camera>>& GetOrCreateReflectionProbeCaptureCameras(size_t count);
  bool PrepareReflectionProbeCaptureResources(VkFormat raw_format);
  bool PrepareDynamicReflectionProbeCaptureResources(VkFormat raw_format);
  void PrepareReflectionProbeBake(const std::shared_ptr<Scene>& scene);
  void RecordPreparedReflectionProbeBake(const std::shared_ptr<RenderInstanceStorage>& render_instances);
  void PublishSubmittedReflectionProbeBake(uint32_t frame_index);
  void PrepareDynamicReflectionProbeUpdate(const std::shared_ptr<Scene>& scene);
  void RecordPreparedDynamicReflectionProbeUpdate(const std::shared_ptr<RenderInstanceStorage>& render_instances);
  void PublishSubmittedDynamicReflectionProbeUpdate(uint32_t frame_index);
  void RetireDynamicReflectionProbeRuntime();
  void RetireDynamicReflectionProbeResources(std::vector<std::shared_ptr<Cubemap>> resources);
  void CollectRetiredDynamicReflectionProbeResources();
  void RebuildDynamicReflectionProbeQueue();
  void SortDynamicReflectionProbeQueue();
  void UpdateDynamicReflectionProbeTransitions(uint64_t face_serial);
  void FailReflectionProbeBakeBatch(const ReflectionProbeBakeBatch& batch, const std::string& error, bool timed_out);
  void EnsureReflectionProbeCaptureRenderGraph();

  /**
   * \brief Performs all rendering operations for this render layer.
   */
  void RenderAll();
  void ExecuteSceneFramePasses(const std::shared_ptr<Scene>& scene);

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
  bool UpdateRenderInstanceStorage(
      const std::shared_ptr<Scene>& scene, uint32_t current_frame_index, bool include_editor_cameras = true,
      bool update_editor_selection = true, bool track_ddgi_scene_inputs = true,
      const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>* injected_cameras = nullptr,
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
  mutable std::vector<uint64_t> per_frame_texture_2d_applied_revisions_ = {};
  mutable std::vector<uint64_t> per_frame_cubemap_applied_revisions_ = {};
  mutable std::vector<std::vector<std::shared_ptr<DescriptorSet>>> raster_lighting_texture_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> meshlet_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> strand_meshlet_descriptor_sets_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> ray_tracing_descriptor_sets_ = {};
  std::vector<std::shared_ptr<Buffer>> kernel_descriptor_buffers_ = {};

#pragma region Graphics Pipelines
  /// Depth-only pipeline for rendering point light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_normal_opaque;
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_normal_masked;

  /// Graphics pipeline for rendering point light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_mesh_shader_opaque;
  std::shared_ptr<GraphicsPipeline> point_light_shadow_pipeline_mesh_shader_masked;

  /// Depth-only pipeline for rendering spot light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_normal_opaque;
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_normal_masked;

  /// Graphics pipeline for rendering spot light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_mesh_shader_opaque;
  std::shared_ptr<GraphicsPipeline> spot_light_shadow_pipeline_mesh_shader_masked;

  /// Depth-only pipeline for rendering directional light shadows with normal meshes.
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_normal_opaque;
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_normal_masked;

  /// Graphics pipeline for rendering directional light shadows with mesh shaders.
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_mesh_shader_opaque;
  std::shared_ptr<GraphicsPipeline> directional_light_shadow_pipeline_mesh_shader_masked;

  /// Mesh-shader pipeline for rendering directional light shadows from strands.
  std::shared_ptr<GraphicsPipeline> strands_directional_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> strands_directional_light_shadow_pipeline_masked;

  /// Depth-only pipeline for rendering instanced point light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_point_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> instanced_point_light_shadow_pipeline_masked;

  /// Depth-only pipeline for rendering instanced spot light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_spot_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> instanced_spot_light_shadow_pipeline_masked;

  /// Depth-only pipeline for rendering instanced directional light shadows.
  std::shared_ptr<GraphicsPipeline> instanced_directional_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> instanced_directional_light_shadow_pipeline_masked;

  /// Depth-only pipeline for rendering point light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_point_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> skinned_point_light_shadow_pipeline_masked;

  /// Depth-only pipeline for rendering spot light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_spot_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> skinned_spot_light_shadow_pipeline_masked;

  /// Depth-only pipeline for rendering directional light shadows with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_directional_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> skinned_directional_light_shadow_pipeline_masked;

  /// Graphics pipeline for rendering point light shadows with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_point_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> strands_point_light_shadow_pipeline_masked;

  /// Graphics pipeline for rendering spot light shadows with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_spot_light_shadow_pipeline_opaque;
  std::shared_ptr<GraphicsPipeline> strands_spot_light_shadow_pipeline_masked;

  /// Graphics pipeline for the deferred shading GBuffer pre-pass using normal meshes.
  std::shared_ptr<GraphicsPipeline> deferred_geometry_pipeline_normal;
  std::shared_ptr<GraphicsPipeline> deferred_masked_geometry_pipeline_normal;

  /// Graphics pipeline for the deferred shading GBuffer pre-pass using mesh shaders.
  std::shared_ptr<GraphicsPipeline> deferred_geometry_pipeline_mesh;
  std::shared_ptr<GraphicsPipeline> deferred_masked_geometry_pipeline_mesh;

  /// Graphics pipeline for rendering instanced deferred shading GBuffer pre-pass.
  std::shared_ptr<GraphicsPipeline> instanced_deferred_geometry_pipeline;
  std::shared_ptr<GraphicsPipeline> instanced_deferred_masked_geometry_pipeline;

  /// Graphics pipeline for rendering deferred shading GBuffer pre-pass with skinned meshes.
  std::shared_ptr<GraphicsPipeline> skinned_deferred_geometry_pipeline;
  std::shared_ptr<GraphicsPipeline> skinned_deferred_masked_geometry_pipeline;

  /// Graphics pipeline for rendering deferred shading GBuffer pre-pass with hair strands.
  std::shared_ptr<GraphicsPipeline> strands_deferred_geometry_pipeline;
  std::shared_ptr<GraphicsPipeline> strands_deferred_masked_geometry_pipeline;

  std::shared_ptr<GraphicsPipeline> entity_selection_highlight_pipeline_;

  /// Graphics pipeline for rendering transparent normal meshes after deferred lighting.
  std::shared_ptr<GraphicsPipeline> transparent_geometry_pipeline_normal;

  std::shared_ptr<GraphicsPipeline> skinned_motion_vectors_opaque_pipeline_;
  std::shared_ptr<GraphicsPipeline> skinned_motion_vectors_masked_pipeline_;
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
  std::shared_ptr<ComputePipeline> deferred_compute_lighting_pipeline_;
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
  std::shared_ptr<RayTracingPipeline> ddgi_probe_trace_pipeline_;
#pragma endregion
};
}  // namespace evo_engine
