
#pragma once
#include "Camera.hpp"
#include "Entity.hpp"
#include "GaussianSplatRenderer.hpp"
#include "GltfMaterialCache.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"

#include <array>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace evo_engine {

using EntitySelectionHighlightCoverage = std::unordered_set<Entity, Entity>;

class EVOENGINE_API BottomLevelAccelerationStructure;
class EVOENGINE_API DescriptorSet;
class EVOENGINE_API DescriptorSetLayout;
class EVOENGINE_API DeferredGeometryPass;
class EVOENGINE_API DirectionalLightShadowPass;
class EVOENGINE_API GaussianSplatCullPass;
class EVOENGINE_API GaussianSplatSortPass;
class EVOENGINE_API GaussianSplatPass;
class EVOENGINE_API MotionCoveragePass;
class EVOENGINE_API Scene;
class EVOENGINE_API TransparentGeometryPass;

/**
 * @brief Struct containing various render settings for the engine.
 */
struct EVOENGINE_API RenderSettings {
  enum class ShadowCascadeFitMode {
    StableSphere,
    TightLightSpaceAabb,
  };

  enum class IndirectLightingDebugView : int {
    Beauty = 0,
    DiffuseIndirect = 1,
    UnoccludedProbeSpecular = 2,
    SpecularVisibility = 3,
    OccludedProbeSpecular = 4,
    DdgiProbeBlendLoss = 5,
  };

  float max_shadow_distance = 400;           ///< Maximum shadow distance in the scene.
  float shadow_cascade_split_lambda = 0.9f;  ///< Blend factor for practical log/uniform cascade splits.
  ShadowCascadeFitMode shadow_cascade_fit_mode = ShadowCascadeFitMode::StableSphere;
  bool enable_debug_visualization = false;  ///< Whether debug visualization is enabled.
  int shadow_debug_mode = 0;                ///< CSM debug visualization mode.
  int shadow_debug_selected_cascade = 0;    ///< Selected cascade for CSM diagnostics.
  int shadow_debug_selected_light = 0;      ///< Selected directional light for CSM diagnostics.
  IndirectLightingDebugView indirect_lighting_debug_view = IndirectLightingDebugView::Beauty;

  float shadow_cascade_transition_width = 5.0f;  ///< Cascade blend width in positive linear view-depth units.
  float shadow_distance_fade = 20.0f;            ///< Final max-shadow-distance fade width in view-depth units.

  float ddgi_hysteresis = 0.97f;                 ///< History weight used by normal DDGI probe updates.
  float ddgi_boosted_hysteresis = 0.85f;         ///< History weight used while a DDGI hysteresis boost is active.
  float ddgi_hysteresis_restore_speed = 0.001f;  ///< Hysteresis restored toward normal per unpaused frame.
  bool ddgi_enable_probe_variability = true;
  bool ddgi_enable_probe_variability_gating = true;
  bool ddgi_pause_probe_updates_after_convergence = true;
  float ddgi_random_ray_backface_threshold = 0.1f;
  float ddgi_fixed_ray_backface_threshold = 0.25f;
  float ddgi_probe_variability_threshold = 0.03f;
  int ddgi_probe_variability_maximum_frames = 128;

  float strands_subdivision_x_factor = 50.0f;  ///< Subdivision factor for strands (in the X-axis).
  float strands_subdivision_y_factor = 50.0f;  ///< Subdivision factor for strands (in the Y-axis).
  int strands_subdivision_max_x = 15;          ///< Maximum subdivision in X-axis for strands.
  int strands_subdivision_max_y = 8;           ///< Maximum subdivision in Y-axis for strands.

  [[nodiscard]] float GetShadowCascadeSplit(int split, float near_distance = 0.1f) const;
  [[nodiscard]] float GetShadowCascadeSplitDistance(int split, float near_distance = 0.1f) const;
  [[nodiscard]] glm::vec4 GetShadowCascadeSplitDistances(float near_distance = 0.1f) const;
  [[nodiscard]] float GetShadowCascadeTransitionHalfWidth(int boundary, float near_distance = 0.1f) const;
  [[nodiscard]] static const char* GetShadowCascadeFitModeName(ShadowCascadeFitMode mode);
};

/**
 * @brief Struct containing push constants for render instances.
 */
struct RenderInstancePushConstant {
  static constexpr uint32_t kRasterDrawInstanceMappingBit = 1u << 31;

  int instance_index = 0;     ///< Canonical instance index or mapped-draw offset.
  int camera_index = 0;       ///< Index of the camera.
  int light_split_index = 0;  ///< Index of the light split for rendering.
  uint32_t meshlet_culling_flags = 0;
};

/**
 * @brief Struct containing push constants for ray tracing.
 */
struct RayTracingCameraPushConstant {
  uint32_t camera_index = 0;   ///< Index of the camera for ray tracing.
  uint32_t frame_id = 0;       ///< Frame ID for the current ray tracing operation.
  uint32_t total_samples = 0;  ///< Samples already accumulated before this dispatch.
  uint32_t frame_samples = 1;  ///< Samples accumulated by this dispatch.
  uint32_t shader_execution_reordering = 0;
  uint32_t max_directional_light_size = 0;
};

struct DdgiProbeRayTracingPushConstant {
  glm::vec4 first_probe = glm::vec4(0.0f);
  glm::vec4 probe_step_x = glm::vec4(0.0f);
  glm::vec4 probe_step_y = glm::vec4(0.0f);
  glm::vec4 probe_step_z = glm::vec4(0.0f);
  glm::uvec4 probe_counts_and_ray_count = glm::uvec4(1, 1, 1, 1);
  glm::uvec4 selected_probe_volume_flags_environment = glm::uvec4(~0u, 0u, 0u, 0u);
  glm::vec4 trace_parameters = glm::vec4(1e27f, 0.001f, 0.0f, 0.0f);
  glm::ivec4 probe_scroll_offset = glm::ivec4(0);
};

struct DdgiProbeAtlasUpdatePushConstant {
  glm::uvec4 probe_count_ray_count_and_tile_sizes = glm::uvec4(1, 1, 1, 1);
  glm::uvec4 atlas_columns_fixed_ray_count_and_update_mode = glm::uvec4(1, 1, 0, 0);
  glm::uvec4 probe_counts_and_rotation = glm::uvec4(1, 1, 1, 0);
  glm::vec4 update_parameters = glm::vec4(1e27f, 0.97f, 5.0f, 0.0f);
  glm::vec4 blend_parameters = glm::vec4(0.1f, 50.0f, 0.10f, 0.2f);
  glm::ivec4 probe_scroll_offset = glm::ivec4(0);
  glm::ivec4 probe_scroll_delta = glm::ivec4(0);
  glm::vec4 probe_step_x = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  glm::vec4 probe_step_y = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
  glm::vec4 probe_step_z = glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
};

struct DdgiProbeScrollPushConstant {
  glm::uvec4 probe_counts_and_irradiance_tile_size = glm::uvec4(1, 1, 1, 1);
  glm::uvec4 atlas_columns_visibility_tile_and_probe_count = glm::uvec4(1, 1, 1, 1);
  glm::ivec4 probe_scroll_offset = glm::ivec4(0);
  glm::ivec4 probe_scroll_delta = glm::ivec4(0);
};

struct DdgiProbeVariabilityPushConstant {
  glm::uvec4 input_output_extent = glm::uvec4(1, 1, 1, 1);
  glm::uvec4 atlas_parameters = glm::uvec4(1, 1, 1, 0);
};

struct DdgiProbeRelocationPushConstant {
  glm::uvec4 probe_count_ray_count_and_flags = glm::uvec4(1, 1, 0, 0);
  glm::uvec4 probe_counts = glm::uvec4(1, 1, 1, 0);
  glm::vec4 relocation_parameters = glm::vec4(1.0f, 0.25f, 0.0f, 0.0f);
  glm::ivec4 probe_scroll_offset = glm::ivec4(0);
  glm::ivec4 probe_scroll_delta = glm::ivec4(0);
  glm::vec4 probe_step_x = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  glm::vec4 probe_step_y = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
  glm::vec4 probe_step_z = glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
};
static_assert(sizeof(DdgiProbeRelocationPushConstant) == 128);

struct DdgiProbeClassificationPushConstant {
  glm::uvec4 probe_count_ray_count_and_flags = glm::uvec4(1, 1, 0, 0);
  glm::uvec4 probe_counts = glm::uvec4(1, 1, 1, 0);
  glm::vec4 classification_parameters = glm::vec4(0.25f, 0.0f, 0.0f, 0.0f);
  glm::ivec4 probe_scroll_offset = glm::ivec4(0);
  glm::vec4 probe_step_x = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  glm::vec4 probe_step_y = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
  glm::vec4 probe_step_z = glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
};

struct DdgiExternalGeometry {
  std::shared_ptr<BottomLevelAccelerationStructure> bottom_level_acceleration_structure{};
  int32_t triangle_offset = -1;
  uint32_t triangle_count = 0;
  uint32_t geometry_version = 0;

  [[nodiscard]] bool IsValid() const {
    return bottom_level_acceleration_structure && triangle_offset >= 0;
  }
};

/**
 * @brief Enumeration for defining the type of render instance.
 */
enum class RenderInstanceType {
  Unknown,       ///< The render instance type is unknown.
  FromRenderer,  ///< Render instance comes from a renderer.
  FromApi        ///< Render instance comes from the API.
};

/**
 * @brief Class for managing render instance storage.
 */
class EVOENGINE_API RenderInstanceStorage {
 public:
  struct EVOENGINE_API EntitySelectionRenderSnapshot {
    std::weak_ptr<Scene> scene;
    uint64_t selection_revision = 0;
    uint64_t hierarchy_revision = 0;
    Bound world_bound{};
    bool has_renderable_bounds = false;

    void Include(const Bound& bound);
    [[nodiscard]] bool Matches(const std::shared_ptr<Scene>& target_scene, uint64_t target_selection_revision,
                               uint64_t target_hierarchy_revision) const;
  };

  static constexpr uint32_t kDdgiMaxVolumeCount = 8;
  static constexpr uint32_t kReflectionProbeMaxCount = 32;

  struct alignas(16) DdgiVolumeInfoBlock {
    glm::vec4 first_probe = glm::vec4(0.0f);
    glm::vec4 probe_step_x = glm::vec4(0.0f);
    glm::vec4 probe_step_y = glm::vec4(0.0f);
    glm::vec4 probe_step_z = glm::vec4(0.0f);
    glm::vec4 probe_counts = glm::vec4(1.0f);
    glm::ivec4 probe_scroll_and_priority = glm::ivec4(0);
    glm::uvec4 atlas_parameters = glm::uvec4(1u);
    glm::vec4 volume_parameters = glm::vec4(0.0f);
    glm::vec4 lighting_parameters = glm::vec4(0.0f);
    glm::uvec4 identity_and_flags = glm::uvec4(0u);

    bool operator!=(const DdgiVolumeInfoBlock& other) const;
  };

  struct alignas(16) ReflectionProbeInfoBlock {
    glm::mat4 world_to_probe = glm::mat4(1.0f);
    glm::vec4 shape_parameters = glm::vec4(0.0f);
    glm::vec4 projection_parameters = glm::vec4(0.0f);
    glm::vec4 lighting_parameters = glm::vec4(0.0f);
    glm::uvec4 identity_and_flags = glm::uvec4(0u);
    glm::uvec4 transition_parameters = glm::uvec4(0u);

    bool operator!=(const ReflectionProbeInfoBlock& other) const;
  };

  struct ReflectionProbeTextureOverride {
    uint32_t source_texture_index = 0u;
    uint32_t target_texture_index = 0u;
    float blend_weight = 0.0f;
    bool source_valid = false;
    bool target_valid = false;
  };

  /**
   * @brief Struct to hold information related to render settings applied.
   */
  struct alignas(16) RenderInfoBlock {
    glm::vec4 split_distances = {};                           ///< Distances for shadow cascade splits.
    alignas(4) int reserved_0 = 0;                            ///< Preserves the shader buffer layout.
    alignas(4) int debug_visualization = 0;                   ///< Debug visualization flag.
    alignas(4) float shadow_cascade_transition_width = 5.0f;  ///< Cascade blend width.
    alignas(4) float indirect_lighting_intensity = 1.0f;

    alignas(4) float strands_subdivision_x_factor = 50.0f;  ///< X factor for strands subdivision.
    alignas(4) float strands_subdivision_y_factor = 50.0f;  ///< Y factor for strands subdivision.
    alignas(4) int strands_subdivision_max_x = 15;          ///< Max subdivisions in X-axis for strands.
    alignas(4) int strands_subdivision_max_y = 8;           ///< Max subdivisions in Y-axis for strands.

    alignas(4) int directional_light_size = 0;  ///< Number of directional lights.
    alignas(4) int point_light_size = 0;        ///< Number of point lights.
    alignas(4) int spot_light_size = 0;         ///< Number of spot lights.
    alignas(4) int brdflut_texture_index = 0;   ///< Texture index for BRDF LUT.

    glm::ivec4 shadow_debug_parameters = glm::ivec4(0);  ///< Debug mode, cascade, light, and reserved value.
    glm::vec4 shadow_fade_parameters = glm::vec4(20.0f, 0.0f, 0.0f, 0.0f);
    glm::uvec4 emissive_triangle_parameters = glm::uvec4(0);
    glm::uvec4 ddgi_volume_header = glm::uvec4(0u);
    std::array<DdgiVolumeInfoBlock, kDdgiMaxVolumeCount> ddgi_volumes{};
    glm::uvec4 reflection_probe_header = glm::uvec4(0u);
    std::array<ReflectionProbeInfoBlock, kReflectionProbeMaxCount> reflection_probes{};

    /**
     * @brief Applies the settings from the target RenderSettings.
     * @param target_render_settings Render settings to be applied.
     */
    EVOENGINE_API void Apply(const RenderSettings& target_render_settings);

    /**
     * @brief Compares two RenderInfoBlock objects for inequality.
     * @param other The other RenderInfoBlock object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const RenderInfoBlock& other) const;
  };

  static_assert(sizeof(glm::vec4) == 16);
  static_assert(sizeof(glm::mat4) == 64);
  static_assert(alignof(DdgiVolumeInfoBlock) == 16);
  static_assert(sizeof(DdgiVolumeInfoBlock) == 160);
  static_assert(alignof(ReflectionProbeInfoBlock) == 16);
  static_assert(sizeof(ReflectionProbeInfoBlock) == 144);
  static_assert(offsetof(ReflectionProbeInfoBlock, world_to_probe) == 0);
  static_assert(offsetof(ReflectionProbeInfoBlock, shape_parameters) == 64);
  static_assert(offsetof(ReflectionProbeInfoBlock, projection_parameters) == 80);
  static_assert(offsetof(ReflectionProbeInfoBlock, lighting_parameters) == 96);
  static_assert(offsetof(ReflectionProbeInfoBlock, identity_and_flags) == 112);
  static_assert(offsetof(ReflectionProbeInfoBlock, transition_parameters) == 128);
  static_assert(offsetof(RenderInfoBlock, indirect_lighting_intensity) == 28);
  static_assert(offsetof(RenderInfoBlock, ddgi_volume_header) == 112);
  static_assert(offsetof(RenderInfoBlock, ddgi_volumes) == 128);
  static_assert(offsetof(RenderInfoBlock, reflection_probe_header) == 1408);
  static_assert(offsetof(RenderInfoBlock, reflection_probes) == 1424);
  static_assert(sizeof(RenderInfoBlock) == 6032);

  struct EmissiveAliasEntry {
    float alias_probability = 1.0f;
    uint32_t alias_index = 0;
    float selection_probability = 0.0f;
  };

  struct EmissiveInstanceInfoBlock {
    uint32_t instance_index = 0;
    uint32_t distribution_index = 0;
    float power_alias_probability = 1.0f;
    uint32_t power_alias_index = 0;
    float uniform_alias_probability = 1.0f;
    uint32_t uniform_alias_index = 0;
    float power_selection_probability = 0.0f;
    float uniform_selection_probability = 0.0f;
  };

  struct EmissiveTriangleDistributionInfoBlock {
    uint32_t triangle_offset = 0;
    uint32_t triangle_count = 0;
  };

  struct EmissiveTriangleInfoBlock {
    uint32_t primitive_id = 0;
    float alias_probability = 1.0f;
    uint32_t alias_index = 0;
    float selection_probability = 0.0f;
  };

  struct EmissiveTriangleCandidate {
    uint32_t primitive_id = 0;
    double area = 0.0;
    double importance = 0.0;
  };

  struct EmissiveTriangleInventoryStats {
    uint32_t eligible_instance_count = 0;
    uint32_t excluded_emissive_instance_count = 0;
    uint32_t unrepresentable_probability_count = 0;
    uint32_t distribution_count = 0;
    uint32_t fallback_distribution_count = 0;
    uint64_t logical_triangle_count = 0;
    uint64_t stored_triangle_count = 0;
    double estimated_emitted_power = 0.0;
    double build_ms = 0.0;
    double upload_ms = 0.0;
  };

  [[nodiscard]] static std::vector<EmissiveAliasEntry> BuildEmissiveAliasTable(const std::vector<double>& weights);
  [[nodiscard]] static std::vector<EmissiveTriangleInfoBlock> BuildEmissiveTriangleDistribution(
      std::vector<EmissiveTriangleCandidate> candidates);

  /**
   * @brief Struct to hold environment-related rendering information.
   */
  struct EnvironmentInfoBlock {
    glm::vec4 background_color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);  ///< Background color of the environment.
    alignas(4) float environmental_map_gamma = 2.2f;                 ///< Gamma correction for the environmental map.
    alignas(4) float diffuse_sky_intensity = 1.0f;                   ///< Effective diffuse sky-source scale.
    alignas(4) float global_reflection_intensity = 1.0f;             ///< Effective global sky-reflection scale.
    alignas(4) float environment_type = 0.0f;                        ///< GPU environment source type.
    alignas(4) float environment_pdf_texture_index = -1.0f;          ///< Texture index for the environment CDF/PDF map.
    alignas(4) float environment_cubemap_index = -1.0f;   ///< Cubemap index for ray-traced environment light.
    alignas(4) float environment_rotation = 0.0f;         ///< Y-axis rotation in radians.
    alignas(4) float diffuse_fallback_intensity = 1.0f;   ///< Raster diffuse IBL fallback scale.
    alignas(4) float specular_fallback_intensity = 1.0f;  ///< Effective raster specular fallback scale.

    /**
     * @brief Compares two EnvironmentInfoBlock objects for inequality.
     * @param other The other EnvironmentInfoBlock object to compare.
     * @return True if the objects are not equal.
     */
    EVOENGINE_API bool operator!=(const EnvironmentInfoBlock& other) const;
  };

  /**
   * @brief Struct to hold instance-related rendering information.
   */
  struct InstanceInfoBlock {
    GlobalTransform model = {};        ///< Global transform of the instance.
    int32_t material_index = 0;        ///< Material index used by the instance.
    int32_t triangle_offset = 0;       ///< Offset for triangles in the mesh.
    int32_t meshlet_index_offset = 0;  ///< Offset for meshlet indices.
    int32_t meshlet_size = 0;          ///< Size of the meshlet.

    int32_t info_index = 0;      ///< Index for additional instance information.
    uint32_t entity_index = 0;   ///< Entity index associated with the instance.
    Handle renderer_handle = 0;  ///< Handle for the renderer.
    glm::ivec4 ray_tracing_geometry = {};
    glm::vec4 world_bound_min = {};
    glm::vec4 world_bound_max = {};

    /**
     * @brief Compares two InstanceInfoBlock objects for inequality.
     * @param other The other InstanceInfoBlock object to compare.
     * @return True if the objects are not equal.
     */
    EVOENGINE_API bool operator!=(const InstanceInfoBlock& other) const;
  };
  static_assert(sizeof(InstanceInfoBlock) == 144);

  struct PreviousInstanceInfoBlock {
    glm::mat4 previous_model = glm::mat4(1.0f);
    glm::uvec4 flags = {};
  };

  /**
   * @brief Abstract struct defining an interface for render instances.
   */
  struct EVOENGINE_API IRenderInstance {
    int32_t instance_index = 0;                                     ///< Index of the render instance.
    int32_t material_index = 0;                                     ///< Material index used by the render instance.
    RenderInstanceType command_type = RenderInstanceType::Unknown;  ///< Type of the render instance.
    Entity owner = Entity();                                        ///< Entity owning the render instance.
    Handle entity_handle;                                           ///< Handle for the entity.
    Handle renderer_handle = 0;                                     ///< Handle for the renderer.
    bool entity_selected = false;                                   ///< Indicates if the entity is selected.
    GlobalTransform model = {};                                     ///< Global transform for the render instance.
    float line_width = 1.0f;                                        ///< Line width for rendering.
    VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;              ///< Culling mode for rendering.
    VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;              ///< Polygon rendering mode.
    bool cast_shadow = true;                                        ///< Indicates if the render instance casts shadows.
    Bound world_bound{};                                            ///< World-space bounds used by shadow culling.
    uint32_t material_version;                                      ///< Material version used by the render instance.
    uint32_t geometry_version;                                      ///< Geometry version used by the render instance.
    std::shared_ptr<Material> material;                             ///< Material used by the render instance.

    /**
     * @brief Applies the rendering information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    virtual void Apply(InstanceInfoBlock& instance_info_block) const = 0;

    /**
     * @brief Renders the instance using the provided Vulkan command buffer and pipeline.
     * @param vk_command_buffer Vulkan command buffer for rendering.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return A 32-bit unsigned integer indicating rendering success.
     */
    virtual uint32_t Render(VkCommandBuffer vk_command_buffer,
                            const RenderInstancePushConstant& render_instance_push_constant,
                            const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const = 0;
  };

  /**
   * @brief Struct for external render instance functionality.
   */
  struct EVOENGINE_API ExternalRenderInstance : IRenderInstance {
    DdgiExternalGeometry ddgi_geometry{};  ///< Optional standard-payload geometry for DDGI ray tracing.

    /**
     * @brief Compares two ExternalRenderInstance objects for inequality.
     * @param other The other ExternalRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const ExternalRenderInstance& other) const;

    [[nodiscard]] bool HasDdgiRayTracingGeometry() const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the external instance using the given Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Struct for mesh render instance functionality.
   */
  struct EVOENGINE_API MeshRenderInstance : IRenderInstance {
    uint32_t ray_tracing_geometry_version = 0;
    uint32_t morph_weights_version = 0;
    std::shared_ptr<Mesh> mesh;  ///< Shared pointer to the mesh rendered.
    std::shared_ptr<RangeDescriptor> ray_tracing_triangle_range;
    std::shared_ptr<BottomLevelAccelerationStructure> ray_tracing_blas;

    /**
     * @brief Compares two MeshRenderInstance objects for inequality.
     * @param other The other MeshRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const MeshRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the mesh instance using the given Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Struct for skinned mesh render instance functionality.
   */
  struct EVOENGINE_API SkinnedMeshRenderInstance : IRenderInstance {
    uint32_t bone_matrices_version;  ///< Version of the bone matrices for the skinned mesh.
    uint32_t ray_tracing_geometry_version = 0;
    uint32_t morph_weights_version = 0;
    std::shared_ptr<SkinnedMesh> skinned_mesh;    ///< Shared pointer to the skinned mesh.
    std::shared_ptr<BoneMatrices> bone_matrices;  ///< Shared pointer to bone matrices needed.
    std::vector<glm::mat4> bone_matrices_snapshot;
    std::shared_ptr<RangeDescriptor> ray_tracing_triangle_range;         ///< Animated ray tracing payload range.
    std::shared_ptr<BottomLevelAccelerationStructure> ray_tracing_blas;  ///< Animated-pose BLAS for ray tracing.

    /**
     * @brief Compares two SkinnedMeshRenderInstance objects for inequality.
     * @param other The other SkinnedMeshRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const SkinnedMeshRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the skinned mesh instance using the given Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Struct for instanced render instance functionality.
   */
  struct EVOENGINE_API InstancedRenderInstance : IRenderInstance {
    uint32_t particle_info_list_version;                 ///< Version of the particle information list.
    std::shared_ptr<Mesh> mesh;                          ///< Shared pointer to the mesh.
    std::shared_ptr<ParticleInfoList> particle_infos;    ///< Shared pointer to the particle information list.
    std::vector<uint32_t> ray_tracing_instance_indices;  ///< Ray-only instance blocks, one per particle.

    /**
     * @brief Compares two InstancedRenderInstance objects for inequality.
     * @param other The other InstancedRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const InstancedRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the instanced object using the provided Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Struct for strands render instance functionality.
   */
  struct EVOENGINE_API StrandsRenderInstance : IRenderInstance {
    std::shared_ptr<Strands> strands;  ///< Shared pointer to the strands to be rendered.

    /**
     * @brief Compares two StrandsRenderInstance objects for inequality.
     * @param other The other StrandsRenderInstance object to compare.
     * @return True if the objects are not equal.
     */
    bool operator!=(const StrandsRenderInstance& other) const;

    /**
     * @brief Apply instance information to the given InstanceInfoBlock.
     * @param instance_info_block The instance information block to update.
     */
    void Apply(InstanceInfoBlock& instance_info_block) const override;

    /**
     * @brief Renders the strands instance using the given Vulkan resources.
     * @param vk_command_buffer Vulkan command buffer to use.
     * @param render_instance_push_constant Push constants for the render instance.
     * @param graphics_pipeline Graphics pipeline for rendering.
     * @return Rendering result as a 32-bit unsigned integer.
     */
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  struct EVOENGINE_API GaussianSplatRenderInstance : IRenderInstance {
    std::shared_ptr<GaussianSplat> gaussian_splat;
    float opacity_scale = 1.0f;
    int sh_degree = 0;
    GaussianSplatSortMode sort_mode = GaussianSplatSortMode::GpuRadix;
    GaussianSplatDepthMode depth_mode = GaussianSplatDepthMode::SceneDepth;
    GaussianSplatRasterMode raster_mode = GaussianSplatRasterMode::Auto;

    bool operator!=(const GaussianSplatRenderInstance& other) const;
    void Apply(InstanceInfoBlock& instance_info_block) const override;
    uint32_t Render(VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
                    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const override;
  };

  /**
   * @brief Interface for a collection of render instances.
   */
  class IRenderInstanceCollection {
   public:
    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    virtual bool Empty() const = 0;

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    virtual void Register(const std::shared_ptr<IRenderInstance>& render_instance) = 0;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    virtual void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) = 0;
  };

  /**
   * @brief Collection of external render instances.
   */
  class EVOENGINE_API ExternalRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<ExternalRenderInstance>> render_commands;  ///< Commands for external rendering.

   public:
    /**
     * @brief Compares two ExternalRenderInstanceCollection objects for inequality.
     * @param other The other ExternalRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const ExternalRenderInstanceCollection& other) const;

    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;
    void Clear();

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;

    void ForEachExternalRenderInstance(
        const std::function<void(const std::shared_ptr<ExternalRenderInstance>&)>& action) const;

    [[nodiscard]] bool HasDdgiRayTracingGeometry() const;
  };

  /**
   * @brief Collection of mesh render instances.
   */
  class EVOENGINE_API MeshRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<MeshRenderInstance>> render_commands;  ///< Commands for mesh rendering.

   public:
    /**
     * @brief Compares two MeshRenderInstanceCollection objects for inequality.
     * @param other The other MeshRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const MeshRenderInstanceCollection& other) const;

    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;
    void Clear();

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;

    void ForEachMeshRenderInstance(const std::function<void(const std::shared_ptr<MeshRenderInstance>&)>& action) const;
  };

  /**
   * @brief Collection of skinned mesh render instances.
   */
  class EVOENGINE_API SkinnedMeshRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<SkinnedMeshRenderInstance>> render_commands;  ///< Commands for skinned mesh rendering.

   public:
    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;
    void Clear();

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Compares two SkinnedMeshRenderInstanceCollection objects for inequality.
     * @param other The other SkinnedMeshRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const SkinnedMeshRenderInstanceCollection& other) const;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;

    void ForEachSkinnedMeshRenderInstance(
        const std::function<void(const std::shared_ptr<SkinnedMeshRenderInstance>&)>& action) const;
  };

  /**
   * @brief Collection of strands render instances.
   */
  class EVOENGINE_API StrandsRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<StrandsRenderInstance>> render_commands;  ///< Commands for strands rendering.

   public:
    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;
    void Clear();

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Compares two StrandsRenderInstanceCollection objects for inequality.
     * @param other The other StrandsRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const StrandsRenderInstanceCollection& other) const;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;

    void ForEachStrandsRenderInstance(
        const std::function<void(const std::shared_ptr<StrandsRenderInstance>&)>& action) const;
  };

  class EVOENGINE_API GaussianSplatRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<GaussianSplatRenderInstance>> render_commands;

   public:
    bool Empty() const override;
    void Clear();
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;
    bool operator!=(const GaussianSplatRenderInstanceCollection& other) const;
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;

    void ForEachGaussianSplatRenderInstance(
        const std::function<void(const std::shared_ptr<GaussianSplatRenderInstance>&)>& action) const;
  };

  /**
   * @brief Collection of instanced render instances.
   */
  class EVOENGINE_API InstancedRenderInstanceCollection : public IRenderInstanceCollection {
    std::vector<std::shared_ptr<InstancedRenderInstance>> render_commands;  ///< Commands for instanced rendering.

   public:
    /**
     * @brief Checks whether the collection is empty.
     * @return True if the collection is empty, false otherwise.
     */
    bool Empty() const override;
    void Clear();

    /**
     * @brief Registers a render instance in the collection.
     * @param render_instance The render instance to register.
     */
    void Register(const std::shared_ptr<IRenderInstance>& render_instance) override;

    /**
     * @brief Compares two InstancedRenderInstanceCollection objects for inequality.
     * @param other The other InstancedRenderInstanceCollection object to compare.
     * @return True if the collections are not equal.
     */
    bool operator!=(const InstancedRenderInstanceCollection& other) const;

    /**
     * @brief Applies an action to each render instance in the collection.
     * @param action The action to apply.
     */
    void ForEachRenderInstance(const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) override;

    void ForEachInstancedRenderInstance(
        const std::function<void(const std::shared_ptr<InstancedRenderInstance>&)>& action) const;
  };

  /**
   * @brief Default constructor for RenderInstanceStorage.
   */
  RenderInstanceStorage();

  /**
   * @brief Compares two RenderInstanceStorage objects for inequality.
   * @param other The other RenderInstanceStorage object to compare.
   * @return True if the objects are not equal.
   */
  bool operator!=(const RenderInstanceStorage& other) const;

  /**
   * @brief Registers a mesh draw command.
   * @param mesh The mesh to draw.
   * @param material The material used for rendering the mesh.
   * @param model The global transform of the mesh.
   * @param cast_shadow Indicates whether the mesh casts shadows.
   * @return True if the registration was successful.
   */
  bool RegisterMeshDrawCommand(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                               const GlobalTransform& model, bool cast_shadow);

  bool RegisterStrandsDrawCommand(const std::shared_ptr<Strands>& strands, const std::shared_ptr<Material>& material,
                                  const GlobalTransform& model, bool cast_shadow);

  /**
   * @brief Registers a mesh draw instanced command.
   * @param mesh The mesh to draw.
   * @param material The material used for rendering the mesh.
   * @param model The global transform of the mesh.
   * @param particle_info_list The particle information list for the mesh instances.
   * @param cast_shadow Indicates whether the mesh casts shadows.
   * @return True if the registration was successful.
   */
  bool RegisterMeshDrawInstancedCommand(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                        const GlobalTransform& model,
                                        const std::shared_ptr<ParticleInfoList>& particle_info_list, bool cast_shadow);

  /**
   * @brief Registers a render instance for the specified entity in the target scene.
   * @param target_scene The scene containing the render instance.
   * @param entity The entity owning the render instance.
   * @param renderer_handle The handle of the renderer.
   * @param material The material associated with the render instance.
   * @return True if the registration was successful.
   */
  bool RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                              const Handle& renderer_handle, const std::shared_ptr<Material>& material,
                              int* out_material_index = nullptr);

  bool RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                              const Handle& renderer_handle, const std::shared_ptr<Material>& material,
                              const DdgiExternalGeometry& ddgi_geometry, int* out_material_index = nullptr);

  /**
   * @brief Registers a material and returns its index.
   * @param material Shared pointer to the material being registered.
   * @return Index of the registered material.
   */
  [[nodiscard]] int RegisterMaterial(const std::shared_ptr<Material>& material);
  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  RenderSettings render_settings{};
  std::shared_ptr<Buffer> gltf_material_descriptor_buffer = {};
  std::shared_ptr<Buffer> gltf_texture_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> instance_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> previous_instance_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> environment_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> directional_light_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> point_light_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> spot_light_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> render_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> camera_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> emissive_instance_info_descriptor_buffer = {};
  std::shared_ptr<Buffer> emissive_triangle_distribution_descriptor_buffer = {};
  std::shared_ptr<Buffer> emissive_triangle_info_descriptor_buffer = {};

  std::shared_ptr<TopLevelAccelerationStructure> mesh_top_level_acceleration_structure{};

  struct DeferredMeshIndirectBatch {
    uint32_t first_command = 0;
    uint32_t command_count = 0;
    uint32_t triangle_count = 0;
    float line_width = 1.0f;
    VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
    VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  };

  class EVOENGINE_API RasterSpatialIndex {
   public:
    struct UpdateStats {
      uint32_t leaf_count = 0;
      uint32_t node_count = 0;
      uint32_t max_depth = 0;
      uint32_t inserted_leaves = 0;
      uint32_t removed_leaves = 0;
      uint32_t reinserted_leaves = 0;
      uint32_t unchanged_leaves = 0;
    };

    struct QueryStats {
      uint32_t visited_nodes = 0;
      uint32_t tested_leaves = 0;
      uint32_t accepted_leaves = 0;
    };

    void BeginUpdate();
    void Upsert(Handle handle, const Bound& bound);
    void EndUpdate();
    [[nodiscard]] std::vector<Handle> Query(const std::function<bool(const Bound&)>& intersects,
                                            QueryStats* stats = nullptr) const;
    [[nodiscard]] const UpdateStats& GetUpdateStats() const;

   private:
    struct EVOENGINE_API Node {
      Bound bound{};
      Bound exact_bound{};
      Handle handle = 0;
      int32_t parent = -1;
      int32_t left = -1;
      int32_t right = -1;
      bool active = false;

      [[nodiscard]] bool IsLeaf() const;
    };

    int32_t root_ = -1;
    std::vector<Node> nodes_{};
    std::vector<int32_t> free_nodes_{};
    std::unordered_map<Handle, int32_t> leaves_{};
    std::unordered_set<Handle> seen_{};
    UpdateStats update_stats_{};

    [[nodiscard]] int32_t AllocateNode();
    void ReleaseNode(int32_t node_index);
    void InsertLeaf(int32_t leaf_index);
    void DetachLeaf(int32_t leaf_index);
    void RefreshAncestors(int32_t node_index);
    void RebuildBalanced();
    void RefreshTreeStats();
  };

  struct CameraRasterVisibility {
    bool enabled = false;
    uint32_t draw_instance_index_offset = 0;
    VkDeviceSize indexed_indirect_buffer_offset = 0;
    VkDeviceSize mesh_task_indirect_buffer_offset = 0;
    std::vector<uint8_t> instance_visibility;
    std::vector<DeferredMeshIndirectBatch> deferred_mesh_indirect_batches;
    std::vector<DeferredMeshIndirectBatch> deferred_masked_mesh_indirect_batches;
    std::vector<VkDrawIndexedIndirectCommand> mesh_draw_indexed_indirect_commands;
    std::shared_ptr<Buffer> mesh_draw_indexed_indirect_commands_buffer;
    std::vector<VkDrawMeshTasksIndirectCommandEXT> mesh_draw_mesh_tasks_indirect_commands;
    std::shared_ptr<Buffer> mesh_draw_mesh_tasks_indirect_commands_buffer;
    uint32_t total_gaussian_splats = 0;
    std::shared_ptr<MeshRenderInstanceCollection> deferred_render_instances;
    std::shared_ptr<SkinnedMeshRenderInstanceCollection> deferred_skinned_render_instances;
    std::shared_ptr<InstancedRenderInstanceCollection> deferred_instanced_render_instances;
    std::shared_ptr<StrandsRenderInstanceCollection> deferred_strands_render_instances;
    std::shared_ptr<MeshRenderInstanceCollection> deferred_masked_render_instances;
    std::shared_ptr<SkinnedMeshRenderInstanceCollection> deferred_masked_skinned_render_instances;
    std::shared_ptr<InstancedRenderInstanceCollection> deferred_masked_instanced_render_instances;
    std::shared_ptr<StrandsRenderInstanceCollection> deferred_masked_strands_render_instances;
    std::shared_ptr<MeshRenderInstanceCollection> forward_render_instances;
    std::shared_ptr<SkinnedMeshRenderInstanceCollection> forward_skinned_render_instances;
    std::shared_ptr<InstancedRenderInstanceCollection> forward_instanced_render_instances;
    std::shared_ptr<StrandsRenderInstanceCollection> forward_strands_render_instances;
    std::shared_ptr<MeshRenderInstanceCollection> transparent_render_instances;
    std::shared_ptr<SkinnedMeshRenderInstanceCollection> transparent_skinned_render_instances;
    std::shared_ptr<InstancedRenderInstanceCollection> transparent_instanced_render_instances;
    std::shared_ptr<StrandsRenderInstanceCollection> transparent_strands_render_instances;
    std::shared_ptr<GaussianSplatRenderInstanceCollection> gaussian_splat_render_instances;
    std::vector<uint32_t> draw_instance_indices;
  };

  struct InstanceUploadRange {
    uint32_t first_instance = 0;
    uint32_t instance_count = 0;
  };

  struct ShadowViewIndirectCommands {
    uint32_t draw_instance_index_offset = 0;
    VkDeviceSize indirect_buffer_offset = 0;
    std::vector<DeferredMeshIndirectBatch> opaque_mesh_indirect_batches;
    std::vector<DeferredMeshIndirectBatch> masked_mesh_indirect_batches;
    std::vector<VkDrawIndexedIndirectCommand> indexed_commands;
    std::vector<VkDrawMeshTasksIndirectCommandEXT> mesh_task_commands;
    std::shared_ptr<Buffer> indirect_buffer;
    std::shared_ptr<MeshRenderInstanceCollection> deferred_render_instances;
    std::shared_ptr<SkinnedMeshRenderInstanceCollection> deferred_skinned_render_instances;
    std::shared_ptr<InstancedRenderInstanceCollection> deferred_instanced_render_instances;
    std::shared_ptr<StrandsRenderInstanceCollection> deferred_strands_render_instances;
    std::shared_ptr<MeshRenderInstanceCollection> deferred_masked_render_instances;
    std::shared_ptr<SkinnedMeshRenderInstanceCollection> deferred_masked_skinned_render_instances;
    std::shared_ptr<InstancedRenderInstanceCollection> deferred_masked_instanced_render_instances;
    std::shared_ptr<StrandsRenderInstanceCollection> deferred_masked_strands_render_instances;
    std::vector<uint32_t> draw_instance_indices;
  };

  std::vector<DeferredMeshIndirectBatch> deferred_mesh_indirect_batches;
  std::vector<DeferredMeshIndirectBatch> deferred_masked_mesh_indirect_batches;

  uint32_t deferred_mesh_draw_instance_index_offset = 0;
  std::vector<uint32_t> raster_draw_instance_indices;
  std::shared_ptr<Buffer> raster_draw_instance_indices_buffer;

  std::vector<VkDrawIndexedIndirectCommand> mesh_draw_indexed_indirect_commands;
  std::shared_ptr<Buffer> mesh_draw_indexed_indirect_commands_buffer;

  std::vector<VkDrawMeshTasksIndirectCommandEXT> mesh_draw_mesh_tasks_indirect_commands;
  std::shared_ptr<Buffer> mesh_draw_mesh_tasks_indirect_commands_buffer;

  std::vector<DeferredMeshIndirectBatch> opaque_shadow_mesh_indirect_batches;
  std::vector<DeferredMeshIndirectBatch> masked_shadow_mesh_indirect_batches;

  std::vector<VkDrawIndexedIndirectCommand> shadow_mesh_draw_indexed_indirect_commands;
  std::shared_ptr<Buffer> shadow_mesh_draw_indexed_indirect_commands_buffer;

  std::vector<VkDrawMeshTasksIndirectCommandEXT> shadow_mesh_draw_mesh_tasks_indirect_commands;
  std::shared_ptr<Buffer> shadow_mesh_draw_mesh_tasks_indirect_commands_buffer;
  std::vector<VkDrawIndexedIndirectCommand> packed_shadow_indexed_commands;
  std::vector<VkDrawMeshTasksIndirectCommandEXT> packed_shadow_mesh_task_commands;
  std::shared_ptr<Buffer> packed_shadow_indirect_buffer;
  bool packed_shadow_uses_mesh_shader_ = false;
  BufferUploadArena upload_arena_{};
  std::unordered_map<const Buffer*, uint64_t> uploaded_payload_signatures_{};
  std::vector<VkDrawIndexedIndirectCommand> packed_camera_indexed_commands;
  std::vector<VkDrawMeshTasksIndirectCommandEXT> packed_camera_mesh_task_commands;
  std::shared_ptr<Buffer> packed_camera_indexed_buffer;
  std::shared_ptr<Buffer> packed_camera_mesh_task_buffer;
  std::vector<ShadowViewIndirectCommands> directional_shadow_views_;
  std::vector<ShadowViewIndirectCommands> point_shadow_views_;
  std::vector<ShadowViewIndirectCommands> spot_shadow_views_;
  uint32_t directional_shadow_light_count_ = 0;

  uint32_t total_skinned_mesh_triangles = 0;
  uint32_t total_instanced_mesh_triangles = 0;
  uint32_t total_strands_segments = 0;
  uint32_t total_gaussian_splats = 0;

  [[nodiscard]] static bool IsFiniteBound(const Bound& bound);
  [[nodiscard]] static bool BoundIntersectsCameraClipSpace(const Bound& bound, const glm::mat4& projection_view);
  [[nodiscard]] static bool BoundIntersectsShadowClipSpace(const Bound& bound, const glm::mat4& projection_view);
  [[nodiscard]] static bool MeshletSphereIntersectsClipSpace(const glm::vec4& local_sphere, const glm::mat4& model,
                                                             const glm::mat4& projection_view, bool zero_near_plane);
  void BuildRasterVisibility(bool use_mesh_shader);
  [[nodiscard]] bool IsInstanceVisible(int32_t camera_index, int32_t instance_index) const;
  [[nodiscard]] const CameraRasterVisibility* GetCameraRasterVisibility(int32_t camera_index) const;
  [[nodiscard]] ShadowViewIndirectCommands BuildShadowViewIndirectCommands(const glm::mat4& light_space_matrix);
  [[nodiscard]] const ShadowViewIndirectCommands* GetDirectionalShadowView(int32_t camera_index, int32_t light_index,
                                                                           uint32_t split) const;
  [[nodiscard]] const ShadowViewIndirectCommands* GetPointShadowView(int32_t light_index, uint32_t face) const;
  [[nodiscard]] const ShadowViewIndirectCommands* GetSpotShadowView(int32_t light_index) const;
  /**
   * @brief Clears all the render instance data and collections.
   */
  void Clear();

  /**
   * @brief Collects editor cameras from the specified scene.
   * @param target_scene The scene from which to collect cameras.
   * @param cameras Vector to store collected cameras and their transforms.
   */
  static void CollectEditorCameras(const std::shared_ptr<Scene>& target_scene,
                                   std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);

  /**
   * @brief Collects all cameras from the specified scene.
   * @param target_scene The scene from which to collect cameras.
   * @param cameras Vector to store collected cameras and their transforms.
   */
  static void CollectCameras(const std::shared_ptr<Scene>& target_scene,
                             std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras);

  /**
   * @brief Calculates the Level of Detail (LOD) factor for objects in the scene based on distance.
   * @param scene The scene containing objects.
   * @param view_position The position of the viewer.
   * @param max_distance The maximum distance for LOD calculations.
   */
  static void CalculateLodFactor(const std::shared_ptr<Scene>& scene, const glm::vec3& view_position,
                                 float max_distance);

  /**
   * @brief Builds render instances from the specified scene, applying render settings.
   * @param render_settings The render settings to apply.
   * @param scene The scene from which to build render instances.
   * @param world_bound The world bounds for the scene.
   */
  void BuildFromScene(
      const RenderSettings& render_settings, const std::shared_ptr<Scene>& scene, Bound& world_bound,
      bool include_editor_cameras = true,
      const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>* injected_cameras = nullptr,
      bool include_reflection_probes = true,
      const std::unordered_map<uint64_t, ReflectionProbeTextureOverride>* reflection_probe_texture_overrides = nullptr,
      std::shared_ptr<const EntitySelectionHighlightCoverage> entity_selection_highlight_coverage = {},
      uint64_t entity_selection_revision = 0, uint64_t scene_hierarchy_revision = 0);

  [[nodiscard]] const EntitySelectionRenderSnapshot& GetEntitySelectionRenderSnapshot() const;

  /**
   * @brief Updates the top-level acceleration structure for ray tracing.
   */
  void UpdateTopLevelAccelerationStructure();

  /**
   * @brief Finds the material index via a material handle.
   * @param material_handle The handle of the target material.
   * @return Index of the material.
   */
  [[nodiscard]] int GetMaterialIndex(const Handle& material_handle);

  /**
   * @brief Finds the renderer index via a render instance index.
   * @param renderer_handle The handle of the target renderer.
   * @return Index of the render instance.
   */
  [[nodiscard]] int GetRenderInstanceIndex(const Handle& renderer_handle);

  /**
   * @brief Finds the camera index via a camera handle.
   * @param camera_handle The handle of the target camera.
   * @return Index of the camera.
   */
  [[nodiscard]] int GetCameraIndex(const Handle& camera_handle);

  struct DirectionalShadowCascadeFitInput {
    RenderSettings::ShadowCascadeFitMode mode = RenderSettings::ShadowCascadeFitMode::StableSphere;
    std::array<glm::vec3, 8> frustum_corners{};
    Bound world_bound{};
    glm::vec3 light_direction = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 light_up = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::ivec2 viewport_extent{};
    float filter_radius_world = 0.0f;
  };

  struct DirectionalShadowCascadeFitResult {
    glm::mat4 light_space_matrix{1.0f};
    glm::vec2 orthographic_min{};
    glm::vec2 orthographic_max{};
    float light_space_depth_half_extent = 0.0f;
  };

  [[nodiscard]] static DirectionalShadowCascadeFitResult CalculateDirectionalShadowCascadeFit(
      const DirectionalShadowCascadeFitInput& input);

  /**
   * @brief Finds the entity handle via a render instance index.
   * @param render_instance_index The index of the render instance.
   * @return Handle of the entity associated with the instance.
   */
  [[nodiscard]] Handle GetInstanceEntityHandle(int render_instance_index);

  /**
   * @brief Finds the renderer handle via a render instance index.
   * @param render_instance_index The index of the render instance.
   * @return Handle of the renderer associated with the instance.
   */
  [[nodiscard]] Handle GetInstanceRendererHandle(int render_instance_index);

  [[nodiscard]] bool HasSelectionHighlightRenderInstances() const;

  /**
   * @brief Uploads all data and render instance information to the GPU.
   * @param immediate Submit and complete uploads immediately instead of recording them into the current frame stream.
   */
  void Upload(bool immediate = false);

  [[nodiscard]] const std::vector<GltfShadeMaterial>& GetGltfShadeMaterials() const;

  [[nodiscard]] const std::vector<GltfTextureInfo>& GetGltfTextureInfos() const;

  [[nodiscard]] uint64_t GetDdgiEmissiveInventorySignature() const;
  [[nodiscard]] const EmissiveTriangleInventoryStats& GetDdgiEmissiveInventoryStats() const;

  /**
   * @brief Retrieves the list of instance information blocks.
   * @return Reference to the vector of InstanceInfoBlock objects.
   */
  [[nodiscard]] const std::vector<InstanceInfoBlock>& GetInstanceInfoBlocks() const;
  [[nodiscard]] const std::vector<PreviousInstanceInfoBlock>& GetPreviousInstanceInfoBlocks() const;
  [[nodiscard]] static std::vector<InstanceUploadRange> PlanInstanceInfoUploadRanges(
      const std::vector<InstanceInfoBlock>& previous, const std::vector<InstanceInfoBlock>& current);
  [[nodiscard]] static std::vector<InstanceUploadRange> PlanPreviousInstanceInfoUploadRanges(
      const std::vector<PreviousInstanceInfoBlock>& previous, const std::vector<PreviousInstanceInfoBlock>& current);
  [[nodiscard]] uint32_t GetReflectionProbeCount() const;
  [[nodiscard]] const std::array<ReflectionProbeInfoBlock, kReflectionProbeMaxCount>& GetReflectionProbeInfoBlocks()
      const;
  void BuildPreviousInstanceInfoBlocks(const std::shared_ptr<RenderInstanceStorage>& previous_render_instances);
  [[nodiscard]] bool RequiresCameraWideTemporalHistoryRejection() const;

 private:
  /**
   * @brief Finds the instance index via a renderer handle.
   */
  std::unordered_map<Handle, int> renderer_indices_;

  /**
   * @brief Finds the material index via a material handle.
   */
  std::unordered_map<Handle, int> material_indices_;

  /**
   * @brief Finds the entity handle via a render instance index.
   */
  std::unordered_map<int, Handle> instance_entity_handles_;

  /**
   * @brief Finds the renderer handle via a render instance index.
   */
  std::unordered_map<int, Handle> instance_renderer_handles_;

  /**
   * @brief Finds the camera index via a camera handle.
   */
  std::unordered_map<Handle, int> camera_indices_;

  GltfMaterialCache gltf_material_cache_{};

  /**
   * @brief Holds the instance information blocks.
   */
  std::vector<InstanceInfoBlock> instance_info_blocks_{};
  std::vector<PreviousInstanceInfoBlock> previous_instance_info_blocks_{};
  std::vector<uint32_t> rigid_motion_supported_{};
  struct PersistentTransformRecord {
    GlobalTransform model{};
    Bound local_bound{};
    Bound world_bound{};
    uint64_t content_signature = 0;
  };
  std::unordered_map<Handle, PersistentTransformRecord> persistent_transform_records_{};
  std::unordered_set<Handle> persistent_transform_seen_{};
  struct StaticMeshRenderInstanceRecord {
    Entity source_owner{};
    Bound local_bound{};
    std::shared_ptr<MeshRenderInstance> render_instance{};
    GltfRasterMaterialClass raster_class = GltfRasterMaterialClass::Opaque;
  };
  std::weak_ptr<Scene> static_mesh_cache_scene_{};
  uint64_t static_mesh_cache_structure_revision_ = 0;
  std::unordered_map<Handle, StaticMeshRenderInstanceRecord> static_mesh_render_instance_cache_{};
  std::vector<InstanceInfoBlock> uploaded_instance_info_blocks_{};
  std::vector<PreviousInstanceInfoBlock> uploaded_previous_instance_info_blocks_{};
  struct CachedMaterialData {
    uint32_t version = 0;
    GltfMaterialData data{};
  };
  std::unordered_map<Handle, CachedMaterialData> material_data_cache_{};
  std::unordered_map<Handle, uint32_t> material_versions_{};
  std::unordered_set<Handle> active_material_handles_{};
  uint64_t canonical_structure_signature_ = 0;
  bool canonical_structure_initialized_ = false;
  bool canonical_structure_changed_this_frame_ = false;
  bool material_cache_changed_this_frame_ = false;
  enum class SpatialRenderCategory : uint8_t {
    Deferred,
    DeferredMasked,
    Forward,
    Transparent,
    Gaussian,
  };
  struct SpatialRenderEntry {
    std::shared_ptr<IRenderInstance> render_instance{};
    SpatialRenderCategory category = SpatialRenderCategory::Deferred;
    int32_t deferred_mesh_command_index = -1;
  };
  RasterSpatialIndex raster_spatial_index_{};
  std::unordered_map<Handle, SpatialRenderEntry> spatial_render_entries_{};
  std::vector<SpatialRenderEntry> spatial_always_visible_entries_{};

  struct EmissiveTriangleInstanceSignature {
    uint64_t mesh_handle = 0;
    uint64_t renderer_handle = 0;
    uint64_t material_handle = 0;
    uint64_t emissive_sampling_signature = 0;
    uint32_t geometry_version = 0;
    int32_t instance_index = -1;
    int32_t material_index = -1;
    uint32_t triangle_offset = 0;
    uint32_t triangle_count = 0;
    GlobalTransform model{};
    double importance = 0.0;

    bool operator==(const EmissiveTriangleInstanceSignature& other) const;
  };
  std::vector<EmissiveInstanceInfoBlock> emissive_instance_info_blocks_{};
  std::vector<EmissiveTriangleDistributionInfoBlock> emissive_triangle_distribution_info_blocks_{};
  std::vector<EmissiveTriangleInfoBlock> emissive_triangle_info_blocks_{};
  std::vector<EmissiveTriangleInstanceSignature> emissive_triangle_instance_signatures_{};
  uint64_t emissive_sampling_signature_ = 0;
  uint64_t ddgi_emissive_inventory_signature_ = 0;
  EmissiveTriangleInventoryStats ddgi_emissive_inventory_stats_{};
  bool emissive_instance_info_dirty_ = false;
  bool emissive_triangle_distribution_info_dirty_ = false;
  bool emissive_triangle_info_dirty_ = false;

  /**
   * @brief Stores rendering-related information like shadow splits and lighting.
   */
  RenderInfoBlock render_info_block = {};

  /**
   * @brief Stores environment-related rendering information such as background color.
   */
  EnvironmentInfoBlock environment_info_block = {};
  std::vector<DirectionalLightInfoBlock> directional_light_info_blocks_;
  std::vector<PointLightInfoBlock> point_light_info_blocks_;
  std::vector<SpotLightInfoBlock> spot_light_info_blocks_;

  std::vector<CameraInfoBlock> camera_info_blocks_{};
  std::vector<CameraRasterVisibility> camera_raster_visibility_{};
  std::shared_ptr<MeshRenderInstanceCollection> deferred_render_instances;
  std::shared_ptr<SkinnedMeshRenderInstanceCollection> deferred_skinned_render_instances;
  std::shared_ptr<InstancedRenderInstanceCollection> deferred_instanced_render_instances;
  std::shared_ptr<StrandsRenderInstanceCollection> deferred_strands_render_instances;

  std::shared_ptr<MeshRenderInstanceCollection> deferred_masked_render_instances;
  std::shared_ptr<SkinnedMeshRenderInstanceCollection> deferred_masked_skinned_render_instances;
  std::shared_ptr<InstancedRenderInstanceCollection> deferred_masked_instanced_render_instances;
  std::shared_ptr<StrandsRenderInstanceCollection> deferred_masked_strands_render_instances;

  std::shared_ptr<MeshRenderInstanceCollection> forward_render_instances;
  std::shared_ptr<SkinnedMeshRenderInstanceCollection> forward_skinned_render_instances;
  std::shared_ptr<InstancedRenderInstanceCollection> forward_instanced_render_instances;
  std::shared_ptr<StrandsRenderInstanceCollection> forward_strands_render_instances;

  std::shared_ptr<MeshRenderInstanceCollection> transparent_render_instances;
  std::shared_ptr<SkinnedMeshRenderInstanceCollection> transparent_skinned_render_instances;
  std::shared_ptr<InstancedRenderInstanceCollection> transparent_instanced_render_instances;
  std::shared_ptr<StrandsRenderInstanceCollection> transparent_strands_render_instances;

  std::shared_ptr<GaussianSplatRenderInstanceCollection> gaussian_splat_render_instances;
  std::shared_ptr<ExternalRenderInstanceCollection> external_render_instances;
  struct TopLevelAccelerationStructureInput {
    std::shared_ptr<IRenderInstance> render_instance;
    std::shared_ptr<BottomLevelAccelerationStructure> bottom_level_acceleration_structure;
    glm::mat4 model{1.0f};
    uint32_t custom_index = 0;
    bool linear_swept_spheres = false;
  };
  std::vector<TopLevelAccelerationStructureInput> top_level_acceleration_structure_inputs_;
  uint32_t geometry_storage_version = 0;
  uint32_t texture_storage_version = 0;

  friend class TopLevelAccelerationStructure;
  friend class RenderLayer;
  friend class DeferredGeometryPass;
  friend class DirectionalLightShadowPass;
  friend class GaussianSplatCullPass;
  friend class GaussianSplatSortPass;
  friend class GaussianSplatPass;
  friend class MotionCoveragePass;
  friend class TransparentGeometryPass;
  friend class CpuRayTracer;
  std::shared_ptr<const EntitySelectionHighlightCoverage> entity_selection_highlight_coverage_;
  EntitySelectionRenderSnapshot entity_selection_render_snapshot_;

  [[nodiscard]] bool IsEntitySelectionHighlighted(const Entity& entity) const;
  /**
   * @brief Collects entity renderers and calculates the world bounding box.
   * @param target_scene The scene containing entities.
   * @param world_bound Output bounding box for the world.
   */
  void CollectEntityRenderers(const std::shared_ptr<Scene>& target_scene, Bound& world_bound);
  [[nodiscard]] const Bound* FindPersistentWorldBound(Handle renderer_handle, const GlobalTransform& model,
                                                      const Bound& local_bound, uint64_t content_signature);
  void StorePersistentWorldBound(Handle renderer_handle, const GlobalTransform& model, const Bound& local_bound,
                                 uint64_t content_signature, const Bound& world_bound);
  void PrunePersistentTransformRecords();
  void PrepareStaticMeshCache(const std::shared_ptr<Scene>& scene);
  void InvalidateStaticEntityCache(const std::shared_ptr<Scene>& scene, const Entity& entity);
  [[nodiscard]] const GltfMaterialData& ResolveMaterialData(const std::shared_ptr<Material>& material);
  [[nodiscard]] uint64_t CalculateCanonicalStructureSignature() const;
  void UpdateRasterSpatialIndex();
  [[nodiscard]] std::vector<SpatialRenderEntry> QueryRasterSpatialEntries(
      const std::function<bool(const Bound&)>& intersects) const;
  [[nodiscard]] CameraRasterVisibility BuildCameraRasterVisibilityResult(size_t camera_index) const;
  void MergeCameraRasterVisibilityResult(size_t camera_index, CameraRasterVisibility&& visibility);
  void MergeShadowRasterVisibilityResult(ShadowViewIndirectCommands& destination,
                                         ShadowViewIndirectCommands&& visibility, bool use_mesh_shader);
  void FinalizeShadowIndirectBuffers(bool use_mesh_shader);

  /**
   * @brief Builds render instance blocks for rendering.
   */
  void BuildRenderInstanceBlocks();
  void BuildEmissiveTriangleInfoBlocks();

  /**
   * @brief Collects lighting information from the scene.
   * @param target_scene The scene from which to collect lights.
   * @param world_bound The world bounds of the scene.
   */
  void CollectLights(const std::shared_ptr<Scene>& target_scene, const Bound& world_bound);

  /**
   * @brief Collects environment-related settings and updates their information.
   * @param target_scene The scene containing the environment settings.
   */
  void CollectEnvironment(const std::shared_ptr<Scene>& target_scene);
  void CollectReflectionProbes(
      const std::shared_ptr<Scene>& target_scene,
      const std::unordered_map<uint64_t, ReflectionProbeTextureOverride>* texture_overrides = nullptr);

  /**
   * @brief Registers an entity with a mesh renderer.
   * @param target_scene The scene containing the entity.
   * @param owner The entity to register.
   * @param mesh_renderer The mesh renderer associated with the entity.
   * @param min_bound Minimum bounds of the entity.
   * @param max_bound Maximum bounds of the entity.
   * @return True if the registration was successful.
   */
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<MeshRenderer>& mesh_renderer, glm::vec3& min_bound, glm::vec3& max_bound);

  /**
   * @brief Registers an entity with a skinned mesh renderer.
   * @param target_scene The scene containing the entity.
   * @param owner The entity to register.
   * @param skinned_mesh_renderer The skinned mesh renderer associated with the entity.
   * @param min_bound Minimum bounds of the entity.
   * @param max_bound Maximum bounds of the entity.
   * @return True if the registration was successful.
   */
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer, glm::vec3& min_bound,
                      glm::vec3& max_bound);

  /**
   * @brief Registers an entity with particle-based rendering.
   * @param target_scene The scene containing the entity.
   * @param owner The entity to register.
   * @param particles The particle system associated with the entity.
   * @param min_bound Minimum bounds of the entity.
   * @param max_bound Maximum bounds of the entity.
   * @return True if the registration was successful.
   */
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<Particles>& particles, glm::vec3& min_bound, glm::vec3& max_bound);

  /**
   * @brief Registers an entity with a strands renderer.
   * @param target_scene The scene containing the entity.
   * @param owner The entity to register.
   * @param strands_renderer The strands renderer associated with the entity.
   * @param min_bound Minimum bounds of the entity.
   * @param max_bound Maximum bounds of the entity.
   * @return True if the registration was successful.
   */
  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<StrandsRenderer>& strands_renderer, glm::vec3& min_bound,
                      glm::vec3& max_bound);

  bool RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                      const std::shared_ptr<GaussianSplatRenderer>& gaussian_splat_renderer, glm::vec3& min_bound,
                      glm::vec3& max_bound);

  [[nodiscard]] int RegisterMaterial(const std::shared_ptr<Material>& material, const GltfMaterialData& material_data);

  /**
   * @brief Registers camera information and returns its index.
   * @param handle The handle associated with the camera.
   * @param camera_info_block The camera information block to register.
   * @return Index of the registered camera.
   */
  [[nodiscard]] int RegisterCamera(const Handle& handle, const CameraInfoBlock& camera_info_block);
};

};  // namespace evo_engine
