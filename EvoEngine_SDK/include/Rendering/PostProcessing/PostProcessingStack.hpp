#pragma once
#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "IAsset.hpp"
#include "RenderTexture.hpp"

#include <array>
#include <functional>
#include <limits>

namespace evo_engine {
class EVOENGINE_API ToneMapping;
class EVOENGINE_API ScreenSpaceReflection;
class EVOENGINE_API Bloom;
class EVOENGINE_API AmbientOcclusion;
class EVOENGINE_API AntiAliasing;
class EVOENGINE_API Camera;
class EVOENGINE_API Image;
class EVOENGINE_API ImageView;
class EVOENGINE_API RenderGraphTransientResourceStore;
class EVOENGINE_API Sampler;
struct EVOENGINE_API PostProcessingCameraResources;
struct PostProcessingExecutionContext;
struct PostProcessingRendererResources;

class EVOENGINE_API PerFrameDescriptorSet {
 public:
  [[nodiscard]] std::shared_ptr<DescriptorSet> GetOrCreate(const std::shared_ptr<DescriptorSetLayout>& layout) const;
  void Retain(RenderGraphTransientResourceStore& transient_resources) const;
  void Reset();

 private:
  struct Slot {
    std::shared_ptr<DescriptorSet> descriptor_set;
    std::vector<std::shared_ptr<DescriptorSet>> duplicate_descriptor_sets;
    uint32_t frame_count = 0;
    bool recorded = false;
  };
  mutable std::vector<Slot> slots_;
};

class EVOENGINE_API PerFrameDescriptorSetList {
 public:
  [[nodiscard]] std::vector<std::shared_ptr<DescriptorSet>>& Get();
  void Retain(RenderGraphTransientResourceStore& transient_resources) const;
  void Reset();

 private:
  struct Slot {
    std::vector<std::shared_ptr<DescriptorSet>> descriptor_sets;
    std::vector<std::vector<std::shared_ptr<DescriptorSet>>> duplicate_descriptor_set_lists;
    uint32_t frame_count = 0;
    bool recorded = false;
  };
  std::vector<Slot> slots_;
};

class EVOENGINE_API PostProcessingStack : public IAsset {
  void Resize(PostProcessingCameraResources& resources, const glm::uvec2& size) const;
  bool BuildNextPipeline(PostProcessingRendererResources& resources) const;

 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  void OnCreate() override;
  void ApplyDefaultSettings();
  void Process(const std::shared_ptr<Camera>& target_camera,
               const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process = {},
               const std::shared_ptr<ImageView>& motion_vectors_image_view = {});
  void ProcessBloomAndToneMappingImmediately(const std::shared_ptr<Camera>& target_camera);
  void ProcessAmbientOcclusion(const std::shared_ptr<Camera>& target_camera,
                               const std::shared_ptr<ImageView>& ambient_occlusion_image_view,
                               const std::shared_ptr<ImageView>& scratch_image_view,
                               const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process = {});
  std::shared_ptr<AmbientOcclusion> ambient_occlusion{};
  std::shared_ptr<Bloom> bloom{};
  std::shared_ptr<ScreenSpaceReflection> screen_space_reflection{};
  std::shared_ptr<AntiAliasing> anti_aliasing{};
  std::shared_ptr<ToneMapping> tone_mapping{};

  struct BlurPushConstant {
    int horizontal = false;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
    float padding0 = 0.0f;
    float padding1 = 0.0f;
  };

  void GaussianBlur(const glm::uvec2& size, PostProcessingExecutionContext& context) const;
  void ProcessRayCamera(const std::shared_ptr<Camera>& target_camera,
                        const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process = {});

  bool enable_ambient_occlusion = true;
  bool enable_bloom = true;
  bool enable_screen_space_reflection = true;
  bool enable_anti_aliasing = true;
  bool enable_tone_mapping = true;
};

class IPostProcessing {
 public:
  virtual void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
                       PostProcessingExecutionContext& context) const = 0;
  virtual void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const = 0;
};

class EVOENGINE_API AmbientOcclusion : public IPostProcessing {
 public:
  struct BlurPushConstant {
    int horizontal = false;
    float camera_near;
    float camera_far;
    float denoise_radius;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
    float padding0 = 0.0f;
    float padding1 = 0.0f;
    float padding2 = 0.0f;
  };
  float radius = 0.4f;
  float bias = 0.02f;
  float intensity = 1.0f;
  float thickness = 1.0f;
  int slice_count = 8;
  int steps_per_slice = 6;
  float denoise_radius = 0.1f;
  struct PushConstant {
    int camera_index;
    float radius;
    float bias;
    float intensity;
    int slice_count;
    int steps_per_slice;
    float thickness;
    float padding = 0.0f;
  };
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class EVOENGINE_API AntiAliasing final : public IPostProcessing {
 public:
  enum class Preset : int32_t { Low = 0, Medium = 1, High = 2, Ultra = 3 };
  enum class DebugMode : int32_t { None = 0, Edges = 1, BlendWeights = 2 };

  struct PushConstant {
    glm::vec4 metrics = glm::vec4(1.0f);
    glm::vec4 subsample_indices = glm::vec4(0.0f);
    int32_t tone_mapped = 1;
    int32_t debug_mode = 0;
    int32_t padding0 = 0;
    int32_t padding1 = 0;
  };

  Preset preset = Preset::Ultra;
  DebugMode debug_mode = DebugMode::None;

  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void NormalizeSettings();
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

 private:
  void EnsureSmaaTargets(const glm::uvec2& size, PostProcessingCameraResources& resources) const;
  void EnsureSmaaLookupTextures(PostProcessingRendererResources& resources) const;
};

class EVOENGINE_API ScreenSpaceReflection : public IPostProcessing {
 public:
  enum class DebugMode : int32_t { None = 0, HitUv = 1, RayDistance = 2, RejectionReason = 3, Confidence = 4 };

  float max_distance = 100.f;
  float distance_confidence = 0.2f;
  int max_iteration_count = 128;
  int binary_search_iteration_count = 8;
  float thickness = 0.5f;
  float start_bias = 0.05f;
  bool blur = true;
  bool temporal_stabilization = true;
  DebugMode debug_mode = DebugMode::None;

  struct PushConstant {
    int32_t camera_index = 0;
    float max_distance;
    float distance_confidence;
    int max_iteration_count = 5;
    int binary_search_iteration_count;
    float thickness;
    float start_bias;
    int32_t debug_mode;
    int32_t temporal_enabled;
    int32_t temporal_history_valid;
  };

  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class EVOENGINE_API Bloom : public IPostProcessing {
 public:
  struct PrefilterPushConstant {
    glm::uvec2 source_resolution = glm::uvec2(1);
    glm::uvec2 target_resolution = glm::uvec2(1);
    float threshold = 1.0f;
    float knee = 0.1f;
    float compression_start = 2.0f;
    float source_ceiling = 8.0f;
  };

  struct DownsamplingPushConstant {
    glm::uvec2 source_resolution = glm::uvec2(1);
    glm::uvec2 target_resolution = glm::uvec2(1);
    int apply_karis = 0;
    glm::ivec3 padding = glm::ivec3(0);
  };

  struct UpsamplingPushConstant {
    glm::uvec2 source_resolution = glm::uvec2(1);
    glm::uvec2 target_resolution = glm::uvec2(1);
    float filter_radius = 1.0f;
    glm::vec3 padding = glm::vec3(0.0f);
  };

  struct MixPushConstant {
    glm::uvec2 resolution = glm::uvec2(1);
    glm::uvec2 bloom_resolution = glm::uvec2(1);
    float intensity = 1.0f;
    glm::vec3 padding = glm::vec3(0.0f);
  };

  float filter_radius = 1.0f;
  float threshold = 1.0f;
  float knee = 0.1f;
  float intensity = 0.05f;
  float compression_start = 2.0f;
  float source_ceiling = 8.0f;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class EVOENGINE_API ToneMapping : public IPostProcessing {
 public:
  enum class ToneMapMethod : int32_t {
    Filmic = 0,
    Uncharted2 = 1,
    Clip = 2,
    Aces = 3,
    Agx = 4,
    KhronosPbr = 5,
    EvoEngineExponential = 6,
  };

  struct PushConstant {
    int32_t camera_index = 0;
    int32_t method = static_cast<int32_t>(ToneMapMethod::Filmic);
    int32_t is_active = 1;
    int32_t auto_exposure = 0;
    int32_t enable_center_metering = 0;
    int32_t average_mode = 1;
    int32_t dither = 1;
    float exposure = 2.0f;
    float brightness = 1.0f;
    float contrast = 1.0f;
    float saturation = 1.0f;
    float vignette = 0.0f;
    float auto_exposure_speed = 10.0f;
    float ev_min_value = -20.0f;
    float ev_max_value = 20.0f;
    float center_metering_size = 0.5f;
  };

  ToneMapMethod method = ToneMapMethod::Filmic;
  float exposure = 1.f;
  float brightness = 1.f;
  float contrast = 1.f;
  float saturation = 1.f;
  float vignette = 0.f;
  bool auto_exposure = true;
  float auto_exposure_speed = 10.f;
  float ev_min_value = -20.f;
  float ev_max_value = 20.f;
  bool enable_center_metering = false;
  float center_metering_size = 0.5f;
  int average_mode = 1;
  bool dither = true;
  float auto_exposure_delta_time_override = -1.0f;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

struct EVOENGINE_API PostProcessingCameraResources {
  struct StackResources {
    glm::uvec2 size = glm::uvec2(0);
    uint64_t generation = 0;
    std::shared_ptr<RenderTexture> source_color_texture;
    std::shared_ptr<RenderTexture> result_texture;
    std::shared_ptr<RenderTexture> swap_texture;
    PerFrameDescriptorSet blur_horizontal_descriptor_set;
    PerFrameDescriptorSet blur_vertical_descriptor_set;
  } stack;

  struct AmbientOcclusionResources {
    PerFrameDescriptorSet blur_horizontal_descriptor_set;
    PerFrameDescriptorSet blur_vertical_descriptor_set;
    PerFrameDescriptorSet geometry_output_descriptor_set;
  } ambient_occlusion;

  struct AntiAliasingResources {
    glm::uvec2 smaa_size = glm::uvec2(0);
    std::shared_ptr<RenderTexture> smaa_edges_texture;
    std::shared_ptr<RenderTexture> smaa_blend_texture;
    PerFrameDescriptorSet smaa_prepare_descriptor_set;
    PerFrameDescriptorSet smaa_edge_descriptor_set;
    PerFrameDescriptorSet smaa_weight_descriptor_set;
    PerFrameDescriptorSet smaa_neighborhood_descriptor_set;
  } anti_aliasing;

  struct ScreenSpaceReflectionResources {
    glm::uvec2 history_size = glm::uvec2(0);
    std::array<std::shared_ptr<RenderTexture>, 2> reflection_history{};
    std::array<std::shared_ptr<RenderTexture>, 2> geometry_history{};
    std::array<std::shared_ptr<RenderTexture>, 2> material_history{};
    uint32_t history_read_index = 0;
    bool history_valid = false;
    PerFrameDescriptorSet combine_descriptor_set;
    PerFrameDescriptorSet reflect_output_descriptor_set;
    PerFrameDescriptorSet spatial_resolve_descriptor_set;
    PerFrameDescriptorSet temporal_descriptor_set;
  } screen_space_reflection;

  struct BloomResources {
    glm::uvec2 size = glm::uvec2(0);
    VkFormat format = VK_FORMAT_UNDEFINED;
    std::shared_ptr<RenderTexture> downsample_texture_a;
    std::shared_ptr<RenderTexture> downsample_texture_b;
    std::shared_ptr<RenderTexture> upsample_texture;
    PerFrameDescriptorSet mix_descriptor_set;
    PerFrameDescriptorSet copy_descriptor_set;
    PerFrameDescriptorSetList downsampling_descriptor_sets;
    PerFrameDescriptorSetList upsampling_descriptor_sets;
  } bloom;

  struct ToneMappingResources {
    PerFrameDescriptorSet auto_exposure_descriptor_set;
    std::shared_ptr<Buffer> histogram_buffer;
    std::shared_ptr<Buffer> luminance_buffer;
    bool auto_exposure_time_initialized = false;
    bool luminance_reset_pending = true;
    double last_auto_exposure_time = 0.0;
    uint64_t auto_exposure_process_count = 0;
    uint64_t auto_exposure_reset_count = 0;
  } tone_mapping;

  uint64_t stack_handle = 0;
  uint32_t stack_version = (std::numeric_limits<uint32_t>::max)();
  uint32_t render_technique = (std::numeric_limits<uint32_t>::max)();
  glm::uvec2 observed_resolution = glm::uvec2(0);
  uint64_t temporal_reset_count = 0;
  uint64_t version_reset_count = 0;
  uint64_t resolution_reset_count = 0;
  uint64_t technique_reset_count = 0;
  glm::mat4 previous_projection_view = glm::mat4(1.0f);
  glm::mat4 previous_inverse_projection = glm::mat4(1.0f);
  glm::mat4 previous_inverse_view = glm::mat4(1.0f);
  glm::mat4 previous_unjittered_projection_view = glm::mat4(1.0f);
  bool previous_matrices_valid = false;

  void ResetTemporalState();
  void Retain(RenderGraphTransientResourceStore& transient_resources) const;
};

struct PostProcessingRendererResources {
  size_t pipeline_build_step = 0;
  bool pipelines_ready = false;

  struct StackResources {
    std::shared_ptr<DescriptorSetLayout> blur_layout;
    std::shared_ptr<ComputePipeline> blur_pipeline;
  } stack;

  struct AmbientOcclusionResources {
    std::shared_ptr<DescriptorSetLayout> blur_layout;
    std::shared_ptr<ComputePipeline> blur_pipeline;
    std::shared_ptr<DescriptorSetLayout> geometry_output_layout;
    std::shared_ptr<ComputePipeline> geometry_pipeline;
    std::shared_ptr<Sampler> sampler;
  } ambient_occlusion;

  struct AntiAliasingResources {
    std::shared_ptr<Image> smaa_area_image;
    std::shared_ptr<Image> smaa_search_image;
    std::shared_ptr<ImageView> smaa_area_view;
    std::shared_ptr<ImageView> smaa_search_view;
    std::shared_ptr<Sampler> smaa_lookup_sampler;
    std::shared_ptr<DescriptorSetLayout> smaa_prepare_layout;
    std::shared_ptr<DescriptorSetLayout> smaa_edge_layout;
    std::shared_ptr<DescriptorSetLayout> smaa_weight_layout;
    std::shared_ptr<DescriptorSetLayout> smaa_neighborhood_layout;
    std::shared_ptr<ComputePipeline> smaa_prepare_pipeline;
    std::array<std::shared_ptr<GraphicsPipeline>, 4> smaa_edge_pipelines{};
    std::array<std::shared_ptr<GraphicsPipeline>, 4> smaa_weight_pipelines{};
    std::shared_ptr<GraphicsPipeline> smaa_neighborhood_pipeline;
  } anti_aliasing;

  struct ScreenSpaceReflectionResources {
    std::shared_ptr<DescriptorSetLayout> combine_layout;
    std::shared_ptr<DescriptorSetLayout> reflect_output_layout;
    std::shared_ptr<DescriptorSetLayout> spatial_resolve_layout;
    std::shared_ptr<DescriptorSetLayout> temporal_layout;
    std::shared_ptr<ComputePipeline> reflect_pipeline;
    std::shared_ptr<ComputePipeline> spatial_resolve_pipeline;
    std::shared_ptr<ComputePipeline> temporal_pipeline;
    std::shared_ptr<ComputePipeline> combine_pipeline;
  } screen_space_reflection;

  struct BloomResources {
    VkFormat format = VK_FORMAT_UNDEFINED;
    std::shared_ptr<DescriptorSetLayout> mix_layout;
    std::shared_ptr<DescriptorSetLayout> copy_layout;
    std::shared_ptr<DescriptorSetLayout> sampling_layout;
    std::shared_ptr<DescriptorSetLayout> upsampling_layout;
    std::shared_ptr<ComputePipeline> downsampling_pipeline;
    std::shared_ptr<ComputePipeline> upsampling_pipeline;
    std::shared_ptr<ComputePipeline> copy_pipeline;
    std::shared_ptr<ComputePipeline> mix_pipeline;
  } bloom;

  struct ToneMappingResources {
    std::shared_ptr<DescriptorSetLayout> auto_exposure_layout;
    std::shared_ptr<ComputePipeline> histogram_pipeline;
    std::shared_ptr<ComputePipeline> auto_exposure_pipeline;
    std::shared_ptr<ComputePipeline> pipeline;
  } tone_mapping;
};

struct PostProcessingExecutionContext {
  PostProcessingCameraResources& camera;
  PostProcessingRendererResources& renderer;
  std::shared_ptr<ImageView> ambient_occlusion_image_view;
  std::shared_ptr<ImageView> ambient_occlusion_scratch_image_view;
  std::shared_ptr<ImageView> motion_vectors_image_view;
  std::function<void(const std::function<void(VkCommandBuffer)>&)> record_commands;
};

}  // namespace evo_engine
