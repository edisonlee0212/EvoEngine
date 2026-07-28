#pragma once
#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "IAsset.hpp"
#include "RenderTexture.hpp"

#include <array>
#include <functional>
#include <limits>

namespace evo_engine {
class ToneMapping;
class ScreenSpaceReflection;
class Bloom;
class AmbientOcclusion;
class AntiAliasing;
class Camera;
class Image;
class ImageView;
class RenderGraphTransientResourceStore;
class Sampler;
struct PostProcessingCameraResources;
struct PostProcessingExecutionContext;
struct PostProcessingRendererResources;

class PerFrameDescriptorSet {
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

class PerFrameDescriptorSetList {
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

class PostProcessingStack : public IAsset {
  void Resize(PostProcessingCameraResources& resources, const glm::uvec2& size) const;
  bool BuildNextPipeline(PostProcessingRendererResources& resources) const;

 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  void OnCreate() override;
  void Process(const std::shared_ptr<Camera>& target_camera,
               const std::shared_ptr<ImageView>& motion_vectors_image_view = {},
               const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process = {});
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
  bool enable_bloom = false;
  bool enable_screen_space_reflection = false;
  bool enable_anti_aliasing = true;
  bool enable_tone_mapping = true;
};

class IPostProcessing {
 public:
  virtual void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
                       PostProcessingExecutionContext& context) const = 0;
  virtual void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const = 0;
};

class AmbientOcclusion : public IPostProcessing {
 public:
  enum class Algorithm : int32_t { Ssao = 0, Gtao = 1 };
  struct BlurPushConstant {
    int horizontal = false;
    float camera_near;
    float camera_far;
    float avoid_distance;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
    float padding0 = 0.0f;
    float padding1 = 0.0f;
    float padding2 = 0.0f;
  };
  float avoid_distance = 0.1f;
  /**
   * \brief Parameters (you'd probably want to use them as uniforms to more easily tweak the effect)
   */
  Algorithm algorithm = Algorithm::Gtao;
  int kernel_size = 64;
  float radius = 0.15f;
  float bias = 0.01f;
  float factor = 0.0f;
  float intensity = 1.0f;
  float gtao_radius = 0.4f;
  float gtao_bias = 0.02f;
  float gtao_intensity = 1.0f;
  float thickness = 1.0f;
  int slice_count = 8;
  int steps_per_slice = 6;
  float denoise_radius = 0.1f;
  struct PushConstant {
    int camera_index;
    // parameters (you'd probably want to use them as uniforms to more easily tweak the effect)
    int kernel_size;
    float radius;
    float bias;
    float factor;
    float intensity;
    int algorithm;
    int slice_count;
    int steps_per_slice;
    float thickness;
  };
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class AntiAliasing final : public IPostProcessing {
 public:
  enum class Algorithm : int32_t { Taa = 0, Smaa = 1 };

  enum class TaaPreset : int32_t {
    BestQuality = 0,
    HighQuality = 1,
    Performance = 2,
    Custom = 3,
  };

  enum class VarianceClippingMode : int32_t {
    Disabled = 0,
    Clamp = 1,
    Intersection = 2,
  };

  enum class HistoryColorMode : int32_t {
    ToneMapped = 0,
    Linear = 1,
  };

  enum class TaaDebugMode : int32_t {
    None = 0,
    Motion = 1,
    DepthConfidence = 2,
    HistoryConfidence = 3,
    NoHistory = 4,
  };

  enum class SmaaPreset : int32_t { Low = 0, Medium = 1, High = 2, Ultra = 3 };
  enum class SmaaDebugMode : int32_t { None = 0, Edges = 1, BlendWeights = 2 };

  struct TaaSettings {
    TaaPreset preset = TaaPreset::BestQuality;
    VarianceClippingMode variance_clipping_mode = VarianceClippingMode::Intersection;
    HistoryColorMode history_color_mode = HistoryColorMode::ToneMapped;
    int variance_sample_count = 9;
    int longest_velocity_sample_count = 9;
    bool use_ycocg = true;
    bool use_neighborhood_sampling = true;
    bool use_bicubic_filter = true;
    bool use_longest_velocity = true;
    bool use_depth_threshold = true;
    bool use_tgsm = true;
    bool use_fp16 = false;
    float min_variance_gamma = 0.75f;
    float max_variance_gamma = 2.0f;
    float velocity_rejection_threshold = 128.0f;
    float depth_threshold = 0.002f;
    float sharpen = 0.0f;
    TaaDebugMode debug_mode = TaaDebugMode::None;
  };

  struct SmaaSettings {
    SmaaPreset preset = SmaaPreset::Ultra;
    SmaaDebugMode debug_mode = SmaaDebugMode::None;
  };

  struct TaaPushConstant {
    int32_t camera_index = 0;
    int32_t history_valid = 0;
    int32_t frame_index = 0;
    int32_t variance_clipping_mode = static_cast<int32_t>(VarianceClippingMode::Intersection);
    int32_t variance_sample_count = 9;
    int32_t use_ycocg = 1;
    int32_t use_neighborhood_sampling = 1;
    int32_t use_bicubic_filter = 1;
    int32_t use_longest_velocity = 1;
    int32_t longest_velocity_sample_count = 9;
    int32_t use_depth_threshold = 1;
    int32_t history_color_mode = static_cast<int32_t>(HistoryColorMode::ToneMapped);
    int32_t debug_mode = 0;
    int32_t padding0 = 0;
    int32_t padding1 = 0;
    int32_t padding2 = 0;
    float min_variance_gamma = 0.75f;
    float max_variance_gamma = 2.0f;
    float velocity_rejection_threshold = 128.0f;
    float depth_threshold = 0.002f;
    float sharpen = 0.0f;
    float padding3 = 0.0f;
    float padding4 = 0.0f;
    float padding5 = 0.0f;
  };

  struct SmaaPushConstant {
    glm::vec4 metrics = glm::vec4(1.0f);
    glm::vec4 subsample_indices = glm::vec4(0.0f);
    int32_t tone_mapped = 1;
    int32_t debug_mode = 0;
    int32_t padding0 = 0;
    int32_t padding1 = 0;
  };

  Algorithm algorithm = Algorithm::Smaa;
  TaaSettings taa{};
  SmaaSettings smaa{};

  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void ApplyTaaPreset(TaaPreset value);
  void NormalizeSettings();
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

 private:
  void ProcessTaa(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
                  PostProcessingExecutionContext& context) const;
  void ProcessSmaa(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
                   PostProcessingExecutionContext& context) const;
  void BuildTaaPipelines(PostProcessingRendererResources& resources, bool force_rebuild) const;
  void BuildSmaaPipelines(PostProcessingRendererResources& resources, bool force_rebuild) const;
  void EnsureSmaaTargets(const glm::uvec2& size, PostProcessingCameraResources& resources) const;
  void EnsureSmaaLookupTextures(PostProcessingRendererResources& resources) const;
};

class ScreenSpaceReflection : public IPostProcessing {
 public:
  float max_distance = 100.f;
  float distance_confidence = 0.2f;
  int max_iteration_count = 128;
  int initial_steps = 32;
  float thickness = 0.5f;
  bool blur = true;

  struct PushConstant {
    int32_t camera_index = 0;
    float max_distance;
    float distance_confidence;
    int max_iteration_count = 5;
    int initial_steps;
    float thickness;
  };

  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class Bloom : public IPostProcessing {
 public:
  struct DownsamplingPushConstant {
    glm::vec2 source_resolution;
    int mip_level;
    int padding;
  };

  struct UpsamplingPushConstant {
    glm::uvec2 target_resolution = glm::uvec2(1);
    float filter_radius = 0.001f;
    float padding = 0.0f;
  };

  struct ComputePushConstant {
    glm::uvec2 resolution = glm::uvec2(1);
  };

  float filter_radius = 0.001f;
  int bloom_chain_length = 2;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera,
               PostProcessingExecutionContext& context) const override;
  void BuildPipelines(PostProcessingRendererResources& resources, bool force_rebuild = false) const override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class ToneMapping : public IPostProcessing {
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
    float auto_exposure_speed = 0.0f;
    float ev_min_value = -5.0f;
    float ev_max_value = 10.0f;
    float center_metering_size = 0.5f;
  };

  ToneMapMethod method = ToneMapMethod::Filmic;
  float exposure = 1.f;
  float brightness = 1.f;
  float contrast = 1.f;
  float saturation = 1.f;
  float vignette = 0.f;
  bool auto_exposure = true;
  float auto_exposure_speed = 5.f;
  float ev_min_value = -5.f;
  float ev_max_value = 10.f;
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

struct PostProcessingCameraResources {
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
    struct HistoryResources {
      std::shared_ptr<RenderTexture> textures[2];
      std::shared_ptr<RenderTexture> depth_textures[2];
      glm::uvec2 size = glm::uvec2(0);
      uint32_t frame_index = 0;
      uint32_t last_processed_frame = 0;
      uint32_t camera_history_version = 0;
      bool valid = false;
    } history;

    PerFrameDescriptorSet copy_descriptor_set;
    PerFrameDescriptorSet resolve_descriptor_set;
    glm::uvec2 smaa_size = glm::uvec2(0);
    std::shared_ptr<RenderTexture> smaa_edges_texture;
    std::shared_ptr<RenderTexture> smaa_blend_texture;
    PerFrameDescriptorSet smaa_prepare_descriptor_set;
    PerFrameDescriptorSet smaa_edge_descriptor_set;
    PerFrameDescriptorSet smaa_weight_descriptor_set;
    PerFrameDescriptorSet smaa_neighborhood_descriptor_set;
  } anti_aliasing;

  struct ScreenSpaceReflectionResources {
    PerFrameDescriptorSet combine_descriptor_set;
    PerFrameDescriptorSet reflect_output_descriptor_set;
  } screen_space_reflection;

  struct BloomResources {
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
  glm::vec2 current_jitter = {};
  glm::vec2 previous_jitter = {};
  uint32_t jitter_frame_index = 0;
  bool jitter_taa_enabled = false;
  glm::mat4 previous_projection_view = glm::mat4(1.0f);
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
    std::shared_ptr<DescriptorSetLayout> copy_layout;
    std::shared_ptr<DescriptorSetLayout> resolve_layout;
    std::shared_ptr<ComputePipeline> copy_pipeline;
    std::array<std::shared_ptr<ComputePipeline>, 4> resolve_pipelines{};
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
    std::shared_ptr<ComputePipeline> reflect_pipeline;
    std::shared_ptr<ComputePipeline> combine_pipeline;
  } screen_space_reflection;

  struct BloomResources {
    std::shared_ptr<DescriptorSetLayout> mix_layout;
    std::shared_ptr<DescriptorSetLayout> copy_layout;
    std::shared_ptr<DescriptorSetLayout> sampling_layout;
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
  std::shared_ptr<ImageView> motion_vectors_image_view;
  std::shared_ptr<ImageView> ambient_occlusion_image_view;
  std::shared_ptr<ImageView> ambient_occlusion_scratch_image_view;
};

}  // namespace evo_engine
