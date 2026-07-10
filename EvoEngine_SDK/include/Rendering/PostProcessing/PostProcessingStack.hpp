#pragma once
#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "IAsset.hpp"
#include "RenderTexture.hpp"

#include <functional>
#include <unordered_map>

namespace evo_engine {
class ToneMapping;
class ScreenSpaceReflection;
class Bloom;
class AmbientOcclusion;
class TemporalAntiAliasing;
class Camera;
class ImageView;

class PostProcessingStack : public IAsset {
  glm::uvec2 current_size = glm::uvec2(1);
  size_t pipeline_build_step_ = 0;
  bool pipelines_ready_ = false;
  void Resize(const glm::uvec2& size);
  bool BuildNextPipeline();

  mutable std::shared_ptr<DescriptorSetLayout> blur_layout;
  mutable std::shared_ptr<ComputePipeline> blur_pipeline;

  std::shared_ptr<DescriptorSet> blur_horizontal_descriptor_set;  // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  std::shared_ptr<DescriptorSet> blur_vertical_descriptor_set;    // RENDER_TEXTURE_PRESENT_LAYOUT: 0
 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  std::shared_ptr<RenderTexture> source_color_texture;
  std::shared_ptr<RenderTexture> result_texture;
  std::shared_ptr<RenderTexture> swap_texture;
  std::shared_ptr<ImageView> motion_vectors_image_view;

  void OnCreate() override;
  void Process(const std::shared_ptr<Camera>& target_camera,
               const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process = {});
  std::shared_ptr<AmbientOcclusion> ambient_occlusion{};
  std::shared_ptr<Bloom> bloom{};
  std::shared_ptr<ScreenSpaceReflection> screen_space_reflection{};
  std::shared_ptr<TemporalAntiAliasing> temporal_anti_aliasing{};
  std::shared_ptr<ToneMapping> tone_mapping{};

  void GaussianBlur(const glm::uvec2& size) const;
  void ProcessRayCamera(const std::shared_ptr<Camera>& target_camera,
                        const std::function<void(VkCommandBuffer vk_command_buffer)>& pre_process = {});

  bool enable_ambient_occlusion = true;
  bool enable_bloom = false;
  bool enable_screen_space_reflection = false;
  bool enable_temporal_anti_aliasing = true;
  bool enable_tone_mapping = true;
};

class IPostProcessing {
 public:
  virtual void Process(const PostProcessingStack& post_processing_stack,
                       const std::shared_ptr<Camera>& target_camera) = 0;
  virtual void BuildPipelines(bool force_rebuild = false) = 0;
};

class AmbientOcclusion : public IPostProcessing {
 public:
  enum class Algorithm : int32_t { Ssao = 0, Gtao = 1 };

  std::shared_ptr<DescriptorSetLayout> blur_layout;
  std::shared_ptr<ComputePipeline> blur_pipeline;

  std::shared_ptr<DescriptorSet> blur_horizontal_descriptor_set;  // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  std::shared_ptr<DescriptorSet> blur_vertical_descriptor_set;    // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  struct BlurPushConstant {
    int horizontal = false;
    float camera_near;
    float camera_far;
    float avoid_distance;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
  };
  float avoid_distance = 0.1f;
  std::shared_ptr<DescriptorSetLayout> combine_layout;
  std::shared_ptr<DescriptorSet> combine_descriptor_set;
  /**
   * \brief Parameters (you'd probably want to use them as uniforms to more easily tweak the effect)
   */
  Algorithm algorithm = Algorithm::Gtao;
  int kernel_size = 64;
  float radius = 0.15f;
  float bias = 0.01f;
  float factor = 0.0f;
  float intensity = 2.0f;
  float gtao_radius = 0.4f;
  float gtao_bias = 0.02f;
  float gtao_intensity = 1.5f;
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
  std::shared_ptr<DescriptorSetLayout> geometry_output_layout;
  std::shared_ptr<DescriptorSet> geometry_output_descriptor_set;
  std::shared_ptr<ComputePipeline> geometry_pipeline;
  std::shared_ptr<ComputePipeline> combine_pipeline;

  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines(bool force_rebuild = false) override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class TemporalAntiAliasing : public IPostProcessing {
 public:
  enum class Preset : int32_t {
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

  enum class DebugMode : int32_t {
    None = 0,
    Motion = 1,
    DepthConfidence = 2,
    HistoryConfidence = 3,
    NoHistory = 4,
  };

  struct PushConstant {
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

  Preset preset = Preset::BestQuality;
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
  DebugMode debug_mode = DebugMode::None;
  bool reset_history = false;

  std::shared_ptr<DescriptorSetLayout> copy_layout;
  std::shared_ptr<DescriptorSetLayout> resolve_layout;
  std::shared_ptr<DescriptorSet> copy_descriptor_set;
  std::shared_ptr<DescriptorSet> resolve_descriptor_set;
  std::shared_ptr<ComputePipeline> copy_pipeline;
  std::shared_ptr<ComputePipeline> resolve_pipeline;

  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void ApplyPreset(Preset value);
  void NormalizeSettings();
  void ResetHistory(const std::shared_ptr<Camera>& target_camera = nullptr);
  void BuildPipelines(bool force_rebuild = false) override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

 private:
  struct HistoryResources {
    std::shared_ptr<RenderTexture> textures[2];
    std::shared_ptr<RenderTexture> depth_textures[2];
    glm::uvec2 size = glm::uvec2(0);
    uint32_t frame_index = 0;
    uint32_t last_processed_frame = 0;
    uint32_t camera_history_version = 0;
    size_t settings_hash = 0;
    bool valid = false;
  };

  [[nodiscard]] size_t ComputeSettingsHash() const;
  void PruneHistory(uint32_t current_frame_index, uint64_t active_camera_handle);

  std::unordered_map<uint64_t, HistoryResources> history_resources_;
  bool built_use_tgsm_ = false;
  bool built_use_fp16_ = false;
  bool resolve_pipeline_configuration_valid_ = false;
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

  std::shared_ptr<DescriptorSetLayout> combine_layout;
  std::shared_ptr<DescriptorSetLayout> reflect_output_layout;
  std::shared_ptr<ComputePipeline> reflect_pipeline;
  std::shared_ptr<ComputePipeline> combine_pipeline;
  std::shared_ptr<DescriptorSet> combine_descriptor_set;  // SSR_COMBINE: 0, 1
  std::shared_ptr<DescriptorSet> reflect_output_descriptor_set;

  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines(bool force_rebuild = false) override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class Bloom : public IPostProcessing {
 public:
  std::shared_ptr<DescriptorSetLayout> mix_layout;
  std::shared_ptr<DescriptorSet> mix_descriptor_set;
  std::shared_ptr<DescriptorSetLayout> copy_layout;
  std::shared_ptr<DescriptorSet> copy_descriptor_set;

  std::shared_ptr<DescriptorSetLayout> sampling_layout;
  std::shared_ptr<ComputePipeline> downsampling_pipeline;
  std::shared_ptr<ComputePipeline> upsampling_pipeline;

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
  std::vector<std::shared_ptr<DescriptorSet>> downsampling_descriptor_set;
  std::vector<std::shared_ptr<DescriptorSet>> upsampling_descriptor_set;
  std::shared_ptr<ComputePipeline> copy_pipeline;
  std::shared_ptr<ComputePipeline> mix_pipeline;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines(bool force_rebuild = false) override;
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
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines(bool force_rebuild = false) override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  std::shared_ptr<DescriptorSetLayout> auto_exposure_layout;
  std::shared_ptr<DescriptorSet> auto_exposure_descriptor_set;
  std::shared_ptr<Buffer> histogram_buffer;
  std::shared_ptr<Buffer> luminance_buffer;
  std::shared_ptr<ComputePipeline> histogram_pipeline;
  std::shared_ptr<ComputePipeline> auto_exposure_pipeline;
  std::shared_ptr<ComputePipeline> pipeline;

 private:
  bool auto_exposure_time_initialized_ = false;
  double last_auto_exposure_time_ = 0.0;
};

}  // namespace evo_engine
