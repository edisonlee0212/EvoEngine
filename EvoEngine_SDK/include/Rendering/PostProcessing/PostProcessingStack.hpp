#pragma once
#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"
#include "IAsset.hpp"
#include "RenderTexture.hpp"
namespace evo_engine {
class ToneMapping;
class ScreenSpaceReflection;
class Bloom;
class ScreenSpaceAmbientOcclusion;
class Camera;

class PostProcessingStack : public IAsset {
  glm::uvec2 current_size = glm::uvec2(1);
  void Resize(const glm::uvec2& size);

  mutable std::shared_ptr<DescriptorSetLayout> blur_layout;
  mutable std::shared_ptr<GraphicsPipeline> blur_pipeline;

  std::shared_ptr<DescriptorSet> blur_horizontal_descriptor_set;  // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  std::shared_ptr<DescriptorSet> blur_vertical_descriptor_set;    // RENDER_TEXTURE_PRESENT_LAYOUT: 0
 public:
  [[nodiscard]] bool SupportsStagedLoading() const override {
    return true;
  }

  std::shared_ptr<RenderTexture> source_color_texture;
  std::shared_ptr<RenderTexture> result_texture;
  std::shared_ptr<RenderTexture> swap_texture;

  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const std::shared_ptr<Camera>& target_camera);
  std::shared_ptr<ScreenSpaceAmbientOcclusion> screen_space_ambient_occlusion{};
  std::shared_ptr<Bloom> bloom{};
  std::shared_ptr<ScreenSpaceReflection> screen_space_reflection{};
  std::shared_ptr<ToneMapping> tone_mapping{};

  void GaussianBlur(const glm::uvec2& size) const;

  bool enable_screen_space_ambient_occlusion = true;
  bool enable_bloom = true;
  bool enable_screen_space_reflection = true;
  bool enable_tone_mapping = true;
};

class IPostProcessing {
 public:
  virtual void Process(const PostProcessingStack& post_processing_stack,
                       const std::shared_ptr<Camera>& target_camera) = 0;
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }

  virtual void BuildPipelines(bool force_rebuild = false) = 0;
};

class ScreenSpaceAmbientOcclusion : public IPostProcessing {
 public:
  std::shared_ptr<DescriptorSetLayout> blur_layout;
  std::shared_ptr<GraphicsPipeline> blur_pipeline;

  std::shared_ptr<DescriptorSet> blur_horizontal_descriptor_set;  // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  std::shared_ptr<DescriptorSet> blur_vertical_descriptor_set;    // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  struct BlurPushConstant {
    int horizontal = false;
    float camera_near;
    float camera_far;
    float avoid_distance;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
  };
  float avoid_distance = 1.f;
  std::shared_ptr<DescriptorSetLayout> combine_layout;
  std::shared_ptr<DescriptorSet> combine_descriptor_set;
  std::shared_ptr<GraphicsPipeline> geometry_pipeline;
  std::shared_ptr<GraphicsPipeline> combine_pipeline;
  /**
   * \brief Parameters (you'd probably want to use them as uniforms to more easily tweak the effect)
   */
  int kernel_size = 64;
  float radius = 0.15f;
  float bias = 0.001f;
  float factor = 0.0f;
  float intensity = 1.0f;
  struct PushConstant {
    int camera_index;
    // parameters (you'd probably want to use them as uniforms to more easily tweak the effect)
    int kernel_size;
    float radius;
    float bias;
    float factor;
    float intensity;
  };

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines(bool force_rebuild = false) override;
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
  std::shared_ptr<GraphicsPipeline> reflect_pipeline;
  std::shared_ptr<GraphicsPipeline> combine_pipeline;
  std::shared_ptr<DescriptorSet> combine_descriptor_set;  // SSR_COMBINE: 0, 1

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines(bool force_rebuild = false) override;
};

class Bloom : public IPostProcessing {
 public:
  std::shared_ptr<DescriptorSetLayout> mix_layout;
  std::shared_ptr<DescriptorSet> mix_descriptor_set;

  std::shared_ptr<DescriptorSetLayout> sampling_layout;
  std::shared_ptr<GraphicsPipeline> downsampling_pipeline;
  std::shared_ptr<GraphicsPipeline> upsampling_pipeline;

  struct DownsamplingPushConstant {
    glm::vec2 source_resolution;
    int mip_level;
    int padding;
  };

  struct UpsamplingPushConstant {
    float filter_radius;
  };

  float filter_radius = 0.001f;
  int bloom_chain_length = 2;
  std::vector<std::shared_ptr<DescriptorSet>> downsampling_descriptor_set;
  std::vector<std::shared_ptr<DescriptorSet>> upsampling_descriptor_set;
  std::shared_ptr<GraphicsPipeline> copy_pipeline;
  std::shared_ptr<GraphicsPipeline> mix_pipeline;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines(bool force_rebuild = false) override;
};

class ToneMapping : public IPostProcessing {
 public:
  struct PushConstant {
    int32_t camera_index = 0;
    float exposure;
    float gamma;
  };

  float exposure = 2.f;
  float gamma = 1.f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines(bool force_rebuild = false) override;
  std::shared_ptr<ComputePipeline> pipeline;
};

}  // namespace evo_engine
