#pragma once
#include "GraphicsPipeline.hpp"
#include "IAsset.hpp"
#include "RenderTexture.hpp"
namespace evo_engine {
class ScreenSpaceReflection;
class Bloom;
class ScreenSpaceAmbientOcclusion;
class Camera;

class PostProcessingStack : public IAsset {
  friend class Camera;

  void Resize(const glm::uvec2& size) const;

  inline static std::shared_ptr<DescriptorSetLayout> blur_layout;
  inline static std::shared_ptr<GraphicsPipeline> blur_pipeline;

  std::shared_ptr<DescriptorSet> blur_horizontal_descriptor_set;  // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  std::shared_ptr<DescriptorSet> blur_vertical_descriptor_set;    // RENDER_TEXTURE_PRESENT_LAYOUT: 0
 public:
  std::shared_ptr<RenderTexture> source_color_texture;
  std::shared_ptr<RenderTexture> result_texture;
  std::shared_ptr<RenderTexture> swap_texture;

  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const std::shared_ptr<Camera>& target_camera) const;
  std::shared_ptr<ScreenSpaceAmbientOcclusion> screen_space_ambient_occlusion{};
  std::shared_ptr<Bloom> bloom{};
  std::shared_ptr<ScreenSpaceReflection> screen_space_reflection{};

  void GaussianBlur(const glm::uvec2& size) const;

  bool enable_screen_space_ambient_occlusion = true;
  bool enable_bloom = true;
  bool enable_screen_space_reflection = true;
};

class IPostProcessing {
 public:
  virtual void Process(const PostProcessingStack& post_processing_stack,
                       const std::shared_ptr<Camera>& target_camera) = 0;
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }

  virtual void BuildPipelines() = 0;
};

class ScreenSpaceAmbientOcclusion : public IPostProcessing {
 public:
  inline static std::shared_ptr<DescriptorSetLayout> blur_layout;
  inline static std::shared_ptr<GraphicsPipeline> blur_pipeline;

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
  inline static std::shared_ptr<DescriptorSetLayout> combine_layout;
  std::shared_ptr<DescriptorSet> combine_descriptor_set;
  inline static std::shared_ptr<GraphicsPipeline> geometry_pipeline;
  inline static std::shared_ptr<GraphicsPipeline> combine_pipeline;
  /**
   * \brief Parameters (you'd probably want to use them as uniforms to more easily tweak the effect)
   */
  int kernel_size = 32;
  float radius = 0.5f;
  float bias = 0.01f;
  float factor = 1.0f;
  float intensity = 3.0f;
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
  void BuildPipelines() override;
};

class ScreenSpaceReflection : public IPostProcessing {
 public:
  float max_distance = 10.f;
  float resolution = 16.f;
  int max_iteration_count = 64;
  int initial_steps = 8;
  float thickness = 0.3f;
  bool blur = true;

  struct PushConstant {
    int32_t camera_index = 0;
    float max_distance;
    float resolution;
    int max_iteration_count = 5;
    int initial_steps;
    float thickness;
  };

  inline static std::shared_ptr<DescriptorSetLayout> combine_layout;
  inline static std::shared_ptr<GraphicsPipeline> reflect_pipeline;
  inline static std::shared_ptr<GraphicsPipeline> combine_pipeline;
  std::shared_ptr<DescriptorSet> combine_descriptor_set;  // SSR_COMBINE: 0, 1

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines() override;
};

class Bloom : public IPostProcessing {
 public:
  inline static std::shared_ptr<DescriptorSetLayout> mix_layout;
  std::shared_ptr<DescriptorSet> mix_descriptor_set;

  inline static std::shared_ptr<DescriptorSetLayout> sampling_layout;
  inline static std::shared_ptr<GraphicsPipeline> downsampling_pipeline;
  inline static std::shared_ptr<GraphicsPipeline> upsampling_pipeline;

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
  inline static std::shared_ptr<GraphicsPipeline> copy_pipeline;
  inline static std::shared_ptr<GraphicsPipeline> mix_pipeline;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const PostProcessingStack& post_processing_stack, const std::shared_ptr<Camera>& target_camera) override;
  void BuildPipelines() override;
};

}  // namespace evo_engine
