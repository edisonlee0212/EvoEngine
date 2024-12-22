#pragma once
#include "GraphicsPipeline.hpp"
#include "IAsset.hpp"
#include "RenderTexture.hpp"
namespace evo_engine {
class Camera;

struct SsaoSettings {};

struct BloomSettings {};

class IPostProcessing {
 public:
  virtual void Process(const std::shared_ptr<RenderTexture>& render_texture0,
                       const std::shared_ptr<RenderTexture>& render_texture1,
                       const std::shared_ptr<RenderTexture>& render_texture2,
                       const std::shared_ptr<Camera>& target_camera) const = 0;
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  };
};

class ScreenSpaceReflection : public IPostProcessing {
 public:
  float max_distance = 0.5f;
  float resolution = 0.3f;
  int initial_steps = 5;
  float thickness = 0.5f;
  struct PushConstant {
    int32_t camera_index = 0;
    float max_distance;
    float resolution;
    int initial_steps;
    float thickness;
    int horizontal = false;
    float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
  };
  inline static std::shared_ptr<DescriptorSetLayout> ssr_reflect_layout;
  inline static std::shared_ptr<DescriptorSetLayout> ssr_blur_layout;
  inline static std::shared_ptr<DescriptorSetLayout> ssr_combine_layout;

  inline static std::shared_ptr<GraphicsPipeline> ssr_reflect_pipeline;
  inline static std::shared_ptr<GraphicsPipeline> ssr_blur_pipeline;
  inline static std::shared_ptr<GraphicsPipeline> ssr_combine_pipeline;

  inline static std::shared_ptr<DescriptorSet> ssr_reflect_descriptor_set;          // SSR_REFLECT_LAYOUT: 0, 1, 2, 3
  inline static std::shared_ptr<DescriptorSet> ssr_blur_horizontal_descriptor_set;  // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  inline static std::shared_ptr<DescriptorSet> ssr_blur_vertical_descriptor_set;    // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  inline static std::shared_ptr<DescriptorSet> ssr_combine_descriptor_set;          // SSR_COMBINE: 0, 1

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const std::shared_ptr<RenderTexture>& render_texture0,
               const std::shared_ptr<RenderTexture>& render_texture1,
               const std::shared_ptr<RenderTexture>& render_texture2,
               const std::shared_ptr<Camera>& target_camera) const override;
  static void BuildPipelines();
};

class PostProcessingStack : public IAsset {
  friend class Camera;
  std::shared_ptr<RenderTexture> render_texture0;
  std::shared_ptr<RenderTexture> render_texture1;
  std::shared_ptr<RenderTexture> render_texture2;
  void Resize(const glm::uvec2& size) const;

 public:
  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const std::shared_ptr<Camera>& target_camera) const;
  SsaoSettings ssao_settings{};
  BloomSettings bloom_settings{};
  ScreenSpaceReflection screen_space_reflection{};

  bool enable_screen_space_ambient_occlusion = false;
  bool enable_bloom = false;
  bool enable_screen_space_reflection = false;
};
}  // namespace evo_engine
