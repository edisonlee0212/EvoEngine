#pragma once
#include "IAsset.hpp"
#include "RenderTexture.hpp"
namespace evo_engine {
class Camera;

struct SsaoSettings {};

struct BloomSettings {};

struct SsrSettings {
  int num_binary_search_steps = 8;
  float step = 0.5f;
  float min_ray_step = 0.1f;
  int max_steps = 16;
};

struct SsaoPushConstant {};

struct BloomPushConstant {};

struct SsrPushConstant {
  uint32_t camera_index = 0;
  int num_binary_search_steps = 8;
  float step = 0.5f;
  float min_ray_step = 0.1f;
  int max_steps = 16;
  float reflection_specular_falloff_exponent = 3.0f;
  int horizontal = false;
  float weight[5] = {0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f};
};

class PostProcessingStack : public IAsset {
  friend class Camera;
  std::shared_ptr<RenderTexture> render_texture0_;
  std::shared_ptr<RenderTexture> render_texture1_;
  std::shared_ptr<RenderTexture> render_texture2_;

  std::shared_ptr<DescriptorSet> ssr_reflect_descriptor_set_ = VK_NULL_HANDLE;  // SSR_REFLECT_LAYOUT: 0, 1, 2, 3
  std::shared_ptr<DescriptorSet> ssr_blur_horizontal_descriptor_set_ =
      VK_NULL_HANDLE;  // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  std::shared_ptr<DescriptorSet> ssr_blur_vertical_descriptor_set_ =
      VK_NULL_HANDLE;                                                           // RENDER_TEXTURE_PRESENT_LAYOUT: 0
  std::shared_ptr<DescriptorSet> ssr_combine_descriptor_set_ = VK_NULL_HANDLE;  // SSR_COMBINE: 0, 1
  void Resize(const glm::uvec2& size) const;

 public:
  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Process(const std::shared_ptr<Camera>& target_camera) const;
  SsaoSettings ssao_settings{};
  BloomSettings bloom_settings{};
  SsrSettings ssr_settings{};

  bool ssao = false;
  bool bloom = false;
  bool ssr = false;
};
}  // namespace evo_engine
