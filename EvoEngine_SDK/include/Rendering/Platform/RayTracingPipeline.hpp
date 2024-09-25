#pragma once
#include "GraphicsResources.hpp"
namespace evo_engine {
class Shader;

class RayTracerPipeline final : public IGraphicsResource {
  friend class Platform;

  std::unique_ptr<PipelineLayout> pipeline_layout_ = {};

  VkPipeline vk_ray_tracing_pipeline_ = VK_NULL_HANDLE;

 public:
  ~RayTracerPipeline() override;
  std::vector<VkPushConstantRange> push_constant_ranges;
  std::vector<std::shared_ptr<DescriptorSetLayout>> descriptor_set_layouts;

  std::shared_ptr<Shader> raygen_shader;
  std::shared_ptr<Shader> closest_hit_shader;
  std::shared_ptr<Shader> miss_shader;
  std::shared_ptr<Shader> any_hit_shader;
  std::shared_ptr<Shader> intersection_shader;
  std::shared_ptr<Shader> callable_shader;

  void Initialize();
  [[nodiscard]] bool Initialized() const;

  void Bind(VkCommandBuffer vk_command_buffer) const;
  void BindDescriptorSet(VkCommandBuffer vk_command_buffer, uint32_t first_set, VkDescriptorSet descriptor_set) const;
  template <typename T>
  void PushConstant(VkCommandBuffer vk_command_buffer, size_t range_index, const T& data);
};
}  // namespace evo_engine