#pragma once
#include "GraphicsResources.hpp"
namespace evo_engine {
class Shader;

class RayTracingPipeline final : public IGraphicsResource {
  friend class Platform;

  std::unique_ptr<PipelineLayout> pipeline_layout_ = {};

  VkPipeline vk_ray_tracing_pipeline_ = VK_NULL_HANDLE;

  std::shared_ptr<Buffer> raygen_shader_binding_table_;
  std::shared_ptr<Buffer> miss_shader_binding_table_;
  std::shared_ptr<Buffer> closest_hit_shader_binding_table_;
  uint32_t handle_size_aligned_ = 0;
 public:
  ~RayTracingPipeline() override;
  std::vector<VkPushConstantRange> push_constant_ranges;
  std::vector<std::shared_ptr<DescriptorSetLayout>> descriptor_set_layouts;

  std::shared_ptr<Shader> raygen_shader;
  std::shared_ptr<Shader> miss_shader;

  std::shared_ptr<Shader> closest_hit_shader;
  std::shared_ptr<Shader> any_hit_shader;

  std::shared_ptr<Shader> intersection_shader;
  std::shared_ptr<Shader> callable_shader;

  void Initialize();
  [[nodiscard]] bool Initialized() const;

  void Bind(VkCommandBuffer vk_command_buffer) const;
  void BindDescriptorSet(VkCommandBuffer vk_command_buffer, uint32_t first_set, VkDescriptorSet descriptor_set) const;
  template <typename T>
  void PushConstant(VkCommandBuffer vk_command_buffer, size_t range_index, const T& data);

  void Trace(VkCommandBuffer vk_command_buffer, uint32_t x, uint32_t y, uint32_t z) const;
};

template <typename T>
void RayTracingPipeline::PushConstant(const VkCommandBuffer vk_command_buffer, const size_t range_index, const T& data) {
  vkCmdPushConstants(vk_command_buffer, pipeline_layout_->GetVkPipelineLayout(),
                     push_constant_ranges[range_index].stageFlags, push_constant_ranges[range_index].offset,
                     push_constant_ranges[range_index].size, &data);
}
}  // namespace evo_engine