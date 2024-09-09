#pragma once
#include "GraphicsResources.hpp"
namespace evo_engine {
class Shader;
class ComputePipeline final : public IGraphicsResource{
  friend class Platform;

  std::unique_ptr<PipelineLayout> pipeline_layout_ = {};

  VkPipeline vk_compute_pipeline_ = VK_NULL_HANDLE;

 public:
  ~ComputePipeline() override;
  std::vector<VkPushConstantRange> push_constant_ranges;
  std::vector<std::shared_ptr<DescriptorSetLayout>> descriptor_set_layouts;

  std::shared_ptr<Shader> compute_shader;

  void Initialize();
  [[nodiscard]] bool Initialized() const;

  void Bind(VkCommandBuffer vk_command_buffer) const;
  void BindDescriptorSet(VkCommandBuffer vk_command_buffer, uint32_t first_set, VkDescriptorSet descriptor_set) const;
  template <typename T>
  void PushConstant(VkCommandBuffer vk_command_buffer, size_t range_index, const T& data);
};

template <typename T>
void ComputePipeline::PushConstant(const VkCommandBuffer vk_command_buffer, const size_t range_index, const T& data) {
  vkCmdPushConstants(vk_command_buffer, pipeline_layout_->GetVkPipelineLayout(),
                     push_constant_ranges[range_index].stageFlags, push_constant_ranges[range_index].offset,
                     push_constant_ranges[range_index].size, &data);
}
}  // namespace evo_engine
