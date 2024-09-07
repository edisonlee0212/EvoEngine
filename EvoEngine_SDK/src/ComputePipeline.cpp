#include "ComputePipeline.hpp"
#include "Platform.hpp"
#include "Shader.hpp"
using namespace evo_engine;

ComputePipeline::~ComputePipeline() {
  if (vk_compute_pipeline_ != VK_NULL_HANDLE) {
    vkDestroyPipeline(Platform::GetVkDevice(), vk_compute_pipeline_, nullptr);
    vk_compute_pipeline_ = nullptr;
  }
}

void ComputePipeline::Initialize() {
  std::vector<VkDescriptorSetLayout> set_layouts = {};
  set_layouts.reserve(descriptor_set_layouts.size());
  for (const auto& i : descriptor_set_layouts) {
    set_layouts.push_back(i->GetVkDescriptorSetLayout());
  }
  VkPipelineLayoutCreateInfo pipeline_layout_info{};
  pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipeline_layout_info.setLayoutCount = set_layouts.size();
  pipeline_layout_info.pSetLayouts = set_layouts.data();
  pipeline_layout_info.pushConstantRangeCount = push_constant_ranges.size();
  pipeline_layout_info.pPushConstantRanges = push_constant_ranges.data();

  pipeline_layout_ = std::make_unique<PipelineLayout>(pipeline_layout_info);

  VkPipelineShaderStageCreateInfo shader_stage_create_info;
  shader_stage_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  shader_stage_create_info.stage = VK_SHADER_STAGE_VERTEX_BIT;
  shader_stage_create_info.module = compute_shader->GetShaderModule()->GetVkShaderModule();
  shader_stage_create_info.pName = "main";
  shader_stage_create_info.flags = 0;
  shader_stage_create_info.pNext = nullptr;

  VkComputePipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeline_info.layout = pipeline_layout_->GetVkPipelineLayout();
  pipeline_info.stage = shader_stage_create_info;
  Platform::CheckVk(vkCreateComputePipelines(Platform::GetVkDevice(), VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                             &vk_compute_pipeline_));
}

bool ComputePipeline::Initialized() const {
  return vk_compute_pipeline_ != VK_NULL_HANDLE;
}

void ComputePipeline::Bind(const CommandBuffer& command_buffer) const {
  vkCmdBindPipeline(command_buffer.GetVkCommandBuffer(), VK_PIPELINE_BIND_POINT_COMPUTE, vk_compute_pipeline_);
}

void ComputePipeline::BindDescriptorSet(const CommandBuffer& command_buffer, const uint32_t first_set,
                                        const VkDescriptorSet descriptor_set) const {
  vkCmdBindDescriptorSets(command_buffer.GetVkCommandBuffer(), VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_->GetVkPipelineLayout(),
                          first_set, 1, &descriptor_set, 0, nullptr);
}