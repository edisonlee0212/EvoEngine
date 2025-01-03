#include "ComputePipeline.hpp"

#include "Console.hpp"
#include "Platform.hpp"
#include "Shader.hpp"
using namespace evo_engine;

ComputePipeline::~ComputePipeline() {
  if (vk_compute_pipeline_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyPipeline(Platform::GetVkDevice(), vk_compute_pipeline_, nullptr);
    vk_compute_pipeline_ = nullptr;
  }
}

void ComputePipeline::Initialize() {
  if (!Platform::Initialized())
    return;
  if (vk_compute_pipeline_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyPipeline(Platform::GetVkDevice(), vk_compute_pipeline_, nullptr);
    vk_compute_pipeline_ = nullptr;
  }
  VkPipelineShaderStageCreateInfo shader_stage_create_info{};
  if (compute_shader && compute_shader->GetShaderType() == ShaderType::Compute) {
    if (!compute_shader->Compiled())
      compute_shader->TryCompile();
    if (compute_shader->Compiled()) {
      shader_stage_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shader_stage_create_info.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      shader_stage_create_info.module = compute_shader->GetShaderModule()->GetVkShaderModule();
      shader_stage_create_info.pName = "main";
      shader_stage_create_info.flags = 0;
      shader_stage_create_info.pNext = nullptr;
    } else {
      EVOENGINE_ERROR("Failed to build graphics pipeline: Attempt to link uncompiled vertex shader!")
      return;
    }
  }

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

  VkSpecializationInfo vk_specialization_info{};
  std::vector<VkSpecializationMapEntry> specialization_map_entries;
  for (int i = 0; i < map_entries.size(); i++) {
    specialization_map_entries.emplace_back();
    auto& entry = specialization_map_entries.back();
    entry.constantID = i;
    entry.offset = sizeof(int32_t) * i;
    entry.size = sizeof(int32_t);
  }
  vk_specialization_info.mapEntryCount = specialization_map_entries.size();
  if (!specialization_map_entries.empty()) {
    vk_specialization_info.pMapEntries = specialization_map_entries.data();
    vk_specialization_info.dataSize = sizeof(int32_t) * map_entries.size();
    vk_specialization_info.pData = map_entries.data();
  }
  shader_stage_create_info.pSpecializationInfo = &vk_specialization_info;

  VkComputePipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeline_info.layout = pipeline_layout_->GetVkPipelineLayout();
  pipeline_info.stage = shader_stage_create_info;
  try {
    Platform::CheckVk(vkCreateComputePipelines(Platform::GetVkDevice(), VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                               &vk_compute_pipeline_));
  } catch (const std::runtime_error& error) {
    EVOENGINE_ERROR(std::string("Failed to build compute pipeline: ") + error.what());
    vk_compute_pipeline_ = nullptr;
  }
}

bool ComputePipeline::Initialized() const {
  return vk_compute_pipeline_ != VK_NULL_HANDLE;
}

void ComputePipeline::Bind(VkCommandBuffer vk_command_buffer) const {
  vkCmdBindPipeline(vk_command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, vk_compute_pipeline_);
}

void ComputePipeline::BindDescriptorSet(VkCommandBuffer vk_command_buffer, const uint32_t first_set,
                                        const VkDescriptorSet descriptor_set) const {
  vkCmdBindDescriptorSets(vk_command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_->GetVkPipelineLayout(),
                          first_set, 1, &descriptor_set, 0, nullptr);
}