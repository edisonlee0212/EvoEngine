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
  creation_feedback_ = {};
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
#ifdef VK_NV_ray_tracing_linear_swept_spheres
  VkPipelineCreateFlags2CreateInfoKHR pipeline_flags{};
  if (linear_swept_spheres_enabled && Platform::RayTracingLinearSweptSpheresEnabled()) {
    pipeline_flags.sType = VK_STRUCTURE_TYPE_PIPELINE_CREATE_FLAGS_2_CREATE_INFO_KHR;
    pipeline_flags.flags = VK_PIPELINE_CREATE_2_RAY_TRACING_ALLOW_SPHERES_AND_LINEAR_SWEPT_SPHERES_BIT_NV;
    pipeline_info.pNext = &pipeline_flags;
  }
#endif
  try {
    Platform::CheckVk(Platform::CreateComputePipeline(pipeline_info, vk_compute_pipeline_, creation_feedback_));
  } catch (const std::runtime_error& error) {
    EVOENGINE_ERROR(std::string("Failed to build compute pipeline: ") + error.what());
    vk_compute_pipeline_ = nullptr;
  }
}

bool ComputePipeline::Initialized() const {
  return vk_compute_pipeline_ != VK_NULL_HANDLE;
}

const PipelineCreationFeedback& ComputePipeline::GetCreationFeedback() const {
  return creation_feedback_;
}

void ComputePipeline::Bind(VkCommandBuffer vk_command_buffer) const {
  vkCmdBindPipeline(vk_command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, vk_compute_pipeline_);
}

void ComputePipeline::BindDescriptorSet(VkCommandBuffer vk_command_buffer, const uint32_t first_set,
                                        const VkDescriptorSet descriptor_set) const {
  vkCmdBindDescriptorSets(vk_command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_->GetVkPipelineLayout(),
                          first_set, 1, &descriptor_set, 0, nullptr);
}

void ComputePipeline::Dispatch(const VkCommandBuffer vk_command_buffer, const uint32_t x, const uint32_t y,
                               const uint32_t z) const {
  vkCmdDispatch(vk_command_buffer, x, y, z);
}

void ComputePipeline::PushConstantData(const VkCommandBuffer vk_command_buffer, const size_t range_index,
                                       const void* data) const {
  const auto& range = push_constant_ranges[range_index];
  vkCmdPushConstants(vk_command_buffer, pipeline_layout_->GetVkPipelineLayout(), range.stageFlags, range.offset,
                     range.size, data);
}
