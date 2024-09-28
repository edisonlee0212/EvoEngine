#include "RayTracingPipeline.hpp"
#include "Platform.hpp"
#include "Shader.hpp"
using namespace evo_engine;

RayTracingPipeline::~RayTracingPipeline() {
  if (vk_ray_tracing_pipeline_ != VK_NULL_HANDLE) {
    vkDestroyPipeline(Platform::GetVkDevice(), vk_ray_tracing_pipeline_, nullptr);
    vk_ray_tracing_pipeline_ = nullptr;
  }
}


void RayTracingPipeline::Initialize() {
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

  std::vector<VkPipelineShaderStageCreateInfo> shader_stages{};
  std::vector<VkRayTracingShaderGroupCreateInfoKHR> shader_groups{};

  if (raygen_shader && raygen_shader->GetShaderType() == ShaderType::RayGen && raygen_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    shader_stage_info.module = raygen_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);

    VkRayTracingShaderGroupCreateInfoKHR raygen_group_ci{};
    raygen_group_ci.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
    raygen_group_ci.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
    raygen_group_ci.generalShader = static_cast<uint32_t>(shader_stages.size()) - 1;
    raygen_group_ci.closestHitShader = VK_SHADER_UNUSED_KHR;
    raygen_group_ci.anyHitShader = VK_SHADER_UNUSED_KHR;
    raygen_group_ci.intersectionShader = VK_SHADER_UNUSED_KHR;
    shader_groups.push_back(raygen_group_ci);
  }

  if (miss_shader && miss_shader->GetShaderType() == ShaderType::Miss && miss_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_MISS_BIT_KHR;
    shader_stage_info.module = miss_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);

    VkRayTracingShaderGroupCreateInfoKHR miss_group_ci{};
    miss_group_ci.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
    miss_group_ci.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
    miss_group_ci.generalShader = static_cast<uint32_t>(shader_stages.size()) - 1;
    miss_group_ci.closestHitShader = VK_SHADER_UNUSED_KHR;
    miss_group_ci.anyHitShader = VK_SHADER_UNUSED_KHR;
    miss_group_ci.intersectionShader = VK_SHADER_UNUSED_KHR;
    shader_groups.push_back(miss_group_ci);
  }

  if (closest_hit_shader && closest_hit_shader->GetShaderType() == ShaderType::ClosestHit &&
      closest_hit_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
    shader_stage_info.module = closest_hit_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);

    VkRayTracingShaderGroupCreateInfoKHR closes_hit_group_ci{};
    closes_hit_group_ci.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
    closes_hit_group_ci.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
    closes_hit_group_ci.generalShader = VK_SHADER_UNUSED_KHR;
    closes_hit_group_ci.closestHitShader = static_cast<uint32_t>(shader_stages.size()) - 1;
    closes_hit_group_ci.anyHitShader = VK_SHADER_UNUSED_KHR;
    closes_hit_group_ci.intersectionShader = VK_SHADER_UNUSED_KHR;
    shader_groups.push_back(closes_hit_group_ci);
  }
  
  if (any_hit_shader && any_hit_shader->GetShaderType() == ShaderType::AnyHit && any_hit_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
    shader_stage_info.module = any_hit_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);

    VkRayTracingShaderGroupCreateInfoKHR closes_hit_group_ci{};
    closes_hit_group_ci.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
    closes_hit_group_ci.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
    closes_hit_group_ci.generalShader = VK_SHADER_UNUSED_KHR;
    closes_hit_group_ci.closestHitShader = VK_SHADER_UNUSED_KHR;
    closes_hit_group_ci.anyHitShader = static_cast<uint32_t>(shader_stages.size()) - 1;
    closes_hit_group_ci.intersectionShader = VK_SHADER_UNUSED_KHR;
    shader_groups.push_back(closes_hit_group_ci);
  }
  

  VkRayTracingPipelineCreateInfoKHR raytracing_pipeline_create_info{};
  raytracing_pipeline_create_info.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
  raytracing_pipeline_create_info.layout = pipeline_layout_->GetVkPipelineLayout();
  raytracing_pipeline_create_info.stageCount = shader_stages.size();
  raytracing_pipeline_create_info.pStages = shader_stages.data();

  raytracing_pipeline_create_info.groupCount = static_cast<uint32_t>(shader_groups.size());
  raytracing_pipeline_create_info.pGroups = shader_groups.data();

  raytracing_pipeline_create_info.maxPipelineRayRecursionDepth = 1;

  Platform::CheckVk(vkCreateRayTracingPipelinesKHR(Platform::GetVkDevice(), VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &raytracing_pipeline_create_info,
                                                   nullptr,
                                             &vk_ray_tracing_pipeline_));

  const auto aligned_size = [&](const uint32_t value, const uint32_t alignment) {
    return value + alignment - 1 & ~(alignment - 1);
  };

  const auto &ray_tracing_pipeline_properties = Platform::GetSelectedPhysicalDevice()->ray_tracing_properties_ext;
  const uint32_t handle_size = ray_tracing_pipeline_properties.shaderGroupHandleSize;
  handle_size_aligned_ = aligned_size(ray_tracing_pipeline_properties.shaderGroupHandleSize,
                                                    ray_tracing_pipeline_properties.shaderGroupHandleAlignment);
  auto group_count = static_cast<uint32_t>(shader_groups.size());
  const uint32_t sbt_size = group_count * handle_size_aligned_;

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                             VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                             VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = handle_size;

  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  

  // Create binding table buffers for each shader type
  raygen_shader_binding_table_ = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  miss_shader_binding_table_ = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  hit_shader_binding_table_ = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  // Copy the pipeline's shader handles into a host buffer
  std::vector<uint8_t> shader_handle_storage(sbt_size);
  Platform::CheckVk(vkGetRayTracingShaderGroupHandlesKHR(Platform::GetVkDevice(), vk_ray_tracing_pipeline_, 0, group_count, sbt_size,
                                                shader_handle_storage.data()));

  // Copy the shader handles from the host buffer to the binding tables
  raygen_shader_binding_table_->UploadData(handle_size, shader_handle_storage.data());
  miss_shader_binding_table_->UploadData(handle_size, shader_handle_storage.data() + handle_size_aligned_);
  hit_shader_binding_table_->UploadData(handle_size, shader_handle_storage.data() + handle_size_aligned_ * 2);

}

bool RayTracingPipeline::Initialized() const {
  return vk_ray_tracing_pipeline_ != VK_NULL_HANDLE;
}

void RayTracingPipeline::Bind(const VkCommandBuffer vk_command_buffer) const {
  vkCmdBindPipeline(vk_command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, vk_ray_tracing_pipeline_);
}

void RayTracingPipeline::BindDescriptorSet(const VkCommandBuffer vk_command_buffer, const uint32_t first_set,
                                           const VkDescriptorSet descriptor_set) const {
  vkCmdBindDescriptorSets(vk_command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
                          pipeline_layout_->GetVkPipelineLayout(),
                          first_set, 1, &descriptor_set, 0, nullptr);
}

void RayTracingPipeline::Trace(const VkCommandBuffer vk_command_buffer, const uint32_t x, const uint32_t y,
                               const uint32_t z) const {
  VkStridedDeviceAddressRegionKHR raygen_shader_sbt_entry{};
  raygen_shader_sbt_entry.deviceAddress = raygen_shader_binding_table_->GetDeviceAddress();
  raygen_shader_sbt_entry.stride = handle_size_aligned_;
  raygen_shader_sbt_entry.size = handle_size_aligned_;

  VkStridedDeviceAddressRegionKHR miss_shader_sbt_entry{};
  miss_shader_sbt_entry.deviceAddress = miss_shader_binding_table_->GetDeviceAddress();
  miss_shader_sbt_entry.stride = handle_size_aligned_;
  miss_shader_sbt_entry.size = handle_size_aligned_;

  VkStridedDeviceAddressRegionKHR hit_shader_sbt_entry{};
  hit_shader_sbt_entry.deviceAddress = hit_shader_binding_table_->GetDeviceAddress();
  hit_shader_sbt_entry.stride = handle_size_aligned_;
  hit_shader_sbt_entry.size = handle_size_aligned_;

  VkStridedDeviceAddressRegionKHR callable_shader_sbt_entry{};

  vkCmdTraceRaysKHR(vk_command_buffer, &raygen_shader_sbt_entry, &miss_shader_sbt_entry, &hit_shader_sbt_entry,
                    &callable_shader_sbt_entry, x, y, z);
}
