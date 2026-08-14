#include "RayTracingPipeline.hpp"

#include "Console.hpp"
#include "Platform.hpp"
#include "Shader.hpp"

using namespace evo_engine;

RayTracingPipeline::~RayTracingPipeline() {
  ReleaseResources();
}

void RayTracingPipeline::ReleaseResources() {
  if (vk_ray_tracing_pipeline_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyPipeline(Platform::GetVkDevice(), vk_ray_tracing_pipeline_, nullptr);
  }
  vk_ray_tracing_pipeline_ = VK_NULL_HANDLE;
  raygen_shader_binding_table_.reset();
  miss_shader_binding_table_.reset();
  closest_hit_shader_binding_table_.reset();
  pipeline_layout_.reset();
}

void RayTracingPipeline::SetMaxRecursionDepth(const uint32_t depth) {
  max_recursion_depth_ = depth;
}

void RayTracingPipeline::SetLinearSweptSpheresEnabled(const bool enabled) {
  linear_swept_spheres_enabled_ = enabled;
}

bool RayTracingPipeline::IsRecursionDepthSupported(const uint32_t requested_depth, const uint32_t device_limit) {
  return requested_depth != 0 && requested_depth <= device_limit;
}

void RayTracingPipeline::Initialize() {
  creation_feedback_ = {};
  if (!Platform::Initialized())
    return;
  ReleaseResources();

  std::vector<VkPipelineShaderStageCreateInfo> shader_stages{};
  std::vector<VkRayTracingShaderGroupCreateInfoKHR> shader_groups{};
  uint32_t closest_hit_shader_index = VK_SHADER_UNUSED_KHR;
  uint32_t any_hit_shader_index = VK_SHADER_UNUSED_KHR;

  if (raygen_shader && raygen_shader->GetShaderType() == ShaderType::RayGen) {
    if (!raygen_shader->Compiled())
      raygen_shader->TryCompile();
    if (raygen_shader->Compiled()) {
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
    } else {
      EVOENGINE_ERROR("Failed to build graphics pipeline: Attempt to link uncompiled ray gen shader!")
      return;
    }
  }

  if (miss_shader && miss_shader->GetShaderType() == ShaderType::Miss) {
    if (!miss_shader->Compiled())
      miss_shader->TryCompile();
    if (miss_shader->Compiled()) {
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
    } else {
      EVOENGINE_ERROR("Failed to build graphics pipeline: Attempt to link uncompiled miss shader!")
      return;
    }
  }

  if (closest_hit_shader && closest_hit_shader->GetShaderType() == ShaderType::ClosestHit) {
    if (!closest_hit_shader->Compiled())
      closest_hit_shader->TryCompile();
    if (closest_hit_shader->Compiled()) {
      VkPipelineShaderStageCreateInfo shader_stage_info{};
      shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shader_stage_info.stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
      shader_stage_info.module = closest_hit_shader->GetShaderModule()->GetVkShaderModule();
      shader_stage_info.pName = "main";
      shader_stages.emplace_back(shader_stage_info);

      closest_hit_shader_index = static_cast<uint32_t>(shader_stages.size()) - 1;
    } else {
      EVOENGINE_ERROR("Failed to build graphics pipeline: Attempt to link uncompiled closest hit shader!")
      return;
    }
  }

  if (any_hit_shader && any_hit_shader->GetShaderType() == ShaderType::AnyHit) {
    if (!any_hit_shader->Compiled())
      any_hit_shader->TryCompile();
    if (any_hit_shader->Compiled()) {
      VkPipelineShaderStageCreateInfo shader_stage_info{};
      shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shader_stage_info.stage = VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
      shader_stage_info.module = any_hit_shader->GetShaderModule()->GetVkShaderModule();
      shader_stage_info.pName = "main";
      shader_stages.emplace_back(shader_stage_info);

      any_hit_shader_index = static_cast<uint32_t>(shader_stages.size()) - 1;
    } else {
      EVOENGINE_ERROR("Failed to build graphics pipeline: Attempt to link uncompiled any hit shader!")
      return;
    }
  }

  if (closest_hit_shader_index != VK_SHADER_UNUSED_KHR || any_hit_shader_index != VK_SHADER_UNUSED_KHR) {
    VkRayTracingShaderGroupCreateInfoKHR closest_hit_group_ci{};
    closest_hit_group_ci.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
    closest_hit_group_ci.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
    closest_hit_group_ci.generalShader = VK_SHADER_UNUSED_KHR;
    closest_hit_group_ci.closestHitShader = closest_hit_shader_index;
    closest_hit_group_ci.anyHitShader = any_hit_shader_index;
    closest_hit_group_ci.intersectionShader = VK_SHADER_UNUSED_KHR;
    shader_groups.push_back(closest_hit_group_ci);
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

  VkRayTracingPipelineCreateInfoKHR raytracing_pipeline_create_info{};
  raytracing_pipeline_create_info.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
#ifdef VK_NV_ray_tracing_linear_swept_spheres
  VkPipelineCreateFlags2CreateInfoKHR pipeline_flags{};
  if (linear_swept_spheres_enabled_) {
    pipeline_flags.sType = VK_STRUCTURE_TYPE_PIPELINE_CREATE_FLAGS_2_CREATE_INFO_KHR;
    pipeline_flags.flags = VK_PIPELINE_CREATE_2_RAY_TRACING_ALLOW_SPHERES_AND_LINEAR_SWEPT_SPHERES_BIT_NV;
    raytracing_pipeline_create_info.pNext = &pipeline_flags;
  }
#endif
  raytracing_pipeline_create_info.layout = pipeline_layout_->GetVkPipelineLayout();
  raytracing_pipeline_create_info.stageCount = shader_stages.size();
  raytracing_pipeline_create_info.pStages = shader_stages.data();

  raytracing_pipeline_create_info.groupCount = static_cast<uint32_t>(shader_groups.size());
  raytracing_pipeline_create_info.pGroups = shader_groups.data();

  const auto& ray_tracing_pipeline_properties = Platform::GetSelectedPhysicalDevice()->ray_tracing_properties_ext;
  if (!IsRecursionDepthSupported(max_recursion_depth_, ray_tracing_pipeline_properties.maxRayRecursionDepth)) {
    EVOENGINE_ERROR("Failed to build ray tracing pipeline: requested recursion depth " +
                    std::to_string(max_recursion_depth_) + " is outside the selected device limit [1, " +
                    std::to_string(ray_tracing_pipeline_properties.maxRayRecursionDepth) + "].");
    return;
  }
  raytracing_pipeline_create_info.maxPipelineRayRecursionDepth = max_recursion_depth_;
  try {
    if (Platform::CheckVk(Platform::CreateRayTracingPipeline(raytracing_pipeline_create_info, vk_ray_tracing_pipeline_,
                                                             creation_feedback_)) != VK_SUCCESS) {
      EVOENGINE_ERROR("Failed to build ray tracing pipeline.");
      if (vk_ray_tracing_pipeline_ != VK_NULL_HANDLE)
        vkDestroyPipeline(Platform::GetVkDevice(), vk_ray_tracing_pipeline_, nullptr);
      vk_ray_tracing_pipeline_ = nullptr;
      return;
    }
  } catch (const std::runtime_error& error) {
    EVOENGINE_ERROR(std::string("Failed to build ray tracing pipeline: ") + error.what());
    if (vk_ray_tracing_pipeline_ != VK_NULL_HANDLE)
      vkDestroyPipeline(Platform::GetVkDevice(), vk_ray_tracing_pipeline_, nullptr);
    vk_ray_tracing_pipeline_ = nullptr;
    return;
  }
  const auto aligned_size = [&](const uint32_t value, const uint32_t alignment) {
    return value + alignment - 1 & ~(alignment - 1);
  };

  const uint32_t handle_size = ray_tracing_pipeline_properties.shaderGroupHandleSize;
  handle_size_aligned_ = aligned_size(ray_tracing_pipeline_properties.shaderGroupHandleSize,
                                      ray_tracing_pipeline_properties.shaderGroupHandleAlignment);
  auto group_count = static_cast<uint32_t>(shader_groups.size());
  const uint32_t sbt_size = group_count * handle_size_aligned_;

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                             VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = handle_size_aligned_;

  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  // Create binding table buffers for each shader type
  auto raygen_shader_binding_table = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  auto miss_shader_binding_table = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  const uint32_t hit_record_count = linear_swept_spheres_enabled_ ? 2u : 1u;
  buffer_create_info.size = handle_size_aligned_ * hit_record_count;
  auto closest_hit_shader_binding_table =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  raygen_shader_binding_table_ = std::move(raygen_shader_binding_table);
  miss_shader_binding_table_ = std::move(miss_shader_binding_table);
  closest_hit_shader_binding_table_ = std::move(closest_hit_shader_binding_table);

  // Copy the pipeline's shader handles into a host buffer
  std::vector<uint8_t> shader_handle_storage(sbt_size);
  try {
    Platform::CheckVk(vkGetRayTracingShaderGroupHandlesKHR(Platform::GetVkDevice(), vk_ray_tracing_pipeline_, 0,
                                                           group_count, sbt_size, shader_handle_storage.data()));
  } catch (const std::runtime_error& error) {
    EVOENGINE_ERROR(std::string("Failed to create ray tracing shader group handles: ") + error.what());
    ReleaseResources();
    return;
  }
  // Copy the shader handles from the host buffer to the binding tables
  raygen_shader_binding_table_->UploadData(handle_size, shader_handle_storage.data());
  miss_shader_binding_table_->UploadData(handle_size, shader_handle_storage.data() + handle_size_aligned_);
  std::vector<uint8_t> hit_shader_handles(handle_size_aligned_ * hit_record_count);
  for (uint32_t record = 0; record < hit_record_count; ++record) {
    memcpy(hit_shader_handles.data() + record * handle_size_aligned_,
           shader_handle_storage.data() + handle_size_aligned_ * 2, handle_size);
  }
  closest_hit_shader_binding_table_->UploadData(hit_shader_handles.size(), hit_shader_handles.data());
}

bool RayTracingPipeline::Initialized() const {
  return vk_ray_tracing_pipeline_ != VK_NULL_HANDLE;
}

const PipelineCreationFeedback& RayTracingPipeline::GetCreationFeedback() const {
  return creation_feedback_;
}

void RayTracingPipeline::Bind(const VkCommandBuffer vk_command_buffer) const {
  vkCmdBindPipeline(vk_command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, vk_ray_tracing_pipeline_);
}

void RayTracingPipeline::BindDescriptorSet(const VkCommandBuffer vk_command_buffer, const uint32_t first_set,
                                           const VkDescriptorSet descriptor_set) const {
  vkCmdBindDescriptorSets(vk_command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
                          pipeline_layout_->GetVkPipelineLayout(), first_set, 1, &descriptor_set, 0, nullptr);
}

void RayTracingPipeline::PushConstantData(const VkCommandBuffer vk_command_buffer, const size_t range_index,
                                          const void* data) const {
  const auto& range = push_constant_ranges[range_index];
  vkCmdPushConstants(vk_command_buffer, pipeline_layout_->GetVkPipelineLayout(), range.stageFlags, range.offset,
                     range.size, data);
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
  hit_shader_sbt_entry.deviceAddress = closest_hit_shader_binding_table_->GetDeviceAddress();
  hit_shader_sbt_entry.stride = handle_size_aligned_;
  hit_shader_sbt_entry.size = closest_hit_shader_binding_table_->GetSize();

  VkStridedDeviceAddressRegionKHR callable_shader_sbt_entry{};

  vkCmdTraceRaysKHR(vk_command_buffer, &raygen_shader_sbt_entry, &miss_shader_sbt_entry, &hit_shader_sbt_entry,
                    &callable_shader_sbt_entry, x, y, z);
}
