#include "RayTracingPipeline.hpp"
#include "Platform.hpp"
#include "Shader.hpp"
using namespace evo_engine;

RayTracerPipeline::~RayTracerPipeline() {
  if (vk_ray_tracing_pipeline_ != VK_NULL_HANDLE) {
    vkDestroyPipeline(Platform::GetVkDevice(), vk_ray_tracing_pipeline_, nullptr);
    vk_ray_tracing_pipeline_ = nullptr;
  }
}

void RayTracerPipeline::Initialize() {
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

  if (raygen_shader && raygen_shader->GetShaderType() == ShaderType::RayGen && raygen_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    shader_stage_info.module = raygen_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);
  }
  if (closest_hit_shader && closest_hit_shader->GetShaderType() == ShaderType::ClosestHit &&
      closest_hit_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
    shader_stage_info.module = closest_hit_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);
  }
  if (miss_shader && miss_shader->GetShaderType() == ShaderType::Miss && miss_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_MISS_BIT_KHR;
    shader_stage_info.module = miss_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);
  }
  if (any_hit_shader && any_hit_shader->GetShaderType() == ShaderType::AnyHit && any_hit_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
    shader_stage_info.module = any_hit_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);
  }
  if (intersection_shader && intersection_shader->GetShaderType() == ShaderType::Intersection &&
      intersection_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_INTERSECTION_BIT_KHR;
    shader_stage_info.module = intersection_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);
  }
  if (callable_shader && callable_shader->GetShaderType() == ShaderType::Callable && callable_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo shader_stage_info{};
    shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shader_stage_info.stage = VK_SHADER_STAGE_CALLABLE_BIT_KHR;
    shader_stage_info.module = callable_shader->GetShaderModule()->GetVkShaderModule();
    shader_stage_info.pName = "main";
    shader_stages.emplace_back(shader_stage_info);
  }



  VkRayTracingPipelineCreateInfoKHR pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeline_info.layout = pipeline_layout_->GetVkPipelineLayout();
  pipeline_info.stageCount = shader_stages.size();
  pipeline_info.pStages = shader_stages.data();


  Platform::CheckVk(vkCreateRayTracingPipelinesKHR(Platform::GetVkDevice(), VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &pipeline_info,
                                                   nullptr,
                                             &vk_ray_tracing_pipeline_));
}
