#pragma once
#include "GraphicsPipelineStates.hpp"
#include "GraphicsResources.hpp"
#include "IGeometry.hpp"
namespace evo_engine {
class Shader;

#pragma region Pipeline Data

struct PipelineShaderStage {
  VkPipelineShaderStageCreateFlags flags;
  VkShaderStageFlagBits stage;
  VkShaderModule module;
  std::string name;
  std::optional<VkSpecializationInfo> specialization_info;
  void Apply(const VkPipelineShaderStageCreateInfo& vk_pipeline_shader_stage_create_info);
};

struct PipelineVertexInputState {
  VkPipelineVertexInputStateCreateFlags flags;
  std::vector<VkVertexInputBindingDescription> vertex_binding_descriptions;
  std::vector<VkVertexInputAttributeDescription> vertex_attribute_descriptions;
  void Apply(const VkPipelineVertexInputStateCreateInfo& vk_pipeline_shader_stage_create_info);
};
struct PipelineInputAssemblyState {
  VkPipelineInputAssemblyStateCreateFlags flags;
  VkPrimitiveTopology topology;
  VkBool32 primitive_restart_enable;
  void Apply(const VkPipelineInputAssemblyStateCreateInfo& vk_pipeline_input_assembly_state_create_info);
};

struct PipelineTessellationState {
  VkPipelineTessellationStateCreateFlags flags;
  uint32_t patch_control_points;
  void Apply(const VkPipelineTessellationStateCreateInfo& vk_pipeline_tessellation_state_create_info);
};

struct PipelineViewportState {
  VkPipelineViewportStateCreateFlags flags;
  std::vector<VkViewport> viewports;
  std::vector<VkRect2D> scissors;
  void Apply(const VkPipelineViewportStateCreateInfo& vk_pipeline_viewport_state_create_info);
};

struct PipelineRasterizationState {
  VkPipelineRasterizationStateCreateFlags flags;
  VkBool32 depth_clamp_enable;
  VkBool32 rasterizer_discard_enable;
  VkPolygonMode polygon_mode;
  VkCullModeFlags cull_mode;
  VkFrontFace front_face;
  VkBool32 depth_bias_enable;
  float depth_bias_constant_factor;
  float depth_bias_clamp;
  float depth_bias_slope_factor;
  float line_width;
  void Apply(const VkPipelineRasterizationStateCreateInfo& vk_pipeline_rasterization_state_create_info);
};

struct PipelineMultisampleState {
  VkPipelineMultisampleStateCreateFlags flags;
  VkSampleCountFlagBits rasterization_samples;
  VkBool32 sample_shading_enable;
  float min_sample_shading;
  std::optional<VkSampleMask> sample_mask;
  VkBool32 alpha_to_coverage_enable;
  VkBool32 alpha_to_one_enable;
  void Apply(const VkPipelineMultisampleStateCreateInfo& vk_pipeline_multisample_state_create_info);
};
struct PipelineDepthStencilState {
  VkPipelineDepthStencilStateCreateFlags flags;
  VkBool32 depth_test_enable;
  VkBool32 depth_write_enable;
  VkCompareOp depth_compare_op;
  VkBool32 depth_bounds_test_enable;
  VkBool32 stencil_test_enable;
  VkStencilOpState front;
  VkStencilOpState back;
  float min_depth_bounds;
  float max_depth_bounds;
  void Apply(const VkPipelineDepthStencilStateCreateInfo& vk_pipeline_depth_stencil_state_create_info);
};
struct PipelineColorBlendState {
  VkPipelineColorBlendStateCreateFlags flags;
  VkBool32 logic_op_enable;
  VkLogicOp logic_op;
  std::vector<VkPipelineColorBlendAttachmentState> attachments;
  float blend_constants[4];
  void Apply(const VkPipelineColorBlendStateCreateInfo& vk_pipeline_color_blend_state_create_info);
};

struct PipelineDynamicState {
  VkPipelineDynamicStateCreateFlags flags;
  std::vector<VkDynamicState> dynamic_states;
  void Apply(const VkPipelineDynamicStateCreateInfo& vk_pipeline_dynamic_state_create_info);
};
#pragma endregion

class GraphicsPipeline {
  friend class Graphics;
  friend class RenderLayer;
  friend class GraphicsPipelineStates;

  std::unique_ptr<PipelineLayout> pipeline_layout_ = {};

  VkPipeline vk_graphics_pipeline_ = VK_NULL_HANDLE;

 public:
  GraphicsPipelineStates states{};

  std::vector<std::shared_ptr<DescriptorSetLayout>> descriptor_set_layouts;

  std::shared_ptr<Shader> vertex_shader;
  std::shared_ptr<Shader> tessellation_control_shader;
  std::shared_ptr<Shader> tessellation_evaluation_shader;
  std::shared_ptr<Shader> geometry_shader;

  std::shared_ptr<Shader> task_shader;
  std::shared_ptr<Shader> mesh_shader;
  std::shared_ptr<Shader> fragment_shader;
  GeometryType geometry_type = GeometryType::Mesh;

  uint32_t view_mask;
  std::vector<VkFormat> color_attachment_formats;
  VkFormat depth_attachment_format;
  VkFormat stencil_attachment_format;

  uint32_t tessellation_patch_control_points = 4;

  std::vector<VkPushConstantRange> push_constant_ranges;

  void PreparePipeline();
  [[nodiscard]] bool PipelineReady() const;
  void Bind(VkCommandBuffer command_buffer);
  void BindDescriptorSet(VkCommandBuffer command_buffer, uint32_t first_set, VkDescriptorSet descriptor_set) const;
  template <typename T>
  void PushConstant(VkCommandBuffer command_buffer, size_t range_index, const T& data);
};

template <typename T>
void GraphicsPipeline::PushConstant(const VkCommandBuffer command_buffer, const size_t range_index, const T& data) {
  vkCmdPushConstants(command_buffer, pipeline_layout_->GetVkPipelineLayout(),
                     push_constant_ranges[range_index].stageFlags, push_constant_ranges[range_index].offset,
                     push_constant_ranges[range_index].size, &data);
}
}  // namespace evo_engine
