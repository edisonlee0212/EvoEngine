#include "GraphicsPipeline.hpp"

#include "Application.hpp"
#include "Console.hpp"
#include "Graphics.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
#include "Utilities.hpp"

using namespace evo_engine;

void GraphicsPipeline::PreparePipeline() {
  const auto render_layer = Application::GetLayer<RenderLayer>();

  std::vector<VkDescriptorSetLayout> set_layouts = {};
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

  VkPipelineInputAssemblyStateCreateInfo input_assembly{};
  input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  input_assembly.primitiveRestartEnable = VK_FALSE;

  if (vertex_shader && vertex_shader->shader_type_ == ShaderType::Vertex && vertex_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo vert_shader_stage_info{};
    vert_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vert_shader_stage_info.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vert_shader_stage_info.module = vertex_shader->shader_module_->GetVkShaderModule();
    vert_shader_stage_info.pName = "main";
    shader_stages.emplace_back(vert_shader_stage_info);
  }
  if (tessellation_control_shader && tessellation_control_shader->shader_type_ == ShaderType::TessellationControl &&
      tessellation_control_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo tess_control_shader_stage_info{};
    tess_control_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    tess_control_shader_stage_info.stage = VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT;
    tess_control_shader_stage_info.module = tessellation_control_shader->shader_module_->GetVkShaderModule();
    tess_control_shader_stage_info.pName = "main";
    shader_stages.emplace_back(tess_control_shader_stage_info);
    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_PATCH_LIST;
  }
  if (tessellation_evaluation_shader &&
      tessellation_evaluation_shader->shader_type_ == ShaderType::TessellationEvaluation &&
      tessellation_evaluation_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo tess_evaluation_shader_stage_info{};
    tess_evaluation_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    tess_evaluation_shader_stage_info.stage = VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT;
    tess_evaluation_shader_stage_info.module = tessellation_evaluation_shader->shader_module_->GetVkShaderModule();
    tess_evaluation_shader_stage_info.pName = "main";
    shader_stages.emplace_back(tess_evaluation_shader_stage_info);
    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_PATCH_LIST;
  }
  if (geometry_shader && geometry_shader->shader_type_ == ShaderType::Geometry && geometry_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo geometry_shader_stage_info{};
    geometry_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    geometry_shader_stage_info.stage = VK_SHADER_STAGE_GEOMETRY_BIT;
    geometry_shader_stage_info.module = geometry_shader->shader_module_->GetVkShaderModule();
    geometry_shader_stage_info.pName = "main";
    shader_stages.emplace_back(geometry_shader_stage_info);
  }
  if (task_shader && task_shader->shader_type_ == ShaderType::Task && task_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo task_shader_stage_info{};
    task_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    task_shader_stage_info.stage = VK_SHADER_STAGE_TASK_BIT_EXT;
    task_shader_stage_info.module = task_shader->shader_module_->GetVkShaderModule();
    task_shader_stage_info.pName = "main";
    shader_stages.emplace_back(task_shader_stage_info);
  }
  if (mesh_shader && mesh_shader->shader_type_ == ShaderType::Mesh && mesh_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo mesh_shader_stage_info{};
    mesh_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    mesh_shader_stage_info.stage = VK_SHADER_STAGE_MESH_BIT_EXT;
    mesh_shader_stage_info.module = mesh_shader->shader_module_->GetVkShaderModule();
    mesh_shader_stage_info.pName = "main";
    shader_stages.emplace_back(mesh_shader_stage_info);
  }
  if (fragment_shader && fragment_shader->shader_type_ == ShaderType::Fragment && fragment_shader->Compiled()) {
    VkPipelineShaderStageCreateInfo fragment_shader_stage_info{};
    fragment_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragment_shader_stage_info.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragment_shader_stage_info.module = fragment_shader->shader_module_->GetVkShaderModule();
    fragment_shader_stage_info.pName = "main";
    shader_stages.emplace_back(fragment_shader_stage_info);
  }

  VkPipelineVertexInputStateCreateInfo vertex_input_info{};
  vertex_input_info.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  auto& binding_description = IGeometry::GetVertexBindingDescriptions(geometry_type);
  auto& attribute_descriptions = IGeometry::GetVertexAttributeDescriptions(geometry_type);
  vertex_input_info.vertexBindingDescriptionCount = static_cast<uint32_t>(binding_description.size());
  vertex_input_info.vertexAttributeDescriptionCount = static_cast<uint32_t>(attribute_descriptions.size());
  vertex_input_info.pVertexBindingDescriptions = binding_description.data();
  vertex_input_info.pVertexAttributeDescriptions = attribute_descriptions.data();

  VkPipelineViewportStateCreateInfo viewport_state{};
  viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewport_state.viewportCount = 1;
  viewport_state.scissorCount = 1;

  VkPipelineTessellationStateCreateInfo tess_info{};
  tess_info.sType = VK_STRUCTURE_TYPE_PIPELINE_TESSELLATION_STATE_CREATE_INFO;
  tess_info.patchControlPoints = tessellation_patch_control_points;

  VkPipelineRasterizationStateCreateInfo rasterizer{};
  rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  rasterizer.depthClampEnable = VK_FALSE;
  rasterizer.rasterizerDiscardEnable = VK_FALSE;
  rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
  rasterizer.lineWidth = 1.0f;
  rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
  rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  rasterizer.depthBiasEnable = VK_FALSE;

  VkPipelineMultisampleStateCreateInfo multisampling{};
  multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisampling.sampleShadingEnable = VK_FALSE;
  multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineColorBlendAttachmentState color_blend_attachment{};
  color_blend_attachment.colorWriteMask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  color_blend_attachment.blendEnable = VK_FALSE;
  std::vector color_blend_attachments = {color_attachment_formats.size(), color_blend_attachment};
  VkPipelineColorBlendStateCreateInfo color_blending{};
  color_blending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  color_blending.logicOpEnable = VK_FALSE;
  color_blending.logicOp = VK_LOGIC_OP_COPY;
  color_blending.attachmentCount = color_blend_attachments.size();
  color_blending.pAttachments = color_blend_attachments.data();
  color_blending.blendConstants[0] = 0.0f;
  color_blending.blendConstants[1] = 0.0f;
  color_blending.blendConstants[2] = 0.0f;
  color_blending.blendConstants[3] = 0.0f;

  VkPipelineDepthStencilStateCreateInfo depth_stencil{};
  depth_stencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depth_stencil.depthTestEnable = VK_TRUE;
  depth_stencil.depthWriteEnable = VK_TRUE;
  depth_stencil.depthCompareOp = VK_COMPARE_OP_LESS;
  depth_stencil.depthBoundsTestEnable = VK_FALSE;
  depth_stencil.stencilTestEnable = VK_FALSE;
  std::vector dynamic_states = {VK_DYNAMIC_STATE_VIEWPORT,
                               VK_DYNAMIC_STATE_SCISSOR,

                               VK_DYNAMIC_STATE_DEPTH_CLAMP_ENABLE_EXT,
                               VK_DYNAMIC_STATE_RASTERIZER_DISCARD_ENABLE,
                               VK_DYNAMIC_STATE_POLYGON_MODE_EXT,
                               VK_DYNAMIC_STATE_CULL_MODE,
                               VK_DYNAMIC_STATE_FRONT_FACE,
                               VK_DYNAMIC_STATE_DEPTH_BIAS_ENABLE,
                               VK_DYNAMIC_STATE_DEPTH_BIAS,
                               VK_DYNAMIC_STATE_LINE_WIDTH,

                               VK_DYNAMIC_STATE_DEPTH_TEST_ENABLE,
                               VK_DYNAMIC_STATE_DEPTH_WRITE_ENABLE,
                               VK_DYNAMIC_STATE_DEPTH_COMPARE_OP,
                               VK_DYNAMIC_STATE_DEPTH_BOUNDS_TEST_ENABLE,
                               VK_DYNAMIC_STATE_STENCIL_TEST_ENABLE,
                               VK_DYNAMIC_STATE_STENCIL_OP,
                               VK_DYNAMIC_STATE_DEPTH_BOUNDS,

                               VK_DYNAMIC_STATE_LOGIC_OP_ENABLE_EXT,
                               VK_DYNAMIC_STATE_LOGIC_OP_EXT,
                               VK_DYNAMIC_STATE_COLOR_BLEND_ENABLE_EXT,
                               VK_DYNAMIC_STATE_COLOR_BLEND_EQUATION_EXT,
                               VK_DYNAMIC_STATE_COLOR_WRITE_MASK_EXT,
                               VK_DYNAMIC_STATE_BLEND_CONSTANTS};
  VkPipelineDynamicStateCreateInfo dynamic_state{};
  dynamic_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynamic_state.dynamicStateCount = static_cast<uint32_t>(dynamic_states.size());
  dynamic_state.pDynamicStates = dynamic_states.data();

  VkPipelineRenderingCreateInfo rendering_create_info{};
  rendering_create_info.viewMask = view_mask;
  rendering_create_info.colorAttachmentCount = color_attachment_formats.size();
  rendering_create_info.pColorAttachmentFormats = color_attachment_formats.data();
  rendering_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
  rendering_create_info.depthAttachmentFormat = depth_attachment_format;
  rendering_create_info.stencilAttachmentFormat = stencil_attachment_format;

  VkGraphicsPipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipeline_info.pDepthStencilState = &depth_stencil;
  pipeline_info.stageCount = shader_stages.size();
  pipeline_info.pTessellationState = &tess_info;
  pipeline_info.pRasterizationState = &rasterizer;
  pipeline_info.pStages = shader_stages.data();
  pipeline_info.pVertexInputState = &vertex_input_info;
  if (mesh_shader)
    pipeline_info.pVertexInputState = nullptr;
  pipeline_info.pInputAssemblyState = &input_assembly;
  pipeline_info.pViewportState = &viewport_state;
  pipeline_info.pMultisampleState = &multisampling;
  pipeline_info.pColorBlendState = &color_blending;
  pipeline_info.pDynamicState = &dynamic_state;
  pipeline_info.layout = pipeline_layout_->GetVkPipelineLayout();
  pipeline_info.pNext = &rendering_create_info;
  pipeline_info.basePipelineHandle = VK_NULL_HANDLE;
  Graphics::CheckVk(vkCreateGraphicsPipelines(Graphics::GetVkDevice(), VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                              &vk_graphics_pipeline_));
  states.ResetAllStates(1);
}

bool GraphicsPipeline::PipelineReady() const {
  return vk_graphics_pipeline_ != VK_NULL_HANDLE;
}

void GraphicsPipeline::Bind(const VkCommandBuffer command_buffer) {
  vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, vk_graphics_pipeline_);
  states.ApplyAllStates(command_buffer, true);
}

void GraphicsPipeline::BindDescriptorSet(const VkCommandBuffer command_buffer, const uint32_t first_set,
                                         const VkDescriptorSet descriptor_set) const {
  vkCmdBindDescriptorSets(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout_->GetVkPipelineLayout(),
                          first_set, 1, &descriptor_set, 0, nullptr);
}

#pragma region Pipeline Data
void PipelineShaderStage::Apply(const VkPipelineShaderStageCreateInfo& vk_pipeline_shader_stage_create_info) {
  flags = vk_pipeline_shader_stage_create_info.flags;
  stage = vk_pipeline_shader_stage_create_info.stage;
  module = vk_pipeline_shader_stage_create_info.module;
  name = vk_pipeline_shader_stage_create_info.pName;
  if (vk_pipeline_shader_stage_create_info.pSpecializationInfo != VK_NULL_HANDLE) {
    specialization_info = *vk_pipeline_shader_stage_create_info.pSpecializationInfo;
  }
}

void PipelineVertexInputState::Apply(const VkPipelineVertexInputStateCreateInfo& vk_pipeline_shader_stage_create_info) {
  flags = vk_pipeline_shader_stage_create_info.flags;
  IGraphicsResource::ApplyVector(vertex_binding_descriptions,
                                 vk_pipeline_shader_stage_create_info.vertexBindingDescriptionCount,
                                 vk_pipeline_shader_stage_create_info.pVertexBindingDescriptions);
  IGraphicsResource::ApplyVector(vertex_attribute_descriptions,
                                 vk_pipeline_shader_stage_create_info.vertexAttributeDescriptionCount,
                                 vk_pipeline_shader_stage_create_info.pVertexAttributeDescriptions);
}

void PipelineInputAssemblyState::Apply(
    const VkPipelineInputAssemblyStateCreateInfo& vk_pipeline_input_assembly_state_create_info) {
  flags = vk_pipeline_input_assembly_state_create_info.flags;
  topology = vk_pipeline_input_assembly_state_create_info.topology;
  primitive_restart_enable = vk_pipeline_input_assembly_state_create_info.primitiveRestartEnable;
}

void PipelineTessellationState::Apply(
    const VkPipelineTessellationStateCreateInfo& vk_pipeline_tessellation_state_create_info) {
  flags = vk_pipeline_tessellation_state_create_info.flags;
  patch_control_points = vk_pipeline_tessellation_state_create_info.patchControlPoints;
}

void PipelineViewportState::Apply(const VkPipelineViewportStateCreateInfo& vk_pipeline_viewport_state_create_info) {
  flags = vk_pipeline_viewport_state_create_info.flags;
  IGraphicsResource::ApplyVector(viewports, vk_pipeline_viewport_state_create_info.viewportCount,
                                 vk_pipeline_viewport_state_create_info.pViewports);
  IGraphicsResource::ApplyVector(scissors, vk_pipeline_viewport_state_create_info.scissorCount,
                                 vk_pipeline_viewport_state_create_info.pScissors);
}

void PipelineRasterizationState::Apply(
    const VkPipelineRasterizationStateCreateInfo& vk_pipeline_rasterization_state_create_info) {
  flags = vk_pipeline_rasterization_state_create_info.flags;
  depth_clamp_enable = vk_pipeline_rasterization_state_create_info.depthClampEnable;
  rasterizer_discard_enable = vk_pipeline_rasterization_state_create_info.rasterizerDiscardEnable;
  polygon_mode = vk_pipeline_rasterization_state_create_info.polygonMode;
  cull_mode = vk_pipeline_rasterization_state_create_info.cullMode;
  front_face = vk_pipeline_rasterization_state_create_info.frontFace;
  depth_bias_enable = vk_pipeline_rasterization_state_create_info.depthBiasEnable;
  depth_bias_constant_factor = vk_pipeline_rasterization_state_create_info.depthBiasConstantFactor;
  depth_bias_clamp = vk_pipeline_rasterization_state_create_info.depthBiasClamp;
  depth_bias_slope_factor = vk_pipeline_rasterization_state_create_info.depthBiasSlopeFactor;
  line_width = vk_pipeline_rasterization_state_create_info.lineWidth;
}

void PipelineMultisampleState::Apply(
    const VkPipelineMultisampleStateCreateInfo& vk_pipeline_multisample_state_create_info) {
  flags = vk_pipeline_multisample_state_create_info.flags;
  rasterization_samples = vk_pipeline_multisample_state_create_info.rasterizationSamples;
  sample_shading_enable = vk_pipeline_multisample_state_create_info.sampleShadingEnable;
  min_sample_shading = vk_pipeline_multisample_state_create_info.minSampleShading;
  if (vk_pipeline_multisample_state_create_info.pSampleMask)
    sample_mask = *vk_pipeline_multisample_state_create_info.pSampleMask;
  alpha_to_coverage_enable = vk_pipeline_multisample_state_create_info.alphaToCoverageEnable;
  alpha_to_one_enable = vk_pipeline_multisample_state_create_info.alphaToOneEnable;
}

void PipelineDepthStencilState::Apply(
    const VkPipelineDepthStencilStateCreateInfo& vk_pipeline_depth_stencil_state_create_info) {
  flags = vk_pipeline_depth_stencil_state_create_info.flags;
  depth_test_enable = vk_pipeline_depth_stencil_state_create_info.depthTestEnable;
  depth_write_enable = vk_pipeline_depth_stencil_state_create_info.depthWriteEnable;
  depth_compare_op = vk_pipeline_depth_stencil_state_create_info.depthCompareOp;
  depth_bounds_test_enable = vk_pipeline_depth_stencil_state_create_info.depthBoundsTestEnable;
  stencil_test_enable = vk_pipeline_depth_stencil_state_create_info.stencilTestEnable;
  front = vk_pipeline_depth_stencil_state_create_info.front;
  back = vk_pipeline_depth_stencil_state_create_info.back;
  min_depth_bounds = vk_pipeline_depth_stencil_state_create_info.minDepthBounds;
  max_depth_bounds = vk_pipeline_depth_stencil_state_create_info.maxDepthBounds;
}

void PipelineColorBlendState::Apply(
    const VkPipelineColorBlendStateCreateInfo& vk_pipeline_color_blend_state_create_info) {
  flags = vk_pipeline_color_blend_state_create_info.flags;
  logic_op_enable = vk_pipeline_color_blend_state_create_info.logicOpEnable;
  logic_op = vk_pipeline_color_blend_state_create_info.logicOp;
  IGraphicsResource::ApplyVector(attachments, vk_pipeline_color_blend_state_create_info.attachmentCount,
                                 vk_pipeline_color_blend_state_create_info.pAttachments);
  blend_constants[0] = vk_pipeline_color_blend_state_create_info.blendConstants[0];
  blend_constants[1] = vk_pipeline_color_blend_state_create_info.blendConstants[1];
  blend_constants[2] = vk_pipeline_color_blend_state_create_info.blendConstants[2];
  blend_constants[3] = vk_pipeline_color_blend_state_create_info.blendConstants[3];
}

void PipelineDynamicState::Apply(const VkPipelineDynamicStateCreateInfo& vk_pipeline_dynamic_state_create_info) {
  flags = vk_pipeline_dynamic_state_create_info.flags;
  IGraphicsResource::ApplyVector(dynamic_states, vk_pipeline_dynamic_state_create_info.dynamicStateCount,
                                 vk_pipeline_dynamic_state_create_info.pDynamicStates);
}
#pragma endregion