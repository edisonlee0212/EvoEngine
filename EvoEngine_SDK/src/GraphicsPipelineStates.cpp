#include "GraphicsPipelineStates.hpp"

#include "GraphicsResources.hpp"

using namespace evo_engine;

void GraphicsPipelineStates::ResetAllStates(const size_t color_attachment_size) {
  view_port = {};
  view_port.width = 1;
  view_port.height = 1;
  scissor = {};
  depth_clamp = false;
  rasterizer_discard = false;
  polygon_mode = VK_POLYGON_MODE_FILL;
  cull_mode = VK_CULL_MODE_NONE;
  front_face = VK_FRONT_FACE_CLOCKWISE;
  depth_bias = false;
  depth_bias_constant_clamp_slope = glm::vec3(0.0f);
  line_width = 1.0f;
  depth_test = true;
  depth_write = true;
  depth_compare = VK_COMPARE_OP_LESS;
  depth_bound_test = false;
  min_max_depth_bound = glm::vec2(0.0f, 1.0f);
  stencil_test = false;
  stencil_face_mask = VK_STENCIL_FACE_FRONT_BIT;
  stencil_fail_op = VK_STENCIL_OP_ZERO;
  stencil_pass_op = VK_STENCIL_OP_ZERO;
  stencil_depth_fail_op = VK_STENCIL_OP_ZERO;
  stencil_compare_op = VK_COMPARE_OP_LESS;

  logic_op_enable = VK_FALSE;
  logic_op = VK_LOGIC_OP_COPY;
  color_blend_attachment_states.clear();
  color_blend_attachment_states.resize(std::max(static_cast<size_t>(1), color_attachment_size));
  for (auto& i : color_blend_attachment_states) {
    i.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    i.blendEnable = VK_FALSE;
    i.colorBlendOp = i.alphaBlendOp = VK_BLEND_OP_ADD;

    i.srcColorBlendFactor = i.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    i.dstColorBlendFactor = i.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  }
  blend_constants[0] = 0;
  blend_constants[1] = 0;
  blend_constants[2] = 0;
  blend_constants[3] = 0;
}

void GraphicsPipelineStates::ApplyAllStates(const VkCommandBuffer vk_command_buffer, const bool force_set) {
  if (force_set || view_port_applied_.x != view_port.x || view_port_applied_.y != view_port.y ||
      view_port_applied_.width != view_port.width || view_port_applied_.height != view_port.height ||
      view_port_applied_.maxDepth != view_port.maxDepth || view_port_applied_.minDepth != view_port.minDepth) {
    view_port_applied_ = view_port;
    view_port_applied_.width = view_port.width = glm::max(1.0f, view_port.width);
    view_port_applied_.height = view_port.height = glm::max(1.0f, view_port.height);
    vkCmdSetViewport(vk_command_buffer, 0, 1, &view_port_applied_);
  }
  if (force_set || scissor_applied_.extent.height != scissor.extent.height ||
      scissor_applied_.extent.width != scissor.extent.width || scissor_applied_.offset.x != scissor.offset.x ||
      scissor_applied_.offset.y != scissor.offset.y) {
    scissor_applied_ = scissor;
    vkCmdSetScissor(vk_command_buffer, 0, 1, &scissor_applied_);
  }

  if (force_set || depth_clamp_applied_ != depth_clamp) {
    depth_clamp_applied_ = depth_clamp;
    vkCmdSetDepthClampEnableEXT(vk_command_buffer, depth_clamp_applied_);
  }
  if (force_set || rasterizer_discard_applied_ != rasterizer_discard) {
    rasterizer_discard_applied_ = rasterizer_discard;
    vkCmdSetRasterizerDiscardEnable(vk_command_buffer, rasterizer_discard_applied_);
  }
  if (force_set || polygon_mode_applied_ != polygon_mode) {
    polygon_mode_applied_ = polygon_mode;
    vkCmdSetPolygonModeEXT(vk_command_buffer, polygon_mode_applied_);
  }
  if (force_set || cull_mode_applied_ != cull_mode) {
    cull_mode_applied_ = cull_mode;
    vkCmdSetCullModeEXT(vk_command_buffer, cull_mode_applied_);
  }
  if (force_set || front_face_applied_ != front_face) {
    front_face_applied_ = front_face;
    vkCmdSetFrontFace(vk_command_buffer, front_face_applied_);
  }
  if (force_set || depth_bias_applied_ != depth_bias) {
    depth_bias_applied_ = depth_bias;
    vkCmdSetDepthBiasEnable(vk_command_buffer, depth_bias_applied_);
  }
  if (force_set || depth_bias_constant_clamp_slope_applied_ != depth_bias_constant_clamp_slope) {
    depth_bias_constant_clamp_slope_applied_ = depth_bias_constant_clamp_slope;
    vkCmdSetDepthBias(vk_command_buffer, depth_bias_constant_clamp_slope_applied_.x, depth_bias_constant_clamp_slope_applied_.y,
                      depth_bias_constant_clamp_slope_applied_.z);
  }
  if (force_set || line_width_applied_ != line_width) {
    line_width_applied_ = line_width;
    vkCmdSetLineWidth(vk_command_buffer, line_width_applied_);
  }
  if (force_set || depth_test_applied_ != depth_test) {
    depth_test_applied_ = depth_test;
    vkCmdSetDepthTestEnableEXT(vk_command_buffer, depth_test_applied_);
  }
  if (force_set || depth_write_applied_ != depth_write) {
    depth_write_applied_ = depth_write;
    vkCmdSetDepthWriteEnableEXT(vk_command_buffer, depth_write_applied_);
  }
  if (force_set || depth_compare_applied_ != depth_compare) {
    depth_compare_applied_ = depth_compare;
    vkCmdSetDepthCompareOpEXT(vk_command_buffer, depth_compare_applied_);
  }
  if (force_set || depth_bound_test_applied_ != depth_bound_test) {
    depth_bound_test_applied_ = depth_bound_test;
    vkCmdSetDepthBoundsTestEnableEXT(vk_command_buffer, depth_bound_test_applied_);
  }
  if (force_set || min_max_depth_bound_applied_ != min_max_depth_bound) {
    min_max_depth_bound_applied_ = min_max_depth_bound;
    vkCmdSetDepthBounds(vk_command_buffer, min_max_depth_bound_applied_.x, min_max_depth_bound_applied_.y);
  }
  if (force_set || stencil_test_applied_ != stencil_test) {
    stencil_test_applied_ = stencil_test;
    vkCmdSetStencilTestEnableEXT(vk_command_buffer, stencil_test_applied_);
  }
  if (force_set || front_face_applied_ != front_face || stencil_fail_op_applied_ != stencil_fail_op ||
      stencil_pass_op_applied_ != stencil_pass_op || stencil_depth_fail_op_applied_ != stencil_depth_fail_op ||
      stencil_compare_op_applied_ != stencil_compare_op) {
    stencil_face_mask_applied_ = stencil_face_mask;
    stencil_fail_op_applied_ = stencil_fail_op;
    stencil_pass_op_applied_ = stencil_pass_op;
    stencil_depth_fail_op_applied_ = stencil_depth_fail_op;
    stencil_compare_op_applied_ = stencil_compare_op;
    vkCmdSetStencilOpEXT(vk_command_buffer, stencil_face_mask_applied_, stencil_fail_op_applied_, stencil_pass_op_applied_,
                         stencil_depth_fail_op_applied_, stencil_compare_op_applied_);
  }

  if (force_set || logic_op_enable_applied_ != logic_op_enable) {
    vkCmdSetLogicOpEnableEXT(vk_command_buffer, logic_op_enable_applied_);
  }

  if (force_set || logic_op_applied_ != logic_op) {
    vkCmdSetLogicOpEXT(vk_command_buffer, logic_op_applied_);
  }
  if (color_blend_attachment_states.empty()) {
    color_blend_attachment_states.emplace_back();
  }
  std::vector<VkBool32> color_write_masks = {};
  color_write_masks.reserve(color_blend_attachment_states.size());
  for (const auto& i : color_blend_attachment_states) {
    color_write_masks.push_back(i.colorWriteMask);
  }
  if (!color_write_masks.empty())
    vkCmdSetColorWriteMaskEXT(vk_command_buffer, 0, color_write_masks.size(), color_write_masks.data());

  std::vector<VkBool32> color_blend_enable = {};
  color_blend_enable.reserve(color_blend_attachment_states.size());
  for (const auto& i : color_blend_attachment_states) {
    color_blend_enable.push_back(i.blendEnable);
  }
  if (!color_blend_enable.empty())
    vkCmdSetColorBlendEnableEXT(vk_command_buffer, 0, color_blend_enable.size(), color_blend_enable.data());

  std::vector<VkColorBlendEquationEXT> equations{};
  for (const auto& i : color_blend_attachment_states) {
    VkColorBlendEquationEXT equation;
    equation.srcColorBlendFactor = i.srcColorBlendFactor;
    equation.dstColorBlendFactor = i.dstColorBlendFactor;
    equation.colorBlendOp = i.colorBlendOp;
    equation.srcAlphaBlendFactor = i.srcAlphaBlendFactor;
    equation.dstAlphaBlendFactor = i.dstAlphaBlendFactor;
    equation.alphaBlendOp = i.alphaBlendOp;
    equations.emplace_back(equation);
  }
  if (!equations.empty())
    vkCmdSetColorBlendEquationEXT(vk_command_buffer, 0, equations.size(), equations.data());
  else {
    int a = 0;
  }
  vkCmdSetBlendConstants(vk_command_buffer, blend_constants_applied_);
}
