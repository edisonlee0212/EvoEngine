#pragma once
#include "GraphicsResources.hpp"

namespace evo_engine {
class GraphicsPipelineStates {
  friend class Platform;
  VkViewport view_port_applied_ = {};
  VkRect2D scissor_applied_ = {};

  bool depth_clamp_applied_ = false;
  bool rasterizer_discard_applied_ = false;
  VkPolygonMode polygon_mode_applied_ = VK_POLYGON_MODE_FILL;
  VkCullModeFlags cull_mode_applied_ = VK_CULL_MODE_BACK_BIT;
  VkFrontFace front_face_applied_ = VK_FRONT_FACE_CLOCKWISE;
  bool depth_bias_applied_ = false;
  glm::vec3 depth_bias_constant_clamp_slope_applied_ = glm::vec3(0.0f);
  float line_width_applied_ = 1.0f;

  bool depth_test_applied_ = true;
  bool depth_write_applied_ = true;
  VkCompareOp depth_compare_applied_ = VK_COMPARE_OP_LESS;
  bool depth_bound_test_applied_ = false;
  glm::vec2 min_max_depth_bound_applied_ = glm::vec2(-1.0f, 1.0f);
  bool stencil_test_applied_ = false;
  VkStencilFaceFlags stencil_face_mask_applied_ = VK_STENCIL_FACE_FRONT_BIT;
  VkStencilOp stencil_fail_op_applied_ = VK_STENCIL_OP_ZERO;
  VkStencilOp stencil_pass_op_applied_ = VK_STENCIL_OP_ZERO;
  VkStencilOp stencil_depth_fail_op_applied_ = VK_STENCIL_OP_ZERO;
  VkCompareOp stencil_compare_op_applied_ = VK_COMPARE_OP_LESS;

  bool logic_op_enable_applied_ = VK_FALSE;
  VkLogicOp logic_op_applied_ = VK_LOGIC_OP_COPY;

  float blend_constants_applied_[4] = {0, 0, 0, 0};

 public:
  void ResetAllStates(size_t color_attachment_size);
  VkViewport view_port = {};
  VkRect2D scissor = {};
  bool depth_clamp = false;
  bool rasterizer_discard = false;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;
  VkFrontFace front_face = VK_FRONT_FACE_CLOCKWISE;
  bool depth_bias = false;
  glm::vec3 depth_bias_constant_clamp_slope = glm::vec3(0.0f);
  float line_width = 1.0f;
  bool depth_test = true;
  bool depth_write = true;
  VkCompareOp depth_compare = VK_COMPARE_OP_LESS;
  bool depth_bound_test = false;
  glm::vec2 min_max_depth_bound = glm::vec2(0.0f, 1.0f);
  bool stencil_test = false;
  VkStencilFaceFlags stencil_face_mask = VK_STENCIL_FACE_FRONT_BIT;
  VkStencilOp stencil_fail_op = VK_STENCIL_OP_ZERO;
  VkStencilOp stencil_pass_op = VK_STENCIL_OP_ZERO;
  VkStencilOp stencil_depth_fail_op = VK_STENCIL_OP_ZERO;
  VkCompareOp stencil_compare_op = VK_COMPARE_OP_LESS;

  bool logic_op_enable = VK_FALSE;
  VkLogicOp logic_op = VK_LOGIC_OP_COPY;
  std::vector<VkPipelineColorBlendAttachmentState> color_blend_attachment_states = {};
  float blend_constants[4] = {0, 0, 0, 0};
  void ApplyAllStates(VkCommandBuffer vk_command_buffer, bool force_set = false);
};
}  // namespace evo_engine