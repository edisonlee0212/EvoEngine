
#pragma once
#include "GraphicsResources.hpp"

namespace evo_engine {

/**
 * @class GraphicsPipelineStates
 * @brief Manages and applies the graphics pipeline states for rendering.
 */
class GraphicsPipelineStates {
  friend class Platform;  ///< Platform has access to private members for management purposes.

  VkViewport view_port_applied_ = {};  ///< Applied viewport configuration.
  VkRect2D scissor_applied_ = {};      ///< Applied scissor rectangle configuration.

  bool depth_clamp_applied_ = false;                           ///< Tracks if depth clamping is applied.
  bool rasterizer_discard_applied_ = false;                    ///< Tracks if rasterizer discard is applied.
  VkPolygonMode polygon_mode_applied_ = VK_POLYGON_MODE_FILL;  ///< Applied polygon mode.
  VkCullModeFlags cull_mode_applied_ = VK_CULL_MODE_BACK_BIT;  ///< Applied culling mode.
  VkFrontFace front_face_applied_ = VK_FRONT_FACE_CLOCKWISE;   ///< Applied front face winding order.
  bool depth_bias_applied_ = false;                            ///< Tracks if depth bias is applied.
  glm::vec3 depth_bias_constant_clamp_slope_applied_ =
      glm::vec3(0.0f);               ///< Applied depth bias parameters (constant, clamp, slope).
  float line_width_applied_ = 1.0f;  ///< Applied line width for rendering.

  bool depth_test_applied_ = true;                                  ///< Tracks if depth testing is applied.
  bool depth_write_applied_ = true;                                 ///< Tracks if depth writing is enabled.
  VkCompareOp depth_compare_applied_ = VK_COMPARE_OP_LESS;          ///< Applied depth comparison operation.
  bool depth_bound_test_applied_ = false;                           ///< Tracks if depth bounds test is applied.
  glm::vec2 min_max_depth_bound_applied_ = glm::vec2(-1.0f, 1.0f);  ///< Applied minimum and maximum depth bounds.
  bool stencil_test_applied_ = false;                               ///< Tracks if stencil testing is applied.
  VkStencilFaceFlags stencil_face_mask_applied_ = VK_STENCIL_FACE_FRONT_BIT;  ///< Applied stencil face mask.
  VkStencilOp stencil_fail_op_applied_ = VK_STENCIL_OP_ZERO;                  ///< Applied stencil fail operation.
  VkStencilOp stencil_pass_op_applied_ = VK_STENCIL_OP_ZERO;                  ///< Applied stencil pass operation.
  VkStencilOp stencil_depth_fail_op_applied_ = VK_STENCIL_OP_ZERO;            ///< Applied stencil depth fail operation.
  VkCompareOp stencil_compare_op_applied_ = VK_COMPARE_OP_LESS;               ///< Applied stencil comparison operation.

  bool logic_op_enable_applied_ = VK_FALSE;        ///< Tracks if logic operations are enabled.
  VkLogicOp logic_op_applied_ = VK_LOGIC_OP_COPY;  ///< Applied logic operation.

  float blend_constants_applied_[4] = {0, 0, 0, 0};  ///< Applied blend constants.

 public:
  /**
   * @brief Resets all graphics pipeline states to their default values.
   * @param color_attachment_size The size of the color attachment array.
   */
  void ResetAllStates(size_t color_attachment_size);

  VkViewport view_port = {};                          ///< Configured viewport settings.
  VkRect2D scissor = {};                              ///< Configured scissor rectangle.
  bool depth_clamp = false;                           ///< Configured depth clamping.
  bool rasterizer_discard = false;                    ///< Configured rasterizer discard state.
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;  ///< Configured polygon mode.
  VkCullModeFlags cull_mode = VK_CULL_MODE_BACK_BIT;  ///< Configured culling mode.
  VkFrontFace front_face = VK_FRONT_FACE_CLOCKWISE;   ///< Configured front-face winding order.
  bool depth_bias = false;                            ///< Configured depth bias state.
  glm::vec3 depth_bias_constant_clamp_slope =
      glm::vec3(0.0f);      ///< Configured depth bias parameters (constant, clamp, slope).
  float line_width = 1.0f;  ///< Configured line width for rendering.

  bool depth_test = true;                                            ///< Configured depth testing state.
  bool depth_write = true;                                           ///< Configured depth writing state.
  VkCompareOp depth_compare = VK_COMPARE_OP_LESS;                    ///< Configured depth comparison operation.
  bool depth_bound_test = false;                                     ///< Configured depth bounds test state.
  glm::vec2 min_max_depth_bound = glm::vec2(0.0f, 1.0f);             ///< Configured minimum and maximum depth bounds.
  bool stencil_test = false;                                         ///< Configured stencil testing state.
  VkStencilFaceFlags stencil_face_mask = VK_STENCIL_FACE_FRONT_BIT;  ///< Configured stencil face mask.
  VkStencilOp stencil_fail_op = VK_STENCIL_OP_ZERO;                  ///< Configured stencil fail operation.
  VkStencilOp stencil_pass_op = VK_STENCIL_OP_ZERO;                  ///< Configured stencil pass operation.
  VkStencilOp stencil_depth_fail_op = VK_STENCIL_OP_ZERO;            ///< Configured stencil depth fail operation.
  VkCompareOp stencil_compare_op = VK_COMPARE_OP_LESS;               ///< Configured stencil comparison operation.

  bool logic_op_enable = VK_FALSE;        ///< Configured logic operation enable state.
  VkLogicOp logic_op = VK_LOGIC_OP_COPY;  ///< Configured logic operation.
  std::vector<VkPipelineColorBlendAttachmentState> color_blend_attachment_states =
      {};                                   ///< Configured color blend attachment states.
  float blend_constants[4] = {0, 0, 0, 0};  ///< Configured blend constants.

  /**
   * @brief Applies all configured states to the provided Vulkan command buffer.
   * @param vk_command_buffer The Vulkan command buffer to apply the states to.
   * @param force_set If true, forces the states to be applied even if they are already set.
   */
  void ApplyAllStates(VkCommandBuffer vk_command_buffer, bool force_set = false);

  /**
   * @brief Sets the viewport and scissor rect based on specified dimensions and depth range.
   * @param value The viewport and scissor rectangle dimensions (x, y, width, height).
   * @param min_depth The minimum depth value (default is 0.0f).
   * @param max_depth The maximum depth value (default is 1.0f).
   */
  void SetViewportScissor(const glm::ivec4& value, float min_depth = 0.0f, float max_depth = 1.0f);
};

}  // namespace evo_engine
