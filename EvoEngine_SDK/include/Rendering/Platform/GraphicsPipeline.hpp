
/**
 * @file GraphicsPipeline.hpp
 * @brief Defines the classes and structures related to graphics pipeline configuration and usage.
 */

#pragma once
#include "GraphicsPipelineStates.hpp"
#include "GraphicsResources.hpp"
#include "IGeometry.hpp"
#include "VulkanPipelineCache.hpp"

namespace evo_engine {

class EVOENGINE_API Shader;

#pragma region Pipeline Data

/**
 * @struct PipelineShaderStage
 * @brief Represents a single shader stage in a graphics pipeline.
 */
struct EVOENGINE_API PipelineShaderStage {
  VkPipelineShaderStageCreateFlags flags;                   ///< Shader stage creation flags.
  VkShaderStageFlagBits stage;                              ///< Type of shader stage (e.g., vertex, fragment).
  VkShaderModule module;                                    ///< Vulkan shader module handle.
  std::string name;                                         ///< Name of the shader stage.
  std::optional<VkSpecializationInfo> specialization_info;  ///< Optional specialization constants info.

  /**
   * @brief Applies the shader stage settings to a Vulkan pipeline shader stage create info structure.
   * @param vk_pipeline_shader_stage_create_info The Vulkan pipeline shader stage create info structure to populate.
   */
  void Apply(const VkPipelineShaderStageCreateInfo& vk_pipeline_shader_stage_create_info);
};

/**
 * @struct PipelineVertexInputState
 * @brief Represents the vertex input state in a graphics pipeline.
 */
struct EVOENGINE_API PipelineVertexInputState {
  VkPipelineVertexInputStateCreateFlags flags;                                   ///< Vertex input state creation flags.
  std::vector<VkVertexInputBindingDescription> vertex_binding_descriptions;      ///< Vertex binding descriptions.
  std::vector<VkVertexInputAttributeDescription> vertex_attribute_descriptions;  ///< Vertex attribute descriptions.

  /**
   * @brief Applies the vertex input state settings to a Vulkan pipeline vertex input state create info structure.
   * @param vk_pipeline_shader_stage_create_info The Vulkan pipeline vertex input state create info structure to
   * populate.
   */
  void Apply(const VkPipelineVertexInputStateCreateInfo& vk_pipeline_shader_stage_create_info);
};

/**
 * @struct PipelineInputAssemblyState
 * @brief Describes the input assembly state of the graphics pipeline.
 */
struct EVOENGINE_API PipelineInputAssemblyState {
  VkPipelineInputAssemblyStateCreateFlags flags;  ///< Input assembly state creation flags.
  VkPrimitiveTopology topology;                   ///< Topology of the primitives (e.g., triangle list, line list).
  VkBool32 primitive_restart_enable;              ///< Enables primitive restart.

  /**
   * @brief Applies the input assembly state settings to a Vulkan pipeline input assembly state create info structure.
   * @param vk_pipeline_input_assembly_state_create_info The Vulkan pipeline input assembly state create info structure
   * to populate.
   */
  void Apply(const VkPipelineInputAssemblyStateCreateInfo& vk_pipeline_input_assembly_state_create_info);
};

/**
 * @struct PipelineTessellationState
 * @brief Represents the tessellation state of a graphics pipeline.
 */
struct EVOENGINE_API PipelineTessellationState {
  VkPipelineTessellationStateCreateFlags flags;  ///< Tessellation state creation flags.
  uint32_t patch_control_points;                 ///< Number of control points per patch.

  /**
   * @brief Applies the tessellation state settings to a Vulkan pipeline tessellation state create info structure.
   * @param vk_pipeline_tessellation_state_create_info The Vulkan pipeline tessellation state create info structure to
   * populate.
   */
  void Apply(const VkPipelineTessellationStateCreateInfo& vk_pipeline_tessellation_state_create_info);
};

/**
 * @struct PipelineViewportState
 * @brief Defines the viewport and scissor state of a graphics pipeline.
 */
struct EVOENGINE_API PipelineViewportState {
  VkPipelineViewportStateCreateFlags flags;  ///< Viewport state creation flags.
  std::vector<VkViewport> viewports;         ///< List of viewports.
  std::vector<VkRect2D> scissors;            ///< List of scissor rectangles.

  /**
   * @brief Applies the viewport state settings to a Vulkan pipeline viewport state create info structure.
   * @param vk_pipeline_viewport_state_create_info The Vulkan pipeline viewport state create info structure to populate.
   */
  void Apply(const VkPipelineViewportStateCreateInfo& vk_pipeline_viewport_state_create_info);
};

/**
 * @struct PipelineRasterizationState
 * @brief Represents the rasterization state in a graphics pipeline.
 */
struct EVOENGINE_API PipelineRasterizationState {
  VkPipelineRasterizationStateCreateFlags flags;  ///< Rasterization state creation flags.
  VkBool32 depth_clamp_enable;                    ///< Enables depth clamping.
  VkBool32 rasterizer_discard_enable;             ///< Disables rasterization (no primitives are produced).
  VkPolygonMode polygon_mode;                     ///< Polygon mode (e.g., fill, line, point).
  VkCullModeFlags cull_mode;                      ///< Culling mode (e.g., back, front, none).
  VkFrontFace front_face;                         ///< Front face winding order.
  VkBool32 depth_bias_enable;                     ///< Enables depth bias.
  float depth_bias_constant_factor;               ///< Constant depth bias factor.
  float depth_bias_clamp;                         ///< Depth bias clamp value.
  float depth_bias_slope_factor;                  ///< Slope scaled depth bias factor.
  float line_width;                               ///< Line width for line rasterization.

  /**
   * @brief Applies the rasterization state settings to a Vulkan pipeline rasterization state create info structure.
   * @param vk_pipeline_rasterization_state_create_info The Vulkan pipeline rasterization state create info structure to
   * populate.
   */
  void Apply(const VkPipelineRasterizationStateCreateInfo& vk_pipeline_rasterization_state_create_info);
};

/**
 * @struct PipelineMultisampleState
 * @brief Describes the multi-sampling state of the graphics pipeline.
 */
struct EVOENGINE_API PipelineMultisampleState {
  VkPipelineMultisampleStateCreateFlags flags;  ///< Multisample state creation flags.
  VkSampleCountFlagBits rasterization_samples;  ///< Number of rasterization samples.
  VkBool32 sample_shading_enable;               ///< Enables sample shading.
  float min_sample_shading;                     ///< Minimum fraction of shading samples.
  std::optional<VkSampleMask> sample_mask;      ///< Optional sample mask.
  VkBool32 alpha_to_coverage_enable;            ///< Enables alpha-to-coverage.
  VkBool32 alpha_to_one_enable;                 ///< Enables alpha-to-one for transparency.

  /**
   * @brief Applies the multisample state settings to a Vulkan pipeline multisample state create info structure.
   * @param vk_pipeline_multisample_state_create_info The Vulkan pipeline multisample state create info structure to
   * populate.
   */
  void Apply(const VkPipelineMultisampleStateCreateInfo& vk_pipeline_multisample_state_create_info);
};

/**
 * @struct PipelineDepthStencilState
 * @brief Represents the depth and stencil test state of the graphics pipeline.
 */
struct EVOENGINE_API PipelineDepthStencilState {
  VkPipelineDepthStencilStateCreateFlags flags;  ///< Depth-stencil state creation flags.
  VkBool32 depth_test_enable;                    ///< Enables depth testing.
  VkBool32 depth_write_enable;                   ///< Enables depth writing.
  VkCompareOp depth_compare_op;                  ///< Depth comparison operation.
  VkBool32 depth_bounds_test_enable;             ///< Enables depth bounds test.
  VkBool32 stencil_test_enable;                  ///< Enables stencil testing.
  VkStencilOpState front;                        ///< Stencil operations for front-facing primitives.
  VkStencilOpState back;                         ///< Stencil operations for back-facing primitives.
  float min_depth_bounds;                        ///< Minimum depth bounds.
  float max_depth_bounds;                        ///< Maximum depth bounds.

  /**
   * @brief Applies the depth-stencil state settings to a Vulkan pipeline depth-stencil state create info structure.
   * @param vk_pipeline_depth_stencil_state_create_info The Vulkan pipeline depth-stencil state create info structure to
   * populate.
   */
  void Apply(const VkPipelineDepthStencilStateCreateInfo& vk_pipeline_depth_stencil_state_create_info);
};

/**
 * @struct PipelineColorBlendState
 * @brief Describes the color blending state of the graphics pipeline.
 */
struct EVOENGINE_API PipelineColorBlendState {
  VkPipelineColorBlendStateCreateFlags flags;                    ///< Color blend state creation flags.
  VkBool32 logic_op_enable;                                      ///< Enables logical operations.
  VkLogicOp logic_op;                                            ///< Logical operation to apply.
  std::vector<VkPipelineColorBlendAttachmentState> attachments;  ///< Per-attachment blending state.
  float blend_constants[4];                                      ///< Blend constants.

  /**
   * @brief Applies the color blend state to a Vulkan pipeline color blend state create info structure.
   * @param vk_pipeline_color_blend_state_create_info The Vulkan pipeline color blend state create info structure to
   * populate.
   */
  void Apply(const VkPipelineColorBlendStateCreateInfo& vk_pipeline_color_blend_state_create_info);
};

/**
 * @struct PipelineDynamicState
 * @brief Configures dynamic states for the graphics pipeline.
 */
struct EVOENGINE_API PipelineDynamicState {
  VkPipelineDynamicStateCreateFlags flags;     ///< Dynamic state creation flags.
  std::vector<VkDynamicState> dynamic_states;  ///< List of dynamic states.

  /**
   * @brief Applies the dynamic state settings to a Vulkan pipeline dynamic state create info structure.
   * @param vk_pipeline_dynamic_state_create_info The Vulkan pipeline dynamic state create info structure to populate.
   */
  void Apply(const VkPipelineDynamicStateCreateInfo& vk_pipeline_dynamic_state_create_info);
};

#pragma endregion

/**
 * @class GraphicsPipeline
 * @brief Manages a Vulkan graphics pipeline and its associated resources.
 */
class EVOENGINE_API GraphicsPipeline final : public IGraphicsResource {
  friend class Platform;
  friend class RenderLayer;
  friend class GraphicsPipelineStates;

  std::unique_ptr<PipelineLayout> pipeline_layout_ = {};  ///< Pipeline layout.

  VkPipeline vk_graphics_pipeline_ = VK_NULL_HANDLE;  ///< Vulkan graphics pipeline handle.

  PipelineCreationFeedback creation_feedback_{};

 public:
  /**
   * @brief Destroys the graphics pipeline and releases resources.
   */
  ~GraphicsPipeline() override;

  GraphicsPipelineStates states{};  ///< Graphics pipeline states.

  std::vector<std::shared_ptr<DescriptorSetLayout>>
      descriptor_set_layouts;  ///< Descriptor set layouts used by the pipeline.

  std::shared_ptr<Shader> vertex_shader;                   ///< Vertex shader.
  std::shared_ptr<Shader> tessellation_control_shader;     ///< Tessellation control shader.
  std::shared_ptr<Shader> tessellation_evaluation_shader;  ///< Tessellation evaluation shader.
  std::shared_ptr<Shader> geometry_shader;                 ///< Geometry shader.

  std::shared_ptr<Shader> task_shader;              ///< Task shader.
  std::shared_ptr<Shader> mesh_shader;              ///< Mesh shader.
  std::shared_ptr<Shader> fragment_shader;          ///< Fragment shader.
  GeometryType geometry_type = GeometryType::Mesh;  ///< Type of geometry used by the pipeline.
  VertexInputAttributeSet vertex_input_attribute_set =
      VertexInputAttributeSet::Full;  ///< Vertex attributes consumed by the pipeline.
  bool vertex_input_enabled = true;
  VkPrimitiveTopology primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

  uint32_t view_mask;                              ///< View mask used for multiview rendering.
  std::vector<VkFormat> color_attachment_formats;  ///< Formats for color attachments.
  VkFormat depth_attachment_format;                ///< Format for the depth attachment.
  VkFormat stencil_attachment_format;              ///< Format for the stencil attachment.

  uint32_t tessellation_patch_control_points = 4;  ///< Number of patch control points for tessellation.

  std::vector<VkPushConstantRange> push_constant_ranges;  ///< Push constant ranges used in the pipeline.

  /**
   * @brief Initializes the graphics pipeline.
   */
  void Initialize();

  /**
   * @brief Checks if the pipeline has been initialized.
   * @return True if initialized, false otherwise.
   */
  [[nodiscard]] bool Initialized() const;

  [[nodiscard]] const PipelineCreationFeedback& GetCreationFeedback() const;

  /**
   * @brief Binds the graphics pipeline to a command buffer.
   * @param vk_command_buffer The Vulkan command buffer.
   */
  void Bind(VkCommandBuffer vk_command_buffer);

  /**
   * @brief Binds a descriptor set to a command buffer.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param first_set The first set number to bind.
   * @param descriptor_set The Vulkan descriptor set to bind.
   */
  void BindDescriptorSet(VkCommandBuffer vk_command_buffer, uint32_t first_set, VkDescriptorSet descriptor_set) const;

  /**
   * @brief Records a mesh shader task draw through the SDK binary.
   *
   * @param vk_command_buffer The Vulkan command buffer.
   * @param x Task count in the x dimension.
   * @param y Task count in the y dimension.
   * @param z Task count in the z dimension.
   */
  void DrawMeshTasks(VkCommandBuffer vk_command_buffer, uint32_t x, uint32_t y = 1, uint32_t z = 1) const;

  /**
   * @brief Pushes a constant value to the pipeline.
   *
   * @tparam T The type of data to push.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param range_index Index of the push constant range.
   * @param data The data to push.
   */
  template <typename T>
  void PushConstant(VkCommandBuffer vk_command_buffer, size_t range_index, const T& data);

  /**
   * @brief Pushes raw constant data through the SDK binary.
   *
   * This keeps the Vulkan command call inside EvoEngine_SDK so runtime packages do not depend on their own volk
   * function table being loaded.
   */
  void PushConstantData(VkCommandBuffer vk_command_buffer, size_t range_index, const void* data) const;
};

template <typename T>
void GraphicsPipeline::PushConstant(const VkCommandBuffer vk_command_buffer, const size_t range_index, const T& data) {
  PushConstantData(vk_command_buffer, range_index, &data);
}

}  // namespace evo_engine
