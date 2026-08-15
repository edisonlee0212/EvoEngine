
#pragma once
#include "GraphicsResources.hpp"
#include "VulkanPipelineCache.hpp"

namespace evo_engine {

/**
 * @brief Forward declaration of the Shader class.
 */
class Shader;

/**
 * @brief A class that represents a Ray Tracing Pipeline in Vulkan.
 *
 * The RayTracingPipeline class is responsible for managing the Vulkan ray tracing pipeline,
 * including its layout, shader binding tables, and various shaders used during ray tracing.
 */
class RayTracingPipeline final : public IGraphicsResource {
  friend class Platform;

  /**
   * @brief The pipeline layout associated with the ray tracing pipeline.
   */
  std::unique_ptr<PipelineLayout> pipeline_layout_ = {};

  /**
   * @brief Vulkan ray tracing pipeline handle.
   */
  VkPipeline vk_ray_tracing_pipeline_ = VK_NULL_HANDLE;

  PipelineCreationFeedback creation_feedback_{};

  /**
   * @brief Shader binding tables for ray generation, miss, and closest-hit shaders.
   */
  std::shared_ptr<Buffer> raygen_shader_binding_table_;
  std::shared_ptr<Buffer> miss_shader_binding_table_;
  std::shared_ptr<Buffer> closest_hit_shader_binding_table_;

  /**
   * @brief Aligned size of the shader binding table handles.
   */
  uint32_t handle_size_aligned_ = 0;

  uint32_t max_recursion_depth_ = 8;
  bool linear_swept_spheres_enabled_ = false;

  void ReleaseResources();

 public:
  /**
   * @brief Destructor for the RayTracingPipeline class.
   */
  ~RayTracingPipeline() override;

  /**
   * @brief Push constant ranges used in the pipeline.
   */
  std::vector<VkPushConstantRange> push_constant_ranges;

  /**
   * @brief Descriptor set layouts used in the pipeline.
   */
  std::vector<std::shared_ptr<DescriptorSetLayout>> descriptor_set_layouts;

  /**
   * @brief Various shaders used in the ray tracing pipeline.
   */
  std::shared_ptr<Shader> raygen_shader;        ///< Ray generation shader.
  std::shared_ptr<Shader> miss_shader;          ///< Miss shader.
  std::shared_ptr<Shader> closest_hit_shader;   ///< Closest-hit shader.
  std::shared_ptr<Shader> any_hit_shader;       ///< Any-hit shader.
  std::shared_ptr<Shader> intersection_shader;  ///< Intersection shader.
  std::shared_ptr<Shader> callable_shader;      ///< Callable shader.

  /**
   * @brief Sets the maximum ray recursion depth requested during pipeline creation.
   * @param depth A non-zero depth supported by the selected physical device.
   */
  void SetMaxRecursionDepth(uint32_t depth);

  void SetLinearSweptSpheresEnabled(bool enabled);

  /**
   * @brief Checks a requested recursion depth against a physical-device limit.
   */
  [[nodiscard]] static bool IsRecursionDepthSupported(uint32_t requested_depth, uint32_t device_limit);

  /**
   * @brief Initializes the RayTracingPipeline.
   */
  void Initialize();

  /**
   * @brief Checks whether the RayTracingPipeline is initialized.
   * @return true if the pipeline is initialized, otherwise false.
   */
  [[nodiscard]] bool Initialized() const;

  [[nodiscard]] const PipelineCreationFeedback& GetCreationFeedback() const;

  /**
   * @brief Binds the ray tracing pipeline to the given command buffer.
   *
   * @param vk_command_buffer The Vulkan command buffer to bind to.
   */
  void Bind(VkCommandBuffer vk_command_buffer) const;

  /**
   * @brief Binds a descriptor set to the pipeline on the specified command buffer.
   *
   * @param vk_command_buffer The Vulkan command buffer to bind to.
   * @param first_set The first set number to bind.
   * @param descriptor_set The descriptor set to bind.
   */
  void BindDescriptorSet(VkCommandBuffer vk_command_buffer, uint32_t first_set, VkDescriptorSet descriptor_set) const;

  /**
   * @brief Pushes constant data to the pipeline.
   *
   * @tparam T Type of the constant data to be pushed.
   * @param vk_command_buffer The Vulkan command buffer used for the operation.
   * @param range_index The index of the push constant range.
   * @param data The constant data to be pushed.
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

  /**
   * @brief Dispatches a ray tracing command.
   *
   * @param vk_command_buffer The Vulkan command buffer to record the command.
   * @param x Number of threads in the x-dimension.
   * @param y Number of threads in the y-dimension.
   * @param z Number of threads in the z-dimension.
   */
  void Trace(VkCommandBuffer vk_command_buffer, uint32_t x, uint32_t y, uint32_t z) const;
};

/**
 * @brief Template implementation for pushing constant data into a specified range.
 *
 * @tparam T Type of the constant data to be pushed.
 * @param vk_command_buffer The Vulkan command buffer used for the operation.
 * @param range_index The index of the push constant range.
 * @param data The constant data to be pushed.
 */
template <typename T>
void RayTracingPipeline::PushConstant(const VkCommandBuffer vk_command_buffer, const size_t range_index,
                                      const T& data) {
  PushConstantData(vk_command_buffer, range_index, &data);
}

}  // namespace evo_engine
