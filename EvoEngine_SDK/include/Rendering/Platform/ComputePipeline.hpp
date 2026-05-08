
#pragma once
#include "GraphicsResources.hpp"

namespace evo_engine {

/**
 * @class ComputePipeline
 * @brief Represents a compute pipeline in a Vulkan-based graphics application.
 *
 * This class manages resources and functionality for a Vulkan compute pipeline,
 * including descriptor sets, push constants, and shader configurations.
 */
class Shader;
class ComputePipeline final : public IGraphicsResource {
  friend class Platform;

  /// Unique pointer to the pipeline layout.
  std::unique_ptr<PipelineLayout> pipeline_layout_ = {};

  /// Vulkan handle for the compute pipeline.
  VkPipeline vk_compute_pipeline_ = VK_NULL_HANDLE;

 public:
  /**
   * @brief Destructor for ComputePipeline.
   */
  ~ComputePipeline() override;

  /// Mapping entries used for the pipeline.
  std::vector<int32_t> map_entries;

  /// Push constant ranges associated with this compute pipeline.
  std::vector<VkPushConstantRange> push_constant_ranges;

  /// Descriptor set layouts used by this compute pipeline.
  std::vector<std::shared_ptr<DescriptorSetLayout>> descriptor_set_layouts;

  /// Compute shader associated with this pipeline.
  std::shared_ptr<Shader> compute_shader;

  /**
   * @brief Initializes the compute pipeline.
   */
  void Initialize();

  /**
   * @brief Checks if the compute pipeline has been initialized.
   *
   * @return True if the pipeline is initialized, otherwise false.
   */
  [[nodiscard]] bool Initialized() const;

  /**
   * @brief Binds the compute pipeline to a command buffer.
   *
   * @param vk_command_buffer The Vulkan command buffer to bind the pipeline to.
   */
  void Bind(VkCommandBuffer vk_command_buffer) const;

  /**
   * @brief Binds a descriptor set to the command buffer.
   *
   * @param vk_command_buffer The Vulkan command buffer to bind the descriptor set to.
   * @param first_set The starting index of the descriptor set.
   * @param descriptor_set The descriptor set to bind.
   */
  void BindDescriptorSet(VkCommandBuffer vk_command_buffer, uint32_t first_set, VkDescriptorSet descriptor_set) const;

  /**
   * @brief Records a compute dispatch through the SDK binary.
   *
   * @param vk_command_buffer The Vulkan command buffer.
   * @param x Workgroup count in the x dimension.
   * @param y Workgroup count in the y dimension.
   * @param z Workgroup count in the z dimension.
   */
  void Dispatch(VkCommandBuffer vk_command_buffer, uint32_t x, uint32_t y = 1, uint32_t z = 1) const;

  /**
   * @brief Updates the push constants of the pipeline.
   *
   * @tparam T The type of data to push.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param range_index The index of the push constant range to update.
   * @param data The data to push to the pipeline.
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

/**
 * @brief Template definition for PushConstant that updates push constants in the pipeline.
 *
 * @tparam T The type of data being pushed.
 * @param vk_command_buffer The Vulkan command buffer.
 * @param range_index The index of the push constant range to update.
 * @param data The data to push to the pipeline.
 */
template <typename T>
void ComputePipeline::PushConstant(const VkCommandBuffer vk_command_buffer, const size_t range_index, const T& data) {
  PushConstantData(vk_command_buffer, range_index, &data);
}

}  // namespace evo_engine
