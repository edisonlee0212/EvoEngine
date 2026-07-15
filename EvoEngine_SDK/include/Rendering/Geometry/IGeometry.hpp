
#pragma once
#include "GraphicsResources.hpp"
#include "Transform.hpp"
#include "Vertex.hpp"

namespace evo_engine {
class GraphicsPipelineStates;

/**
 * @brief Specifies the type of geometry.
 */
enum class GeometryType {
  Mesh,        ///< Static mesh geometry.
  SkinnedMesh  ///< Skinned mesh geometry (e.g., animated).
};

/**
 * @brief Interface for geometry objects in the engine.
 */
class IGeometry {
 public:
  /**
   * @brief Virtual destructor for IGeometry.
   */
  virtual ~IGeometry() = default;

  /**
   * @brief Draws the geometry using indexed rendering.
   *
   * @param vk_command_buffer The Vulkan command buffer used for the rendering commands.
   * @param global_pipeline_state The global graphics pipeline state to use for the rendering.
   * @param instance_count The number of instances to draw.
   */
  virtual void DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                           int instance_count) const = 0;

  /**
   * @brief Gets the vertex binding descriptions for the specified geometry type.
   *
   * @param geometry_type The type of geometry (e.g., Mesh or SkinnedMesh).
   * @return A reference to a vector of vertex input binding descriptions.
   * These describe how vertex buffer data is organized.
   */
  static const std::vector<VkVertexInputBindingDescription>& GetVertexBindingDescriptions(GeometryType geometry_type);

  /**
   * @brief Gets the vertex attribute descriptions for the specified geometry type.
   *
   * @param geometry_type The type of geometry (e.g., Mesh or SkinnedMesh).
   * @return A reference to a vector of vertex input attribute descriptions.
   * These define how vertex attributes (e.g., position, normal, UV) are fetched.
   */
  static const std::vector<VkVertexInputAttributeDescription>& GetVertexAttributeDescriptions(
      GeometryType geometry_type);
};

}  // namespace evo_engine
