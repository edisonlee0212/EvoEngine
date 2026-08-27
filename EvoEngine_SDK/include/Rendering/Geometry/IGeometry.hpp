
#pragma once
#include "GraphicsResources.hpp"
#include "Transform.hpp"
#include "Vertex.hpp"

namespace evo_engine {
class EVOENGINE_API GraphicsPipelineStates;

/**
 * @brief Specifies the type of geometry.
 */
enum class GeometryType {
  Mesh,        ///< Static mesh geometry.
  SkinnedMesh  ///< Skinned mesh geometry (e.g., animated).
};

/**
 * @brief Specifies how many attributes a graphics pipeline consumes from a geometry vertex buffer.
 */
enum class VertexInputAttributeSet {
  Full,              ///< All material-facing vertex attributes.
  Base,              ///< Position/normal/tangent/UV/color, plus skinning attributes for skinned meshes.
  Position,          ///< Position only, plus skinning attributes for skinned meshes.
  PositionNormal,    ///< Position and normal, plus skinning attributes for skinned meshes.
  PositionColor,     ///< Position and vertex color.
  PositionTexCoord,  ///< Position and primary UV.
  MotionVectors      ///< Motion-vector pass attributes, excluding tangent handedness.
};

/**
 * @brief Interface for geometry objects in the engine.
 */
class EVOENGINE_API IGeometry {
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
   * @param attribute_set The subset of vertex attributes consumed by the shader.
   * @return A reference to a vector of vertex input attribute descriptions.
   * These define how vertex attributes (e.g., position, normal, UV) are fetched.
   */
  static const std::vector<VkVertexInputAttributeDescription>& GetVertexAttributeDescriptions(
      GeometryType geometry_type, VertexInputAttributeSet attribute_set = VertexInputAttributeSet::Full);
};

}  // namespace evo_engine
