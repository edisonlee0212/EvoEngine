#pragma once
#include "GraphicsResources.hpp"
#include "Transform.hpp"
#include "Vertex.hpp"
namespace evo_engine {
class GraphicsPipelineStates;

enum class GeometryType { Mesh, SkinnedMesh, Strands };
class IGeometry {
 public:
  virtual void DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                           int instance_count) const = 0;
  static const std::vector<VkVertexInputBindingDescription>& GetVertexBindingDescriptions(GeometryType geometry_type);
  static const std::vector<VkVertexInputAttributeDescription>& GetVertexAttributeDescriptions(
      GeometryType geometry_type);
};
}  // namespace evo_engine