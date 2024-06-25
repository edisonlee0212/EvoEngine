#include "IGeometry.hpp"

using namespace evo_engine;
const std::vector<VkVertexInputBindingDescription>& IGeometry::GetVertexBindingDescriptions(
    const GeometryType geometry_type) {
  static std::vector<VkVertexInputBindingDescription> mesh{};
  static std::vector<VkVertexInputBindingDescription> skinned_mesh{};
  static std::vector<VkVertexInputBindingDescription> strands{};
  if (mesh.empty()) {
    mesh.resize(1);
    mesh[0].binding = 0;
    mesh[0].stride = sizeof(Vertex);
    mesh[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
  }
  if (skinned_mesh.empty()) {
    skinned_mesh.resize(1);
    skinned_mesh[0].binding = 0;
    skinned_mesh[0].stride = sizeof(SkinnedVertex);
    skinned_mesh[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
  }
  if (strands.empty()) {
    strands.resize(1);
    strands[0].binding = 0;
    strands[0].stride = sizeof(StrandPoint);
    strands[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
  }
  switch (geometry_type) {
    case GeometryType::Mesh:
      return mesh;
    case GeometryType::SkinnedMesh:
      return skinned_mesh;
    case GeometryType::Strands:
      return strands;
  }
  throw std::runtime_error("Unhandled geometry type!");
}

const std::vector<VkVertexInputAttributeDescription>& IGeometry::GetVertexAttributeDescriptions(
    const GeometryType geometry_type) {
  static std::vector<VkVertexInputAttributeDescription> mesh{};
  static std::vector<VkVertexInputAttributeDescription> skinned_mesh{};
  static std::vector<VkVertexInputAttributeDescription> strands{};
  if (mesh.empty()) {
    mesh.resize(5);
    mesh[0].binding = 0;
    mesh[0].location = 0;
    mesh[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    mesh[0].offset = offsetof(Vertex, position);

    mesh[1].binding = 0;
    mesh[1].location = 1;
    mesh[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    mesh[1].offset = offsetof(Vertex, normal);

    mesh[2].binding = 0;
    mesh[2].location = 2;
    mesh[2].format = VK_FORMAT_R32G32B32_SFLOAT;
    mesh[2].offset = offsetof(Vertex, tangent);

    mesh[3].binding = 0;
    mesh[3].location = 3;
    mesh[3].format = VK_FORMAT_R32G32_SFLOAT;
    mesh[3].offset = offsetof(Vertex, tex_coord);

    mesh[4].binding = 0;
    mesh[4].location = 4;
    mesh[4].format = VK_FORMAT_R32G32B32A32_SFLOAT;
    mesh[4].offset = offsetof(Vertex, color);
  }

  if (skinned_mesh.empty()) {
    skinned_mesh.resize(9);
    skinned_mesh[0].binding = 0;
    skinned_mesh[0].location = 0;
    skinned_mesh[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    skinned_mesh[0].offset = offsetof(SkinnedVertex, position);

    skinned_mesh[1].binding = 0;
    skinned_mesh[1].location = 1;
    skinned_mesh[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    skinned_mesh[1].offset = offsetof(SkinnedVertex, normal);

    skinned_mesh[2].binding = 0;
    skinned_mesh[2].location = 2;
    skinned_mesh[2].format = VK_FORMAT_R32G32B32_SFLOAT;
    skinned_mesh[2].offset = offsetof(SkinnedVertex, tangent);

    skinned_mesh[3].binding = 0;
    skinned_mesh[3].location = 3;
    skinned_mesh[3].format = VK_FORMAT_R32G32_SFLOAT;
    skinned_mesh[3].offset = offsetof(SkinnedVertex, tex_coord);

    skinned_mesh[4].binding = 0;
    skinned_mesh[4].location = 4;
    skinned_mesh[4].format = VK_FORMAT_R32G32B32A32_SFLOAT;
    skinned_mesh[4].offset = offsetof(SkinnedVertex, color);

    skinned_mesh[5].binding = 0;
    skinned_mesh[5].location = 5;
    skinned_mesh[5].format = VK_FORMAT_R32G32B32A32_SINT;
    skinned_mesh[5].offset = offsetof(SkinnedVertex, bond_id);

    skinned_mesh[6].binding = 0;
    skinned_mesh[6].location = 6;
    skinned_mesh[6].format = VK_FORMAT_R32G32B32A32_SFLOAT;
    skinned_mesh[6].offset = offsetof(SkinnedVertex, weight);

    skinned_mesh[7].binding = 0;
    skinned_mesh[7].location = 7;
    skinned_mesh[7].format = VK_FORMAT_R32G32B32A32_SINT;
    skinned_mesh[7].offset = offsetof(SkinnedVertex, bond_id2);

    skinned_mesh[8].binding = 0;
    skinned_mesh[8].location = 8;
    skinned_mesh[8].format = VK_FORMAT_R32G32B32A32_SFLOAT;
    skinned_mesh[8].offset = offsetof(SkinnedVertex, weight2);
  }

  if (strands.empty()) {
    strands.resize(5);
    strands[0].binding = 0;
    strands[0].location = 0;
    strands[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    strands[0].offset = offsetof(StrandPoint, position);

    strands[1].binding = 0;
    strands[1].location = 1;
    strands[1].format = VK_FORMAT_R32_SFLOAT;
    strands[1].offset = offsetof(StrandPoint, thickness);

    strands[2].binding = 0;
    strands[2].location = 2;
    strands[2].format = VK_FORMAT_R32G32B32_SFLOAT;
    strands[2].offset = offsetof(StrandPoint, normal);

    strands[3].binding = 0;
    strands[3].location = 3;
    strands[3].format = VK_FORMAT_R32_SFLOAT;
    strands[3].offset = offsetof(StrandPoint, tex_coord);

    strands[4].binding = 0;
    strands[4].location = 4;
    strands[4].format = VK_FORMAT_R32G32B32A32_SFLOAT;
    strands[4].offset = offsetof(StrandPoint, color);
  }
  switch (geometry_type) {
    case GeometryType::Mesh:
      return mesh;
    case GeometryType::SkinnedMesh:
      return skinned_mesh;
    case GeometryType::Strands:
      return strands;
  }
  throw std::runtime_error("Unhandled geometry type!");
}