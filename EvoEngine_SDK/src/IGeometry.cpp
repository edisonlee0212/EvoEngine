#include "IGeometry.hpp"

using namespace evo_engine;
const std::vector<VkVertexInputBindingDescription>& IGeometry::GetVertexBindingDescriptions(
    const GeometryType geometry_type) {
  static std::vector<VkVertexInputBindingDescription> mesh{};
  static std::vector<VkVertexInputBindingDescription> skinned_mesh{};
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
  switch (geometry_type) {
    case GeometryType::Mesh:
      return mesh;
    case GeometryType::SkinnedMesh:
      return skinned_mesh;
  }
  throw std::runtime_error("Unhandled geometry type!");
}

const std::vector<VkVertexInputAttributeDescription>& IGeometry::GetVertexAttributeDescriptions(
    const GeometryType geometry_type, const VertexInputAttributeSet attribute_set) {
  static std::vector<VkVertexInputAttributeDescription> mesh{};
  static std::vector<VkVertexInputAttributeDescription> skinned_mesh{};
  static std::vector<VkVertexInputAttributeDescription> mesh_base{};
  static std::vector<VkVertexInputAttributeDescription> skinned_mesh_base{};
  static std::vector<VkVertexInputAttributeDescription> mesh_position{};
  static std::vector<VkVertexInputAttributeDescription> skinned_mesh_position{};
  static std::vector<VkVertexInputAttributeDescription> mesh_position_normal{};
  static std::vector<VkVertexInputAttributeDescription> skinned_mesh_position_normal{};
  static std::vector<VkVertexInputAttributeDescription> mesh_position_color{};
  static std::vector<VkVertexInputAttributeDescription> skinned_mesh_position_color{};
  static std::vector<VkVertexInputAttributeDescription> mesh_position_tex_coord{};
  static std::vector<VkVertexInputAttributeDescription> skinned_mesh_position_tex_coord{};
  static std::vector<VkVertexInputAttributeDescription> mesh_motion_vectors{};
  static std::vector<VkVertexInputAttributeDescription> skinned_mesh_motion_vectors{};
  if (mesh.empty()) {
    mesh.resize(7);
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

    mesh[5].binding = 0;
    mesh[5].location = 9;
    mesh[5].format = VK_FORMAT_R32_SFLOAT;
    mesh[5].offset = offsetof(Vertex, vertex_info3);

    mesh[6].binding = 0;
    mesh[6].location = 10;
    mesh[6].format = VK_FORMAT_R32G32_SFLOAT;
    mesh[6].offset = offsetof(Vertex, tex_coord_1);
  }

  if (skinned_mesh.empty()) {
    skinned_mesh.resize(11);
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

    skinned_mesh[9].binding = 0;
    skinned_mesh[9].location = 9;
    skinned_mesh[9].format = VK_FORMAT_R32_SFLOAT;
    skinned_mesh[9].offset = offsetof(SkinnedVertex, vertex_info3);

    skinned_mesh[10].binding = 0;
    skinned_mesh[10].location = 10;
    skinned_mesh[10].format = VK_FORMAT_R32G32_SFLOAT;
    skinned_mesh[10].offset = offsetof(SkinnedVertex, tex_coord_1);
  }
  if (mesh_base.empty()) {
    mesh_base.assign(mesh.begin(), mesh.begin() + 5);
  }
  if (skinned_mesh_base.empty()) {
    skinned_mesh_base.assign(skinned_mesh.begin(), skinned_mesh.begin() + 9);
  }
  if (mesh_position.empty()) {
    mesh_position = {mesh[0]};
  }
  if (skinned_mesh_position.empty()) {
    skinned_mesh_position = {skinned_mesh[0], skinned_mesh[5], skinned_mesh[6], skinned_mesh[7], skinned_mesh[8]};
  }
  if (mesh_position_normal.empty()) {
    mesh_position_normal = {mesh[0], mesh[1]};
  }
  if (skinned_mesh_position_normal.empty()) {
    skinned_mesh_position_normal = {skinned_mesh[0], skinned_mesh[1], skinned_mesh[5],
                                    skinned_mesh[6], skinned_mesh[7], skinned_mesh[8]};
  }
  if (mesh_position_color.empty()) {
    mesh_position_color = {mesh[0], mesh[4]};
  }
  if (skinned_mesh_position_color.empty()) {
    skinned_mesh_position_color = {skinned_mesh[0], skinned_mesh[4], skinned_mesh[5],
                                   skinned_mesh[6], skinned_mesh[7], skinned_mesh[8]};
  }
  if (mesh_position_tex_coord.empty()) {
    mesh_position_tex_coord = {mesh[0], mesh[3]};
  }
  if (skinned_mesh_position_tex_coord.empty()) {
    skinned_mesh_position_tex_coord = {skinned_mesh[0], skinned_mesh[3], skinned_mesh[5],
                                       skinned_mesh[6], skinned_mesh[7], skinned_mesh[8]};
  }
  if (mesh_motion_vectors.empty()) {
    mesh_motion_vectors = {mesh[0], mesh[3], mesh[4], mesh[6]};
  }
  if (skinned_mesh_motion_vectors.empty()) {
    skinned_mesh_motion_vectors = {skinned_mesh[0], skinned_mesh[3], skinned_mesh[4], skinned_mesh[5],
                                   skinned_mesh[6], skinned_mesh[7], skinned_mesh[8], skinned_mesh[10]};
  }

  switch (attribute_set) {
    case VertexInputAttributeSet::Full:
      switch (geometry_type) {
        case GeometryType::Mesh:
          return mesh;
        case GeometryType::SkinnedMesh:
          return skinned_mesh;
      }
      break;
    case VertexInputAttributeSet::Base:
      switch (geometry_type) {
        case GeometryType::Mesh:
          return mesh_base;
        case GeometryType::SkinnedMesh:
          return skinned_mesh_base;
      }
      break;
    case VertexInputAttributeSet::Position:
      switch (geometry_type) {
        case GeometryType::Mesh:
          return mesh_position;
        case GeometryType::SkinnedMesh:
          return skinned_mesh_position;
      }
      break;
    case VertexInputAttributeSet::PositionNormal:
      switch (geometry_type) {
        case GeometryType::Mesh:
          return mesh_position_normal;
        case GeometryType::SkinnedMesh:
          return skinned_mesh_position_normal;
      }
      break;
    case VertexInputAttributeSet::PositionColor:
      switch (geometry_type) {
        case GeometryType::Mesh:
          return mesh_position_color;
        case GeometryType::SkinnedMesh:
          return skinned_mesh_position_color;
      }
      break;
    case VertexInputAttributeSet::PositionTexCoord:
      switch (geometry_type) {
        case GeometryType::Mesh:
          return mesh_position_tex_coord;
        case GeometryType::SkinnedMesh:
          return skinned_mesh_position_tex_coord;
      }
      break;
    case VertexInputAttributeSet::MotionVectors:
      switch (geometry_type) {
        case GeometryType::Mesh:
          return mesh_motion_vectors;
        case GeometryType::SkinnedMesh:
          return skinned_mesh_motion_vectors;
      }
      break;
  }
  throw std::runtime_error("Unhandled geometry vertex input attribute set!");
}
