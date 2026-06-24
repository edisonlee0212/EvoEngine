#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace l_system_package {

struct SnapshotVertexAttributes {
  bool normal = false;
  bool tangent = false;
  bool tex_coord = false;
  bool color = false;
};

struct SnapshotStrandPointAttributes {
  bool normal = false;
  bool tex_coord = false;
  bool color = false;
};

struct SnapshotVertex {
  glm::vec3 position = glm::vec3(0.0f);
  glm::vec3 normal = glm::vec3(0.0f);
  glm::vec3 tangent = glm::vec3(0.0f);
  glm::vec4 color = glm::vec4(1.0f);
  glm::vec2 tex_coord = glm::vec2(0.0f);
};

struct SnapshotStrandPoint {
  glm::vec3 position = glm::vec3(0.0f);
  float thickness = 0.0f;
  glm::vec3 normal = glm::vec3(0.0f);
  float tex_coord = 0.0f;
  glm::vec4 color = glm::vec4(1.0f);
  glm::vec4 material_properties = glm::vec4(0.0f);
};

struct SnapshotInstance {
  glm::mat4 transform = glm::mat4(1.0f);
  glm::vec4 color = glm::vec4(1.0f);
};

/**
 * @brief Render primitive categories emitted from L-system CPU snapshots.
 */
enum class RenderPrimitiveKind : std::uint8_t {
  None = 0,
  Instance = 1,
  Strands = 2,
  Mesh = 3,
};

/**
 * @brief Stable identity key for one render-emitting module instance.
 */
struct RenderNodeKey {
  int node_handle = -1;
  int flow_handle = -1;
  int symbol_id = -1;
};

/**
 * @brief CPU snapshot payload for one instanced primitive.
 */
struct InstancePrimitiveSnapshot {
  RenderNodeKey key{};
  SnapshotInstance instance{};
};

/**
 * @brief CPU snapshot payload for one strands primitive.
 */
struct StrandsPrimitiveSnapshot {
  RenderNodeKey key{};
  SnapshotStrandPointAttributes attributes{};
  std::vector<glm::uint> segments;
  std::vector<SnapshotStrandPoint> points;
};

/**
 * @brief CPU snapshot payload for one mesh primitive.
 */
struct MeshPrimitiveSnapshot {
  RenderNodeKey key{};
  SnapshotVertexAttributes attributes{};
  std::vector<SnapshotVertex> vertices;
  std::vector<glm::uvec3> triangles;
};

/**
 * @brief Versioned render snapshot produced from one L-system growth state.
 *
 * This is the CPU-authoritative bridge between graph/growth state and
 * channel publication.
 */
struct LSystemRenderSnapshot {
  std::uint64_t version = 0;

  std::vector<InstancePrimitiveSnapshot> instances;
  std::vector<StrandsPrimitiveSnapshot> strands;
  std::vector<MeshPrimitiveSnapshot> meshes;

  void Clear() {
    instances.clear();
    strands.clear();
    meshes.clear();
  }

  [[nodiscard]] bool Empty() const {
    return instances.empty() && strands.empty() && meshes.empty();
  }
};

}  // namespace l_system_package
