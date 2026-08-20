#pragma once

#include "LSystem_PCH.hpp"

#include <Particles.hpp>

#include <cstdint>
#include <vector>

namespace l_system_package {
using namespace evo_engine;

enum class SorghumOrganGeometryKind : uint8_t {
  Culm,
  InternodeInstance,
  Leaf,
  PanicleRachis,
  PanicleBranch,
  PanicleSpikelet
};

struct SorghumOrganGeometryRange {
  SorghumOrganGeometryKind kind = SorghumOrganGeometryKind::Leaf;
  int axis_id = 0;
  int rank = 0;
  int node_id = -1;
  uint32_t vertex_offset = 0;
  uint32_t vertex_count = 0;
  uint32_t triangle_offset = 0;
  uint32_t triangle_count = 0;
  uint32_t instance_offset = 0;
  uint32_t instance_count = 0;
};

struct SorghumGeometrySnapshot {
  static constexpr uint32_t kSchemaVersion = 2;

  uint32_t schema_version = kSchemaVersion;
  uint64_t geometry_version = 0;
  uint32_t seed = 0;
  float target_gdd = 0.0f;
  uint32_t node_count = 0;
  uint32_t internode_count = 0;
  uint32_t leaf_count = 0;
  uint32_t live_leaf_count = 0;
  uint32_t panicle_branch_count = 0;
  uint32_t panicle_spikelet_count = 0;
  uint32_t invalid_instance_count = 0;
  std::vector<ParticleInfo> internode_instances;
  std::vector<Vertex> culm_vertices;
  std::vector<glm::uvec3> culm_triangles;
  std::vector<Vertex> leaf_vertices;
  std::vector<glm::uvec3> leaf_triangles;
  std::vector<Vertex> panicle_vertices;
  std::vector<glm::uvec3> panicle_triangles;
  std::vector<SorghumOrganGeometryRange> organ_ranges;
};

}  // namespace l_system_package
