#pragma once

#include "SorghumModules.hpp"
#include "SorghumRules.hpp"

#include <Vertex.hpp>

#include <glm/common.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace l_system_package {

// Runtime meshing knobs that replace SorghumLayer dependencies from
// DigitalAgriculture's SorghumLeafDescriptor::GenerateGeometry.
struct SorghumLeafMeshSettings {
  float vertical_subdivision_length = 0.01f;
  int horizontal_subdivision_step = 6;
  bool enable_leaf_sheath = true;
  float leaf_width_scale = 1.0f;
  float leaf_thickness = 0.001f;
  glm::vec3 gravity_local_m_s2 = glm::vec3(0.0f, -9.80665f, 0.0f);
};

struct SorghumLeafAtlasLayout {
  uint32_t variant_columns = 1u;
  uint32_t variant_rows = 1u;
  uint32_t variant_count = 1u;
  uint32_t variant_index = 0u;
  float tile_uv_inset = 0.001f;
  bool distal_region_uses_top_half = false;
  bool semantic_quadrants = false;
};

[[nodiscard]] inline SorghumLeafAtlasLayout NormalizeSorghumLeafAtlasLayout(SorghumLeafAtlasLayout layout) {
  layout.variant_columns = std::max(1u, layout.variant_columns);
  layout.variant_rows = std::max(1u, layout.variant_rows);
  const uint32_t max_count = layout.variant_columns * layout.variant_rows;
  layout.variant_count = std::clamp(layout.variant_count, 1u, max_count);
  layout.variant_index %= layout.variant_count;
  const float tile_width = 1.0f / static_cast<float>(layout.variant_columns);
  const float tile_height = 1.0f / static_cast<float>(layout.variant_rows);
  layout.tile_uv_inset = std::clamp(layout.tile_uv_inset, 0.0f, std::min(tile_width, tile_height) * 0.45f);
  return layout;
}

[[nodiscard]] inline glm::vec2 RemapSorghumLeafAtlasUv(const glm::vec2 local_uv,
                                                       const SorghumLeafAtlasLayout& raw_layout) {
  const auto layout = NormalizeSorghumLeafAtlasLayout(raw_layout);
  const uint32_t tile_x = layout.variant_index % layout.variant_columns;
  const uint32_t tile_y = layout.variant_index / layout.variant_columns;
  const float tile_width = 1.0f / static_cast<float>(layout.variant_columns);
  const float tile_height = 1.0f / static_cast<float>(layout.variant_rows);
  const float inset = layout.variant_columns * layout.variant_rows > 1u ? layout.tile_uv_inset : 0.0f;
  const float u0 = static_cast<float>(tile_x) * tile_width + inset;
  const float u1 = static_cast<float>(tile_x + 1u) * tile_width - inset;
  const float v0 = 1.0f - static_cast<float>(tile_y + 1u) * tile_height + inset;
  const float v1 = 1.0f - static_cast<float>(tile_y) * tile_height - inset;
  const glm::vec2 uv = glm::clamp(local_uv, glm::vec2(0.0f), glm::vec2(1.0f));
  return glm::vec2(glm::mix(u0, u1, uv.x), glm::mix(v0, v1, uv.y));
}

[[nodiscard]] inline uint32_t MixSorghumLeafAtlasSeed(uint32_t value) {
  value ^= value >> 16;
  value *= 0x7feb352du;
  value ^= value >> 15;
  value *= 0x846ca68bu;
  value ^= value >> 16;
  return value;
}

[[nodiscard]] inline uint32_t ComputeSorghumLeafAtlasVariant(const uint32_t plant_seed, const uint32_t leaf_index,
                                                             const float leaf_random, const uint32_t variant_count) {
  if (variant_count <= 1u) {
    return 0u;
  }
  const float clamped_random = std::clamp(leaf_random, 0.0f, 1.0f);
  const uint32_t random_bits =
      clamped_random >= 1.0f ? 0xffffffffu : static_cast<uint32_t>(static_cast<double>(clamped_random) * 4294967296.0);
  return MixSorghumLeafAtlasSeed(plant_seed ^ (leaf_index + 0x9e3779b9u) ^ (random_bits + 0x85ebca6bu)) % variant_count;
}

// Ordered parent-axis context at leaf insertion time. Phase E.2 builds this
// from internode nodes on the host axis (base -> tip order).
struct StemContext {
  struct Segment {
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 up = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 front = glm::vec3(0.0f, 1.0f, 0.0f);
    float radius = 0.005f;
    float theta = 180.0f;
    float left_height_offset = 0.0f;
    float right_height_offset = 0.0f;
  };

  std::vector<Segment> segments;
};

class SorghumSplineSegment {
 public:
  glm::vec3 position = glm::vec3(0.0f);
  glm::vec3 front = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec3 up = glm::vec3(0.0f, 0.0f, 1.0f);
  float radius = 0.005f;
  float cross_section_aspect_ratio = 1.0f;
  float theta = 180.0f;
  float left_height_offset = 0.0f;
  float right_height_offset = 0.0f;

  SorghumSplineSegment() = default;
  SorghumSplineSegment(const glm::vec3& position, const glm::vec3& up, const glm::vec3& front, float radius,
                       float theta, float left_height_offset = 0.0f, float right_height_offset = 0.0f,
                       float cross_section_aspect_ratio = 1.0f);

  [[nodiscard]] glm::vec3 GetLeafPoint(float angle_deg) const;
  [[nodiscard]] glm::vec3 GetStemPoint(float angle_deg) const;
  [[nodiscard]] glm::vec3 GetNormal(float angle_deg) const;
};

class SorghumSpline {
 public:
  std::vector<SorghumSplineSegment> segments;

  void SubdivideByDistance(float subdivision_distance, std::vector<SorghumSplineSegment>& subdivided_segments) const;

  void GetPositionControlPoints(uint32_t segment_index, glm::vec3& p0, glm::vec3& p1, glm::vec3& p2,
                                glm::vec3& p3) const;

  [[nodiscard]] float GetSegmentArcLength(uint32_t segment_index, float t_start = 0.0f, float t_end = 1.0f,
                                          float tolerance = 0.0001f) const;

  [[nodiscard]] float GetArcLength(float tolerance = 0.0001f) const;

  [[nodiscard]] SorghumSplineSegment InterpolateSegment(uint32_t segment_index, float t) const;

  [[nodiscard]] std::vector<SorghumSplineSegment> RebuildFixedSizeSegments(size_t segment_count,
                                                                           float tolerance = 0.0001f) const;

  [[nodiscard]] std::vector<SorghumSplineSegment> RebuildFixedLengthSegments(float segment_length,
                                                                             float tolerance = 0.0001f) const;

  [[nodiscard]] std::vector<SorghumSplineSegment> GetStemPart() const;
  [[nodiscard]] std::vector<SorghumSplineSegment> GetLeafPart() const;
};

// Build a procedural leaf spline from runtime module state (SorghumLeaf) +
// parent-axis context. This mirrors SorghumLeafState::Apply semantics.
void BuildLeafSplineFromState(const SorghumLeaf& leaf, const StemContext& stem_ctx, const SampledSorghumParams& params,
                              const SorghumLeafMeshSettings& settings, SorghumSpline& out_spline);

// Append one blade surface pass (top or bottom) to shared mesh buffers.
// Call twice with current_bottom_face false/true to emit both sides.
void GenerateBladeGeometry(const SorghumSpline& spline, const SorghumLeaf& leaf, const SampledSorghumParams& params,
                           const SorghumLeafMeshSettings& settings, std::vector<evo_engine::Vertex>& vertices,
                           std::vector<glm::uvec3>& triangles, bool current_bottom_face = false,
                           uint32_t leaf_index = 0u,
                           const SorghumLeafAtlasLayout& atlas_layout = SorghumLeafAtlasLayout{});

}  // namespace l_system_package
