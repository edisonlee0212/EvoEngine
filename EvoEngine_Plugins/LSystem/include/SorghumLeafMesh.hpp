#pragma once

#include "SorghumModules.hpp"
#include "SorghumRules.hpp"

#include <Vertex.hpp>

#include <cstdint>
#include <vector>

namespace l_system_plugin {

// Runtime meshing knobs that replace SorghumLayer dependencies from
// DigitalAgriculture's SorghumLeafDescriptor::GenerateGeometry.
struct SorghumLeafMeshSettings {
  float vertical_subdivision_length = 0.01f;
  int horizontal_subdivision_step = 6;
  bool enable_leaf_sheath = true;
  float leaf_thickness = 0.001f;
};

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
  float theta = 180.0f;
  float left_height_offset = 0.0f;
  float right_height_offset = 0.0f;

  SorghumSplineSegment() = default;
  SorghumSplineSegment(const glm::vec3& position,
                       const glm::vec3& up,
                       const glm::vec3& front,
                       float radius,
                       float theta,
                       float left_height_offset = 0.0f,
                       float right_height_offset = 0.0f);

  [[nodiscard]] glm::vec3 GetLeafPoint(float angle_deg) const;
  [[nodiscard]] glm::vec3 GetStemPoint(float angle_deg) const;
  [[nodiscard]] glm::vec3 GetNormal(float angle_deg) const;
};

class SorghumSpline {
 public:
  std::vector<SorghumSplineSegment> segments;

  void SubdivideByDistance(float subdivision_distance,
                           std::vector<SorghumSplineSegment>& subdivided_segments) const;

  void GetPositionControlPoints(uint32_t segment_index,
                                glm::vec3& p0,
                                glm::vec3& p1,
                                glm::vec3& p2,
                                glm::vec3& p3) const;

  [[nodiscard]] float GetSegmentArcLength(uint32_t segment_index,
                                          float t_start = 0.0f,
                                          float t_end = 1.0f,
                                          float tolerance = 0.0001f) const;

  [[nodiscard]] float GetArcLength(float tolerance = 0.0001f) const;

  [[nodiscard]] SorghumSplineSegment InterpolateSegment(uint32_t segment_index,
                                                        float t) const;

  [[nodiscard]] std::vector<SorghumSplineSegment> RebuildFixedSizeSegments(
      size_t segment_count,
      float tolerance = 0.0001f) const;

  [[nodiscard]] std::vector<SorghumSplineSegment> RebuildFixedLengthSegments(
      float segment_length,
      float tolerance = 0.0001f) const;

  [[nodiscard]] std::vector<SorghumSplineSegment> GetStemPart() const;
  [[nodiscard]] std::vector<SorghumSplineSegment> GetLeafPart() const;
};

// Build a procedural leaf spline from runtime module state (SorghumLeaf) +
// parent-axis context. This mirrors SorghumLeafState::Apply semantics.
void BuildLeafSplineFromState(const SorghumLeaf& leaf,
                              const StemContext& stem_ctx,
                              const SampledSorghumParams& params,
                              const SorghumLeafMeshSettings& settings,
                              SorghumSpline& out_spline);

// Append one blade surface pass (top or bottom) to shared mesh buffers.
// Call twice with current_bottom_face false/true to emit both sides.
void GenerateBladeGeometry(const SorghumSpline& spline,
                           const SorghumLeaf& leaf,
                           const SampledSorghumParams& params,
                           const SorghumLeafMeshSettings& settings,
                           std::vector<evo_engine::Vertex>& vertices,
                           std::vector<unsigned int>& indices,
                           bool current_bottom_face = false,
                           uint32_t leaf_index = 0u);

}  // namespace l_system_plugin
