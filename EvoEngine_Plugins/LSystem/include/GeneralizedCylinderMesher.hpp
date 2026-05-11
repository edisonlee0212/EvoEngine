#pragma once

// ---------------------------------------------------------------------------
// GeneralizedCylinderMesher
//
// Phase 1 of the biologically-emergent organ growth substrate. Sweeps a
// CrossSectionProfile along an OrganCenterline at uniformly spaced arc-
// length stations, emitting an explicit triangle mesh (`evo_engine::Vertex`
// + `glm::uvec3` index list). Produces:
//
//   * deterministic vertex ordering (station-major, then perimeter-major)
//     so visual diffs and PLY snapshot tests are stable across runs,
//   * smooth shading via per-vertex normals derived from the profile's
//     `outward_normal` rotated into the centerline frame,
//   * one anchor transform applied to every vertex so the same mesher is
//     used for needles attached to different internodes / orientations.
//
// The mesher is allocation-aware: caller-supplied `out_vertices` /
// `out_indices` are *appended to*, not cleared, so a single mesh entity can
// aggregate every needle in the tree in one pass with zero per-needle
// allocations.
//
// Phase 1 contract: with a Straight() centerline and a CircularProfile, the
// emitted mesh has the same topology (segments × stations - 2 quads) as the
// existing GenerateUnitCylinderMesh helper in ScotsPine.cpp, modulo the
// flexibility to vary radius along arc length.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "CrossSectionProfile.hpp"
#include "OrganCenterline.hpp"

namespace l_system_plugin {

struct GeneralizedCylinderMesherConfig {
  int station_count = 12;        ///< Number of arc-length sampling stations along the centerline (>=2).
  int perimeter_count = 8;        ///< Number of perimeter samples per station (>=3 for closed profiles).
  glm::vec4 vertex_color = glm::vec4(1.0f);
  /// Optional station-wise colors over arc-length [0,1]. If non-empty, this
  /// overrides `vertex_color` and is linearly interpolated by station index.
  std::vector<glm::vec4> station_color_table;
  /// Anchor transform applied to every emitted vertex. Position-only translation +
  /// rotation; no scaling. Defaults to identity (organ-local frame == world).
  glm::vec3 anchor_position = glm::vec3(0.0f);
  glm::quat anchor_rotation = glm::quat(1, 0, 0, 0);
  /// Mode for cross-section radius along arc length:
  ///   * if `radius_table.empty()`, uses constant `base_radius`.
  ///   * else linearly interpolates `radius_table` over arc length.
  float base_radius = 0.05f;
  std::vector<float> radius_table;
  /// Optional secondary axis radius (for ellipsoid cross-sections).
  /// If `secondary_radius_table` is empty, `secondary_base_radius` is used.
  float secondary_base_radius = 0.05f;
  std::vector<float> secondary_radius_table;
  /// Reference adaxial direction at the base of the organ (in organ-local frame).
  /// Drives the rotation-minimizing parallel-transport frame.
  glm::vec3 base_adaxial = glm::vec3(1, 0, 0);
};

namespace mesher_detail {

inline float ResolveRadius(const GeneralizedCylinderMesherConfig& cfg,
                            float arc_length, float total_length) {
  if (cfg.radius_table.empty() || total_length <= 0.0f) return cfg.base_radius;
  if (cfg.radius_table.size() == 1) return cfg.radius_table.front();
  const float t = std::clamp(arc_length / total_length, 0.0f, 1.0f);
  const float pos = t * static_cast<float>(cfg.radius_table.size() - 1);
  const int i = std::min(static_cast<int>(cfg.radius_table.size()) - 2,
                         static_cast<int>(std::floor(pos)));
  const float u = pos - static_cast<float>(i);
  return glm::mix(cfg.radius_table[i], cfg.radius_table[i + 1], u);
}

inline float ResolveSecondaryRadius(const GeneralizedCylinderMesherConfig& cfg,
                                    float arc_length, float total_length) {
  if (cfg.secondary_radius_table.empty() || total_length <= 0.0f) {
    return cfg.secondary_base_radius;
  }
  if (cfg.secondary_radius_table.size() == 1) return cfg.secondary_radius_table.front();
  const float t = std::clamp(arc_length / total_length, 0.0f, 1.0f);
  const float pos = t * static_cast<float>(cfg.secondary_radius_table.size() - 1);
  const int i = std::min(static_cast<int>(cfg.secondary_radius_table.size()) - 2,
                         static_cast<int>(std::floor(pos)));
  const float u = pos - static_cast<float>(i);
  return glm::mix(cfg.secondary_radius_table[i], cfg.secondary_radius_table[i + 1], u);
}

inline glm::vec4 ResolveStationColor(const GeneralizedCylinderMesherConfig& cfg,
                                     int station_index,
                                     int station_count) {
  if (cfg.station_color_table.empty()) return cfg.vertex_color;
  if (cfg.station_color_table.size() == 1) return cfg.station_color_table.front();
  const float denom = static_cast<float>(std::max(1, station_count - 1));
  const float t = std::clamp(static_cast<float>(station_index) / denom, 0.0f, 1.0f);
  const float pos = t * static_cast<float>(cfg.station_color_table.size() - 1);
  const int i = std::min(static_cast<int>(cfg.station_color_table.size()) - 2,
                         static_cast<int>(std::floor(pos)));
  const float u = pos - static_cast<float>(i);
  return glm::mix(cfg.station_color_table[i], cfg.station_color_table[i + 1], u);
}

}  // namespace mesher_detail

/// Sweep `profile` along `centerline` and append geometry to `out_vertices` /
/// `out_indices`. Returns the number of vertices appended (use as a future
/// re-entry index into `out_vertices`).
template <typename VertexT>
inline std::size_t SweepGeneralizedCylinder(const OrganCenterline& centerline,
                                             const CrossSectionProfile& profile,
                                             const GeneralizedCylinderMesherConfig& cfg,
                                             std::vector<VertexT>& out_vertices,
                                             std::vector<glm::uvec3>& out_indices) {
  const int n_stations = std::max(2, cfg.station_count);
  const int n_perim = std::max(3, cfg.perimeter_count);
  const float total_length = centerline.TotalLength();
  if (total_length <= 0.0f) return 0;

  const std::size_t vertex_offset = out_vertices.size();
  std::vector<CrossSectionSample> profile_samples;

  // Cached per-station data so we can emit triangles between station i and i+1
  // without re-sampling.
  for (int i = 0; i < n_stations; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(n_stations - 1);
    const float s = t * total_length;
    const CenterlineSample frame = centerline.Sample(s, cfg.base_adaxial);
    const float radius = mesher_detail::ResolveRadius(cfg, s, total_length);
    const float secondary_radius =
      mesher_detail::ResolveSecondaryRadius(cfg, s, total_length);
    const glm::vec4 station_color =
        mesher_detail::ResolveStationColor(cfg, i, n_stations);
    profile.SampleWithAxes(n_perim, radius, secondary_radius, profile_samples);

    for (const auto& ps : profile_samples) {
      const glm::vec3 local_pos = frame.position +
                                  ps.radial_offset.x * frame.normal +
                                  ps.radial_offset.y * frame.binormal;
      const glm::vec3 local_nrm = ps.outward_normal.x * frame.normal +
                                  ps.outward_normal.y * frame.binormal;
      VertexT v{};
      v.position = cfg.anchor_position + cfg.anchor_rotation * local_pos;
      v.normal = cfg.anchor_rotation * glm::normalize(local_nrm);
      v.tangent = cfg.anchor_rotation * frame.tangent;
      v.color = station_color;
      v.tex_coord = glm::vec2(ps.u, t);
      out_vertices.push_back(v);
    }
  }

  // Emit quads (two triangles each) between consecutive stations.
  const bool closed = profile.IsClosed();
  const std::size_t emitted_per_station = static_cast<std::size_t>(n_perim);
  for (int station = 0; station < n_stations - 1; ++station) {
    const std::size_t row_a = vertex_offset + static_cast<std::size_t>(station) * emitted_per_station;
    const std::size_t row_b = row_a + emitted_per_station;
    const int last_p = closed ? n_perim : n_perim - 1;
    for (int p = 0; p < last_p; ++p) {
      const std::size_t p0 = row_a + static_cast<std::size_t>(p);
      const std::size_t p1 = row_a + static_cast<std::size_t>((p + 1) % n_perim);
      const std::size_t p2 = row_b + static_cast<std::size_t>(p);
      const std::size_t p3 = row_b + static_cast<std::size_t>((p + 1) % n_perim);
      // Outward winding (consistent with right-handed (n, b) frame).
      out_indices.emplace_back(static_cast<unsigned int>(p0),
                               static_cast<unsigned int>(p2),
                               static_cast<unsigned int>(p1));
      out_indices.emplace_back(static_cast<unsigned int>(p1),
                               static_cast<unsigned int>(p2),
                               static_cast<unsigned int>(p3));
    }
  }

  // Caps for closed profiles only (open profiles e.g. leaf blade need a
  // distinct algorithm). Triangle fan from the first vertex of each end ring.
  if (closed && n_perim >= 3) {
    // Base cap: fan around station 0, normal = -tangent_at_base.
    const std::size_t row_base = vertex_offset;
    for (int p = 1; p + 1 < n_perim; ++p) {
      out_indices.emplace_back(static_cast<unsigned int>(row_base),
                               static_cast<unsigned int>(row_base + p + 1),
                               static_cast<unsigned int>(row_base + p));
    }
    // Tip cap: fan around last station.
    const std::size_t row_tip = vertex_offset +
                                static_cast<std::size_t>(n_stations - 1) * emitted_per_station;
    for (int p = 1; p + 1 < n_perim; ++p) {
      out_indices.emplace_back(static_cast<unsigned int>(row_tip),
                               static_cast<unsigned int>(row_tip + p),
                               static_cast<unsigned int>(row_tip + p + 1));
    }
  }

  return out_vertices.size() - vertex_offset;
}

}  // namespace l_system_plugin
