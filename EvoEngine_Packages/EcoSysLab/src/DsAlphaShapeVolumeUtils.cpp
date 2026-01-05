#include "DsAlphaShapeVolumeUtils.hpp"

#include <algorithm>
#include <cmath>

namespace eco_sys_lab_package {
namespace {

constexpr double kDegenerateVolumeEpsilon = 1e-18;

const glm::vec3& ParticlePosition(const DsAlphaShapeMeshing::GpuUniformParticle& particle,
                                  const bool use_current_position) {
  return use_current_position ? particle.position : particle.initial_position;
}

double MaxSquaredEdgeLength(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& d) {
  const glm::dvec3 edges[6] = {b - a, c - a, d - a, c - b, d - b, d - c};
  double max_edge2 = 0.0;
  for (const glm::dvec3& edge : edges) {
    max_edge2 = std::max(max_edge2, glm::dot(edge, edge));
  }
  return max_edge2;
}

bool TetIndicesValid(const DsAlphaShapeMeshing::GpuDelaunayTetrahedron& tet, const size_t particle_count) {
  for (int corner = 0; corner < 4; ++corner) {
    if (tet.indices[corner] < 0 || static_cast<size_t>(tet.indices[corner]) >= particle_count) {
      return false;
    }
  }
  return true;
}

}  // namespace

double DsAlphaShapeVolumeUtils::nearly_degenerate_relative_volume = 1e-6;

double DsAlphaShapeVolumeUtils::TetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                                                  const glm::vec3& d) {
  const glm::dvec3 u = glm::dvec3(b) - glm::dvec3(a);
  const glm::dvec3 v = glm::dvec3(c) - glm::dvec3(a);
  const glm::dvec3 w = glm::dvec3(d) - glm::dvec3(a);
  const double triple = glm::dot(u, glm::cross(v, w));
  const double volume = std::abs(triple) / 6.0;
  return volume < kDegenerateVolumeEpsilon ? 0.0 : volume;
}

bool DsAlphaShapeVolumeUtils::IsNearlyDegenerate(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                                                 const glm::vec3& d) {
  const glm::dvec3 da(a);
  const glm::dvec3 db(b);
  const glm::dvec3 dc(c);
  const glm::dvec3 dd(d);
  const double max_edge2 = MaxSquaredEdgeLength(da, db, dc, dd);
  if (max_edge2 <= kDegenerateVolumeEpsilon) {
    return true;
  }
  const double max_edge = std::sqrt(max_edge2);
  const double volume = TetrahedronVolume(a, b, c, d);
  const double relative = volume / (max_edge * max_edge * max_edge);
  return relative <= nearly_degenerate_relative_volume;
}

std::vector<uint8_t> DsAlphaShapeVolumeUtils::ClassifyNearlyDegenerate(
    const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
    const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons, const bool use_current_position) {
  std::vector<uint8_t> nearly_degenerate(tetrahedrons.size(), 0);
  for (size_t tet_index = 0; tet_index < tetrahedrons.size(); ++tet_index) {
    const auto& tet = tetrahedrons[tet_index];
    if (!IsTetrahedronAlive(tet) || !TetIndicesValid(tet, particles.size())) {
      continue;
    }
    const glm::vec3& p0 = ParticlePosition(particles[tet.indices[0]], use_current_position);
    const glm::vec3& p1 = ParticlePosition(particles[tet.indices[1]], use_current_position);
    const glm::vec3& p2 = ParticlePosition(particles[tet.indices[2]], use_current_position);
    const glm::vec3& p3 = ParticlePosition(particles[tet.indices[3]], use_current_position);
    if (IsNearlyDegenerate(p0, p1, p2, p3)) {
      nearly_degenerate[tet_index] = 1;
    }
  }
  return nearly_degenerate;
}

DsAlphaShapeVolumeUtils::VolumeResult DsAlphaShapeVolumeUtils::ComputeTetrahedronVolumes(
    const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
    const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons, const bool use_current_position) {
  VolumeResult result;
  result.per_tet_volume.assign(tetrahedrons.size(), kDeadTetrahedronVolumeSentinel);

  for (size_t tet_index = 0; tet_index < tetrahedrons.size(); ++tet_index) {
    const auto& tet = tetrahedrons[tet_index];
    if (!IsTetrahedronAlive(tet) || !TetIndicesValid(tet, particles.size())) {
      ++result.dead_count;
      continue;
    }

    const double volume = TetrahedronVolume(ParticlePosition(particles[tet.indices[0]], use_current_position),
                                            ParticlePosition(particles[tet.indices[1]], use_current_position),
                                            ParticlePosition(particles[tet.indices[2]], use_current_position),
                                            ParticlePosition(particles[tet.indices[3]], use_current_position));
    result.per_tet_volume[tet_index] = volume;
    result.cumulative_volume += volume;
    ++result.alive_count;
  }
  return result;
}

}  // namespace eco_sys_lab_package
