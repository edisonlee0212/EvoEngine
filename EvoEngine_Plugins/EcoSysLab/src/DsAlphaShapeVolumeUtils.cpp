#include "DsAlphaShapeVolumeUtils.hpp"

#include <cmath>

namespace eco_sys_lab_plugin {
namespace {

constexpr double kDegenerateVolumeEpsilon = 1e-18;

const glm::vec3& ParticlePosition(const DsAlphaShapeMeshing::GpuUniformParticle& particle,
                                  const bool use_current_position) {
  return use_current_position ? particle.position : particle.initial_position;
}

}  // namespace

double DsAlphaShapeVolumeUtils::TetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                                                  const glm::vec3& d) {
  const glm::dvec3 u = glm::dvec3(b) - glm::dvec3(a);
  const glm::dvec3 v = glm::dvec3(c) - glm::dvec3(a);
  const glm::dvec3 w = glm::dvec3(d) - glm::dvec3(a);
  const double triple = glm::dot(u, glm::cross(v, w));
  const double volume = std::abs(triple) / 6.0;
  return volume < kDegenerateVolumeEpsilon ? 0.0 : volume;
}

DsAlphaShapeVolumeUtils::VolumeResult DsAlphaShapeVolumeUtils::ComputeTetrahedronVolumes(
    const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
    const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons,
    const bool use_current_position) {
  VolumeResult result;
  result.per_tet_volume.assign(tetrahedrons.size(), 0.0);

  for (size_t tet_index = 0; tet_index < tetrahedrons.size(); ++tet_index) {
    const auto& tet = tetrahedrons[tet_index];
    if (!IsTetrahedronAlive(tet)) {
      ++result.dead_count;
      continue;
    }

    bool indices_valid = true;
    for (int corner = 0; corner < 4; ++corner) {
      if (tet.indices[corner] < 0 || static_cast<size_t>(tet.indices[corner]) >= particles.size()) {
        indices_valid = false;
        break;
      }
    }
    if (!indices_valid) {
      ++result.dead_count;
      continue;
    }

    const double volume =
        TetrahedronVolume(ParticlePosition(particles[tet.indices[0]], use_current_position),
                          ParticlePosition(particles[tet.indices[1]], use_current_position),
                          ParticlePosition(particles[tet.indices[2]], use_current_position),
                          ParticlePosition(particles[tet.indices[3]], use_current_position));
    result.per_tet_volume[tet_index] = volume;
    result.cumulative_volume += volume;
    ++result.alive_count;
  }
  return result;
}

}  // namespace eco_sys_lab_plugin
