#include "DsAlphaShapeUtils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace eco_sys_lab_package;

float DsAlphaShapeUtils::PointPlaneDistance(const glm::vec3& target_point, const glm::vec3& target_a,
                                            const glm::vec3& target_b, const glm::vec3& target_c) {
  // Compute the normal of the triangle
  const glm::vec3 ab = target_b - target_a;
  const glm::vec3 ac = target_c - target_a;
  const glm::vec3 normal = glm::normalize(glm::cross(ab, ac));

  // Compute signed distance from point to triangle's plane
  const float distance = glm::dot(normal, target_point - target_a);

  return distance;
}

/// @brief Given two arrays of size 4, each containing one element that is not in
/// the other, compute the respective indices of these elements in the arrays
/// @param a index array of size 4
/// @param b index array of size 4
/// @return pair of indices:
/// first indicates the position in a that does not occur in b,
/// second indicates the position in b that does not occur in a.
std::pair<int, int> DsAlphaShapeUtils::CompareIndices(const int a[4], const int b[4]) {
  std::vector a_in_both(4, false);
  std::vector b_in_both(4, false);

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      if (a[i] == b[j]) {
        a_in_both[i] = true;
        b_in_both[j] = true;
      }
    }
  }

  int a_not_in_b = -1, b_not_in_a = -1;

  for (int i = 0; i < 4; i++) {
    if (!a_in_both[i]) {
      a_not_in_b = i;
    }
    if (!b_in_both[i]) {
      b_not_in_a = i;
    }
  }

  if (a_not_in_b >= 4 || b_not_in_a >= 4) {
    return std::make_pair(a_not_in_b, b_not_in_a);
    // throw std::exception("Did not find mismatched indices!");
  }

  return std::make_pair(a_not_in_b, b_not_in_a);
}

bool DsAlphaShapeUtils::IsBetweenPlanes(const int target_indices[4],
                                        std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles) {
  int max_difference = -1;

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = i + 1; j < 4; j++) {
      int diff = glm::abs(particles[target_indices[i]].segment_index - particles[target_indices[j]].segment_index);
      if (diff > max_difference) {
        max_difference = diff;
      }
    }
  }
  // return max_difference == 1;
  return max_difference <= 1;  // for now also permit same distance
}

bool DsAlphaShapeUtils::IsValid(const int target_indices[4], int size) {
  for (size_t i = 0; i < 4; i++) {
    if (static_cast<unsigned>(target_indices[i]) >= size) {
      // EVOENGINE_ERROR("tetrahedron vertex index out of range, will be discarded: " << target_indices[i]);
      return false;
    }
  }

  // check if all indices are distinct
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = i + 1; j < 4; j++) {
      if (target_indices[i] == target_indices[j]) {
        return false;
      }
    }
  }

  return true;
}

glm::vec3 DsAlphaShapeUtils::CubicHermiteSpline(const glm::vec3& P0, const glm::vec3& P1, const glm::vec3& M0,
                                                const glm::vec3& M1, float t) {
  float t2 = t * t;
  float t3 = t2 * t;

  float h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
  float h10 = t3 - 2.0 * t2 + t;
  float h01 = -2.0 * t3 + 3.0 * t2;
  float h11 = t3 - t2;

  return h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1;
}

glm::vec3 DsAlphaShapeUtils::CubicHermiteSplineTangent(const glm::vec3& P0, const glm::vec3& P1, const glm::vec3& M0,
                                                       const glm::vec3& M1, float t) {
  float t2 = t * t;

  float h00 = 6.0 * t2 - 6.0 * t;
  float h10 = 3.0 * t2 - 4.0 * t + 1.0;
  float h01 = -6.0 * t2 + 6.0 * t;
  float h11 = 3.0 * t2 - 2.0 * t;

  return h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1;
}

std::vector<std::map<int, std::vector<size_t>>> DsAlphaShapeUtils::ComputeBundleMaps(
    std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& uniform_particles) {
  int max_dist_from_root = 0;

  for (int i = 0; i < uniform_particles.size(); i++) {
    max_dist_from_root = std::max(max_dist_from_root, uniform_particles[i].segment_index);
  }

  std::vector<std::map<int, std::vector<size_t>>> bundle_maps(max_dist_from_root + 1);
  std::vector<size_t> offsets(max_dist_from_root + 1, 0);
  std::vector<std::vector<size_t>> particle_adjacent_tets(uniform_particles.size(), std::vector<size_t>{});

  for (int i = 0; i < uniform_particles.size(); i++) {
    auto& particle = uniform_particles[i];
    auto& node_handle = particle.node_index;

    if (bundle_maps[particle.segment_index].find(node_handle) == bundle_maps[particle.segment_index].end()) {
      bundle_maps[particle.segment_index][node_handle] = std::vector<size_t>();
    }

    bundle_maps[particle.segment_index][node_handle].push_back(i);
  }

  return bundle_maps;
}

namespace {

bool RaySegmentIntersection(const glm::dvec2& origin, const glm::dvec2& dir, const glm::dvec2& a, const glm::dvec2& b,
                            double& t_out) {
  const glm::dvec2 ac = b - a;
  const glm::dvec2 ab = origin - a;
  const double det = dir.x * ac.y - dir.y * ac.x;
  if (std::abs(det) < 1e-15) {
    return false;
  }
  const double t = (ab.x * ac.y - ab.y * ac.x) / det;
  const double u = (ab.x * dir.y - ab.y * dir.x) / det;
  if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
    t_out = t;
    return true;
  }
  return false;
}

std::vector<double> RayCastPolygon(const std::vector<glm::dvec2>& polygon, const glm::dvec2& origin,
                                   const glm::dvec2& dir) {
  if (glm::length(dir) < 1e-12 || polygon.size() < 3) {
    return {};
  }
  std::vector<double> hits;
  hits.reserve(polygon.size());
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::dvec2& a = polygon[i];
    const glm::dvec2& b = polygon[(i + 1) % polygon.size()];
    double t = 0.0;
    if (RaySegmentIntersection(origin, dir, a, b, t)) {
      hits.push_back(t);
    }
  }
  return hits;
}

std::vector<glm::dvec2> BuildBoundaryPolygonForBundle(
    const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& uniform_particles,
    const std::vector<size_t>& particle_indices) {
  if (particle_indices.empty()) {
    return {};
  }

  glm::dvec2 centroid(0.0);
  for (const size_t index : particle_indices) {
    const auto& p = uniform_particles[index];
    centroid += glm::dvec2(p.profile_position.x, p.profile_position.y);
  }
  centroid /= static_cast<double>(particle_indices.size());

  constexpr double kNearBarkThreshold = 0.08;
  std::vector<glm::dvec2> near_bark;
  near_bark.reserve(particle_indices.size());
  for (const size_t index : particle_indices) {
    const auto& p = uniform_particles[index];
    if (p.distance_to_boundary <= kNearBarkThreshold) {
      near_bark.emplace_back(p.profile_position.x, p.profile_position.y);
    }
  }

  std::vector<glm::dvec2> fallback_all;
  if (near_bark.size() < 3) {
    fallback_all.reserve(particle_indices.size());
    for (const size_t index : particle_indices) {
      const auto& p = uniform_particles[index];
      fallback_all.emplace_back(p.profile_position.x, p.profile_position.y);
    }
  }
  const std::vector<glm::dvec2>& source = near_bark.size() >= 3 ? near_bark : fallback_all;

  if (source.size() < 3) {
    return source;
  }

  std::vector<std::pair<double, glm::dvec2>> by_angle;
  by_angle.reserve(source.size());
  for (const glm::dvec2& point : source) {
    const glm::dvec2 delta = point - centroid;
    by_angle.emplace_back(std::atan2(delta.y, delta.x), point);
  }
  std::sort(by_angle.begin(), by_angle.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });

  std::vector<glm::dvec2> polygon;
  polygon.reserve(by_angle.size());
  for (const auto& entry : by_angle) {
    polygon.push_back(entry.second);
  }
  return polygon;
}

}  // namespace

uint64_t DsAlphaShapeUtils::ProfileBundleKey(const int segment_index, const int node_index) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(segment_index)) << 32) | static_cast<uint32_t>(node_index);
}

std::unordered_map<uint64_t, std::vector<glm::dvec2>> DsAlphaShapeUtils::BuildProfileBundleBoundaryPolygons(
    const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& uniform_particles) {
  std::unordered_map<uint64_t, std::vector<glm::dvec2>> polygons;
  if (uniform_particles.empty()) {
    return polygons;
  }

  std::vector<DsAlphaShapeMeshing::GpuUniformParticle> mutable_particles = uniform_particles;
  const auto bundle_maps = ComputeBundleMaps(mutable_particles);
  for (size_t segment_index = 0; segment_index < bundle_maps.size(); ++segment_index) {
    for (const auto& [node_index, indices] : bundle_maps[segment_index]) {
      if (indices.size() < 3) {
        continue;
      }
      std::vector<glm::dvec2> polygon = BuildBoundaryPolygonForBundle(uniform_particles, indices);
      if (polygon.size() >= 3) {
        polygons.emplace(ProfileBundleKey(static_cast<int>(segment_index), node_index), std::move(polygon));
      }
    }
  }
  return polygons;
}

double DsAlphaShapeUtils::RelativeDistanceFromProfileCenter(const std::vector<glm::dvec2>& boundary_polygon,
                                                            const glm::dvec2& centroid,
                                                            const glm::dvec2& profile_position) {
  if (boundary_polygon.size() < 3) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const glm::dvec2 dir = profile_position - centroid;
  const auto hits = RayCastPolygon(boundary_polygon, centroid, dir);
  if (hits.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double t_max = -std::numeric_limits<double>::infinity();
  for (const double t : hits) {
    t_max = std::max(t_max, t);
  }
  if (t_max <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 1.0 / t_max;
}

glm::dvec3 DsAlphaShapeUtils::ComputeAlphaTetCornerUv(const DsAlphaShapeMeshing::GpuUniformParticle& particle,
                                                      const bool is_bark_face,
                                                      const std::vector<glm::dvec2>* boundary_polygon,
                                                      const float u_multiplier, const float v_multiplier,
                                                      const float texture_diameter, const bool use_polar_coordinates) {
  const double height = static_cast<double>(particle.segment_index) * static_cast<double>(v_multiplier);

  if (is_bark_face) {
    if (use_polar_coordinates) {
      constexpr double kPi = 3.14159265358979323846;
      double tex_x = std::fmod(
          static_cast<double>(particle.profile_polar_coordinate.y) * static_cast<double>(u_multiplier) / kPi, 2.0);
      if (tex_x < 0.0) {
        tex_x += 2.0;
      }
      if (tex_x > 1.0) {
        tex_x = 2.0 - tex_x;
      }
      return glm::dvec3(tex_x, height, height);
    }
    const double tex_x = static_cast<double>(particle.initial_position.y + particle.initial_position.z) *
                         static_cast<double>(u_multiplier);
    const double tex_y = static_cast<double>(particle.initial_position.x) * static_cast<double>(v_multiplier);
    return glm::dvec3(tex_x, tex_y, tex_y);
  }

  const glm::dvec2 profile(particle.profile_position.x, particle.profile_position.y);
  constexpr glm::dvec2 kProfileCentroid(0.0);
  double relative_distance = std::numeric_limits<double>::quiet_NaN();
  if (boundary_polygon != nullptr && boundary_polygon->size() >= 3) {
    relative_distance = RelativeDistanceFromProfileCenter(*boundary_polygon, kProfileCentroid, profile);
  }
  if (std::isnan(relative_distance)) {
    relative_distance = 1.0 - static_cast<double>(glm::clamp(particle.distance_to_boundary, 0.0f, 1.0f));
  }
  relative_distance = glm::clamp(relative_distance, 0.0, 1.0);

  const double angle = std::atan2(kProfileCentroid.y - profile.y, kProfileCentroid.x - profile.x);
  const double radial_scale = static_cast<double>(texture_diameter) * relative_distance * 0.5;
  return glm::dvec3(0.5 + radial_scale * std::cos(angle), 0.5 + radial_scale * std::sin(angle), height);
}
