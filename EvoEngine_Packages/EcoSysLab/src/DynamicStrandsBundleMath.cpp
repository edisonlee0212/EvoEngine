#include "DynamicStrandsBundleMath.hpp"

#include <map>

namespace eco_sys_lab_package {
namespace {
glm::mat3 CrossMatrix(const glm::vec3 vector) {
  return {{0.f, vector.z, -vector.y}, {-vector.z, 0.f, vector.x}, {vector.y, -vector.x, 0.f}};
}

glm::vec3 Solve(const glm::mat3& matrix, const glm::vec3& value) {
  return glm::abs(glm::determinant(matrix)) > 1e-12f ? glm::inverse(matrix) * value : glm::vec3(0.f);
}
}  // namespace

glm::vec3 BundleQuaternionLog(glm::quat rotation) {
  if (rotation.w < 0.f)
    rotation = -rotation;
  const float sine = glm::length(glm::vec3(rotation.x, rotation.y, rotation.z));
  return sine < 1e-7f ? 2.f * glm::vec3(rotation.x, rotation.y, rotation.z)
                      : 2.f * std::atan2(sine, rotation.w) * glm::vec3(rotation.x, rotation.y, rotation.z) / sine;
}

BundlePairCorrection SolveBundlePairReference(const BundleRigidBodyState& body0, const BundleRigidBodyState& body1,
                                              BundlePairReferenceState& constraint) {
  BundlePairCorrection correction;
  const glm::vec3 arm0 = body0.rotation * constraint.segment0_midpoint_offset;
  const glm::vec3 arm1 = body1.rotation * constraint.segment1_midpoint_offset;
  const glm::mat3 effective_mass =
      (body0.inverse_mass + body1.inverse_mass) * glm::mat3(1.f) -
      CrossMatrix(arm0) * body0.inverse_inertia * CrossMatrix(arm0) -
      CrossMatrix(arm1) * body1.inverse_inertia * CrossMatrix(arm1) +
      glm::mat3(constraint.positional_compliance.x, 0.f, 0.f, 0.f, constraint.positional_compliance.y, 0.f, 0.f, 0.f,
                constraint.positional_compliance.z);
  const glm::vec3 error = body0.position + arm0 - body1.position - arm1;
  const glm::vec3 delta_lambda =
      Solve(effective_mass, -(error + constraint.positional_compliance * constraint.positional_lambda));
  constraint.positional_lambda += delta_lambda;
  correction.position0 = body0.inverse_mass * delta_lambda;
  correction.position1 = -body1.inverse_mass * delta_lambda;
  correction.angular0 = body0.inverse_inertia * glm::cross(arm0, delta_lambda);
  correction.angular1 = body1.inverse_inertia * glm::cross(arm1, -delta_lambda);

  const glm::vec3 angular_error =
      body0.rotation * BundleQuaternionLog(glm::conjugate(body0.rotation) * body1.rotation *
                                           glm::conjugate(constraint.rest_relative_rotation));
  const glm::mat3 angular_compliance(constraint.angular_compliance.x, 0.f, 0.f, 0.f, constraint.angular_compliance.y,
                                     0.f, 0.f, 0.f, constraint.angular_compliance.z);
  const glm::vec3 angular_delta = Solve(body0.inverse_inertia + body1.inverse_inertia + angular_compliance,
                                        -(angular_error + angular_compliance * constraint.angular_lambda));
  constraint.angular_lambda += angular_delta;
  correction.angular0 += body0.inverse_inertia * angular_delta;
  correction.angular1 -= body1.inverse_inertia * angular_delta;
  return correction;
}

void ApplyBundlePairCorrection(BundleRigidBodyState& body0, BundleRigidBodyState& body1,
                               const BundlePairCorrection& correction) {
  const auto apply = [](BundleRigidBodyState& body, const glm::vec3& position, const glm::vec3& angular) {
    body.position += position;
    const float angle = glm::length(angular);
    if (angle > 1e-8f)
      body.rotation = glm::normalize(glm::angleAxis(angle, angular / angle) * body.rotation);
  };
  apply(body0, correction.position0, correction.angular0);
  apply(body1, correction.position1, correction.angular1);
}

std::vector<uint32_t> BuildBundleBaseSlices(const std::vector<BundleSliceSegment>& segments,
                                            const float spacing_factor) {
  float average_length = 0.f;
  for (const auto& segment : segments)
    average_length += segment.rest_length;
  average_length /= glm::max(static_cast<float>(segments.size()), 1.f);
  const float spacing = glm::max(average_length * spacing_factor, 1e-6f);
  std::map<std::pair<int32_t, int32_t>, uint32_t> indices;
  std::vector<uint32_t> result(segments.size());
  for (size_t index = 0; index < segments.size(); ++index) {
    const auto key = std::make_pair(segments[index].node_handle,
                                    static_cast<int32_t>(glm::floor(segments[index].root_distance / spacing)));
    result[index] = indices.try_emplace(key, static_cast<uint32_t>(indices.size())).first->second;
  }
  return result;
}

BundleSliceFit FitBundleSliceReference(const std::vector<BundleSlicePoint>& points, const size_t minimum_points) {
  BundleSliceFit result;
  if (points.size() < minimum_points)
    return result;
  float mass_sum = 0.f;
  for (const auto& point : points) {
    const float mass = glm::max(point.mass, 1e-6f);
    mass_sum += mass;
    result.rest_center += mass * point.rest;
    result.center += mass * point.current;
  }
  result.rest_center /= mass_sum;
  result.center /= mass_sum;
  glm::mat3 covariance(0.f);
  for (const auto& point : points) {
    const float mass = glm::max(point.mass, 1e-6f);
    covariance += mass * glm::outerProduct(point.current - result.center, point.rest - result.rest_center);
  }
  float norm_squared = 0.f;
  for (int column = 0; column < 3; ++column)
    norm_squared += glm::dot(covariance[column], covariance[column]);
  if (norm_squared < 1e-12f)
    return result;
  glm::mat3 rotation = covariance / glm::sqrt(norm_squared);
  for (int iteration = 0; iteration < 8; ++iteration) {
    if (glm::abs(glm::determinant(rotation)) < 1e-6f)
      return result;
    rotation = .5f * (rotation + glm::inverse(glm::transpose(rotation)));
  }
  if (glm::determinant(rotation) < 0.f)
    rotation[2] = -rotation[2];
  result.rotation = glm::normalize(glm::quat_cast(rotation));
  result.valid = true;
  return result;
}

std::vector<BundleCoarseEdge> BuildBundleCoarseEdges(const std::vector<uint32_t>& segment_slices,
                                                     const std::vector<int32_t>& segment_groups,
                                                     const std::vector<BundleCoarsePair>& pairs) {
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> counts;
  for (const auto& pair : pairs) {
    if (pair.connectivity <= 1e-6f || segment_groups[pair.segment0] != segment_groups[pair.segment1])
      continue;
    const auto slices = std::minmax(segment_slices[pair.segment0], segment_slices[pair.segment1]);
    if (slices.first != slices.second)
      ++counts[slices];
  }
  std::vector<BundleCoarseEdge> result;
  result.reserve(counts.size());
  for (const auto& [slices, count] : counts)
    result.push_back({slices.first, slices.second, count});
  return result;
}
}  // namespace eco_sys_lab_package
