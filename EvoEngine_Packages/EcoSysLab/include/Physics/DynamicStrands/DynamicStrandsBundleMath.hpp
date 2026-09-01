#pragma once

#include "glm/gtc/quaternion.hpp"

#include <vector>

namespace eco_sys_lab_package {

struct BundleRigidBodyState {
  glm::vec3 position{};
  glm::quat rotation{1.f, 0.f, 0.f, 0.f};
  float inverse_mass = 0.f;
  glm::mat3 inverse_inertia{0.f};
};

struct BundlePairReferenceState {
  glm::vec3 segment0_midpoint_offset{};
  glm::vec3 segment1_midpoint_offset{};
  glm::quat rest_relative_rotation{1.f, 0.f, 0.f, 0.f};
  glm::vec3 positional_compliance{};
  glm::vec3 angular_compliance{};
  glm::vec3 positional_lambda{};
  glm::vec3 angular_lambda{};
};

struct BundlePairCorrection {
  glm::vec3 position0{};
  glm::vec3 angular0{};
  glm::vec3 position1{};
  glm::vec3 angular1{};
};

struct BundleSliceSegment {
  int32_t node_handle = 0;
  float root_distance = 0.f;
  float rest_length = 0.f;
};

struct BundleSlicePoint {
  glm::vec3 rest{};
  glm::vec3 current{};
  float mass = 1.f;
};

struct BundleSliceFit {
  glm::vec3 rest_center{};
  glm::vec3 center{};
  glm::quat rotation{1.f, 0.f, 0.f, 0.f};
  bool valid = false;
};

struct BundleCoarsePair {
  uint32_t segment0 = 0;
  uint32_t segment1 = 0;
  float connectivity = 1.f;
};

struct BundleCoarseEdge {
  uint32_t slice0 = 0;
  uint32_t slice1 = 0;
  uint32_t count = 0;

  bool operator==(const BundleCoarseEdge& other) const {
    return slice0 == other.slice0 && slice1 == other.slice1 && count == other.count;
  }
};

[[nodiscard]] glm::vec3 BundleQuaternionLog(glm::quat rotation);
[[nodiscard]] BundlePairCorrection SolveBundlePairReference(const BundleRigidBodyState& body0,
                                                            const BundleRigidBodyState& body1,
                                                            BundlePairReferenceState& constraint);
void ApplyBundlePairCorrection(BundleRigidBodyState& body0, BundleRigidBodyState& body1,
                               const BundlePairCorrection& correction);
[[nodiscard]] std::vector<uint32_t> BuildBundleBaseSlices(const std::vector<BundleSliceSegment>& segments,
                                                          float spacing_factor);
[[nodiscard]] BundleSliceFit FitBundleSliceReference(const std::vector<BundleSlicePoint>& points,
                                                     size_t minimum_points);
[[nodiscard]] std::vector<BundleCoarseEdge> BuildBundleCoarseEdges(const std::vector<uint32_t>& segment_slices,
                                                                   const std::vector<int32_t>& segment_groups,
                                                                   const std::vector<BundleCoarsePair>& pairs);

}  // namespace eco_sys_lab_package
