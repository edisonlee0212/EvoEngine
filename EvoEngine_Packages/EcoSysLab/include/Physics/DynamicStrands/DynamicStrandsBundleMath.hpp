#pragma once

#include "glm/gtc/quaternion.hpp"

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

[[nodiscard]] glm::vec3 BundleQuaternionLog(glm::quat rotation);
[[nodiscard]] BundlePairCorrection SolveBundlePairReference(const BundleRigidBodyState& body0,
                                                            const BundleRigidBodyState& body1,
                                                            BundlePairReferenceState& constraint);
void ApplyBundlePairCorrection(BundleRigidBodyState& body0, BundleRigidBodyState& body1,
                               const BundlePairCorrection& correction);

}  // namespace eco_sys_lab_package
