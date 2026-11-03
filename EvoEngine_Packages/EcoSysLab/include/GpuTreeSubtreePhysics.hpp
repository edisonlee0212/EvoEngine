#pragma once

#include <optional>
#include "GpuTreePhysics.hpp"

namespace eco_sys_lab_package {
// Rigid-subtree mechanics prototype with a driven base.
struct TreeSubtreeBody {
  struct Rod {
    glm::vec4 start_radius{}, end_length{}, relative_rotation{};
    glm::uvec4 indices{};
  };
  struct Vertex {
    glm::vec4 offset{};
    glm::uvec4 indices{};
  };
  glm::vec3 anchor{}, center_of_mass{};
  glm::quat rotation{1, 0, 0, 0};
  float mass = 0;
  glm::mat3 inertia_center{0}, inertia_pivot{0};
  std::vector<Rod> rods;
  std::vector<Vertex> vertices;
};
// Members must be unique, with the attachment rod first. Fixed members are allowed.
std::optional<TreeSubtreeBody> BuildTreeRigidBody(const DynamicTreeSkeleton::GpuRestState& rest,
                                                  const std::vector<uint32_t>& members);
std::optional<TreeSubtreeBody> BuildTreeSubtreeBody(const DynamicTreeSkeleton::GpuRestState& rest, uint32_t root_rod);
std::optional<std::array<TreeSubtreeBody, 2>> BuildTreeSubtreePair(const DynamicTreeSkeleton::GpuRestState& rest,
                                                                   uint32_t parent_rod, uint32_t child_rod);

class GpuTreeSubtreePhysics {
 public:
  struct Settings {
    float time_step = 0.016f;
    uint32_t substeps = 4;
    glm::vec3 gravity{0, -9.8f, 0};
    // Torque per radian in the attachment frame: twist X, bend Y/Z.
    glm::vec3 angular_stiffness{1};
    glm::vec3 base_angular_stiffness{1};  // Coupled mode: parent attachment to the driven base.
    float damping = 0;
    glm::vec3 wind_velocity{};
    float air_density = 1.225f, wind_drag = 20;
  };
  struct Motion {
    glm::vec4 rotation{0, 0, 0, 1}, angular_velocity{}, anchor{}, anchor_velocity{};
    glm::vec4 external_force{}, external_torque{};
  };
  bool Initialize(const DynamicTreeSkeleton::GpuRestState& rest, uint32_t root_rod);
  bool InitializeCoupled(const DynamicTreeSkeleton::GpuRestState& rest, uint32_t parent_rod, uint32_t child_rod);
  void Step(const Settings& settings, const glm::vec3& anchor, const glm::quat& target_rotation);
  const std::shared_ptr<GpuTreePhysicsState>& GetTreeState() const {
    return tree_;
  }
  const std::shared_ptr<Buffer>& GetMotionBuffer() const {
    return motion_;
  }

 private:
  struct Properties {
    glm::vec4 center_mass{}, inertia0{}, inertia1{}, inertia2{};
    glm::uvec4 counts{};
  };
  struct Parameters {
    glm::vec4 anchor{}, target_rotation{}, gravity{}, wind_density{}, stiffness_damping{}, step{}, base_stiffness{};
    glm::uvec4 control{};
  };
  struct Joint {
    glm::vec4 parent_offset{}, rest_rotation{};
  };
  bool InitializeBodies(const DynamicTreeSkeleton::GpuRestState& rest, const std::vector<TreeSubtreeBody>& bodies);
  static_assert(sizeof(Motion) == 96 && sizeof(Properties) == 80 && sizeof(Parameters) == 128);
  std::shared_ptr<GpuTreePhysicsState> tree_;
  std::shared_ptr<DescriptorSetLayout> layout_;
  std::shared_ptr<DescriptorSet> set_;
  std::shared_ptr<Buffer> properties_, motion_, rods_, vertices_, registry_, joint_;
  std::array<std::shared_ptr<ComputePipeline>, 2> pipelines_;
  uint32_t rod_count_ = 0, vertex_count_ = 0, body_count_ = 0;
};
}  // namespace eco_sys_lab_package
