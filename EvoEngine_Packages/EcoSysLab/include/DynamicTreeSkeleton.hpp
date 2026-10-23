#pragma once
#include "Tree.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class DynamicTreeSkeleton : public IPrivateComponent {
 public:
  struct PhysicsSettings {
    float time_step = 0.016f;
    int substeps = 4;
    int iterations = 8;
    float stretch_stiffness = 8192.0f;
    float bend_stiffness = 512.0f;
    float density = 700.0f;
    float damping = 0.0f;
    float contact_friction = 0.6f;
    glm::vec3 gravity{0, -9.8f, 0};
    glm::vec3 anchor_offset{};
    bool fracture_enabled = false;
    float fracture_angle_degrees = 45.0f;
    float fracture_duration = 0.05f;

    [[nodiscard]] float FractureBendLimit(const float joint_radius, const float joint_length) const {
      return glm::radians(glm::clamp(fracture_angle_degrees, 0.0f, 180.0f)) * 0.05f * joint_length /
             glm::max(joint_radius, 1e-4f);
    }
  } physics_settings;

  bool physics_running = false;
  bool InitializePhysics(Tree& tree);
  bool InitializePhysics(Tree& tree, const GlobalTransform& model);
  void ResetPhysics(Tree& tree);
  void StepPhysics(Tree& tree);
  void StepPhysics(Tree& tree, const GlobalTransform& model);
  [[nodiscard]] bool HasPhysics(const Tree& tree) const {
    return !physics_rods_.empty() && model_revision_ == tree.GetShootModelRevision().content;
  }

 private:
  struct PhysicsVertex {
    glm::vec3 position{};
    glm::vec3 velocity{};
    float inverse_mass = 0;
    float radius = 0;
    bool pinned = false;
    bool fallen = false;
  };
  struct PhysicsRod {
    int start = -1;
    int end = -1;
    SkeletonNodeHandle node = -1;
    float length = 0;
    float mass = 0;
    float radius = 0;
    float structural_length_scale = 1;
    glm::quat rotation{1, 0, 0, 0};
    glm::quat rest_rotation{1, 0, 0, 0};
    glm::vec3 visual_offset{};
    bool pinned = false;
    bool detached = false;
    bool fallen = false;
  };
  struct PhysicsAngle {
    int first = -1;
    int second = -1;
    glm::quat rest_relative{1, 0, 0, 0};
    float bend_scale = 1;
    float over_limit_time = 0;
    bool broken = false;
  };
  std::vector<PhysicsVertex> physics_vertices_;
  std::vector<PhysicsRod> physics_rods_;
  std::vector<PhysicsAngle> physics_angles_;
  std::vector<int> physics_node_rods_;
  std::vector<glm::vec3> physics_predicted_;
  std::vector<glm::vec3> physics_previous_;
  std::vector<std::vector<int>> physics_vertex_rods_;
  std::vector<std::vector<int>> physics_rod_angles_;
  std::vector<glm::vec3> physics_rest_positions_;
  glm::mat4 previous_model_{1.0f};
  glm::vec3 initial_scale_{1.0f};
  glm::quat initial_rotation_{1, 0, 0, 0};
  glm::quat previous_rotation_{1, 0, 0, 0};
  uint64_t model_revision_ = 0;
  bool has_detached_branches_ = false;
};
}  // namespace eco_sys_lab_package
