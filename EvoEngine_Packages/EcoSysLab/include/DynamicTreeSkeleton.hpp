#pragma once
#include "Tree.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

class DynamicTreeSkeleton : public IPrivateComponent {
 public:
  struct PhysicsSettings {
    float time_step = 0.016f;
    int substeps = 4;
    int iterations = 4;
    float stretch_stiffness = 100000.0f;
    float bend_stiffness = 5000.0f;
    float density = 750.0f;
    float damping = 2.0f;
    glm::vec3 gravity{0, -9.81f, 0};
    glm::vec3 tip_force{};
    SkeletonNodeHandle force_node = -1;
    glm::vec3 anchor_offset{};
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
    bool pinned = false;
  };
  struct PhysicsRod {
    int start = -1;
    int end = -1;
    SkeletonNodeHandle node = -1;
    float length = 0;
    float area_scale = 1;
    float bend_scale = 1;
    glm::quat rotation{1, 0, 0, 0};
    glm::quat rest_rotation{1, 0, 0, 0};
    glm::vec3 visual_offset{};
    bool pinned = false;
  };
  struct PhysicsAngle {
    int first = -1;
    int second = -1;
    glm::quat rest_relative{1, 0, 0, 0};
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
};
}  // namespace eco_sys_lab_package
