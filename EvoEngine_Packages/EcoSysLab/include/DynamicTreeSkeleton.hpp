#pragma once
#include "Tree.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;
class GpuTreePhysics;

class DynamicTreeSkeleton : public IPrivateComponent {
 public:
  struct GpuRestState {
    glm::mat4 initial_model{1.0f};
    struct Vertex {
      glm::vec4 position_inverse_mass;
      glm::vec4 velocity_radius;
      glm::vec4 rest_local;
      glm::uvec4 incident_rods;
    };
    struct Rod {
      glm::uvec4 links;
      glm::vec4 length_scale_mass_radius;
      glm::vec4 rotation;
      glm::vec4 rest_rotation;
      glm::uvec4 incident_angles;
    };
    struct Angle {
      glm::uvec4 rods;
      glm::vec4 rest_relative;
      glm::vec4 bend_scale;
    };
    std::vector<Vertex> vertices;
    std::vector<Rod> rods;
    std::vector<Angle> angles;
    std::vector<uint32_t> vertex_rods;
    std::vector<uint32_t> rod_angles;
    std::vector<uint32_t> node_rods;
  };

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
  bool gpu_physics_enabled = false;
  bool InitializePhysics(Tree& tree);
  bool InitializePhysics(Tree& tree, const GlobalTransform& model);
  void RequestGpuPoseInitialization() {
    restore_gpu_pose_on_load_ = gpu_physics_enabled;
  }
  void RestoreGpuPoseAfterLoad(Tree& tree);
  bool SyncGpuPhysics(Tree& tree, const GlobalTransform& model, bool growing);
  [[nodiscard]] GpuRestState ExportGpuRestState() const;
  void ResetPhysics(Tree& tree);
  void StepPhysics(Tree& tree);
  void StepPhysics(Tree& tree, const GlobalTransform& model);
  void SetGpuPhysics(std::shared_ptr<GpuTreePhysics> physics, uint32_t tree_index) {
    gpu_physics_ = std::move(physics);
    gpu_tree_index_ = tree_index;
  }
  [[nodiscard]] std::shared_ptr<GpuTreePhysics> GetGpuPhysics() const {
    return gpu_physics_;
  }
  [[nodiscard]] uint32_t GetGpuTreeIndex() const {
    return gpu_tree_index_;
  }
  [[nodiscard]] bool HasPhysics(const Tree& tree) const {
    return !physics_rods_.empty() && model_revision_ == tree.GetShootModelRevision().content;
  }
  [[nodiscard]] bool GpuScaleChanged(const GlobalTransform& model) const {
    return glm::any(glm::greaterThan(glm::abs(model.GetScale() - initial_scale_), glm::vec3(1e-4f)));
  }

 private:
  bool restore_gpu_pose_on_load_ = false;
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
  std::shared_ptr<GpuTreePhysics> gpu_physics_;
  uint32_t gpu_tree_index_ = 0;
};
static_assert(sizeof(DynamicTreeSkeleton::GpuRestState::Vertex) == 64);
static_assert(sizeof(DynamicTreeSkeleton::GpuRestState::Rod) == 80);
static_assert(sizeof(DynamicTreeSkeleton::GpuRestState::Angle) == 48);
}  // namespace eco_sys_lab_package
