#pragma once
#include "StrandGroup.hpp"
#include "StrandModelData.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DynamicStrandsPreStep;
class IDynamicStrandsOperator;
class IDynamicStrandsConstraint;

class DynamicStrands {
 public:
  DynamicStrands();
#pragma region Initialization
  struct InitializeParameters {
    bool static_root = true;
    float wood_density = 1.f;
    float shear_stiffness = 1.f;
    float stretch_stiffness = 1.f;

    float bending_stiffness = 0.9f;
    float twisting_stiffness = 0.9f;

    float velocity_damping = 0.0f;
    float angular_velocity_damping = 0.0f;


    float connectivity_detection_range = 0.05f;
    GlobalTransform root_transform{};
  };
  void InitializeStrandsGroup(const InitializeParameters& initialize_parameters,
                              const StrandModelStrandGroup& strand_group);
#pragma endregion
#pragma region Step
  struct PhysicsParameters {
    float time_step = 0.01f;
    uint32_t constraint_iteration = 5;
  };

  struct RenderParameters {
    std::shared_ptr<Camera> target_camera{};
  };

  struct StepParameters {
    bool physics = false;
    PhysicsParameters physics_parameters{};

    bool render = false;
    RenderParameters render_parameters;
  };

  std::vector<std::shared_ptr<IDynamicStrandsOperator>> operators;
  std::shared_ptr<DynamicStrandsPreStep> pre_step;
  std::vector<std::shared_ptr<IDynamicStrandsConstraint>> constraints;

  void Step(const StepParameters& target_step_parameters) const;
#pragma endregion
#pragma region Shared Data
  struct GpuStrand {
    int begin_segment_handle = -1;
    int end_segment_handle = -1;

    int begin_particle_handle = -1;
    int end_particle_handle = -1;
  };

  struct GpuSegment {
    int prev_handle = -1;
    int next_handle = -1;
    int strand_handle = -1;
    float inv_mass = 0.0f;

    glm::vec4 color;

    // Initial rotation
    glm::quat q0;
    // Current rotation
    glm::quat q;
    // Last frame rotation
    glm::quat last_q;
    glm::quat old_q;
    glm::vec3 torque = glm::vec3(0.f);
    float rest_length;

    float radius;
    float shearing_stiffness;
    float stretching_stiffness;
    float damping;
    

    glm::vec3 inertia_tensor;
    int particle0_handle = -1;

    glm::vec3 inv_inertia_tensor;
    int particle1_handle = -1;

    glm::mat4 inertia_w;
    glm::mat4 inv_inertia_w;
  };
  struct GpuParticle {
    // Initial position
    glm::vec3 x0;
    float damping;
    // Current position
    glm::vec3 x;
    int padding0;
    // Last frame position
    glm::vec3 last_x;
    int prev_handle = -1;
    glm::vec3 old_x;
    int next_handle = -1;

    glm::vec3 acceleration = glm::vec3(0.f);
    float inv_mass;

    int node_handle;
    int strand_handle;
    int padding1;
    int padding2;
  };
  inline static std::shared_ptr<DescriptorSetLayout> strands_layout{};

  std::shared_ptr<Buffer> device_strands_buffer;
  std::shared_ptr<Buffer> device_segments_buffer;
  std::shared_ptr<Buffer> device_particles_buffer;

  std::vector<GpuStrand> strands;
  std::vector<GpuSegment> segments;
  std::vector<GpuParticle> particles;
#pragma endregion

  void Upload() const;
  void Download();

  void Clear();

  std::vector<std::shared_ptr<DescriptorSet>> strands_descriptor_sets;

 private:
  static glm::vec3 ComputeInertiaTensorBox(float mass, float width, float height, float depth);
  static glm::vec3 ComputeInertiaTensorRod(float mass, float radius, float length);
  void InitializeOperators(const InitializeParameters& initialize_parameters) const;
  void InitializeConstraints(const InitializeParameters& initialize_parameters) const;
  void Render(const RenderParameters& render_parameters) const;
  void Physics(const PhysicsParameters& physics_parameters,
               const std::vector<std::shared_ptr<IDynamicStrandsOperator>>& target_operators,
               const std::shared_ptr<DynamicStrandsPreStep>& target_pre_step,
               const std::vector<std::shared_ptr<IDynamicStrandsConstraint>>& target_constraints) const;
};
}  // namespace eco_sys_lab_plugin