#pragma once
#include "StrandGroup.hpp"
#include "StrandModelData.hpp"
#include "TreeGrowthData.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DynamicStrandsPreStep;
class IDynamicStrandsOperator;
class IDynamicStrandsConstraint;
class DynamicStrandsPrediction;
class DynamicStrands {
 public:
  DynamicStrands();
#pragma region Initialization
  struct InitializeParameters {
    bool static_root = true;
    float wood_density = 1.f;
    float shear_stiffness = 1.f;
    float stretch_stiffness = 1.f;

    float bending_stiffness = 1.f;
    float twisting_stiffness = 1.f;

    float velocity_damping = 0.01f;
    float angular_velocity_damping = 0.01f;

    float neighbor_range = 0.05f;
    GlobalTransform root_transform{};

    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  void Initialize(const InitializeParameters& initialize_parameters, const StrandModelSkeleton& strand_model_skeleton,
                  const StrandModelStrandGroup& strand_group);

#pragma endregion
#pragma region Step
  struct PhysicsParameters {
    float time_step = 0.01f;
    int constraint_iteration = 50;
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct VisualizationParameters {
    enum class RenderMode { Default, BendTwistStrain, StretchShearStrain, ConnectivityStrain };

    uint32_t render_mode = 1;
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);
    float multiplier = 1.0f;
    std::shared_ptr<Camera> target_visualization_camera{};

    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct StepParameters {
    bool physics = false;
    PhysicsParameters physics_parameters{};

    bool visualization = false;
    VisualizationParameters visualization_parameters;
  };
  std::shared_ptr<DynamicStrandsPreStep> pre_step;

  std::vector<std::shared_ptr<IDynamicStrandsOperator>> operators;
  std::shared_ptr<DynamicStrandsPrediction> prediction;
  std::vector<std::shared_ptr<IDynamicStrandsConstraint>> constraints;

  void Step(const StepParameters& target_step_parameters) const;
#pragma endregion
#pragma region Shared Data
  struct GpuStrand {
    int begin_segment_handle = -1;
    int end_segment_handle = -1;

    int begin_connection_handle = -1;
    int end_connection_handle = -1;
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

    float bend_twist_strain0 = 0.0;
    float bend_twist_strain1 = 0.0;
    float stretch_shear_strain = 0.0;
    float padding;
  };
  struct GpuParticle {
    // Initial position
    glm::vec3 x0;
    float damping;
    // Current position
    glm::vec3 x;
    int padding0 = -1;
    // Last frame position
    glm::vec3 last_x;
    int padding1 = -1;
    glm::vec3 old_x;
    int padding2 = -1;

    glm::vec3 acceleration = glm::vec3(0.f);
    float inv_mass;

    int node_handle;
    int strand_handle;
    int segment_handle;
    float connectivity_strain = 0.0;
  };

  struct GpuConnection {
    int segment0_handle;
    int segment1_handle;
    int segment0_particle_handle;
    int segment1_particle_handle;

    glm::quat rest_darboux_vector;
    float bending_stiffness;
    float twisting_stiffness;

    int prev_handle = -1;
    int next_handle = -1;
  };

  inline static std::shared_ptr<DescriptorSetLayout> strands_layout{};

  std::shared_ptr<Buffer> device_strands_buffer;
  std::shared_ptr<Buffer> device_segments_buffer;
  std::shared_ptr<Buffer> device_particles_buffer;
  std::shared_ptr<Buffer> device_connections_buffer;
  std::vector<GpuStrand> strands;
  std::vector<GpuSegment> segments;
  std::vector<GpuParticle> particles;
  std::vector<GpuConnection> connections;
#pragma endregion

  void Upload() const;
  void Download();

  void Clear();

  std::vector<std::shared_ptr<DescriptorSet>> strands_descriptor_sets;

 private:
  static glm::vec3 ComputeInertiaTensorBox(float mass, float width, float height, float depth);
  static glm::vec3 ComputeInertiaTensorRod(float mass, float radius, float length);
  void Visualization(const VisualizationParameters& render_parameters) const;
  void Physics(const PhysicsParameters& physics_parameters) const;
};
}  // namespace eco_sys_lab_plugin