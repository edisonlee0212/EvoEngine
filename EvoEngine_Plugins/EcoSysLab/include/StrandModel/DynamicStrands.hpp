#pragma once
#include "StrandGroup.hpp"

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
    GlobalTransform root_transform{};
  };
  template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
  void InitializeStrandsGroup(const InitializeParameters& initialize_parameters,
                              const StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strand_group);
#pragma endregion
#pragma region Step
  struct PhysicsParameters {
    float time_step = 0.01f;
    uint32_t max_iteration = 50;
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

  void Step(const StepParameters& target_step_parameters);
#pragma endregion
#pragma region Shared Data
  struct GpuStrand {
    int begin_segment_handle = -1;
    int end_segment_handle = -1;

    int segment_size;
    int padding0;
  };

  struct GpuSegment {
    int prev_handle = -1;
    int next_handle = -1;
    int strand_handle = -1;
    float inv_mass = 0.0f;

    glm::vec3 color;
    float radius;

    // Initial rotation
    glm::quat q0;
    // Current rotation
    glm::quat q;
    // Last frame rotation
    glm::quat last_q;
    glm::quat old_q;
    glm::vec3 torque = glm::vec3(0.f);
    float rest_length;

    float shearing_stiffness;
    float stretching_stiffness;
    float damping;
    float padding1;

    glm::vec3 inertia_tensor;
    int particle0_handle = -1;

    glm::vec3 inv_inertia_tensor;
    int particle1_handle = -1;

    glm::mat4 inertia_w;
    glm::mat4 inv_inertia_w;

    int neighbors[8];

    glm::vec3 p0;
    int p0_handle;
    glm::vec3 p1;
    int p1_handle;

    glm::vec4 original_gamma;
    glm::vec4 d3;
    glm::vec4 gamma;
  };
  struct GpuParticle {
    // Initial position
    glm::vec3 x0;
    float damping;
    // Current position
    glm::vec4 x;
    // Last frame position
    glm::vec4 last_x;
    glm::vec4 old_x;

    glm::vec3 acceleration = glm::vec3(0.f);
    float inv_mass;
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
               const std::vector<std::shared_ptr<IDynamicStrandsConstraint>>& target_constraints);
};

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void DynamicStrands::InitializeStrandsGroup(
    const InitializeParameters& initialize_parameters,
    const StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strand_group) {
  Clear();
  assert(initialize_parameters.root_transform.GetScale() == glm::vec3(1.0f));
  const auto& target_strands = strand_group.PeekStrands();
  const auto& target_strand_segments = strand_group.PeekStrandSegments();
  strands.resize(target_strands.size());

  Jobs::RunParallelFor(target_strands.size(), [&](const size_t i) {
    auto& strand = strands[i];
    const auto& target_strand = target_strands[i];
    const auto& handles = target_strand.PeekStrandSegmentHandles();
    strand.segment_size = handles.size();
    if (!handles.empty()) {
      strand.begin_segment_handle = handles.front();
      strand.end_segment_handle = handles.back();
    } else {
      strand.begin_segment_handle = -1;
      strand.end_segment_handle = -1;
    }
  });

  segments.resize(target_strand_segments.size());
  Jobs::RunParallelFor(target_strand_segments.size(), [&](const size_t i) {
    auto& segment = segments[i];
    const auto& target_strand_segment = target_strand_segments[i];
    segment.prev_handle = target_strand_segment.GetPrevHandle();
    segment.next_handle = target_strand_segment.GetNextHandle();
    segment.strand_handle = target_strand_segment.GetStrandHandle();
    segment.rest_length = strand_group.GetStrandSegmentLength(i);

    segment.color = target_strand_segment.end_color;
    segment.radius = target_strand_segment.end_thickness * .5f;
    segment.damping = initialize_parameters.angular_velocity_damping;
    segment.q0 = segment.q = segment.last_q = segment.old_q =
        initialize_parameters.root_transform.GetRotation() * target_strand_segment.rotation;
    segment.torque = glm::vec3(0.f);

    if (segment.prev_handle == -1 && initialize_parameters.static_root) {
      segment.inertia_tensor = segment.inv_inertia_tensor = glm::vec3(0.0f);
      segment.inv_mass = 0.f;
    } else {
      const float mass =
          segment.radius * segment.radius * glm::pi<float>() * initialize_parameters.wood_density * segment.rest_length;
      segment.inertia_tensor = ComputeInertiaTensorRod(mass, segment.radius, segment.rest_length);
      segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
      segment.inv_mass = 1.f / mass;
    }

    /*
    const float youngs_modulus = initialize_parameters.youngs_modulus * 1000000000.f;
    const float shear_modulus = initialize_parameters.torsion_modulus * 1000000000.f;
    const auto second_moment_of_area = glm::pi<float>() * std::pow(segment.radius * .5f, 4.f);
    const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(segment.radius, 4.f) / 2.f;

    segment.stretching_stiffness = segment.shearing_stiffness =
        youngs_modulus * glm::pi<float>() * segment.radius * segment.radius / segment.rest_length;
    segment.bending_stiffness = youngs_modulus * second_moment_of_area / glm::pow(segment.rest_length, 3.f);
    segment.twisting_stiffness = shear_modulus * polar_moment_of_inertia / segment.rest_length;*/

    segment.stretching_stiffness = initialize_parameters.stretch_stiffness;
    segment.shearing_stiffness = initialize_parameters.shear_stiffness;
  });

  particles.resize(strands.size() + segments.size());

  int particle_index = 0;
  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& target_strand = target_strands[strand_index];
    auto& first_particle = particles[particle_index];
    first_particle.x = first_particle.last_x = first_particle.old_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(target_strand.start_position), 0.0);
    first_particle.damping = initialize_parameters.velocity_damping;
    first_particle.x0 = first_particle.x;
    first_particle.acceleration = glm::vec3(0.f);
    first_particle.inv_mass = 0;
    particle_index++;

    const auto& handles = target_strand.PeekStrandSegmentHandles();
    for (const auto& i : handles) {
      auto& segment = segments[i];
      segment.particle0_handle = particle_index - 1;
      segment.particle1_handle = particle_index;
      const auto segment_length = strand_group.GetStrandSegmentLength(i);
      auto& particle = particles[particle_index];

      particle.x = particle.last_x = particle.old_x =
          glm::vec4(initialize_parameters.root_transform.TransformPoint(target_strand_segments[i].end_position), 0.0);

      particle.x0 = particle.x;
      particle.acceleration = glm::vec3(0.f);
      particle.damping = initialize_parameters.velocity_damping;
      const float mass =
          segment.radius * segment.radius * glm::pi<float>() * initialize_parameters.wood_density * segment_length;
      particle.inv_mass = 1.f / mass;

      particle_index++;
    }
    strands[strand_index].begin_segment_handle = handles.front();
    strands[strand_index].end_segment_handle = handles.back();
  }

  Upload();

  InitializeOperators(initialize_parameters);
  InitializeConstraints(initialize_parameters);
}
}  // namespace eco_sys_lab_plugin