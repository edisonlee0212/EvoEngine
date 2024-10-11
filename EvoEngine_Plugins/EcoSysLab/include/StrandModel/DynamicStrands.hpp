#pragma once
#include "StrandGroup.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DynamicStrandsPreStep;
class DynamicStrandsPostStep;
class IDynamicStrandsOperator;
class IDynamicStrandsConstraint;

class DynamicStrands {
 public:
  DynamicStrands();
#pragma region Initialization
  struct InitializeParameters {
    bool static_root = true;
    float wood_density = 1.f;
    float youngs_modulus = 1.f;
    float torsion_modulus = 1.f;
    GlobalTransform root_transform{};
  };
  template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
  void InitializeStrandsGroup(const InitializeParameters& initialize_parameters,
                      const StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strand_group);
#pragma endregion
#pragma region Step
  struct PhysicsParameters {
    float time_step = 0.01f;
    uint32_t sub_step = 1;
    uint32_t max_iteration;
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
  std::shared_ptr<DynamicStrandsPostStep> post_step;

  void Step(const StepParameters& target_step_parameters);
#pragma endregion
#pragma region Shared Data
  struct GpuStrand {
    int begin_segment_handle = -1;
    int end_segment_handle = -1;

    int segment_size;
    int padding0;
  };

  struct GpuStrandSegment {
    int prev_handle = -1;
    int next_handle = -1;
    int strand_handle = -1;
    int index = -1;

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
    float padding0;

    // Initial position
    glm::vec3 x0;
    float length;
    // Current position
    glm::vec4 x;
    // Last frame position
    glm::vec4 last_x;
    glm::vec4 old_x;

    glm::vec3 acceleration = glm::vec3(0.f);
    float padding1;

    glm::vec3 inertia_tensor;
    float mass;

    glm::vec3 inv_inertia_tensor;
    float inv_mass;

    glm::mat4 inertia_w;
    glm::mat4 inverse_inertia_w;

    int neighbors[8];
  };

  inline static std::shared_ptr<DescriptorSetLayout> strands_layout{};

  std::shared_ptr<Buffer> device_strand_segments_buffer;
  std::shared_ptr<Buffer> device_strands_buffer;

  std::vector<GpuStrand> strands;
  std::vector<GpuStrandSegment> strand_segments;
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
               const std::vector<std::shared_ptr<IDynamicStrandsConstraint>>& target_constraints,
               const std::shared_ptr<DynamicStrandsPostStep>& target_post_step);
};

template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
void DynamicStrands::InitializeStrandsGroup(const InitializeParameters& initialize_parameters,
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
    }else {
      strand.begin_segment_handle = -1;
      strand.end_segment_handle = -1;
    }
  });

  strand_segments.resize(target_strand_segments.size());
  Jobs::RunParallelFor(target_strand_segments.size(), [&](const size_t i) {
    auto& segment = strand_segments[i];
    const auto& target_strand_segment = target_strand_segments[i];
    segment.prev_handle = target_strand_segment.GetPrevHandle();
    segment.next_handle = target_strand_segment.GetNextHandle();
    segment.strand_handle = target_strand_segment.GetStrandHandle();
    segment.index = target_strand_segment.GetIndex();

    segment.color = target_strand_segment.end_color;
    segment.radius = target_strand_segment.end_thickness * .5f;

    segment.q0 = segment.q = segment.last_q = segment.old_q =
        initialize_parameters.root_transform.GetRotation() * target_strand_segment.rotation;
    segment.torque = glm::vec3(0.f);
    
    segment.x = segment.last_x = segment.old_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(strand_group.GetStrandSegmentCenter(static_cast<int>(i))), 0.0);

    segment.x0 = segment.x;
    segment.acceleration = glm::vec3(0.f);

    if (segment.prev_handle == -1 && initialize_parameters.static_root) {
      segment.mass = segment.inv_mass = 0.0f;
      segment.inertia_tensor = segment.inv_inertia_tensor = glm::vec3(0.0f);
    }else {
      segment.mass = segment.radius * segment.radius * glm::pi<float>() * initialize_parameters.wood_density;
      segment.inv_mass = 1.f / segment.mass;
      segment.inertia_tensor = ComputeInertiaTensorRod(segment.mass, segment.radius, segment.length);
      segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
    }
    
    segment.length = strand_group.GetStrandSegmentLength(static_cast<int>(i));
  });
  Upload();

  InitializeOperators(initialize_parameters);
  InitializeConstraints(initialize_parameters);
}
}  // namespace eco_sys_lab_plugin