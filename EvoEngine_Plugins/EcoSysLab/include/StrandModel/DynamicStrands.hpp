#pragma once
#include "StrandGroup.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

class DynamicStrands {
  void Upload() const;
  bool current_left = true;
  uint32_t current_version = 0;
  uint32_t device_buffer_version[2];
  std::shared_ptr<Buffer> device_ref_strand_segments_buffer[2];
  std::shared_ptr<Buffer> device_ref_strands_buffer[2];
  std::shared_ptr<Buffer> device_ref_strand_segment_particles_buffer[2];

  std::shared_ptr<Buffer> device_strand_segments_buffer[2];
  std::shared_ptr<Buffer> device_strands_buffer[2];
  std::shared_ptr<Buffer> device_strand_segment_particles_buffer[2];

 public:
  struct InitializeParameters {
    GlobalTransform root_transform{};
  };

  struct StepParameters {
    bool physics = true;
    struct PhysicsParameters {
    } physics_parameters{};

    bool render = true;
    struct RenderParameters {
      std::shared_ptr<Camera> target_camera{};
    } render_parameters;
  } step_parameters{};

  struct GpuStrand {
    StrandSegmentHandle first_strand_segment_handle = -1;
    StrandSegmentHandle last_strand_segment_handle = -1;
  };
  struct GpuStrandSegment {
    int prev_handle = -1;
    int next_handle = -1;
    int strand_handle = -1;
    int index = -1;

    glm::quat rotation;

    int start_particle_index;
    int end_particle_index;
    int neighbors[10];
  };
  struct GpuStrandSegmentParticle {
    glm::vec3 position;
    float thickness;
    glm::vec4 color;
  };
  std::vector<GpuStrandSegment> ref_strand_segments;
  std::vector<GpuStrandSegmentParticle> ref_strand_segment_particles;
  std::vector<GpuStrand> ref_strands;
  inline static std::shared_ptr<DescriptorSetLayout> strands_layout{};
  DynamicStrands();
  void UpdateData(const InitializeParameters& initialize_parameters, const std::vector<Strand>& target_strands,
                  const std::vector<StrandSegment>& target_strand_segments);
  void Step(const StepParameters& target_step_parameters);
  void Step();
  void Clear();

 private:
  void BindStrandsDescriptorSet(const std::shared_ptr<DescriptorSet>& target_descriptor_set) const;

  void Render(const StepParameters::RenderParameters& render_parameters) const;
  void Physics(const StepParameters::PhysicsParameters& physics_parameters) const;
};
}  // namespace eco_sys_lab_plugin