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
  std::shared_ptr<Buffer> device_ref_strand_segment_handles_buffer[2];

  std::shared_ptr<Buffer> device_strand_segments_buffer[2];
  std::shared_ptr<Buffer> device_strands_buffer[2];
  std::shared_ptr<Buffer> device_strand_segment_handles_buffer[2];

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
    int32_t strand_segment_handles_offset = -1;
    int32_t strand_segment_handles_size = -1;
    StrandSegmentHandle first_strand_segment_handle = -1;
    StrandSegmentHandle last_strand_segment_handle = -1;

    /**
     * \brief The position of the [[[START]]] of first strand segment.
     */
    glm::vec3 start_position = glm::vec3(0.0f);
    /**
     * \brief The thickness of the [[[START]]] current strand segment.
     */
    float start_thickness = 0.0f;

    /**
     * \brief The color of the [[[START]]] current strand segment.
     */
    glm::vec4 start_color = glm::vec4(1.0f);
  };

  std::vector<StrandSegment> ref_strand_segments;
  std::vector<GpuStrand> ref_strands;
  std::vector<StrandSegmentHandle> ref_strand_segment_handles;
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