#pragma once
#include "StrandGroup.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

class DynamicStrands{
 public:
  struct InitializeParameters {
    
  };
  struct RenderParameters {
    
  };
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

  std::vector<StrandSegment> strand_segments;
  std::vector<GpuStrand> strands;
  std::vector<StrandSegmentHandle> strand_segment_handles;

  std::vector<std::shared_ptr<Buffer>> device_strand_segments_buffer;
  std::vector<std::shared_ptr<Buffer>> device_strands_buffer;
  std::vector<std::shared_ptr<Buffer>> device_strand_segment_handles_buffer;
  void Initialize(const InitializeParameters& initialize_parameters, const std::vector<Strand>& strands,
                  const std::vector<StrandSegment>& strand_segments);
  void Render(const RenderParameters& render_parameters,
      const std::shared_ptr<Camera>& camera, const GlobalTransform& camera_model,
              const GlobalTransform& strands_model) const;
  void Upload(uint32_t current_frame_index) const;
  void Download(uint32_t current_frame_index);
  void Clear();
};
}  // namespace eco_sys_lab_plugin