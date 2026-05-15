#pragma once
#include "Skeleton.hpp"
#include "StrandGroup.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

struct DtsStrandGroupData {};

struct DtsStrandData {};

/**
 * \brief Stores data related to a single strand segment.
 */
struct DtsStrandSegmentData {
  float start_root_distance = 0.0f;  ///< Distance from the strand root to the start of this segment.
  float end_root_distance = 0.0f;    ///< Distance from the strand root to the end of this segment.
  uint32_t original_segment_index;   ///< Index of the original segment this corresponds to.

  /**
   * \brief The handle of the internode this pipe segment belongs to.
   * Pipe -> PipeSegment <-> Cell <- Profile <- Internode
   */
  SkeletonNodeHandle node_handle = -1;
  StrandSegmentHandle original_segment_handle;

  float original_segment_t;  ///< Parameterized position within the strand's original segmentation.
  uint32_t segment_index;    ///< Index of the segment within the strand.

  glm::vec2 profile_position;          ///< Position in the profile space.
  glm::vec2 profile_polar_coordinate;  ///< Polar coordinate in the segment profile.
  float initial_distance_to_boundary;  ///< Initial computed distance to the segment boundary.
};

typedef StrandGroup<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData> DtsStrandGroup;
}  // namespace eco_sys_lab_package