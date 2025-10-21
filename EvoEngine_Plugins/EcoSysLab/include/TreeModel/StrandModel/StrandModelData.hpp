
#pragma once
#include "Skeleton.hpp"
#include "StrandGroup.hpp"

#include "StrandModelProfile.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * \brief Represents data associated with a strand group in the strand model.
 */
struct StrandModelStrandGroupData {};

/**
 * \brief Represents data associated with an individual strand in the strand model.
 */
struct StrandModelStrandData {};

/**
 * \brief Represents data for a segment within a strand in the strand model.
 */
struct StrandModelStrandSegmentData {
  /**
   * \brief The handle of the internode this pipe segment belongs to.
   *
   * The hierarchy follows: Pipe -> PipeSegment <-> Cell <- Profile <- Internode.
   */
  SkeletonNodeHandle node_handle = -1;

  /**
   * \brief Handle to the particle within the profile that corresponds to this segment.
   */
  ParticleHandle profile_particle_handle = -1;

  /**
   * \brief Indicates whether this segment lies on the boundary of the structure.
   */
  bool is_boundary = false;

  /**
   * \brief Position of the profile in 2D space relative to the strand model.
   */
  glm::vec2 profile_position;

  /**
   * \brief Initial distance of this segment from the boundary.
   */
  float initial_distance_to_boundary;
};

/**
 * \brief Typedef for a strand group in the strand model, parameterized with specific data structures.
 */
typedef StrandGroup<StrandModelStrandGroupData, StrandModelStrandData, StrandModelStrandSegmentData>
    StrandModelStrandGroup;

/**
 * \brief Represents physics-related data associated with a cell particle in the strand model.
 */
struct CellParticlePhysicsData {};

}  // namespace eco_sys_lab_plugin
