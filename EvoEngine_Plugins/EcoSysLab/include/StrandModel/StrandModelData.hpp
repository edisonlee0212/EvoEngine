#pragma once
#include "Skeleton.hpp"
#include "StrandGroup.hpp"

#include "StrandModelProfile.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
struct StrandModelStrandGroupData {};

struct StrandModelStrandData {};

struct StrandModelStrandSegmentData {
  /**
   * \brief The handle of the internode this pipe segment belongs to. Pipe -> PipeSegment <-> Cell <- Profile <-
   * Internode
   */
  SkeletonNodeHandle node_handle = -1;
  ParticleHandle profile_particle_handle = -1;
};

typedef StrandGroup<StrandModelStrandGroupData, StrandModelStrandData, StrandModelStrandSegmentData>
    StrandModelStrandGroup;

struct CellParticlePhysicsData {};
}  // namespace eco_sys_lab