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
 * \brief Tissue classification for particles and strand segments.
 */
enum class TissueType : uint8_t {
  kSapwood = 0,     ///< Living wood that conducts water.
  kHeartwood = 1,   ///< Dead inner wood (retired sapwood).
  kBarkLiving = 2,  ///< Living bark (phloem / inner bark).
  kBarkDead = 3     ///< Dead outer bark (rhytidome).
};

/**
 * \brief Wound healing state for particles and strand segments.
 */
enum class WoundState : uint8_t {
  kNone = 0,     ///< No wound.
  kWounded = 1,  ///< Freshly wounded (exposed tissue).
  kHealing = 2,  ///< Callus tissue forming over wound.
  kHealed = 3    ///< Wound fully sealed by callus.
};

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

  /**
   * \brief Biological tissue classification (sapwood, heartwood, bark).
   * Copied from the corresponding profile particle during ApplyProfiles().
   */
  TissueType tissue_type = TissueType::kSapwood;

  /**
   * \brief Wound healing state at this segment location.
   * Copied from the corresponding profile particle during ApplyProfiles().
   */
  WoundState wound_state = WoundState::kNone;
};

/**
 * \brief Typedef for a strand group in the strand model, parameterized with specific data structures.
 */
typedef StrandGroup<StrandModelStrandGroupData, StrandModelStrandData, StrandModelStrandSegmentData>
    StrandModelStrandGroup;

/**
 * \brief Represents physics-related data associated with a cell particle in the strand model.
 *
 * Carried by each Particle2D<CellParticlePhysicsData> in the 2D profile system.
 * Stores biological metadata that is propagated to 3D strand segments via ApplyProfiles().
 */
struct CellParticlePhysicsData {
  /// Biological tissue classification.
  TissueType tissue_type = TissueType::kSapwood;

  /// Wound healing state.
  WoundState wound_state = WoundState::kNone;

  /// Growth step when this particle was created (for future annual ring tracking).
  uint16_t birth_step = 0;
};
}  // namespace eco_sys_lab_plugin