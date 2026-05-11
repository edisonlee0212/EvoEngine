#pragma once

// =============================================================================
//  TasselGrowthSoA — std430 SoA layout consumed by grow.comp + propagate.comp.
//
//  Phase 2a contract (design-only, no Vulkan plumbing):
//
//    * One flat block of parallel arrays per instance, all length = node_count.
//    * Tightly packed; no per-node struct, to avoid std430 vec3 stride
//      pitfalls (see /memories/glsl_pitfalls.md).
//    * GPU-friendly types throughout: int / uint / vec4 / vec2.
//
//  Only compiled when LSYSTEM_GPU_PIPELINE is defined. The CPU mirror types
//  here are exact std430-compatible POD; the packer (TasselGrowthPacker.cpp)
//  produces them from a TasselGraph; LSystemGPUEngine uploads them; the
//  shaders read them.
//
//  Channel breakdown
//  -----------------
//   parent_index     int   per node, -1 = root, used by propagate.comp
//   depth            uint  per node, BFS depth band, used to dispatch waves
//   type_tag         uint  per node, mirrors TasselSymbol enum (0..4)
//   local_rotation   vec4  per node, quaternion (x,y,z,w), pre-baked by CPU
//                          (the existing local_rotation_fn in MaizeTassel.cpp
//                          is type-dispatched and captures descriptor state;
//                          baking it CPU-side keeps grow.comp/propagate.comp
//                          plant-agnostic in Phase 2a)
//   growth_state     vec4  per node, (age_gdd, target_length, target_thickness,
//                          packed_flags) — packed_flags bit 0 = is_spike
//   node_random_misc vec4  per node, (node_random, order_as_float,
//                          growth_progress_in, type_specific_aux)
//                          The .w slot carries type-specific aux scalars
//                          (phyllotaxis_azimuth for SpikeletPair etc.) —
//                          see TasselGrowthPacker.cpp for the per-type map.
//   length_thickness vec2  per node, OUTPUT of grow.comp:
//                          (length, half_thickness)
//   global_position  vec4  per node, OUTPUT of propagate.comp:
//                          (px, py, pz, length)  — w channel duplicates
//                          length so the mesh shader needs only one
//                          buffer binding for instance position+length
//   global_rotation  vec4  per node, OUTPUT of propagate.comp: quaternion
//
//  Header buffer (separate SSBO, push-constant-sized)
//  --------------------------------------------------
//   uint node_count, uint depth_band_count, uint instance_id, uint flags
//   uint depth_band_offsets[MAX_DEPTH_BANDS+1]   // prefix-sum start indices
//                                                  into a depth-sorted index
//                                                  permutation (filled by the
//                                                  packer; propagate.comp
//                                                  dispatches one wave per
//                                                  band reading nodes in
//                                                  band-permuted order).
//
//  Phase 3 will replace the CPU pre-baked local_rotation channel with a
//  derive-time GPU computation. Phase 4 will add an instance dimension to
//  every array and a per-instance offset table.
// =============================================================================

#include <cstdint>
#include <vector>

#if defined(LSYSTEM_GPU_PIPELINE)

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace l_system_plugin::gpu {

// ---- Compile-time limits (mirrored in shader as #define) -------------------

// Hard cap on BFS depth bands. Maize tassel observed max ~12; conifer worst
// case ~25. Bumping requires bumping the shader-side define and rebuilding.
inline constexpr uint32_t kMaxDepthBands = 32;

// Number of taps per Curve2D in the baked curve atlas.
inline constexpr uint32_t kCurveBakeSamples = 256;

// Type-tag values; MUST match l_system_plugin::TasselSymbol exactly.
enum class TasselTypeTag : uint32_t {
  Apex          = 0,
  Internode     = 1,
  Lateral       = 2,
  SpikeletPair  = 3,
  SpikeApex     = 4,
};

// Bit definitions for the growth_state.w packed_flags channel.
inline constexpr uint32_t kFlagIsSpike      = 1u << 0;
inline constexpr uint32_t kFlagMainRachis   = 1u << 1;  // reserved for pair
inline constexpr uint32_t kFlagFinitenessOk = 1u << 31; // packer-side check

// ---- Header (one per instance) --------------------------------------------

struct TasselGrowthHeader {
  uint32_t node_count        = 0;
  uint32_t depth_band_count  = 0;
  uint32_t instance_id       = 0;
  uint32_t flags             = 0;

  // Inclusive-exclusive offsets into depth_sorted_index[]. Always
  // depth_band_offsets[depth_band_count] == node_count.
  // Sized at kMaxDepthBands+1 so a fixed-size SSBO/UBO is safe.
  uint32_t depth_band_offsets[kMaxDepthBands + 1] = {0};
};

// 4-byte / 8-byte / 16-byte std430-clean POD. No vec3 anywhere.
//
// CPU-side mirror types — when uploaded as raw bytes, the layout matches
// the corresponding GLSL `layout(std430)` array stride exactly:
//   * int        => stride 4
//   * uint       => stride 4
//   * glm::vec2  => stride 8
//   * glm::vec4  => stride 16
//
// These are kept as separate parallel std::vector buffers (struct-of-arrays)
// rather than packed into a single struct, so each can be uploaded as its
// own SSBO and updated independently (e.g. growth_state mutates often,
// parent_index only on topology change).
struct TasselGrowthSoA {
  // ---- Topology channels (set on topology change) -----------------------
  std::vector<int32_t>  parent_index;        ///< -1 for root
  std::vector<uint32_t> depth;               ///< BFS depth band
  std::vector<uint32_t> type_tag;            ///< TasselTypeTag
  std::vector<glm::vec4> local_rotation;     ///< pre-baked local quat per node

  // ---- Growth-input channels (set per-frame from sampled descriptor) ----
  std::vector<glm::vec4> growth_state;       ///< (age_gdd, target_length,
                                              ///<  target_thickness, flags)
  std::vector<glm::vec4> node_random_misc;   ///< (node_random, order, growth_progress_in, type_specific_aux)

  // ---- Outputs (filled by grow.comp / propagate.comp) -------------------
  std::vector<glm::vec2> length_thickness;   ///< (length, half_thickness)
  std::vector<glm::vec4> global_position;    ///< (px,py,pz, length)
  std::vector<glm::vec4> global_rotation;    ///< (x,y,z,w) quat

  // ---- BFS dispatch helper ---------------------------------------------
  // Permutation of [0..node_count) sorted by depth so propagate.comp can
  // launch one wave per depth band over a contiguous range of indices.
  std::vector<uint32_t> depth_sorted_index;

  // ---- Header ----------------------------------------------------------
  TasselGrowthHeader header;

  // Resize all per-node arrays at once. Header.node_count is updated.
  void Resize(uint32_t node_count);

  // Total bytes of the per-node arrays + header. Useful for diagnostics
  // and Phase 1b SSBO sizing.
  [[nodiscard]] size_t TotalBytes() const;
};

inline void TasselGrowthSoA::Resize(uint32_t node_count) {
  parent_index.assign(node_count, -1);
  depth.assign(node_count, 0u);
  type_tag.assign(node_count, 0u);
  local_rotation.assign(node_count, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
  growth_state.assign(node_count, glm::vec4(0.0f));
  node_random_misc.assign(node_count, glm::vec4(0.5f, 0.0f, 0.0f, 0.0f));
  length_thickness.assign(node_count, glm::vec2(0.0f));
  global_position.assign(node_count, glm::vec4(0.0f));
  global_rotation.assign(node_count, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
  depth_sorted_index.assign(node_count, 0u);
  header.node_count = node_count;
}

inline size_t TasselGrowthSoA::TotalBytes() const {
  return sizeof(TasselGrowthHeader)
       + parent_index.size()       * sizeof(int32_t)
       + depth.size()              * sizeof(uint32_t)
       + type_tag.size()           * sizeof(uint32_t)
       + local_rotation.size()     * sizeof(glm::vec4)
       + growth_state.size()       * sizeof(glm::vec4)
       + node_random_misc.size()   * sizeof(glm::vec4)
       + length_thickness.size()   * sizeof(glm::vec2)
       + global_position.size()    * sizeof(glm::vec4)
       + global_rotation.size()    * sizeof(glm::vec4)
       + depth_sorted_index.size() * sizeof(uint32_t);
}

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
