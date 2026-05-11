#include "gpu/TasselGrowthPacker.hpp"

#if defined(LSYSTEM_GPU_PIPELINE)

#include <algorithm>
#include <cmath>
#include <cstring>

namespace l_system_plugin::gpu {

namespace {

bool IsFiniteVec3(const glm::vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool IsFiniteQuat(const glm::quat& q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}
// C++17-safe bit-cast (uint -> float). Used to smuggle the packed flags
// word through the float-typed growth_state.w channel without changing the
// SoA layout. grow.comp / propagate.comp recover it via floatBitsToUint.
float UintBitsToFloat(uint32_t u) {
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

}  // namespace

PackGrowthResult PackTasselGrowth(const TasselGraph& graph,
                                  const PackGrowthOptions& options,
                                  TasselGrowthSoA& out) {
  PackGrowthResult result;

  const auto& sorted = graph.PeekSortedNodeList();
  const uint32_t node_count = static_cast<uint32_t>(sorted.size());
  out.Resize(node_count);
  out.header.instance_id = options.instance_id;

  if (!options.local_rotation_fn) {
    return result;  // ok=false; caller misconfigured
  }
  if (node_count == 0) {
    out.header.depth_band_count = 0;
    out.header.depth_band_offsets[0] = 0;
    result.ok = true;
    return result;
  }

  // ---- First pass: depth + per-node packing ------------------------------
  //
  // sorted[] is already topology-ordered (parent precedes child) per
  // GeometryPass::Execute's contract, so depth[parent] is always set
  // before depth[child].
  uint32_t max_depth = 0;
  for (uint32_t i = 0; i < node_count; ++i) {
    const auto handle = sorted[i];
    const auto& node  = graph.PeekNode(handle);
    const int node_idx = node.GetIndex();
    if (node_idx < 0 || static_cast<uint32_t>(node_idx) >= node_count) {
      // Defensive: graph indices should be 0..node_count-1 because
      // PeekSortedNodeList enumerates exactly node_count nodes; a mismatch
      // means the graph was mutated mid-pack.
      return result;
    }
    const uint32_t my_idx = static_cast<uint32_t>(node_idx);

    // Parent index + depth.
    const auto parent_handle = node.GetParentHandle();
    int32_t parent_idx = -1;
    uint32_t my_depth = 0;
    if (parent_handle != -1) {
      const auto& parent = graph.PeekNode(parent_handle);
      parent_idx = parent.GetIndex();
      if (parent_idx >= 0 && static_cast<uint32_t>(parent_idx) < node_count) {
        my_depth = out.depth[parent_idx] + 1u;
      }
    }
    out.parent_index[my_idx] = parent_idx;
    out.depth[my_idx]        = my_depth;
    max_depth = std::max(max_depth, my_depth);

    // Type tag.
    uint32_t tag = 0u;
    if (node.data.template Is<TasselApex>())            tag = static_cast<uint32_t>(TasselTypeTag::Apex);
    else if (node.data.template Is<TasselInternode>())  tag = static_cast<uint32_t>(TasselTypeTag::Internode);
    else if (node.data.template Is<TasselLateral>())    tag = static_cast<uint32_t>(TasselTypeTag::Lateral);
    else if (node.data.template Is<TasselSpikeletPair>())tag= static_cast<uint32_t>(TasselTypeTag::SpikeletPair);
    else if (node.data.template Is<TasselSpikeApex>())  tag = static_cast<uint32_t>(TasselTypeTag::SpikeApex);
    out.type_tag[my_idx] = tag;

    // Local rotation (host-supplied).
    glm::quat local = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (parent_handle != -1) {
      const auto& parent = graph.PeekNode(parent_handle);
      local = options.local_rotation_fn(node, parent);
    }
    if (!IsFiniteQuat(local)) {
      ++result.skipped_invalid;
      local = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    local = glm::normalize(local);
    out.local_rotation[my_idx] = glm::vec4(local.x, local.y, local.z, local.w);

    // Growth state per type.
    glm::vec4 gs(0.0f);
    glm::vec4 misc(0.5f, 0.0f, 0.0f, 0.0f);
    if (node.data.template Is<TasselInternode>()) {
      const auto& in = node.data.template Get<TasselInternode>();
      uint32_t flags = 0u;
      if (in.is_spike) flags |= kFlagIsSpike;
      gs = glm::vec4(in.age_gdd,
                     in.target_length,
                     in.target_thickness,
                     UintBitsToFloat(flags));
      misc = glm::vec4(in.node_random,
                       static_cast<float>(in.order),
                       in.growth_progress,
                       0.0f);
    } else if (node.data.template Is<TasselSpikeletPair>()) {
      const auto& sp = node.data.template Get<TasselSpikeletPair>();
      uint32_t flags = 0u;
      if (sp.main_rachis_pair) flags |= kFlagMainRachis;
      gs = glm::vec4(sp.age_gdd,
                     sp.pair_internode_target_length,
                     sp.pair_internode_target_thickness,
                     UintBitsToFloat(flags));
      misc = glm::vec4(sp.node_random,
                       static_cast<float>(sp.pair_ordinal),
                       0.0f,
                       sp.phyllotaxis_azimuth);
    } else if (node.data.template Is<TasselApex>()) {
      const auto& ap = node.data.template Get<TasselApex>();
      gs   = glm::vec4(ap.age_gdd, 0.0f, 0.0f, 0.0f);
      misc = glm::vec4(ap.node_random, static_cast<float>(ap.order), 0.0f, ap.phyllotaxis_phase);
    } else if (node.data.template Is<TasselLateral>()) {
      const auto& la = node.data.template Get<TasselLateral>();
      gs   = glm::vec4(la.age_gdd, la.target_length, la.target_thickness, 0.0f);
      misc = glm::vec4(la.node_random, static_cast<float>(la.order), 0.0f, la.azimuth_offset);
    } else if (node.data.template Is<TasselSpikeApex>()) {
      const auto& sa = node.data.template Get<TasselSpikeApex>();
      gs   = glm::vec4(sa.age_gdd, 0.0f, 0.0f, 0.0f);
      misc = glm::vec4(sa.node_random, 0.0f, 0.0f, sa.phyllotaxis_phase);
    }
    out.growth_state[my_idx]     = gs;
    out.node_random_misc[my_idx] = misc;

    // Seed outputs from current CPU info so the CPU-shadow validator can
    // diff vs the post-grow result.
    out.length_thickness[my_idx] = glm::vec2(node.info.length, node.info.thickness * 0.5f);
    if (options.reject_non_finite) {
      if (!IsFiniteVec3(node.info.global_position) ||
          !IsFiniteQuat(node.info.global_rotation)) {
        ++result.skipped_invalid;
      }
    }
    out.global_position[my_idx] = glm::vec4(node.info.global_position, node.info.length);
    out.global_rotation[my_idx] = glm::vec4(node.info.global_rotation.x,
                                            node.info.global_rotation.y,
                                            node.info.global_rotation.z,
                                            node.info.global_rotation.w);
  }

  // ---- Second pass: depth-band offsets + sorted index permutation -------
  const uint32_t band_count = std::min(max_depth + 1u, kMaxDepthBands);
  out.header.depth_band_count = band_count;

  // Histogram per depth band.
  std::vector<uint32_t> band_count_buf(band_count, 0u);
  for (uint32_t i = 0; i < node_count; ++i) {
    const uint32_t d = std::min(out.depth[i], band_count - 1u);
    band_count_buf[d]++;
  }
  // Exclusive prefix sum into header.depth_band_offsets.
  uint32_t accum = 0;
  for (uint32_t b = 0; b < band_count; ++b) {
    out.header.depth_band_offsets[b] = accum;
    accum += band_count_buf[b];
  }
  out.header.depth_band_offsets[band_count] = accum;

  // Fill depth_sorted_index using a running cursor per band.
  std::vector<uint32_t> band_cursor(band_count, 0u);
  for (uint32_t i = 0; i < node_count; ++i) {
    const uint32_t d = std::min(out.depth[i], band_count - 1u);
    const uint32_t slot = out.header.depth_band_offsets[d] + band_cursor[d]++;
    out.depth_sorted_index[slot] = i;
  }

  result.node_count       = node_count;
  result.depth_band_count = band_count;
  result.ok               = true;
  return result;
}

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
