#pragma once

// =============================================================================
//  TasselGrowthPacker — flatten a TasselGraph into TasselGrowthSoA for GPU.
//
//  Phase 2a contract:
//    * Walks graph.PeekSortedNodeList() exactly once.
//    * Computes BFS depth per node from parent depth + 1 (root = 0). The
//      sorted-node list is already topology-ordered so a single forward
//      pass suffices.
//    * Builds depth_band_offsets + depth_sorted_index permutation so
//      propagate.comp can dispatch contiguous waves per depth band.
//    * Pre-bakes the local rotation per node by calling the host-supplied
//      ``local_rotation_fn`` (the same callable MaizeTassel.cpp already
//      passes to GeometryPass::Execute). Keeps grow.comp / propagate.comp
//      plant-agnostic.
//    * Fills growth_state from per-type fields:
//        Internode    -> (age_gdd, target_length, target_thickness, flags)
//        SpikeletPair -> (age_gdd, pair_internode_target_length,
//                         pair_internode_target_thickness, MainRachis flag)
//        Apex/Lateral/SpikeApex -> (age_gdd, 0, 0, 0)
//    * Initialises length_thickness from the current CPU info so callers
//      can validate the GPU-shadow grow output against the CPU baseline
//      without first running grow().
//
//  Output is independent of any Vulkan state — the engine's
//  UploadTasselGrowthSoA (Phase 2a stub) currently only retains a CPU
//  shadow copy.
// =============================================================================

#include "TasselGrowthSoA.hpp"

#include <functional>

#if defined(LSYSTEM_GPU_PIPELINE)

#include "MaizeTasselModules.hpp"

namespace l_system_plugin::gpu {

/// Callback signature matching MaizeTassel.cpp's existing
/// ``local_rotation_fn`` plumbed into GeometryPass::Execute. The packer
/// invokes it once per node so the SoA's local_rotation channel ships a
/// fully-resolved per-node quaternion without GPU-side type dispatch in
/// Phase 2a.
using LocalRotationFn = std::function<glm::quat(
    const LGraphNode<TasselModuleData>& node,
    const LGraphNode<TasselModuleData>& parent)>;

struct PackGrowthOptions {
  /// Per-node local rotation producer. Required.
  LocalRotationFn local_rotation_fn;
  /// Instance id stamped into the resulting header (for multi-instance
  /// debugging in Phase 4).
  uint32_t instance_id = 0;
  /// If true, abort early when an internode's CPU info contains non-finite
  /// values (mirrors MaizeTassel::last_invalid_instance_count). Defaults
  /// to true so a corrupted CPU graph never poisons the GPU buffer.
  bool reject_non_finite = true;
};

struct PackGrowthResult {
  uint32_t node_count        = 0;
  uint32_t depth_band_count  = 0;
  uint32_t skipped_invalid   = 0;
  bool ok                    = false;
};

/// Pack ``graph`` into ``out``. Resizes ``out`` to graph.PeekSortedNodeList().size().
PackGrowthResult PackTasselGrowth(const TasselGraph& graph,
                                  const PackGrowthOptions& options,
                                  TasselGrowthSoA& out);

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
