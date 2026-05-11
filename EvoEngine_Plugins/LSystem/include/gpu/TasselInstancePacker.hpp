#pragma once

// =============================================================================
//  TasselInstancePacker — walks the CPU TasselGraph into a GPU-ready SoA.
//
//  Phase 1a. This is the CPU-side counterpart of tassel_internode.mesh.
//
//  Contract: for a given ``TasselGraph`` + color mode + instance color, the
//  packer produces the same visible output as the existing ``RebuildGeometry``
//  ParticleInfo path in MaizeTassel.cpp (same cylinder transform, same
//  color). The Phase 1 acceptance gate is visual-equivalent output.
//
//  Behavioral notes, lifted from MaizeTassel.cpp to avoid drift:
//    * Skip nodes that are not ``TasselInternode``.
//    * Skip internodes with non-finite position / length / thickness (count
//      them in ``invalid_instance_count`` like the CPU path does).
//    * Skip zero-length or zero-thickness internodes.
//    * Apply the ``cylinder_axis_fix`` rotation so the unit cylinder's +Y
//      axis maps to the node's growth direction (+Z).
//    * Emit the pair-internode instances (the small connector between the
//      proximal and distal spikelet ellipsoids) after all the regular
//      internodes, matching the current ordering.
//
//  Spikelet ellipsoid instances are intentionally NOT packed here. They go
//  through a separate SoA + shader pair added later in Phase 1b / 1c.
// =============================================================================

#include "TasselInstanceSoA.hpp"

#include <cstdint>
#include <vector>

#if defined(LSYSTEM_GPU_PIPELINE)

#include "MaizeTasselModules.hpp"  // defines TasselGraph (template alias)

namespace l_system_plugin::gpu {

/// Color-resolution mode. Mirrors ``MaizeTassel::ColorMode`` but kept
/// independent so the gpu/ subtree has no dependency on ``MaizeTassel.hpp``.
enum class ColorMode : uint32_t {
  Shaded = 0,      ///< fixed stem color
  ByType = 1,      ///< fixed per-type color
  ByInstance = 2,  ///< one color per tassel (passed in as instance_color)
  ByNode = 3,      ///< hashed from node index
};

struct PackInternodesOptions {
  ColorMode color_mode = ColorMode::Shaded;
  glm::vec4 instance_color = glm::vec4(1.0f);  ///< used when ``color_mode == ByInstance``
  bool include_pair_internodes = true;
};

struct PackInternodesResult {
  uint32_t total_nodes_scanned = 0;
  uint32_t internode_count = 0;
  uint32_t pair_internode_count = 0;
  uint32_t invalid_instance_count = 0;
};

/// Append the CPU graph's internode instances (and optionally pair
/// internodes) into ``out``. Callers typically ``clear()`` ``out`` first;
/// the packer appends to let callers batch multiple plants into one buffer.
PackInternodesResult PackTasselInternodes(
    const l_system_plugin::TasselGraph& graph,
    const PackInternodesOptions& options,
    std::vector<TasselInternodeInstance>& out);

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
