#pragma once

// =============================================================================
//  RuleTable — GPU-resident description of one production rule.
//
//  Phase 3a contract (interface lock; no shader-side rule kernels yet):
//    A rule is fully described by:
//      * predecessor_symbol — which TasselSymbol triggers it (-1 = wildcard)
//      * priority           — lower wins among matching rules
//      * rule_kind          — switch tag dispatched inside derive.comp
//      * max_successors     — hard upper bound, used for atomic reservation
//      * rng_salt           — passed to hashNodeSeedShared() per draw
//      * param_block_offset — index into a per-plant ParamBlock SSBO where
//                             this rule's lambda-captured constants live
//      * param_block_size   — bytes; bounds-checked in derive.comp
//
//  RuleKind is closed-set per plant bundle. For the maize tassel the set
//  is the seven topology rules in MaizeTasselRules.hpp + a sentinel.
//  derive.comp's switch on rule_kind picks the right kernel; each kernel
//  reads its own slice of the ParamBlock and emits up to max_successors
//  Successor records.
//
//  Why not a fully data-driven rule VM?
//    The CPU rules call into SamplePlotted, EvaluateSecondaryStageRamp,
//    init_pair, etc. Encoding all of those as a portable byte-code is
//    Phase 3c+ work. For Phase 3 we instead specialise per-rule kernels
//    in GLSL (compiled into the same compute shader via #include + switch).
//    This keeps the contract simple and divergence per-warp manageable
//    when modules are pre-sorted by rule_kind.
//
//  ParamBlock layout convention (CPU side):
//    A single std::vector<uint8_t> per plant bundle. Each rule reserves a
//    fixed offset+size. Layout is fixed at bundle-load time, so derive.comp
//    can index via push-constant (rule_kind → offset table baked into the
//    pipeline-state push range).
// =============================================================================

#include <cstdint>
#include <string>
#include <vector>

#if defined(LSYSTEM_GPU_PIPELINE)

namespace l_system_plugin::gpu {

/// Sentinel value indicating "no specific predecessor — try all symbols".
inline constexpr int32_t kRuleWildcardSymbol = -1;

/// Hard cap on successors emitted by a single rule application. The CPU
/// tassel rules emit at most 4 (rule 1: internode + spikelet pair +
/// lateral + new apex). Bumping requires a wider successor staging buffer
/// in derive.comp.
inline constexpr uint32_t kMaxSuccessorsPerRule = 4;

/// Closed-set rule kinds for the maize tassel bundle. One-to-one with
/// MaizeTasselRules.hpp lines 365-845. Each entry's name documents the
/// CPU rule it mirrors so derive.comp's per-kind switch can be reviewed
/// against the CPU source.
enum class TasselRuleKind : uint32_t {
  // -- Topology rules --
  Order0AxisExtension          = 0,  ///< Rule 1: order-0 main axis
  Order0AxisExhaustion         = 1,  ///< Rule 2: order-0 cleanup
  SpikeApexExtension           = 2,  ///< Rule 3: spike-zone extension
  SpikeApexTerminal            = 3,  ///< Rule 4: spike terminal pair
  LateralApexExtension         = 4,  ///< Rule 5: lateral/secondary extension
  LateralApexTerminal          = 5,  ///< Rule 6: lateral terminal pair
  LateralBudExpansion          = 6,  ///< Rule 7: lateral bud → first internode
  // -- Sentinel --
  Count                        = 7,
};

/// Descriptor of a single rule, uploaded as one std430-aligned record.
struct RuleDescriptor {
  uint32_t rule_kind         = 0;   ///< TasselRuleKind value
  int32_t  predecessor_symbol = -1; ///< TasselSymbol or kRuleWildcardSymbol
  uint32_t priority          = 0;   ///< lower wins
  uint32_t max_successors    = 0;   ///< ≤ kMaxSuccessorsPerRule
  uint32_t rng_salt          = 0;   ///< PCG salt; matches CPU rules in
                                    ///<   MaizeTasselRules.hpp (e.g. 0xC8013EA4u)
  uint32_t param_block_offset = 0;  ///< bytes into ParamBlock SSBO
  uint32_t param_block_size  = 0;   ///< bytes
  uint32_t _pad              = 0;   ///< 32 B aligned
};
static_assert(sizeof(RuleDescriptor) == 32, "RuleDescriptor must be std430-aligned 32 B");

/// CPU-side rule table builder. Phase 3a populates only the kind +
/// predecessor + salt fields; ParamBlock packing is Phase 3b.
struct RuleTable {
  std::vector<RuleDescriptor> rules;
  std::vector<uint8_t>        param_block;  ///< filled by per-bundle packer

  [[nodiscard]] uint32_t Count() const { return static_cast<uint32_t>(rules.size()); }
  [[nodiscard]] size_t   ParamBytes() const { return param_block.size(); }
};

/// Build the canonical tassel rule table (descriptor records only —
/// ParamBlock left empty in Phase 3a). Salt values mirror the literals
/// passed to MakeNodeRng() inside MaizeTasselRules.hpp, line by line.
RuleTable BuildTasselRuleTable();

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
