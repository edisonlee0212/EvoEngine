#pragma once

// =============================================================================
//  CurveBaker — flatten any Curve2D-like sampler into a std430-friendly atlas.
//
//  Phase 2a contract:
//    * Bakes N curves into a single flat float array of size
//      N * kCurveBakeSamples * 2 (channel R = value, channel G = derivative
//      reserved for Phase 7 slope-aware editor scrubbing).
//    * Sample density is fixed at 256 taps per curve. ~2 KB per curve;
//      12 tassel curves ≈ 24 KB total — trivial to re-upload per frame on
//      Curve dirty bit.
//    * Sampler is provided as std::function<float(float t)> so this header
//      has zero dependency on MaizeTasselDescriptor or Curve2D's actual API
//      (which lives in EvoEngine_SDK and is not visible from the LSystem
//      plugin's public include set without extra coupling).
//    * Output buffer matches the GLSL declaration:
//        layout(std430) readonly buffer CurveAtlas {
//          uint  num_curves;
//          uint  samples_per_curve;
//          uint  stride;          // == samples_per_curve * 2
//          uint  _pad;
//          float taps[];          // num_curves * stride values
//        };
//
//  Bit-exact equivalence with CPU `Curve2D::GetValue(t)` is NOT guaranteed —
//  the linear-interpolation reconstruction will differ from a cubic CPU
//  curve at sub-tap scales. Acceptance: KS ≤ 0.05 per the determinism gate
//  (see docs/gpu_pipeline.md §6). Sample count can be raised to 1024 if
//  the gate fails on slope-sensitive curves.
// =============================================================================

#include "TasselGrowthSoA.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#if defined(LSYSTEM_GPU_PIPELINE)

namespace l_system_plugin::gpu {

/// One source curve to bake. `name` is for debug/inspector.
struct CurveSource {
  std::string name;
  std::function<float(float t)> sample;  ///< t in [0,1]
};

/// Output atlas mirroring the GLSL CurveAtlas SSBO layout above.
struct CurveAtlas {
  uint32_t num_curves        = 0;
  uint32_t samples_per_curve = 0;
  uint32_t stride            = 0;
  uint32_t _pad              = 0;
  std::vector<float> taps;   ///< size = num_curves * stride

  // Convenience: linear-interp sample mirroring what grow.comp does.
  // Used by the equivalence harness and CPU-shadow validator.
  [[nodiscard]] float SampleLinear(uint32_t curve_id, float t) const;
};

/// Bake N curves into a single atlas. `samples_per_curve` defaults to
/// kCurveBakeSamples but can be overridden for stress-tests.
CurveAtlas BakeCurves(const std::vector<CurveSource>& sources,
                      uint32_t samples_per_curve = kCurveBakeSamples);

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
