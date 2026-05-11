#include "gpu/CurveBaker.hpp"

#if defined(LSYSTEM_GPU_PIPELINE)

#include <algorithm>
#include <cmath>

namespace l_system_plugin::gpu {

float CurveAtlas::SampleLinear(uint32_t curve_id, float t) const {
  if (num_curves == 0 || samples_per_curve < 2) return 0.0f;
  if (curve_id >= num_curves) return 0.0f;
  const float clamped = std::min(std::max(t, 0.0f), 1.0f);
  const float scaled  = clamped * static_cast<float>(samples_per_curve - 1);
  const uint32_t i0   = static_cast<uint32_t>(std::floor(scaled));
  const uint32_t i1   = std::min(i0 + 1u, samples_per_curve - 1u);
  const float frac    = scaled - static_cast<float>(i0);
  // Channel R only (channel G = derivative slot, reserved for Phase 7).
  const uint32_t base = curve_id * stride;
  const float v0 = taps[base + i0 * 2u + 0u];
  const float v1 = taps[base + i1 * 2u + 0u];
  return v0 + (v1 - v0) * frac;
}

CurveAtlas BakeCurves(const std::vector<CurveSource>& sources,
                      uint32_t samples_per_curve) {
  CurveAtlas atlas;
  atlas.num_curves        = static_cast<uint32_t>(sources.size());
  atlas.samples_per_curve = samples_per_curve;
  atlas.stride            = samples_per_curve * 2u;
  atlas._pad              = 0u;
  atlas.taps.assign(static_cast<size_t>(atlas.num_curves) * atlas.stride, 0.0f);

  if (samples_per_curve < 2) return atlas;
  const float dt = 1.0f / static_cast<float>(samples_per_curve - 1);

  for (uint32_t c = 0; c < atlas.num_curves; ++c) {
    const auto& src = sources[c];
    if (!src.sample) continue;
    const uint32_t base = c * atlas.stride;
    // First pass: values.
    for (uint32_t s = 0; s < samples_per_curve; ++s) {
      const float t = static_cast<float>(s) * dt;
      atlas.taps[base + s * 2u + 0u] = src.sample(t);
    }
    // Second pass: central-difference derivatives (reserved channel G).
    for (uint32_t s = 0; s < samples_per_curve; ++s) {
      const uint32_t sp = std::min(s + 1u, samples_per_curve - 1u);
      const uint32_t sm = (s == 0u) ? 0u : (s - 1u);
      const float vp = atlas.taps[base + sp * 2u + 0u];
      const float vm = atlas.taps[base + sm * 2u + 0u];
      const float ds = static_cast<float>(sp - sm) * dt;
      atlas.taps[base + s * 2u + 1u] = (ds > 0.0f) ? (vp - vm) / ds : 0.0f;
    }
  }
  return atlas;
}

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
