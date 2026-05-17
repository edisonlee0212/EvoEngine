#pragma once

// Generic L-system rule helpers and tropism types shared by all species
// (maize tassel, Scots pine, etc.). These were originally embedded in
// MaizeTasselRules.hpp; they were extracted unchanged in Phase 2.0 of the
// L-system consolidation work so additional species can reuse them without
// duplicating definitions. See docs/seam_inventory.md for the broader plan.
//
// Bit-exact-parity contract: function bodies, RNG distribution construction,
// and integer constants below MUST remain byte-identical to the originals to
// preserve MaizeTassel output hashes (validated by tools/lsystem_parity_check.py).

#include <Plot2D.hpp>
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

namespace l_system_plugin {

// ---------------------------------------------------------------------------
// Helper: deterministic sampling from a SingleDistribution using a seeded RNG.
// ---------------------------------------------------------------------------

template <typename T>
T SampleDistribution(const evo_engine::SingleDistribution<T>& dist, std::mt19937& rng) {
  if (dist.deviation <= 0.0f)
    return dist.mean;
  std::normal_distribution<float> normal(0.0f, 1.0f);
  return dist.mean + T(dist.deviation * normal(rng));
}

// ---------------------------------------------------------------------------
// Helper: deterministic sampling from a PlottedDistribution using a seeded RNG.
// ---------------------------------------------------------------------------

inline float SamplePlotted(const evo_engine::PlottedDistribution<float>& pd, float t, std::mt19937& rng) {
  const float mean_val = pd.mean.GetValue(t);
  const float dev_val = pd.deviation.GetValue(t);
  if (dev_val <= 0.0f)
    return mean_val;
  std::normal_distribution<float> dist(mean_val, dev_val);
  return dist(rng);
}

inline uint32_t HashNodeSeed(const float node_random, const uint32_t salt) {
  const float clamped = std::clamp(node_random, 0.0f, 1.0f);
  uint32_t x = static_cast<uint32_t>(clamped * 4294967295.0f) ^ (salt + 0x9e3779b9u);
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

inline std::mt19937 MakeNodeRng(const float node_random, const uint32_t salt) {
  return std::mt19937(HashNodeSeed(node_random, salt));
}

inline float EvaluatePlottedDeterministic(const evo_engine::PlottedDistribution<float>& pd,
                                          const float t,
                                          const float node_random,
                                          const uint32_t salt,
                                          const float lo = -std::numeric_limits<float>::infinity(),
                                          const float hi = std::numeric_limits<float>::infinity()) {
  const float mean_val = pd.mean.GetValue(t);
  const float sigma_val = std::max(0.0f, pd.deviation.GetValue(t));
  if (!(sigma_val > 0.0f)) {
    return std::clamp(mean_val, lo, hi);
  }

  auto rng = MakeNodeRng(node_random, salt);
  std::normal_distribution<float> unit_normal(0.0f, 1.0f);
  const float z = unit_normal(rng);
  return std::clamp(mean_val + sigma_val * z, lo, hi);
}

inline float SampleUnit01(std::mt19937& rng) {
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);
  return dist(rng);
}

// ---------------------------------------------------------------------------
// Per-node stochastic samplers.
//
// These complement the per-plant `SampleDistribution` / `SamplePlotted` helpers
// above. They are intended for use *inside* production-rule producers, where
// each emitted module draws fresh noise from a per-node RNG (see
// `MakeNodeRng`). Driving every draw from a per-node RNG keeps the L-system
// deterministic for a fixed plant seed: tree(seed=N) is reproducible bit-for-
// bit regardless of derivation order.
//
// All samplers are no-ops when their dispersion parameter is <= 0, so a
// descriptor with default (zero) per-node noise produces the same trajectory
// as the legacy deterministic baseline.
// ---------------------------------------------------------------------------

/// Gaussian sample clamped to [lo, hi]. Returns `mean` unchanged when
/// `sigma <= 0`. Up to 8 redraws to land inside [lo, hi]; if all fail, the
/// returned value is hard-clamped (avoids infinite loops on extreme bounds).
inline float SampleGaussianClamped(const float mean, const float sigma,
                                   const float lo, const float hi,
                                   std::mt19937& rng) {
  if (!(sigma > 0.0f)) return std::clamp(mean, lo, hi);
  std::normal_distribution<float> dist(mean, sigma);
  for (int attempt = 0; attempt < 8; ++attempt) {
    const float v = dist(rng);
    if (v >= lo && v <= hi) return v;
  }
  return std::clamp(dist(rng), lo, hi);
}

/// Poisson sample clipped to >= 0. Returns `static_cast<int>(std::round(lambda))`
/// when `lambda <= 0` (degenerate distribution). std::poisson_distribution
/// requires positive `mean`, so we guard with `lambda > 0`.
inline int SamplePoissonNonneg(const float lambda, std::mt19937& rng) {
  if (!(lambda > 0.0f)) return std::max(0, static_cast<int>(std::round(lambda)));
  std::poisson_distribution<int> dist(static_cast<double>(lambda));
  return std::max(0, dist(rng));
}

/// Discrete count sampler used for "branches per whorl"-style fields.
/// If `poisson_lambda > 0`, sample Poisson(poisson_lambda); else return
/// `default_count`. Result clamped to [min_count, max_count].
inline int SampleCountPoissonOrFixed(const int default_count,
                                     const float poisson_lambda,
                                     const int min_count,
                                     const int max_count,
                                     std::mt19937& rng) {
  const int raw = (poisson_lambda > 0.0f) ? SamplePoissonNonneg(poisson_lambda, rng)
                                          : default_count;
  return std::clamp(raw, min_count, max_count);
}

// ---------------------------------------------------------------------------
// JitteredScalar — POD bundle for a per-node noise-augmented scalar.
//
// Used by per-node sampling sites that don't need a full PlottedDistribution.
// `mean`         : central value (already drawn per-plant from a SingleDistribution).
// `sigma`        : per-node Gaussian noise (additive when `relative` is false,
//                  fractional/CV when `relative` is true).
// `relative`     : if true, sigma is interpreted as coefficient of variation
//                  (final value = mean * (1 + sigma*N(0,1))).
// `lo` / `hi`    : clamp bounds. Default range is the full float line.
//
// Construction is cheap and the type is trivially copyable; callers can stash
// it inside `SampledPineParams` without inflating the per-plant footprint.
// ---------------------------------------------------------------------------

struct JitteredScalar {
  float mean = 0.0f;
  float sigma = 0.0f;
  float lo = -std::numeric_limits<float>::infinity();
  float hi =  std::numeric_limits<float>::infinity();
  bool relative = false;

  /// Sample once from a per-node RNG; returns `mean` unchanged when sigma==0.
  float Sample(std::mt19937& rng) const {
    if (!(sigma > 0.0f)) return std::clamp(mean, lo, hi);
    if (relative) {
      std::normal_distribution<float> dist(0.0f, sigma);
      const float v = mean * (1.0f + dist(rng));
      return std::clamp(v, lo, hi);
    }
    return SampleGaussianClamped(mean, sigma, lo, hi, rng);
  }
};

// ---------------------------------------------------------------------------
// TropismEntry — user-facing tropism descriptor (one per dynamic list entry).
// Generic across species: a tropism is a directional bias modulated by
// branching order, with a per-plant activation roll.
// ---------------------------------------------------------------------------

struct TropismEntry {
  evo_engine::SingleDistribution<float> direction_x{0.0f};
  evo_engine::SingleDistribution<float> direction_y{-1.0f};
  evo_engine::SingleDistribution<float> direction_z{0.0f};
  evo_engine::SingleDistribution<float> strength{0.0f};
  float usage_chance_percent = 100.0f;  ///< Per-plant activation chance in [0, 100].

  /// Curve: x = normalized branching order (0=rachis..1=max order), y = response multiplier.
  evo_engine::PlottedDistribution<float> order_response;
};

// ---------------------------------------------------------------------------
// SampledTropism — concrete sampled tropism values for one instance.
// ---------------------------------------------------------------------------

struct SampledTropism {
  glm::vec3 direction{0.0f, -1.0f, 0.0f};
  float strength = 0.0f;
  evo_engine::PlottedDistribution<float> order_response;
};

}  // namespace l_system_plugin
