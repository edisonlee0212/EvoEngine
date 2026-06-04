#pragma once

// ---------------------------------------------------------------------------
// GrowthFunction
//
// Phase 2 of the biologically-emergent organ growth substrate. Each organ
// follows a determinate growth trajectory: starts slowly, accelerates,
// asymptotes to its mature size. This header provides four sigmoidal /
// polynomial families, all normalized so that:
//
//   Value(t <= 0)  == 0
//   Value(t == 1)  ~= 1   (within tolerance set by `kAsymptoteTolerance`)
//   Value(t >= ~)  -> 1
//
// Time `t` is the *normalized* organ age in [0, +inf): 0 at initiation,
// 1 at "nominal maturity". The caller is responsible for the conversion
// `t = (clock.now_years - t_init_years) / maturation_years`.
//
// `Derivative(t)` returns d(Value)/dt in the same normalized time. Useful
// for elastica feedback in Phase 5 (high derivative => active elongation
// region; high stress here both slows growth and conditions stiffness).
//
// All evaluators are pure functions; the GrowthFunction struct itself is a
// small POD that selects the family and its parameters. Safe to pass by
// value into compute shaders later.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace l_system_package {

constexpr float kAsymptoteTolerance = 1e-4f;

enum class GrowthFunctionKind : std::uint8_t {
  /// Symmetric logistic: f(t) = 1 / (1 + exp(-k(t - 0.5))) renormalized to f(0)=0, f(1)~1.
  /// Default for tissue elongation under steady auxin/cytokinin balance.
  Logistic = 0,
  /// Richards (generalized logistic): f(t) = (1 + (nu) * exp(-k(t - tm)))^(-1/nu).
  /// `nu > 1` skews growth peak later (toward tip); `nu < 1` earlier (toward base).
  /// Used for asymmetric organs whose maximum growth rate occurs off-center.
  Richards = 1,
  /// Gompertz: f(t) = exp(-b * exp(-c * t)) renormalized.
  /// Heavily right-skewed; matches needle elongation in long-day flush conditions.
  Gompertz = 2,
  /// Cubic Hermite: smooth-step 3t^2 - 2t^3, derivative zero at endpoints.
  /// Cheapest; useful as a baseline / unit-test reference.
  Cubic = 3,
  /// Monotonic half-cosine easing in [0,1]: f(t)=0.5-0.5*cos(pi*t).
  /// Provides sinusoidal growth timing without shrink/regrow oscillations.
  Sinusoidal = 4,
};

struct GrowthFunction {
  GrowthFunctionKind kind = GrowthFunctionKind::Logistic;
  /// Logistic / Richards steepness (higher = sharper transition near tm).
  float k = 8.0f;
  /// Richards shape parameter nu > 0.
  float nu = 1.0f;
  /// Gompertz displacement b (controls onset delay).
  float b = 4.0f;
  /// Gompertz rate c.
  float c = 4.0f;
  /// Logistic / Richards midpoint in normalized time.
  float tm = 0.5f;

  /// Normalized growth output in [0, ~1]. `t <= 0` returns 0.
  float Value(float t) const {
    if (!std::isfinite(t))
      return 0.0f;
    if (t <= 0.0f)
      return 0.0f;
    switch (kind) {
      case GrowthFunctionKind::Logistic: {
        const float raw = 1.0f / (1.0f + std::exp(-k * (t - tm)));
        const float r0 = 1.0f / (1.0f + std::exp(-k * (0.0f - tm)));
        const float r1 = 1.0f / (1.0f + std::exp(-k * (1.0f - tm)));
        return std::clamp((raw - r0) / std::max(kAsymptoteTolerance, (r1 - r0)), 0.0f, 1.0f);
      }
      case GrowthFunctionKind::Richards: {
        const float nu_safe = std::max(0.05f, nu);
        auto eval = [&](float x) {
          return std::pow(1.0f + nu_safe * std::exp(-k * (x - tm)), -1.0f / nu_safe);
        };
        const float r0 = eval(0.0f);
        const float r1 = eval(1.0f);
        return std::clamp((eval(t) - r0) / std::max(kAsymptoteTolerance, (r1 - r0)), 0.0f, 1.0f);
      }
      case GrowthFunctionKind::Gompertz: {
        auto eval = [&](float x) {
          return std::exp(-b * std::exp(-c * x));
        };
        const float r0 = eval(0.0f);
        const float r1 = eval(1.0f);
        return std::clamp((eval(t) - r0) / std::max(kAsymptoteTolerance, (r1 - r0)), 0.0f, 1.0f);
      }
      case GrowthFunctionKind::Cubic: {
        if (t >= 1.0f)
          return 1.0f;
        return t * t * (3.0f - 2.0f * t);
      }
      case GrowthFunctionKind::Sinusoidal: {
        if (t >= 1.0f)
          return 1.0f;
        const float x = std::clamp(t, 0.0f, 1.0f);
        constexpr float kPi = 3.14159265358979323846f;
        return 0.5f - 0.5f * std::cos(kPi * x);
      }
    }
    return 0.0f;
  }

  /// Derivative d(Value)/dt at normalized time t. Approximate (analytic for
  /// Cubic and unnormalized logistic / Gompertz; central-difference for the
  /// renormalized variants - accuracy not critical for Phase 5 feedback).
  float Derivative(float t) const {
    if (!std::isfinite(t))
      return 0.0f;
    if (t <= 0.0f || t >= 1.0f + 1e-3f)
      return 0.0f;
    if (kind == GrowthFunctionKind::Cubic) {
      return 6.0f * t * (1.0f - t);
    }
    if (kind == GrowthFunctionKind::Sinusoidal) {
      const float x = std::clamp(t, 0.0f, 1.0f);
      constexpr float kPi = 3.14159265358979323846f;
      return 0.5f * kPi * std::sin(kPi * x);
    }
    constexpr float h = 1e-3f;
    return (Value(t + h) - Value(t - h)) * (1.0f / (2.0f * h));
  }
};

/// Per-organ continuous-growth state. Stored on every module that has a
/// determinate trajectory (PineNeedleCluster, PineInternode after Phase 6,
/// future maize/sorghum leaf). Owns the data needed to evaluate
/// `GrowthFunction.Value((clock.now() - t_init_years) / maturation_years)`.
struct ContinuousGrowthState {
  float t_init_years = 0.0f;      ///< Physiological time at module initiation.
  float maturation_years = 1.0f;  ///< Time to reach "nominal mature size" (Value(1)).
  GrowthFunction function{};      ///< Family + parameters.

  /// Normalized age in [0, +inf). Negative ages clamp to 0.
  float NormalizedAge(float clock_now_years) const {
    if (maturation_years <= 0.0f)
      return 1.0f;
    const float dt = clock_now_years - t_init_years;
    return std::max(0.0f, dt) / maturation_years;
  }

  /// Convenience: the unnormalized growth multiplier for the organ's target size.
  float Multiplier(float clock_now_years) const {
    return function.Value(NormalizedAge(clock_now_years));
  }
};

}  // namespace l_system_package