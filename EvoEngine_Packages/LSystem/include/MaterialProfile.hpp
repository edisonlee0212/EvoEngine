#pragma once

// ---------------------------------------------------------------------------
// MaterialProfile1D
//
// Phase 4 of the biologically-emergent organ growth substrate. Pure-POD
// description of a rod's mechanical properties along its arc length and over
// physiological time. Provides:
//
//   * Young's modulus E(t) following a maturation/lignification sigmoid.
//   * Cross-section second moment of area I(s_norm) for a circular profile
//     parameterized by base/tip radius (linear taper). Other profiles can be
//     wired in by callers via `bending_stiffness_override_Pa_m4` if needed.
//   * Tissue density (kg/m^3) used to derive distributed weight per arc length.
//
// Default-constructed values are *inert*: `young_modulus_baseline_Pa = 0`
// short-circuits `IsActive()` to false, in which case the elastica solver is
// skipped entirely and the centerline retains its intrinsic-curvature shape.
// This preserves Phase 1/Phase 2/Phase 3 behavior bit-exactly when the
// descriptor has not opted into mechanics.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>

#include "GrowthFunction.hpp"
#include "SimulationClock.hpp"

namespace l_system_package {

/// Mechanical properties of a single rod-like organ (needle, internode).
/// Quantities are SI: Pa (= N/m^2) for moduli, m for radii, kg/m^3 for
/// density. The solver treats the rod as Euler-Bernoulli (small-strain,
/// large-rotation), which is appropriate for the slender geometries here
/// (length/diameter > ~30 for both Scots pine needles and most internodes).
struct MaterialProfile1D {
  /// Asymptotic Young's modulus (Pa) at maturity. Typical values:
  ///   - young needle parenchyma: 1e7 .. 1e8
  ///   - mature needle (lignified midrib): 5e8 .. 5e9
  ///   - mature softwood internode: 5e9 .. 1.5e10
  /// Setting this to 0 marks the profile inert (solver is skipped).
  float young_modulus_baseline_Pa = 0.0f;

  /// Per-organ lignification trajectory. `Multiplier(t)` ramps from a small
  /// floor (default 0) up to 1.0 over `maturation_years`. Defaults to inert
  /// (maturation_years = 0 -> Multiplier returns 1.0 after t_init).
  ContinuousGrowthState lignification{};

  /// Cross-section radius at the base (s_norm = 0). Combined with `tip_radius_m`
  /// for a linear taper, gives `r(s_norm) = mix(base_radius_m, tip_radius_m, s_norm)`.
  float base_radius_m = 0.0f;
  float tip_radius_m = 0.0f;

  /// Tissue density (kg/m^3). Used to derive distributed weight per unit arc
  /// length. Typical values:
  ///   - fresh needle tissue: ~700 .. 900 (mostly water)
  ///   - dry softwood: ~400 .. 550
  /// 0 -> no distributed weight (solver still runs if a tip force is supplied).
  float density_kg_m3 = 0.0f;

  /// Optional override on EI (Pa*m^4) - when > 0, supersedes the
  /// E(t) * I(s_norm) computation. Useful for analytic cantilever tests.
  float bending_stiffness_override_Pa_m4 = 0.0f;

  /// True iff the profile has any mechanical effect. When false, the solver
  /// is skipped and the centerline retains its intrinsic-curvature shape.
  [[nodiscard]] bool IsActive() const {
    return young_modulus_baseline_Pa > 0.0f || bending_stiffness_override_Pa_m4 > 0.0f;
  }

  /// Time-varying Young's modulus. Returns the baseline scaled by the
  /// lignification multiplier evaluated at the supplied physiological time.
  [[nodiscard]] float YoungModulus_Pa(const float t_now_years) const {
    if (lignification.maturation_years > 0.0f) {
      return young_modulus_baseline_Pa * lignification.Multiplier(t_now_years);
    }
    return young_modulus_baseline_Pa;
  }

  /// Linear taper radius along normalized arc length s_norm in [0, 1].
  [[nodiscard]] float Radius_m(const float s_norm) const {
    const float s = std::clamp(s_norm, 0.0f, 1.0f);
    return base_radius_m * (1.0f - s) + tip_radius_m * s;
  }

  /// Second moment of area for a solid circular cross-section: I = pi * r^4 / 4.
  [[nodiscard]] float SecondMomentOfArea_m4(const float s_norm) const {
    const float r = Radius_m(s_norm);
    return 0.25f * 3.14159265358979323846f * r * r * r * r;
  }

  /// Bending stiffness EI(s_norm, t). Honors the override when set.
  [[nodiscard]] float BendingStiffness_Pa_m4(const float s_norm, const float t_now_years) const {
    if (bending_stiffness_override_Pa_m4 > 0.0f) {
      return bending_stiffness_override_Pa_m4;
    }
    return YoungModulus_Pa(t_now_years) * SecondMomentOfArea_m4(s_norm);
  }

  /// Mass per unit arc length (kg/m) for distributed weight. Returns 0 when
  /// density or radius is unset.
  [[nodiscard]] float MassPerLength_kg_m(const float s_norm) const {
    const float r = Radius_m(s_norm);
    const float area = 3.14159265358979323846f * r * r;
    return area * density_kg_m3;
  }
};

}  // namespace l_system_package