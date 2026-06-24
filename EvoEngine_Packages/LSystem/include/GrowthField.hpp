#pragma once

// ---------------------------------------------------------------------------
// GrowthField
//
// Phase 3 of the biologically-emergent organ growth substrate. A growth field
// is the *driver* of intrinsic centerline curvature: it does not store a
// shape, it stores per-side elongation biases from which the equilibrium
// intrinsic curvature emerges.
//
// For a 1D rod (needle, internode), each material point has two opposing
// "sides" (adaxial = stem-facing / inner; abaxial = away / outer). A
// per-side elongation bias `g_side(s, t)` produces local strain differential
// across the cross-section diameter `d(s, t)`, which, integrated along arc
// length, becomes intrinsic curvature:
//
//     kappa_eq(s) = (g_abaxial(s) - g_adaxial(s)) / d(s)
//
// This module provides only the *equilibrium* curvature (steady-state shape
// the rod relaxes toward as it matures). The time evolution is handled by
// the caller (typically `ContinuousGrowthState::Multiplier(t - t_init)`),
// so the actual instantaneous intrinsic curvature is the equilibrium value
// scaled by the organ's age multiplier. This keeps biology and timing
// decoupled.
//
// Default-constructed values are *inert* (all biases and gradients zero) so
// any organ that does not opt in produces kappa ~= 0 - preserving Phase 1/Phase 2
// straight geometry exactly. Phase 5 (stress feedback) will modulate the
// stored biases at runtime; Phase 6 (internodes) reuses this same struct.
//
// 2D extension for maize/sorghum leaves (curl across blade *width* in
// addition to arc length) is deliberately deferred to a separate
// `BilateralGrowthField2D`; the 1D version here keeps the needle math
// compact and the per-needle CPU cost trivial.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>

#include "OrganCenterline.hpp"

namespace l_system_package {

/// 1D bilateral growth field for a single rod-like organ.
///
/// All units are dimensionless elongation rates (relative growth per unit
/// physiological time). Values are typically in [-0.2, 0.2]; larger values
/// produce sharper curvature.
struct BilateralGrowthField1D {
  /// Elongation bias on the adaxial side (s = base..tip, uniform).
  float adaxial_bias = 0.0f;
  /// Elongation bias on the abaxial side (s = base..tip, uniform).
  float abaxial_bias = 0.0f;
  /// Linear gradient added to the (abaxial - adaxial) differential along
  /// normalized arc length s_norm in [0, 1]. Positive values bias the tip
  /// to bend more than the base (apical-dominant curvature).
  float gradient_per_arclen = 0.0f;
  /// Effective cross-section diameter used to convert strain differential
  /// into curvature. For Pinus sylvestris: ~1.0 mm. Must be > 0 to be active.
  float diameter_m = 0.0f;

  /// True iff every term is exactly zero (or diameter is unset). When true,
  /// callers should skip integration and produce a straight centerline so
  /// behavior is bit-identical to Phase 2.
  [[nodiscard]] bool IsInert() const {
    return diameter_m <= 0.0f || (adaxial_bias == 0.0f && abaxial_bias == 0.0f && gradient_per_arclen == 0.0f);
  }

  /// Equilibrium intrinsic curvature (1/m) at normalized arc length
  /// `s_norm` in [0, 1]. Positive values bend the rod toward the abaxial
  /// side; negative values toward adaxial. Returns 0 when inert.
  [[nodiscard]] float EquilibriumCurvature(const float s_norm) const {
    if (IsInert())
      return 0.0f;
    const float s = std::clamp(s_norm, 0.0f, 1.0f);
    const float strain_diff = (abaxial_bias - adaxial_bias) + gradient_per_arclen * s;
    return strain_diff / diameter_m;
  }
};

/// Build a planar bent centerline by integrating the equilibrium curvature
/// of `field` along arc length `length`, scaled by `maturation_multiplier`
/// (the organ's age-driven growth multiplier in [0, 1]). The bend lies in
/// the local x-z plane: tangent starts along +Z, curvature rotates the
/// tangent toward +X (the adaxial->abaxial direction in the organ frame).
///
/// Optional sinusoidal waviness is injected as an additional intrinsic
/// curvature term, with amplitude in degrees, frequency in cycles over
/// the full needle length, and a phase offset in radians. The wave uses a
/// tip-emphasized envelope so distal segments undulate more than basal ones.
///
/// When the field is inert OR `maturation_multiplier <= 0`, this returns a
/// straight centerline equivalent to `OrganCenterline::Straight(length, segments)`.
/// This preserves the Phase 1/Phase 2 visual baseline whenever the descriptor
/// has not opted into curvature.
inline OrganCenterline BuildBentNeedleCenterline(const float length, const int segments,
                                                 const BilateralGrowthField1D& field,
                                                 const float maturation_multiplier = 1.0f,
                                                 const float sinusoidal_amplitude_deg = 0.0f,
                                                 const float sinusoidal_frequency_cycles = 0.0f,
                                                 const float sinusoidal_phase_rad = 0.0f) {
  const int n = std::max(2, segments + 1);
  const float clamped_length = std::max(0.0f, length);
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kTwoPi = 6.28318530717958647692f;
  const float clamped_maturation = std::max(0.0f, maturation_multiplier);
  const bool has_growth_field = !field.IsInert();
  const float clamped_wave_amplitude_deg = std::clamp(sinusoidal_amplitude_deg, 0.0f, 45.0f);
  const float clamped_wave_frequency = std::clamp(sinusoidal_frequency_cycles, 0.0f, 12.0f);
  const bool has_wave = clamped_wave_amplitude_deg > 0.0f && clamped_wave_frequency > 0.0f;
  if ((!has_growth_field && !has_wave) || clamped_maturation <= 0.0f || clamped_length <= 0.0f) {
    return OrganCenterline::Straight(clamped_length, segments);
  }
  const float wave_amplitude_rad = (clamped_wave_amplitude_deg * (kPi / 180.0f)) * std::min(clamped_maturation, 1.0f);

  // Sub-step the integration finely so that even short needles with high
  // curvature land control points on a smooth arc. We integrate over a dense
  // grid then resample to (n) control points for the Catmull-Rom spline.
  const int sub_steps = std::max(64, n * 8);
  const float ds = clamped_length / static_cast<float>(sub_steps);
  std::vector<glm::vec3> dense_positions;
  dense_positions.reserve(static_cast<size_t>(sub_steps + 1));
  dense_positions.emplace_back(0.0f, 0.0f, 0.0f);

  // Planar Frenet integration in the x-z plane:
  //   theta(0) = 0  (tangent along +Z)
  //   dtheta/ds = kappa(s_norm) * maturation_multiplier
  //   x(s) = integral sin(theta) ds, z(s) = integral cos(theta) ds
  // Mid-point rule for second-order accuracy without an extra control flag.
  float theta = 0.0f;
  glm::vec3 cursor(0.0f);
  for (int i = 0; i < sub_steps; ++i) {
    const float s_mid = (static_cast<float>(i) + 0.5f) * ds;
    const float s_norm = s_mid / clamped_length;
    float kappa = has_growth_field ? field.EquilibriumCurvature(s_norm) * clamped_maturation : 0.0f;
    if (has_wave) {
      // Tip-emphasized envelope: suppress base wobble and keep most wave
      // expression toward distal needle segments.
      const float envelope = std::clamp(s_norm * s_norm, 0.0f, 1.0f);
      const float phase = kTwoPi * clamped_wave_frequency * s_norm + sinusoidal_phase_rad;
      const float kappa_wave =
          (wave_amplitude_rad * kTwoPi * clamped_wave_frequency * envelope * std::cos(phase)) / clamped_length;
      kappa += kappa_wave;
    }
    const float theta_mid = theta + 0.5f * kappa * ds;
    cursor.x += std::sin(theta_mid) * ds;
    cursor.z += std::cos(theta_mid) * ds;
    theta += kappa * ds;
    dense_positions.push_back(cursor);
  }

  // Resample to (n) control points by uniform index spacing on the dense
  // grid. Control points end up roughly arc-length-uniform because the
  // dense grid itself is arc-length-uniform.
  OrganCenterline centerline;
  auto& cps = centerline.MutableControlPoints();
  cps.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const float u = static_cast<float>(i) / static_cast<float>(n - 1);
    const int idx = std::clamp(static_cast<int>(std::round(u * static_cast<float>(sub_steps))), 0, sub_steps);
    cps[static_cast<size_t>(i)] = dense_positions[static_cast<size_t>(idx)];
  }
  centerline.Invalidate();
  return centerline;
}

}  // namespace l_system_package
