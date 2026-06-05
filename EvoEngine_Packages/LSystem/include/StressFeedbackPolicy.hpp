#pragma once

// ---------------------------------------------------------------------------
// StressFeedbackPolicy
//
// Closes the cell-mechanics loop the brief calls for: bending stress extracted from the
// elastica solver feeds back into both the growth field
// (`g(s, side, t) <- g_baseline * feedback_growth(sigma)`) and the material
// stiffness (`E(t) <- E_baseline * feedback_stiffness(sigma_history)`).
//
// Default policies:
//   - feedback_growth: small *negative* feedback. Highly stressed regions
//     elongate slower (suppression of growth in over-loaded tissue).
//   - feedback_stiffness: *positive* feedback. Sustained stress raises local
//     stiffness ("mechanical conditioning" / accelerated lignification).
//
// Numerical hygiene per the plan:
//   - feedback multipliers are clamped (configurable floors/ceilings).
//   - sigma history is exponentially-moving-averaged in time (alpha = 0 disables
//     all updates -> policy never engages -> preserves elastica solver results bit-
//     exactly).
//   - per-station sigma samples may be smoothed in arc length via a simple
//     box average (window = 0 disables smoothing).
//
// Default-constructed values are *inert*: zero gains and zero EMA alpha
// short-circuit IsActive() -> caller skips the feedback path entirely.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace l_system_package {

struct StressFeedbackPolicy {
  /// Magnitude of growth suppression per unit of normalized stress. Setting
  /// this to 0 disables growth feedback. Typical values: 0.1 .. 0.5 - beyond
  /// 1.0 the suppression saturates at `max_suppression_clamp` very quickly.
  float growth_suppression_gain = 0.0f;

  /// Magnitude of stiffness hardening per unit of normalized stress history.
  /// Setting this to 0 disables stiffness feedback. Typical values: 0.5 .. 2.
  float stiffness_hardening_gain = 0.0f;

  /// Normalization stress (Pa). sigma_norm = sigma / sigma_reference_Pa. Order of
  /// magnitude for slender needle bending: 1e5 .. 1e6 Pa. Must be > 0 for
  /// the policy to be active.
  float sigma_reference_Pa = 1.0e6f;

  /// Exponential-moving-average rate per *rebuild tick*:
  /// sigma_ema <- (1 - alpha) * sigma_ema + alpha * |sigma|.
  /// alpha = 0 -> EMA is never updated (policy effectively inert across rebuilds).
  /// alpha = 1 -> no smoothing (instantaneous sigma replaces history every tick).
  float time_ema_alpha = 0.0f;

  /// Arc-length smoothing window in *normalized* arc length [0,1] applied to
  /// the per-station sigma samples *before* EMA blending. A simple box average
  /// over neighbours within +/- window/2 of each station. 0 disables smoothing.
  float arc_length_smoothing_window = 0.0f;

  /// Floor on the growth multiplier (i.e. maximum suppression). Clamps
  /// `feedback_growth` from below; default 0.5 means growth never drops
  /// below 50% of baseline regardless of stress.
  float max_suppression_clamp = 0.5f;

  /// Ceiling on the stiffness multiplier. Default 4.0 means EI can rise to
  /// at most 4x baseline regardless of stress history.
  float max_hardening_clamp = 4.0f;

  /// True iff the policy has any effect across rebuilds. Requires non-zero
  /// reference stress, non-zero EMA alpha, and at least one non-zero gain.
  /// When false, callers MUST short-circuit and use baseline EI / kappa.
  [[nodiscard]] bool IsActive() const {
    if (sigma_reference_Pa <= 0.0f)
      return false;
    if (time_ema_alpha <= 0.0f)
      return false;
    return growth_suppression_gain != 0.0f || stiffness_hardening_gain != 0.0f;
  }

  /// Growth multiplier in (clamp, 1] for the supplied stress sample.
  /// `exp(-gain * sigma_norm)` saturates smoothly to 0; the explicit clamp
  /// keeps geometry sane under unphysical sigma spikes.
  [[nodiscard]] float feedback_growth(const float sigma_Pa) const {
    if (growth_suppression_gain == 0.0f)
      return 1.0f;
    const float sigma_norm = sigma_Pa / sigma_reference_Pa;
    const float raw = std::exp(-growth_suppression_gain * std::abs(sigma_norm));
    return std::clamp(raw, max_suppression_clamp, 1.0f);
  }

  /// Stiffness multiplier in [1, ceiling] for the supplied stress-history
  /// sample. `1 + gain * sigma_ema_norm` rises linearly; the explicit clamp
  /// caps unbounded hardening.
  [[nodiscard]] float feedback_stiffness(const float sigma_ema_Pa) const {
    if (stiffness_hardening_gain == 0.0f)
      return 1.0f;
    const float sigma_norm = sigma_ema_Pa / sigma_reference_Pa;
    const float raw = 1.0f + stiffness_hardening_gain * std::abs(sigma_norm);
    return std::clamp(raw, 1.0f, max_hardening_clamp);
  }

  /// Update an EMA buffer in place from a fresh per-station |sigma| sample
  /// vector. Resizes the EMA buffer if empty (first-touch initialization
  /// from the raw sample, no blending). Optionally box-smooths the raw
  /// samples in arc length first.
  void UpdateEma(const std::vector<float>& sigma_samples_Pa, std::vector<float>& ema_buffer_Pa) const {
    const std::size_t N = sigma_samples_Pa.size();
    if (N == 0)
      return;

    // Optional arc-length smoothing (box filter over normalized window).
    std::vector<float> smoothed;
    const std::vector<float>* src = &sigma_samples_Pa;
    if (arc_length_smoothing_window > 0.0f && N > 2) {
      smoothed.assign(N, 0.0f);
      const float half_window = 0.5f * std::clamp(arc_length_smoothing_window, 0.0f, 1.0f);
      const int half_stations = std::max(1, static_cast<int>(std::round(half_window * static_cast<float>(N - 1))));
      for (std::size_t i = 0; i < N; ++i) {
        const int lo = std::max(0, static_cast<int>(i) - half_stations);
        const int hi = std::min(static_cast<int>(N) - 1, static_cast<int>(i) + half_stations);
        float acc = 0.0f;
        int cnt = 0;
        for (int j = lo; j <= hi; ++j) {
          acc += sigma_samples_Pa[static_cast<std::size_t>(j)];
          ++cnt;
        }
        smoothed[i] = acc / static_cast<float>(cnt);
      }
      src = &smoothed;
    }

    if (ema_buffer_Pa.size() != N) {
      ema_buffer_Pa = *src;  // first-touch: seed EMA from raw sample.
      return;
    }
    const float a = std::clamp(time_ema_alpha, 0.0f, 1.0f);
    for (std::size_t i = 0; i < N; ++i) {
      ema_buffer_Pa[i] = (1.0f - a) * ema_buffer_Pa[i] + a * (*src)[i];
    }
  }
};

}  // namespace l_system_package