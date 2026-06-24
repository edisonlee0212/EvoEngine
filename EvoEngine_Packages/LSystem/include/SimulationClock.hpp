#pragma once

// ---------------------------------------------------------------------------
// SimulationClock
//
// Phase 2 of the biologically-emergent organ growth substrate. A monotonic
// physiological-time accumulator decoupled from rendering frames, GDD ticks,
// or wall-clock. Every plant carries one in its `GraphData`; every module
// born during a derivation step records `t_init_years = clock.NowYears()`,
// so the per-organ growth function can later evaluate
// `Value((NowYears() - t_init_years) / maturation_years)` regardless of who
// or what is driving the clock.
//
// Three drivers are supported:
//
//   * `AdvanceYears(dt)` - explicit; for headless dataset generation,
//                          unit tests, and direct UI scrubbing.
//   * `AdvanceFromGDD(dgdd, gdd_per_year)` - wraps GDD increments into
//                          fractional years; used by the existing
//                          LSystemGrowthModelBase::GrowStep pipeline.
//   * `SetYears(t)` - non-monotonic; for backward-scrubbing scenarios
//                          (the editor allows dragging target_gdd backward).
//                          Skips the monotonic guard so callers can rebuild
//                          state from scratch.
//
// The clock is intentionally free of distribution machinery: no RNG, no
// per-organ state. It is a single float (plus a derived-step counter for
// telemetry). Every other piece of biology consults it through the graph.
// ---------------------------------------------------------------------------

#include <cmath>
#include <cstdint>

namespace l_system_package {

class SimulationClock {
 public:
  SimulationClock() = default;

  /// Physiological time in years since clock zero. Monotonic when only
  /// `AdvanceYears` / `AdvanceFromGDD` are used.
  float NowYears() const {
    return now_years_;
  }

  /// Number of `Advance*` calls since construction or the last `Reset()`.
  /// Used by Phase 2 unit tests to verify deterministic replay.
  std::uint64_t StepCount() const {
    return step_count_;
  }

  /// Advance the clock by `dt_years`. Negative `dt_years` is silently dropped
  /// (use `SetYears` for backward scrubbing).
  void AdvanceYears(float dt_years) {
    if (!std::isfinite(dt_years) || dt_years <= 0.0f)
      return;
    now_years_ += dt_years;
    ++step_count_;
  }

  /// Convert a GDD increment to years using the supplied conversion and
  /// advance. `gdd_per_year` must be > 0.
  void AdvanceFromGDD(float dgdd, float gdd_per_year) {
    if (gdd_per_year <= 0.0f || !std::isfinite(dgdd) || dgdd <= 0.0f)
      return;
    AdvanceYears(dgdd / gdd_per_year);
  }

  /// Force the clock to an absolute physiological time. Bypasses the
  /// monotonic guard so callers can rewind for re-derivation. Increments
  /// `StepCount` to mark the discontinuity.
  void SetYears(float t_years) {
    if (!std::isfinite(t_years))
      return;
    now_years_ = t_years < 0.0f ? 0.0f : t_years;
    ++step_count_;
  }

  /// Zero the clock and step counter.
  void Reset() {
    now_years_ = 0.0f;
    step_count_ = 0;
    year_index_ = 0;
    t_year_start_years_ = 0.0f;
    active_season_length_years_ = 1.0f;
    in_active_season_ = true;
    has_seen_active_season_ = false;
  }

  // -------------------------------------------------------------------------
  // Seasonal year tracking (Scots-pine phytomer model).
  //
  // The phytomer growth rules need three pieces of state that are naturally
  // owned by the clock:
  //   * `YearIndex()`   - integer year counter, incremented on each
  //                       dormant->active edge by the LSystem layer.
  //   * `YearStartYears()` - physiological time at which the current
  //                          active season began. Rules use this to compute
  //                          `temporal_year_progress` for the bare-zone gate.
  //   * `ActiveSeasonLengthYears()` - duration of the current active season
  //                                   (in physiological years). Defaults to
  //                                   1.0 when no calendar gating is active.
  //   * `InActiveSeason()` - whether the apex is currently allowed to emit.
  // -------------------------------------------------------------------------
  int YearIndex() const {
    return year_index_;
  }
  float YearStartYears() const {
    return t_year_start_years_;
  }
  float ActiveSeasonLengthYears() const {
    return active_season_length_years_;
  }
  bool InActiveSeason() const {
    return in_active_season_;
  }

  /// Increment the year counter and stamp the new active-season start time.
  /// Called by the LSystem layer when it detects a dormant->active edge.
  void BumpYear(float active_season_length_years) {
    ++year_index_;
    t_year_start_years_ = now_years_;
    active_season_length_years_ = (active_season_length_years > 1.0e-4f) ? active_season_length_years : 1.0f;
    in_active_season_ = true;
    has_seen_active_season_ = true;
  }

  /// Toggle whether the apex is currently in its active growth season.
  /// Driven by the LSystem layer's day-of-year gate.
  void SetInActiveSeason(bool in_season) {
    in_active_season_ = in_season;
  }

  /// Drive descriptor phenology state once per calendar step. Detects the
  /// dormant -> active transition and bumps the year counter. When phenology
  /// gating is disabled the active-season flag is held true and year rollover
  /// happens whenever `now_years_` crosses the next integer.
  void SyncSeasonalState(bool phenology_enabled, bool descriptor_in_active_growth, float active_season_length_years) {
    if (phenology_enabled) {
      const bool was_active = in_active_season_;
      in_active_season_ = descriptor_in_active_growth;
      if (in_active_season_ && !has_seen_active_season_) {
        t_year_start_years_ = now_years_;
        active_season_length_years_ = (active_season_length_years > 1.0e-4f) ? active_season_length_years : 1.0f;
        has_seen_active_season_ = true;
      } else if (in_active_season_ && !was_active) {
        BumpYear(active_season_length_years);
      } else if (in_active_season_) {
        // If the descriptor is effectively active all year, there is no
        // dormant->active edge to trigger BumpYear(). Still roll the annual
        // Scots-pine shoot budget forward when the active-season span elapses.
        const float safe_active_length = (active_season_length_years > 1.0e-4f) ? active_season_length_years : 1.0f;
        const float elapsed_since_year_start = now_years_ - t_year_start_years_;
        if (std::isfinite(elapsed_since_year_start) && elapsed_since_year_start >= safe_active_length) {
          const int elapsed_years = static_cast<int>(std::floor(elapsed_since_year_start / safe_active_length));
          if (elapsed_years > 0) {
            year_index_ += elapsed_years;
            t_year_start_years_ += static_cast<float>(elapsed_years) * safe_active_length;
            active_season_length_years_ = safe_active_length;
          }
        }
      }
    } else {
      in_active_season_ = true;
      has_seen_active_season_ = true;
      // Year-from-time fallback: bump whenever we cross into a new whole year.
      const int candidate_year = static_cast<int>(std::floor(now_years_));
      if (candidate_year > year_index_) {
        // Stamp year_start at the integer boundary so temporal_progress
        // computed in rules stays in [0, 1).
        year_index_ = candidate_year;
        t_year_start_years_ = static_cast<float>(candidate_year);
        active_season_length_years_ = 1.0f;
      }
    }
  }

 private:
  float now_years_ = 0.0f;
  std::uint64_t step_count_ = 0;
  int year_index_ = 0;
  float t_year_start_years_ = 0.0f;
  float active_season_length_years_ = 1.0f;
  bool in_active_season_ = true;
  bool has_seen_active_season_ = false;
};

}  // namespace l_system_package
