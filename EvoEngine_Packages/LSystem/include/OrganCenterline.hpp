#pragma once

// ---------------------------------------------------------------------------
// OrganCenterline
//
// Phase 1 of the biologically-emergent organ growth substrate. A centerline
// is a sequence of control points expressed in a local "organ frame" (origin
// at the attachment point, +Z forward = principal growth axis, +X = adaxial
// reference direction). The centerline carries:
//
//   * a Catmull-Rom interpolant over its control points (centripetal
//     parameterization for stability against unequal control-point spacing),
//   * a cached arc-length table so per-station sampling at uniform `s` is
//     O(log N) per query without re-walking the spline,
//   * a rotation-minimizing parallel-transport frame at every sample so the
//     downstream cross-section sweep does not introduce visible twist.
//
// The class is deliberately decoupled from any L-system module type so the
// same primitive serves PineNeedle, PineInternode (Phase 6) and the
// future maize/sorghum leaf blade (post-Phase 6).
//
// Phase 1 invariant: a freshly constructed centerline with two control
// points and no curvature is geometrically identical to a straight cylinder
// of the requested length, modulo tessellation.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace l_system_package {

/// Per-station sample produced by `OrganCenterline::Sample(s)`.
struct CenterlineSample {
  glm::vec3 position = glm::vec3(0.0f);     ///< Position in the organ-local frame.
  glm::vec3 tangent = glm::vec3(0, 0, 1);   ///< Unit tangent (forward).
  glm::vec3 normal = glm::vec3(1, 0, 0);    ///< Unit normal (adaxial reference).
  glm::vec3 binormal = glm::vec3(0, 1, 0);  ///< cross(tangent, normal); right-handed frame.
  float arc_length = 0.0f;                  ///< Arc length from the base.
};

class OrganCenterline {
 public:
  OrganCenterline() = default;

  /// Construct a straight centerline of `length` along +Z with `segments + 1`
  /// uniformly spaced control points (segments >= 1). This is the Phase 1
  /// default state for every organ before the differential growth field
  /// (Phase 3) and elastica solver (Phase 4) deflect it.
  static OrganCenterline Straight(float length, int segments = 4) {
    OrganCenterline c;
    const int n = std::max(2, segments + 1);
    c.control_points_.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(n - 1);
      c.control_points_[i] = glm::vec3(0.0f, 0.0f, t * length);
    }
    c.Invalidate();
    return c;
  }

  /// Direct accessor - Phase 3 differential growth writes to this directly.
  std::vector<glm::vec3>& MutableControlPoints() {
    arc_length_dirty_ = true;
    return control_points_;
  }
  const std::vector<glm::vec3>& ControlPoints() const {
    return control_points_;
  }

  /// Total arc length along the spline (cached; trigger rebuild if dirty).
  float TotalLength() const {
    EnsureArcLengthTable();
    return arc_length_table_.empty() ? 0.0f : arc_length_table_.back();
  }

  /// Number of resampling stations used for the arc-length table. Higher =
  /// more accurate `Sample()` at the cost of construction time. Must be
  /// called before any `Sample()` if changed at runtime.
  void SetResampleCount(int count) {
    resample_count_ = std::max(8, count);
    arc_length_dirty_ = true;
  }

  /// Sample the centerline at arc length `s` (clamped to [0, TotalLength()]).
  /// Returns a rotation-minimizing frame computed by parallel transport from
  /// the base, with the base normal anchored to `base_normal`.
  CenterlineSample Sample(float s, const glm::vec3& base_normal = glm::vec3(1, 0, 0)) const {
    EnsureArcLengthTable();
    EnsureFrameTable(base_normal);
    const float total = TotalLength();
    if (total <= 0.0f) {
      CenterlineSample sample;
      sample.position = control_points_.empty() ? glm::vec3(0.0f) : control_points_.front();
      sample.tangent = glm::vec3(0, 0, 1);
      sample.normal = SafeNormalize(base_normal, glm::vec3(1, 0, 0));
      sample.binormal = glm::cross(sample.tangent, sample.normal);
      sample.arc_length = 0.0f;
      return sample;
    }
    s = std::clamp(s, 0.0f, total);
    // Locate the resample interval [i, i+1] containing arc length s.
    const int last = static_cast<int>(arc_length_table_.size()) - 1;
    int lo = 0;
    int hi = last;
    while (lo + 1 < hi) {
      const int mid = (lo + hi) / 2;
      if (arc_length_table_[mid] <= s)
        lo = mid;
      else
        hi = mid;
    }
    const float s_lo = arc_length_table_[lo];
    const float s_hi = arc_length_table_[hi];
    const float u = (s_hi > s_lo) ? (s - s_lo) / (s_hi - s_lo) : 0.0f;
    CenterlineSample a = sampled_frames_[lo];
    CenterlineSample b = sampled_frames_[hi];
    CenterlineSample out;
    out.position = glm::mix(a.position, b.position, u);
    out.tangent = SafeNormalize(glm::mix(a.tangent, b.tangent, u), a.tangent);
    out.normal = SafeNormalize(
        glm::mix(a.normal, b.normal, u) - out.tangent * glm::dot(glm::mix(a.normal, b.normal, u), out.tangent),
        a.normal);
    out.binormal = SafeNormalize(glm::cross(out.tangent, out.normal), glm::cross(a.tangent, a.normal));
    out.arc_length = s;
    return out;
  }

  /// Mark the arc-length and frame tables stale (call after any external
  /// mutation of the control points besides MutableControlPoints()).
  void Invalidate() {
    arc_length_dirty_ = true;
  }

 private:
  std::vector<glm::vec3> control_points_;
  int resample_count_ = 32;

  mutable bool arc_length_dirty_ = true;
  mutable std::vector<float> arc_length_table_;  ///< Arc length per resample station.
  mutable std::vector<glm::vec3> sampled_positions_;
  mutable std::vector<CenterlineSample> sampled_frames_;
  mutable glm::vec3 cached_base_normal_ = glm::vec3(1, 0, 0);
  mutable bool frames_dirty_ = true;

  static glm::vec3 SafeNormalize(const glm::vec3& v, const glm::vec3& fallback) {
    const float l2 = glm::dot(v, v);
    if (l2 > 1e-20f)
      return v * (1.0f / std::sqrt(l2));
    return fallback;
  }

  // Centripetal Catmull-Rom evaluation between p1 and p2 with neighbors p0, p3.
  // u in [0,1]. Falls back to linear interpolation at the endpoints.
  static glm::vec3 CatmullRom(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3,
                              float u) {
    auto knot = [](float prev_t, const glm::vec3& a, const glm::vec3& b) {
      const float d = std::sqrt(glm::length(b - a));  // centripetal exponent = 0.5.
      return prev_t + std::max(d, 1e-6f);
    };
    const float t0 = 0.0f;
    const float t1 = knot(t0, p0, p1);
    const float t2 = knot(t1, p1, p2);
    const float t3 = knot(t2, p2, p3);
    const float t = glm::mix(t1, t2, u);
    const glm::vec3 a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
    const glm::vec3 a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
    const glm::vec3 a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;
    const glm::vec3 b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2;
    const glm::vec3 b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
    return (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2;
  }

  // Evaluate at normalized parameter t in [0,1] across the control polyline.
  glm::vec3 EvaluatePolyline(float t) const {
    if (control_points_.size() < 2) {
      return control_points_.empty() ? glm::vec3(0.0f) : control_points_.front();
    }
    if (control_points_.size() == 2) {
      return glm::mix(control_points_[0], control_points_[1], t);
    }
    const int segs = static_cast<int>(control_points_.size()) - 1;
    const float ts = t * static_cast<float>(segs);
    int i = std::min(segs - 1, std::max(0, static_cast<int>(std::floor(ts))));
    const float u = ts - static_cast<float>(i);
    const int i0 = std::max(0, i - 1);
    const int i1 = i;
    const int i2 = i + 1;
    const int i3 = std::min(segs, i + 2);
    return CatmullRom(control_points_[i0], control_points_[i1], control_points_[i2], control_points_[i3], u);
  }

  void EnsureArcLengthTable() const {
    if (!arc_length_dirty_)
      return;
    sampled_positions_.clear();
    sampled_positions_.reserve(static_cast<size_t>(resample_count_));
    arc_length_table_.assign(static_cast<size_t>(resample_count_), 0.0f);
    if (control_points_.empty()) {
      arc_length_dirty_ = false;
      frames_dirty_ = true;
      return;
    }
    for (int i = 0; i < resample_count_; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(resample_count_ - 1);
      sampled_positions_.push_back(EvaluatePolyline(t));
    }
    arc_length_table_[0] = 0.0f;
    for (int i = 1; i < resample_count_; ++i) {
      arc_length_table_[i] = arc_length_table_[i - 1] + glm::length(sampled_positions_[i] - sampled_positions_[i - 1]);
    }
    arc_length_dirty_ = false;
    frames_dirty_ = true;
  }

  void EnsureFrameTable(const glm::vec3& base_normal) const {
    if (!frames_dirty_ && glm::length(base_normal - cached_base_normal_) < 1e-6f)
      return;
    cached_base_normal_ = base_normal;
    sampled_frames_.assign(sampled_positions_.size(), CenterlineSample{});
    if (sampled_positions_.empty()) {
      frames_dirty_ = false;
      return;
    }
    // Initial tangent: forward difference; if degenerate fall back to +Z.
    glm::vec3 prev_tangent(0, 0, 1);
    if (sampled_positions_.size() >= 2) {
      const glm::vec3 d = sampled_positions_[1] - sampled_positions_[0];
      prev_tangent = SafeNormalize(d, glm::vec3(0, 0, 1));
    }
    glm::vec3 prev_normal = base_normal - prev_tangent * glm::dot(base_normal, prev_tangent);
    prev_normal = SafeNormalize(prev_normal, glm::vec3(1, 0, 0));

    for (size_t i = 0; i < sampled_positions_.size(); ++i) {
      glm::vec3 tangent;
      if (i + 1 < sampled_positions_.size()) {
        const glm::vec3 d = sampled_positions_[i + 1] - sampled_positions_[i];
        tangent = SafeNormalize(d, prev_tangent);
      } else {
        const glm::vec3 d = sampled_positions_[i] - sampled_positions_[i - 1];
        tangent = SafeNormalize(d, prev_tangent);
      }
      // Rotation-minimizing transport (double-reflection method, Wang et al. 2008).
      glm::vec3 normal = prev_normal;
      const glm::vec3 v1 = sampled_positions_[i] - (i == 0 ? sampled_positions_[i] : sampled_positions_[i - 1]);
      const float c1 = glm::dot(v1, v1);
      if (c1 > 1e-20f) {
        const glm::vec3 r_l = prev_normal - (2.0f / c1) * glm::dot(v1, prev_normal) * v1;
        const glm::vec3 t_l = prev_tangent - (2.0f / c1) * glm::dot(v1, prev_tangent) * v1;
        const glm::vec3 v2 = tangent - t_l;
        const float c2 = glm::dot(v2, v2);
        if (c2 > 1e-20f) {
          normal = r_l - (2.0f / c2) * glm::dot(v2, r_l) * v2;
        } else {
          normal = r_l;
        }
        normal -= tangent * glm::dot(normal, tangent);
        normal = SafeNormalize(normal, prev_normal);
      } else {
        normal = prev_normal - tangent * glm::dot(prev_normal, tangent);
        normal = SafeNormalize(normal, glm::vec3(1, 0, 0));
      }
      CenterlineSample s;
      s.position = sampled_positions_[i];
      s.tangent = tangent;
      s.normal = normal;
      s.binormal = SafeNormalize(glm::cross(tangent, normal), glm::vec3(0, 1, 0));
      s.arc_length = arc_length_table_[i];
      sampled_frames_[i] = s;
      prev_tangent = tangent;
      prev_normal = normal;
    }
    frames_dirty_ = false;
  }
};

}  // namespace l_system_package
