#pragma once

// ---------------------------------------------------------------------------
// CrossSectionProfile
//
// Phase 1 of the biologically-emergent organ growth substrate. Defines the
// shape of the cross-section that is swept along an OrganCenterline by the
// GeneralizedCylinderMesher. Each profile reports two things at every
// requested azimuthal sample u in [0,1):
//
//   * `radial_offset`  - vector from the centerline (in {normal, binormal})
//                        to the surface, in metres at the requested
//                        `radius` parameter,
//   * `outward_normal` - surface outward direction in the same 2D basis.
//
// Profiles are deliberately stateless (per-station radius is passed in by
// the mesher) so the same profile object can be reused across all stations
// of all organs of the same kind. The pine needle now uses an
// `EllipticProfile` whose major/minor vary along arc length; the
// maize/sorghum leaf blade can reuse the same two-axis interface.
//
// Phase 1 invariant: `CircularProfile` with N samples produces a regular
// N-gon identical to the existing GenerateUnitCylinderMesh tessellation
// modulo orientation, so the new mesher is a strict superset of the old
// straight-cylinder behaviour.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

namespace l_system_package {

/// One sample around the perimeter of the cross-section, expressed in the
/// 2D basis (normal, binormal) anchored to the centerline frame.
struct CrossSectionSample {
  glm::vec2 radial_offset = glm::vec2(0.0f);   ///< (n, b) coordinates of the surface point.
  glm::vec2 outward_normal = glm::vec2(1, 0);  ///< Surface outward direction (unit).
  float u = 0.0f;                              ///< Parameter in [0,1) used for V tex coord.
  bool seam = false;                           ///< Profile starts/ends here (open profiles only).
};

/// Polymorphic interface so the mesher is profile-agnostic.
class CrossSectionProfile {
 public:
  virtual ~CrossSectionProfile() = default;

  /// Returns true for closed profiles (full perimeter, last vertex stitches
  /// back to first), false for open profiles (e.g. a leaf blade with two
  /// distinct edges).
  virtual bool IsClosed() const = 0;

  /// Sample the perimeter at `sample_count` evenly spaced parameters and
  /// fill `out_samples` (cleared first). `radius` is the per-station radius
  /// supplied by the mesher; profile implementations may interpret it
  /// freely (e.g. semi-major axis for ellipses).
  virtual void Sample(int sample_count, float radius, std::vector<CrossSectionSample>& out_samples) const = 0;

  /// Optional two-axis sampling path for anisotropic cross-sections (ellipse
  /// major/minor semi-axes). Profiles that are axis-symmetric can ignore the
  /// secondary radius and reuse the legacy one-axis implementation.
  virtual void SampleWithAxes(int sample_count, float primary_radius, float secondary_radius,
                              std::vector<CrossSectionSample>& out_samples) const {
    (void)secondary_radius;
    Sample(sample_count, primary_radius, out_samples);
  }
};

// ---------------------------------------------------------------------------
// CircularProfile - closed, regular n-gon. Default for woody internodes and
// the Phase 1 visual-no-op fallback for needles.
// ---------------------------------------------------------------------------
class CircularProfile final : public CrossSectionProfile {
 public:
  bool IsClosed() const override {
    return true;
  }

  void Sample(int sample_count, float radius, std::vector<CrossSectionSample>& out_samples) const override {
    out_samples.clear();
    const int n = std::max(3, sample_count);
    out_samples.reserve(static_cast<size_t>(n));
    const float two_pi = glm::two_pi<float>();
    for (int i = 0; i < n; ++i) {
      const float u = static_cast<float>(i) / static_cast<float>(n);
      const float theta = u * two_pi;
      const float c = std::cos(theta);
      const float s = std::sin(theta);
      CrossSectionSample sample;
      sample.radial_offset = glm::vec2(c, s) * radius;
      sample.outward_normal = glm::vec2(c, s);
      sample.u = u;
      sample.seam = (i == 0);
      out_samples.push_back(sample);
    }
  }
};

// ---------------------------------------------------------------------------
// SemiCircularProfile - closed, flat-adaxial / convex-abaxial. Models the
// actual cross-section of a Scots pine fascicle needle (each needle in a
// 2-needle bundle is approximately a semicircle whose flat face touches its
// sibling along the bundle axis).
//
// Layout in the (n=adaxial, b=binormal) basis:
//
//      flat side  +------------------+  adaxial face (n = -radius, flat across b)
//                 |                  |
//                 |  convex abaxial  |
//                 |       side       |
//                 +------------------+
//                    arc bulges toward +n
//
// `n_flat_samples` controls how many vertices lie on the flat edge; the
// remaining `sample_count - n_flat_samples` cover the convex half-arc.
// ---------------------------------------------------------------------------
class SemiCircularProfile final : public CrossSectionProfile {
 public:
  explicit SemiCircularProfile(float adaxial_offset = 0.0f, int n_flat_samples = 2)
      : adaxial_offset_(adaxial_offset), n_flat_samples_(std::max(2, n_flat_samples)) {
  }

  bool IsClosed() const override {
    return true;
  }

  void Sample(int sample_count, float radius, std::vector<CrossSectionSample>& out_samples) const override {
    out_samples.clear();
    const int n_total = std::max(n_flat_samples_ + 3, sample_count);
    const int n_arc = n_total - n_flat_samples_;
    out_samples.reserve(static_cast<size_t>(n_total));

    // Flat adaxial edge: parameterized along -binormal -> +binormal at
    // n = -adaxial_offset (flush with the bundle axis when offset = 0).
    for (int i = 0; i < n_flat_samples_; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(n_flat_samples_ - 1);
      const float b = (-1.0f + 2.0f * t) * radius;
      CrossSectionSample s;
      s.radial_offset = glm::vec2(-adaxial_offset_, b);
      s.outward_normal = glm::vec2(-1.0f, 0.0f);  // adaxial side faces -n.
      s.u = t * 0.5f;                             // first half of u.
      s.seam = (i == 0);
      out_samples.push_back(s);
    }

    // Convex abaxial half: half-arc from +binormal back to -binormal through
    // +normal. theta runs 0..pi where theta=0 is at (+0, +radius) and
    // theta=pi is at (+0, -radius), with peak at (+radius, 0).
    for (int i = 1; i < n_arc; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(n_arc);
      const float theta = t * glm::pi<float>();
      const float c = std::cos(theta);  // 1 -> -1
      const float s = std::sin(theta);  // 0 -> 0 (peak at theta=pi/2 = 1)
      CrossSectionSample sample;
      sample.radial_offset = glm::vec2(s * radius, c * radius);
      sample.outward_normal = glm::vec2(s, c);
      sample.u = 0.5f + 0.5f * t;
      sample.seam = false;
      out_samples.push_back(sample);
    }
  }

 private:
  float adaxial_offset_;  ///< Pushes the flat face away from the bundle axis (m).
  int n_flat_samples_;    ///< Vertices laid along the flat adaxial edge.
};

// ---------------------------------------------------------------------------
// EllipticProfile - closed ellipse. Stub for the future maize/sorghum leaf
// blade; semi_minor varies along arc length will be done by the mesher
// passing a different `radius` at every station, with a profile-specific
// aspect ratio applied here. Wired up here so the maize port is data-only.
// ---------------------------------------------------------------------------
class EllipticProfile final : public CrossSectionProfile {
 public:
  EllipticProfile(float aspect_ratio = 1.0f, float twist_radians = 0.0f)
      : aspect_(std::max(0.05f, aspect_ratio)), twist_(twist_radians) {
  }

  bool IsClosed() const override {
    return true;
  }

  void Sample(int sample_count, float radius, std::vector<CrossSectionSample>& out_samples) const override {
    SampleWithAxes(sample_count, radius, radius * aspect_, out_samples);
  }

  void SampleWithAxes(int sample_count, float primary_radius, float secondary_radius,
                      std::vector<CrossSectionSample>& out_samples) const override {
    out_samples.clear();
    const int n = std::max(3, sample_count);
    out_samples.reserve(static_cast<size_t>(n));
    const float two_pi = glm::two_pi<float>();
    const float ct = std::cos(twist_);
    const float st = std::sin(twist_);
    const float major_radius = std::max(1.0e-6f, primary_radius);
    const float minor_radius = std::max(1.0e-6f, secondary_radius);
    for (int i = 0; i < n; ++i) {
      const float u = static_cast<float>(i) / static_cast<float>(n);
      const float theta = u * two_pi;
      const float c = std::cos(theta);
      const float s = std::sin(theta);
      // Ellipse in (n, b): semi-major along n, semi-minor along b.
      glm::vec2 p(c * major_radius, s * minor_radius);
      // Outward normal of the ellipse at parameter theta (un-normalized then normalised).
      glm::vec2 nrm(c / major_radius, s / minor_radius);
      const float l = std::sqrt(nrm.x * nrm.x + nrm.y * nrm.y);
      if (l > 1e-20f)
        nrm /= l;
      // Rotate by twist_ in the (n, b) plane.
      const glm::vec2 p_r(p.x * ct - p.y * st, p.x * st + p.y * ct);
      const glm::vec2 n_r(nrm.x * ct - nrm.y * st, nrm.x * st + nrm.y * ct);
      CrossSectionSample sample;
      sample.radial_offset = p_r;
      sample.outward_normal = n_r;
      sample.u = u;
      sample.seam = (i == 0);
      out_samples.push_back(sample);
    }
  }

 private:
  float aspect_;
  float twist_;
};

}  // namespace l_system_package